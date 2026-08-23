import { clamp } from './core/math.js';

const safeArray = (value) => (Array.isArray(value) ? value : []);
const finite = (value, fallback = 0) => (Number.isFinite(value) ? value : fallback);

/** Procedural WebAudio soundscape for racing and vehicle dynamics. */
export class SynthAudio {
  constructor() {
    this.context = null;
    this.master = null;
    this.engineGain = null;
    this.engine2Gain = null;
    this.gearGain = null;
    this.ersGain = null;
    this.regenGain = null;
    this.skidGain = null;
    this.scrubGain = null;
    this.curbGain = null;
    this.windGain = null;
    this.engineOscillator = null;
    this.engine2Oscillator = null;
    this.gearOscillator = null;
    this.ersOscillator = null;
    this.regenOscillator = null;
    this.continuousSources = [];
    this.muted = false;
    this.ready = false;
    this.impactCooldown = 0;
    this.popCooldown = 0;
    this.shiftCooldown = 0;
    this.curbCooldown = 0;
    this.lastGear = 1;
    this.lastThrottle = 0;
  }

  async unlock() {
    if (!this.ready) this._create();
    if (this.context?.state === 'suspended') {
      await this.context.resume();
    }
  }

  _createNoise(context, seconds = 2) {
    const buffer = context.createBuffer(1, context.sampleRate * seconds, context.sampleRate);
    const data = buffer.getChannelData(0);
    for (let i = 0; i < data.length; i += 1) {
      data[i] = Math.sin(i * 12.9898) * 0.72;
    }
    return buffer;
  }

  _gain(context, value = 0.0001) {
    const gain = context.createGain();
    gain.gain.value = value;
    return gain;
  }

  _continuousOscillator(context, type, frequency, gain, destination) {
    const oscillator = context.createOscillator();
    oscillator.type = type;
    oscillator.frequency.value = frequency;
    oscillator.connect(gain).connect(destination);
    oscillator.start();
    this.continuousSources.push(oscillator);
    return oscillator;
  }

  _continuousNoise(context, buffer, filterType, frequency, gain) {
    const source = context.createBufferSource();
    source.buffer = buffer;
    source.loop = true;
    const filter = context.createBiquadFilter();
    filter.type = filterType;
    filter.frequency.value = frequency;
    source.connect(filter).connect(gain).connect(this.master);
    source.start();
    this.continuousSources.push(source);
    return source;
  }

  _create() {
    const AudioContextClass = typeof window === 'undefined' ? null : (window.AudioContext || window.webkitAudioContext);
    if (!AudioContextClass) return;
    this.context = new AudioContextClass();
    const context = this.context;
    this.master = this._gain(context, this.muted ? 0 : 0.45);
    this.master.connect(context.destination);

    const engineFilter = context.createBiquadFilter();
    engineFilter.type = 'lowpass';
    engineFilter.frequency.value = 900;
    engineFilter.Q.value = 1.3;
    engineFilter.connect(this.master);

    this.engineGain = this._gain(context);
    this.engineOscillator = this._continuousOscillator(context, 'sawtooth', 54, this.engineGain, engineFilter);

    this.engine2Gain = this._gain(context);
    this.engine2Oscillator = this._continuousOscillator(context, 'square', 108, this.engine2Gain, engineFilter);

    const gearFilter = context.createBiquadFilter();
    gearFilter.type = 'bandpass';
    gearFilter.frequency.value = 1450;
    gearFilter.Q.value = 0.75;
    gearFilter.connect(this.master);

    this.gearGain = this._gain(context);
    this.gearOscillator = this._continuousOscillator(context, 'triangle', 140, this.gearGain, gearFilter);

    const electricFilter = context.createBiquadFilter();
    electricFilter.type = 'bandpass';
    electricFilter.frequency.value = 750;
    electricFilter.Q.value = 0.5;
    electricFilter.connect(this.master);

    this.ersGain = this._gain(context);
    this.ersOscillator = this._continuousOscillator(context, 'triangle', 220, this.ersGain, electricFilter);

    this.regenGain = this._gain(context);
    this.regenOscillator = this._continuousOscillator(context, 'triangle', 180, this.regenGain, electricFilter);

    const noise = this._createNoise(context);
    this.skidGain = this._gain(context);
    this._continuousNoise(context, noise, 'bandpass', 1650, this.skidGain);

    this.scrubGain = this._gain(context);
    this._continuousNoise(context, noise, 'highpass', 420, this.scrubGain);

    this.curbGain = this._gain(context);
    this._continuousNoise(context, noise, 'bandpass', 165, this.curbGain);

    this.windGain = this._gain(context);
    this._continuousNoise(context, noise, 'lowpass', 410, this.windGain);

    this.ready = true;
  }

  update(vehicle = {}, dt = 0) {
    if (!this.ready || !this.context) return;
    const time = this.context.currentTime;
    const safeDt = clamp(finite(dt), 0, 1);
    const rpmFactor = clamp((finite(vehicle.rpm) - 900) / 6800, 0, 1);
    const speedFactor = clamp(finite(vehicle.speed) / 70, 0, 1);
    const throttle = clamp(finite(vehicle.controls?.throttle), 0, 1);

    this.engineGain.gain.setTargetAtTime(0.01 + rpmFactor * 0.115 * (0.22 + throttle), time, 0.03);
    this.engine2Gain.gain.setTargetAtTime(0.002 + rpmFactor * 0.03, time, 0.045);
    const baseFrequency = 24 + finite(vehicle.rpm) / 62;
    this.engineOscillator.frequency.setTargetAtTime(baseFrequency, time, 0.025);
    this.engine2Oscillator.frequency.setTargetAtTime(baseFrequency * 2.015, time, 0.035);

    const shaftFrequency = 75 + finite(vehicle.rpm) / Math.max(1, finite(vehicle.gear, 1)) / 34;
    const drivelineLoad = clamp(Math.abs(finite(vehicle.engineTorque)) / 620 + speedFactor * 0.16, 0, 1);
    this.gearOscillator.frequency.setTargetAtTime(shaftFrequency, time, 0.035);
    this.gearGain.gain.setTargetAtTime(0.001 + drivelineLoad * 0.026, time, 0.05);

    const ers = vehicle.ers ?? {};
    const deployPowerW = Math.max(0, finite(ers.deployPowerW, finite(vehicle.ersDeployPowerW)));
    const regenPowerW = Math.max(0, finite(ers.regenPowerW, finite(vehicle.regenPowerW)));
    const deployFactor = clamp(deployPowerW / 150000, 0, 1);
    const regenFactor = clamp(regenPowerW / 150000, 0, 1);
    this.ersOscillator.frequency.setTargetAtTime(185 + deployFactor * 570 + rpmFactor * 80, time, 0.06);
    this.ersGain.gain.setTargetAtTime(0.0001 + deployFactor * (0.009 + throttle * 0.011), time, 0.07);
    this.regenOscillator.frequency.setTargetAtTime(125 + regenFactor * 380 + speedFactor * 55, time, 0.055);
    this.regenGain.gain.setTargetAtTime(0.0001 + regenFactor * 0.016, time, 0.06);

    const wheels = safeArray(vehicle.wheels);
    const tyreSlip = wheels.length ? Math.max(0, ...wheels.map((w) => finite(w?.slip))) : 0;
    const tyreUtilisation = wheels.length ? Math.max(0, ...wheels.map((w) => finite(w?.utilisation))) : 0;
    this.skidGain.gain.setTargetAtTime(clamp((tyreSlip - 0.13) * 0.13 * speedFactor, 0.0001, 0.12), time, 0.04);
    this.scrubGain.gain.setTargetAtTime(clamp((tyreUtilisation - 0.58) * 0.055 * speedFactor, 0.0001, 0.045), time, 0.06);
    this.windGain.gain.setTargetAtTime(0.001 + speedFactor * speedFactor * 0.052, time, 0.08);

    const curbContact = wheels.reduce(
      (peak, w) => Math.max(
        peak,
        finite(w?.curbContact) ? 1 : 0,
        w?.surface === 'curb' || w?.contactZone === 'curb' || w?.onCurb ? 1 : 0,
        finite(w?.contactHeight) > 0.018 ? 0.55 : 0,
        Math.abs(finite(w?.suspensionVelocity)) * 0.28
      ),
      0
    );
    const curbRumble = clamp(curbContact * (0.15 + speedFactor * 0.85), 0, 1);
    this.curbGain.gain.setTargetAtTime(0.0001 + curbRumble * 0.055, time, 0.028);

    this.impactCooldown = Math.max(0, this.impactCooldown - safeDt);
    this.popCooldown = Math.max(0, this.popCooldown - safeDt);
    this.shiftCooldown = Math.max(0, this.shiftCooldown - safeDt);
    this.curbCooldown = Math.max(0, this.curbCooldown - safeDt);

    if (finite(vehicle.impact) > 0.24 && this.impactCooldown <= 0) {
      this._impact(finite(vehicle.impact));
      this.impactCooldown = 0.16;
    }
    if (curbRumble > 0.46 && this.curbCooldown <= 0) {
      this._curbRumble(curbRumble);
      this.curbCooldown = 0.1;
    }
    const gear = finite(vehicle.gear, this.lastGear);
    if (gear !== this.lastGear && this.shiftCooldown <= 0) {
      this._shift(gear > this.lastGear, clamp(rpmFactor, 0.2, 1));
      this.shiftCooldown = 0.11;
    }
    if ((gear < this.lastGear || (this.lastThrottle > 0.45 && throttle < 0.08 && finite(vehicle.rpm) > 3900)) && this.popCooldown <= 0) {
      this._pop(clamp(rpmFactor, 0.25, 1));
      this.popCooldown = 0.19;
    }
    this.lastGear = gear;
    this.lastThrottle = throttle;
  }

  _oneShot(type, frequency, peak, duration, endRatio = 0.34) {
    if (!this.context || !this.master) return;
    const context = this.context;
    const oscillator = context.createOscillator();
    const gain = context.createGain();
    oscillator.type = type;
    oscillator.frequency.setValueAtTime(Math.max(20, frequency), context.currentTime);
    oscillator.frequency.exponentialRampToValueAtTime(Math.max(20, frequency * endRatio), context.currentTime + duration);
    gain.gain.setValueAtTime(0.0001, context.currentTime);
    gain.gain.exponentialRampToValueAtTime(Math.max(0.00011, peak), context.currentTime + 0.008);
    gain.gain.exponentialRampToValueAtTime(0.0001, context.currentTime + duration);
    oscillator.connect(gain).connect(this.master);
    oscillator.onended = () => {
      oscillator.disconnect();
      gain.disconnect();
    };
    oscillator.start();
    oscillator.stop(context.currentTime + duration + 0.02);
  }

  _impact(intensity) {
    this._oneShot('triangle', 95 + intensity * 90, 0.14 * intensity, 0.16);
  }

  _curbRumble(intensity) {
    this._oneShot('triangle', 54 + intensity * 38, 0.052 * intensity, 0.13, 0.55);
  }

  _pop(intensity) {
    this._oneShot('square', 145 + intensity * 130, 0.034 * intensity, 0.075);
  }

  _shift(upshift, intensity) {
    this._oneShot(upshift ? 'triangle' : 'sawtooth', upshift ? 440 : 330, 0.021 * intensity, upshift ? 0.06 : 0.085, upshift ? 0.56 : 0.28);
    this._oneShot('triangle', upshift ? 165 : 125, 0.015 * intensity, 0.075, upshift ? 0.42 : 0.24);
  }

  toggleMute() {
    this.muted = !this.muted;
    if (this.master && this.context) {
      this.master.gain.setTargetAtTime(this.muted ? 0 : 0.45, this.context.currentTime, 0.03);
    }
    return this.muted;
  }

  dispose() {
    for (const source of this.continuousSources) {
      try { source.stop?.(); } catch {}
      try { source.disconnect?.(); } catch {}
    }
    this.continuousSources.length = 0;
    try { this.master?.disconnect?.(); } catch {}
    this.context = null;
    this.master = null;
    this.ready = false;
  }
}
