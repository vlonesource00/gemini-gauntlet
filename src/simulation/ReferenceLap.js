/**
 * ReferenceLap.js
 * User Reference Lap Recorder & Telemetry Baseline Engine:
 * - Records player driving trajectory (distance, lateral offset, speed, timestamps)
 * - Tracks lap times, sectors, and lap completion
 * - Generates high-density interpolated reference racing lines and target speeds
 * - Calculates live lap delta time (+/- 0.000s) vs user baseline
 * - LocalStorage persistence and AI controller synchronization
 */

const finite = (val, fallback = 0) => (Number.isFinite(val) ? val : fallback);
const clamp = (val, min, max) => Math.max(min, Math.min(max, val));

export class ReferenceLapManager {
  /**
   * @param {Object} track - Circuit track instance
   */
  constructor(track) {
    this.track = track;
    this.trackLength = finite(track?.totalLength ?? track?.length, 3061.7);

    // Live lap tracking
    this.currentLapTime = 0;
    this.currentLapDistance = 0;
    this.previousDistance = 0;
    this.lapCount = 0;
    this.lastLapTime = null;
    this.bestLapTime = null;
    this.isRecording = true;

    // Buffer of samples for the ongoing lap
    this.liveSamples = [];
    this.sampleIntervalM = 2.0; // 2m high-resolution spatial sampling
    this.lastSampleDistance = -999;
    this.lapHistory = []; // All recorded laps

    // Active baseline profile (either user recorded or default optimal)
    this.userBaseline = null;
    this.activeProfile = null;

    this._loadSavedBaseline();
  }

  reset() {
    this.currentLapTime = 0;
    this.currentLapDistance = 0;
    this.previousDistance = 0;
    this.liveSamples = [];
    this.lastSampleDistance = -999;
  }

  /**
   * Update recorder with player vehicle telemetry every physics frame.
   * @param {Object} vehicle - Player vehicle
   * @param {number} dt - Step duration in seconds
   * @returns {Object} Live lap status { currentLapTime, lastLapTime, bestLapTime, deltaS, lapCompleted, newBest }
   */
  update(vehicle, dt) {
    if (!vehicle) return { currentLapTime: 0, deltaS: 0 };

    this.currentLapTime += dt;
    const dist = finite(vehicle.distance, 0);
    const speed = finite(vehicle.speed, 0);
    const speedKph = speed * 3.6;
    const lateral = finite(vehicle.surface?.lateral, 0);

    let lapCompleted = false;
    let newBest = false;

    // Detect start/finish line wrap-around (distance wraps from near end to near start)
    const crossedLine = this.previousDistance > (this.trackLength - 50) && dist < 50;

    if (crossedLine && this.currentLapTime > 15.0) {
      // Completed full lap!
      lapCompleted = true;
      this.lapCount += 1;
      this.lastLapTime = this.currentLapTime;

      if (!this.bestLapTime || this.currentLapTime < this.bestLapTime) {
        this.bestLapTime = this.currentLapTime;
        newBest = true;
      }

      // Finalize full telemetry lap profile
      const completedProfile = this._buildProfileFromSamples(this.liveSamples, this.currentLapTime, vehicle);
      this.lapHistory.push(completedProfile);

      // Automatically set new best lap as active baseline
      if (newBest || !this.userBaseline) {
        this.setBaselineFromProfile(completedProfile);
      }

      // Automatically trigger JSON export / persistence
      this.autoSaveTelemetryJSON(completedProfile);

      // Reset for next lap
      this.currentLapTime = 0;
      this.liveSamples = [];
      this.lastSampleDistance = -999;
    }

    this.previousDistance = dist;

    // Record high-resolution multi-channel spatial sample
    if (Math.abs(dist - this.lastSampleDistance) >= this.sampleIntervalM || this.lastSampleDistance < 0) {
      const controls = vehicle.controls ?? {};
      const powertrain = vehicle.powertrain ?? {};
      const ers = vehicle.ers ?? {};
      const surface = vehicle.surface ?? {};

      this.liveSamples.push({
        distance: Number(dist.toFixed(2)),
        timeS: Number(this.currentLapTime.toFixed(3)),
        x: Number(finite(vehicle.position?.x, 0).toFixed(2)),
        y: Number(finite(vehicle.position?.y, 0).toFixed(2)),
        z: Number(finite(vehicle.position?.z, 0).toFixed(2)),
        lateralOffset: Number(lateral.toFixed(3)),
        speedMps: Number(speed.toFixed(2)),
        speedKph: Number(speedKph.toFixed(1)),
        throttle: Number(finite(controls.throttle, 0).toFixed(2)),
        brake: Number(finite(controls.brake, 0).toFixed(2)),
        steer: Number(finite(controls.steer, 0).toFixed(3)),
        gear: powertrain.gear ?? 1,
        rpm: Math.round(finite(powertrain.rpm, 0)),
        yaw: Number(finite(vehicle.yaw, 0).toFixed(3)),
        yawRate: Number(finite(vehicle.yawRate, 0).toFixed(3)),
        lateralG: Number(finite(vehicle.telemetry?.lateralG, 0).toFixed(2)),
        longitudinalG: Number(finite(vehicle.telemetry?.longitudinalG, 0).toFixed(2)),
        slipAngleRad: Number(finite(vehicle.telemetry?.slipAngle, 0).toFixed(3)),
        ersSoc: Number(finite(ers.soc, 1.0).toFixed(3)),
        ersDeployKw: Number(finite(ers.deployPowerKw, 0).toFixed(1)),
        surfaceZone: surface.zone ?? 'road',
        surfaceGrip: Number(finite(surface.grip, 1.0).toFixed(2))
      });
      this.lastSampleDistance = dist;
    }

    // Compute live delta vs baseline
    const deltaS = this.calculateDelta(dist, this.currentLapTime);

    return {
      currentLapTime: this.currentLapTime,
      lastLapTime: this.lastLapTime,
      bestLapTime: this.bestLapTime,
      deltaS,
      lapCompleted,
      newBest,
      lapCount: this.lapCount,
      hasBaseline: Boolean(this.userBaseline)
    };
  }

  /**
   * Set the current live lap or last completed lap as the user baseline.
   */
  captureLiveBaseline(vehicle = null) {
    if (this.liveSamples.length < 20) {
      return { success: false, reason: 'NOT_ENOUGH_DATA' };
    }

    const lapTime = this.lastLapTime ?? this.currentLapTime;
    const profile = this._buildProfileFromSamples(this.liveSamples, lapTime, vehicle);
    this.setBaselineFromProfile(profile);
    return { success: true, lapTime, sampleCount: profile.samples.length };
  }

  /**
   * Assign a structured profile as the active user baseline.
   */
  setBaselineFromProfile(profile) {
    if (!profile || !profile.samples?.length) return;
    this.userBaseline = profile;
    this.activeProfile = profile;
    this._saveBaseline(profile);
  }

  /**
   * Clear the user baseline and revert to default.
   */
  clearBaseline() {
    this.userBaseline = null;
    this.activeProfile = null;
    try {
      localStorage.removeItem('gemini_gauntlet_user_baseline');
      localStorage.removeItem('gemini_gauntlet_last_lap_telemetry');
    } catch {
      // Storage quota or unavailable
    }
  }

  /**
   * Automatically save telemetry JSON to LocalStorage and provide export format.
   */
  autoSaveTelemetryJSON(profile) {
    try {
      const jsonStr = JSON.stringify(profile, null, 2);
      localStorage.setItem('gemini_gauntlet_last_lap_telemetry', jsonStr);
    } catch {
      // Storage full
    }
  }

  /**
   * Export structured JSON telemetry object or download file.
   */
  exportTelemetryJSON(lapIndex = null) {
    const profile = lapIndex !== null && this.lapHistory[lapIndex]
      ? this.lapHistory[lapIndex]
      : this.activeProfile ?? this.lapHistory[this.lapHistory.length - 1] ?? null;

    if (!profile) return null;
    return JSON.stringify(profile, null, 2);
  }

  /**
   * Trigger browser file download of recorded lap telemetry.
   */
  downloadTelemetryFile(lapIndex = null) {
    const jsonStr = this.exportTelemetryJSON(lapIndex);
    if (!jsonStr) return false;

    const profile = JSON.parse(jsonStr);
    const fileName = `lap_telemetry_${(profile.lapTime ?? 0).toFixed(3)}s_${Date.now()}.json`;
    const blob = new Blob([jsonStr], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = fileName;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
    return true;
  }

  /**
   * Get reference racing line and target speed at a specific track distance.
   * @param {number} distance - Track distance in meters
   * @returns {Object} { lineLateral, targetSpeed, referenceTimeS, throttle, brake }
   */
  paceAtDistance(distance) {
    const wrappedDist = ((distance % this.trackLength) + this.trackLength) % this.trackLength;

    if (this.activeProfile?.sampleMap) {
      const idx = clamp(Math.round(wrappedDist / this.activeProfile.resolutionM), 0, this.activeProfile.sampleMap.length - 1);
      const sample = this.activeProfile.sampleMap[idx];
      if (sample) {
        return {
          lineLateral: sample.lateralOffset ?? sample.lateral ?? 0,
          targetSpeed: sample.speedMps ?? sample.speed ?? 40.0,
          speedKph: sample.speedKph ?? (sample.speed ? sample.speed * 3.6 : 144.0),
          throttle: sample.throttle ?? 1.0,
          brake: sample.brake ?? 0.0,
          referenceTimeS: sample.timeS
        };
      }
    }

    // Default geometric line fallback
    const trackPoint = this.track?.atDistance ? this.track.atDistance(wrappedDist) : { curvature: 0 };
    const curv = finite(trackPoint.curvature, 0);
    const lineLateral = clamp(-Math.sign(curv) * Math.min(2.5, Math.abs(curv) * 600), -4.5, 4.5);
    const targetSpeed = Math.max(18, Math.sqrt(18.0 / Math.max(1e-4, Math.abs(curv))));

    return {
      lineLateral,
      targetSpeed,
      speedKph: targetSpeed * 3.6,
      throttle: 1.0,
      brake: 0.0,
      referenceTimeS: (wrappedDist / 45.0)
    };
  }

  /**
   * Calculate live delta vs baseline in seconds (+ = slower, - = faster).
   */
  calculateDelta(distance, currentLapTime) {
    if (!this.userBaseline) return 0;
    const ref = this.paceAtDistance(distance);
    if (!ref || !Number.isFinite(ref.referenceTimeS)) return 0;
    return currentLapTime - ref.referenceTimeS;
  }

  /**
   * Build a spatial lookup table from recorded discrete samples.
   * @private
   */
  _buildProfileFromSamples(samples, totalLapTime, vehicle = null) {
    if (!samples.length) return null;

    // Sort by distance
    const sorted = [...samples].sort((a, b) => a.distance - b.distance);
    const resolutionM = 2.0;
    const totalSteps = Math.ceil(this.trackLength / resolutionM);
    const sampleMap = [];

    let srcIdx = 0;
    let maxSpeedKph = 0;
    let sumSpeedKph = 0;

    for (let step = 0; step < totalSteps; step++) {
      const targetDist = step * resolutionM;

      // Find surrounding samples
      while (srcIdx < sorted.length - 1 && sorted[srcIdx + 1].distance < targetDist) {
        srcIdx++;
      }

      const p0 = sorted[srcIdx];
      const p1 = sorted[Math.min(sorted.length - 1, srcIdx + 1)];

      let lateral = p0.lateralOffset ?? p0.lateral ?? 0;
      let speed = p0.speedMps ?? p0.speed ?? 40.0;
      let speedKph = p0.speedKph ?? (speed * 3.6);
      let timeS = p0.timeS;
      let throttle = p0.throttle ?? 1.0;
      let brake = p0.brake ?? 0.0;

      if (p1 && p1.distance > p0.distance) {
        const t = clamp((targetDist - p0.distance) / (p1.distance - p0.distance), 0, 1);
        const lat1 = p1.lateralOffset ?? p1.lateral ?? lateral;
        const spd1 = p1.speedMps ?? p1.speed ?? speed;
        lateral = lateral + (lat1 - lateral) * t;
        speed = speed + (spd1 - speed) * t;
        speedKph = speed * 3.6;
        timeS = p0.timeS + (p1.timeS - p0.timeS) * t;
        throttle = throttle + ((p1.throttle ?? throttle) - throttle) * t;
        brake = brake + ((p1.brake ?? brake) - brake) * t;
      }

      maxSpeedKph = Math.max(maxSpeedKph, speedKph);
      sumSpeedKph += speedKph;

      sampleMap.push({
        distance: targetDist,
        lateralOffset: clamp(Number(lateral.toFixed(3)), -6.5, 6.5),
        speedMps: Math.max(8.0, Number(speed.toFixed(2))),
        speedKph: Number(speedKph.toFixed(1)),
        throttle: Number(throttle.toFixed(2)),
        brake: Number(brake.toFixed(2)),
        timeS: Number(timeS.toFixed(3))
      });
    }

    return {
      lapTime: totalLapTime,
      lapTimeFormatted: ReferenceLapManager.formatTime(totalLapTime),
      date: new Date().toISOString(),
      track: 'Endurance Park',
      trackLengthM: this.trackLength,
      carSpec: vehicle?.spec ?? 'prototype',
      maxSpeedKph: Number(maxSpeedKph.toFixed(1)),
      avgSpeedKph: Number((sumSpeedKph / totalSteps).toFixed(1)),
      resolutionM,
      sampleMap,
      samples: sorted
    };
  }

  _saveBaseline(profile) {
    try {
      const summary = {
        lapTime: profile.lapTime,
        lapTimeFormatted: profile.lapTimeFormatted,
        date: profile.date,
        resolutionM: profile.resolutionM,
        sampleMap: profile.sampleMap,
        samples: profile.samples
      };
      localStorage.setItem('gemini_gauntlet_user_baseline', JSON.stringify(summary));
    } catch {
      // Storage quota or unavailable
    }
  }

  _loadSavedBaseline() {
    try {
      const raw = localStorage.getItem('gemini_gauntlet_user_baseline');
      if (raw) {
        const parsed = JSON.parse(raw);
        if (parsed?.sampleMap?.length) {
          this.userBaseline = parsed;
          this.activeProfile = parsed;
        }
      }
    } catch {
      // Ignore
    }
  }

  static formatTime(seconds) {
    if (!Number.isFinite(seconds) || seconds <= 0) return '--:--.---';
    const mins = Math.floor(seconds / 60);
    const secs = Math.floor(seconds % 60);
    const millis = Math.floor((seconds % 1) * 1000);
    return `${String(mins).padStart(2, '0')}:${String(secs).padStart(2, '0')}.${String(millis).padStart(3, '0')}`;
  }
}
