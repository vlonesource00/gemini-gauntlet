import { clamp } from '../core/math.js';
import { CAR_COMMON } from './CarSpecs.js';

const KELVIN_OFFSET = 273.15;

const finite = (value, fallback = 0) => Number.isFinite(value) ? value : fallback;
const magic = (slip, B, C, E, peak) => {
  const x = clamp(slip, -3, 3);
  const bx = B * x;
  return peak * Math.sin(C * Math.atan(bx - E * (bx - Math.atan(bx))));
};

export function createTireState(spec = CAR_COMMON.tire) {
  const ambient = spec.ambientC ?? 22;
  const preheat = clamp(finite(spec.preheatC, ambient + 38), ambient - 4, 130);
  const carcassPreheat = clamp(finite(spec.preheatCarcassC, preheat - 3), ambient - 4, 140);
  const coldPressure = spec.coldPressurePa ?? 185000;
  const pressurePa = coldPressure * ((carcassPreheat + KELVIN_OFFSET) / (ambient + KELVIN_OFFSET));
  return {
    kappa: 0,
    alpha: 0,
    temperatureInnerC: clamp(preheat - 1.5, ambient - 4, 180),
    temperatureMiddleC: preheat,
    temperatureOuterC: clamp(preheat - 1.5, ambient - 4, 180),
    carcassTemperatureC: carcassPreheat,
    flashTemperatureC: preheat,
    pressurePa,
    wear: 0,
    energyJ: 0,
    thermalEnergyJ: 0,
    previousNormalLoadN: 0,
    lastFx: 0,
    lastFy: 0,
    lastMz: 0
  };
}

export const TireState = class TireState {
  constructor(spec) { Object.assign(this, createTireState(spec)); }
  reset(spec) { Object.assign(this, createTireState(spec)); }
};

// Pacejka-inspired, stateful brush model. It uses physically named inputs and
// a friction ellipse, while remaining deliberately small and stable at 120 Hz.
export function tireForces(args = {}, providedState = null, dt = 1 / 120) {
  const state = providedState ?? createTireState(args.spec ?? CAR_COMMON.tire);
  const spec = args.spec ?? CAR_COMMON.tire;
  const safeDt = clamp(finite(dt, 1 / 120), 1 / 1000, 1 / 20);
  const radius = Math.max(0.05, finite(args.radius, 0.335));
  const vx = finite(args.longitudinalVelocity);
  const vy = finite(args.lateralVelocity);
  const omega = finite(args.wheelAngularSpeed);
  const normalLoad = clamp(finite(args.normalLoad, 0), 0, 42000);
  const grip = clamp(finite(args.grip, 1), 0.12, 1.6);
  const camber = clamp(finite(args.camber), -0.22, 0.22);
  const speedReference = Math.max(2.5, Math.abs(vx));
  // Vehicle supplies small axle-specific stiffness/peak adjustments.  The
  // defaults preserve the public tireForces argument contract used by tests
  // and by external callers.
  const corneringStiffness = clamp(finite(args.corneringStiffness ?? args.lateralStiffness, 1), 0.55, 1.55);
  const longitudinalStiffness = clamp(finite(args.longitudinalStiffness, 1), 0.55, 1.55);
  const peakScale = clamp(finite(args.peakScale ?? args.peakForceScale, 1), 0.55, 1.3);
  const targetKappa = clamp((omega * radius - vx) / speedReference, -3, 3);
  // Positive lateral velocity is a movement to the tyre's right; the contact
  // patch force therefore points left, matching Vehicle's local basis.
  const targetAlpha = clamp(Math.atan2(vy, speedReference), -1.2, 1.2);
  const relaxX = Math.max(0.08, spec.relaxationLengthX ?? 0.42);
  const relaxY = Math.max(0.08, spec.relaxationLengthY ?? 0.58);
  const relaxRateX = Math.max(2, Math.abs(vx)) / relaxX;
  const relaxRateY = Math.max(2, Math.abs(vx)) / relaxY;
  state.kappa += (targetKappa - state.kappa) * (1 - Math.exp(-relaxRateX * safeDt));
  state.alpha += (targetAlpha - state.alpha) * (1 - Math.exp(-relaxRateY * safeDt));

  const nominalLoad = 3200;
  const loadRatio = Math.max(0.04, normalLoad / nominalLoad);
  const loadSensitivity = spec.loadSensitivity ?? 0.16;
  const muLoad = Math.max(0.52, 1 - loadSensitivity * Math.log(loadRatio));
  const idealTemp = spec.idealTempC ?? 88;
  const carcass = finite(state.carcassTemperatureC, spec.ambientC ?? 22);
  const temperatureFactor = clamp(1 - Math.pow((carcass - idealTemp) / 78, 2), 0.55, 1.04);
  const wearFactor = clamp(1 - finite(state.wear) * 0.43, 0.58, 1);
  const pressureRatio = clamp(finite(state.pressurePa, spec.coldPressurePa ?? 185000) / (spec.coldPressurePa ?? 185000), 0.82, 1.22);
  const pressureFactor = clamp(1 - Math.abs(pressureRatio - 1) * 0.22, 0.87, 1);
  const mu = (spec.mu ?? 1.35) * grip * muLoad * temperatureFactor * wearFactor * pressureFactor;
  const dx = Math.max(1, mu * normalLoad * peakScale);
  const dy = Math.max(1, mu * normalLoad * peakScale * (1 + Math.abs(camber) * 0.035));
  const longitudinal = spec.longitudinal ?? CAR_COMMON.tire.longitudinal;
  const lateral = spec.lateral ?? CAR_COMMON.tire.lateral;
  const fxPure = magic(state.kappa, longitudinal.B * longitudinalStiffness, longitudinal.C, longitudinal.E, dx);
  const fyPure = -magic(state.alpha, lateral.B * corneringStiffness, lateral.C, lateral.E, dy)
    - camber * normalLoad * (spec.camberStiffness ?? 0.09);
  const ux = Math.abs(fxPure) / dx;
  const uy = Math.abs(fyPure) / dy;
  const gx = Math.sqrt(Math.max(0.035, 1 - uy * uy * 0.92));
  const gy = Math.sqrt(Math.max(0.035, 1 - ux * ux * 0.86));
  let fx = fxPure * gx;
  let fy = fyPure * gy;
  const ellipse = Math.max(1, Math.hypot(fx / dx, fy / dy));
  fx /= ellipse;
  fy /= ellipse;
  const pneumaticTrail = clamp(0.105 * (1 - Math.min(0.92, Math.abs(state.alpha) * 3.3)), 0.012, 0.105);
  const mz = -fy * pneumaticTrail * (1 - Math.min(0.85, Math.abs(state.kappa) * 0.45));
  const utilisation = Math.min(1.5, Math.hypot(fx / dx, fy / dy));

  const surfaceSpeed = Math.max(1, Math.abs(vx));
  const slipPower = (Math.abs(fx * state.kappa) + Math.abs(fy * Math.tan(state.alpha))) * surfaceSpeed;
  const ambient = spec.ambientC ?? 22;
  const thermalMass = Math.max(1000, spec.thermalMass ?? 13500);
  const airflowCooling = Math.max(0, spec.airflowCooling ?? 0.018);
  const cooling = Math.max(0.001, spec.coolingRate ?? 0.032) * (1 + Math.abs(vx) * airflowCooling);
  // A rolling tyre dissipates energy even when it is not visibly sliding.
  // The load-change term is a compact hysteresis/deformation approximation;
  // it is bounded so kerbs and suspension transients cannot create spikes.
  const previousLoad = finite(state.previousNormalLoadN, normalLoad);
  const loadCycle = clamp(Math.abs(normalLoad - previousLoad) / Math.max(500, normalLoad), 0, 1.8);
  const deformationCoefficient = Math.max(0, spec.deformationHeatCoefficient ?? 0.012);
  const hysteresisCoefficient = Math.max(0, spec.hysteresisHeatCoefficient ?? 0.004);
  const deformationPower = normalLoad * Math.abs(vx) * deformationCoefficient;
  const hysteresisPower = normalLoad * Math.abs(vx) * hysteresisCoefficient * (0.28 + loadCycle * 0.72);
  const totalHeatPower = slipPower + deformationPower + hysteresisPower;
  const camberHeat = camber * 7;
  const heatGain = totalHeatPower / thermalMass;
  const zoneWeights = [1.12 - camberHeat * 0.025, 0.92, 1.12 + camberHeat * 0.025];
  const temperatureKeys = ['temperatureInnerC', 'temperatureMiddleC', 'temperatureOuterC'];
  for (let i = 0; i < temperatureKeys.length; i += 1) {
    const key = temperatureKeys[i];
    const temp = finite(state[key], ambient);
    state[key] = clamp(temp + (heatGain * zoneWeights[i] - (temp - ambient) * cooling) * safeDt, ambient - 5, 220);
  }
  const averageSurface = (state.temperatureInnerC + state.temperatureMiddleC + state.temperatureOuterC) / 3;
  state.flashTemperatureC = clamp(averageSurface + Math.min(42, slipPower / 1050), ambient - 5, 250);
  state.thermalEnergyJ = finite(state.thermalEnergyJ) + totalHeatPower * safeDt;
  state.carcassTemperatureC = clamp(
    finite(state.carcassTemperatureC, ambient) + ((averageSurface - state.carcassTemperatureC) * Math.max(0.02, spec.treadToCarcassRate ?? 0.24) - (state.carcassTemperatureC - ambient) * cooling * 0.18) * safeDt,
    ambient - 5, 180
  );
  state.pressurePa = clamp(
    (spec.coldPressurePa ?? 185000) * ((state.carcassTemperatureC + KELVIN_OFFSET) / ((ambient ?? 22) + KELVIN_OFFSET)),
    110000, 320000
  );
  // A locked or airborne wheel can create a one-frame numerical slip-power
  // spike.  Cap only the wear-energy channel (not force or temperature), so a
  // genuine slide still damages the tyre without erasing an entire stint in a
  // single solver sample.
  const wearSlipPower = Math.min(slipPower, 60000);
  state.wear = clamp(finite(state.wear) + wearSlipPower * (spec.wearRate ?? 1.35e-8) * safeDt, 0, 1);
  state.energyJ = finite(state.energyJ) + slipPower * safeDt;
  state.previousNormalLoadN = normalLoad;
  state.lastFx = fx;
  state.lastFy = fy;
  state.lastMz = mz;
  return {
    fx, fy, mz,
    slipRatio: state.kappa,
    slipAngle: state.alpha,
    targetSlipRatio: targetKappa,
    targetSlipAngle: targetAlpha,
    utilisation,
    temperature: averageSurface,
    temperatureInnerC: state.temperatureInnerC,
    temperatureMiddleC: state.temperatureMiddleC,
    temperatureOuterC: state.temperatureOuterC,
    carcassTemperatureC: state.carcassTemperatureC,
    flashTemperatureC: state.flashTemperatureC,
    pressurePa: state.pressurePa,
    pressureBar: state.pressurePa / 100000,
    wear: state.wear,
    pneumaticTrail,
    state
  };
}
