// Browser-scale race-car presets. Values use SI units unless a property names
// another unit explicitly. They are calibrated handling targets, not data from
// a proprietary vehicle or tire model.
const freeze = (value) => Object.freeze(value);

const common = {
  airDensity: 1.225,
  gravity: 9.81,
  wheelInertia: 1.42,
  wheelRadius: 0.335,
  finalDrive: 3.72,
  drivetrainEfficiency: 0.9,
  idleRpm: 1050,
  limiterRpm: 7700,
  engineInertia: 0.24,
  engineBrakeNm: 104,
  steeringLock: 0.51,
  steeringRate: 11,
  suspension: freeze({
    travel: 0.145,
    bumpStopStart: 0.116,
    bumpStopRate: 92000,
    slowBump: 2600,
    slowRebound: 3300,
    fastBump: 1600,
    fastRebound: 2050,
    camberGain: 0.56,
    staticCompression: 0.054
  }),
  tire: freeze({
    coldPressurePa: 185000,
    ambientC: 22,
    idealTempC: 88,
    // Race tyres leave the garage pre-heated.  The value is deliberately
    // below the optimum so the first lap still has a useful warm-up phase.
    preheatC: 60,
    thermalMass: 13500,
    coolingRate: 0.014,
    airflowCooling: 0.012,
    treadToCarcassRate: 0.24,
    deformationHeatCoefficient: 0.085,
    hysteresisHeatCoefficient: 0.018,
    wearRate: 1.35e-8,
    relaxationLengthX: 0.42,
    relaxationLengthY: 0.58,
    camberStiffness: 0.09,
    loadSensitivity: 0.16,
    longitudinal: freeze({ B: 10.8, C: 1.66, E: 0.78 }),
    lateral: freeze({ B: 8.9, C: 1.38, E: 0.81 })
  }),
  // These are high-level calibration inputs, named after the physical
  // effects they represent.  They are intentionally modest multipliers: the
  // tire model, load transfer and aero remain the source of handling truth.
  handling: freeze({
    axleGrip: freeze({ front: 1, rear: 1 }),
    axleStiffness: freeze({ front: 1, rear: 1 }),
    longitudinalStiffness: freeze({ front: 1, rear: 1 }),
    coastEngineBrakeScale: 1,
    yawDamping: 1,
    yawStability: 1,
    keyboardLateralTarget: 11.5,
    tcSlipTarget: 0.105,
    tcMaxTorqueReduction: 0.58,
    lowSpeedYawDampingBoost: 0,
    aeroPlatformResponse: 0,
    brakePressureResponse: 0,
    brakeInitialGain: 1
  })
};

export const CAR_SPECS = freeze({
  gt: freeze({
    ...common,
    key: 'gt', label: 'GT', subtitle: 'BALANCED ENDURANCE',
    mass: 1265, inertia: freeze({ x: 720, y: 1750, z: 2030 }),
    wheelBase: 2.68, trackWidth: 1.62, cgHeight: 0.49, weightFront: 0.52,
    drive: 'rear', maxTorqueNm: 520, torquePeakRpm: 5250,
    gearRatios: freeze([0, 3.16, 2.18, 1.62, 1.29, 1.04, 0.86]),
    brakeTorqueNm: 8900, brakeBias: 0.57,
    springRate: freeze({ front: 59000, rear: 64000 }), arb: freeze({ front: 9400, rear: 7600 }),
    aero: freeze({
      area: 1.78, cd: 0.72, frontCl: 0.88, rearCl: 1.16, groundEffect: 0.24,
      designRideHeight: 0.068, stallHeight: 0.026,
      // Wake calibration is deliberately bounded and immutable.  It only
      // scales the existing aero result when a following car is aligned.
      wake: freeze({
        lengthM: 32, widthM: 3.9, maxDragReduction: 0.155,
        frontDownforceLoss: 0.24, rearDownforceLoss: 0.15
      })
    }),
    lsd: freeze({ preloadNm: 112, driveRampDeg: 42, coastRampDeg: 58 }),
    tire: freeze({ ...common.tire, mu: 1.7, preheatC: 62, idealTempC: 88, wearRate: 2.35e-7 }),
    handling: freeze({
      ...common.handling,
      axleGrip: freeze({ front: 0.985, rear: 1.055 }),
      axleStiffness: freeze({ front: 1.015, rear: 1.035 }),
      longitudinalStiffness: freeze({ front: 1, rear: 1.045 }),
      coastEngineBrakeScale: 0.62,
      yawDamping: 1.34,
      yawStability: 1.18,
      keyboardLateralTarget: 10.5,
      keyboardAeroUtilization: 0.68,
      keyboardMaxLateralTarget: 18,
      tcSlipTarget: 0.105,
      tcMaxTorqueReduction: 0.54
    }),
    summary: '1,265 KG · RWD · 520 NM · 2.0 CL'
  }),
  prototype: freeze({
    ...common,
    key: 'prototype', label: 'PROTOTYPE', subtitle: 'GROUND-EFFECT ATTACK',
    mass: 925, inertia: freeze({ x: 520, y: 1260, z: 1480 }),
    wheelBase: 2.62, trackWidth: 1.72, cgHeight: 0.38, weightFront: 0.48,
    drive: 'rear', maxTorqueNm: 665, torquePeakRpm: 5850, steeringLock: 0.55,
    gearRatios: freeze([0, 3.04, 2.17, 1.65, 1.31, 1.08, 0.91]),
    brakeTorqueNm: 9800, brakeBias: 0.59,
    springRate: freeze({ front: 79000, rear: 86000 }), arb: freeze({ front: 12200, rear: 11300 }),
    aero: freeze({
      area: 1.52, cd: 0.81,
      frontCl: 1.86, rearCl: 2.04,
      groundEffect: 0.94, frontGroundShare: 0.49,
      designRideHeight: 0.048, lowRideOnset: 0.032,
      stallHeight: 0.014, stallFloor: 0.58,
      highRideOnset: 0.066, highRideFloor: 0.74,
      wake: freeze({
        lengthM: 35, widthM: 4.4, maxDragReduction: 0.18,
        frontDownforceLoss: 0.34, rearDownforceLoss: 0.22
      })
    }),
    lsd: freeze({ preloadNm: 146, driveRampDeg: 36, coastRampDeg: 53 }),
    tire: freeze({ ...common.tire, mu: 1.92, idealTempC: 94, preheatC: 70, wearRate: 1.75e-7, relaxationLengthX: 0.32, relaxationLengthY: 0.42 }),
    handling: freeze({
      ...common.handling,
      axleGrip: freeze({ front: 1.055, rear: 1.07 }),
      axleStiffness: freeze({ front: 1.08, rear: 1.06 }),
      longitudinalStiffness: freeze({ front: 1, rear: 1.04 }),
      coastEngineBrakeScale: 0.44,
      yawDamping: 1.58,
      yawStability: 1.19,
      lowSpeedYawDampingBoost: 0.28,
      keyboardLowSpeedBlendEnd: 14.5,
      aeroPlatformResponse: 10,
      brakePressureResponse: 22,
      brakeInitialGain: 0.58,
      keyboardLateralTarget: 10,
      keyboardAeroUtilization: 0.78,
      keyboardMaxLateralTarget: 32,
      tcSlipTarget: 0.125,
      tcMaxTorqueReduction: 0.5
    }),
    // Prototype-only, simplified LMDh-inspired rear-axle hybrid.  The
    // numbers describe the electrical system boundary, not a homologation
    // claim; the ICE remains below the combined 500 kW target in this model.
    ers: freeze({
      enabled: true,
      // A tactically finite store for the prototype abstraction.  This is
      // deliberately much smaller than a full endurance energy budget so a
      // driver can deploy it for a few meaningful attacks per lap.
      capacityJ: 8.0e6,
      initialSoc: 0.74,
      minSoc: 0.04,
      maxDeployPowerW: 120e3,
      // AUTO is a sustainable road-car strategy; ATTACK alone gets the full
      // electrical ceiling.  Lift harvest is a small, paid rear-axle brake
      // torque, distinct from the 200 kW brake-regen ceiling.
      autoDeployPowerW: 56e3,
      maxRegenPowerW: 200e3,
      maxLiftRegenPowerW: 44e3,
      maxLiftRegenTorqueNm: 1020,
      liftRegenThrottleThreshold: 0.12,
      deployEfficiency: 0.91,
      regenEfficiency: 0.72,
      // Torque limits are rear-axle values.  They are chosen against the
      // existing 0.335 m wheel radius and rear brake torque, after converting
      // power through axle angular speed rather than vehicle speed.
      maxDeployTorqueNm: 1350,
      maxRegenTorqueNm: 3000,
      minDeploySpeed: 4.5,
      maxDeploySpeed: 94,
      rearAxle: true
    }),
    summary: '925 KG · RWD · 665 NM + 120 KW ERS · 4.8 CL'
  }),
  touring: freeze({
    ...common,
    key: 'touring', label: 'TOURING', subtitle: 'FRONT-DRIVE COMBAT',
    mass: 1390, inertia: freeze({ x: 810, y: 1980, z: 2240 }),
    wheelBase: 2.72, trackWidth: 1.59, cgHeight: 0.53, weightFront: 0.61,
    drive: 'front', maxTorqueNm: 395, torquePeakRpm: 5000,
    gearRatios: freeze([0, 3.34, 2.13, 1.48, 1.13, 0.91, 0.76]),
    brakeTorqueNm: 8200, brakeBias: 0.64,
    springRate: freeze({ front: 54500, rear: 50000 }), arb: freeze({ front: 10300, rear: 5900 }),
    aero: freeze({
      area: 2.06, cd: 0.58, frontCl: 0.32, rearCl: 0.43, groundEffect: 0.06,
      designRideHeight: 0.078, stallHeight: 0.03,
      wake: freeze({
        lengthM: 34, widthM: 3.4, maxDragReduction: 0.12,
        frontDownforceLoss: 0.13, rearDownforceLoss: 0.085
      })
    }),
    lsd: freeze({ preloadNm: 132, driveRampDeg: 31, coastRampDeg: 65 }),
    tire: freeze({ ...common.tire, mu: 1.38, idealTempC: 82, preheatC: 58, wearRate: 2.25e-7, loadSensitivity: 0.19 }),
    handling: freeze({
      ...common.handling,
      axleGrip: freeze({ front: 1.015, rear: 1.06 }),
      axleStiffness: freeze({ front: 1.01, rear: 1.015 }),
      longitudinalStiffness: freeze({ front: 1.055, rear: 0.98 }),
      coastEngineBrakeScale: 0.5,
      yawDamping: 5,
      yawStability: 1.04,
      keyboardLateralTarget: 8,
      keyboardAeroUtilization: 0.52,
      keyboardMaxLateralTarget: 11.5,
      tcSlipTarget: 0.12,
      tcMaxTorqueReduction: 0.62
    }),
    summary: '1,390 KG · FWD · 395 NM · 0.8 CL'
  })
});

export const DEFAULT_CAR_CLASS = 'gt';

export function carSpecFor(value = DEFAULT_CAR_CLASS) {
  if (typeof value === 'string') return CAR_SPECS[value] ?? CAR_SPECS[DEFAULT_CAR_CLASS];
  if (value?.key && CAR_SPECS[value.key]) return CAR_SPECS[value.key];
  return CAR_SPECS[DEFAULT_CAR_CLASS];
}

export function carClassSummaries() {
  return Object.values(CAR_SPECS).map(({ key, label, subtitle, summary }) => ({ key, label, subtitle, summary }));
}

export { common as CAR_COMMON };
