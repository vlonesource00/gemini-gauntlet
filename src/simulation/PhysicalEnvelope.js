/**
 * PhysicalEnvelope.js
 * 
 * Canonical physical envelope and road boundary service for Gemini Supreme.
 * Centralizes vehicle capabilities across GlobalTimeOptimalEngine, PaceOptimizer,
 * FrenetLatticePlanner, and CoupledMPCC.
 * 
 * Accurately derives limits from actual vehicle spec, speed, aero downforce,
 * tire friction, and surface conditions.
 */

const G = 9.81;

const finite = (val, fallback = 0) => (Number.isFinite(val) ? val : fallback);
const clamp = (val, min, max) => Math.min(max, Math.max(min, val));

/**
 * Calculate physical acceleration envelope for a vehicle at a given speed.
 * 
 * @param {Object} options
 * @param {Object} [options.vehicle] - Vehicle or shadow vehicle
 * @param {number} [options.speed] - Speed in m/s
 * @param {Object} [options.surface] - Track surface data
 * @param {number} [options.tireGripFactor=1.0] - Tire wear / grip multiplier
 * @param {number} [options.aggression=0.5] - AI aggression factor [0..1]
 * @returns {Object} Physical envelope limits
 */
export function getPhysicalEnvelope({
  vehicle = null,
  speed = 0,
  surface = null,
  tireGripFactor = 1.0,
  aggression = 0.5
} = {}) {
  const vSpeed = Math.max(0.1, finite(speed, finite(vehicle?.speed, 0)));
  const spec = vehicle?.spec ?? {};
  const mass = finite(vehicle?.mass, finite(spec.mass, 1290));
  const vClass = vehicle?.classKey || (mass < 1000 ? 'prototype' : 'gt');

  // Surface and tire grip
  const surfaceGrip = finite(surface?.grip, 1.0);
  const baseTireGrip = finite(spec?.tire?.grip, finite(spec?.tyreGrip, 1.0));
  const effectiveGrip = baseTireGrip * tireGripFactor * surfaceGrip;

  let baseLatG = 1.15;
  let baseBrakeG = 1.05;
  let baseDriveAccel = 3.20;

  if (vClass === 'prototype') {
    // High-downforce prototype / Hypercar (ground effect aero)
    const dfFactor = clamp((vSpeed - 16.0) / 38.0, 0, 1.0);
    const aeroMult = clamp(1.0 + 0.00022 * vSpeed * vSpeed, 1.0, 1.55);
    baseLatG = (1.58 + 0.44 * dfFactor) * aeroMult;
    baseBrakeG = (2.20 + 1.30 * dfFactor) * aeroMult;
    baseDriveAccel = 4.80;
  } else if (vClass === 'gt') {
    // Astra GT / Gauntlet GT (Astra host plant calibration: 1.14G mechanical + modest aero)
    const dfFactor = clamp((vSpeed - 18.0) / 45.0, 0, 1.0);
    const aeroMult = clamp(1.0 + 0.00008 * vSpeed * vSpeed, 1.0, 1.22);
    baseLatG = (1.14 + 0.08 * dfFactor) * aeroMult;
    baseBrakeG = (1.10 + 0.15 * dfFactor) * aeroMult;
    baseDriveAccel = 3.60;
  } else {
    // Touring / road car
    baseLatG = 1.05;
    baseBrakeG = 1.00;
    baseDriveAccel = 2.80;
  }

  // Available capacities in m/s²
  const availableLatAccel = baseLatG * G * effectiveGrip;
  const availableBrakeAccel = baseBrakeG * G * effectiveGrip;
  const availableDriveAccel = baseDriveAccel * effectiveGrip;
  const combinedGripLimit = Math.max(availableLatAccel, availableBrakeAccel);

  return {
    availableLatAccel,
    availableBrakeAccel,
    availableDriveAccel,
    combinedGripLimit,
    peakLatG: baseLatG * effectiveGrip,
    peakBrakeG: baseBrakeG * effectiveGrip,
    vClass
  };
}

/**
 * Extract collision half-extents from vehicle instance or specifications.
 */
export function getVehicleBoundingExtents(vehicle) {
  const halfLength = finite(
    vehicle?.collisionHalfLength,
    finite(vehicle?.spec?.wheelBase, 2.78) * 0.5 + finite(vehicle?.spec?.collision?.overhangM, 0.56)
  );
  const halfWidth = finite(
    vehicle?.collisionHalfWidth,
    finite(vehicle?.spec?.trackWidth, 1.72) * 0.5 + finite(vehicle?.spec?.collision?.bodyMarginM, 0.15)
  );
  return { halfLength: Math.max(1.5, halfLength), halfWidth: Math.max(0.75, halfWidth) };
}

/**
 * Canonical legal center bounds service for track lateral position.
 * Returns the exact usable lateral range for the vehicle center, taking
 * into account vehicle width, track asphalt width, and kerb allowances.
 * 
 * @param {Object} track - Track instance or adapter
 * @param {number} s - Station distance along track
 * @param {Object} [vehicle] - Vehicle instance
 * @param {Object} [options]
 * @param {number} [options.kerbAllowance=0] - Extra kerb usage allowance in meters
 * @param {number} [options.safetyMargin=0.15] - Safety margin from asphalt edge in meters
 * @returns {Object} Legal bounds
 */
export function getLegalCenterBounds(track, s, vehicle, { kerbAllowance = 0, safetyMargin = 0.15 } = {}) {
  const egoExtents = getVehicleBoundingExtents(vehicle);
  const egoHalfWidth = egoExtents.halfWidth; // ~0.99 - 1.01m for GT
  const roadHalfWidth = finite(track?.roadHalfWidth, finite(track?.halfWidth, 8.2));
  const curbWidth = finite(track?.curbWidth, 1.25);
  const ref = track?.atDistance ? track.atDistance(s) : null;

  const asphaltLimit = Math.max(1.5, roadHalfWidth - egoHalfWidth - safetyMargin);

  let leftKerb = 0;
  let rightKerb = 0;
  if (kerbAllowance > 0) {
    const maxKerb = Math.min(kerbAllowance, curbWidth * 0.35);
    if (ref?.curbSide !== undefined && ref.curbSide !== 0) {
      // Authored kerb side: +1 = left, -1 = right
      if (ref.curbSide > 0) leftKerb = maxKerb;
      else rightKerb = maxKerb;
    } else if (ref?.turnSign !== undefined && ref.turnSign !== 0) {
      // In a corner, inside kerb is usable: turnSign > 0 (left turn) -> inside is left (+1)
      if (ref.turnSign > 0) leftKerb = maxKerb;
      else rightKerb = maxKerb;
    } else {
      leftKerb = maxKerb * 0.5;
      rightKerb = maxKerb * 0.5;
    }
  }

  return {
    minLateral: -(asphaltLimit + rightKerb),
    maxLateral: asphaltLimit + leftKerb,
    asphaltMin: -asphaltLimit,
    asphaltMax: asphaltLimit,
    usableKerbMin: -(asphaltLimit + rightKerb),
    usableKerbMax: asphaltLimit + leftKerb
  };
}
