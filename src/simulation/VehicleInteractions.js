import { clamp } from '../core/math.js';

const finite = (value, fallback = 0) => Number.isFinite(value) ? value : fallback;

const cleanWakeState = () => ({
  strength: 0,
  wakeStrength: 0,
    dragReduction: 0,
    frontDownforceLoss: 0,
    rearDownforceLoss: 0,
    dragMultiplier: 1,
    frontDownforceMultiplier: 1,
    rearDownforceMultiplier: 1,
  sourceId: null,
  sourceClass: null,
  source: null
});

const liveVehicle = (vehicle) => Boolean(vehicle && !vehicle.despawned && !vehicle.trafficGhost);

const speedOf = (vehicle) => {
  const vectorSpeed = Math.hypot(finite(vehicle?.velocity?.x), finite(vehicle?.velocity?.z));
  return Math.max(0, finite(vehicle?.speed, 0), vectorSpeed);
};

const wakeCalibration = (leader) => leader?.spec?.aero?.wake ?? {
  lengthM: 28, widthM: 3.7, maxDragReduction: 0.12,
  frontDownforceLoss: 0.2, rearDownforceLoss: 0.13
};

const writeWake = (vehicle, state) => {
  const wake = {
    strength: clamp(finite(state.strength), 0, 1),
    wakeStrength: clamp(finite(state.strength), 0, 1),
    dragReduction: clamp(finite(state.dragReduction), 0, 0.18),
    frontDownforceLoss: clamp(finite(state.frontDownforceLoss), 0, 0.38),
    rearDownforceLoss: clamp(finite(state.rearDownforceLoss), 0, 0.28),
    dragMultiplier: 1 - clamp(finite(state.dragReduction), 0, 0.18),
    frontDownforceMultiplier: 1 - clamp(finite(state.frontDownforceLoss), 0, 0.38),
    rearDownforceMultiplier: 1 - clamp(finite(state.rearDownforceLoss), 0, 0.28),
    sourceId: state.sourceId ?? null,
    sourceClass: state.sourceClass ?? null,
    source: state.sourceId ?? null
  };
  vehicle.wake = wake;
  if (vehicle.aero) {
    vehicle.aero.wakeStrength = wake.strength;
    vehicle.aero.dragReduction = wake.dragReduction;
    vehicle.aero.frontDownforceLoss = wake.frontDownforceLoss;
    vehicle.aero.rearDownforceLoss = wake.rearDownforceLoss;
    vehicle.aero.wakeSource = wake.sourceId;
    vehicle.aero.dragMultiplier = wake.dragMultiplier;
    vehicle.aero.frontDownforceMultiplier = wake.frontDownforceMultiplier;
    vehicle.aero.rearDownforceMultiplier = wake.rearDownforceMultiplier;
    vehicle.aero.wakeDragMultiplier = wake.dragMultiplier;
    vehicle.aero.wakeFrontDownforceMultiplier = wake.frontDownforceMultiplier;
    vehicle.aero.wakeRearDownforceMultiplier = wake.rearDownforceMultiplier;
  }
  if (vehicle.telemetry) {
    vehicle.telemetry.wakeStrength = wake.strength;
    vehicle.telemetry.dragReduction = wake.dragReduction;
    vehicle.telemetry.frontDownforceLoss = wake.frontDownforceLoss;
    vehicle.telemetry.rearDownforceLoss = wake.rearDownforceLoss;
    vehicle.telemetry.wakeSource = wake.sourceId;
    vehicle.telemetry.dragMultiplier = wake.dragMultiplier;
    vehicle.telemetry.frontDownforceMultiplier = wake.frontDownforceMultiplier;
    vehicle.telemetry.rearDownforceMultiplier = wake.rearDownforceMultiplier;
    vehicle.telemetry.wakeDragMultiplier = wake.dragMultiplier;
    vehicle.telemetry.wakeFrontDownforceMultiplier = wake.frontDownforceMultiplier;
    vehicle.telemetry.wakeRearDownforceMultiplier = wake.rearDownforceMultiplier;
  }
  return wake;
};

// Pairwise, world-space wake field.  The leader's forward/right basis is the
// only alignment authority, so a car ahead or beside the leader remains clean
// even if its track-distance ordering is stale during a contact.
export function updateAerodynamicWakes(vehicles = []) {
  const list = Array.isArray(vehicles) ? vehicles : [];
  for (const vehicle of list) {
    if (!vehicle) continue;
    if (typeof vehicle._resetWake === 'function') vehicle._resetWake();
    else writeWake(vehicle, {});
  }

  let activeWakes = 0;
  let maxStrength = 0;
  for (let leaderIndex = 0; leaderIndex < list.length; leaderIndex += 1) {
    const leader = list[leaderIndex];
    if (!liveVehicle(leader) || speedOf(leader) < 4) continue;
    const calibration = wakeCalibration(leader);
    const lengthM = clamp(finite(calibration.lengthM, 28), 12, 60);
    const widthM = clamp(finite(calibration.widthM, 3.7), 1.8, 8);
    const maxDragReduction = clamp(finite(calibration.maxDragReduction, 0.12), 0.02, 0.18);
    const maxFrontLoss = clamp(finite(calibration.frontDownforceLoss, 0.2), 0.04, 0.38);
    const maxRearLoss = clamp(finite(calibration.rearDownforceLoss, 0.13), 0.03, 0.28);
    const forward = leader.forward ?? { x: 0, z: 1 };
    const right = leader.right ?? { x: 1, z: 0 };
    const leaderSpeed = speedOf(leader);
    const speedFactor = clamp((leaderSpeed - 7) / 42, 0, 1);
    for (let followerIndex = 0; followerIndex < list.length; followerIndex += 1) {
      if (followerIndex === leaderIndex) continue;
      const follower = list[followerIndex];
      if (!liveVehicle(follower)) continue;
      const dx = finite(follower.position?.x) - finite(leader.position?.x);
      const dz = finite(follower.position?.z) - finite(leader.position?.z);
      const longitudinal = dx * forward.x + dz * forward.z;
      const behindDistance = -longitudinal;
      if (!(behindDistance > 1.8 && behindDistance < lengthM)) continue;
      const lateral = Math.abs(dx * right.x + dz * right.z);
      if (!(lateral < widthM)) continue;
      const longitudinalDecay = clamp(1 - behindDistance / lengthM, 0, 1);
      const lateralAlignment = Math.pow(clamp(1 - lateral / widthM, 0, 1), 1.35);
      // Retain a finite wake tail near the calibrated length; this keeps a
      // 25–30 m following car measurably helped without making the field
      // infinite or overpowering close racing.
      const distanceFactor = 0.35 + 0.65 * Math.sqrt(longitudinalDecay);
      const strength = clamp(speedFactor * lateralAlignment * distanceFactor, 0, 1);
      if (strength <= 0.005) continue;
      const existing = follower.wake ?? {};
      const existingStrength = finite(existing.strength, 0);
      const existingSource = String(existing.sourceId ?? '');
      const leaderId = String(leader.id ?? leaderIndex);
      if (strength < existingStrength - 1e-9 || (Math.abs(strength - existingStrength) <= 1e-9 && leaderId >= existingSource)) continue;
      writeWake(follower, {
        strength,
        dragReduction: maxDragReduction * strength,
        frontDownforceLoss: maxFrontLoss * strength,
        rearDownforceLoss: maxRearLoss * strength,
        sourceId: leader.id ?? leaderIndex,
        sourceClass: leader.classKey ?? leader.spec?.key ?? null
      });
    }
  }
  for (const vehicle of list) {
    if (!liveVehicle(vehicle)) continue;
    const strength = finite(vehicle.wake?.strength, 0);
    if (strength > 0.005) activeWakes += 1;
    maxStrength = Math.max(maxStrength, strength);
  }
  return { activeWakes, maxStrength, vehicleCount: list.length };
}

// Repeated deterministic pair solving lets a three-car pile-up unwind in the
// same fixed step while preserving the scalar Vehicle.collide() contract.
export function resolveVehicleCollisions(vehicles = [], iterations = 3) {
  const list = Array.isArray(vehicles) ? vehicles : [];
  const passes = clamp(Math.trunc(iterations), 1, 8);
  const stats = {
    iterations: passes,
    contacts: 0,
    pairs: 0,
    maxImpact: 0,
    maxPenetration: 0,
    deepOverlaps: 0,
    contactPairs: []
  };
  for (let iteration = 0; iteration < passes; iteration += 1) {
    for (let a = 0; a < list.length; a += 1) {
      const first = list[a];
      if (!liveVehicle(first)) continue;
      for (let b = a + 1; b < list.length; b += 1) {
        const second = list[b];
        if (!liveVehicle(second)) continue;
        const contact = first.obbContact?.(second);
        if (!contact) continue;
        stats.contacts += 1;
        stats.pairs += iteration === 0 ? 1 : 0;
        stats.maxPenetration = Math.max(stats.maxPenetration, finite(contact.penetration));
        const impact = finite(first.collide?.(second), 0);
        stats.maxImpact = Math.max(stats.maxImpact, impact);
        if (iteration === 0) stats.contactPairs.push({ a: first.id, b: second.id, impact });
      }
    }
  }
  for (let a = 0; a < list.length; a += 1) {
    const first = list[a];
    if (!liveVehicle(first)) continue;
    for (let b = a + 1; b < list.length; b += 1) {
      const second = list[b];
      if (!liveVehicle(second)) continue;
      const contact = first.obbContact?.(second);
      if (contact && contact.penetration > 0.18) stats.deepOverlaps += 1;
    }
  }
  return stats;
}

export default { updateAerodynamicWakes, resolveVehicleCollisions };
