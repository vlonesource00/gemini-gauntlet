/**
 * FrenetLatticePlanner.js
 * Modular Multi-Candidate Frenet Trajectory Lattice & Dynamic Line Adaptation Engine:
 * - Smooth C2 Quintic Minimum-Jerk Spatial Trajectories
 * - (1-x)^2 Parabolic Constant-Acceleration Line Rejoin (REJOIN_A = 4.0 m/s^2, ANCHOR_DEAD = 1.5m)
 * - Station-Stencil Derivative Differencing (dq/ds and d2q/ds2) & Comb-Free Path Curvature
 * - Zero-Hesitation Overtaking Candidate Generation & Saturating Intent Cost Field
 * - Dynamic Line Adaptation (Alternative Racing Lines when blocked or forced off-line)
 * - Dirty-Air Wake Avoidance Corridors
 * - Bounding-Capsule Collision Prediction & Clearance Assessment
 * - Candidate Selection Hysteresis (-22.0 bonus) and 3D Visual Spline Extraction
 */

import { clamp, wrap, wrapAngle } from '../core/math.js';
import { getPhysicalEnvelope, getVehicleBoundingExtents, getLegalCenterBounds } from '../simulation/PhysicalEnvelope.js';

const finite = (value, fallback = 0) => (Number.isFinite(value) ? value : fallback);

export const ANCHOR_DEAD = 1.5;      // metres of cross-track error controller owns outright
export const REJOIN_A = 4.0;         // m/s^2 constant lateral acceleration spent rejoining the line
export const LINE_PULL = 0.006;      // tie-breaking line pull weight
export const LINE_PULL_SAT = 4.0;    // metres^2 at which the pull saturates

/**
 * Quintic polynomial minimum-jerk lateral transition curve: S(u) = 10u^3 - 15u^4 + 6u^5.
 * Ensures continuous lateral position, velocity, and acceleration (C2 continuity)
 * with zero first and second derivatives at boundaries u=0 and u=1.
 * @param {number} value - Normalized progress parameter u in [0, 1]
 * @returns {number} Minimum-jerk blend value in [0, 1]
 */
export const minimumJerk = (value) => {
  const u = clamp(value, 0, 1);
  return u * u * u * (10 + u * (-15 + u * 6));
};

/**
 * First derivative of the quintic minimum-jerk polynomial: S'(u) = 30u^2(1 - u)^2.
 * @param {number} value - Normalized progress parameter u in [0, 1]
 * @returns {number} First derivative rate of change
 */
export const minimumJerkDerivative = (value) => {
  const u = clamp(value, 0, 1);
  return 30 * u * u * (1 - u) * (1 - u);
};

/**
 * Second derivative of the quintic minimum-jerk polynomial: S''(u) = 60u(1 - u)(1 - 2u).
 * @param {number} value - Normalized progress parameter u in [0, 1]
 * @returns {number} Second derivative (acceleration)
 */
export const minimumJerkSecondDerivative = (value) => {
  const u = clamp(value, 0, 1);
  return 60 * u * (1 - u) * (1 - 2 * u);
};

/**
 * Categorize a candidate trajectory into a persistent semantic ManeuverDescriptor.
 * Prevents 25Hz jitter between near-equivalent candidates by providing persistent family identity.
 * @param {Object} params
 * @returns {Object} ManeuverDescriptor { family, tacticalRole, flankSide, lateralBand, targetId, racecraftPhase }
 */
export function determineManeuverDescriptor({
  terminalLateral = 0,
  currentLateral = 0,
  intentType = 'PRIMARY_INTENT',
  racecraftPhase = 'NONE',
  targetId = null,
  trackCurvature = 0,
  turnSign = 0,
  roadMargin = 6.0
} = {}) {
  const signedCurv = turnSign * trackCurvature;
  const isCorner = Math.abs(trackCurvature) > 0.002;
  const cornerSide = signedCurv > 0 ? 1 : (signedCurv < 0 ? -1 : 0);
  const candidateSide = Math.sign(terminalLateral);

  let family = 'PACE_CENTER_FLOW';
  let tacticalRole = 'PACE';
  let lateralBand = Math.abs(terminalLateral) < roadMargin * 0.28 ? 'CENTER'
    : (candidateSide === cornerSide && isCorner ? 'INSIDE' : 'OUTSIDE');

  const isInside = isCorner && (
    (cornerSide > 0 && terminalLateral > 0.2) ||
    (cornerSide < 0 && terminalLateral < -0.2)
  );
  const isOutside = isCorner && (
    (cornerSide > 0 && terminalLateral < -0.2) ||
    (cornerSide < 0 && terminalLateral > 0.2)
  );

  const phaseUpper = String(racecraftPhase || '').toUpperCase();

  if (phaseUpper.includes('ATTACK') || phaseUpper === 'DIVEBOMB' || phaseUpper === 'SWITCHBACK' || phaseUpper === 'SLINGSHOT' || phaseUpper === 'SIDE_BY_SIDE' || phaseUpper === 'OVERTAKE') {
    tacticalRole = 'ATTACK';
    if (phaseUpper === 'SWITCHBACK' || intentType === 'SWITCHBACK') {
      family = 'ATTACK_SWITCHBACK';
    } else if (isInside || phaseUpper.includes('INSIDE') || intentType === 'ATTACK_INSIDE' || phaseUpper === 'DIVEBOMB') {
      family = 'ATTACK_INSIDE_DIVE';
    } else if (isOutside || phaseUpper.includes('OUTSIDE') || intentType === 'ATTACK_OUTSIDE') {
      family = 'ATTACK_OUTSIDE_MOMENTUM';
    } else if (phaseUpper === 'SLINGSHOT') {
      family = 'ATTACK_PULLOUT';
    } else if (phaseUpper === 'SIDE_BY_SIDE') {
      family = candidateSide >= 0 ? 'ATTACK_SIDE_BY_SIDE_RIGHT' : 'ATTACK_SIDE_BY_SIDE_LEFT';
    } else {
      family = 'ATTACK_CORRIDOR_COMMIT';
    }
  } else if (phaseUpper.includes('DEFEND') || phaseUpper === 'APEX_SHIELD' || phaseUpper === 'EXIT_SQUEEZE' || phaseUpper === 'BREAK_TOW' || phaseUpper === 'LOCK_LANE') {
    tacticalRole = 'DEFEND';
    if (phaseUpper === 'EXIT_SQUEEZE') {
      family = 'DEFEND_EXIT_SQUEEZE';
    } else if (phaseUpper === 'BREAK_TOW') {
      family = 'DEFEND_BREAK_TOW';
    } else if (isInside || phaseUpper === 'APEX_SHIELD' || phaseUpper.includes('INSIDE')) {
      family = 'DEFEND_INSIDE_SHIELD';
    } else {
      family = 'DEFEND_RACING_LINE_HOLD';
    }
  } else if (intentType === 'AVOIDANCE' || intentType.startsWith('TRAFFIC_BYPASS')) {
    tacticalRole = 'AVOIDANCE';
    family = terminalLateral > currentLateral ? 'TRAFFIC_BYPASS_RIGHT' : 'TRAFFIC_BYPASS_LEFT';
  } else {
    tacticalRole = 'PACE';
    if (isInside) {
      family = 'PACE_APEX';
    } else if (isOutside) {
      family = 'PACE_OUTSIDE_ENTRY';
    } else {
      family = 'PACE_CENTER_FLOW';
    }
  }

  return {
    family,
    tacticalRole,
    flankSide: candidateSide,
    lateralBand,
    targetId: targetId ?? null,
    racecraftPhase
  };
}

/**
 * Parabolic constant-acceleration decay function: w(x) = (1 - x)^2 for x in [0, 1).
 * Yields non-zero initial slope -2/L and constant second derivative 2/L^2,
 * perfectly closing tracking offset without receding-horizon fixed-point lag.
 * @param {number} shift - Offset to decay
 * @param {number} distance - Forward distance traveled
 * @param {number} span - Spatial decay span L
 * @returns {number} Decayed offset
 */
export const parabolicRejoin = (shift, distance, span) => {
  if (shift === 0 || span <= 0) return 0;
  const x = distance / span;
  return x >= 1.0 ? 0 : shift * (1 - x) * (1 - x);
};

/**
 * Exact differential geometry curvature & scale of Frenet path offset q(s)
 * with derivatives qp = dq/ds, qpp = d2q/ds2 off reference line of curvature k(s)
 * and curvature rate kp = dk/ds.
 * @param {number} k - Reference curvature
 * @param {number} kp - Reference curvature rate dk/ds
 * @param {number} q - Lateral offset
 * @param {number} qp - dq/ds
 * @param {number} qpp - d2q/ds2
 * @param {Object} [out]
 * @returns {Object} { kappa, scale }
 */
export const pathGeom = (k, kp, q, qp, qpp, out = { kappa: 0, scale: 1 }) => {
  const A = 1 - k * q;
  const B = qp;
  const Ap = -(kp * q + k * qp);
  const Bp = qpp;
  const n2 = A * A + B * B;
  const n1 = Math.sqrt(n2);
  out.kappa = Math.abs((n2 * k + A * Bp - B * Ap) / Math.max(1e-7, n2 * n1));
  out.scale = n1;
  return out;
};

/**
 * Filter unique lateral offset values within [min, max] clamped bounds.
 */
const uniqueOffsets = (values, min, max, tolerance = 0.08) => {
  const result = [];
  for (const val of values) {
    if (!Number.isFinite(val)) continue;
    const bounded = clamp(val, min, max);
    if (!result.some((existing) => Math.abs(existing - bounded) < tolerance)) {
      result.push(bounded);
    }
  }
  return result;
};

export const worldHeading = (a, b) => Math.atan2(b.x - a.x, b.z - a.z);

/**
 * Generalized quintic polynomial with arbitrary boundary conditions (q0, v0, a0) and (q1, v1, a1).
 * Yields exact C2 continuity at both boundaries.
 */
export class QuinticPolynomial {
  constructor(q0, v0, a0, q1, v1, a1, T) {
    this.T = Math.max(0.05, finite(T, 1.0));
    this.q0 = finite(q0, 0);
    this.q1 = finite(q1, 0);
    this.v0 = finite(v0, 0);
    this.v1 = finite(v1, 0);
    this.a0 = finite(a0, 0);
    this.a1 = finite(a1, 0);

    this.c0 = this.q0;
    this.c1 = this.v0;
    this.c2 = 0.5 * this.a0;

    const T2 = this.T * this.T;
    const T3 = T2 * this.T;
    const T4 = T3 * this.T;
    const T5 = T4 * this.T;
    const dq = this.q1 - this.q0;

    this.c3 = (20 * dq - (8 * this.v1 + 12 * this.v0) * this.T - (3 * this.a0 - this.a1) * T2) / (2 * T3);
    this.c4 = (-30 * dq + (14 * this.v1 + 16 * this.v0) * this.T + (3 * this.a0 - 2 * this.a1) * T2) / (2 * T4);
    this.c5 = (12 * dq - 6 * (this.v1 + this.v0) * this.T + (this.a1 - this.a0) * T2) / (2 * T5);
  }

  eval(t) {
    if (t <= 0) return this.c0;
    if (t >= this.T) return this.q1;
    const t2 = t * t;
    const t3 = t2 * t;
    const t4 = t3 * t;
    const t5 = t4 * t;
    return this.c0 + this.c1 * t + this.c2 * t2 + this.c3 * t3 + this.c4 * t4 + this.c5 * t5;
  }

  deriv(t) {
    if (t <= 0) return this.c1;
    if (t >= this.T) return this.v1;
    const t2 = t * t;
    const t3 = t2 * t;
    const t4 = t3 * t;
    return this.c1 + 2 * this.c2 * t + 3 * this.c3 * t2 + 4 * this.c4 * t3 + 5 * this.c5 * t4;
  }

  accel(t) {
    if (t <= 0) return 2 * this.c2;
    if (t >= this.T) return this.a1;
    const t2 = t * t;
    const t3 = t2 * t;
    return 2 * this.c2 + 6 * this.c3 * t + 12 * this.c4 * t2 + 20 * this.c5 * t3;
  }
}

/**
 * Two-stage piecewise quintic spline with exact C2 boundary matching across transition.
 */
export class TwoStageSpline {
  constructor(stage1, stage2) {
    this.poly1 = stage1;
    this.poly2 = stage2;
    this.T1 = stage1.T;
    this.T2 = stage2.T;
    this.T = this.T1 + this.T2;
  }

  eval(t) {
    if (t <= this.T1) return this.poly1.eval(t);
    return this.poly2.eval(t - this.T1);
  }

  deriv(t) {
    if (t <= this.T1) return this.poly1.deriv(t);
    return this.poly2.deriv(t - this.T1);
  }

  accel(t) {
    if (t <= this.T1) return this.poly1.accel(t);
    return this.poly2.accel(t - this.T1);
  }
}

/**
 * Compute Frenet frame kinematics (lateral velocity, longitudinal progress rate,
 * normal acceleration, and lateral acceleration qDDot) relative to an offset curve.
 *
 * Exact offset Frenet formulation:
 *   a_N = qDDot + (1 - kappa * q) * kappa * sDot^2
 *   qDDot = a_N - (1 - kappa * q) * kappa * sDot^2
 *
 * @param {Object} params
 * @param {{ x: number, z: number }} [params.velocity] - Vehicle planar world velocity
 * @param {{ x: number, z: number }} [params.acceleration] - Vehicle planar world acceleration
 * @param {number} [params.lateral=0] - Current lateral displacement q from reference centerline
 * @param {{ x: number, z: number }} params.tangent - Track unit tangent vector
 * @param {{ x: number, z: number }} params.normal - Track unit normal vector (left-positive)
 * @param {number} [params.curvature=0] - Track unsigned curvature magnitude (1/R)
 * @param {number} [params.turnSign=1] - Direction of turn (+1 for left, -1 for right)
 * @returns {{ qDot: number, sDotMeasured: number, sDot: number, aDotN: number, qDDot: number, denom: number }}
 */
export function computeFrenetKinematics({
  velocity,
  acceleration,
  lateral = 0,
  tangent,
  normal,
  curvature = 0,
  turnSign = 1
}) {
  const vx = finite(velocity?.x, 0);
  const vz = finite(velocity?.z, 0);
  const qDot = normal ? (vx * normal.x + vz * normal.z) : 0;
  const sDotMeasured = tangent ? (vx * tangent.x + vz * tangent.z) : 0;

  const kappaTrack = (turnSign ?? 1) * finite(curvature, 0);
  const currentLateral = finite(lateral, 0);
  const oneMinusKappaQ = 1.0 - kappaTrack * currentLateral;
  const denom = Math.abs(oneMinusKappaQ) < 0.1
    ? Math.sign(oneMinusKappaQ || 1) * 0.1
    : oneMinusKappaQ;

  const sDot = sDotMeasured / denom;

  const ax = finite(acceleration?.x, 0);
  const az = finite(acceleration?.z, 0);
  const aDotN = normal ? (ax * normal.x + az * normal.z) : 0;
  const qDDot = aDotN - denom * kappaTrack * sDot * sDot;

  return {
    qDot,
    sDotMeasured,
    sDot,
    aDotN,
    qDDot,
    denom
  };
}

export class FrenetLatticePlanner {
  /**
   * @param {Object} [options]
   * @param {number} [options.pointCount=24] - Number of discretized trajectory points
   * @param {number} [options.horizonS=3.4] - Planning time horizon in seconds
   */
  constructor({ pointCount = 24, horizonS = 3.4 } = {}) {
    this.pointCount = Math.max(12, Math.trunc(pointCount));
    this.horizonS = Math.max(2.8, finite(horizonS, 3.4));
    this.lastSelectedOffset = null;
    this.lastSelectedTrajectory = null;
    this.lastCandidates = [];
  }

  /**
   * Reset internal planner state.
   */
  reset() {
    this.lastSelectedOffset = null;
    this.lastSelectedTrajectory = null;
    this.lastCandidates = [];
  }

  /**
   * Generate and evaluate a single candidate trajectory.
   * @private
   */
  _evaluateCandidate({
    vehicle,
    track,
    startLateral,
    terminalLateral,
    desiredOffset,
    transitionTime,
    targetSpeed,
    trafficEntries,
    roadMargin,
    committed,
    aggression,
    horizon,
    targetId,
    referenceLineAtDistance,
    optimalSpeedAtDistance = null,
    kerbAllowance = 0,
    intentType = 'STANDARD',
    racecraftPhase = 'NONE',
    weights = {},
    curve = null,
    startV = 0,
    startA = 0,
    previousPlan = null,
    dtSinceLastPlan = 0.04,
    isDiagnostic = false
  }) {
    const points = [];
    const smoothQ = new Float64Array(this.pointCount);
    const startSpeed = Math.max(0, finite(vehicle?.speed, 0));
    const startEnvelope = getPhysicalEnvelope({
      vehicle,
      speed: startSpeed,
      aggression: clamp(finite(aggression, 0.5), 0, 1)
    });
    const maxBrakeDecel = Math.max(7.0, startEnvelope.availableBrakeAccel * 0.95);
    const maxDriveAccel = Math.max(4.0, startEnvelope.availableDriveAccel);
    const targetAccel = (finite(targetSpeed, startSpeed) - startSpeed) * 0.65;
    const acceleration = clamp(targetAccel, -maxBrakeDecel, maxDriveAccel);

    const egoExtents = getVehicleBoundingExtents(vehicle);

    let roadViolation = 0;
    let edgeRisk = 0;
    let collisionRisk = 0;
    let predictedCollisions = 0;
    let firstConflictPoint = null;
    let firstConflictStation = null;
    let firstConflictTime = null;
    let firstRoadViolationPoint = null;
    let minimumClearance = 99;
    let futureMinimumClearance = 99;
    let maxLateralAcceleration = 0;
    let maxCurvature = 0;
    let speedSum = 0;

    // Check if candidate follows a dynamic reference racing line
    const hasReference = typeof referenceLineAtDistance === 'function';
    const isPaceLine = intentType === 'PRIMARY_INTENT' || intentType === 'RACING_LINE' || intentType === 'PACE';

    // Initial lateral offset relative to nominal reference line
    const startDistance = finite(vehicle?.distance, 0);
    const startRef = track?.atDistance ? track.atDistance(startDistance) : { s: startDistance };
    const startBounds = getLegalCenterBounds(track, startRef.s, vehicle, { kerbAllowance });

    const refLine0 = hasReference
      ? clamp(finite(referenceLineAtDistance(startRef.s), 0), startBounds.minLateral, startBounds.maxLateral)
      : 0;

    // Only apply deadbanded parabolic shift when tracking reference line or recovering
    const dev0 = isPaceLine ? (startLateral - refLine0) : 0;
    const raw0 = dev0;
    const shift = Math.abs(raw0) <= ANCHOR_DEAD ? 0 : raw0 - Math.sign(raw0) * ANCHOR_DEAD;

    // Parabolic constant-acceleration rejoin span: L = v * sqrt(2 * |shift| / REJOIN_A)
    const vRejoin = Math.max(6.0, startSpeed);
    const rejoinL = shift !== 0 ? vRejoin * Math.sqrt((2 * Math.abs(shift)) / REJOIN_A) : 0;
    const rejoinSpan = Math.max(3.0, rejoinL);

    const poly = (!curve && !(isPaceLine && hasReference))
      ? new QuinticPolynomial(startLateral, startV, startA, terminalLateral, 0, 0, Math.max(0.2, transitionTime))
      : null;

    for (let index = 0; index < this.pointCount; index += 1) {
      const time = (horizon * index) / (this.pointCount - 1);
      const forwardDistance = Math.max(0, startSpeed * time + 0.5 * acceleration * time * time);
      const currentDistance = startDistance + forwardDistance;
      const speedCeiling = typeof optimalSpeedAtDistance === 'function'
        ? Math.max(12.0, optimalSpeedAtDistance(currentDistance) + 3.5)
        : 95.0;
      const predictedSpeed = Math.min(speedCeiling, clamp(startSpeed + acceleration * time, 0, 95));
      speedSum += predictedSpeed;

      const blend = minimumJerk(time / Math.max(0.2, transitionTime));

      const reference = track?.atDistance
        ? track.atDistance(currentDistance)
        : { s: currentDistance, x: 0, y: 0, z: 0, curvature: 0 };

      const refLineVal = hasReference
        ? finite(referenceLineAtDistance(reference.s), 0)
        : 0;

      const bounds = getLegalCenterBounds(track, reference.s, vehicle, { kerbAllowance });

      // Combine C2 quintic transitions with (1-x)^2 parabolic line rejoin
      let unclampedLateral;
      if (curve) {
        unclampedLateral = curve.eval(time);
      } else if (isPaceLine && hasReference) {
        const rejoinW = shift !== 0 ? parabolicRejoin(shift, forwardDistance, rejoinSpan) : 0;
        unclampedLateral = clamp(refLineVal, bounds.minLateral, bounds.maxLateral) + rejoinW;
      } else if (poly) {
        unclampedLateral = poly.eval(time);
      } else {
        const targetQ = clamp(terminalLateral, bounds.minLateral, bounds.maxLateral);
        unclampedLateral = startLateral + (targetQ - startLateral) * blend;
      }

      smoothQ[index] = unclampedLateral;
      const clampedLat = clamp(unclampedLateral, bounds.minLateral, bounds.maxLateral);

      // Phase 7: Compute raw world position for genuine unconstrained mathematical trajectory
      let rawWorld;
      if (track?.lateralPoint) {
        rawWorld = track.lateralPoint(reference, unclampedLateral, 0.08);
      } else {
        rawWorld = {
          x: reference.x ?? 0,
          y: (reference.y ?? 0) + 0.08,
          z: reference.z ?? 0
        };
      }

      const pointRoadLegal = unclampedLateral >= bounds.minLateral && unclampedLateral <= bounds.maxLateral;
      if (!pointRoadLegal) {
        const excess = unclampedLateral < bounds.minLateral
          ? (bounds.minLateral - unclampedLateral)
          : (unclampedLateral - bounds.maxLateral);
        roadViolation += excess + 1.0;
        if (!firstRoadViolationPoint) {
          firstRoadViolationPoint = {
            x: finite(rawWorld.x),
            y: finite(rawWorld.y),
            z: finite(rawWorld.z)
          };
        }
      }

      // Edge risk builds when close to the track boundary
      const edgeBuffer = Math.max(0.12, 0.40 - aggression * 0.22 - (kerbAllowance > 0 ? 0.12 : 0));
      const latMargin = Math.min(bounds.maxLateral - clampedLat, clampedLat - bounds.minLateral);
      if (latMargin < edgeBuffer) {
        edgeRisk += (edgeBuffer - latMargin) ** 2;
      }

      // Spatial bounding-capsule collision checking against traffic entries using actual path
      for (const entry of trafficEntries || []) {
        if (!entry?.other || entry.other.finished || entry.other.despawned || entry.other.trafficGhost) {
          continue;
        }

        const opponentExtents = getVehicleBoundingExtents(entry.other);
        const opponentProgress = Math.max(0, finite(entry.other.speed, 0) * time);
        const longitudinalGap = finite(entry.delta, 0) + opponentProgress - forwardDistance;
        const opponentStart = finite(entry.otherLateral, finite(entry.other.surface?.lateral, 0));
        const opponentTarget = finite(entry.otherTargetLateral, opponentStart);
        const opponentLateral = opponentStart + (opponentTarget - opponentStart) * minimumJerk(time / 1.35);

        const lateralGap = Math.abs(unclampedLateral - opponentLateral);

        // Physical spatial geometry
        const physicalHalfWidth = egoExtents.halfWidth + opponentExtents.halfWidth;
        const physicalHalfLength = egoExtents.halfLength + opponentExtents.halfLength;

        const longitudinalClearance = Math.abs(longitudinalGap) - (physicalHalfLength + 0.35);
        const lateralClearance = lateralGap - physicalHalfWidth;
        const combinedClearance = Math.max(longitudinalClearance, lateralClearance);

        minimumClearance = Math.min(minimumClearance, combinedClearance);
        if (time >= 0.4) {
          futureMinimumClearance = Math.min(futureMinimumClearance, combinedClearance);
        }

        const isPhysicalOverlap = longitudinalClearance < 0 && lateralClearance < 0;
        const isIncidentalNumericalContact = isPhysicalOverlap
          && lateralClearance > -0.06
          && Math.abs(entry.relativeLongitudinalVelocity || 0) < 3.5
          && Math.abs(entry.relativeLateralVelocity || 0) < 1.2;

        if (isPhysicalOverlap && !isIncidentalNumericalContact) {
          predictedCollisions += 1;
          collisionRisk += 35000 + (-longitudinalClearance + 0.25) * (-lateralClearance + 0.25) * 3500;
          if (!firstConflictPoint) {
            firstConflictPoint = {
              x: finite(rawWorld.x),
              y: finite(rawWorld.y),
              z: finite(rawWorld.z)
            };
            firstConflictStation = finite(reference.s);
            firstConflictTime = finite(time);
          }
        } else if (isIncidentalNumericalContact) {
          collisionRisk += 75.0 + (-lateralClearance) * 250.0;
        } else {
          const distAbs = Math.abs(longitudinalGap);
          const proximityHorizon = Math.max(4.5, 8.5 - aggression * 2.0);
          if (distAbs < proximityHorizon && lateralClearance < 0.85) {
            const timeDiscount = Math.max(0.2, 1.0 - time / Math.max(0.5, horizon));
            collisionRisk += (proximityHorizon - distAbs) * Math.max(0, 0.85 - lateralClearance) * 16 * timeDiscount;
          }
        }
      }

      points.push({
        x: finite(rawWorld.x),
        y: finite(rawWorld.y),
        z: finite(rawWorld.z),
        s: finite(reference.s),
        rawLateral: finite(unclampedLateral),
        lateral: finite(unclampedLateral),
        clampedLateral: finite(clampedLat),
        evaluatedWorld: track?.lateralPoint ? track.lateralPoint(reference, clampedLat, 0.08) : rawWorld,
        time: finite(time),
        speed: finite(predictedSpeed),
        predictedSpeed: finite(predictedSpeed),
        forwardDistance: finite(forwardDistance),
        curvature: 0,
        roadLegal: pointRoadLegal,
        bounds
      });
    }

    // Station-stencil derivative differencing: compute dq/ds and d2q/ds2 on smooth deviation profile
    const S = points.length;
    const d1 = new Float64Array(S);
    const d2 = new Float64Array(S);

    for (let index = 0; index < S; index += 1) {
      const im = index > 0 ? index - 1 : 0;
      const ip = index < S - 1 ? index + 1 : S - 1;
      const dsSpan = points[ip].forwardDistance - points[im].forwardDistance;
      d1[index] = dsSpan > 0.01 ? (smoothQ[ip] - smoothQ[im]) / dsSpan : 0;

      const ds1 = points[index].forwardDistance - points[im].forwardDistance;
      const ds2 = points[ip].forwardDistance - points[index].forwardDistance;
      const dsAvg = 0.5 * (ds1 + ds2);
      d2[index] = (ip > index && index > im && dsAvg > 0.01 && ds1 > 0.005 && ds2 > 0.005)
        ? ((smoothQ[ip] - smoothQ[index]) / ds2 - (smoothQ[index] - smoothQ[im]) / ds1) / dsAvg
        : 0;
    }

    const gScratch = { kappa: 0, scale: 1 };

    for (let index = 0; index < S; index += 1) {
      const curr = points[index];
      const ref = track?.atDistance ? track.atDistance(curr.s) : { curvature: 0, turnSign: 0 };
      const rawCurv = finite(ref?.curvature, 0);
      const turnSign = ref?.turnSign !== undefined ? ref.turnSign : (rawCurv > 0 ? 1 : 0);
      const signedK = turnSign * rawCurv;

      let kp = finite(ref?.curvRate, 0);
      if (!ref?.curvRate && track?.atDistance) {
        const refP = track.atDistance(curr.s + 2.0);
        const refM = track.atDistance(curr.s - 2.0);
        const sKP = finite(refP?.turnSign, 1) * finite(refP?.curvature, 0);
        const sKM = finite(refM?.turnSign, 1) * finite(refM?.curvature, 0);
        kp = (sKP - sKM) / 4.0;
      }

      let pointLatAccel;
      if (curve && typeof curve.accel === 'function') {
        const curveAccel = finite(curve.accel(curr.time), 0);
        pointLatAccel = Math.abs(curveAccel + curr.predictedSpeed ** 2 * signedK);
        curr.curvature = pointLatAccel / Math.max(1.0, curr.predictedSpeed ** 2);
      } else {
        pathGeom(signedK, kp, curr.lateral, d1[index], d2[index], gScratch);
        curr.curvature = gScratch.kappa;
        pointLatAccel = curr.predictedSpeed ** 2 * gScratch.kappa;
      }
      maxCurvature = Math.max(maxCurvature, curr.curvature);
      maxLateralAcceleration = Math.max(maxLateralAcceleration, pointLatAccel);
    }

    // Physical envelope derived from canonical service (Phase 5)
    const avgSpeed = speedSum / this.pointCount;
    const envelope = getPhysicalEnvelope({
      vehicle,
      speed: avgSpeed || startSpeed,
      aggression
    });
    const availableLatG = envelope.availableLatAccel * (1.0 + kerbAllowance * 0.08);
    const dynamicExcess = Math.max(0, maxLateralAcceleration - availableLatG);

    let dynamicsState = 'FEASIBLE';
    if (dynamicExcess > 3.50) {
      dynamicsState = 'HARD_INFEASIBLE';
    } else if (dynamicExcess > 0.10) {
      dynamicsState = 'SOFT_EXCESS';
    }
    const dynamicallyFeasible = dynamicsState !== 'HARD_INFEASIBLE';

    // Multi-objective cost weighting
    const wProg = weights.prog ?? (0.8 + aggression * 0.4);
    const wColl = weights.coll ?? 1.0;
    const wEdge = weights.edge ?? (42.0 * (1.0 - aggression * 0.45));
    const wAccel = weights.accel ?? 3.5;
    const wJerk = weights.jerk ?? (committed ? 0.7 : 1.3);
    const wIntent = weights.intent ?? (committed ? 190.0 : 45.0);

    const lateralDelta = Math.abs(terminalLateral - startLateral);
    const intentError = Math.abs(terminalLateral - desiredOffset);

    // Reward clipping inside apex curb during cornering
    const trackPoint = track?.atDistance ? track.atDistance(vehicle?.distance ?? 0) : { curvature: 0, turnSign: 0 };
    const trackCurvMag = finite(trackPoint?.curvature, 0);
    const trackSign = trackPoint?.turnSign !== undefined ? trackPoint.turnSign : 0;
    const trackSignedCurv = trackSign * trackCurvMag;

    const isInsideApex = (trackSignedCurv > 0.003 && terminalLateral > 0) || (trackSignedCurv < -0.003 && terminalLateral < 0);
    const kerbReward = (kerbAllowance > 0 && isInsideApex) ? (0.6 + aggression * 0.8) : 0;
    const rewardWidth = isInsideApex ? -(kerbReward + 0.5) : 0;

    const costRoadViolation = roadViolation * 1e6;
    const costCollision = collisionRisk * wColl;
    const costEdge = edgeRisk * wEdge;
    const costAccel = (dynamicsState === 'SOFT_EXCESS')
      ? dynamicExcess * dynamicExcess * wAccel * 8.0
      : (dynamicsState === 'HARD_INFEASIBLE' ? 1e5 + dynamicExcess * 1e4 : 0);
    const costJerk = (lateralDelta * 0.20 + (committed ? transitionTime * 1.0 : transitionTime * 0.22)) * wJerk;
    const costIntent = intentError * intentError * wIntent;

    // Compute persistent semantic maneuver descriptor
    const maneuver = determineManeuverDescriptor({
      terminalLateral,
      currentLateral: startLateral,
      intentType,
      racecraftPhase,
      targetId,
      trackCurvature: trackCurvMag,
      turnSign: trackSign,
      roadMargin
    });

    // Near-horizon continuity and trajectory switching cost against previously active plan
    let costSwitching = 0;
    let costNearHorizon = 0;
    let meanNearHorizonDivergence = 0;
    let maxNearHorizonDivergence = 0;
    let rmsNearHorizonDivergence = 0;
    let maxNearHorizonHeadingDiscontinuity = 0;
    let maxNearHorizonCurvatureDiscontinuity = 0;

    if (previousPlan && Array.isArray(previousPlan.points) && previousPlan.points.length > 0) {
      let pathDiffSum = 0;
      let nearHorizonDivSum = 0;
      let nearHorizonSqSum = 0;
      let nearHorizonCount = 0;
      let nearHorizonWeightedCost = 0;
      const dt = horizon / Math.max(1, this.pointCount - 1);
      const prevPoints = previousPlan.points;
      const nPrev = prevPoints.length;

      for (let i = 0; i < points.length; i++) {
        const pt = points[i];
        const t = pt.time;
        const tCheck = t + dtSinceLastPlan;
        let pLat = prevPoints[nPrev - 1].lateral;
        let pCurv = prevPoints[nPrev - 1].curvature || 0;
        let pDot = 0;
        let pDDot = 0;
        let pSpeed = Math.max(1.0, finite(prevPoints[nPrev - 1].predictedSpeed ?? prevPoints[nPrev - 1].speed, startSpeed));

        // 1. Temporally aligned previous curvature and speed from points
        if (tCheck <= prevPoints[0].time) {
          pCurv = prevPoints[0].curvature || 0;
          pSpeed = Math.max(1.0, finite(prevPoints[0].predictedSpeed ?? prevPoints[0].speed, startSpeed));
        } else if (tCheck >= prevPoints[nPrev - 1].time) {
          pCurv = prevPoints[nPrev - 1].curvature || 0;
          pSpeed = Math.max(1.0, finite(prevPoints[nPrev - 1].predictedSpeed ?? prevPoints[nPrev - 1].speed, startSpeed));
        } else {
          for (let j = 0; j < nPrev - 1; j++) {
            if (tCheck >= prevPoints[j].time && tCheck <= prevPoints[j + 1].time) {
              const span = Math.max(1e-4, prevPoints[j + 1].time - prevPoints[j].time);
              const frac = (tCheck - prevPoints[j].time) / span;
              pCurv = (prevPoints[j].curvature || 0) + frac * ((prevPoints[j + 1].curvature || 0) - (prevPoints[j].curvature || 0));
              const s0 = finite(prevPoints[j].predictedSpeed ?? prevPoints[j].speed, startSpeed);
              const s1 = finite(prevPoints[j + 1].predictedSpeed ?? prevPoints[j + 1].speed, startSpeed);
              pSpeed = Math.max(1.0, s0 + frac * (s1 - s0));
              break;
            }
          }
        }

        // 2. Temporally aligned position and derivatives
        if (previousPlan.curve && typeof previousPlan.curve.eval === 'function') {
          pLat = previousPlan.curve.eval(tCheck);
          pDot = typeof previousPlan.curve.deriv === 'function' ? previousPlan.curve.deriv(tCheck) : 0;
          pDDot = typeof previousPlan.curve.accel === 'function' ? previousPlan.curve.accel(tCheck) : 0;
        } else if (previousPlan.poly && typeof previousPlan.poly.eval === 'function') {
          pLat = previousPlan.poly.eval(tCheck);
          pDot = typeof previousPlan.poly.deriv === 'function' ? previousPlan.poly.deriv(tCheck) : 0;
          pDDot = typeof previousPlan.poly.accel === 'function' ? previousPlan.poly.accel(tCheck) : 0;
        } else if (tCheck <= prevPoints[0].time) {
          pLat = prevPoints[0].lateral;
          pDot = 0;
        } else if (tCheck < prevPoints[nPrev - 1].time) {
          for (let j = 0; j < nPrev - 1; j++) {
            if (tCheck >= prevPoints[j].time && tCheck <= prevPoints[j + 1].time) {
              const span = Math.max(1e-4, prevPoints[j + 1].time - prevPoints[j].time);
              const frac = (tCheck - prevPoints[j].time) / span;
              pLat = prevPoints[j].lateral + frac * (prevPoints[j + 1].lateral - prevPoints[j].lateral);
              pDot = (prevPoints[j + 1].lateral - prevPoints[j].lateral) / span;
              break;
            }
          }
        }

        const dLat = pt.lateral - pLat;
        const absDLat = Math.abs(dLat);
        pathDiffSum += dLat * dLat * dt;

        // Near-horizon window t in [0, 0.85s] heavily weighted in first 0.4s
        if (t <= 0.85) {
          let cDot = 0;
          let cDDot = 0;
          if (curve && typeof curve.deriv === 'function') {
            cDot = curve.deriv(t);
            cDDot = typeof curve.accel === 'function' ? curve.accel(t) : 0;
          } else if (poly && typeof poly.deriv === 'function') {
            cDot = poly.deriv(t);
            cDDot = typeof poly.accel === 'function' ? poly.accel(t) : 0;
          } else if (i > 0) {
            cDot = (pt.lateral - points[i - 1].lateral) / Math.max(1e-4, pt.time - points[i - 1].time);
          }

          const cSpeed = Math.max(1.0, finite(pt.predictedSpeed ?? pt.speed, startSpeed));
          const cHeading = Math.atan2(cDot, cSpeed);
          const pHeading = Math.atan2(pDot, pSpeed);

          const dDot = Math.abs(cDot - pDot);
          const dDDot = Math.abs(cDDot - pDDot);
          const dCurv = Math.abs((pt.curvature || 0) - pCurv);
          const dHeading = Math.abs(wrapAngle(cHeading - pHeading));

          const wT = clamp(1.0 - t / 0.8, 0, 1.0);
          nearHorizonWeightedCost += wT * (
            absDLat * absDLat * 60.0 +
            dDot * dDot * 15.0 +
            dDDot * dDDot * 2.0 +
            dCurv * dCurv * 40.0
          ) * dt;

          nearHorizonDivSum += absDLat;
          nearHorizonSqSum += absDLat * absDLat;
          maxNearHorizonDivergence = Math.max(maxNearHorizonDivergence, absDLat);
          maxNearHorizonHeadingDiscontinuity = Math.max(maxNearHorizonHeadingDiscontinuity, dHeading);
          maxNearHorizonCurvatureDiscontinuity = Math.max(maxNearHorizonCurvatureDiscontinuity, dCurv);
          nearHorizonCount += 1;
        }
      }

      costNearHorizon = nearHorizonWeightedCost;
      costSwitching = pathDiffSum * 24.0 + costNearHorizon;
      meanNearHorizonDivergence = nearHorizonCount > 0 ? (nearHorizonDivSum / nearHorizonCount) : 0;
      rmsNearHorizonDivergence = nearHorizonCount > 0 ? Math.sqrt(nearHorizonSqSum / nearHorizonCount) : 0;

      const prevTargetLat = previousPlan.selectedOffset ?? previousPlan.terminalLateral;
      if (Number.isFinite(prevTargetLat)) {
        const prevDir = Math.sign(prevTargetLat - startLateral);
        const newDir = Math.sign(terminalLateral - startLateral);
        if (prevDir !== 0 && newDir !== 0 && prevDir !== newDir && Math.abs(terminalLateral - prevTargetLat) > 0.8) {
          costSwitching += 35.0;
        }
      }

      const prevFamily = previousPlan.maneuver?.family;
      if (prevFamily && prevFamily === maneuver.family) {
        costSwitching -= 12.0;
      } else if (prevFamily && prevFamily !== maneuver.family) {
        costSwitching += 15.0;
      }
    }

    const rewardProgress = -avgSpeed * wProg;
    const costHysteresis = (this.lastSelectedOffset !== null && Math.abs(terminalLateral - this.lastSelectedOffset) < 0.25) ? -22.0 : 0;

    const totalScore = costRoadViolation
      + costCollision
      + costEdge
      + costAccel
      + costJerk
      + costIntent
      + costSwitching
      + rewardProgress
      + rewardWidth
      + costHysteresis;

    // Phase 4: Separate immutable physical constraints from selection status
    const constraints = {
      roadLegal: roadViolation < 1e-4,
      collisionFree: predictedCollisions === 0,
      dynamicsState,
      maxLatAccel: maxLateralAcceleration,
      availableLatAccel: availableLatG,
      dynamicExcess,
      maxCurvature,
      minimumClearance
    };

    const selection = {
      state: 'UNEVALUATED',
      selected: false,
      switchRejected: false,
      score: totalScore
    };

    let initialRejectionReason = 'VIABLE_ALTERNATIVE';
    let conflictPt = firstConflictPoint;
    if (!constraints.roadLegal) {
      initialRejectionReason = 'ROAD_LIMIT';
      conflictPt = conflictPt || firstRoadViolationPoint;
    } else if (!constraints.collisionFree) {
      initialRejectionReason = 'COLLISION';
    } else if (dynamicsState === 'HARD_INFEASIBLE') {
      initialRejectionReason = 'DYNAMIC_LIMIT';
    } else if (dynamicsState === 'SOFT_EXCESS') {
      initialRejectionReason = 'SOFT_DYNAMIC_EXCESS';
    }

    const candidateId = `traj_${terminalLateral.toFixed(2)}_${transitionTime.toFixed(2)}_${intentType}${isDiagnostic ? '_diag' : ''}`;

    return {
      id: candidateId,
      points,
      score: totalScore,
      terminalLateral,
      transitionTime,
      intentType,
      maneuver,
      meanNearHorizonDivergence,
      maxNearHorizonDivergence,
      rmsNearHorizonDivergence,
      maxNearHorizonHeadingDiscontinuity,
      maxNearHorizonCurvatureDiscontinuity,
      collisionFree: constraints.collisionFree,
      roadLegal: constraints.roadLegal,
      dynamicallyFeasible,
      constraints,
      selection,
      minimumClearanceM: minimumClearance,
      futureMinimumClearanceM: futureMinimumClearance,
      maxCurvaturePerM: maxCurvature,
      maxLateralAccelerationMps2: maxLateralAcceleration,
      rejectionReason: initialRejectionReason,
      conflictPoint: conflictPt,
      conflictStation: firstConflictStation,
      conflictTime: firstConflictTime,
      isDiagnostic: Boolean(isDiagnostic),
      curve: curve || poly,
      costBreakdown: {
        roadViolation: costRoadViolation,
        collisionRisk: costCollision,
        edgeRisk: costEdge,
        accelerationExcess: costAccel,
        jerk: costJerk,
        intent: costIntent,
        switching: costSwitching,
        nearHorizonCost: costNearHorizon,
        progressReward: rewardProgress,
        trackWidthReward: rewardWidth,
        hysteresisBonus: costHysteresis
      }
    };
  }

  /**
   * Plan optimal Frenet trajectory from candidate lattice.
   * @param {Object} params
   * @returns {Object} Optimal trajectory and candidate diagnostics
   */
  plan({
    vehicle,
    track,
    desiredOffset = 0,
    fallbackOffsets = [],
    tacticalCandidates = [], // Array of { offset, intentType, transitionScales, curve }
    trafficEntries = [],
    targetSpeed = vehicle?.speed ?? 0,
    aggression = 0.5,
    racecraftPhase = 'NONE',
    targetId = null,
    recovering = false,
    pitActive = false,
    urgent = false,
    roadMargin = null,
    kerbAllowance = 0,
    dMin = null,
    dMax = null,
    lookAhead = 12,
    trackingDistance = null,
    referenceLineAtDistance = null,
    optimalSpeedAtDistance = null,
    weights = {},
    previousPlan = null,
    dtSinceLastPlan = 0.04,
    includeDiagnostics = false
  }) {
    const currentLateral = finite(vehicle?.surface?.lateral, 0);
    const startDistance = finite(vehicle?.distance, 0);
    const startBounds = getLegalCenterBounds(track, startDistance, vehicle, { kerbAllowance });

    const minBound = Number.isFinite(dMin) ? Math.max(startBounds.minLateral, dMin) : startBounds.minLateral;
    const maxBound = Number.isFinite(dMax) ? Math.min(startBounds.maxLateral, dMax) : startBounds.maxLateral;
    const intendedOffset = clamp(finite(desiredOffset), minBound, maxBound);
    const committed = pitActive || ['SLINGSHOT', 'ATTACK', 'ATTACK_LEFT', 'ATTACK_RIGHT', 'ATTACK_INSIDE', 'ATTACK_OUTSIDE', 'DIVEBOMB', 'SWITCHBACK', 'DEFEND_LEFT', 'DEFEND_RIGHT', 'DEFEND_INSIDE', 'BREAK_TOW', 'APEX_SHIELD', 'EXIT_SQUEEZE', 'ABORT_HOLD', 'ABORT_BLEND'].includes(racecraftPhase);
    const urgentManeuver = committed || recovering || urgent;

    // Phase 3: Frenet state semantics derived from actual track frame
    const ref0 = track?.atDistance ? track.atDistance(startDistance) : null;
    let qDotMeasured = 0;
    let sDotMeasured = finite(vehicle?.speed, 0);
    let qDDotMeasured = 0;

    if (ref0 && ref0.normal && ref0.tangent) {
      const vx = finite(vehicle?.velocity?.x, finite(vehicle?.speed, 0) * Math.sin(vehicle?.yaw || 0));
      const vz = finite(vehicle?.velocity?.z, finite(vehicle?.speed, 0) * Math.cos(vehicle?.yaw || 0));
      const ax = finite(vehicle?.acceleration?.x, 0);
      const az = finite(vehicle?.acceleration?.z, 0);

      const kinematics = computeFrenetKinematics({
        velocity: { x: vx, z: vz },
        acceleration: { x: ax, z: az },
        lateral: currentLateral,
        tangent: ref0.tangent,
        normal: ref0.normal,
        curvature: ref0.curvature,
        turnSign: ref0.turnSign
      });

      qDotMeasured = kinematics.qDot;
      sDotMeasured = kinematics.sDotMeasured;
      qDDotMeasured = kinematics.qDDot;
    }

    // Active-trajectory warm start: reconcile plan derivatives with measured physical velocity
    let startV = 0;
    let startA = 0;
    if (previousPlan && !recovering && dtSinceLastPlan < 0.35) {
      let qDotPlan = 0;
      let qDDotPlan = 0;
      if (previousPlan.curve && typeof previousPlan.curve.deriv === 'function') {
        qDotPlan = finite(previousPlan.curve.deriv(dtSinceLastPlan), 0);
        qDDotPlan = finite(previousPlan.curve.accel(dtSinceLastPlan), 0);
      } else if (previousPlan.poly && typeof previousPlan.poly.deriv === 'function') {
        qDotPlan = finite(previousPlan.poly.deriv(dtSinceLastPlan), 0);
        qDDotPlan = finite(previousPlan.poly.accel(dtSinceLastPlan), 0);
      } else if (Array.isArray(previousPlan.points) && previousPlan.points.length >= 2) {
        const pts = previousPlan.points;
        const idx = pts.findIndex((p) => p.time >= dtSinceLastPlan);
        if (idx > 0) {
          const p0 = pts[idx - 1];
          const p1 = pts[idx];
          const dtSpan = Math.max(1e-4, p1.time - p0.time);
          qDotPlan = (p1.lateral - p0.lateral) / dtSpan;
        }
      }
      startV = clamp(qDotPlan * 0.70 + qDotMeasured * 0.30, -4.5, 4.5);
      startA = clamp(qDDotPlan * 0.70 + qDDotMeasured * 0.30, -8.0, 8.0);
    } else {
      startV = clamp(qDotMeasured, -4.5, 4.5);
      startA = clamp(qDDotMeasured, -8.0, 8.0);
    }

    // Collect lateral target offsets for lattice generation
    const customOffsetEntries = [];
    if (tacticalCandidates && tacticalCandidates.length > 0) {
      for (const tc of tacticalCandidates) {
        if (Number.isFinite(tc.offset) || tc.curve) {
          customOffsetEntries.push({
            offset: Number.isFinite(tc.offset) ? clamp(tc.offset, minBound, maxBound) : intendedOffset,
            intentType: tc.intentType || 'TACTICAL',
            transitionScales: tc.transitionScales || null,
            curve: tc.curve || null
          });
        }
      }
    }

    const envelope = getPhysicalEnvelope({
      vehicle,
      speed: vehicle?.speed,
      aggression
    });
    const availableLatAccel = envelope.availableLatAccel;
    if (racecraftPhase === 'SLINGSHOT') {
      const latSpan = Math.abs(intendedOffset - currentLateral);
      const T1 = clamp(Math.sqrt(5.8 * latSpan / Math.max(2.0, availableLatAccel)), 0.80, 1.60);
      const T2 = 1.20;
      const p1 = new QuinticPolynomial(currentLateral, startV, startA, intendedOffset, 0, 0, T1);
      const p2 = new QuinticPolynomial(intendedOffset, 0, 0, intendedOffset, 0, 0, T2);
      customOffsetEntries.push({
        offset: intendedOffset,
        intentType: 'SLINGSHOT_TWO_STAGE',
        transitionScales: [1.0],
        curve: new TwoStageSpline(p1, p2)
      });
    } else if (racecraftPhase === 'SWITCHBACK') {
      const wideLat = clamp(currentLateral + (Math.sign(currentLateral) || 1) * 0.9, minBound, maxBound);
      const latSpan = Math.abs(wideLat - currentLateral);
      const T1 = clamp(Math.sqrt(5.8 * latSpan / Math.max(2.0, availableLatAccel)), 0.80, 1.40);
      const T2 = clamp(Math.sqrt(5.8 * Math.abs(intendedOffset - wideLat) / Math.max(2.0, availableLatAccel)), 0.90, 1.60);
      const p1 = new QuinticPolynomial(currentLateral, startV, startA, wideLat, 0, 0, T1);
      const p2 = new QuinticPolynomial(wideLat, 0, 0, intendedOffset, 0, 0, T2);
      customOffsetEntries.push({
        offset: intendedOffset,
        intentType: 'SWITCHBACK_TWO_STAGE',
        transitionScales: [1.0],
        curve: new TwoStageSpline(p1, p2)
      });
    } else if (racecraftPhase === 'ABORT_HOLD' || racecraftPhase === 'ABORT_BLEND') {
      const T1 = 0.50;
      const T2 = 1.50;
      const p1 = new QuinticPolynomial(currentLateral, startV, startA, currentLateral, 0, 0, T1);
      const p2 = new QuinticPolynomial(currentLateral, 0, 0, intendedOffset, 0, 0, T2);
      customOffsetEntries.push({
        offset: intendedOffset,
        intentType: 'SOFT_ABORT',
        transitionScales: [1.0],
        curve: new TwoStageSpline(p1, p2)
      });
    }

    // Phase 2: Genuine control candidates only. No spanSteps or corridorSpan pollution!
    const oppositeLane = intendedOffset > 0.5 ? -Math.min(startBounds.maxLateral * 0.75, intendedOffset) : (intendedOffset < -0.5 ? Math.min(-startBounds.minLateral * 0.75, -intendedOffset) : 0);
    const rawPool = recovering
      ? [intendedOffset, currentLateral, 0]
      : [
          intendedOffset,
          ...fallbackOffsets,
          currentLateral,
          0,
          oppositeLane,
          startBounds.minLateral * 0.65,
          startBounds.maxLateral * 0.65
        ];

    if (!recovering && Math.abs(intendedOffset - currentLateral) > 0.6) {
      rawPool.push(currentLateral + 0.5 * (intendedOffset - currentLateral));
    }

    const candidatePool = committed
      ? rawPool.map((c) => clamp(c, minBound, maxBound))
      : rawPool;

    // Zero-hesitation overtaking flank candidate generation
    if (!recovering && trafficEntries && trafficEntries.length > 0) {
      const egoExtents = getVehicleBoundingExtents(vehicle);
      for (const entry of trafficEntries) {
        if (!entry?.other || entry.other.finished || entry.other.despawned || entry.other.trafficGhost) continue;
        const delta = finite(entry.delta, 999);
        if (delta < -2.0 || delta > Math.max(25.0, lookAhead * 2.0)) continue;

        const oppExtents = getVehicleBoundingExtents(entry.other);
        const oppLat = finite(entry.otherLateral, finite(entry.other.surface?.lateral, 0));
        const passGap = egoExtents.halfWidth + oppExtents.halfWidth + 0.95;

        if (Math.abs(intendedOffset - oppLat) < passGap) {
          const leftFlank = clamp(oppLat - passGap, minBound, maxBound);
          const rightFlank = clamp(oppLat + passGap, minBound, maxBound);
          candidatePool.push(leftFlank, rightFlank);
        }
      }
    }

    // Filter out intermediate candidates that fall inside an active opponent's occupied lateral zone
    const egoExtents = getVehicleBoundingExtents(vehicle);
    const cleanPool = candidatePool.filter((offset) => {
      if (Math.abs(offset - intendedOffset) < 0.05) return true;
      if (Math.abs(offset - currentLateral) < 0.05) return true;
      if (fallbackOffsets.some((fb) => Math.abs(offset - fb) < 0.05)) return true;
      for (const entry of trafficEntries || []) {
        if (!entry?.other || entry.other.finished || entry.other.despawned || entry.other.trafficGhost) continue;
        const delta = finite(entry.delta, 999);
        if (delta < -2.0 || delta > Math.max(25.0, lookAhead * 1.8)) continue;
        const egoExtents = getVehicleBoundingExtents(vehicle);
        const oppExtents = getVehicleBoundingExtents(entry.other);
        const passGap = egoExtents.halfWidth + oppExtents.halfWidth + 0.85;
        const oppLat = finite(entry.otherLateral, finite(entry.other.surface?.lateral, 0));
        if (Math.abs(offset - oppLat) < passGap - 0.15) {
          return false;
        }
      }
      return true;
    });

    const standardOffsets = uniqueOffsets(cleanPool, minBound, maxBound);

    const lateralDelta = Math.abs(intendedOffset - currentLateral);
    const physicalMinTime = Math.sqrt(5.8 * lateralDelta / Math.max(2.0, availableLatAccel));
    const vSpeed = finite(vehicle?.speed, 10);
    const speedTransitionFloor = vSpeed > 40.0 ? 1.25 : (urgentManeuver ? 0.75 : 1.05);

    const nominalTransition = clamp(
      physicalMinTime * (urgentManeuver ? 0.98 : 1.08) + (urgentManeuver ? 0.15 : 0.28),
      speedTransitionFloor,
      urgentManeuver ? 2.4 : 3.0
    );

    const defaultScales = urgentManeuver ? [0.85, 1.0, 1.25] : [0.85, 1.0, 1.30];
    const horizon = Math.max(this.horizonS, finite(lookAhead, 12) / Math.max(5, finite(vehicle?.speed, 10)));

    const candidateTrajectories = [];

    // Evaluate standard offsets
    for (const offset of standardOffsets) {
      const intentType = Math.abs(offset - intendedOffset) < 0.08 ? 'PRIMARY_INTENT'
        : Math.abs(offset - currentLateral) < 0.08 ? 'HOLD_LANE' : 'FALLBACK';

      for (const scale of defaultScales) {
        candidateTrajectories.push(this._evaluateCandidate({
          vehicle,
          track,
          startLateral: currentLateral,
          terminalLateral: offset,
          desiredOffset: intendedOffset,
          transitionTime: nominalTransition * scale,
          targetSpeed,
          trafficEntries,
          roadMargin: startBounds.maxLateral,
          committed: urgentManeuver,
          aggression: clamp(finite(aggression, 0.5), 0, 1),
          horizon,
          targetId,
          referenceLineAtDistance,
          optimalSpeedAtDistance,
          kerbAllowance,
          intentType,
          racecraftPhase,
          weights,
          curve: null,
          startV,
          startA,
          previousPlan,
          dtSinceLastPlan,
          isDiagnostic: false
        }));
      }
    }

    // Evaluate tactical custom candidate offsets
    for (const entry of customOffsetEntries) {
      const scales = entry.transitionScales || [1.0];
      for (const scale of scales) {
        candidateTrajectories.push(this._evaluateCandidate({
          vehicle,
          track,
          startLateral: currentLateral,
          terminalLateral: entry.offset,
          desiredOffset: intendedOffset,
          transitionTime: nominalTransition * scale,
          targetSpeed,
          trafficEntries,
          roadMargin: startBounds.maxLateral,
          committed: urgentManeuver,
          aggression: clamp(finite(aggression, 0.5), 0, 1),
          horizon,
          targetId,
          referenceLineAtDistance,
          optimalSpeedAtDistance,
          kerbAllowance,
          intentType: entry.intentType,
          racecraftPhase,
          weights,
          curve: entry.curve || null,
          startV,
          startA,
          previousPlan,
          dtSinceLastPlan,
          isDiagnostic: false
        }));
      }
    }

    // Loyalty hysteresis bonus for continuing the ongoing candidate
    if (previousPlan && Number.isFinite(previousPlan.selectedOffset)) {
      const prevFamily = previousPlan.maneuver?.family;
      const prevOffset = previousPlan.selectedOffset ?? previousPlan.terminalLateral;

      let ongoing = null;
      if (prevFamily) {
        const matchingFamily = candidateTrajectories.filter((c) =>
          c.maneuver?.family === prevFamily && c.constraints.roadLegal && c.constraints.collisionFree
        );
        if (matchingFamily.length > 0) {
          ongoing = [...matchingFamily].sort((a, b) =>
            (a.meanNearHorizonDivergence ?? 0) - (b.meanNearHorizonDivergence ?? 0)
            || a.score - b.score
          )[0];
        }
      }
      if (!ongoing && Number.isFinite(prevOffset)) {
        ongoing = candidateTrajectories.find((c) =>
          c.constraints.roadLegal && c.constraints.collisionFree && Math.abs(c.terminalLateral - prevOffset) < 0.35
        );
      }
      if (ongoing) {
        ongoing.score -= 35.0;
      }
    }

    // Sort candidates by total score
    candidateTrajectories.sort((a, b) => a.score - b.score
      || Math.abs(a.terminalLateral - intendedOffset) - Math.abs(b.terminalLateral - intendedOffset)
      || a.transitionTime - b.transitionTime);

    // Filter safe candidates using hard dynamic feasibility gate (Phase 4 & 5)
    const safeCandidates = candidateTrajectories.filter(
      (c) => c.constraints.collisionFree && c.constraints.roadLegal && c.constraints.dynamicsState !== 'HARD_INFEASIBLE'
    );

    // Pick best candidate, fallback with collision/road priority while preserving score continuity
    let selected = safeCandidates[0] ?? [...candidateTrajectories].sort((a, b) => {
      if (a.constraints.collisionFree !== b.constraints.collisionFree) {
        return a.constraints.collisionFree ? -1 : 1;
      }
      if (a.constraints.roadLegal !== b.constraints.roadLegal) {
        return a.constraints.roadLegal ? -1 : 1;
      }
      return Math.abs(a.terminalLateral - intendedOffset) - Math.abs(b.terminalLateral - intendedOffset)
        || a.score - b.score;
    })[0];

    // Net Switch Benefit Gate:
    // Only switch away from ongoing candidate if new candidate decisively improves score after continuity/switch margins
    let switchOverriddenCandidate = null;

    if (previousPlan) {
      const prevFamily = previousPlan.maneuver?.family;
      const prevTarget = previousPlan.selectedOffset ?? previousPlan.terminalLateral;

      let ongoingCandidate = null;
      if (prevFamily) {
        const safeFamilyMatches = safeCandidates.filter((c) => c.maneuver?.family === prevFamily);
        if (safeFamilyMatches.length > 0) {
          ongoingCandidate = [...safeFamilyMatches].sort((a, b) =>
            (a.meanNearHorizonDivergence ?? 0) - (b.meanNearHorizonDivergence ?? 0)
            || a.score - b.score
          )[0];
        }
      }
      if (!ongoingCandidate && Number.isFinite(prevTarget)) {
        ongoingCandidate = safeCandidates.find((c) => Math.abs(c.terminalLateral - prevTarget) < 0.35);
      }

      if (ongoingCandidate && selected !== ongoingCandidate) {
        const ongoingSafe = ongoingCandidate.constraints.collisionFree
          && ongoingCandidate.constraints.roadLegal
          && ongoingCandidate.constraints.dynamicsState !== 'HARD_INFEASIBLE'
          && (ongoingCandidate.minimumClearanceM ?? 99) >= 0.80;

        if (!ongoingSafe) {
          // Immediate emergency safety override: safety always trumps hysteresis
        } else {
          const sameFamily = selected.maneuver?.family && selected.maneuver.family === ongoingCandidate.maneuver?.family;
          const isDirectionReversal = Math.sign(selected.terminalLateral - currentLateral) !== Math.sign(ongoingCandidate.terminalLateral - currentLateral)
            && Math.abs(selected.terminalLateral - ongoingCandidate.terminalLateral) > 0.8;

          const switchMargin = sameFamily ? 5.0 : (isDirectionReversal ? 35.0 : 18.0);
          if (selected.score > ongoingCandidate.score - switchMargin) {
            switchOverriddenCandidate = selected;
            selected = ongoingCandidate;
          }
        }
      }
    }

    this.lastSelectedOffset = selected.terminalLateral;
    this.lastSelectedTrajectory = selected;

    // Selection status
    selected.selected = true;
    selected.selection.selected = true;
    selected.selection.state = 'SELECTED';
    selected.rejectionReason = 'SELECTED';

    if (switchOverriddenCandidate) {
      switchOverriddenCandidate.selection.switchRejected = true;
      switchOverriddenCandidate.selection.state = 'SWITCH_MARGIN';
      switchOverriddenCandidate.rejectionReason = 'SWITCH_MARGIN';
    }

    let viableSafeCount = 0;
    for (const cand of safeCandidates) {
      if (cand === selected || cand === switchOverriddenCandidate) continue;
      if (viableSafeCount < 5) {
        cand.selection.state = 'SAFE_ALTERNATIVE';
        cand.rejectionReason = 'VIABLE_ALTERNATIVE';
        viableSafeCount += 1;
      } else {
        cand.selection.state = 'HIGHER_COST';
        cand.rejectionReason = 'HIGHER_COST';
      }
    }

    for (const cand of candidateTrajectories) {
      if (cand === selected || cand === switchOverriddenCandidate || safeCandidates.includes(cand)) continue;
      if (!cand.constraints.roadLegal) {
        cand.selection.state = 'ROAD_LIMIT';
        cand.rejectionReason = 'ROAD_LIMIT';
      } else if (!cand.constraints.collisionFree) {
        cand.selection.state = 'COLLISION';
        cand.rejectionReason = 'COLLISION';
      } else if (cand.constraints.dynamicsState === 'HARD_INFEASIBLE') {
        cand.selection.state = 'DYNAMIC_LIMIT';
        cand.rejectionReason = 'DYNAMIC_LIMIT';
      }
    }

    // Diagnostic Candidates: generated strictly for visual inspection and never enter control (Phase 2)
    const diagnosticCandidates = [];
    if (includeDiagnostics) {
      const diagOffsets = [startBounds.minLateral * 0.90, startBounds.minLateral * 0.40, startBounds.maxLateral * 0.40, startBounds.maxLateral * 0.90];
      for (const diagOffset of diagOffsets) {
        if (!standardOffsets.some((so) => Math.abs(so - diagOffset) < 0.25)) {
          const diagCand = this._evaluateCandidate({
            vehicle,
            track,
            startLateral: currentLateral,
            terminalLateral: diagOffset,
            desiredOffset: intendedOffset,
            transitionTime: nominalTransition,
            targetSpeed,
            trafficEntries,
            roadMargin: startBounds.maxLateral,
            committed: urgentManeuver,
            aggression: clamp(finite(aggression, 0.5), 0, 1),
            horizon,
            targetId,
            referenceLineAtDistance,
            kerbAllowance,
            intentType: 'DIAGNOSTIC_PROBE',
            racecraftPhase,
            weights,
            curve: null,
            startV,
            startA,
            previousPlan,
            dtSinceLastPlan,
            isDiagnostic: true
          });
          diagCand.selected = false;
          diagCand.selection.state = 'DIAGNOSTIC_PROBE';
          diagnosticCandidates.push(diagCand);
        }
      }
    }

    // Build bounded 24-32 candidate presentation bundle for visual inspection (Phase 2 & 14)
    const visualCandidates = [selected];
    const seenIds = new Set([selected.id]);
    const addVisualCandidate = (cand) => {
      if (!cand || seenIds.has(cand.id)) return;
      seenIds.add(cand.id);
      visualCandidates.push(cand);
    };

    if (switchOverriddenCandidate) {
      addVisualCandidate(switchOverriddenCandidate);
    }
    for (const cand of safeCandidates) {
      if (visualCandidates.length >= 10) break;
      addVisualCandidate(cand);
    }
    for (const cand of candidateTrajectories) {
      if (visualCandidates.length >= 24) break;
      addVisualCandidate(cand);
    }

    // If candidateTrajectories has fewer than 24 paths, evaluate diagnostic exploration probes
    if (visualCandidates.length < 24) {
      const diagOffsets = [
        startBounds.minLateral * 0.95,
        startBounds.minLateral * 0.70,
        startBounds.minLateral * 0.45,
        startBounds.minLateral * 0.20,
        startBounds.maxLateral * 0.20,
        startBounds.maxLateral * 0.45,
        startBounds.maxLateral * 0.70,
        startBounds.maxLateral * 0.95
      ];
      for (const diagOffset of diagOffsets) {
        if (visualCandidates.length >= 26) break;
        if (candidateTrajectories.some((c) => Math.abs(c.terminalLateral - diagOffset) < 0.25)) continue;
        for (const scale of defaultScales) {
          if (visualCandidates.length >= 26) break;
          const diagCand = this._evaluateCandidate({
            vehicle,
            track,
            startLateral: currentLateral,
            terminalLateral: diagOffset,
            desiredOffset: intendedOffset,
            transitionTime: nominalTransition * scale,
            targetSpeed,
            trafficEntries,
            roadMargin: startBounds.maxLateral,
            committed: urgentManeuver,
            aggression: clamp(finite(aggression, 0.5), 0, 1),
            horizon,
            targetId,
            referenceLineAtDistance,
            kerbAllowance,
            intentType: 'DIAGNOSTIC_PROBE',
            racecraftPhase,
            weights,
            curve: null,
            startV,
            startA,
            previousPlan,
            dtSinceLastPlan,
            isDiagnostic: true
          });
          diagCand.selected = false;
          diagCand.selection.state = 'DIAGNOSTIC_PROBE';
          addVisualCandidate(diagCand);
        }
      }
    }

    for (const cand of visualCandidates) {
      if (cand !== selected) cand.selected = false;
    }

    this.lastCandidates = visualCandidates;

    // Compute pursuit tracking target point
    const pursuitDist = clamp(
      finite(trackingDistance, finite(lookAhead, 12) * 0.85),
      6.5,
      25.0
    );

    let trackingIndex = selected.points.findIndex((p) => p.forwardDistance >= pursuitDist);
    if (trackingIndex < 1) {
      trackingIndex = Math.min(selected.points.length - 1, 2);
    }
    const trackingPoint = selected.points[trackingIndex];

    let isMaterialSwitch = false;
    let isGeometrySwitch = false;
    let isSemanticSwitch = false;
    let isFlankReversal = false;
    let headingDiscontinuity = 0;
    let curvatureDiscontinuity = 0;
    let rmsNearHorizonDivergence = 0;
    let maxNearHorizonDivergence = 0;
    let trackingPointDisplacement = 0;

    if (previousPlan) {
      rmsNearHorizonDivergence = selected.rmsNearHorizonDivergence ?? 0;
      maxNearHorizonDivergence = selected.maxNearHorizonDivergence ?? 0;
      headingDiscontinuity = selected.maxNearHorizonHeadingDiscontinuity ?? 0;
      curvatureDiscontinuity = selected.maxNearHorizonCurvatureDiscontinuity ?? 0;

      if (previousPlan.trackingPoint) {
        trackingPointDisplacement = Math.abs(trackingPoint.lateral - previousPlan.trackingPoint.lateral);
      }

      const prevFamily = previousPlan.maneuver?.family;
      isSemanticSwitch = Boolean(prevFamily && selected.maneuver?.family && prevFamily !== selected.maneuver.family);

      const prevTarget = previousPlan.selectedOffset ?? previousPlan.terminalLateral;
      if (Number.isFinite(prevTarget)) {
        const prevDir = Math.sign(prevTarget - currentLateral);
        const newDir = Math.sign(selected.terminalLateral - currentLateral);
        isFlankReversal = prevDir !== 0 && newDir !== 0 && prevDir !== newDir && Math.abs(selected.terminalLateral - prevTarget) > 0.8;
      }

      isGeometrySwitch = maxNearHorizonDivergence > 0.35
        || rmsNearHorizonDivergence > 0.22
        || headingDiscontinuity > 0.08
        || curvatureDiscontinuity > 0.012
        || trackingPointDisplacement > 0.45;

      isMaterialSwitch = isGeometrySwitch || isSemanticSwitch || isFlankReversal;
    }

    return {
      points: selected.points,
      trackingPoint,
      trackingIndex,
      selectedOffset: selected.terminalLateral,
      requestedOffset: intendedOffset,
      transitionTimeS: selected.transitionTime,
      score: selected.score,
      candidateCount: candidateTrajectories.length,
      controlCandidates: candidateTrajectories,
      diagnosticCandidates,
      rawCandidateCount: candidatePool.length,
      uniqueOffsetCount: standardOffsets.length,
      evaluatedTrajectoryCount: candidateTrajectories.length,
      safeTrajectoryCount: safeCandidates.length,
      collisionFree: selected.collisionFree,
      roadLegal: selected.roadLegal,
      dynamicallyFeasible: selected.dynamicallyFeasible,
      constraints: selected.constraints,
      selection: selected.selection,
      minimumClearanceM: selected.minimumClearanceM,
      futureMinimumClearanceM: selected.futureMinimumClearanceM,
      maxCurvaturePerM: selected.maxCurvaturePerM,
      maxLateralAccelerationMps2: selected.maxLateralAccelerationMps2,
      intentType: selected.intentType,
      maneuver: selected.maneuver,
      isMaterialSwitch,
      isGeometrySwitch,
      isSemanticSwitch,
      isFlankReversal,
      headingDiscontinuity,
      curvatureDiscontinuity,
      rmsNearHorizonDivergence,
      maxNearHorizonDivergence,
      meanNearHorizonDivergence: selected.meanNearHorizonDivergence ?? 0,
      trackingPointDisplacement,
      candidates: visualCandidates,
      committed,
      recovering: Boolean(recovering),
      curve: selected.curve,
      poly: selected.poly,
      startV,
      startA
    };
  }
}
