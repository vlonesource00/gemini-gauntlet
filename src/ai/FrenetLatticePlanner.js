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
    this.c5 = (12 * dq - 6 * (this.v1 + this.v0) * this.T - (this.a1 - this.a0) * T2) / (2 * T5);
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
 * Extract collision half-extents from vehicle instance or specifications.
 */
const getVehicleBoundingExtents = (v) => {
  const halfLength = finite(
    v?.collisionHalfLength,
    finite(v?.spec?.wheelBase, 2.7) * 0.5 + finite(v?.spec?.collision?.overhangM, 0.56)
  );
  const halfWidth = finite(
    v?.collisionHalfWidth,
    finite(v?.spec?.trackWidth, 1.6) * 0.5 + finite(v?.spec?.collision?.bodyMarginM, 0.15)
  );
  return { halfLength: Math.max(1.5, halfLength), halfWidth: Math.max(0.75, halfWidth) };
};

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
    kerbAllowance = 0,
    intentType = 'STANDARD',
    racecraftPhase = 'NONE',
    weights = {},
    curve = null,
    startV = 0,
    startA = 0,
    previousPlan = null,
    dtSinceLastPlan = 0.04
  }) {
    const points = [];
    const smoothQ = new Float64Array(this.pointCount);
    const startSpeed = Math.max(0, finite(vehicle?.speed, 0));
    const acceleration = clamp((finite(targetSpeed, startSpeed) - startSpeed) * 0.42, -7.0, 5.0);

    const egoExtents = getVehicleBoundingExtents(vehicle);

    let roadViolation = 0;
    let edgeRisk = 0;
    let collisionRisk = 0;
    let predictedCollisions = 0;
    let firstConflictPoint = null;
    let firstConflictStation = null;
    let firstConflictTime = null;
    let minimumClearance = 99;
    let futureMinimumClearance = 99;
    let maxLateralAcceleration = 0;
    let maxCurvature = 0;
    let speedSum = 0;

    const effectiveRoadMargin = roadMargin + kerbAllowance;

    // Check if candidate follows a dynamic reference racing line
    const hasReference = typeof referenceLineAtDistance === 'function';
    const isPaceLine = intentType === 'PRIMARY_INTENT' || intentType === 'RACING_LINE' || intentType === 'PACE';

    // Initial lateral offset relative to nominal reference line
    const startDistance = finite(vehicle?.distance, 0);
    const startRef = track?.atDistance ? track.atDistance(startDistance) : { s: startDistance };
    const startSurfaceLimit = Math.min(
      effectiveRoadMargin,
      finite(track?.planningLateralLimit?.(startRef.s, terminalLateral), effectiveRoadMargin) + kerbAllowance
    );

    const refLine0 = hasReference
      ? clamp(finite(referenceLineAtDistance(startRef.s), 0), -startSurfaceLimit, startSurfaceLimit)
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
      const predictedSpeed = clamp(startSpeed + acceleration * time, 0, 95);
      speedSum += predictedSpeed;

      const forwardDistance = Math.max(0, startSpeed * time + 0.5 * acceleration * time * time);
      const blend = minimumJerk(time / Math.max(0.2, transitionTime));

      const currentDistance = startDistance + forwardDistance;
      const reference = track?.atDistance
        ? track.atDistance(currentDistance)
        : { s: currentDistance, x: 0, y: 0, z: 0, curvature: 0 };

      const refLineVal = hasReference
        ? finite(referenceLineAtDistance(reference.s), 0)
        : 0;

      const localLimit = track?.planningLateralLimit
        ? track.planningLateralLimit(reference.s, terminalLateral)
        : effectiveRoadMargin;
      const surfaceLimit = Math.min(effectiveRoadMargin, finite(localLimit, effectiveRoadMargin) + kerbAllowance);

      // Combine C2 quintic transitions with (1-x)^2 parabolic line rejoin
      let unclampedLateral;
      if (curve) {
        unclampedLateral = curve.eval(time);
      } else if (isPaceLine && hasReference) {
        const rejoinW = shift !== 0 ? parabolicRejoin(shift, forwardDistance, rejoinSpan) : 0;
        unclampedLateral = clamp(refLineVal, -surfaceLimit, surfaceLimit) + rejoinW;
      } else if (poly) {
        unclampedLateral = poly.eval(time);
      } else {
        const targetQ = clamp(terminalLateral, -surfaceLimit, surfaceLimit);
        unclampedLateral = startLateral + (targetQ - startLateral) * blend;
      }

      smoothQ[index] = unclampedLateral;
      const lateral = clamp(unclampedLateral, -surfaceLimit, surfaceLimit);

      let world;
      if (track?.lateralPoint) {
        world = track.lateralPoint(reference, lateral, 0.08);
      } else {
        world = {
          x: reference.x ?? 0,
          y: (reference.y ?? 0) + 0.08,
          z: reference.z ?? 0
        };
      }

      if (Math.abs(unclampedLateral) > surfaceLimit || Math.abs(terminalLateral) > surfaceLimit) {
        const excess = Math.max(Math.abs(unclampedLateral) - surfaceLimit, Math.abs(terminalLateral) - surfaceLimit);
        roadViolation += excess + 1.0;
      }

      // Edge risk builds when close to the track boundary
      const edgeBuffer = Math.max(0.12, 0.40 - aggression * 0.22 - (kerbAllowance > 0 ? 0.12 : 0));
      edgeRisk += Math.max(0, Math.abs(lateral) - (surfaceLimit - edgeBuffer)) ** 2;

      // Spatial bounding-capsule collision checking against traffic entries
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

        const lateralGap = Math.abs(lateral - opponentLateral);

        // Physical spatial geometry
        const physicalHalfWidth = egoExtents.halfWidth + opponentExtents.halfWidth; // ~2.05m
        const physicalHalfLength = egoExtents.halfLength + opponentExtents.halfLength; // ~4.65m

        const longitudinalClearance = Math.abs(longitudinalGap) - (physicalHalfLength + 0.35);
        const lateralClearance = lateralGap - physicalHalfWidth; // > 0 means clear daylight between bodies!
        const combinedClearance = Math.max(longitudinalClearance, lateralClearance);

        minimumClearance = Math.min(minimumClearance, combinedClearance);
        if (time >= 0.4) {
          futureMinimumClearance = Math.min(futureMinimumClearance, combinedClearance);
        }

        // Check if candidate establishes a viable passing lane
        const terminalClearance = Math.abs(terminalLateral - opponentTarget) - physicalHalfWidth;
        const isViablePassLane = terminalClearance >= 0.15;

        // Physical collision occurs when BOTH longitudinal AND lateral footprints overlap
        const isPhysicalOverlap = longitudinalClearance < 0 && lateralClearance < 0;
        // Tiny numerical tolerance (0.06m) for simulation discretization noise; anything deeper is a severe collision event
        const isIncidentalNumericalContact = isPhysicalOverlap
          && lateralClearance > -0.06
          && Math.abs(entry.relativeLongitudinalVelocity || 0) < 3.5
          && Math.abs(entry.relativeLateralVelocity || 0) < 1.2;

        if (isPhysicalOverlap && !isIncidentalNumericalContact) {
          // Intermediate physical overlap is ALWAYS a collision; terminal pass lane viability NEVER excuses intermediate collision!
          predictedCollisions += 1;
          collisionRisk += 35000 + (-longitudinalClearance + 0.25) * (-lateralClearance + 0.25) * 3500;
          if (!firstConflictPoint) {
            firstConflictPoint = {
              x: finite(world.x),
              y: finite(world.y),
              z: finite(world.z)
            };
            firstConflictStation = finite(reference.s);
            firstConflictTime = finite(time);
          }
        } else if (isIncidentalNumericalContact) {
          // Soft numerical contact cost
          collisionRisk += 75.0 + (-lateralClearance) * 250.0;
        } else {
          // Smooth proximity penalty for tight corridors
          const distAbs = Math.abs(longitudinalGap);
          const proximityHorizon = Math.max(4.5, 8.5 - aggression * 2.0);
          if (distAbs < proximityHorizon && lateralClearance < 0.85) {
            const timeDiscount = Math.max(0.2, 1.0 - time / Math.max(0.5, horizon));
            collisionRisk += (proximityHorizon - distAbs) * Math.max(0, 0.85 - lateralClearance) * 16 * timeDiscount;
          }
        }
      }

      points.push({
        x: finite(world.x),
        y: finite(world.y),
        z: finite(world.z),
        s: finite(reference.s),
        lateral: finite(lateral),
        time: finite(time),
        speed: finite(predictedSpeed),
        predictedSpeed: finite(predictedSpeed),
        forwardDistance: finite(forwardDistance),
        curvature: 0
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

    // Lateral dynamics budget calibrated by vehicle class capability and ground-effect aero
    const vClass = vehicle?.classKey || 'prototype';
    const baseLatCap = vClass === 'prototype' ? 22.0 : (vClass === 'gt' ? 13.5 : 10.5);
    const availableLatG = (baseLatCap + aggression * 4.5) * (1.0 + kerbAllowance * 0.12);
    const accelerationExcess = Math.max(0, maxLateralAcceleration - availableLatG);
    const clampedExcess = Math.min(10.0, accelerationExcess);

    // Multi-objective cost weighting
    const wProg = weights.prog ?? (0.8 + aggression * 0.4);
    const wColl = weights.coll ?? 1.0;
    const wEdge = weights.edge ?? (42.0 * (1.0 - aggression * 0.45));
    const wAccel = weights.accel ?? 3.5;
    const wJerk = weights.jerk ?? (committed ? 0.7 : 1.3);
    const wIntent = weights.intent ?? (committed ? 190.0 : 45.0);

    const lateralDelta = Math.abs(terminalLateral - startLateral);
    const intentError = Math.abs(terminalLateral - desiredOffset);
    const avgSpeed = speedSum / this.pointCount;

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
    const costAccel = clampedExcess * clampedExcess * wAccel;
    const costJerk = (lateralDelta * 0.20 + (committed ? transitionTime * 1.0 : transitionTime * 0.22)) * wJerk;
    const costIntent = intentError * intentError * wIntent;

    // Full-path trajectory switching cost against previously active plan
    let costSwitching = 0;
    if (previousPlan && Array.isArray(previousPlan.points) && previousPlan.points.length > 0) {
      let pathDiffSum = 0;
      const dt = horizon / Math.max(1, this.pointCount - 1);
      const prevPoints = previousPlan.points;
      const nPrev = prevPoints.length;
      for (let i = 0; i < points.length; i++) {
        const tCheck = points[i].time + dtSinceLastPlan;
        let pLat = prevPoints[nPrev - 1].lateral;
        if (tCheck <= prevPoints[0].time) {
          pLat = prevPoints[0].lateral;
        } else if (tCheck < prevPoints[nPrev - 1].time) {
          for (let j = 0; j < nPrev - 1; j++) {
            if (tCheck >= prevPoints[j].time && tCheck <= prevPoints[j + 1].time) {
              const span = Math.max(1e-4, prevPoints[j + 1].time - prevPoints[j].time);
              const frac = (tCheck - prevPoints[j].time) / span;
              pLat = prevPoints[j].lateral + frac * (prevPoints[j + 1].lateral - prevPoints[j].lateral);
              break;
            }
          }
        }
        const dLat = points[i].lateral - pLat;
        pathDiffSum += dLat * dLat * dt;
      }
      costSwitching = pathDiffSum * 28.0;

      const prevTargetLat = previousPlan.selectedOffset ?? previousPlan.terminalLateral;
      if (Number.isFinite(prevTargetLat)) {
        const prevDir = Math.sign(prevTargetLat - startLateral);
        const newDir = Math.sign(terminalLateral - startLateral);
        if (prevDir !== 0 && newDir !== 0 && prevDir !== newDir && Math.abs(terminalLateral - prevTargetLat) > 0.8) {
          costSwitching += 35.0;
        }
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

    const collisionFree = predictedCollisions === 0;
    const roadLegal = roadViolation < 1e-4;
    let initialRejectionReason = 'VIABLE_ALTERNATIVE';
    if (!roadLegal) {
      initialRejectionReason = 'ROAD_LIMIT';
    } else if (!collisionFree) {
      initialRejectionReason = 'COLLISION';
    } else if (clampedExcess > 0.05) {
      initialRejectionReason = 'DYNAMIC_LIMIT';
    }

    const candidateId = `traj_${terminalLateral.toFixed(2)}_${transitionTime.toFixed(2)}_${intentType}`;

    return {
      id: candidateId,
      points,
      score: totalScore,
      terminalLateral,
      transitionTime,
      intentType,
      collisionFree,
      roadLegal,
      minimumClearanceM: minimumClearance,
      futureMinimumClearanceM: futureMinimumClearance,
      maxCurvaturePerM: maxCurvature,
      maxLateralAccelerationMps2: maxLateralAcceleration,
      rejectionReason: initialRejectionReason,
      conflictPoint: firstConflictPoint,
      conflictStation: firstConflictStation,
      conflictTime: firstConflictTime,
      costBreakdown: {
        roadViolation: costRoadViolation,
        collisionRisk: costCollision,
        edgeRisk: costEdge,
        accelerationExcess: costAccel,
        jerk: costJerk,
        intent: costIntent,
        switching: costSwitching,
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
    weights = {},
    previousPlan = null,
    dtSinceLastPlan = 0.04
  }) {
    const currentLateral = finite(vehicle?.surface?.lateral, 0);
    const nominalHalfWidth = finite(track?.roadHalfWidth, 6.5);
    const maximumSurfaceMargin = Math.max(2.1, nominalHalfWidth - 1.18 + kerbAllowance);
    const margin = Math.max(1.8, finite(roadMargin, maximumSurfaceMargin));

    const minBound = Number.isFinite(dMin) ? Math.max(-margin, dMin) : -margin;
    const maxBound = Number.isFinite(dMax) ? Math.min(margin, dMax) : margin;
    const intendedOffset = clamp(finite(desiredOffset), minBound, maxBound);
    const committed = pitActive || ['SLINGSHOT', 'ATTACK', 'ATTACK_LEFT', 'ATTACK_RIGHT', 'ATTACK_INSIDE', 'ATTACK_OUTSIDE', 'DIVEBOMB', 'SWITCHBACK', 'DEFEND_LEFT', 'DEFEND_RIGHT', 'DEFEND_INSIDE', 'BREAK_TOW', 'APEX_SHIELD', 'EXIT_SQUEEZE', 'ABORT_HOLD', 'ABORT_BLEND'].includes(racecraftPhase);
    const urgentManeuver = committed || recovering || urgent;

    let startV = 0;
    let startA = 0;
    if (previousPlan && !recovering) {
      startV = clamp(finite(vehicle?.localVelocity?.x, 0), -3.0, 3.0);
      startA = clamp(finite(vehicle?.localAcceleration?.x, 0), -6.0, 6.0);
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

    const availableLatAccel = 7.5 + clamp(finite(aggression, 0.5), 0, 1) * 5.0;

    // Multi-stage trajectory families for combat maneuvers
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

    // Generate balanced left, center, right, and dense tactical candidates
    const oppositeLane = intendedOffset > 0.5 ? -Math.min(margin * 0.75, intendedOffset) : (intendedOffset < -0.5 ? Math.min(margin * 0.75, -intendedOffset) : 0);
    const rawPool = recovering
      ? [intendedOffset, currentLateral, 0]
      : [
          intendedOffset,
          ...fallbackOffsets,
          currentLateral,
          0,
          oppositeLane,
          -margin * 0.65,
          margin * 0.65
        ];

    if (!recovering) {
      const spanSteps = [-0.85, -0.65, -0.45, -0.25, 0.25, 0.45, 0.65, 0.85];
      for (const factor of spanSteps) {
        rawPool.push(factor * margin);
      }
    }

    if (!recovering && Math.abs(intendedOffset - currentLateral) > 0.4) {
      rawPool.push(
        currentLateral + 0.33 * (intendedOffset - currentLateral),
        currentLateral + 0.66 * (intendedOffset - currentLateral)
      );
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

        // If the opponent occupies or threatens the intended line, synthesize open flanking corridors
        if (Math.abs(intendedOffset - oppLat) < passGap) {
          const leftFlank = clamp(oppLat - passGap, minBound, maxBound);
          const rightFlank = clamp(oppLat + passGap, minBound, maxBound);
          const wideLeftFlank = clamp(oppLat - passGap - 0.75, minBound, maxBound);
          const wideRightFlank = clamp(oppLat + passGap + 0.75, minBound, maxBound);

          candidatePool.push(leftFlank, rightFlank, wideLeftFlank, wideRightFlank);
        }
      }
    }

    // Filter out intermediate candidates that fall inside an active opponent's occupied lateral zone
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

    const defaultScales = urgentManeuver ? [0.75, 1.0, 1.25] : [0.85, 1.0, 1.35];
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
          roadMargin: margin,
          committed: urgentManeuver,
          aggression: clamp(finite(aggression, 0.5), 0, 1),
          horizon,
          targetId,
          referenceLineAtDistance,
          kerbAllowance,
          intentType,
          racecraftPhase,
          weights,
          curve: null,
          startV,
          startA,
          previousPlan,
          dtSinceLastPlan
        }));
      }
    }

    // Evaluate tactical custom candidate offsets
    for (const entry of customOffsetEntries) {
      const scales = entry.transitionScales || defaultScales;
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
          roadMargin: margin,
          committed: urgentManeuver,
          aggression: clamp(finite(aggression, 0.5), 0, 1),
          horizon,
          targetId,
          referenceLineAtDistance,
          kerbAllowance,
          intentType: entry.intentType,
          racecraftPhase,
          weights,
          curve: entry.curve || null,
          startV,
          startA,
          previousPlan,
          dtSinceLastPlan
        }));
      }
    }

    // Loyalty hysteresis bonus for continuing the ongoing candidate
    if (previousPlan && Number.isFinite(previousPlan.selectedOffset)) {
      const prevTarget = previousPlan.selectedOffset;
      const ongoing = candidateTrajectories.find((c) =>
        Math.abs(c.terminalLateral - prevTarget) < 0.35
      );
      if (ongoing) {
        ongoing.score -= 35.0;
      }
    }

    // Sort candidates by total score
    candidateTrajectories.sort((a, b) => a.score - b.score
      || Math.abs(a.terminalLateral - intendedOffset) - Math.abs(b.terminalLateral - intendedOffset)
      || a.transitionTime - b.transitionTime);

    const safeCandidates = candidateTrajectories.filter((c) => c.collisionFree && c.roadLegal);

    // Pick best candidate, fallback to candidate with maximum future clearance
    let selected = safeCandidates[0] ?? [...candidateTrajectories].sort((a, b) =>
      b.futureMinimumClearanceM - a.futureMinimumClearanceM || a.score - b.score
    )[0];

    // Switching margin: if switching away from ongoing candidate to a divergent trajectory, require decisive improvement
    let switchOverriddenCandidate = null;
    if (previousPlan && Number.isFinite(previousPlan.selectedOffset) && safeCandidates.length > 1) {
      const prevTarget = previousPlan.selectedOffset;
      const ongoingCandidate = safeCandidates.find((c) => Math.abs(c.terminalLateral - prevTarget) < 0.35);
      if (ongoingCandidate && selected !== ongoingCandidate) {
        const isDirectionReversal = Math.sign(selected.terminalLateral - currentLateral) !== Math.sign(prevTarget - currentLateral)
          && Math.abs(selected.terminalLateral - prevTarget) > 0.8;
        const switchMargin = isDirectionReversal ? 40.0 : 20.0;
        if (selected.score > ongoingCandidate.score - switchMargin) {
          switchOverriddenCandidate = selected;
          selected = ongoingCandidate;
        }
      }
    }

    this.lastSelectedOffset = selected.terminalLateral;
    this.lastSelectedTrajectory = selected;

    // Classify rejection reasons
    selected.selected = true;
    selected.rejectionReason = 'SELECTED';

    if (switchOverriddenCandidate) {
      switchOverriddenCandidate.rejectionReason = 'SWITCH_MARGIN';
    }

    let viableSafeCount = 0;
    for (const cand of safeCandidates) {
      if (cand === selected || cand === switchOverriddenCandidate) continue;
      if (viableSafeCount < 6) {
        cand.rejectionReason = 'VIABLE_ALTERNATIVE';
        viableSafeCount += 1;
      } else {
        cand.rejectionReason = 'HIGHER_COST';
      }
    }

    // Build bounded 24-32 diagnostic candidate set for visual inspection
    const visualCandidates = [selected];
    const seenIds = new Set([selected.id]);

    const addVisualCandidate = (cand) => {
      if (!cand || seenIds.has(cand.id)) return;
      seenIds.add(cand.id);
      visualCandidates.push(cand);
    };

    // 1. Ongoing candidate
    if (previousPlan && Number.isFinite(previousPlan.selectedOffset)) {
      const prevTarget = previousPlan.selectedOffset;
      const ongoing = candidateTrajectories.find((c) => Math.abs(c.terminalLateral - prevTarget) < 0.35);
      if (ongoing) addVisualCandidate(ongoing);
    }

    // 2. Overridden candidate if any
    if (switchOverriddenCandidate) {
      addVisualCandidate(switchOverriddenCandidate);
    }

    // 3. Top safe alternatives (up to 8)
    for (const cand of safeCandidates) {
      if (visualCandidates.length >= 10) break;
      addVisualCandidate(cand);
    }

    // 4. Collision candidates (up to 6)
    const collisionCands = candidateTrajectories.filter((c) => !c.collisionFree);
    for (const cand of collisionCands) {
      if (visualCandidates.filter((c) => c.rejectionReason === 'COLLISION').length >= 6) break;
      addVisualCandidate(cand);
    }

    // 5. Road limit candidates (up to 4)
    const roadLimitCands = candidateTrajectories.filter((c) => !c.roadLegal);
    for (const cand of roadLimitCands) {
      if (visualCandidates.filter((c) => c.rejectionReason === 'ROAD_LIMIT').length >= 4) break;
      addVisualCandidate(cand);
    }

    // 6. Dynamic limit candidates (up to 4)
    const dynLimitCands = candidateTrajectories.filter((c) => c.rejectionReason === 'DYNAMIC_LIMIT');
    for (const cand of dynLimitCands) {
      if (visualCandidates.filter((c) => c.rejectionReason === 'DYNAMIC_LIMIT').length >= 4) break;
      addVisualCandidate(cand);
    }

    // 7. Fill remainder up to 28-32 candidates from remaining candidate pool
    for (const cand of candidateTrajectories) {
      if (visualCandidates.length >= 30) break;
      addVisualCandidate(cand);
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

    return {
      points: selected.points,
      trackingPoint,
      trackingIndex,
      selectedOffset: selected.terminalLateral,
      requestedOffset: intendedOffset,
      transitionTimeS: selected.transitionTime,
      score: selected.score,
      candidateCount: candidateTrajectories.length,
      rawCandidateCount: candidatePool.length,
      uniqueOffsetCount: standardOffsets.length,
      evaluatedTrajectoryCount: candidateTrajectories.length,
      safeTrajectoryCount: safeCandidates.length,
      collisionFree: selected.collisionFree,
      roadLegal: selected.roadLegal,
      minimumClearanceM: selected.minimumClearanceM,
      futureMinimumClearanceM: selected.futureMinimumClearanceM,
      maxCurvaturePerM: selected.maxCurvaturePerM,
      maxLateralAccelerationMps2: selected.maxLateralAccelerationMps2,
      intentType: selected.intentType,
      candidates: visualCandidates,
      committed,
      recovering: Boolean(recovering)
    };
  }
}
