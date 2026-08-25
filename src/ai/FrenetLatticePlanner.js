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

const worldHeading = (a, b) => Math.atan2(b.x - a.x, b.z - a.z);

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
    this.lastCandidates = [];
  }

  /**
   * Reset internal planner state.
   */
  reset() {
    this.lastSelectedOffset = null;
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
    weights = {}
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
        ? track.planningLateralLimit(reference.s, refLineVal + terminalLateral)
        : effectiveRoadMargin;
      const surfaceLimit = Math.min(effectiveRoadMargin, finite(localLimit, effectiveRoadMargin) + kerbAllowance);

      // Combine minimum-jerk intentional transition with (1-x)^2 parabolic line rejoin
      let unclampedLateral;
      if (isPaceLine && hasReference) {
        const rejoinW = shift !== 0 ? parabolicRejoin(shift, forwardDistance, rejoinSpan) : 0;
        unclampedLateral = clamp(refLineVal, -surfaceLimit, surfaceLimit) + rejoinW;
      } else if (hasReference && intentType !== 'LANE_HOLD' && intentType !== 'RECOVER') {
        // Tactical candidate relative to reference racing line
        const targetQ = clamp(refLineVal + terminalLateral, -surfaceLimit, surfaceLimit);
        unclampedLateral = startLateral + (targetQ - startLateral) * blend;
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

        // Spatial capsule collision geometry
        const longitudinalEnvelope = egoExtents.halfLength + opponentExtents.halfLength + 0.40;
        const lateralEnvelope = egoExtents.halfWidth + opponentExtents.halfWidth + 0.90;

        const longitudinalClearance = Math.abs(longitudinalGap) - longitudinalEnvelope;
        const lateralClearance = lateralGap - lateralEnvelope;
        const combinedClearance = Math.max(longitudinalClearance, lateralClearance);

        minimumClearance = Math.min(minimumClearance, combinedClearance);
        if (time >= 0.4) {
          futureMinimumClearance = Math.min(futureMinimumClearance, combinedClearance);
        }

        const isPassTarget = targetId !== null && entry.other.id === targetId;
        const initialTargetSeparation = Math.abs(startLateral - opponentStart);
        const separatingPassTrajectory = isPassTarget
          && (Math.abs(terminalLateral - opponentStart) >= 2.6
            || (Math.abs(longitudinalGap) > 2.0 && lateralGap >= initialTargetSeparation - 0.08));

        const isSlowObstaclePass = isPassTarget && entry.other.speed < 15.0 && Math.abs(terminalLateral - opponentStart) >= 2.6;

        if (longitudinalClearance < 0 && lateralClearance < 0 && !separatingPassTrajectory && !isSlowObstaclePass) {
          predictedCollisions += 1;
          collisionRisk += 25000 + (-longitudinalClearance + 0.2) * (-lateralClearance + 0.2) * 2500;
        } else if (!isSlowObstaclePass && !separatingPassTrajectory) {
          const distAbs = Math.abs(longitudinalGap);
          const proximityHorizon = Math.max(4.5, 8.5 - aggression * 2.0);
          if (distAbs < proximityHorizon && lateralClearance < 1.0) {
            const timeDiscount = Math.max(0.2, 1.0 - time / Math.max(0.5, horizon));
            collisionRisk += (proximityHorizon - distAbs) * (1.0 - lateralClearance) * 12 * timeDiscount;
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

      pathGeom(signedK, kp, curr.lateral, d1[index], d2[index], gScratch);
      curr.curvature = gScratch.kappa;
      maxCurvature = Math.max(maxCurvature, gScratch.kappa);

      const totalLatAccel = curr.predictedSpeed ** 2 * gScratch.kappa;
      maxLateralAcceleration = Math.max(maxLateralAcceleration, totalLatAccel);
    }

    // Lateral dynamics budget
    const availableLatG = (7.5 + aggression * 5.0) * (1.0 + kerbAllowance * 0.12);
    const accelerationExcess = Math.max(0, maxLateralAcceleration - availableLatG);

    // Multi-objective cost weighting
    const wProg = weights.prog ?? (0.8 + aggression * 0.4);
    const wColl = weights.coll ?? 1.0;
    const wEdge = weights.edge ?? (42.0 * (1.0 - aggression * 0.45));
    const wAccel = weights.accel ?? 9.0;
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

    const isInsideApex = (trackSignedCurv > 0.003 && terminalLateral < 0) || (trackSignedCurv < -0.003 && terminalLateral > 0);
    const kerbReward = (kerbAllowance > 0 && isInsideApex) ? (0.6 + aggression * 0.8) : 0;
    const rewardWidth = isInsideApex ? -(kerbReward + 0.5) : 0;

    const costRoadViolation = roadViolation * 1e6;
    const costCollision = collisionRisk * wColl;
    const costEdge = edgeRisk * wEdge;
    const costAccel = accelerationExcess * accelerationExcess * wAccel;
    const costJerk = (lateralDelta * 0.20 + (committed ? transitionTime * 1.0 : transitionTime * 0.22)) * wJerk;
    const costIntent = intentError * intentError * wIntent;

    const rewardProgress = -avgSpeed * wProg;
    const costHysteresis = (this.lastSelectedOffset !== null && Math.abs(terminalLateral - this.lastSelectedOffset) < 0.25) ? -22.0 : 0;

    const totalScore = costRoadViolation
      + costCollision
      + costEdge
      + costAccel
      + costJerk
      + costIntent
      + rewardProgress
      + rewardWidth
      + costHysteresis;

    return {
      points,
      score: totalScore,
      terminalLateral,
      transitionTime,
      intentType,
      collisionFree: predictedCollisions === 0,
      roadLegal: roadViolation < 1e-4,
      minimumClearanceM: minimumClearance,
      futureMinimumClearanceM: futureMinimumClearance,
      maxCurvaturePerM: maxCurvature,
      maxLateralAccelerationMps2: maxLateralAcceleration,
      costBreakdown: {
        roadViolation: costRoadViolation,
        collisionRisk: costCollision,
        edgeRisk: costEdge,
        accelerationExcess: costAccel,
        jerk: costJerk,
        intent: costIntent,
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
    tacticalCandidates = [], // Array of { offset, intentType, transitionScales }
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
    lookAhead = 12,
    trackingDistance = null,
    referenceLineAtDistance = null,
    weights = {}
  }) {
    const currentLateral = finite(vehicle?.surface?.lateral, 0);
    const nominalHalfWidth = finite(track?.roadHalfWidth, 6.5);
    const maximumSurfaceMargin = Math.max(2.1, nominalHalfWidth - 1.18 + kerbAllowance);
    const margin = Math.max(1.8, finite(roadMargin, maximumSurfaceMargin));

    const intendedOffset = clamp(finite(desiredOffset), -margin, margin);
    const committed = pitActive || ['SLINGSHOT', 'ATTACK', 'ATTACK_LEFT', 'ATTACK_RIGHT', 'ATTACK_INSIDE', 'ATTACK_OUTSIDE', 'DIVEBOMB', 'SWITCHBACK', 'SIDE_BY_SIDE', 'DEFEND_LEFT', 'DEFEND_RIGHT', 'DEFEND_INSIDE', 'BREAK_TOW', 'APEX_SHIELD', 'EXIT_SQUEEZE'].includes(racecraftPhase);
    const urgentManeuver = committed || recovering || urgent;

    // Collect lateral target offsets for lattice generation
    const customOffsetEntries = [];
    if (tacticalCandidates && tacticalCandidates.length > 0) {
      for (const tc of tacticalCandidates) {
        if (Number.isFinite(tc.offset)) {
          customOffsetEntries.push({
            offset: clamp(tc.offset, -margin, margin),
            intentType: tc.intentType || 'TACTICAL',
            transitionScales: tc.transitionScales || null
          });
        }
      }
    }

    // Generate balanced left, center, right, and evasive candidates
    const oppositeLane = intendedOffset > 0.5 ? -Math.min(margin * 0.75, intendedOffset) : (intendedOffset < -0.5 ? Math.min(margin * 0.75, -intendedOffset) : 0);
    const candidatePool = recovering
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
          const leftFlank = clamp(oppLat - passGap, -margin, margin);
          const rightFlank = clamp(oppLat + passGap, -margin, margin);
          const wideLeftFlank = clamp(oppLat - passGap - 0.75, -margin, margin);
          const wideRightFlank = clamp(oppLat + passGap + 0.75, -margin, margin);

          candidatePool.push(leftFlank, rightFlank, wideLeftFlank, wideRightFlank);
        }
      }
    }

    const standardOffsets = uniqueOffsets(candidatePool, -margin, margin);

    const lateralDelta = Math.abs(intendedOffset - currentLateral);
    const availableLatAccel = 7.5 + clamp(finite(aggression, 0.5), 0, 1) * 5.0;
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
          weights
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
          weights
        }));
      }
    }

    // Sort candidates by total score
    candidateTrajectories.sort((a, b) => a.score - b.score
      || Math.abs(a.terminalLateral - intendedOffset) - Math.abs(b.terminalLateral - intendedOffset)
      || a.transitionTime - b.transitionTime);

    const safeCandidates = candidateTrajectories.filter((c) => c.collisionFree && c.roadLegal);

    // Pick best candidate, fallback to candidate with maximum future clearance
    const selected = safeCandidates[0] ?? [...candidateTrajectories].sort((a, b) =>
      b.futureMinimumClearanceM - a.futureMinimumClearanceM || a.score - b.score
    )[0];

    this.lastSelectedOffset = selected.terminalLateral;

    // Filter diagnostic candidates for 3D visualization
    const visualCandidates = [selected];
    selected.selected = true;
    for (const cand of candidateTrajectories) {
      if (cand === selected) continue;
      cand.selected = false;
      if (!visualCandidates.some((v) => Math.abs(v.terminalLateral - cand.terminalLateral) < 0.35)) {
        visualCandidates.push(cand);
      }
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
