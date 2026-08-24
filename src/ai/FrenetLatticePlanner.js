/**
 * FrenetLatticePlanner.js
 * Multi-candidate Frenet-space trajectory generation and lattice evaluation engine.
 * Computes minimum-jerk spatial trajectories, curvature profiles, G-limits,
 * collision risks, track limit compliance, and multi-objective cost optimization.
 */

const finite = (value, fallback = 0) => (Number.isFinite(value) ? value : fallback);

const clamp = (value, min, max) => Math.max(min, Math.min(max, value));

const wrapAngle = (angle) => {
  let result = (angle + Math.PI) % (Math.PI * 2);
  if (result < 0) result += Math.PI * 2;
  return result - Math.PI;
};

/**
 * Quintic polynomial minimum-jerk lateral transition curve.
 * Ensures continuous lateral position, velocity, and acceleration at boundaries.
 */
export const minimumJerk = (value) => {
  const u = clamp(value, 0, 1);
  return u * u * u * (10 + u * (-15 + u * 6));
};

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

export class FrenetLatticePlanner {
  /**
   * @param {Object} options
   * @param {number} [options.pointCount=24] - Number of discretized trajectory points
   * @param {number} [options.horizonS=3.4] - Planning time horizon in seconds
   */
  constructor({ pointCount = 24, horizonS = 3.4 } = {}) {
    this.pointCount = Math.max(12, Math.trunc(pointCount));
    this.horizonS = Math.max(2.8, finite(horizonS, 3.4));
    this.lastSelectedOffset = null;
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
    const startSpeed = Math.max(0, finite(vehicle.speed, 0));
    const acceleration = clamp((finite(targetSpeed, startSpeed) - startSpeed) * 0.42, -7.0, 5.0);

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

    for (let index = 0; index < this.pointCount; index += 1) {
      const time = (horizon * index) / (this.pointCount - 1);
      const predictedSpeed = clamp(startSpeed + acceleration * time, 0, 95);
      speedSum += predictedSpeed;

      const forwardDistance = Math.max(0, startSpeed * time + 0.5 * acceleration * time * time);
      const blend = minimumJerk(time / Math.max(0.2, transitionTime));

      const currentDistance = finite(vehicle.distance, 0) + forwardDistance;
      const reference = track?.atDistance
        ? track.atDistance(currentDistance)
        : { s: currentDistance, x: 0, y: 0, z: 0 };

      const surfaceLimit = Math.min(
        effectiveRoadMargin,
        finite(track?.planningLateralLimit?.(reference.s, terminalLateral), effectiveRoadMargin) + kerbAllowance
      );

      const clampedTerminal = clamp(terminalLateral, -surfaceLimit, surfaceLimit);
      const followsReference = typeof referenceLineAtDistance === 'function'
        && (intentType === 'PRIMARY_INTENT' || intentType === 'RACING_LINE')
        && Math.abs(desiredOffset) < 0.25;

      const guidedLateral = followsReference
        ? clamp(finite(referenceLineAtDistance(reference.s), clampedTerminal), -surfaceLimit, surfaceLimit)
        : clampedTerminal;

      const unclampedLateral = startLateral + (guidedLateral - startLateral) * blend;
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

      // Edge risk starts building only when exceedingly close to the actual surface boundary
      const edgeBuffer = Math.max(0.12, 0.40 - aggression * 0.22 - (kerbAllowance > 0 ? 0.12 : 0));
      edgeRisk += Math.max(0, Math.abs(lateral) - (surfaceLimit - edgeBuffer)) ** 2;

      // Traffic proximity and collision evaluation
      for (const entry of trafficEntries || []) {
        if (!entry?.other || entry.other.finished || entry.other.despawned || entry.other.trafficGhost) {
          continue;
        }

        const opponentProgress = Math.max(0, finite(entry.other.speed, 0) * time);
        const longitudinalGap = finite(entry.delta, 0) + opponentProgress - forwardDistance;
        const opponentStart = finite(entry.otherLateral, finite(entry.other.surface?.lateral, 0));
        const opponentTarget = finite(entry.otherTargetLateral, opponentStart);
        const opponentLateral = opponentStart + (opponentTarget - opponentStart) * minimumJerk(time / 1.35);

        const lateralGap = Math.abs(lateral - opponentLateral);
        const longitudinalClearance = Math.abs(longitudinalGap) - 5.2;
        const lateralClearance = lateralGap - 3.0;
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
          // Attenuate distant proximity penalty so AI does not hesitate to set up bold overtaking maneuvers
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

    // Compute curvature and lateral acceleration along trajectory
    for (let index = 1; index < points.length - 1; index += 1) {
      const prev = points[index - 1];
      const curr = points[index];
      const next = points[index + 1];

      const segment = Math.max(0.5, Math.hypot(next.x - prev.x, next.z - prev.z) * 0.5);
      const headingCurr = worldHeading(prev, curr);
      const headingNext = worldHeading(curr, next);
      const curvature = Math.abs(wrapAngle(headingNext - headingCurr)) / segment;

      curr.curvature = curvature;
      maxCurvature = Math.max(maxCurvature, curvature);
      maxLateralAcceleration = Math.max(maxLateralAcceleration, curr.predictedSpeed ** 2 * curvature);
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
    const trackPoint = track?.atDistance ? track.atDistance(vehicle.distance) : { curvature: 0 };
    const trackCurv = finite(trackPoint?.curvature, 0);
    const isInsideApex = (trackCurv > 0.003 && terminalLateral < 0) || (trackCurv < -0.003 && terminalLateral > 0);
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
        trackWidthReward: rewardWidth
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
    const committed = pitActive || ['SLINGSHOT', 'ATTACK', 'ATTACK_LEFT', 'ATTACK_RIGHT', 'ATTACK_INSIDE', 'ATTACK_OUTSIDE', 'DIVEBOMB', 'SWITCHBACK', 'DEFEND_LEFT', 'DEFEND_RIGHT', 'DEFEND_INSIDE', 'BREAK_TOW', 'APEX_SHIELD', 'EXIT_SQUEEZE'].includes(racecraftPhase);
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

    // Generate balanced left, center, right, and evasive candidates so AI can dynamically adapt if blocked
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

    // Filter diagnostic candidates for 3D visualization: keep 1 best candidate per distinct lateral corridor
    const visualCandidates = [];
    for (const cand of candidateTrajectories) {
      if (!visualCandidates.some((v) => Math.abs(v.terminalLateral - cand.terminalLateral) < 0.35)) {
        visualCandidates.push(cand);
      }
    }

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
      candidates: visualCandidates, // Clean deduplicated candidate array for 3D visualization
      committed,
      recovering: Boolean(recovering)
    };
  }
}
