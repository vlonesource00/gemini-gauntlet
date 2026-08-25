/**
 * FrenetLatticePlanner.js
 * Modular Multi-Candidate Frenet Trajectory Lattice & Dynamic Line Adaptation Engine:
 * - Smooth C2 Quintic Minimum-Jerk Spatial Trajectories
 * - (1-x)^2 Parabolic Constant-Acceleration Line Rejoin (REJOIN_A = 4.0 m/s^2, ANCHOR_DEAD = 1.5m)
 * - Station-Stencil Derivative Differencing (dq/ds and d2q/ds2) & Comb-Free Path Curvature
 * - Friction-Circle Coupled Longitudinal Velocity Propagation (v_k+1 = sqrt(v_k^2 + 2*ax*ds))
 * - Dynamic Exit-Horizon Outcome Evaluation (Exit Speed & Progress Optimization)
 * - Counterfactual Action-Conditioned Opponent Response Model (P(m | X, a_ego) over {HOLD, COVER_INSIDE, COVER_OUTSIDE})
 * - Expected Collision Risk Integration (E[CollisionRisk] = sum_m P(m | a_ego) * Risk(tau, m))
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
 */
export const minimumJerk = (value) => {
  const u = clamp(value, 0, 1);
  return u * u * u * (10 + u * (-15 + u * 6));
};

/**
 * Parabolic constant-acceleration decay function: w(x) = (1 - x)^2 for x in [0, 1).
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
    finite(v?.spec?.wheelTrack, 1.62) * 0.5 + finite(v?.spec?.collision?.tireWidthM, 0.32)
  );
  return { halfLength, halfWidth };
};

export class FrenetLatticePlanner {
  constructor(options = {}) {
    this.pointCount = Math.max(16, Math.min(48, options.pointCount || 24));
    this.horizonS = clamp(finite(options.horizonS, 3.2), 1.5, 5.0);
    this.lastSelectedOffset = null;
    this.lastSelectedIntent = 'PACE';
    this.lastPlanTime = 0;
  }

  /**
   * Reset planner internal hysteresis state.
   */
  reset() {
    this.lastSelectedOffset = null;
    this.lastSelectedIntent = 'PACE';
    this.lastPlanTime = 0;
  }

  /**
   * Evaluates a single candidate trajectory with along-path velocity propagation,
   * exit-horizon outcome scoring, and counterfactual multi-modal opponent response.
   */
  generateCandidate({
    vehicle,
    track,
    startLateral,
    startSpeed,
    startDistance,
    acceleration = 0,
    terminalLateral,
    transitionTime,
    horizon,
    effectiveRoadMargin,
    kerbAllowance = 0,
    aggression = 0.5,
    committed = false,
    intentType = 'LANE_HOLD',
    targetId = null,
    trafficEntries = [],
    hasReference = false,
    referenceLineAtDistance = null,
    desiredOffset = 0,
    weights = {}
  }) {
    const egoExtents = getVehicleBoundingExtents(vehicle);
    const points = [];
    const smoothQ = new Float64Array(this.pointCount);

    let roadViolation = 0;
    let edgeRisk = 0;
    let minimumClearance = 99.0;
    let futureMinimumClearance = 99.0;

    const isPaceLine = intentType === 'PACE_LINE' || (intentType === 'LANE_HOLD' && Math.abs(terminalLateral) < 0.25);

    const startRef = track?.atDistance
      ? track.atDistance(startDistance)
      : { s: startDistance, x: 0, y: 0, z: 0, curvature: 0 };

    const startLocalLimit = track?.planningLateralLimit
      ? track.planningLateralLimit(startRef.s, startLateral)
      : effectiveRoadMargin;
    const startSurfaceLimit = Math.min(
      effectiveRoadMargin,
      finite(startLocalLimit, effectiveRoadMargin) + kerbAllowance
    );

    const refLine0 = hasReference
      ? clamp(finite(referenceLineAtDistance(startRef.s), 0), -startSurfaceLimit, startSurfaceLimit)
      : 0;

    const dev0 = isPaceLine ? (startLateral - refLine0) : 0;
    const raw0 = dev0;
    const shift = Math.abs(raw0) <= ANCHOR_DEAD ? 0 : raw0 - Math.sign(raw0) * ANCHOR_DEAD;

    const vRejoin = Math.max(6.0, startSpeed);
    const rejoinL = shift !== 0 ? vRejoin * Math.sqrt((2 * Math.abs(shift)) / REJOIN_A) : 0;
    const rejoinSpan = Math.max(3.0, rejoinL);

    // 1. Generate spatial points in Frenet frame
    for (let index = 0; index < this.pointCount; index += 1) {
      const time = (horizon * index) / (this.pointCount - 1);
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

      let unclampedLateral;
      if (isPaceLine && hasReference) {
        const rejoinW = shift !== 0 ? parabolicRejoin(shift, forwardDistance, rejoinSpan) : 0;
        unclampedLateral = clamp(refLineVal, -surfaceLimit, surfaceLimit) + rejoinW;
      } else if (hasReference && intentType !== 'LANE_HOLD' && intentType !== 'RECOVER') {
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

      const edgeBuffer = Math.max(0.12, 0.40 - aggression * 0.22 - (kerbAllowance > 0 ? 0.12 : 0));
      edgeRisk += Math.max(0, Math.abs(lateral) - (surfaceLimit - edgeBuffer)) ** 2;

      points.push({
        x: finite(world.x),
        y: finite(world.y),
        z: finite(world.z),
        s: finite(reference.s),
        lateral: finite(lateral),
        time: finite(time),
        speed: finite(startSpeed),
        predictedSpeed: finite(startSpeed),
        forwardDistance: finite(forwardDistance),
        curvature: 0
      });
    }

    // 2. Compute station-stencil derivatives (dq/ds and d2q/ds2) for smooth curvature
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
    let maxCurvature = 0;
    let maxLateralAcceleration = 0;

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
    }

    // 3. True Along-Path Longitudinal Speed Propagation under 2D Friction Circle
    const availableLatG = (7.5 + aggression * 5.0) * (1.0 + kerbAllowance * 0.12);
    const aLongMax = 9.5;
    let vSim = Math.max(4.0, startSpeed);
    let exitIndex = S - 1;
    let foundApex = false;

    points[0].predictedSpeed = vSim;
    for (let i = 0; i < S - 1; i += 1) {
      const p0 = points[i];
      const p1 = points[i + 1];
      const ds = Math.max(0.1, p1.forwardDistance - p0.forwardDistance);
      const kappa = Math.max(0, p0.curvature);

      if (kappa > 0.012) foundApex = true;
      if (foundApex && kappa < 0.0035 && exitIndex === S - 1) {
        exitIndex = i;
      }

      const latAccel = vSim * vSim * kappa;
      maxLateralAcceleration = Math.max(maxLateralAcceleration, latAccel);

      const latUtil = Math.min(0.96, latAccel / Math.max(1.0, availableLatG));
      const longBudget = aLongMax * Math.sqrt(Math.max(0.01, 1.0 - latUtil * latUtil));

      // v_k+1 = sqrt(v_k^2 + 2 * a_x * ds)
      const vNextSq = vSim * vSim + 2.0 * longBudget * ds;
      vSim = Math.min(92.0, Math.sqrt(Math.max(0, vNextSq)));
      p1.predictedSpeed = vSim;
    }

    const exitPoint = points[exitIndex];
    const actualExitSpeed = exitPoint.predictedSpeed;
    const actualExitDistance = exitPoint.forwardDistance;
    const exitSpeedGain = actualExitSpeed - startSpeed;

    // 4. Counterfactual Action-Conditioned Defender Response Model (P(m | X, a_ego))
    const trackPoint = track?.atDistance ? track.atDistance(vehicle?.distance ?? 0) : { curvature: 0, turnSign: 0 };
    const trackCurvMag = finite(trackPoint?.curvature, 0);
    const trackSign = trackPoint?.turnSign !== undefined ? trackPoint.turnSign : 0;
    const trackSignedCurv = trackSign * trackCurvMag;

    let expectedCollisionRisk = 0;
    let predictedCollisions = 0;

    const opponentEntries = (trafficEntries || []).filter(
      (e) => e?.other && !e.other.finished && !e.other.despawned && !e.other.trafficGhost
    );

    for (const entry of opponentEntries) {
      const isDefender = targetId !== null && entry.other.id === targetId;
      const opponentExtents = getVehicleBoundingExtents(entry.other);
      const defStart = finite(entry.otherLateral, finite(entry.other.surface?.lateral, 0));
      const defSpeed = finite(entry.other.speed, 0);

      if (isDefender && Math.abs(entry.delta || 0) < 35.0) {
        // Discrete 3-Mode Counterfactual Hypothesis: {HOLD, COVER_INSIDE, COVER_OUTSIDE}
        const deltaEgoLat = terminalLateral - startLateral;
        const insideSign = trackSignedCurv !== 0 ? -Math.sign(trackSignedCurv) : -1;
        const isShowingInside = deltaEgoLat * insideSign > 0.35;
        const isShowingOutside = deltaEgoLat * insideSign < -0.35;

        // Action-conditioned logits: P(m | a_ego)
        let zHold = 0.8;
        let zCoverIn = 0.5;
        let zCoverOut = 0.3;

        if (isShowingInside) {
          zCoverIn = 2.2;
          zHold = 0.6;
          zCoverOut = -0.6;
        } else if (isShowingOutside) {
          zCoverOut = 1.8;
          zHold = 0.9;
          zCoverIn = -0.3;
        }

        // Softmax normalization
        const maxZ = Math.max(zHold, zCoverIn, zCoverOut);
        const expHold = Math.exp(zHold - maxZ);
        const expIn = Math.exp(zCoverIn - maxZ);
        const expOut = Math.exp(zCoverOut - maxZ);
        const sumExp = expHold + expIn + expOut;

        const pHold = expHold / sumExp;
        const pCoverIn = expIn / sumExp;
        const pCoverOut = expOut / sumExp;

        const modes = [
          { name: 'HOLD', prob: pHold, targetLat: defStart },
          { name: 'COVER_INSIDE', prob: pCoverIn, targetLat: clamp(defStart + 2.2 * insideSign, -effectiveRoadMargin, effectiveRoadMargin) },
          { name: 'COVER_OUTSIDE', prob: pCoverOut, targetLat: clamp(defStart - 2.2 * insideSign, -effectiveRoadMargin, effectiveRoadMargin) }
        ];

        // E[Risk] = sum_m P(m | a_ego) * Risk(tau, m)
        for (const mode of modes) {
          let modeRisk = 0;
          for (let k = 0; k < this.pointCount; k += 1) {
            const pt = points[k];
            const oppDist = finite(entry.delta, 0) + defSpeed * pt.time;
            const longGap = oppDist - pt.forwardDistance;
            const oppLat = defStart + (mode.targetLat - defStart) * minimumJerk(pt.time / 1.25);
            const latGap = Math.abs(pt.lateral - oppLat);

            const longClear = Math.abs(longGap) - (egoExtents.halfLength + opponentExtents.halfLength + 0.4);
            const latClear = latGap - (egoExtents.halfWidth + opponentExtents.halfWidth + 0.85);

            const combinedClearance = Math.max(longClear, latClear);
            minimumClearance = Math.min(minimumClearance, combinedClearance);
            if (pt.time >= 0.4) {
              futureMinimumClearance = Math.min(futureMinimumClearance, combinedClearance);
            }

            if (longClear < 0 && latClear < 0) {
              modeRisk += 25000 + (-longClear + 0.2) * (-latClear + 0.2) * 2500;
              predictedCollisions += 1;
            } else {
              const distAbs = Math.abs(longGap);
              const prox = Math.max(4.5, 8.5 - aggression * 2.0);
              if (distAbs < prox && latClear < 1.0) {
                const timeDisc = Math.max(0.2, 1.0 - pt.time / Math.max(0.5, horizon));
                modeRisk += (prox - distAbs) * (1.0 - latClear) * 12 * timeDisc;
              }
            }
          }
          expectedCollisionRisk += mode.prob * modeRisk;
        }
      } else {
        // Standard deterministic collision check for other grid traffic
        for (let k = 0; k < this.pointCount; k += 1) {
          const pt = points[k];
          const oppProgress = Math.max(0, finite(entry.other.speed, 0) * pt.time);
          const longGap = finite(entry.delta, 0) + oppProgress - pt.forwardDistance;
          const oppStart = finite(entry.otherLateral, finite(entry.other.surface?.lateral, 0));
          const oppTarget = finite(entry.otherTargetLateral, oppStart);
          const oppLat = oppStart + (oppTarget - oppStart) * minimumJerk(pt.time / 1.35);
          const latGap = Math.abs(pt.lateral - oppLat);

          const longClear = Math.abs(longGap) - (egoExtents.halfLength + opponentExtents.halfLength + 0.4);
          const latClear = latGap - (egoExtents.halfWidth + opponentExtents.halfWidth + 0.85);
          const combinedClear = Math.max(longClear, latClear);

          minimumClearance = Math.min(minimumClearance, combinedClear);
          if (pt.time >= 0.4) futureMinimumClearance = Math.min(futureMinimumClearance, combinedClear);

          if (longClear < 0 && latClear < 0) {
            predictedCollisions += 1;
            expectedCollisionRisk += 25000 + (-longClear + 0.2) * (-latClear + 0.2) * 2500;
          }
        }
      }
    }

    // 5. Multi-Objective Cost & Race Utility Formulation
    const accelerationExcess = Math.max(0, maxLateralAcceleration - availableLatG);
    const lateralDelta = Math.abs(terminalLateral - startLateral);
    const intentError = Math.abs(terminalLateral - desiredOffset);

    const wProg = weights.prog ?? (0.8 + aggression * 0.4);
    const wColl = weights.coll ?? 1.0;
    const wEdge = weights.edge ?? (42.0 * (1.0 - aggression * 0.45));
    const wAccel = weights.accel ?? 9.0;
    const wJerk = weights.jerk ?? (committed ? 0.7 : 1.3);
    const wIntent = weights.intent ?? (committed ? 190.0 : 45.0);
    const wExit = weights.exit ?? (1.8 + aggression * 1.0);

    const isInsideApex = (trackSignedCurv > 0.003 && terminalLateral < 0) || (trackSignedCurv < -0.003 && terminalLateral > 0);
    const kerbReward = (kerbAllowance > 0 && isInsideApex) ? (0.6 + aggression * 0.8) : 0;
    const rewardWidth = isInsideApex ? -(kerbReward + 0.5) : 0;

    const costRoadViolation = roadViolation * 1e6;
    const costCollision = expectedCollisionRisk * wColl;
    const costEdge = edgeRisk * wEdge;
    const costAccel = accelerationExcess * accelerationExcess * wAccel;
    const costJerk = (lateralDelta * 0.20 + (committed ? transitionTime * 1.0 : transitionTime * 0.22)) * wJerk;
    const costIntent = intentError * intentError * wIntent;

    const rewardProgress = -actualExitDistance * wProg;
    const rewardExitMomentum = -exitSpeedGain * wExit;
    const costHysteresis = (this.lastSelectedOffset !== null && Math.abs(terminalLateral - this.lastSelectedOffset) < 0.25) ? -22.0 : 0;

    const totalScore = costRoadViolation
      + costCollision
      + costEdge
      + costAccel
      + costJerk
      + costIntent
      + rewardProgress
      + rewardWidth
      + rewardExitMomentum
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
      actualExitSpeedMps: actualExitSpeed,
      exitSpeedGainMps: exitSpeedGain,
      costBreakdown: {
        roadViolation: costRoadViolation,
        collisionRisk: costCollision,
        edgeRisk: costEdge,
        accelerationExcess: costAccel,
        jerk: costJerk,
        intent: costIntent,
        progressReward: rewardProgress,
        trackWidthReward: rewardWidth,
        exitMomentumReward: rewardExitMomentum,
        hysteresisBonus: costHysteresis
      }
    };
  }

  /**
   * Plan optimal Frenet trajectory from candidate lattice.
   */
  plan({
    vehicle,
    track,
    desiredOffset = 0,
    fallbackOffsets = [],
    tacticalCandidates = [],
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

    const baselineOffsets = [intendedOffset];
    if (Math.abs(intendedOffset) > 0.4) baselineOffsets.push(0);

    const step = 0.85;
    for (let offset = -margin; offset <= margin + 1e-4; offset += step) {
      baselineOffsets.push(clamp(offset, -margin, margin));
    }
    for (const fb of fallbackOffsets) {
      if (Number.isFinite(fb)) baselineOffsets.push(clamp(fb, -margin, margin));
    }

    const uniqueLateralTargets = uniqueOffsets(baselineOffsets, -margin, margin, 0.12);
    const allTargets = [];

    for (const off of uniqueLateralTargets) {
      const isIntent = Math.abs(off - intendedOffset) < 0.15;
      const isCenter = Math.abs(off) < 0.15;
      allTargets.push({
        offset: off,
        intentType: isIntent ? (racecraftPhase !== 'NONE' ? racecraftPhase : 'PACE_LINE') : (isCenter ? 'CENTERLINE' : 'ALTERNATIVE'),
        transitionScales: null
      });
    }

    for (const custom of customOffsetEntries) {
      allTargets.push(custom);
    }

    const currentSpeed = Math.max(3.0, finite(vehicle?.speed, 10.0));
    const startDistance = finite(vehicle?.distance, 0);
    const horizonDistance = clamp(currentSpeed * this.horizonS, 18.0, 95.0);
    const nominalTransitionTime = clamp(horizonDistance / currentSpeed * (urgentManeuver ? 0.35 : 0.65), 0.35, 2.2);

    const candidates = [];
    const hasReference = typeof referenceLineAtDistance === 'function';

    for (const target of allTargets) {
      const lateralDelta = Math.abs(target.offset - currentLateral);
      const transitionTimeScales = target.transitionScales || (lateralDelta > 1.8 ? [0.65, 1.0, 1.35] : [1.0]);

      for (const scale of transitionTimeScales) {
        const transitionTime = clamp(nominalTransitionTime * scale, 0.25, 3.0);
        const cand = this.generateCandidate({
          vehicle,
          track,
          startLateral: currentLateral,
          startSpeed: currentSpeed,
          startDistance,
          acceleration: 0,
          terminalLateral: target.offset,
          transitionTime,
          horizon: this.horizonS,
          effectiveRoadMargin: margin,
          kerbAllowance,
          aggression,
          committed,
          intentType: target.intentType,
          targetId,
          trafficEntries,
          hasReference,
          referenceLineAtDistance,
          desiredOffset: intendedOffset,
          weights
        });
        candidates.push(cand);
      }
    }

    // Sort candidates by lowest total score (highest race utility)
    candidates.sort((a, b) => a.score - b.score);

    let best = candidates[0];
    for (const c of candidates) {
      if (c.roadLegal && c.collisionFree) {
        best = c;
        break;
      }
    }

    this.lastSelectedOffset = best.terminalLateral;
    this.lastSelectedIntent = best.intentType;

    const sampleIndex = Math.min(
      best.points.length - 1,
      Math.max(1, Math.round((lookAhead / Math.max(1, horizonDistance)) * (best.points.length - 1)))
    );
    const trackingPoint = best.points[sampleIndex];

    return {
      selectedOffset: best.terminalLateral,
      targetOffset: best.terminalLateral,
      intentType: best.intentType,
      score: best.score,
      collisionFree: best.collisionFree,
      roadLegal: best.roadLegal,
      minimumClearanceM: best.minimumClearanceM,
      futureMinimumClearanceM: best.futureMinimumClearanceM,
      maxCurvaturePerM: best.maxCurvaturePerM,
      maxLateralAccelerationMps2: best.maxLateralAccelerationMps2,
      actualExitSpeedMps: best.actualExitSpeedMps,
      exitSpeedGainMps: best.exitSpeedGainMps,
      trackingPoint,
      points: best.points,
      candidates
    };
  }
}

export default FrenetLatticePlanner;
