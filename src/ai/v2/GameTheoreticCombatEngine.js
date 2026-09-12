/**
 * GameTheoreticCombatEngine.js (V2 Layer 2)
 * Predictive Adversarial Racecraft Engine:
 * - Dynamic tactical attack and defense state transitions with multi-agent threat assessment
 * - Defense:
 *     - Proactive inside lane claim up to 45m ahead of braking zones
 *     - FIA Single Defensive Move Rule enforcement & anti-weave feint filtering
 *     - Apex shielding (APEX_SHIELD): pins inside apex curb tight (0.35m-0.5m margin)
 *     - Exit squeeze (EXIT_SQUEEZE): smoothly drifts wide to leave exactly 1 car width at the edge
 *     - Break-tow (BREAK_TOW): stepped lateral shift destroying slipstream draft on straights
 * - Attack (Iterative Best Response):
 *     - Slingshot draft pull-out with +18 m/s closing speed floor and dynamic threshold timing
 *     - Fearless late divebomb (DIVEBOMB) with -3.5G threshold deceleration model
 *     - Diamond line switchback undercut (SWITCHBACK) countering inside-blocking defenders
 *     - Resilient side-by-side overlap holding without yielding
 * - Multi-Apex & Compound Turn Handling:
 *     - Multi-horizon curvature scanner detecting chicanes, S-bends, and double-apex complexes
 *     - Geometric compromise trajectory line preventing getting trapped on outside runoff
 * - Dynamically morphs track corridors [d_min(s), d_max(s)] and target velocity envelopes
 */

import { clamp, saturate, lerp, damp, wrap } from '../../core/math.js';

const finite = (val, fallback = 0) => (Number.isFinite(val) ? val : fallback);

function signedTrackDelta(sA, sB, trackLength) {
  if (!trackLength || trackLength <= 0) return sA - sB;
  let delta = (sA - sB) % trackLength;
  if (delta > 0.5 * trackLength) delta -= trackLength;
  if (delta < -0.5 * trackLength) delta += trackLength;
  return delta;
}

function integrateBoundedMotion(v0, targetSpeed, t, maxAccel = 4.5, maxDecel = -12.5, responseTime = 0.35) {
  if (t <= 0) return 0;
  const speedDiff = targetSpeed - v0;
  const rawAccel = speedDiff / Math.max(0.05, responseTime);
  const a = clamp(rawAccel, maxDecel, maxAccel);
  if (Math.abs(a) < 1e-4) {
    return v0 * t;
  }
  const tReach = speedDiff / a;
  if (tReach > 0 && t > tReach) {
    const sAcc = v0 * tReach + 0.5 * a * tReach * tReach;
    return sAcc + targetSpeed * (t - tReach);
  }
  return v0 * t + 0.5 * a * t * t;
}

export class GameTheoreticCombatEngine {
  /**
   * @param {Object} [options]
   * @param {number} [options.roadHalfWidth=8.2]
   * @param {number} [options.curbWidth=1.25]
   * @param {number} [options.carWidth=2.05]
   * @param {number} [options.carLength=4.65]
   */
  constructor({
    roadHalfWidth = 8.2,
    curbWidth = 1.25,
    carWidth = 2.05,
    carLength = 4.65
  } = {}) {
    this.roadHalfWidth = roadHalfWidth;
    this.curbWidth = curbWidth;
    this.carWidth = carWidth;
    this.carLength = carLength;

    // Defense Stackelberg state
    this.defenseMode = 'PACE'; // 'PACE', 'BREAK_TOW', 'LOCK_LANE', 'APEX_SHIELD', 'EXIT_SQUEEZE', 'RETURN'
    this.lockedDefensiveLane = null;
    this.defenseDirection = 0; // -1 (left), 0 (none), +1 (right)
    this.oneMoveLocked = false;
    this.defenseTimer = 0;
    this.defenseDwellTimer = 0;
    this.defenseTargetId = null;
    this.filteredAttackerLateral = 0;
    this.lastAttackerLateral = 0;
    this.feintFilterTimer = 0;
    this.threatScore = 0;

    // Attack Iterative Best Response state
    this.attackMode = 'NONE'; // 'NONE', 'DRAFT_HOLD', 'SLINGSHOT', 'DIVEBOMB', 'SWITCHBACK', 'OVERTAKE', 'SIDE_BY_SIDE'
    this.attackTargetId = null;
    this.passedTargetId = null;
    this.targetLockTimer = 0;
    this.attackTimer = 0;
    this.attackIntensity = 0;
    this.attackSide = 0; // -1 (left), 0 (center), +1 (right)
    this.attackSideLocked = false;
    this.passClearDwell = 0;
    this.switchbackStage = 'NONE'; // 'NONE', 'ENTRY_WIDE', 'APEX_CUT', 'EXIT_UNDERCUT'
    this.divebombCommitted = false;

    // Dwell timers to prevent high-frequency mode chatter
    this.commitDwellTimer = 0;
    this.abortDwellTimer = 0;
    this.stabilizeDwellTimer = 0;
    this.insideClosedFilterTimer = 0;

    // Compound corner geometry state
    this.compoundTurnDetected = false;
    this.multiApexState = {
      isCompound: false,
      isChicane: false,
      primaryCurv: 0,
      primarySign: 1,
      primaryDist: 999,
      secondaryCurv: 0,
      secondarySign: 1,
      secondaryDist: 999
    };

    // 8-State Pass Machine (Phase 7 & V3.2)
    // 'NONE' -> 'APPROACH' -> 'COMMITTED' -> 'OVERLAP' -> 'NOSE_AHEAD' -> 'FULL_CLEAR' -> 'RETAINING' -> 'RETAINED'
    this.passState = 'NONE';
    this.passStateTargetId = null;
    this.passStateTimer = 0;
    this.retainedTimer = 0;
    this.retainedDistance = 0;
    this.passStartDistance = 0;
    this.totalCombatTimeS = 0;
    this.episodeSequence = 0;
    this.activePassEpisode = null;
    this.completedPassEpisodes = [];
    this.passEpisode = null; // Backwards compatibility

    this.lastTacticalDiagnostics = {
      actionUtilities: [],
      selectedAction: 'HOLD_POSITION',
      secondBestAction: null,
      utilityMargin: 0
    };

    // Defensive Episode Memory (FIA single-move rule enforcement)
    this.defensiveEpisode = {
      active: false,
      threatId: null,
      moveCount: 0,
      initialLane: 0,
      lockedLane: null,
      defenseDirection: 0,
      dwellTimer: 0,
      startTime: 0,
      duration: 0
    };

    // Three-wide spatial presence
    this.threeWideActive = false;
  }

  /**
   * Reset internal combat engine state machine.
   */
  reset() {
    this.defenseMode = 'PACE';
    this.lockedDefensiveLane = null;
    this.defenseDirection = 0;
    this.oneMoveLocked = false;
    this.defenseTimer = 0;
    this.defenseDwellTimer = 0;
    this.defenseTargetId = null;
    this.filteredAttackerLateral = 0;
    this.lastAttackerLateral = 0;
    this.feintFilterTimer = 0;
    this.threatScore = 0;

    this.attackMode = 'NONE';
    this.attackTargetId = null;
    this.passedTargetId = null;
    this.targetLockTimer = 0;
    this.attackTimer = 0;
    this.attackIntensity = 0;
    this.attackSide = 0;
    this.attackSideLocked = false;
    this.passClearDwell = 0;
    this.switchbackStage = 'NONE';
    this.divebombCommitted = false;

    this.commitDwellTimer = 0;
    this.abortDwellTimer = 0;
    this.stabilizeDwellTimer = 0;
    this.insideClosedFilterTimer = 0;

    this.passState = 'NONE';
    this.passStateTargetId = null;
    this.passStateTimer = 0;
    this.retainedTimer = 0;
    this.retainedDistance = 0;
    this.passStartDistance = 0;
    this.totalCombatTimeS = 0;
    this.episodeSequence = 0;
    this.activePassEpisode = null;
    this.completedPassEpisodes = [];
    this.passEpisode = null;

    this.lastTacticalDiagnostics = {
      actionUtilities: [],
      selectedAction: 'HOLD_POSITION',
      secondBestAction: null,
      utilityMargin: 0
    };

    this.defensiveEpisode = {
      active: false,
      threatId: null,
      moveCount: 0,
      initialLane: 0,
      lockedLane: null,
      defenseDirection: 0,
      dwellTimer: 0,
      startTime: 0,
      duration: 0
    };

    this.threeWideActive = false;
    this.compoundTurnDetected = false;
    this.role = 'PACE';
    return this;
  }

  /**
   * Set configuration parameters dynamically.
   */
  setParameters({ roadHalfWidth, curbWidth, carWidth, carLength } = {}) {
    if (Number.isFinite(roadHalfWidth)) this.roadHalfWidth = roadHalfWidth;
    if (Number.isFinite(curbWidth)) this.curbWidth = curbWidth;
    if (Number.isFinite(carWidth)) this.carWidth = carWidth;
    if (Number.isFinite(carLength)) this.carLength = carLength;
  }

  /**
   * Scan multi-horizon track curvature to identify compound turns, chicanes, and apex sequences.
   * Prevents getting cast wide into outside runoff between linked apexes.
   * @private
   */
  _scanMultiApexGeometry(track, currentDist) {
    if (!track?.atDistance) {
      return {
        isCompound: false,
        isChicane: false,
        isDoubleApex: false,
        primaryCurv: 0,
        primarySign: 1,
        primaryDist: 999,
        secondaryCurv: 0,
        secondarySign: 1,
        secondaryDist: 999,
        geometricCompromiseLateral: 0
      };
    }

    const horizons = [6, 14, 24, 38, 56, 80, 110];
    const samples = horizons.map((d) => {
      const pt = track.atDistance(currentDist + d);
      const rawCurv = finite(pt?.curvature, 0);
      const sign = Math.sign(finite(pt?.turnSign, rawCurv)) || 1;
      return {
        dist: d,
        curvature: Math.abs(rawCurv),
        rawCurv,
        sign,
        s: currentDist + d
      };
    });

    // Find primary turn (highest curvature within near horizon)
    const turnCandidates = samples.filter((s) => s.curvature > 0.0035);
    if (turnCandidates.length === 0) {
      const sharpest = samples.sort((a, b) => b.curvature - a.curvature)[0];
      return {
        isCompound: false,
        isChicane: false,
        isDoubleApex: false,
        primaryCurv: sharpest.curvature,
        primarySign: sharpest.sign,
        primaryDist: sharpest.dist,
        secondaryCurv: 0,
        secondarySign: sharpest.sign,
        secondaryDist: 999,
        geometricCompromiseLateral: 0
      };
    }

    const primaryTurn = turnCandidates[0];

    // Find secondary turn (turn after primary with significant curvature)
    const secondaryCandidates = turnCandidates.filter(
      (s) => s.dist >= primaryTurn.dist + 12 && s.curvature > 0.0035
    );
    const secondaryTurn = secondaryCandidates[0] ?? null;

    const isCompound = Boolean(secondaryTurn && (secondaryTurn.dist - primaryTurn.dist) <= 65);
    const isChicane = Boolean(isCompound && primaryTurn.sign !== secondaryTurn.sign);
    const isDoubleApex = Boolean(isCompound && primaryTurn.sign === secondaryTurn.sign);

    return {
      isCompound,
      isChicane,
      isDoubleApex,
      primaryCurv: primaryTurn.curvature,
      primarySign: primaryTurn.sign,
      primaryDist: primaryTurn.dist,
      secondaryCurv: secondaryTurn?.curvature ?? 0,
      secondarySign: secondaryTurn?.sign ?? primaryTurn.sign,
      secondaryDist: secondaryTurn?.dist ?? 999
    };
  }

  /**
   * Causal expected-utility tactical action evaluator.
   * Computes U(action) = sum_i P(response_i) * U(action, response_i)
   * @private
   */
  _evaluateActionExpectedUtilities({
    passTarget,
    vDist,
    vSpeed,
    currentLat,
    targetSpeed,
    opponentLat,
    gap,
    closingSpeed,
    isApproachingCorner,
    isStraight,
    insideOffset,
    maxMargin,
    primaryInsideSign,
    multiApex,
    aggression,
    optimalLat,
    optimalTargetSpeed,
    track,
    entries = []
  }) {
    const predictions = passTarget.predictions || [];
    const carW = this.carWidth;
    const minPassWidth = carW + 0.85;
    const trackLength = finite(track?.length, 1000);

    const leftSpace = Math.max(minPassWidth, maxMargin + opponentLat);
    const rightSpace = Math.max(minPassWidth, maxMargin - opponentLat);
    const targetLeft = clamp(opponentLat - Math.max(minPassWidth, Math.min(4.5, leftSpace * 0.50)), -maxMargin, maxMargin);
    const targetRight = clamp(opponentLat + Math.max(minPassWidth, Math.min(4.5, rightSpace * 0.50)), -maxMargin, maxMargin);

    // Candidate actions
    const candidateActions = [
      {
        id: 'DRAFT_HOLD',
        targetLat: opponentLat,
        targetSpeed: optimalTargetSpeed,
        mode: 'SLINGSHOT',
        side: 0,
        notes: 'ATTACK_HOLD_TUCK_SLIPSTREAM',
        dwell: 0.25
      },
      {
        id: 'ATTACK_LEFT',
        targetLat: targetLeft,
        targetSpeed: Math.max(optimalTargetSpeed * 1.02, targetSpeed + 2.5),
        mode: 'SLINGSHOT',
        side: -1,
        notes: 'ATTACK_OVERTAKE_LEFT',
        dwell: 0.35
      },
      {
        id: 'ATTACK_RIGHT',
        targetLat: targetRight,
        targetSpeed: Math.max(optimalTargetSpeed * 1.02, targetSpeed + 2.5),
        mode: 'SLINGSHOT',
        side: 1,
        notes: 'ATTACK_OVERTAKE_RIGHT',
        dwell: 0.35
      },
      {
        id: 'INSIDE_DIVE',
        targetLat: insideOffset,
        targetSpeed: multiApex.isChicane
          ? Math.max(optimalTargetSpeed * 0.99, targetSpeed + 3.5)
          : Math.max(optimalTargetSpeed * 1.02, targetSpeed + 4.0),
        mode: 'DIVEBOMB',
        side: primaryInsideSign,
        notes: multiApex.isChicane ? 'ATTACK_CHICANE_IBR_DIVEBOMB' : 'ATTACK_FEARLESS_IBR_DIVEBOMB',
        dwell: 0.50
      },
      {
        id: 'OUTSIDE_MOMENTUM',
        targetLat: clamp(-primaryInsideSign * (maxMargin * 0.82), -maxMargin, maxMargin),
        targetSpeed: Math.min(optimalTargetSpeed * 1.04, targetSpeed + 4.0),
        mode: 'OVERTAKE',
        side: -primaryInsideSign,
        notes: 'ATTACK_OUTSIDE_MOMENTUM',
        dwell: 0.40
      },
      {
        id: 'SWITCHBACK',
        targetLat: clamp(-primaryInsideSign * (maxMargin * 0.82), -maxMargin, maxMargin),
        targetSpeed: optimalTargetSpeed * 0.97,
        mode: 'SWITCHBACK',
        side: -primaryInsideSign,
        stage: 'ENTRY_WIDE',
        notes: 'ATTACK_SWITCHBACK_WIDE_ENTRY',
        dwell: 0.50
      },
      {
        id: 'HOLD_POSITION',
        targetLat: optimalLat,
        targetSpeed: optimalTargetSpeed,
        mode: 'PACE',
        notes: 'ATTACK_PURSUIT_LINE',
        dwell: 0.20
      }
    ];

    // Evaluate utility for each action against each opponent response hypothesis
    const actionScores = candidateActions.map((act) => {
      let expectedUtility = 0.0;

      // Base physical feasibility checks
      const isRoadLegal = Math.abs(act.targetLat) <= maxMargin - 0.20;
      if (!isRoadLegal) expectedUtility -= 50.0;

      // Real corridor feasibility check (Requirement 7)
      let isCorridorFeasible = true;
      if (Math.abs(act.targetLat) + carW * 0.5 > maxMargin) {
        isCorridorFeasible = false;
      }
      if (isCorridorFeasible && gap < 25.0 && act.side !== 0 && act.side !== undefined) {
        const availableFlankSpace = act.side > 0 ? (maxMargin - opponentLat) : (opponentLat - (-maxMargin));
        if (availableFlankSpace < carW + 0.40) {
          isCorridorFeasible = false;
        }
      }
      if (isCorridorFeasible && Array.isArray(entries) && entries.length > 0) {
        for (const entry of entries) {
          if (!entry?.other || entry.other.id === passTarget.other?.id || entry.other.finished || entry.other.despawned || entry.other.trafficGhost) continue;
          if (entry.delta > 1.0 && entry.delta < 30.0) {
            const thirdLat = finite(entry.otherLateral ?? entry.other?.surface?.lateral, 0);
            if (Math.abs(thirdLat - act.targetLat) < carW + 0.55) {
              isCorridorFeasible = false;
              break;
            }
          }
        }
      }
      if (!isCorridorFeasible && act.id !== 'HOLD_POSITION' && act.id !== 'DRAFT_HOLD') {
        expectedUtility -= 80.0;
      }

      // Hysteresis bonus for current mode
      if (this.attackMode === act.mode) expectedUtility += 4.0;
      if (act.side && this.attackSideLocked && this.attackSide !== 0 && act.side === this.attackSide) {
        expectedUtility += 35.0; // Strong loyalty to locked overtake flank
      } else if (act.side && this.attackSideLocked && this.attackSide !== 0 && act.side !== this.attackSide) {
        expectedUtility -= 35.0; // Strongly disfavor swapping flanks mid-maneuver
      }

      if (predictions.length === 0) {
        // Fallback heuristic scoring if no predictions supplied
        if (isStraight && act.id === 'DRAFT_HOLD') expectedUtility += 15.0;
        if (isApproachingCorner && act.id === 'INSIDE_DIVE') expectedUtility += 12.0;
        return { action: act, expectedUtility };
      }

      for (const pred of predictions) {
        const prob = finite(pred.probability, 1.0 / predictions.length);
        let u = 0.0;

        // 1. Spatio-temporal envelope overlap penalty using bounded acceleration integration & wrapped interval geometry
        const envelopes = pred.envelopes || [];
        for (const env of envelopes) {
          const t = env.timeS;
          const deltaS = integrateBoundedMotion(vSpeed, act.targetSpeed, t);
          const egoPredS = vDist + deltaS;
          const sDelta = signedTrackDelta(egoPredS, env.centerS, trackLength);
          const halfLongSpan = 0.5 * (env.sMax - env.sMin) + 1.2;
          const sOverlap = Math.abs(sDelta) <= halfLongSpan;
          if (sOverlap) {
            const latOverlap = Math.abs(act.targetLat - env.centerQ);
            if (latOverlap < (carW + 0.55)) {
              const penetration = (carW + 0.55) - latOverlap;
              u -= penetration * 40.0;
            }
          }
        }

        // 2. Tactical hypothesis synergies & penalties
        let coveredSide = 0;
        let openSide = 0;
        if (pred.id === 'DEFEND_INSIDE') {
          coveredSide = primaryInsideSign;
          openSide = -primaryInsideSign;
        } else if (pred.id === 'DEFEND_OUTSIDE') {
          coveredSide = -primaryInsideSign;
          openSide = primaryInsideSign;
        } else if (isApproachingCorner) {
          coveredSide = (opponentLat * primaryInsideSign > 0) ? primaryInsideSign : -primaryInsideSign;
          openSide = -coveredSide;
        } else {
          coveredSide = Math.abs(opponentLat) > 1.0 ? Math.sign(opponentLat) : 0;
          openSide = -coveredSide;
        }

        if (pred.id === 'DEFEND_INSIDE') {
          if (act.id === 'INSIDE_DIVE') u -= 32.0; // Squeezed door!
          if (act.id === 'SWITCHBACK') u += 28.0;  // Defender overcommitted inside!
          if (act.id === 'OUTSIDE_MOMENTUM') u += 20.0;
          if (act.side === openSide) u += 24.0;   // Open corridor commitment!
          if (act.side === coveredSide) u -= 28.0; // Avoid attacking into covered side!
        } else if (pred.id === 'DEFEND_OUTSIDE') {
          if (act.id === 'INSIDE_DIVE') u += 32.0; // Inside apex left wide open!
          if (act.side === openSide) u += 24.0;    // Open inside flank commitment!
          if (act.side === coveredSide) u -= 28.0; // Blocked outside!
          if (act.id === 'OUTSIDE_MOMENTUM') u -= 26.0;
          if (act.id === 'SWITCHBACK') u -= 20.0;
        } else if (pred.id === 'LATE_BRAKE_OVERSHOOT') {
          if (act.id === 'SWITCHBACK') u += 28.0; // Classic undercut of overshooter!
          if (act.id === 'INSIDE_DIVE') u -= 18.0; // High risk of T-bone / apex sweep
          if (act.id === 'OUTSIDE_MOMENTUM') u += 16.0;
        } else if (pred.id === 'BRAKE_EARLY') {
          if (act.id === 'INSIDE_DIVE') u += 22.0; // Free inside lane on early braker
          if (act.id === 'ATTACK_LEFT' || act.id === 'ATTACK_RIGHT') u += 18.0;
        } else if (pred.id === 'HOLD_LINE') {
          const insideOpening = Math.abs(opponentLat - insideOffset);
          if (act.id === 'INSIDE_DIVE') {
            u += insideOpening > 1.8 ? 16.0 : -14.0;
          }
          if (act.side === openSide) u += 10.0;
          if (act.id === 'DRAFT_HOLD' && isStraight && Math.abs(currentLat - opponentLat) < 1.6) u += 14.0;
        } else if (pred.id === 'RETURN_TO_RACING_LINE') {
          if (act.id === 'SWITCHBACK') u += 12.0;
          if (act.id === 'INSIDE_DIVE' && Math.sign(insideOffset) !== Math.sign(opponentLat)) u += 14.0;
          if (act.side === openSide) u += 12.0;
          if (act.side === coveredSide) u -= 12.0;
        }

        // Straight line vs corner preferences
        if (isStraight) {
          const isAlignedForDraft = Math.abs(currentLat - opponentLat) < 1.6;
          if (act.id === 'DRAFT_HOLD') {
            if (gap > 5.0 && isAlignedForDraft && closingSpeed < 4.0) {
              u += 16.0;
            } else {
              u -= 15.0; // Don't tuck into draft pocket when already separated or much faster!
            }
          }
          if (act.id === 'ATTACK_LEFT' || act.id === 'ATTACK_RIGHT') {
            const isMyFlank = (act.id === 'ATTACK_RIGHT' && currentLat > opponentLat) ||
                              (act.id === 'ATTACK_LEFT' && currentLat < opponentLat);
            u += (gap <= 28.0 ? 16.0 : 6.0);
            if (isMyFlank) u += 14.0; // Maintain natural open flank momentum!
            if (closingSpeed > 1.5) u += 8.0;
            if (act.side === openSide) u += 16.0;
            if (act.side === coveredSide) u -= 20.0;
          }
          if (act.id === 'INSIDE_DIVE') u -= 15.0;
          if (act.id === 'SWITCHBACK') u -= 15.0;
        } else if (isApproachingCorner) {
          if (act.id === 'DRAFT_HOLD') u -= 20.0; // Never draft-hold into a braking zone!
        }

        expectedUtility += prob * u;
      }

      return { action: act, expectedUtility };
    });

    actionScores.sort((a, b) => b.expectedUtility - a.expectedUtility);

    const selected = actionScores[0] || { action: candidateActions[candidateActions.length - 1], expectedUtility: 0 };
    const secondBest = actionScores[1] || null;
    const utilityMargin = secondBest ? (selected.expectedUtility - secondBest.expectedUtility) : 0;

    this.lastTacticalDiagnostics = {
      actionUtilities: actionScores.map((s) => ({ action: s.action.id, expectedUtility: Number(s.expectedUtility.toFixed(2)) })),
      selectedAction: selected.action.id,
      secondBestAction: secondBest ? secondBest.action.id : null,
      utilityMargin: Number(utilityMargin.toFixed(2))
    };

    return actionScores;
  }

  /**
   * Main adversarial game evaluation cycle.
   * @param {Object} params
   * @param {Object} params.vehicle - Ego vehicle state
   * @param {Object} params.track - Circuit track object
   * @param {Object} params.traffic - Multi-agent traffic perception
   * @param {Object} [params.optimalProfile] - Layer 1 2D Optimal Profile Solver
   * @param {number} [params.aggression=0.85] - Aggression factor (0.5 - 1.0)
   * @param {number} [params.dt=0.016] - Simulation time step
   * @returns {Object} Tactical corridor, lateral target, desired speed, and racecraft metadata
   */
  update(params) {
    return this.evaluate(params);
  }

  evaluate({
    vehicle,
    track,
    traffic,
    optimalProfile,
    aggression = 0.85,
    dt = 0.016
  } = {}) {
    const vSpeed = finite(vehicle?.speed, 0);
    const vDist = finite(vehicle?.distance, 0);
    const currentLat = finite(vehicle?.surface?.lateral, 0);
    this.totalCombatTimeS += dt;

    const roadHalfW = finite(track?.roadHalfWidth, this.roadHalfWidth);
    const curbW = finite(track?.curbWidth, this.curbWidth);
    // Utilize full physical track width respecting vehicle half-width
    const maxMargin = Math.max(2.1, roadHalfW - 0.85 + Math.min(0.65, curbW * 0.50));

    // Lockout timer for recently passed cars
    this.targetLockTimer = Math.max(0, this.targetLockTimer - dt);
    if (this.targetLockTimer <= 0) {
      this.passedTargetId = null;
    }

    // Sample Layer 1 globally optimal baseline profile
    const vClass = vehicle?.classKey || 'gt';
    const optimalSample = optimalProfile?.sampleAtDistance?.(vDist, vClass) ?? {
      lateral: 0,
      lineLateral: 0,
      targetSpeed: Math.max(25.0, vSpeed),
      curvature: 0
    };
    const optimalLat = clamp(optimalSample.lateral, -maxMargin, maxMargin);

    // Multi-horizon curvature & compound corner scanning
    const multiApex = this._scanMultiApexGeometry(track, vDist);
    this.multiApexState = multiApex;
    this.compoundTurnDetected = multiApex.isCompound;

    const isApproachingCorner = multiApex.primaryCurv > 0.0042;
    const isStraight = multiApex.primaryCurv < 0.0028;
    const primaryInsideSign = multiApex.primarySign; // Inside lateral sign (+1 for left turn, -1 for right turn)
    const insideOffset = clamp(primaryInsideSign * (maxMargin * 0.85), -maxMargin, maxMargin);

    // Scan traffic entries (Challenger behind, Target ahead)
    const entries = traffic?.entries ?? [];

    if (this.passedTargetId && entries.some((e) => e.other?.id === this.passedTargetId && e.delta > 2.0)) {
      // Opponent repassed ego: unlock immediately for legitimate new attack episode
      this.passedTargetId = null;
      this.targetLockTimer = 0;
    }

    const challenger = (traffic?.primaryDefenseThreat && traffic.primaryDefenseThreat.delta > -45.0)
      ? traffic.primaryDefenseThreat
      : entries.find((e) => {
          if (!e?.other || e.other.finished || e.other.despawned || e.other.trafficGhost) return false;
          if (e.delta >= -0.8 || e.delta <= -45.0) return false;
          const closing = finite(e.otherForwardSpeed - traffic.egoForwardSpeed, 0);
          if (this.passedTargetId && e.other.id === this.passedTargetId) {
            if (Math.abs(e.delta) > 6.0 || closing <= 0.35) return false;
          }
          if (closing < -0.3 && Math.abs(e.delta) > 8.0) return false;
          return true;
        });

    const activeAttackEntry = ((this.activePassEpisode || this.passState !== 'NONE') && (this.attackTargetId || this.passStateTargetId))
      ? entries.find((e) => e.other?.id === (this.attackTargetId || this.passStateTargetId))
      : null;

    const requiredPassClearance = this.carLength + 1.20;

    const passTarget = (this.passState === 'RETAINING')
      ? (activeAttackEntry ?? entries.find((e) => !e?.other?.finished && !e?.other?.despawned && e.delta < -0.2 && e.delta > -25.0) ?? null)
      : (activeAttackEntry ?? (traffic?.primaryAttackTarget && traffic.primaryAttackTarget.delta < 55.0 && traffic.primaryAttackTarget.other?.id !== this.passedTargetId ? traffic.primaryAttackTarget : null) ?? entries.find((e) => {
          if (!e?.other || e.other.finished || e.other.despawned || e.other.trafficGhost) return false;
          if (e.delta <= 0.4 || e.delta >= 55.0) return false;
          if (e.other.id === this.passedTargetId) return false;
          return true;
        }));

    // =========================================================================
    // 1. EVALUATE DEFENSE THREAD (Stackelberg Leader)
    // =========================================================================
    let isDefending = false;
    let defTargetLat = optimalLat;
    let defDesiredSpeed = optimalSample.targetSpeed;
    let defNotes = 'PACE';

    if (challenger && challenger.delta > -45.0) {
      const gap = Math.abs(challenger.delta);
      const challengerSpeed = finite(challenger.other?.speed ?? challenger.otherForwardSpeed, vSpeed);
      const closingSpeed = Math.max(0, challengerSpeed - vSpeed);
      const bodyGap = Math.max(0, gap - this.carLength);
      const ttc = closingSpeed > 0.20 ? bodyGap / closingSpeed : (closingSpeed > -0.2 ? bodyGap / 0.25 : 99.0);

      const rawAttackerLat = finite(challenger.otherLateral ?? challenger.other?.surface?.lateral, currentLat);
      this.filteredAttackerLateral = damp(this.filteredAttackerLateral, rawAttackerLat, 6.5, dt);

      // Composite Stackelberg threat score T(t) in [0, 1]
      const fGap = Math.exp(-gap / 16.0);
      const fClose = saturate((closingSpeed + 0.4) / 4.8);
      const fTtc = ttc <= 4.5 ? Math.pow(1.0 - ttc / 4.5, 2) : 0;
      const fCorner = Math.exp(-multiApex.primaryDist / 45.0) * saturate(multiApex.primaryCurv / 0.003);

      const isClosingThreat = (gap < 10.0 && closingSpeed > 0.20 && vSpeed > 14.0)
        || (gap <= 24.0 && closingSpeed >= 0.40 && vSpeed > 16.0)
        || (gap <= 42.0 && closingSpeed >= 0.80 && multiApex.primaryDist < 85.0 && vSpeed > 20.0)
        || (ttc < 3.0 && closingSpeed > 0.35 && vSpeed > 16.0);

      const isAttackerRealThreat = (closingSpeed > 0.30 && vSpeed > 14.0)
        || (gap < 7.0 && closingSpeed > 0.15 && vSpeed > 12.0)
        || (ttc < 3.0 && gap < 20.0 && vSpeed > 16.0);

      this.threatScore = isAttackerRealThreat
        ? saturate(fGap * 0.30 + fClose * 0.25 + fTtc * 0.25 + fCorner * 0.15 + (isClosingThreat ? 0.30 : 0.0))
        : 0;

      if ((isClosingThreat || this.threatScore > 0.35 || this.defenseDwellTimer > 0) && isAttackerRealThreat) {
        isDefending = true;
        this.defenseTargetId = challenger.other?.id ?? null;
        this.defenseTimer += dt;
        this.defenseDwellTimer = 0.65;

        let preferredDefensiveOffset = insideOffset;
        if (isStraight) {
          const attackerOffsetSign = Math.sign(this.filteredAttackerLateral) || 1;
          preferredDefensiveOffset = Math.abs(this.filteredAttackerLateral) > 1.2
            ? clamp(this.filteredAttackerLateral * 0.75, -maxMargin * 0.75, maxMargin * 0.75)
            : clamp(insideOffset * 0.65, -maxMargin * 0.75, maxMargin * 0.75);
        }

        // Defensive Episode Memory & FIA Single Defensive Move Rule
        const challengerId = challenger.other?.id ?? 'challenger';
        if (!this.defensiveEpisode.active || this.defensiveEpisode.threatId !== challengerId) {
          this.defensiveEpisode = {
            active: true,
            threatId: challengerId,
            moveCount: 0,
            initialLane: currentLat,
            lockedLane: null,
            defenseDirection: 0,
            dwellTimer: 1.2,
            startTime: this.defenseTimer
          };
        } else {
          this.defensiveEpisode.dwellTimer = 1.2;
        }

        if (this.defensiveEpisode.moveCount === 0) {
          this.defenseDirection = Math.sign(preferredDefensiveOffset) || primaryInsideSign;
          this.lockedDefensiveLane = preferredDefensiveOffset;
          this.defensiveEpisode.lockedLane = preferredDefensiveOffset;
          this.defensiveEpisode.defenseDirection = this.defenseDirection;
          this.defensiveEpisode.moveCount = 1;
          this.oneMoveLocked = true;
        }

        const lockedSign = this.defensiveEpisode.defenseDirection || this.defenseDirection;
        const defensiveTargetLat = (lockedSign === primaryInsideSign)
          ? insideOffset
          : clamp(lockedSign * (maxMargin * 0.75), -maxMargin, maxMargin);

        // State Machine Evaluation:
        // Evaluate EXIT_SQUEEZE on corner exit BEFORE generic corner approach so it is not shadowed
        const pastCurv = Math.abs(track?.atDistance ? track.atDistance(vDist - 20)?.curvature ?? 0 : 0);
        const currCurv = Math.abs(track?.atDistance ? track.atDistance(vDist)?.curvature ?? 0 : 0);
        const isCornerExit = (!isStraight && (multiApex.primaryDist > 14.0 || multiApex.primaryCurv < 0.0035))
          || (pastCurv > 0.006 && currCurv < pastCurv * 0.75);

        if (isStraight && gap > 11.0 && closingSpeed > 0.6) {
          this.defenseMode = 'BREAK_TOW';
          defTargetLat = clamp(defensiveTargetLat * 0.70, -maxMargin * 0.75, maxMargin * 0.75);
          defNotes = 'DEFEND_BREAK_TOW';
        } else if (isCornerExit && gap < 9.0) {
          this.defenseMode = 'EXIT_SQUEEZE';
          const outsideBoundary = clamp(-primaryInsideSign * (maxMargin - this.carWidth - 0.20), -maxMargin, maxMargin);
          defTargetLat = outsideBoundary;
          defNotes = 'DEFEND_EXIT_SQUEEZE';
        } else if (isApproachingCorner || gap < 15.0 || ttc < 2.4) {
          this.defenseMode = 'APEX_SHIELD';
          if (multiApex.isChicane && multiApex.primaryDist < 12.0) {
            const secondaryInsideSign = multiApex.secondarySign;
            const transitionBlend = saturate((12.0 - multiApex.primaryDist) / 12.0);
            const secondaryInsideOffset = secondaryInsideSign * (maxMargin * 0.85);
            defTargetLat = lerp(insideOffset, secondaryInsideOffset, transitionBlend * 0.60);
            defNotes = 'DEFEND_CHICANE_MULTI_APEX_SHIELD';
          } else {
            defTargetLat = insideOffset;
            defNotes = 'DEFEND_APEX_SHIELD';
          }
        } else {
          this.defenseMode = 'LOCK_LANE';
          defTargetLat = defensiveTargetLat;
          defNotes = 'DEFEND_HOLD_LANE';
        }
      } else {
        if (this.defensiveEpisode.active) {
          this.defensiveEpisode.dwellTimer = Math.max(0, this.defensiveEpisode.dwellTimer - dt);
          if (this.defensiveEpisode.dwellTimer <= 0) {
            this.defensiveEpisode.active = false;
            this.defensiveEpisode.threatId = null;
            this.defensiveEpisode.moveCount = 0;
            this.defensiveEpisode.lockedLane = null;
          }
        }
        this.defenseDwellTimer = Math.max(0, this.defenseDwellTimer - dt);
        if (this.defenseDwellTimer <= 0) {
          this.defenseMode = 'PACE';
          this.defenseTargetId = null;
          this.oneMoveLocked = false;
          this.lockedDefensiveLane = null;
          this.defenseDirection = 0;
          this.defenseTimer = 0;
        }
      }
    } else {
      if (this.defensiveEpisode.active) {
        this.defensiveEpisode.dwellTimer = Math.max(0, this.defensiveEpisode.dwellTimer - dt);
        if (this.defensiveEpisode.dwellTimer <= 0) {
          this.defensiveEpisode.active = false;
          this.defensiveEpisode.threatId = null;
          this.defensiveEpisode.moveCount = 0;
          this.defensiveEpisode.lockedLane = null;
        }
      }
      this.defenseDwellTimer = Math.max(0, this.defenseDwellTimer - dt);
      if (this.defenseDwellTimer <= 0) {
        this.defenseMode = 'PACE';
        this.defenseTargetId = null;
        this.oneMoveLocked = false;
        this.lockedDefensiveLane = null;
        this.defenseDirection = 0;
        this.defenseTimer = 0;
        this.threatScore = 0;
      }
    }

    // =========================================================================
    // 2. EVALUATE ATTACK THREAD (Iterative Best Response & Fearless Divebomb)
    // =========================================================================
    let isAttacking = false;
    let atkTargetLat = optimalLat;
    let atkDesiredSpeed = optimalSample.targetSpeed;
    let atkNotes = 'NONE';

    if (this.commitDwellTimer > 0) this.commitDwellTimer = Math.max(0, this.commitDwellTimer - dt);
    if (this.stabilizeDwellTimer > 0) this.stabilizeDwellTimer = Math.max(0, this.stabilizeDwellTimer - dt);

    if (passTarget && passTarget.delta < 55.0) {
      isAttacking = true;
      this.attackTargetId = passTarget.other?.id ?? null;
      this.attackTimer += dt;

      const gap = passTarget.delta;
      const targetSpeed = finite(passTarget.other?.speed ?? passTarget.otherForwardSpeed, vSpeed);
      const opponentLat = finite(passTarget.otherLateral ?? passTarget.other?.surface?.lateral, 0);
      const closingSpeed = Math.max(0, vSpeed - targetSpeed);
      const isSideBySide = Math.abs(gap) < this.carLength * 1.35;

      // 8-State Pass Machine Transitions (Phase 7 & V3.2)
      if (this.passStateTargetId == null && this.attackTargetId) {
        this.passStateTargetId = this.attackTargetId;
      }
      if (this.passState === 'NONE' || (this.passStateTargetId && this.passStateTargetId !== this.attackTargetId)) {
        this.passState = 'APPROACH';
        this.passStateTargetId = this.attackTargetId;
        this.passStateTimer = 0;
        this.retainedTimer = 0;
        this.retainedDistance = 0;
        this.passStartDistance = vDist;
        this.episodeSequence += 1;
        this.activePassEpisode = {
          episodeId: `pass_${this.attackTargetId}_${this.episodeSequence}_${Math.round(this.totalCombatTimeS * 100)}`,
          id: `pass_${this.attackTargetId}_${this.episodeSequence}_${Math.round(this.totalCombatTimeS * 100)}`,
          targetId: this.attackTargetId,
          state: 'APPROACH',
          startTimeS: this.totalCombatTimeS,
          endTimeS: null,
          durationS: 0,
          startDistanceM: vDist,
          endDistanceM: null,
          retainedDistanceM: 0,
          completionReason: 'IN_PROGRESS'
        };
        this.passEpisode = this.activePassEpisode;
      } else if (this.activePassEpisode == null) {
        this.passStateTargetId = this.attackTargetId;
        this.episodeSequence += 1;
        this.activePassEpisode = {
          episodeId: `pass_${this.attackTargetId}_${this.episodeSequence}_${Math.round(this.totalCombatTimeS * 100)}`,
          id: `pass_${this.attackTargetId}_${this.episodeSequence}_${Math.round(this.totalCombatTimeS * 100)}`,
          targetId: this.attackTargetId,
          state: this.passState,
          startTimeS: this.totalCombatTimeS,
          endTimeS: null,
          durationS: 0,
          startDistanceM: vDist,
          endDistanceM: null,
          retainedDistanceM: 0,
          completionReason: 'IN_PROGRESS'
        };
        this.passEpisode = this.activePassEpisode;
      }

      this.passStateTimer += dt;
      if (this.activePassEpisode) {
        this.activePassEpisode.durationS = this.totalCombatTimeS - this.activePassEpisode.startTimeS;
        this.activePassEpisode.duration = this.activePassEpisode.durationS;
        this.activePassEpisode.state = this.passState;
        this.passEpisode = this.activePassEpisode;
      }

      if (this.passState === 'APPROACH' && (this.commitDwellTimer > 0 || ['DIVEBOMB', 'SWITCHBACK', 'SLINGSHOT', 'OVERTAKE', 'SIDE_BY_SIDE'].includes(this.attackMode))) {
        this.passState = 'COMMITTED';
        if (this.activePassEpisode) this.activePassEpisode.state = 'COMMITTED';
      }

      if ((this.passState === 'COMMITTED' || this.passState === 'APPROACH') && Math.abs(gap) < this.carLength * 1.15) {
        this.passState = 'OVERLAP';
        if (this.activePassEpisode) this.activePassEpisode.state = 'OVERLAP';
      }

      if (this.passState === 'OVERLAP' && gap < -0.30) {
        this.passState = 'NOSE_AHEAD';
        if (this.activePassEpisode) this.activePassEpisode.state = 'NOSE_AHEAD';
      }

      if ((this.passState === 'NOSE_AHEAD' || this.passState === 'OVERLAP') && gap < -requiredPassClearance) {
        this.passState = 'FULL_CLEAR';
        this.passedTargetId = this.attackTargetId;
        if (this.activePassEpisode) this.activePassEpisode.state = 'FULL_CLEAR';
      }

      if (this.passState === 'FULL_CLEAR') {
        this.passState = 'RETAINING';
        this.retainedTimer = 0;
        this.retainedDistance = 0;
        if (this.activePassEpisode) this.activePassEpisode.state = 'RETAINING';
      }

      // Inside opening width relative to inside curb apex offset
      const insideOpeningWidth = Math.abs(opponentLat - insideOffset);

      // Schmitt trigger hysteresis:
      // - To ENTER divebomb: inside corridor must be wide open (> 1.80m)
      // - To EXIT/ABORT divebomb: inside corridor must pinch below 1.25m AND stay pinched for >= 0.20s
      let isInsideOpen;
      if (this.attackMode === 'DIVEBOMB') {
        if (insideOpeningWidth < 1.25) {
          this.insideClosedFilterTimer += dt;
        } else {
          this.insideClosedFilterTimer = 0;
        }
        isInsideOpen = this.insideClosedFilterTimer < 0.20;
      } else {
        this.insideClosedFilterTimer = 0;
        isInsideOpen = insideOpeningWidth > 1.80;
      }

      // In RETAINING state: post-pass safe merge & retention check
      if (this.passState === 'RETAINING') {
        this.retainedTimer += dt;
        this.retainedDistance += vSpeed * dt;

        // Smooth post-pass merge back to optimal line (without chopping across opponent's nose)
        const mergeProgress = clamp(this.retainedTimer / 1.8, 0, 1.0);
        const passSideSign = this.attackSide !== 0 ? this.attackSide : (currentLat >= opponentLat ? 1 : -1);
        const postPassOffset = clamp(opponentLat + passSideSign * (this.carWidth + 0.8), -maxMargin, maxMargin);
        atkTargetLat = lerp(postPassOffset, optimalLat, mergeProgress);
        atkDesiredSpeed = optimalSample.targetSpeed;
        atkNotes = 'POST_PASS_SAFE_MERGE_RETAINING';

        if (gap > -this.carLength * 0.75) {
          this.passState = 'REPASSED';
          this.retainedTimer = 0;
          if (this.activePassEpisode) {
            this.activePassEpisode.state = 'REPASSED';
            this.activePassEpisode.completionReason = 'REPASSED_BY_OPPONENT';
            this.activePassEpisode.endTimeS = this.totalCombatTimeS;
            this.activePassEpisode.durationS = this.totalCombatTimeS - this.activePassEpisode.startTimeS;
            const trackLen = finite(track?.length, 1000);
            const deltaDist = signedTrackDelta(vDist, this.activePassEpisode.startDistanceM, trackLen);
            const effectiveEndDist = this.activePassEpisode.startDistanceM + (deltaDist >= 0 ? deltaDist : deltaDist + trackLen);
            this.activePassEpisode.endDistanceM = effectiveEndDist;
            this.activePassEpisode.retainedDistanceM = this.retainedDistance;
            this.completedPassEpisodes.push({ ...this.activePassEpisode });
            this.passEpisode = { ...this.activePassEpisode };
          }
          this.activePassEpisode = null;
          this.attackTargetId = null;
          this.passStateTargetId = null;
          this.passState = 'NONE';
          isAttacking = false;
        } else if (this.retainedTimer >= 1.8 || this.retainedDistance >= 45.0) {
          this.passState = 'RETAINED';
          if (this.activePassEpisode) {
            this.activePassEpisode.state = 'RETAINED';
            this.activePassEpisode.completionReason = 'RETAINED';
            this.activePassEpisode.endTimeS = this.totalCombatTimeS;
            this.activePassEpisode.durationS = this.totalCombatTimeS - this.activePassEpisode.startTimeS;
            this.activePassEpisode.duration = this.activePassEpisode.durationS;
            const trackLen = finite(track?.length, 1000);
            const deltaDist = signedTrackDelta(vDist, this.activePassEpisode.startDistanceM, trackLen);
            const effectiveEndDist = this.activePassEpisode.startDistanceM + (deltaDist >= 0 ? deltaDist : deltaDist + trackLen);
            this.activePassEpisode.endDistanceM = effectiveEndDist;
            this.activePassEpisode.retainedDistanceM = this.retainedDistance;
            this.completedPassEpisodes.push({ ...this.activePassEpisode });
            this.passEpisode = { ...this.activePassEpisode };
          }
          this.passedTargetId = this.passStateTargetId || this.attackTargetId;
          this.targetLockTimer = 16.0;
          this.activePassEpisode = null;
          this.attackMode = 'NONE';
          this.attackTargetId = null;
          this.passStateTargetId = null;
          this.passState = 'NONE';
          this.passStateTimer = 0;
          this.retainedTimer = 0;
          this.attackTimer = 0;
          this.divebombCommitted = false;
          this.switchbackStage = 'NONE';
          this.attackSideLocked = false;
          this.attackSide = 0;
          this.passClearDwell = 0;
          this.commitDwellTimer = 0;
          this.abortDwellTimer = 0;
          this.stabilizeDwellTimer = 0.35;
          isAttacking = false;
        }
      } else if (gap < -requiredPassClearance) {
        this.passClearDwell += dt;
        if (this.passClearDwell >= 0.20) {
          this.passedTargetId = this.attackTargetId;
          this.passState = 'RETAINING';
          this.retainedTimer = 0;
          this.retainedDistance = 0;
          if (this.activePassEpisode) this.activePassEpisode.state = 'RETAINING';
        }
      } else {
        this.passClearDwell = 0;
      }

      if (isAttacking) {
        if (this.attackMode === 'ABORT_HOLD') {
          this.abortDwellTimer -= dt;
          atkTargetLat = currentLat;
          atkDesiredSpeed = Math.min(optimalSample.targetSpeed, targetSpeed + 1.0);
          atkNotes = 'ATTACK_ABORT_HOLD_MOMENTUM';
          if (this.abortDwellTimer <= 0) {
            this.attackMode = 'ABORT_BLEND';
            this.abortDwellTimer = 0.25;
          }
        } else if (this.attackMode === 'ABORT_BLEND') {
          this.abortDwellTimer -= dt;
          atkTargetLat = optimalLat;
          atkDesiredSpeed = optimalSample.targetSpeed;
          atkNotes = 'ATTACK_ABORT_BLEND_LINE';
          if (this.abortDwellTimer <= 0) {
            this.attackMode = 'NONE';
            this.attackSideLocked = false;
            this.attackSide = 0;
            this.stabilizeDwellTimer = 0.25;
          }
        } else if (isSideBySide) {
          // Resilient Side-by-Side Overlap Combat (Keep assigned locked flank with guaranteed daylight)
          this.attackMode = 'SIDE_BY_SIDE';
          this.commitDwellTimer = 0.35;
          if (!this.attackSideLocked || this.attackSide === 0) {
            this.attackSide = currentLat >= opponentLat ? 1 : -1;
            this.attackSideLocked = true;
          }
          const mySide = this.attackSide;
          const minDaylight = 0.35;
          atkTargetLat = clamp(opponentLat + mySide * (this.carWidth + minDaylight), -maxMargin, maxMargin);
          atkDesiredSpeed = Math.min(optimalSample.targetSpeed * 1.04, Math.max(optimalSample.targetSpeed * 0.95, targetSpeed + 2.5));
          atkNotes = 'ATTACK_SIDE_BY_SIDE_HOLD';
        } else if (this.attackMode === 'DIVEBOMB') {
          // In committed divebomb: check if inside closed to trigger soft abort, else hold line
          if (!isInsideOpen) {
            this.attackMode = 'ABORT_HOLD';
            this.abortDwellTimer = 0.30;
            this.divebombCommitted = false;
            this.commitDwellTimer = 0;
            atkTargetLat = currentLat;
            atkDesiredSpeed = Math.min(optimalSample.targetSpeed, targetSpeed + 1.0);
            atkNotes = 'ATTACK_ABORT_HOLD_MOMENTUM';
          } else {
            this.attackIntensity = 0.96;
            this.attackSide = primaryInsideSign;
            this.attackSideLocked = true;
            atkTargetLat = insideOffset;
            atkDesiredSpeed = multiApex.isChicane
              ? Math.max(optimalSample.targetSpeed * 0.99, targetSpeed + 3.5)
              : Math.max(optimalSample.targetSpeed * 1.02, targetSpeed + 4.0);
            atkNotes = multiApex.isChicane ? 'ATTACK_CHICANE_IBR_DIVEBOMB' : 'ATTACK_FEARLESS_IBR_DIVEBOMB';
          }
        } else if (this.attackMode === 'SWITCHBACK' && this.commitDwellTimer > 0) {
          // Committed switchback dwell
          this.attackIntensity = 0.90;
          this.attackSide = -primaryInsideSign;
          this.attackSideLocked = true;
          const isAtApex = multiApex.primaryDist < 12.0;
          if (!isAtApex) {
            this.switchbackStage = 'ENTRY_WIDE';
            atkTargetLat = clamp(-primaryInsideSign * (maxMargin * 0.82), -maxMargin, maxMargin);
            atkDesiredSpeed = optimalSample.targetSpeed * 0.97;
            atkNotes = 'ATTACK_SWITCHBACK_WIDE_ENTRY';
          } else {
            this.switchbackStage = 'EXIT_UNDERCUT';
            atkTargetLat = clamp(primaryInsideSign * (maxMargin * 0.50), -maxMargin, maxMargin);
            atkDesiredSpeed = Math.max(optimalSample.targetSpeed * 1.04, targetSpeed + 3.5);
            atkNotes = 'ATTACK_SWITCHBACK_EXIT_UNDERCUT';
          }
        } else if (this.stabilizeDwellTimer > 0) {
          // Post-abort or post-pass stabilization dwell: track optimal line smoothly
          atkTargetLat = optimalLat;
          atkDesiredSpeed = optimalSample.targetSpeed;
          atkNotes = 'ATTACK_STABILIZE_LINE';
        } else {
          // Causal Expected Utility Tactical Action Selection
          const scoredActions = this._evaluateActionExpectedUtilities({
            passTarget,
            vDist,
            vSpeed,
            currentLat,
            targetSpeed,
            opponentLat,
            gap,
            closingSpeed,
            isApproachingCorner,
            isStraight,
            insideOffset,
            maxMargin,
            primaryInsideSign,
            multiApex,
            aggression,
            optimalLat,
            optimalTargetSpeed: optimalSample.targetSpeed,
            track,
            entries
          });

          const best = scoredActions[0]?.action || {
            id: 'HOLD_POSITION',
            targetLat: optimalLat,
            targetSpeed: optimalSample.targetSpeed,
            mode: 'PACE',
            notes: 'ATTACK_PURSUIT_LINE'
          };

          this.attackMode = best.mode;
          if (best.side !== undefined) {
            this.attackSide = best.side;
            this.attackSideLocked = true;
          }
          if (best.dwell) {
            this.commitDwellTimer = best.dwell;
          }
          if (best.id === 'DIVEBOMB' || best.id === 'INSIDE_DIVE') {
            this.divebombCommitted = true;
            this.attackIntensity = 0.96;
          } else if (best.id === 'SWITCHBACK') {
            this.attackIntensity = 0.90;
            this.switchbackStage = multiApex.primaryDist < 12.0 ? 'EXIT_UNDERCUT' : 'ENTRY_WIDE';
          }

          atkTargetLat = best.targetLat;
          atkDesiredSpeed = best.targetSpeed;
          atkNotes = best.notes;
        }
      }
    } else {
      this.attackMode = 'NONE';
      this.attackTargetId = null;
      this.attackTimer = 0;
      this.attackIntensity = 0;
      this.switchbackStage = 'NONE';
      this.divebombCommitted = false;
      this.attackSideLocked = false;
      this.attackSide = 0;
      this.passClearDwell = 0;
      this.commitDwellTimer = 0;
      this.abortDwellTimer = 0;
    }

    // =========================================================================
    // 3. DUAL-THREAD TACTICAL SYNTHESIS (Simultaneous Attack & Defense)
    // =========================================================================
    let tacticalRole = 'PACE';
    let targetLateral = optimalLat;
    let desiredSpeed = optimalSample.targetSpeed;
    let dMin = -maxMargin;
    let dMax = maxMargin;
    let combatNotes = 'OPTIMAL_RACING_LINE';

    // Asymmetric Three-Wide Pack Reasoning (Phase 8, 11 & V3.2)
    const leftFlankCar = entries.find((e) => Math.abs(e.delta) < this.carLength * 1.35 && e.side < -0.9 && e.side > -5.5);
    const rightFlankCar = entries.find((e) => Math.abs(e.delta) < this.carLength * 1.35 && e.side > 0.9 && e.side < 5.5);
    this.threeWideActive = Boolean(leftFlankCar && rightFlankCar);

    if (this.threeWideActive) {
      const leftLat = finite(leftFlankCar.otherLateral ?? leftFlankCar.other?.surface?.lateral, currentLat - 2.5);
      const rightLat = finite(rightFlankCar.otherLateral ?? rightFlankCar.other?.surface?.lateral, currentLat + 2.5);

      const lowerLat = Math.min(leftLat, rightLat);
      const upperLat = Math.max(leftLat, rightLat);

      const lowerInner = lowerLat + this.carWidth * 0.5;
      const upperInner = upperLat - this.carWidth * 0.5;
      const freeSpaceGap = upperInner - lowerInner;
      const qMid = 0.5 * (lowerInner + upperInner);

      const minRequiredWidth = this.carWidth + 0.50; // Required body width + safety margin

      if (freeSpaceGap < minRequiredWidth) {
        // Severe pinch: do not invent nominal lane corridor; controlled yield/braking
        tacticalRole = 'THREE_WIDE_PINCH_YIELD';
        const minFlankSpeed = Math.min(
          finite(leftFlankCar.otherForwardSpeed, vSpeed),
          finite(rightFlankCar.otherForwardSpeed, vSpeed)
        );
        desiredSpeed = Math.min(optimalSample.targetSpeed * 0.85, Math.min(vSpeed - 3.5, minFlankSpeed - 1.5));

        // Yield toward rear of leading or trailing car safely
        if (leftFlankCar.delta > rightFlankCar.delta) {
          targetLateral = clamp(rightLat, -maxMargin, maxMargin);
        } else {
          targetLateral = clamp(leftLat, -maxMargin, maxMargin);
        }
        dMin = targetLateral - 0.40;
        dMax = targetLateral + 0.40;
        combatNotes = 'COMBAT_THREE_WIDE_IMPOSSIBLE_PINCH_YIELD';
      } else {
        tacticalRole = 'THREE_WIDE_HOLD';
        dMin = clamp(lowerInner + this.carWidth * 0.5 + 0.25, -maxMargin, maxMargin);
        dMax = clamp(upperInner - this.carWidth * 0.5 - 0.25, -maxMargin, maxMargin);
        if (dMin > dMax) {
          dMin = qMid - 0.45;
          dMax = qMid + 0.45;
        }
        targetLateral = clamp(qMid, dMin, dMax);
        desiredSpeed = isApproachingCorner ? Math.min(optimalSample.targetSpeed, vSpeed * 0.96) : optimalSample.targetSpeed;
        combatNotes = 'COMBAT_THREE_WIDE_CENTER_HOLD';
      }
    } else if (isDefending && isAttacking) {
      tacticalRole = 'DUAL_COMBAT';

      if (this.attackMode === 'DIVEBOMB') {
        // Diving inside target ahead naturally closes the inside on the challenger behind!
        targetLateral = atkTargetLat;
        desiredSpeed = Math.max(atkDesiredSpeed, defDesiredSpeed + 3.0);
        dMin = targetLateral - 1.2;
        dMax = targetLateral + 1.2;
        combatNotes = 'COMBAT_DUAL_DIVE_AND_SHIELD';
      } else if (this.attackMode === 'SWITCHBACK') {
        // Carry diamond entry while maintaining high speed so car behind cannot lunge
        targetLateral = atkTargetLat;
        desiredSpeed = Math.max(atkDesiredSpeed, vSpeed + 1.5);
        dMin = targetLateral - 1.4;
        dMax = targetLateral + 1.4;
        combatNotes = 'COMBAT_DUAL_SWITCHBACK_AND_DEFEND';
      } else if (this.attackMode === 'SLINGSHOT') {
        // Slingshot forward while breaking tow for the car behind
        targetLateral = atkTargetLat;
        desiredSpeed = Math.max(atkDesiredSpeed, defDesiredSpeed + 4.0);
        dMin = targetLateral - 1.6;
        dMax = targetLateral + 1.6;
        combatNotes = 'COMBAT_DUAL_SLINGSHOT_TOW_BREAK';
      } else {
        // Side by side combat: hold assigned flank firmly
        targetLateral = atkTargetLat;
        desiredSpeed = Math.max(atkDesiredSpeed, defDesiredSpeed);
        dMin = targetLateral - 1.0;
        dMax = targetLateral + 1.0;
        combatNotes = 'COMBAT_DUAL_TACTICAL_HOLD';
      }
    } else if (isDefending) {
      tacticalRole = 'DEFEND';
      targetLateral = defTargetLat;
      desiredSpeed = defDesiredSpeed;
      dMin = (this.defenseMode === 'APEX_SHIELD') ? targetLateral - 0.45 : targetLateral - 1.4;
      dMax = (this.defenseMode === 'APEX_SHIELD') ? targetLateral + 0.45 : targetLateral + 1.4;
      combatNotes = defNotes;
    } else if (isAttacking) {
      tacticalRole = 'ATTACK';
      targetLateral = atkTargetLat;
      desiredSpeed = atkDesiredSpeed;
      dMin = -maxMargin;
      dMax = maxMargin;
      combatNotes = atkNotes;
    }

    // Ensure target lateral never exceeds physical track limits or curb boundary
    targetLateral = clamp(targetLateral, -maxMargin, maxMargin);
    dMin = clamp(Math.min(dMin, targetLateral), -maxMargin, maxMargin);
    dMax = clamp(Math.max(dMax, targetLateral), -maxMargin, maxMargin);
    this.role = tacticalRole;

    return {
      role: tacticalRole,
      defenseMode: this.defenseMode,
      attackMode: this.attackMode,
      attackTargetId: this.attackTargetId,
      attackSide: this.attackSide,
      attackSideLocked: this.attackSideLocked,
      passedTargetId: this.passedTargetId,
      targetLateral,
      desiredSpeed,
      dMin,
      dMax,
      notes: combatNotes,
      multiApex: {
        isCompound: multiApex.isCompound,
        isChicane: multiApex.isChicane,
        isDoubleApex: multiApex.isDoubleApex,
        primaryCurv: multiApex.primaryCurv,
        secondaryCurv: multiApex.secondaryCurv
      },
      threatScore: this.threatScore,
      attackIntensity: this.attackIntensity,
      commitDwellRemaining: Math.max(0, this.commitDwellTimer),
      abortDwellRemaining: Math.max(0, this.abortDwellTimer),
      tacticalPhase: (this.threeWideActive ? 'THREE_WIDE' : (isDefending ? this.defenseMode : (isAttacking ? this.attackMode : 'PACE'))),
      actionUtilities: this.lastTacticalDiagnostics.actionUtilities,
      selectedAction: this.lastTacticalDiagnostics.selectedAction,
      secondBestAction: this.lastTacticalDiagnostics.secondBestAction,
      utilityMargin: this.lastTacticalDiagnostics.utilityMargin,
      passState: this.passState,
      passStateTargetId: this.passStateTargetId,
      activePassEpisode: this.activePassEpisode ? { ...this.activePassEpisode } : null,
      completedPassEpisodes: this.completedPassEpisodes.map((ep) => ({ ...ep })),
      passEpisode: this.activePassEpisode ? { ...this.activePassEpisode } : (this.passEpisode ? { ...this.passEpisode } : null),
      retainedTimer: this.retainedTimer,
      retainedDistance: this.retainedDistance,
      defensiveEpisode: {
        active: this.defensiveEpisode.active,
        threatId: this.defensiveEpisode.threatId,
        moveCount: this.defensiveEpisode.moveCount,
        lockedLane: this.defensiveEpisode.lockedLane
      },
      threeWideActive: this.threeWideActive
    };
  }
}
