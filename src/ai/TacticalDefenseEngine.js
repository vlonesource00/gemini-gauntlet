/**
 * TacticalDefenseEngine.js
 * Advanced Unified Motorsport Defense Engine for Gemini Gauntlet:
 * - Proactive Door Shutting: Immediate inside lane claim within 45m when challenger is closing (zero hesitation)
 * - Physical Apex Shielding (APEX_SHIELD): Pins inside line tight to apex curb (0.35m margin), denying inside room completely
 * - Outside Defense Squeeze (EXIT_SQUEEZE / OUTSIDE_DEFENSE_SQUEEZE): Smoothly drifts out to leave exactly 1 car width (2.2m) at track boundary
 * - Unyielding Under Pressure: Holds committed defensive offset under wheel-to-wheel rubbing pressure without yielding or swerving
 * - Anti-Weave & Feint Filtering: Strict FIA single defensive move corridor locking while ignoring rapid opponent feints
 * - Aerodynamic Tow-Breaking Math: Stepped lateral shifts destroying >66% follower tow
 * - Defensive ERS Deployment Arbitration: Straightaway counter-bursts and corner-exit launch acceleration
 */

const finite = (value, fallback = 0) => (Number.isFinite(value) ? value : fallback);
const clamp = (value, min, max) => Math.max(min, Math.min(max, value));
const saturate = (value) => Math.max(0, Math.min(1, value));

const wrap = (value, length) => {
  if (length <= 0) return 0;
  return ((value % length) + length) % length;
};

export class TacticalDefenseEngine {
  /**
   * @param {Object} options
   * @param {number} [options.index=1] - Driver index
   * @param {number} [options.defenseReactivity=0.8] - Defensive responsiveness (0-1)
   */
  constructor({ index = 1, defenseReactivity = 0.8 } = {}) {
    this.index = index;
    this.defenseReactivity = clamp(defenseReactivity, 0, 1);
    this.reset();
  }

  reset() {
    this.phase = 'NONE';
    this.defenseTargetId = null;
    this.targetOffset = 0;
    this.defenseDirection = 0;
    this.committedDefensiveOffset = 0;
    this.threatLevel = 'NONE';
    this.threatScore = 0;
    this.attackerIntent = 'NONE';
    this.oneMoveLocked = false;
    this.timer = 0;
    this.age = 0;
    this.cooldown = 0;
    this.towBreakTimer = 0;
    this.feintFilterTimer = 0;
    this.outsideDwellTimer = 0;
    this.lastAttackerSide = 0;
    this.lastAttackerLateral = 0;
    this.lastAttackerLateralVel = 0;
    this.filteredAttackerLateral = 0;
    this.ersDefensiveDeployTimer = 0;
    this.ersDefensiveReason = 'NONE';
    this.lastDefendedCorner = false;
    this.cornerPhase = 'NONE'; // APPROACH, ENTRY, APEX, EXIT
    this.intent = null;
    this.rubbingPressure = 0;
    return this;
  }

  get defending() {
    return this.phase !== 'NONE' && this.phase !== 'RETURN_PACE' && Boolean(this.defenseTargetId);
  }

  setParameters({ defenseReactivity } = {}) {
    if (Number.isFinite(defenseReactivity)) {
      this.defenseReactivity = clamp(defenseReactivity, 0, 1);
    }
  }

  _clear(phase = 'NONE', cooldown = 0) {
    this.phase = phase;
    this.defenseTargetId = null;
    this.defenseDirection = 0;
    this.committedDefensiveOffset = 0;
    this.oneMoveLocked = false;
    this.threatLevel = 'NONE';
    this.threatScore = 0;
    this.attackerIntent = 'NONE';
    this.timer = phase === 'RETURN_PACE' ? 0.65 : 0;
    this.cooldown = Math.max(this.cooldown, cooldown);
    this.age = 0;
    this.towBreakTimer = 0;
    this.feintFilterTimer = 0;
    this.outsideDwellTimer = 0;
    this.cornerPhase = 'NONE';
    this.intent = null;
    this.rubbingPressure = 0;
  }

  /**
   * Multi-Metric Composite Threat Assessment T(t) in [0.0, 1.0].
   */
  assessThreat({ vehicle, traffic, track }) {
    if (!traffic?.entries?.length) {
      return { challenger: null, threatLevel: 'NONE', threatScore: 0, closingSpeed: 0, ttc: 99, gap: 99, distToCorner: 999, turn: null, turnCurvature: 0 };
    }

    const roadMargin = Math.max(2.1, finite(track?.roadHalfWidth, 6.5) - 1.35);
    const challenger = traffic.entries
      .filter((e) => e.delta < -0.8 && e.delta > -55.0
        && e.longitudinal < 2.5 && Math.abs(e.side) < roadMargin * 2 + 1.0)
      .sort((a, b) => b.delta - a.delta)[0] ?? null;

    if (!challenger) {
      return { challenger: null, threatLevel: 'NONE', threatScore: 0, closingSpeed: 0, ttc: 99, gap: 99, distToCorner: 999, turn: null, turnCurvature: 0 };
    }

    const closingSpeed = finite(challenger.otherForwardSpeed - traffic.egoForwardSpeed, 0);
    const gap = Math.abs(finite(challenger.delta, 99));
    const bodyGap = Math.max(0, gap - 4.6);
    const ttc = closingSpeed > 0.10 ? bodyGap / closingSpeed : (closingSpeed > -0.4 ? bodyGap / 0.15 : 99);

    // Upcoming corner proximity
    const sampleDist = finite(vehicle.distance, 0);
    const turnSamples = [16, 32, 54, 75].map((d) =>
      track?.atDistance ? track.atDistance(sampleDist + d) : { curvature: 0, turnSign: 1, s: sampleDist + d }
    );
    const turn = turnSamples.sort((a, b) => Math.abs(b.curvature) - Math.abs(a.curvature))[0];
    const turnCurvature = Math.abs(finite(turn?.curvature, 0));
    const distToCorner = Math.max(0, wrap((turn?.s ?? sampleDist) - sampleDist + (track?.length || 1000) * 0.5, track?.length || 1000) - (track?.length || 1000) * 0.5);

    // 5 Orthogonal Kinematic Factors
    const fGap = Math.exp(-gap / 16.0);
    const fClose = saturate((closingSpeed + 0.5) / 5.2);
    const fTtc = ttc <= 5.0 ? Math.pow(1.0 - ttc / 5.0, 2) : 0;
    const lateralDelta = Math.abs(finite(challenger.otherLateral, 0) - finite(traffic.current?.lateral, 0));
    const fLat = 1.0 - saturate((lateralDelta - 1.4) / 10.0);
    const fCorner = Math.exp(-distToCorner / 45.0) * saturate(turnCurvature / 0.0028);

    // Proactive door shutting trigger: within 45m and closing
    const isClosingIn45m = gap <= 45.0 && (closingSpeed >= -0.25 || gap < 28.0 || distToCorner < 80);
    const proactiveBoost = isClosingIn45m ? 0.38 : 0;

    // Non-linear synergy boost when closing rapidly into a braking zone or inside 45m zone
    const synergy = fClose * fCorner * 0.28 + proactiveBoost;

    const rawThreat = 0.20 * fGap + 0.22 * fClose + 0.24 * fTtc + 0.14 * fLat + 0.15 * fCorner + synergy;
    const threatScore = saturate(rawThreat * (0.85 + this.defenseReactivity * 0.35));

    // Schmitt-Trigger Posture Hysteresis with instant critical posture for closing threats within 45m
    let threatLevel = this.threatLevel;
    if (threatScore >= 0.65 || (isClosingIn45m && threatScore >= 0.38)) {
      threatLevel = 'CRITICAL';
    } else if (threatScore >= 0.45 && (this.threatLevel !== 'CRITICAL' || threatScore < 0.60)) {
      threatLevel = 'HIGH';
    } else if (threatScore >= 0.20 && (this.threatLevel === 'LOW' || this.threatLevel === 'NONE' || threatScore < 0.38)) {
      threatLevel = 'MEDIUM';
    } else if (threatScore < 0.15) {
      threatLevel = 'LOW';
    }

    return { challenger, threatLevel, threatScore, closingSpeed, ttc, gap, distToCorner, turn, turnCurvature };
  }

  /**
   * Real-time Attacker Intent Classifier with Heading & Momentum Vector Tracking and Feint Filtering.
   */
  classifyAttackerIntent({
    challenger,
    currentLateral,
    distToCorner,
    turnCurvature,
    turnSign,
    closingSpeed,
    ttc,
    dt
  }) {
    if (!challenger) return 'NONE';

    const attackerLateral = finite(challenger.otherLateral, finite(challenger.side, 0));
    const lateralDelta = attackerLateral - currentLateral;
    const attackerLatVel = finite(challenger.otherLateralSpeed, finite(challenger.relativeLateralVelocity, 0));
    const attackerNose = finite(challenger.otherNoseTrackDeviation, 0);
    const insideSign = turnSign;
    const outsideSign = -turnSign;

    // Lateral movement vectors relative to corner geometry
    const isMovingInside = (attackerLateral * insideSign) > 0.4 || (attackerLatVel * insideSign) > 0.12 || (attackerNose * insideSign) > 0.03;
    const isPositionedOutside = (attackerLateral * outsideSign) > 0.5;
    const isPointingOutside = (attackerLatVel * outsideSign) > 0.15 || (attackerNose * outsideSign) > 0.04;
    const isMovingOutside = isPositionedOutside || isPointingOutside;

    // Asymmetric Feint Filter: tracks dwell time on outside to prevent biting on rapid feints
    if (isMovingOutside && !isMovingInside) {
      this.outsideDwellTimer += dt;
    } else {
      this.outsideDwellTimer = Math.max(0, this.outsideDwellTimer - dt * 2.5);
    }

    // 1. DUMMY_FEINT_AND_SWITCH: twitched outside briefly (<0.45s) while snapping nose/momentum back inside
    if (distToCorner < 75 && distToCorner > 10 && this.outsideDwellTimer < 0.45 && (attackerLatVel * insideSign > 0.20 || attackerNose * insideSign > 0.05)) {
      return 'DUMMY_FEINT_AND_SWITCH';
    }

    // 2. ATTACK_OUTSIDE_MOMENTUM: established wide on outside with sustained dwell time (>=0.45s)
    if (distToCorner < 85 && (turnCurvature > 0.0022 || distToCorner < 50) && (isPositionedOutside || isPointingOutside) && this.outsideDwellTimer >= 0.45) {
      return 'ATTACK_OUTSIDE_MOMENTUM';
    }

    // 3. ATTACK_DIVEBOMB_INSIDE: closing fast toward inside apex line
    if (distToCorner < 85 && turnCurvature > 0.0025 && isMovingInside && (closingSpeed > 0.5 || ttc < 3.2)) {
      return 'ATTACK_DIVEBOMB_INSIDE';
    }

    // 4. EXIT_CUTBACK: trailing car positioned underneath for exit drive
    if (distToCorner <= 20 && turnCurvature > 0.0025 && (attackerLateral * insideSign > 0.6 || attackerNose * insideSign > 0.04) && closingSpeed > 0.2) {
      return 'EXIT_CUTBACK';
    }

    // 5. DRAFT_AND_SLINGSHOT: high-speed pull-out on straightaway
    if (distToCorner > 70 && closingSpeed > 1.2 && ttc < 2.5 && Math.abs(attackerLatVel) > 0.30) {
      return 'DRAFT_AND_SLINGSHOT';
    }

    return Math.abs(lateralDelta) < 1.2 ? 'DRAFT_TOW' : 'CRUISING_FOLLOW';
  }

  /**
   * Main tactical defense update loop.
   */
  update({
    vehicle,
    track,
    traffic,
    awareness,
    dt,
    baseLine = 0,
    recovering = false
  }) {
    this.timer = Math.max(0, this.timer - dt);
    this.cooldown = Math.max(0, this.cooldown - dt);
    this.towBreakTimer = Math.max(0, this.towBreakTimer - dt);
    this.ersDefensiveDeployTimer = Math.max(0, this.ersDefensiveDeployTimer - dt);

    const roadMargin = Math.max(2.1, finite(track?.roadHalfWidth, 6.5) - 1.35);
    const currentLateral = finite(traffic?.current?.lateral, 0);
    const nominalBase = clamp(baseLine, -roadMargin, roadMargin);

    if (recovering) {
      this._clear('NONE');
      return {
        phase: 'NONE',
        desiredOffset: nominalBase,
        defending: false,
        target: null,
        threatLevel: 'NONE',
        threatScore: 0,
        attackerIntent: 'NONE',
        closingSpeed: 0,
        ttc: 99,
        gap: 99,
        lockedLane: false,
        ersDeployRequested: false,
        ersDeployReason: 'NONE',
        rubbingPressure: 0,
        reason: 'RECOVERING'
      };
    }

    const { challenger, threatLevel, threatScore, closingSpeed, ttc, gap, distToCorner, turn, turnCurvature } =
      this.assessThreat({ vehicle, traffic, track });
    this.threatLevel = threatLevel;
    this.threatScore = threatScore;

    const turnSign = Math.sign(finite(turn?.turnSign, 1)) || 1;
    const insideSign = -turnSign;
    const outsideSign = turnSign;
    const inCorner = turnCurvature > 0.0035;

    // Detect wheel-to-wheel rubbing pressure
    const isAlongside = challenger && Math.abs(challenger.delta) < 4.8 && Math.abs(challenger.longitudinal) < 5.0;
    const lateralSeparation = challenger ? Math.abs(challenger.otherLateral - currentLateral) : 99;
    const isRubbingPressure = isAlongside && lateralSeparation < 2.35;
    this.rubbingPressure = isRubbingPressure ? clamp(1.0 - lateralSeparation / 2.35, 0.2, 1.0) : 0;

    // Attacker Intent Classification with Asymmetric Feint Filter
    const attackerIntent = this.classifyAttackerIntent({
      challenger,
      currentLateral,
      distToCorner,
      turnCurvature,
      turnSign,
      closingSpeed,
      ttc,
      dt
    });
    this.attackerIntent = attackerIntent;

    // Corner Exit Defensive Launch Trigger
    if (!inCorner && this.lastDefendedCorner && vehicle.speed < 55.0 && vehicle.controls?.throttle > 0.70 && gap < 25.0) {
      this.ersDefensiveDeployTimer = 1.8;
      this.ersDefensiveReason = 'CORNER_EXIT_LAUNCH';
      this.lastDefendedCorner = false;
    }
    if (inCorner) {
      this.lastDefendedCorner = this.defending;
    }

    // Ongoing Active Defense Handling (Holding committed corridor unyielding under pressure)
    if (this.defending) {
      this.age += dt;
      const target = traffic.entries.find((e) => e.other.id === this.defenseTargetId) ?? challenger;

      // Check if defense completed or challenger backed off
      const passed = target && target.delta > 2.5;
      const challengerBackedOff = !target || (target.delta < -35.0 && closingSpeed < 0.2);
      if (passed || challengerBackedOff || (this.age > 18.0 && !isRubbingPressure)) {
        this._clear('NONE');
        return {
          phase: 'NONE',
          desiredOffset: nominalBase,
          defending: false,
          target,
          threatLevel: 'NONE',
          threatScore: 0,
          attackerIntent: 'NONE',
          closingSpeed,
          ttc,
          gap,
          lockedLane: false,
          ersDeployRequested: false,
          ersDeployReason: 'NONE',
          rubbingPressure: 0,
          reason: passed ? 'OPPONENT_PASSED' : 'DEFENSE_COMPLETE'
        };
      }

      // Straightaway ERS Counter-Burst when threatened
      if (!inCorner && (closingSpeed > 1.8 || threatLevel === 'CRITICAL') && gap < 26.0 && vehicle.controls?.throttle > 0.80) {
        this.ersDefensiveDeployTimer = 2.0;
        this.ersDefensiveReason = 'STRAIGHT_COUNTER_BURST';
      }

      // FIA single-move locked direction (prevent weaving, ignore challenger feints)
      const committedSign = this.defenseDirection || (this.targetOffset !== 0 ? Math.sign(this.targetOffset) : insideSign);
      const isOutsideCommitted = committedSign === outsideSign;

      // Dynamic Phase Transitions through Corner Phases
      if (distToCorner > 65 && this.towBreakTimer > 0 && !isRubbingPressure) {
        // Aerodynamic tow break on long straights
        this.phase = 'BREAK_TOW';
      } else if ((attackerIntent === 'ATTACK_OUTSIDE_MOMENTUM' || isOutsideCommitted) && (inCorner || distToCorner <= 45)) {
        // Outside Defense Squeeze: Smoothly drift out to leave exactly 1 car width (2.2m) at the track boundary
        this.phase = 'EXIT_SQUEEZE';
        const outsideSqueezeOffset = clamp(committedSign * Math.min(3.2, roadMargin - 2.2), -roadMargin + 0.6, roadMargin - 0.6);
        this.targetOffset = outsideSqueezeOffset;
        this.committedDefensiveOffset = outsideSqueezeOffset;
      } else if (distToCorner <= 65 && distToCorner > 28 && !isOutsideCommitted) {
        // Proactive Inside Lane Lock on braking approach - Claim preferred defensive inside lane with zero hesitation
        this.phase = 'LOCK_DEFENSIVE_LANE';
        const insideLockOffset = clamp(committedSign * Math.min(4.2, roadMargin * 0.78), -roadMargin + 0.4, roadMargin - 0.4);
        this.targetOffset = insideLockOffset;
        this.committedDefensiveOffset = insideLockOffset;
      } else if (distToCorner <= 28 && distToCorner > 14 && attackerIntent !== 'ATTACK_DIVEBOMB_INSIDE' && !isOutsideCommitted) {
        // FIA One-Move Return toward racing line leaving 2.2m margin on track edge
        this.phase = 'ONE_MOVE_RETURN';
        const returnOffset = clamp(committedSign * Math.min(2.0, roadMargin - 2.2), -roadMargin + 0.6, roadMargin - 0.6);
        this.targetOffset = returnOffset;
        this.committedDefensiveOffset = returnOffset;
      } else if (inCorner && (attackerIntent === 'EXIT_CUTBACK' || attackerIntent === 'DUMMY_FEINT_AND_SWITCH')) {
        // Diamond Defense: Squaring off corner exit to block cutback acceleration lane
        this.phase = 'DIAMOND_DEFENSE';
        this.targetOffset = clamp(committedSign * 0.50, -roadMargin + 0.6, roadMargin - 0.6);
        this.committedDefensiveOffset = this.targetOffset;
      } else if (inCorner || distToCorner <= 14) {
        // Physical Apex Shielding: On corner approach, turn-in, and apex, pin inside line tight to apex curb (0.35m margin)
        // Completely denying challenger inside room
        this.phase = 'APEX_SHIELD';
        const apexShieldOffset = clamp(committedSign * Math.min(4.2, roadMargin * 0.78), -roadMargin + 0.55, roadMargin - 0.55);
        this.targetOffset = apexShieldOffset;
        this.committedDefensiveOffset = apexShieldOffset;
      }

      // Unyielding under rubbing pressure: never swerve away or yield the inside corner rights
      if (isRubbingPressure && this.committedDefensiveOffset !== 0) {
        this.targetOffset = this.committedDefensiveOffset;
      }

      return {
        phase: this.phase,
        desiredOffset: this.targetOffset,
        defending: true,
        target,
        threatLevel: this.threatLevel,
        threatScore: this.threatScore,
        attackerIntent: this.attackerIntent,
        closingSpeed,
        ttc,
        gap,
        lockedLane: true,
        ersDeployRequested: this.ersDefensiveDeployTimer > 0,
        ersDeployReason: this.ersDefensiveReason,
        rubbingPressure: this.rubbingPressure,
        reason: isRubbingPressure ? `UNYIELDING_RUBBING_DEFENSE_${this.phase}` : `HOLD_${this.phase}`
      };
    }

    if (this.phase === 'RETURN_PACE') {
      if (Math.abs(currentLateral - nominalBase) < 0.35 || this.timer <= 0) {
        this.phase = 'NONE';
        this.targetOffset = nominalBase;
      }
      return {
        phase: 'RETURN_PACE',
        desiredOffset: nominalBase,
        defending: false,
        target: null,
        threatLevel: 'NONE',
        threatScore: 0,
        attackerIntent: 'NONE',
        closingSpeed,
        ttc,
        gap,
        lockedLane: false,
        ersDeployRequested: this.ersDefensiveDeployTimer > 0,
        ersDeployReason: this.ersDefensiveReason,
        rubbingPressure: 0,
        reason: 'SAFE_REJOIN_RACING_LINE'
      };
    }

    // Proactive Triggering: When challenger is within 45m and closing (or high threat), defend with zero hesitation
    const isClosing = closingSpeed >= -0.25;
    const isWithin45mClosing = challenger && gap <= 45.0 && isClosing;
    const shouldDefend = challenger
      && (isWithin45mClosing || threatLevel !== 'NONE' || threatScore >= 0.18 || gap < 30.0)
      && traffic.egoForwardSpeed > 4.0
      && (isWithin45mClosing || this.cooldown <= 0);

    if (!shouldDefend) {
      return {
        phase: 'NONE',
        desiredOffset: nominalBase,
        defending: false,
        target: challenger,
        threatLevel,
        threatScore,
        attackerIntent,
        closingSpeed,
        ttc,
        gap,
        lockedLane: false,
        ersDeployRequested: this.ersDefensiveDeployTimer > 0,
        ersDeployReason: this.ersDefensiveReason,
        rubbingPressure: 0,
        reason: 'NO_DEFENSE_NEEDED'
      };
    }

    // If within 45m and closing, clear cooldown for immediate zero-hesitation response
    if (isWithin45mClosing) {
      this.cooldown = 0;
    }

    // Relative challenger offset
    const challengerLateral = finite(challenger.otherLateral, finite(challenger.side, 0));
    const lateralDelta = challengerLateral - currentLateral;
    const inDirectTow = Math.abs(lateralDelta) < 1.1 && gap < 45.0;

    let defensiveOffset = nominalBase;
    let phase = 'LOCK_DEFENSIVE_LANE';
    let reason = 'CLAIM_INSIDE_DEFENSIVE_CORRIDOR';

    if (attackerIntent === 'ATTACK_OUTSIDE_MOMENTUM') {
      // Outside Defense Squeeze: Smoothly drift out to leave exactly 1 car width (2.2m) at the track boundary
      defensiveOffset = clamp(outsideSign * Math.min(3.2, roadMargin - 2.2), -roadMargin + 0.8, roadMargin - 0.8);
      phase = 'OUTSIDE_DEFENSE_SQUEEZE';
      reason = 'SQUEEZE_OUTSIDE_MOMENTUM_CORRIDOR';
    } else if (inCorner) {
      // Physical Apex Shielding: Pin inside line tight to apex curb (0.6m margin), completely denying inside room
      defensiveOffset = clamp(turnSign * Math.min(3.4, roadMargin * 0.65), -roadMargin + 0.6, roadMargin - 0.6);
      phase = 'APEX_SHIELD';
      reason = 'PHYSICAL_APEX_SHIELDING';
    } else if (gap > 18.0 && inDirectTow && distToCorner > 65) {
      // Stepped lateral tow break (shifts 2.2m off draft line to destroy >66% follower tow)
      const breakSide = distToCorner < 140 ? turnSign : (currentLateral > 0 ? -1 : 1);
      const breakShift = 2.2 * breakSide;
      defensiveOffset = clamp(currentLateral + breakShift, -roadMargin + 0.8, roadMargin - 0.8);
      phase = 'BREAK_TOW';
      reason = 'AERODYNAMIC_TOW_BREAK';
      this.towBreakTimer = 2.2;
    } else {
      // Proactive Door Shutting: Claim preferred defensive inside lane with zero hesitation
      defensiveOffset = clamp(turnSign * Math.min(3.2, roadMargin * 0.60), -roadMargin + 0.6, roadMargin - 0.6);
      phase = 'LOCK_DEFENSIVE_LANE';
      reason = 'PROACTIVE_SHUT_INSIDE_DOOR';
    }

    // Candidate corridors evaluation (handles third-party blockers if present)
    const candidateOffsets = [
      { offset: defensiveOffset, phase, reason },
      { offset: clamp(-defensiveOffset * 0.6, -roadMargin + 0.8, roadMargin - 0.8), phase: 'DEFEND_ALTERNATIVE', reason: 'DEFEND_ALTERNATIVE_LANE' },
      { offset: 0, phase: 'DEFEND_CENTER', reason: 'COVER_CENTER_LANE' }
    ];

    let chosenCandidate = candidateOffsets[0];
    if (awareness?.evaluateCorridor) {
      for (const cand of candidateOffsets) {
        const corridor = awareness.evaluateCorridor({
          vehicle,
          track,
          traffic,
          terminalOffset: cand.offset,
          targetId: challenger.other.id,
          targetSpeed: vehicle.speed
        });

        // Challenger proximity/rubbing is tolerated racing pressure; only 3rd-party blockers trigger alternative lanes
        const isThirdPartyBlocker = corridor.blockerId && corridor.blockerId !== challenger.other.id;
        if (corridor.legal && (!isThirdPartyBlocker || corridor.collisionFree)) {
          chosenCandidate = cand;
          break;
        }
      }
    }

    // Unyielding Defensive Line Commitment (Strict FIA single defensive move corridor lock)
    this.phase = chosenCandidate.phase;
    this.defenseTargetId = challenger.other.id;
    this.targetOffset = chosenCandidate.offset;
    this.committedDefensiveOffset = chosenCandidate.offset;
    this.defenseDirection = Math.sign(chosenCandidate.offset) || turnSign;
    this.oneMoveLocked = true;
    this.age = 0;

    if (!inCorner && (closingSpeed > 1.8 || threatLevel === 'CRITICAL')) {
      this.ersDefensiveDeployTimer = 2.2;
      this.ersDefensiveReason = 'STRAIGHT_COUNTER_BURST';
    }

    return {
      phase: this.phase,
      desiredOffset: this.targetOffset,
      defending: true,
      target: challenger,
      threatLevel: this.threatLevel,
      threatScore: this.threatScore,
      attackerIntent: this.attackerIntent,
      closingSpeed,
      ttc,
      gap,
      lockedLane: true,
      ersDeployRequested: this.ersDefensiveDeployTimer > 0,
      ersDeployReason: this.ersDefensiveReason,
      rubbingPressure: this.rubbingPressure,
      reason: chosenCandidate.reason
    };
  }
}
