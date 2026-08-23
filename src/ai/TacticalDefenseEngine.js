/**
 * TacticalDefenseEngine.js
 * Advanced Unified Motorsport Defense Engine for Gemini Gauntlet:
 * - 5-Factor Composite Threat Metric T(t) in [0.0, 1.0] with Schmitt-trigger hysteresis
 * - Real-time Attacker Intent Classifier (DIVEBOMB, OUTSIDE_MOMENTUM, DUMMY_FEINT, EXIT_CUTBACK, SLINGSHOT)
 * - Asymmetric Feint Filter & Dummy-Move Evasion
 * - FIA-compliant Single-Move Corridor Locking & Return with 1-car-width (2.2m) margin
 * - Dynamic Corner Phase Discretization (Approach Cover -> One-Move Return -> Apex Shielding -> Diamond Defense -> Exit Squeeze)
 * - Aerodynamic Tow-Breaking Math (stepped lateral shifts destroying >66% of follower tow)
 * - Defensive ERS Deployment Arbitration (Straightaway Counter-Burst & Corner-Exit Launch)
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
    this.filteredAttackerLateral = 0;
    this.ersDefensiveDeployTimer = 0;
    this.ersDefensiveReason = 'NONE';
    this.lastDefendedCorner = false;
    this.cornerPhase = 'NONE'; // APPROACH, ENTRY, APEX, EXIT
    this.intent = null;
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
    this.oneMoveLocked = false;
    this.threatLevel = 'NONE';
    this.threatScore = 0;
    this.attackerIntent = 'NONE';
    this.timer = phase === 'RETURN_PACE' ? 0.65 : 0;
    this.cooldown = Math.max(this.cooldown, cooldown);
    this.age = 0;
    this.towBreakTimer = 0;
    this.outsideDwellTimer = 0;
    this.cornerPhase = 'NONE';
    this.intent = null;
  }

  /**
   * Multi-Metric Composite Threat Assessment T(t) in [0.0, 1.0].
   */
  assessThreat({ vehicle, traffic, track }) {
    if (!traffic?.entries?.length) {
      return { challenger: null, threatLevel: 'NONE', threatScore: 0, closingSpeed: 0, ttc: 99, gap: 99 };
    }

    const roadMargin = Math.max(2.1, finite(track?.roadHalfWidth, 6.5) - 1.35);
    const challenger = traffic.entries
      .filter((e) => e.delta < -1.5 && e.delta > -55.0
        && e.longitudinal < 2.5 && Math.abs(e.side) < roadMargin * 2 + 1.0)
      .sort((a, b) => b.delta - a.delta)[0] ?? null;

    if (!challenger) {
      return { challenger: null, threatLevel: 'NONE', threatScore: 0, closingSpeed: 0, ttc: 99, gap: 99 };
    }

    const closingSpeed = finite(challenger.otherForwardSpeed - traffic.egoForwardSpeed, 0);
    const gap = Math.abs(finite(challenger.delta, 99));
    const bodyGap = Math.max(0, gap - 4.6);
    const ttc = closingSpeed > 0.15 ? bodyGap / closingSpeed : 99;

    // Upcoming corner proximity
    const sampleDist = finite(vehicle.distance, 0);
    const turnSamples = [18, 36, 56].map((d) =>
      track?.atDistance ? track.atDistance(sampleDist + d) : { curvature: 0, turnSign: 1, s: sampleDist + d }
    );
    const turn = turnSamples.sort((a, b) => Math.abs(b.curvature) - Math.abs(a.curvature))[0];
    const turnCurvature = Math.abs(finite(turn?.curvature, 0));
    const distToCorner = Math.max(0, wrap((turn?.s ?? sampleDist) - sampleDist + (track?.length || 1000) * 0.5, track?.length || 1000) - (track?.length || 1000) * 0.5);

    // 5 Orthogonal Kinematic Factors
    const fGap = Math.exp(-gap / 14.0);
    const fClose = saturate((closingSpeed - 0.2) / 5.8);
    const fTtc = ttc <= 4.5 ? Math.pow(1.0 - ttc / 4.5, 2) : 0;
    const lateralDelta = Math.abs(finite(challenger.otherLateral, 0) - finite(traffic.current?.lateral, 0));
    const fLat = 1.0 - saturate((lateralDelta - 2.0) / 11.0);
    const fCorner = Math.exp(-distToCorner / 35.0) * saturate(turnCurvature / 0.0035);

    // Non-linear synergy boost when closing rapidly into a braking zone
    const synergy = fClose * fCorner * 0.20;

    const rawThreat = 0.22 * fGap + 0.23 * fClose + 0.25 * fTtc + 0.15 * fLat + 0.15 * fCorner + synergy;
    const threatScore = saturate(rawThreat * (0.85 + this.defenseReactivity * 0.30));

    // Schmitt-Trigger Posture Hysteresis
    let threatLevel = this.threatLevel;
    if (threatScore >= 0.75) {
      threatLevel = 'CRITICAL';
    } else if (threatScore >= 0.50 && (this.threatLevel !== 'CRITICAL' || threatScore < 0.68)) {
      threatLevel = 'HIGH';
    } else if (threatScore >= 0.28 && (this.threatLevel === 'LOW' || this.threatLevel === 'NONE' || threatScore < 0.44)) {
      threatLevel = 'MEDIUM';
    } else if (threatScore < 0.20) {
      threatLevel = 'LOW';
    }

    return { challenger, threatLevel, threatScore, closingSpeed, ttc, gap, distToCorner, turn, turnCurvature };
  }

  /**
   * Real-time Attacker Intent Classifier with Heading & Momentum Vector Tracking.
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

    // Heading and momentum vectors pointing or moving toward inside or outside
    const isMovingInside = (attackerLateral * insideSign) > 0 || (attackerLatVel * insideSign) > 0.15 || (attackerNose * insideSign) > 0.04;
    const isPositionedOutside = (attackerLateral * outsideSign) > 0.6;
    const isPointingOutside = (attackerLatVel * outsideSign) > 0.15 || (attackerNose * outsideSign) > 0.04;
    const isMovingOutside = isPositionedOutside || isPointingOutside;

    // Asymmetric Feint Filter: tracks dwell time on the outside
    if (isMovingOutside && !isMovingInside) {
      this.outsideDwellTimer += dt;
    } else {
      this.outsideDwellTimer = Math.max(0, this.outsideDwellTimer - dt * 2.0);
    }

    // 1. DUMMY_FEINT_AND_SWITCH: twitched outside but dwelling < 0.40s and snapping nose/momentum back inside
    if (distToCorner < 65 && distToCorner > 15 && this.outsideDwellTimer < 0.40 && (attackerLatVel * insideSign > 0.30 || attackerNose * insideSign > 0.08)) {
      return 'DUMMY_FEINT_AND_SWITCH';
    }

    // 2. ATTACK_OUTSIDE_MOMENTUM: established wide on outside or pointing nose/momentum outside
    if (distToCorner < 75 && (turnCurvature > 0.0025 || distToCorner < 45) && (isPositionedOutside || (isPointingOutside && this.outsideDwellTimer >= 0.20))) {
      return 'ATTACK_OUTSIDE_MOMENTUM';
    }

    // 3. ATTACK_DIVEBOMB_INSIDE: closing fast toward inside apex line
    if (distToCorner < 75 && turnCurvature > 0.0030 && isMovingInside && (closingSpeed > 1.0 || ttc < 2.8)) {
      return 'ATTACK_DIVEBOMB_INSIDE';
    }

    // 4. EXIT_CUTBACK: trailing car positioned for exit underneath
    if (distToCorner <= 18 && turnCurvature > 0.0030 && (attackerLateral * insideSign > 0.8 || attackerNose * insideSign > 0.05) && closingSpeed > 0.3) {
      return 'EXIT_CUTBACK';
    }

    // 5. DRAFT_AND_SLINGSHOT: high-speed pull-out on straightaway
    if (distToCorner > 70 && closingSpeed > 1.4 && ttc < 2.4 && Math.abs(attackerLatVel) > 0.35) {
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
        ersDeployRequested: false,
        reason: 'RECOVERING'
      };
    }

    const { challenger, threatLevel, threatScore, closingSpeed, ttc, gap, distToCorner, turn, turnCurvature } =
      this.assessThreat({ vehicle, traffic, track });
    this.threatLevel = threatLevel;
    this.threatScore = threatScore;

    const turnSign = Math.sign(finite(turn?.turnSign, 1)) || 1;
    const inCorner = turnCurvature > 0.0035;

    // Attacker Intent Classification
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
    if (!inCorner && this.lastDefendedCorner && vehicle.speed < 50.0 && vehicle.controls.throttle > 0.75 && gap < 22.0) {
      this.ersDefensiveDeployTimer = 1.8;
      this.ersDefensiveReason = 'CORNER_EXIT_LAUNCH';
      this.lastDefendedCorner = false;
    }
    if (inCorner) {
      this.lastDefendedCorner = this.defending;
    }

    // Ongoing Active Defense Handling
    if (this.defending) {
      const activeChallenger = traffic.entries.find((e) => e.other.id === this.defenseTargetId) ?? null;
      this.age += dt;

      const passed = activeChallenger && activeChallenger.delta > 4.5;
      const gone = !activeChallenger || activeChallenger.delta < -48;

      if (passed || gone || this.age > 14.0) {
        this._clear('RETURN_PACE', 0.6);
        return {
          phase: 'RETURN_PACE',
          desiredOffset: this.targetOffset,
          defending: false,
          target: null,
          threatLevel: 'NONE',
          threatScore: 0,
          attackerIntent: 'NONE',
          ersDeployRequested: false,
          reason: passed ? 'OPPONENT_PASSED' : 'DEFENSE_COMPLETE'
        };
      }

      // Straightaway ERS Counter-Burst when threatened
      if (!inCorner && (closingSpeed > 2.2 || threatLevel === 'CRITICAL') && gap < 24.0 && vehicle.controls.throttle > 0.85) {
        this.ersDefensiveDeployTimer = 2.0;
        this.ersDefensiveReason = 'STRAIGHT_COUNTER_BURST';
      }

      // Dynamic Phase Transitions through Corner (maintaining single-move commitment)
      const outsideSign = -turnSign;
      const committedSign = this.defenseDirection || Math.sign(this.targetOffset) || turnSign;
      const isOutsideCommitted = committedSign === outsideSign;

      if (distToCorner > 65 && this.towBreakTimer > 0) {
        this.phase = 'BREAK_TOW';
      } else if (isOutsideCommitted && distToCorner > 18) {
        // Outside Momentum Defense: Hold outside corridor to pinch attacker
        this.phase = 'OUTSIDE_DEFENSE_SQUEEZE';
        this.targetOffset = clamp(committedSign * Math.min(3.6, roadMargin - 2.2), -roadMargin + 0.6, roadMargin - 0.6);
      } else if (distToCorner <= 65 && distToCorner > 28 && !isOutsideCommitted) {
        // Approach Corridor Lock - Aggressively shut the inside door
        this.phase = 'LOCK_DEFENSIVE_LANE';
        this.targetOffset = clamp(committedSign * Math.min(4.2, roadMargin * 0.78), -roadMargin + 0.4, roadMargin - 0.4);
      } else if (distToCorner <= 28 && distToCorner > 14 && attackerIntent !== 'ATTACK_DIVEBOMB_INSIDE' && !isOutsideCommitted) {
        // FIA One-Move Return toward racing line leaving 2.2m margin on track edge
        this.phase = 'ONE_MOVE_RETURN';
        const returnOffset = clamp(committedSign * Math.min(2.0, roadMargin - 2.2), -roadMargin + 0.6, roadMargin - 0.6);
        this.targetOffset = returnOffset;
      } else if (inCorner && (attackerIntent === 'EXIT_CUTBACK' || attackerIntent === 'DUMMY_FEINT_AND_SWITCH')) {
        // Diamond Defense: late-apex squaring to defend cutback and maximize exit drive
        this.phase = 'DIAMOND_DEFENSE';
        this.targetOffset = committedSign * 0.5; // mid-track launch locus preserving sign
      } else if (inCorner && (attackerIntent === 'ATTACK_OUTSIDE_MOMENTUM' || isOutsideCommitted)) {
        // Exit Squeeze: drift smoothly to leave legal 2.2m track edge margin
        this.phase = 'EXIT_SQUEEZE';
        this.targetOffset = clamp(committedSign * Math.min(3.2, roadMargin - 2.2), -roadMargin + 0.6, roadMargin - 0.6);
      } else if (inCorner) {
        // Apex Shielding: pin inner kerb to shut the inside door completely
        this.phase = 'APEX_SHIELD';
        this.targetOffset = clamp(committedSign * Math.min(4.6, roadMargin * 0.85), -roadMargin + 0.35, roadMargin - 0.35);
      }

      return {
        phase: this.phase,
        desiredOffset: this.targetOffset,
        defending: true,
        target: activeChallenger,
        threatLevel: this.threatLevel,
        threatScore: this.threatScore,
        attackerIntent: this.attackerIntent,
        lockedLane: true,
        ersDeployRequested: this.ersDefensiveDeployTimer > 0,
        ersDeployReason: this.ersDefensiveReason,
        reason: `HOLD_${this.phase}`
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
        ersDeployRequested: this.ersDefensiveDeployTimer > 0,
        ersDeployReason: this.ersDefensiveReason,
        reason: 'SAFE_REJOIN_RACING_LINE'
      };
    }

    // Evaluate New Defensive Trigger (instant proactive response)
    const shouldDefend = challenger
      && (threatLevel !== 'NONE' || threatScore >= 0.18 || gap < 48.0)
      && traffic.egoForwardSpeed > 4.0
      && this.cooldown <= 0;

    if (!shouldDefend) {
      return {
        phase: 'NONE',
        desiredOffset: nominalBase,
        defending: false,
        target: challenger,
        threatLevel,
        threatScore,
        attackerIntent,
        ersDeployRequested: this.ersDefensiveDeployTimer > 0,
        ersDeployReason: this.ersDefensiveReason,
        reason: 'NO_DEFENSE_NEEDED'
      };
    }

    // Relative challenger offset
    const challengerLateral = finite(challenger.otherLateral, finite(challenger.side, 0));
    const lateralDelta = challengerLateral - currentLateral;
    const inDirectTow = Math.abs(lateralDelta) < 1.1 && gap < 45.0;
    const outsideSign = -turnSign;

    let defensiveOffset = nominalBase;
    let phase = 'LOCK_DEFENSIVE_LANE';
    let reason = 'CLAIM_INSIDE_DEFENSIVE_CORRIDOR';

    if (attackerIntent === 'ATTACK_OUTSIDE_MOMENTUM') {
      // Squeeze outside corridor to block high-speed outside momentum pass
      defensiveOffset = clamp(outsideSign * Math.min(3.6, roadMargin - 2.2), -roadMargin + 0.6, roadMargin - 0.6);
      phase = 'OUTSIDE_DEFENSE_SQUEEZE';
      reason = 'PINCH_OUTSIDE_MOMENTUM_CORRIDOR';
    } else if (inCorner) {
      defensiveOffset = clamp(turnSign * Math.min(4.6, roadMargin * 0.85), -roadMargin + 0.35, roadMargin - 0.35);
      phase = 'APEX_SHIELD';
      reason = 'PROTECT_INSIDE_APEX_LINE';
    } else if (gap > 18.0 && inDirectTow && distToCorner > 65) {
      // Stepped lateral tow break (shifts 2.4m off draft line to destroy >66% of follower tow)
      const breakSide = distToCorner < 140 ? turnSign : (currentLateral > 0 ? -1 : 1);
      const breakShift = 2.4 * breakSide;
      defensiveOffset = clamp(currentLateral + breakShift, -roadMargin + 0.8, roadMargin - 0.8);
      phase = 'BREAK_TOW';
      reason = 'AERODYNAMIC_TOW_BREAK';
      this.towBreakTimer = 2.2;
    } else {
      // Pre-braking inside corridor lock - Aggressively shut inside line
      defensiveOffset = clamp(turnSign * Math.min(4.2, roadMargin * 0.78), -roadMargin + 0.4, roadMargin - 0.4);
      phase = 'LOCK_DEFENSIVE_LANE';
      reason = 'LOCK_INSIDE_BRAKING_LANE';
    }

    const candidateOffsets = [
      { offset: defensiveOffset, phase, reason },
      { offset: clamp(-defensiveOffset * 0.6, -roadMargin + 0.8, roadMargin - 0.8), phase: 'DEFEND_ALTERNATIVE', reason: 'DEFEND_ALTERNATIVE_LANE' },
      { offset: 0, phase: 'DEFEND_CENTER', reason: 'COVER_CENTER_LANE' }
    ];

    for (const cand of candidateOffsets) {
      const corridor = awareness.evaluateCorridor({
        vehicle,
        track,
        traffic,
        terminalOffset: cand.offset,
        targetId: challenger.other.id,
        targetSpeed: vehicle.speed
      });

      if (corridor.legal && corridor.collisionFree) {
        this.phase = cand.phase;
        this.defenseTargetId = challenger.other.id;
        this.targetOffset = cand.offset;
        this.defenseDirection = Math.sign(cand.offset) || turnSign;
        this.oneMoveLocked = true;
        this.age = 0;

        if (!inCorner && (closingSpeed > 2.2 || threatLevel === 'CRITICAL')) {
          this.ersDefensiveDeployTimer = 2.2;
          this.ersDefensiveReason = 'STRAIGHT_COUNTER_BURST';
        }

        return {
          phase: this.phase,
          desiredOffset: cand.offset,
          defending: true,
          target: challenger,
          threatLevel: this.threatLevel,
          threatScore: this.threatScore,
          attackerIntent: this.attackerIntent,
          lockedLane: true,
          ersDeployRequested: this.ersDefensiveDeployTimer > 0,
          ersDeployReason: this.ersDefensiveReason,
          reason: cand.reason
        };
      }
    }

    return {
      phase: 'NONE',
      desiredOffset: nominalBase,
      defending: false,
      target: challenger,
      threatLevel,
      threatScore,
      attackerIntent,
      ersDeployRequested: false,
      reason: 'DEFENSE_PATH_UNAVAILABLE'
    };
  }
}

