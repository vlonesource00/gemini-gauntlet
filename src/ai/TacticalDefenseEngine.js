/**
 * TacticalDefenseEngine.js
 * Modular Motorsport Combat Defense, Line Protection & Racecraft Intelligence:
 * - FIA Single-Move Rule Enforcement (Strict Non-Weaving Directional Locking)
 * - Proactive Door Shutting (Claiming inside defensive lane up to 45m ahead)
 * - Physical Apex Shielding (APEX_SHIELD: pinning inside line tight to apex curb)
 * - Outside Exit Squeezing (EXIT_SQUEEZE: leaving exactly 1 car width at edge)
 * - Aerodynamic Tow Breaking (BREAK_TOW: stepped lateral shifts destroying follower draft)
 * - Unyielding Side-by-Side Rubbing Pressure Defense
 * - Asymmetric Attacker Feint Filtering
 */

import { clamp, wrap, saturate } from '../core/math.js';

const finite = (value, fallback = 0) => (Number.isFinite(value) ? value : fallback);

export class TacticalDefenseEngine {
  /**
   * @param {Object} [options]
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
    this.cornerPhase = 'NONE';
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
  assessThreat({ vehicle, traffic, track, passedTargetId = null }) {
    if (!traffic?.entries?.length) {
      return { challenger: null, threatLevel: 'NONE', threatScore: 0, closingSpeed: 0, ttc: 99, gap: 99, distToCorner: 999, turn: null, turnCurvature: 0 };
    }

    const roadMargin = Math.max(2.1, finite(track?.roadHalfWidth, 6.5) - 1.35);
    const challenger = traffic.entries
      .filter((e) => {
        if (e.delta >= -0.8 || e.delta <= -55.0 || e.longitudinal >= 2.5 || Math.abs(e.side) >= roadMargin * 2 + 1.0) {
          return false;
        }
        if (passedTargetId && e.other.id === passedTargetId) {
          const closing = e.otherForwardSpeed - traffic.egoForwardSpeed;
          if (Math.abs(e.delta) > 5.5 || closing <= 0.4) {
            return false;
          }
        }
        return true;
      })
      .sort((a, b) => b.delta - a.delta)[0] ?? null;

    if (!challenger) {
      return { challenger: null, threatLevel: 'NONE', threatScore: 0, closingSpeed: 0, ttc: 99, gap: 99, distToCorner: 999, turn: null, turnCurvature: 0 };
    }

    const closingSpeed = finite(challenger.otherForwardSpeed - traffic.egoForwardSpeed, 0);
    const gap = Math.abs(finite(challenger.delta, 99));
    const bodyGap = Math.max(0, gap - 4.6);
    const ttc = closingSpeed > 0.10 ? bodyGap / closingSpeed : (closingSpeed > -0.4 ? bodyGap / 0.15 : 99);

    const sampleDist = finite(vehicle.distance, 0);
    const turnSamples = [0, 6, 12, 20, 32, 50, 75].map((d) =>
      track?.atDistance ? track.atDistance(sampleDist + d) : { curvature: 0, turnSign: 1, s: sampleDist + d }
    );
    const turn = turnSamples.find((t) => Math.abs(finite(t?.curvature, 0)) > 0.003)
      || turnSamples.sort((a, b) => Math.abs(b.curvature) - Math.abs(a.curvature))[0];
    const turnCurvature = Math.abs(finite(turn?.curvature, 0));
    const distToCorner = Math.max(0, wrap((turn?.s ?? sampleDist) - sampleDist + (track?.length || 1000) * 0.5, track?.length || 1000) - (track?.length || 1000) * 0.5);

    // 5 Orthogonal Kinematic Factors
    const fGap = Math.exp(-gap / 16.0);
    const fClose = saturate((closingSpeed + 0.5) / 5.2);
    const fTtc = ttc <= 5.0 ? Math.pow(1.0 - ttc / 5.0, 2) : 0;
    const lateralDelta = Math.abs(finite(challenger.otherLateral, 0) - finite(traffic.current?.lateral, 0));
    const fLat = 1.0 - saturate((lateralDelta - 1.4) / 10.0);
    const fCorner = Math.exp(-distToCorner / 45.0) * saturate(turnCurvature / 0.0028);

    // Proactive door shutting trigger: within striking distance or actively closing
    const isClosingThreat = (gap < 9.0) || (gap <= 30.0 && closingSpeed >= 0.25) || (gap <= 45.0 && closingSpeed >= 1.2 && distToCorner < 75);
    const proactiveBoost = isClosingThreat ? 0.30 : 0;

    const synergy = fClose * fCorner * 0.28 + proactiveBoost;

    const rawThreat = 0.20 * fGap + 0.22 * fClose + 0.24 * fTtc + 0.14 * fLat + 0.15 * fCorner + synergy;
    const threatScore = saturate(rawThreat * (0.85 + this.defenseReactivity * 0.35));

    // Schmitt-Trigger Posture Hysteresis
    let threatLevel = this.threatLevel;
    if (threatScore >= 0.65 || (isClosingThreat && threatScore >= 0.40)) {
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
   * Real-time Attacker Intent Classifier with Feint Filtering.
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

    const isMovingInside = (attackerLateral * insideSign) > 0.4 || (attackerLatVel * insideSign) > 0.12 || (attackerNose * insideSign) > 0.03;
    const isPositionedOutside = (attackerLateral * outsideSign) > 0.5;
    const isPointingOutside = (attackerLatVel * outsideSign) > 0.15 || (attackerNose * outsideSign) > 0.04;
    const isMovingOutside = isPositionedOutside || isPointingOutside;

    // Asymmetric Feint Filter: tracks dwell time on outside
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
    recovering = false,
    passedTargetId = null
  }) {
    this.timer = Math.max(0, this.timer - dt);
    this.cooldown = Math.max(0, this.cooldown - dt);
    this.towBreakTimer = Math.max(0, this.towBreakTimer - dt);
    this.ersDefensiveDeployTimer = Math.max(0, this.ersDefensiveDeployTimer - dt);

    const roadMargin = Math.max(2.1, Math.min(5.2, finite(track?.roadHalfWidth, 6.5) - 1.8));
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
      this.assessThreat({ vehicle, traffic, track, passedTargetId });
    this.threatLevel = threatLevel;
    this.threatScore = threatScore;

    const turnSign = Math.sign(finite(turn?.turnSign, 1)) || 1;
    const insideSign = turnSign;
    const outsideSign = -turnSign;
    const inCorner = turnCurvature > 0.0035;

    // Detect wheel-to-wheel rubbing pressure
    const isAlongside = challenger && Math.abs(challenger.delta) < 4.8 && Math.abs(challenger.longitudinal) < 5.0;
    const lateralSeparation = challenger ? Math.abs(challenger.otherLateral - currentLateral) : 99;
    const isRubbingPressure = isAlongside && lateralSeparation < 2.35;
    this.rubbingPressure = isRubbingPressure ? clamp(1.0 - lateralSeparation / 2.35, 0.2, 1.0) : 0;

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

    // Ongoing Active Defense Handling
    if (this.defending) {
      this.age += dt;
      const target = traffic.entries.find((e) => e.other.id === this.defenseTargetId) ?? challenger;

      const passed = target && target.delta > 2.5;
      const challengerBackedOff = !target || (target.delta < -18.0 && closingSpeed < 0.3) || (target.delta < -12.0 && closingSpeed < -0.4);
      if (passed || challengerBackedOff || (this.age > 6.0 && !isRubbingPressure)) {
        this._clear('RETURN_PACE', 0.5);
        return {
          phase: 'RETURN_PACE',
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

      if (!inCorner && (closingSpeed > 1.8 || threatLevel === 'CRITICAL') && gap < 26.0 && vehicle.controls?.throttle > 0.80) {
        this.ersDefensiveDeployTimer = 2.0;
        this.ersDefensiveReason = 'STRAIGHT_COUNTER_BURST';
      }

      // FIA single-move locked direction (strictly prevent weaving)
      const committedSign = this.defenseDirection || (this.targetOffset !== 0 ? Math.sign(this.targetOffset) : insideSign);
      const isOutsideCommitted = this.defenseDirection !== 0 && this.defenseDirection === outsideSign;

      // Dynamic Phase Transitions through Corner Phases
      if (distToCorner > 65) {
        if (this.towBreakTimer > 0 && !isRubbingPressure) {
          this.phase = 'BREAK_TOW';
        } else if (gap < 24.0 && (closingSpeed > 0.6 || threatLevel === 'CRITICAL')) {
          this.phase = 'LOCK_DEFENSIVE_LANE';
          this.targetOffset = clamp(committedSign * Math.min(2.5, roadMargin * 0.50), -roadMargin + 1.0, roadMargin - 1.0);
          this.committedDefensiveOffset = this.targetOffset;
        } else {
          this._clear('RETURN_PACE', 0.5);
          return {
            phase: 'RETURN_PACE',
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
            reason: 'DEFENSE_RELEASE_STRAIGHT'
          };
        }
      } else if (gap > 12.0 && inCorner) {
        this.phase = 'PACE_DEFEND';
        this.targetOffset = nominalBase;
      } else if ((attackerIntent === 'ATTACK_OUTSIDE_MOMENTUM' || isOutsideCommitted) && (inCorner || distToCorner <= 45)) {
        // Outside Defense Squeeze: Smoothly drift out to leave exactly 1 car width (2.2m) at the track boundary
        this.phase = 'EXIT_SQUEEZE';
        const outsideSqueezeOffset = clamp(committedSign * Math.min(2.8, roadMargin - 2.4), -roadMargin + 1.0, roadMargin - 1.0);
        this.targetOffset = outsideSqueezeOffset;
        this.committedDefensiveOffset = outsideSqueezeOffset;
      } else if (distToCorner <= 65 && distToCorner > 28 && !isOutsideCommitted) {
        // Proactive Inside Lane Lock on braking approach - Claim inside defensive lane with zero hesitation
        this.phase = 'LOCK_DEFENSIVE_LANE';
        const insideLockOffset = clamp(committedSign * Math.min(3.5, roadMargin * 0.65), -roadMargin + 1.0, roadMargin - 1.0);
        this.targetOffset = insideLockOffset;
        this.committedDefensiveOffset = insideLockOffset;
      } else if (distToCorner <= 28 && distToCorner > 14 && attackerIntent !== 'ATTACK_DIVEBOMB_INSIDE' && !isOutsideCommitted) {
        // FIA One-Move Return toward racing line leaving 2.2m margin on track edge
        this.phase = 'ONE_MOVE_RETURN';
        const returnOffset = clamp(committedSign * Math.min(1.8, roadMargin - 2.4), -roadMargin + 1.0, roadMargin - 1.0);
        this.targetOffset = returnOffset;
        this.committedDefensiveOffset = returnOffset;
      } else if (inCorner && (attackerIntent === 'EXIT_CUTBACK' || attackerIntent === 'DUMMY_FEINT_AND_SWITCH')) {
        // Diamond Defense: Squaring off corner exit to block cutback acceleration lane
        this.phase = 'DIAMOND_DEFENSE';
        this.targetOffset = clamp(committedSign * 0.50, -roadMargin + 1.0, roadMargin - 1.0);
        this.committedDefensiveOffset = this.targetOffset;
      } else if (inCorner || distToCorner <= 14) {
        if (inCorner) {
          // Physical Apex Shielding through corner sequences & chicanes
          this.phase = 'APEX_SHIELD';
          const apexShieldOffset = clamp(insideSign * Math.min(2.8, roadMargin * 0.50), -roadMargin + 1.0, roadMargin - 1.0);
          this.targetOffset = apexShieldOffset;
          this.committedDefensiveOffset = apexShieldOffset;
        } else {
          const isCurrentOutside = (currentLateral * outsideSign) > 0.8;
          if (isCurrentOutside || isOutsideCommitted) {
            this.phase = 'EXIT_SQUEEZE';
            const outsideSqueezeOffset = clamp(committedSign * Math.min(2.8, roadMargin - 2.4), -roadMargin + 1.0, roadMargin - 1.0);
            this.targetOffset = outsideSqueezeOffset;
            this.committedDefensiveOffset = outsideSqueezeOffset;
          } else {
            this.phase = 'APEX_SHIELD';
            const apexShieldOffset = clamp(committedSign * Math.min(2.8, roadMargin * 0.50), -roadMargin + 1.0, roadMargin - 1.0);
            this.targetOffset = apexShieldOffset;
            this.committedDefensiveOffset = apexShieldOffset;
          }
        }
      }

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

    const isThreatening = threatLevel === 'CRITICAL' || threatLevel === 'HIGH' || (gap < 12.0 && closingSpeed > -0.4) || (gap < 24.0 && closingSpeed > 0.6);
    const shouldDefend = challenger
      && isThreatening
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

    if (isThreatening) {
      this.cooldown = 0;
    }

    const challengerLateral = finite(challenger.otherLateral, finite(challenger.side, 0));
    const lateralDelta = challengerLateral - currentLateral;
    const inDirectTow = Math.abs(lateralDelta) < 1.1 && gap < 45.0;

    let defensiveOffset = nominalBase;
    let phase = 'LOCK_DEFENSIVE_LANE';
    let reason = 'CLAIM_INSIDE_DEFENSIVE_CORRIDOR';

    if (attackerIntent === 'ATTACK_OUTSIDE_MOMENTUM') {
      defensiveOffset = clamp(outsideSign * Math.min(2.8, roadMargin - 2.4), -roadMargin + 1.0, roadMargin - 1.0);
      phase = 'OUTSIDE_DEFENSE_SQUEEZE';
      reason = 'SQUEEZE_OUTSIDE_MOMENTUM_CORRIDOR';
    } else if (inCorner) {
      defensiveOffset = clamp(insideSign * Math.min(3.4, roadMargin * 0.65), -roadMargin + 1.0, roadMargin - 1.0);
      phase = 'APEX_SHIELD';
      reason = 'PHYSICAL_APEX_SHIELDING';
    } else if (gap > 18.0 && inDirectTow && distToCorner > 65) {
      const breakSide = distToCorner < 140 ? insideSign : (currentLateral > 0 ? -1 : 1);
      const breakShift = 2.0 * breakSide;
      defensiveOffset = clamp(currentLateral + breakShift, -roadMargin + 1.0, roadMargin - 1.0);
      phase = 'BREAK_TOW';
      reason = 'AERODYNAMIC_TOW_BREAK';
      this.towBreakTimer = 2.2;
    } else {
      defensiveOffset = clamp(insideSign * Math.min(3.2, roadMargin * 0.60), -roadMargin + 1.0, roadMargin - 1.0);
      phase = 'LOCK_DEFENSIVE_LANE';
      reason = 'PROACTIVE_SHUT_INSIDE_DOOR';
    }

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

        const isThirdPartyBlocker = corridor.blockerId && corridor.blockerId !== challenger.other.id;
        if (corridor.legal && (!isThirdPartyBlocker || corridor.collisionFree)) {
          chosenCandidate = cand;
          break;
        }
      }
    }

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
