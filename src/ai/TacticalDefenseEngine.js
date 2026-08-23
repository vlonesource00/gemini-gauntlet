/**
 * TacticalDefenseEngine.js
 * High-Performance Motorsport Tactical Defense Engine:
 * - Multi-tier threat assessment & closing speed estimation
 * - FIA-compliant single-move straightaway defense
 * - Aerodynamic tow breaking (stepped lateral shifts on straights)
 * - Defensive lane locking (inside corridor protection before braking)
 * - Apex radius shielding against divebombs & cutbacks
 * - Defensive ERS deployment triggers (straightaway burst & exit launch)
 */

const finite = (value, fallback = 0) => (Number.isFinite(value) ? value : fallback);

const clamp = (value, min, max) => Math.max(min, Math.min(max, value));

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
    this.oneMoveLocked = false;
    this.timer = 0;
    this.age = 0;
    this.cooldown = 0;
    this.towBreakTimer = 0;
    this.ersDefensiveDeployTimer = 0;
    this.ersDefensiveReason = 'NONE';
    this.lastDefendedCorner = false;
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
    this.timer = phase === 'RETURN_PACE' ? 0.65 : 0;
    this.cooldown = Math.max(this.cooldown, cooldown);
    this.age = 0;
    this.towBreakTimer = 0;
    this.intent = null;
  }

  /**
   * Assess threat severity of rear challengers.
   * @param {Object} params
   * @returns {Object} Threat evaluation
   */
  assessThreat({ vehicle, traffic }) {
    if (!traffic?.entries?.length) {
      return { challenger: null, threatLevel: 'NONE', closingSpeed: 0, ttc: 99, gap: 99 };
    }

    const roadMargin = 6.5;
    const challenger = traffic.entries
      .filter((e) => e.delta < -1.8 && e.delta > -50.0
        && e.longitudinal < 2.5 && Math.abs(e.side) < roadMargin * 2 + 1.0)
      .sort((a, b) => b.delta - a.delta)[0] ?? null;

    if (!challenger) {
      return { challenger: null, threatLevel: 'NONE', closingSpeed: 0, ttc: 99, gap: 99 };
    }

    const closingSpeed = challenger.otherForwardSpeed - traffic.egoForwardSpeed;
    const bodyGap = Math.abs(challenger.longitudinal) - 5.4 * 0.5;
    const ttc = closingSpeed > 0.2 ? Math.max(0, bodyGap) / closingSpeed : 99;
    const gap = Math.abs(challenger.delta);

    let threatLevel = 'NONE';
    if (gap < 15.0 && (ttc < 1.8 || closingSpeed > 1.8)) {
      threatLevel = 'CRITICAL';
    } else if (gap < 26.0 && (ttc < 3.0 || closingSpeed > 1.0)) {
      threatLevel = 'HIGH';
    } else if (gap < 42.0 && closingSpeed > 0.4) {
      threatLevel = 'MEDIUM';
    } else if (gap < 50.0) {
      threatLevel = 'LOW';
    }

    return { challenger, threatLevel, closingSpeed, ttc, gap };
  }

  /**
   * Main tactical defense update loop.
   * @param {Object} params
   * @returns {Object} Defensive directives
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
        ersDeployRequested: false,
        reason: 'RECOVERING'
      };
    }

    const { challenger, threatLevel, closingSpeed, ttc, gap } = this.assessThreat({ vehicle, traffic });
    this.threatLevel = threatLevel;

    // Track geometry ahead
    const turnSamples = [18, 38, 65].map((d) =>
      track?.atDistance ? track.atDistance(vehicle.distance + d) : { curvature: 0, turnSign: 1, s: vehicle.distance + d }
    );
    const turn = turnSamples.sort((a, b) => Math.abs(b.curvature) - Math.abs(a.curvature))[0];
    const turnCurvature = Math.abs(finite(turn?.curvature, 0));
    const turnSign = Math.sign(finite(turn?.turnSign, 1)) || 1;
    const inCorner = turnCurvature > 0.0035;
    const distToCorner = wrap((turn?.s ?? vehicle.distance) - vehicle.distance + (track?.length || 1000) * 0.5, track?.length || 1000) - (track?.length || 1000) * 0.5;

    // Evaluate Corner-Exit Defensive ERS Launch trigger
    if (!inCorner && this.lastDefendedCorner && vehicle.speed < 48.0 && vehicle.controls.throttle > 0.75 && gap < 20.0) {
      this.ersDefensiveDeployTimer = 1.8;
      this.ersDefensiveReason = 'CORNER_EXIT_LAUNCH';
      this.lastDefendedCorner = false;
    }
    if (inCorner) {
      this.lastDefendedCorner = this.defending;
    }

    // Handle ongoing defense state
    if (this.defending) {
      const activeChallenger = traffic.entries.find((e) => e.other.id === this.defenseTargetId) ?? null;
      this.age += dt;

      const passed = activeChallenger && activeChallenger.delta > 4.5;
      const gone = !activeChallenger || activeChallenger.delta < -45;

      const maxDefenseAge = 12.0;
      if (passed || gone || this.age > maxDefenseAge) {
        this._clear('RETURN_PACE', 0.6);
        return {
          phase: 'RETURN_PACE',
          desiredOffset: this.targetOffset,
          defending: false,
          target: null,
          threatLevel: 'NONE',
          ersDeployRequested: false,
          reason: passed ? 'OPPONENT_PASSED' : 'DEFENSE_COMPLETE'
        };
      }

      // Straightaway ERS Defense Burst if challenger is surging
      if (!inCorner && closingSpeed > 2.5 && gap < 22.0 && vehicle.controls.throttle > 0.85) {
        this.ersDefensiveDeployTimer = 2.0;
        this.ersDefensiveReason = 'STRAIGHT_COUNTER_BURST';
      }

      // Transition from BREAK_TOW into LOCK_DEFENSIVE_LANE approaching braking zone
      if (this.phase === 'BREAK_TOW' && (distToCorner < 55 || gap < 18.0 || this.towBreakTimer <= 0)) {
        const insideOffset = clamp(turnSign * Math.min(3.2, roadMargin * 0.62), -roadMargin + 0.6, roadMargin - 0.6);
        this.phase = 'LOCK_DEFENSIVE_LANE';
        this.targetOffset = insideOffset;
        this.oneMoveLocked = true;
      }

      // Transition from LOCK_DEFENSIVE_LANE to DEFEND_INSIDE in corner
      if (this.phase === 'LOCK_DEFENSIVE_LANE' && inCorner) {
        this.phase = 'DEFEND_INSIDE';
        this.targetOffset = clamp(turnSign * Math.min(3.4, roadMargin * 0.68), -roadMargin + 0.6, roadMargin - 0.6);
      }

      return {
        phase: this.phase,
        desiredOffset: this.targetOffset,
        defending: true,
        target: activeChallenger,
        threatLevel: this.threatLevel,
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
        ersDeployRequested: this.ersDefensiveDeployTimer > 0,
        ersDeployReason: this.ersDefensiveReason,
        reason: 'SAFE_REJOIN_RACING_LINE'
      };
    }

    // Evaluate New Defensive Trigger
    const defenseTriggerThreshold = (1.2 - this.defenseReactivity * 0.5);
    const shouldDefend = challenger
      && (threatLevel === 'CRITICAL' || threatLevel === 'HIGH' || (threatLevel === 'MEDIUM' && closingSpeed > defenseTriggerThreshold))
      && traffic.egoForwardSpeed > 6.0
      && this.cooldown <= 0;

    if (!shouldDefend) {
      return {
        phase: 'NONE',
        desiredOffset: nominalBase,
        defending: false,
        target: challenger,
        threatLevel,
        ersDeployRequested: this.ersDefensiveDeployTimer > 0,
        ersDeployReason: this.ersDefensiveReason,
        reason: 'NO_DEFENSE_NEEDED'
      };
    }

    // Relative challenger offset
    const challengerLateral = finite(challenger.otherLateral, finite(challenger.side, 0));
    const lateralDelta = challengerLateral - currentLateral;
    const inDirectTow = Math.abs(lateralDelta) < 1.0 && gap < 42.0;

    let defensiveOffset = nominalBase;
    let phase = 'LOCK_DEFENSIVE_LANE';
    let reason = 'CLAIM_INSIDE_DEFENSIVE_CORRIDOR';

    if (inCorner) {
      // 1. In corner: secure inside apex line against divebombs
      defensiveOffset = clamp(turnSign * Math.min(3.4, roadMargin * 0.68), -roadMargin + 0.6, roadMargin - 0.6);
      phase = 'DEFEND_INSIDE';
      reason = 'PROTECT_INSIDE_APEX_LINE';
    } else if (gap > 18.0 && inDirectTow && distToCorner > 55) {
      // 2. Early straightaway tow break: stepped lateral shift off draft line
      const breakSide = currentLateral > 0 ? -1 : 1;
      const breakShift = 2.4 * breakSide;
      defensiveOffset = clamp(currentLateral + breakShift, -roadMargin + 0.8, roadMargin - 0.8);
      phase = 'BREAK_TOW';
      reason = 'AERODYNAMIC_TOW_BREAK';
      this.towBreakTimer = 2.2;
    } else {
      // 3. Straightaway corridor locking: claim inside line before braking zone
      const insideOffset = clamp(turnSign * Math.min(3.2, roadMargin * 0.62), -roadMargin + 0.6, roadMargin - 0.6);
      defensiveOffset = insideOffset;
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
        this.oneMoveLocked = true;
        this.age = 0;

        // Straightaway ERS Defense Burst if threatened
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
      ersDeployRequested: false,
      reason: 'DEFENSE_PATH_UNAVAILABLE'
    };
  }
}
