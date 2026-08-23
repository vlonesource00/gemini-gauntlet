/**
 * TacticalAttackEngine.js
 * Advanced racecraft offensive maneuvers:
 * - Dynamic divebomb calculations (braking point advantage, apex rights, corner entry limits)
 * - Switchback / cutback counter-tactics against tight inside defenders
 * - Slipstream wake management & slingshot pull-out timing
 * - Aggressive kerb & track boundary utilization
 */

const finite = (value, fallback = 0) => (Number.isFinite(value) ? value : fallback);

const clamp = (value, min, max) => Math.max(min, Math.min(max, value));

const wrap = (value, length) => {
  if (length <= 0) return 0;
  return ((value % length) + length) % length;
};

const ATTACK_PHASES = new Set([
  'ATTACK_LEFT',
  'ATTACK_RIGHT',
  'ATTACK_INSIDE',
  'ATTACK_OUTSIDE',
  'DIVEBOMB',
  'SWITCHBACK'
]);

export class TacticalAttackEngine {
  /**
   * @param {Object} options
   * @param {number} [options.index=1] - Driver index
   * @param {number} [options.aggression=0.75] - Aggression factor (0-1)
   * @param {number} [options.diveMargin=0.6] - Divebomb aggressiveness threshold (0-1)
   * @param {number} [options.kerbUsage=0.8] - Kerb utilization factor (0-1)
   */
  constructor({
    index = 1,
    aggression = 0.75,
    diveMargin = 0.6,
    kerbUsage = 0.8
  } = {}) {
    this.index = index;
    this.aggression = clamp(aggression, 0, 1);
    this.diveMargin = clamp(diveMargin, 0, 1);
    this.kerbUsage = clamp(kerbUsage, 0, 1);
    this.reset();
  }

  reset() {
    this.phase = 'NONE';
    this.targetId = null;
    this.targetOffset = 0;
    this.timer = 0;
    this.age = 0;
    this.cooldown = 0;
    this.draftAge = 0;
    this.side = 0;
    this.lastTargetDelta = 99;
    this.noProgressAge = 0;
    this.intent = null;
    this.passedTargetId = null;
    this.targetLockTime = 0;
    this.divebombActive = false;
    this.switchbackActive = false;
    return this;
  }

  get attacking() {
    return ATTACK_PHASES.has(this.phase) && Boolean(this.targetId);
  }

  setParameters({ aggression, diveMargin, kerbUsage } = {}) {
    if (Number.isFinite(aggression)) this.aggression = clamp(aggression, 0, 1);
    if (Number.isFinite(diveMargin)) this.diveMargin = clamp(diveMargin, 0, 1);
    if (Number.isFinite(kerbUsage)) this.kerbUsage = clamp(kerbUsage, 0, 1);
  }

  _clear(phase = 'NONE', cooldown = 0) {
    this.phase = phase;
    this.targetId = null;
    this.timer = phase === 'RETURN' ? 0.75 : 0;
    this.cooldown = Math.max(this.cooldown, cooldown);
    this.age = 0;
    this.noProgressAge = 0;
    this.intent = null;
    this.divebombActive = false;
    this.switchbackActive = false;
  }

  /**
   * Evaluate slipstream wake strength and slingshot pull-out criteria.
   * @param {Object} vehicle
   * @param {Object} target
   * @returns {Object} Slipstream metrics
   */
  evaluateSlipstream(vehicle, target) {
    if (!target || target.delta <= 0 || target.delta > 45) {
      return { wakeStrength: 0, dragReduction: 0, shouldPullOut: false };
    }

    const lateralOffset = Math.abs(target.side);
    const alignment = clamp(1.0 - lateralOffset / 2.4, 0, 1);
    const distanceFactor = clamp(1.0 - target.delta / 45, 0, 1);
    const wakeStrength = alignment * distanceFactor;

    // Up to 30% drag reduction in direct draft
    const dragReduction = wakeStrength * 0.30;

    // Pull-out criteria: high closing speed & close gap or imminent TTC
    const closingSpeed = target.relativeLongitudinalVelocity;
    const shouldPullOut = (target.delta < 16 && closingSpeed > 1.8)
      || (target.ttc < 1.35 && target.delta < 20);

    return { wakeStrength, dragReduction, shouldPullOut };
  }

  /**
   * Calculate dynamic divebomb feasibility into upcoming braking zone.
   * @param {Object} params
   * @returns {Object} Divebomb evaluation
   */
  evaluateDivebomb({
    vehicle,
    target,
    track,
    nextTurn,
    roadMargin,
    egoGripFactor = 1.0
  }) {
    if (!target || !nextTurn) return { feasible: false };

    const curvature = Math.abs(finite(nextTurn.curvature, 0));
    if (curvature < 0.003) return { feasible: false }; // straight or gentle bend

    const turnSign = Math.sign(finite(nextTurn.turnSign, 1)) || 1;
    const distToCorner = wrap(nextTurn.s - vehicle.distance + track.length * 0.5, track.length) - track.length * 0.5;

    // Divebomb window is relevant when approaching braking zone (15m to 65m ahead)
    if (distToCorner < 12 || distToCorner > 65) return { feasible: false };

    // Baseline braking capabilities
    const classBrakingG = vehicle.classKey === 'prototype' ? 1.45 : vehicle.classKey === 'gt' ? 1.25 : 1.05;
    const egoMaxDecel = classBrakingG * 9.81 * egoGripFactor * (1.0 + this.aggression * 0.15);
    const oppDecelEst = (classBrakingG * 0.88) * 9.81;

    // Corner speed limit at apex
    const apexRadius = 1.0 / Math.max(1e-4, curvature);
    const apexMaxSpeed = Math.sqrt(classBrakingG * 9.81 * apexRadius);

    const egoSpeed = Math.max(5, finite(vehicle.speed, 0));
    const egoBrakingDist = Math.max(0, (egoSpeed * egoSpeed - apexMaxSpeed * apexMaxSpeed) / (2 * egoMaxDecel));

    // Opponent braking point estimate
    const oppSpeed = Math.max(5, finite(target.other.speed, 0));
    const oppBrakingDist = Math.max(0, (oppSpeed * oppSpeed - apexMaxSpeed * apexMaxSpeed) / (2 * oppDecelEst));

    // Divebomb advantage: ego can brake later by (oppBrakingDist - egoBrakingDist)
    const brakingAdvantage = oppBrakingDist - egoBrakingDist;
    const gapToBridge = target.delta;

    // Target inside line for the dive (safely within road boundaries)
    const targetInsideOffset = clamp(
      turnSign * Math.min(3.6, roadMargin * 0.55),
      -roadMargin + 1.2,
      roadMargin - 1.2
    );

    // Feasible if braking advantage + speed advantage covers the delta before apex
    const feasible = (gapToBridge < (18 + this.aggression * 10))
      && (brakingAdvantage + (egoSpeed - oppSpeed) * 0.8 > gapToBridge * 0.45)
      && (this.aggression >= this.diveMargin);

    return {
      feasible,
      insideOffset: targetInsideOffset,
      brakingAdvantage,
      apexSpeed: apexMaxSpeed,
      turnSign
    };
  }

  /**
   * Evaluate switchback / cutback counter-attack against over-defending lead car.
   * @param {Object} params
   * @returns {Object} Switchback evaluation
   */
  evaluateSwitchback({
    vehicle,
    target,
    nextTurn,
    roadMargin
  }) {
    if (!target || !nextTurn) return { feasible: false };

    const curvature = Math.abs(finite(nextTurn.curvature, 0));
    if (curvature < 0.0035) return { feasible: false };

    const turnSign = Math.sign(finite(nextTurn.turnSign, 1)) || 1;
    const leadLateral = finite(target.otherLateral, 0);

    // Defender has committed heavily to the inside line
    const defenderHuggingInside = (leadLateral * turnSign) > (roadMargin * 0.35);
    const speedAdvantage = vehicle.speed - target.other.speed;

    if (defenderHuggingInside && target.delta < 25 && speedAdvantage > -3.0) {
      // Setup wide entry on the outside to square off corner exit
      const outsideEntryOffset = clamp(
        -turnSign * (roadMargin * 0.85),
        -roadMargin,
        roadMargin
      );

      return {
        feasible: true,
        outsideOffset: outsideEntryOffset,
        turnSign,
        exitAdvantage: 0.35 + this.aggression * 0.25
      };
    }

    return { feasible: false };
  }

  /**
   * Main tactical attack update tick.
   * @param {Object} params
   * @returns {Object} Tactical decision and offset directive
   */
  update({
    vehicle,
    track,
    traffic,
    awareness,
    dt,
    aggression = this.aggression,
    policyLine = 0,
    recovering = false,
    pitIntent = null
  }) {
    this.timer = Math.max(0, this.timer - dt);
    this.cooldown = Math.max(0, this.cooldown - dt);
    this.targetLockTime = Math.max(0, this.targetLockTime - dt);
    if (this.targetLockTime <= 0) this.passedTargetId = null;

    const kerbAllowance = this.kerbUsage * Math.min(1.35, finite(track?.curbWidth, 0.8) * 0.9);
    const roadMargin = Math.max(2.1, finite(track?.roadHalfWidth, 6.5) - 1.2 + kerbAllowance);
    const currentLateral = finite(traffic?.current?.lateral, 0);
    const baseOffset = clamp(policyLine, -roadMargin, roadMargin);

    if (pitIntent?.active || recovering) {
      this._clear('NONE');
      return {
        phase: 'NONE',
        desiredOffset: baseOffset,
        target: null,
        corridor: null,
        committed: false,
        kerbAllowance
      };
    }

    // Handle ongoing active attack state
    if (this.attacking) {
      const target = traffic.entries.find((e) => e.other.id === this.targetId) ?? null;
      this.age += dt;

      // Completed pass: target is now behind
      if (!target || target.delta < -4.8) {
        this.passedTargetId = this.targetId;
        this.targetLockTime = 16.0;
        this._clear('RETURN', 0.8);
        return {
          phase: 'RETURN',
          desiredOffset: baseOffset,
          target,
          corridor: null,
          committed: false,
          kerbAllowance
        };
      }

      const targetSpeed = Math.max(vehicle.speed, target.other.speed + 7.5);
      const corridor = awareness.evaluateCorridor({
        vehicle,
        track,
        traffic,
        terminalOffset: this.targetOffset,
        targetId: this.targetId,
        targetSpeed,
        kerbAllowance
      });

      const lateralPassClear = Math.abs(currentLateral - target.otherLateral) >= 3.2;
      if (lateralPassClear && target.relativeLongitudinalVelocity < 0.12) {
        this.noProgressAge += dt;
      } else {
        this.noProgressAge = Math.max(0, this.noProgressAge - dt * 2.0);
      }
      this.lastTargetDelta = target.delta;

      const targetEnvelopeAllowed = Boolean(this.intent?.straightSend)
        && corridor.legal
        && corridor.targetSeparationM >= 3.4
        && corridor.minimumClearanceM >= finite(this.intent?.safetyThresholdM, -0.15)
        && corridor.blockerTimeS > 0.85;

      const staticTargetEscape = target.other.speed < 6.0
        && corridor.legal
        && corridor.targetSeparationM >= 3.4
        && corridor.blockerId === this.targetId;

      const newlyUnsafe = !corridor.collisionFree && !targetEnvelopeAllowed && !staticTargetEscape;
      const maxAttackAge = target.other.speed < 3.0 ? 14.0 : 8.5;

      if (this.age > maxAttackAge || this.noProgressAge > 2.2 || newlyUnsafe) {
        // If blocked by defender, immediately clear commitment without long cooldown so alternative side can be taken
        this._clear('RETURN', newlyUnsafe ? 0.05 : 0.6);
        return {
          phase: 'RETURN',
          desiredOffset: baseOffset,
          target,
          corridor,
          committed: false,
          abortReason: newlyUnsafe ? 'BLOCKED_BY_DEFENDER' : 'NO_PROGRESS',
          kerbAllowance
        };
      }

      return {
        phase: this.phase,
        desiredOffset: this.targetOffset,
        target,
        corridor,
        committed: true,
        straightSend: Boolean(this.intent?.straightSend),
        safetyThresholdM: finite(this.intent?.safetyThresholdM, 0),
        predictedTimeGainS: finite(this.intent?.predictedTimeGainS, 0),
        divebombing: this.divebombActive,
        switchbacking: this.switchbackActive,
        kerbAllowance
      };
    }

    // Select primary overtake target ahead
    const target = traffic.entries
      .filter((e) => e.other.id !== this.passedTargetId
        && e.delta > 0 && e.delta < 58 && e.longitudinal > -1.5
        && !(e.other.aiTactical?.passTargetId === vehicle.id && e.delta < 8)
        && Math.abs(e.side) < roadMargin * 2 + 1.0)
      .sort((a, b) => a.delta - b.delta)[0] ?? null;

    if (!target || this.cooldown > 0) {
      this.draftAge = 0;
      return {
        phase: this.phase === 'RETURN' ? 'RETURN' : 'NONE',
        desiredOffset: baseOffset,
        target,
        corridor: null,
        committed: false,
        kerbAllowance
      };
    }

    // Analyze upcoming turn geometry
    const turns = [18, 36, 56].map((dist) =>
      track?.atDistance ? track.atDistance(vehicle.distance + dist) : { curvature: 0, turnSign: 1, s: vehicle.distance + dist }
    );
    const turn = turns.sort((a, b) => Math.abs(b.curvature) - Math.abs(a.curvature))[0];
    const turnCurvature = Math.abs(finite(turn?.curvature, 0));
    const inCorner = turnCurvature >= 0.003;
    const turnSign = Math.sign(finite(turn?.turnSign, 1)) || 1;

    // 1. Slipstream check
    const slipstream = this.evaluateSlipstream(vehicle, target);
    if (slipstream.wakeStrength > 0.2) {
      this.draftAge += dt;
    } else {
      this.draftAge = Math.max(0, this.draftAge - dt);
    }

    // Calculate available track space to the left and right of the lead vehicle
    const leadLateral = finite(target.otherLateral, 0);
    const spaceOnLeft = leadLateral - (-roadMargin); // Distance from left boundary to target
    const spaceOnRight = roadMargin - leadLateral;   // Distance from target to right boundary

    // 2. Dynamic Divebomb & Switchback checks in corners
    const divebomb = inCorner ? this.evaluateDivebomb({
      vehicle,
      target,
      track,
      nextTurn: turn,
      roadMargin,
      egoGripFactor: 1.0 + this.kerbUsage * 0.08
    }) : { feasible: false };

    const switchback = inCorner ? this.evaluateSwitchback({
      vehicle,
      target,
      nextTurn: turn,
      roadMargin
    }) : { feasible: false };

    const targetSpeed = Math.max(vehicle.speed, target.other.speed + 7.5);
    const candidates = [];

    // Left Attack Lane Candidate
    if (spaceOnLeft >= 2.6) {
      const isLeftInside = inCorner && turnSign < 0;
      const isLeftDive = isLeftInside && divebomb.feasible;
      const isLeftSwitch = !isLeftInside && inCorner && switchback.feasible;

      let leftAttackOffset = clamp(leadLateral - 4.2, -roadMargin + 0.65, roadMargin - 0.65);
      if (leadLateral > 0) {
        leftAttackOffset = Math.min(-2.4, Math.max(-roadMargin * 0.75, leadLateral - spaceOnLeft * 0.72));
      } else if (spaceOnLeft >= 3.6) {
        leftAttackOffset = clamp(leadLateral - Math.max(3.8, spaceOnLeft * 0.68), -roadMargin + 0.65, roadMargin - 0.65);
      }

      const leftTargetOffset = isLeftDive
        ? divebomb.insideOffset
        : (isLeftSwitch ? switchback.outsideOffset : leftAttackOffset);

      const leftCorridor = awareness.evaluateCorridor({
        vehicle,
        track,
        traffic,
        terminalOffset: leftTargetOffset,
        targetId: target.other.id,
        targetSpeed,
        kerbAllowance
      });

      if (leftCorridor.targetSeparationM >= 2.8 && leftCorridor.legal) {
        let phase = 'ATTACK_LEFT';
        if (inCorner) {
          phase = isLeftDive ? 'DIVEBOMB' : (isLeftInside ? 'ATTACK_INSIDE' : (isLeftSwitch ? 'SWITCHBACK' : 'ATTACK_OUTSIDE'));
        }
        const timeGain = (90 / Math.max(4, target.other.speed)) - (90 / Math.max(4, targetSpeed));
        const spaceAdvantage = (spaceOnLeft - spaceOnRight) * 1.2;
        const isSqueezed = spaceOnLeft < 4.0 && spaceOnRight >= 4.6;
        const isOpenSweep = spaceOnLeft >= 4.8 && spaceOnRight < 4.0;
        const score = leftCorridor.minimumClearanceM * 1.8
          + timeGain * 2.5
          + spaceAdvantage
          + (isLeftDive ? 2.0 + this.aggression * 1.2 : (isLeftInside ? 0.8 : 0.4))
          - (isSqueezed ? 3.0 : 0)
          + (isOpenSweep ? 2.0 : 0);

        candidates.push({
          phase,
          side: -1,
          offset: leftTargetOffset,
          corridor: leftCorridor,
          score: leftCorridor.collisionFree ? score : score - 200,
          timeGainS: timeGain,
          isDive: isLeftDive,
          isSwitchback: isLeftSwitch
        });
      }
    }

    // Right Attack Lane Candidate
    if (spaceOnRight >= 2.6) {
      const isRightInside = inCorner && turnSign > 0;
      const isRightDive = isRightInside && divebomb.feasible;
      const isRightSwitch = !isRightInside && inCorner && switchback.feasible;

      let rightAttackOffset = clamp(leadLateral + 4.2, -roadMargin + 0.65, roadMargin - 0.65);
      if (leadLateral < 0) {
        rightAttackOffset = Math.max(2.4, Math.min(roadMargin * 0.75, leadLateral + spaceOnRight * 0.72));
      } else if (spaceOnRight >= 3.6) {
        rightAttackOffset = clamp(leadLateral + Math.max(3.8, spaceOnRight * 0.68), -roadMargin + 0.65, roadMargin - 0.65);
      }

      const rightTargetOffset = isRightDive
        ? divebomb.insideOffset
        : (isRightSwitch ? switchback.outsideOffset : rightAttackOffset);

      const rightCorridor = awareness.evaluateCorridor({
        vehicle,
        track,
        traffic,
        terminalOffset: rightTargetOffset,
        targetId: target.other.id,
        targetSpeed,
        kerbAllowance
      });

      if (rightCorridor.targetSeparationM >= 2.8 && rightCorridor.legal) {
        let phase = 'ATTACK_RIGHT';
        if (inCorner) {
          phase = isRightDive ? 'DIVEBOMB' : (isRightInside ? 'ATTACK_INSIDE' : (isRightSwitch ? 'SWITCHBACK' : 'ATTACK_OUTSIDE'));
        }
        const timeGain = (90 / Math.max(4, target.other.speed)) - (90 / Math.max(4, targetSpeed));
        const spaceAdvantage = (spaceOnRight - spaceOnLeft) * 1.2;
        const isSqueezed = spaceOnRight < 4.0 && spaceOnLeft >= 4.6;
        const isOpenSweep = spaceOnRight >= 4.8 && spaceOnLeft < 4.0;
        const score = rightCorridor.minimumClearanceM * 1.8
          + timeGain * 2.5
          + spaceAdvantage
          + (isRightDive ? 2.0 + this.aggression * 1.2 : (isRightInside ? 0.8 : 0.4))
          - (isSqueezed ? 3.0 : 0)
          + (isOpenSweep ? 2.0 : 0);

        candidates.push({
          phase,
          side: 1,
          offset: rightTargetOffset,
          corridor: rightCorridor,
          score: rightCorridor.collisionFree ? score : score - 200,
          timeGainS: timeGain,
          isDive: isRightDive,
          isSwitchback: isRightSwitch
        });
      }
    }

    candidates.sort((a, b) => b.score - a.score);
    const chosen = candidates[0] ?? null;

    const isSlowObstacle = target.other.speed < 18.0 || (target.delta < 28.0 && target.relativeLongitudinalVelocity < -3.5);

    const attackRange = target.other.speed < vehicle.speed * 0.8
      ? 48
      : 34 + aggression * 8;

    const straightSend = !inCorner && (chosen?.timeGainS ?? 0) > 0.05;

    if (chosen && (target.delta < attackRange || straightSend || slipstream.shouldPullOut || isSlowObstacle)) {
      this.phase = chosen.phase;
      this.targetId = target.other.id;
      this.targetOffset = chosen.offset;
      this.side = chosen.side;
      this.age = 0;
      this.noProgressAge = 0;
      this.divebombActive = chosen.isDive;
      this.switchbackActive = chosen.isSwitchback;
      this.intent = {
        targetId: this.targetId,
        side: this.side,
        lane: chosen.phase,
        gapM: target.delta,
        predictedTimeGainS: chosen.timeGainS,
        straightSend,
        safetyThresholdM: vehicle.classKey === 'prototype' ? -0.22 : -0.15,
        targetClosingSpeed: 5.0 + target.delta * 0.15,
        commitmentDuration: target.other.speed < 2.5 ? 14.0 : 8.5
      };

      return {
        phase: this.phase,
        desiredOffset: this.targetOffset,
        target,
        corridor: chosen.corridor,
        committed: true,
        straightSend,
        safetyThresholdM: this.intent.safetyThresholdM,
        predictedTimeGainS: chosen.timeGainS,
        divebombing: this.divebombActive,
        switchbacking: this.switchbackActive,
        kerbAllowance
      };
    }

    // If target is slow or stopped, NEVER fall into DRAFT behind it! Force immediate open flank evasion pass!
    if (isSlowObstacle) {
      const openSide = spaceOnRight >= spaceOnLeft ? 1 : -1;
      const evasionOffset = openSide > 0
        ? clamp(leadLateral + Math.max(3.4, spaceOnRight * 0.65), -roadMargin + 0.6, roadMargin - 0.6)
        : clamp(leadLateral - Math.max(3.4, spaceOnLeft * 0.65), -roadMargin + 0.6, roadMargin - 0.6);

      this.phase = openSide > 0 ? 'ATTACK_RIGHT' : 'ATTACK_LEFT';
      this.targetId = target.other.id;
      this.targetOffset = evasionOffset;
      this.side = openSide;
      this.age = 0;
      this.noProgressAge = 0;
      this.divebombActive = false;
      this.switchbackActive = false;
      this.intent = {
        targetId: this.targetId,
        side: this.side,
        lane: this.phase,
        gapM: target.delta,
        predictedTimeGainS: 1.8,
        straightSend: true,
        safetyThresholdM: -0.35,
        targetClosingSpeed: 8.0,
        commitmentDuration: 12.0
      };

      return {
        phase: this.phase,
        desiredOffset: this.targetOffset,
        target,
        corridor: null,
        committed: true,
        straightSend: true,
        safetyThresholdM: -0.35,
        predictedTimeGainS: 1.8,
        divebombing: false,
        switchbacking: false,
        kerbAllowance,
        reason: 'OBSTACLE_EVASION_OVERTAKE'
      };
    }

    // Default to drafting in tow if no clear attack corridor yet
    this.phase = 'DRAFT';
    return {
      phase: 'DRAFT',
      desiredOffset: target.otherLateral,
      target,
      corridor: null,
      committed: false,
      kerbAllowance,
      reason: 'DRAFTING_IN_WAKE'
    };
  }
}

export { ATTACK_PHASES };
