/**
 * TacticalAttackEngine.js
 * Modular Motorsport Combat Attack, Overtaking & Racecraft Intelligence:
 * - Opportunistic Dual-Flank Attacks (Continuous Left vs Right Corridor Scoring)
 * - Inside vs Outside Tactical Decision-Making (Inside Apex Claim vs Outside Momentum Carry)
 * - High-Speed Slingshot (+18.0 m/s closing speed floor with ERS boost & tow pull-out)
 * - Late-Braking Apex Divebombs (-3.5G decel model claiming apex rights)
 * - Diamond Switchback / Cutback Undercut against inside-overslowing defenders
 * - Side-by-Side Combat Rubbing Resilience (< 0.4m contact acceptance)
 */

import { clamp, wrap, saturate } from '../core/math.js';

const finite = (value, fallback = 0) => (Number.isFinite(value) ? value : fallback);

const ATTACK_PHASES = new Set([
  'ATTACK_LEFT',
  'ATTACK_RIGHT',
  'ATTACK_INSIDE',
  'ATTACK_OUTSIDE',
  'DIVEBOMB',
  'SWITCHBACK',
  'SLINGSHOT'
]);

export class TacticalAttackEngine {
  /**
   * @param {Object} [options]
   * @param {number} [options.index=1] - Driver index
   * @param {number} [options.aggression=0.88] - Aggression factor (0-1)
   * @param {number} [options.diveMargin=0.45] - Divebomb threshold factor (0-1)
   * @param {number} [options.kerbUsage=0.90] - Kerb utilization factor (0-1)
   * @param {number} [options.rubbingTolerance=0.40] - Rubbing & contact tolerance in meters
   * @param {number} [options.closingSpeedFloor=18.0] - Straightaway slingshot closing velocity floor in m/s (+65 km/h)
   */
  constructor({
    index = 1,
    aggression = 0.88,
    diveMargin = 0.45,
    kerbUsage = 0.90,
    rubbingTolerance = 0.40,
    closingSpeedFloor = 18.0
  } = {}) {
    this.index = index;
    this.aggression = clamp(aggression, 0, 1);
    this.diveMargin = clamp(diveMargin, 0, 1);
    this.kerbUsage = clamp(kerbUsage, 0, 1);
    this.rubbingTolerance = clamp(rubbingTolerance, 0, 1.5);
    this.closingSpeedFloor = Math.max(12.0, closingSpeedFloor);
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
    this.switchbackStage = 'NONE';
    this.contactRubbingActive = false;
    this.ersAttackActive = false;
    return this;
  }

  get attacking() {
    return ATTACK_PHASES.has(this.phase) && Boolean(this.targetId);
  }

  setParameters({
    aggression,
    diveMargin,
    kerbUsage,
    rubbingTolerance,
    closingSpeedFloor
  } = {}) {
    if (Number.isFinite(aggression)) this.aggression = clamp(aggression, 0, 1);
    if (Number.isFinite(diveMargin)) this.diveMargin = clamp(diveMargin, 0, 1);
    if (Number.isFinite(kerbUsage)) this.kerbUsage = clamp(kerbUsage, 0, 1);
    if (Number.isFinite(rubbingTolerance)) this.rubbingTolerance = clamp(rubbingTolerance, 0, 1.5);
    if (Number.isFinite(closingSpeedFloor)) this.closingSpeedFloor = Math.max(12.0, closingSpeedFloor);
  }

  _clear(phase = 'NONE', cooldown = 0) {
    this.phase = phase;
    this.targetId = null;
    this.timer = phase === 'RETURN' ? 0.45 : 0;
    this.cooldown = Math.max(this.cooldown, cooldown);
    this.age = 0;
    this.noProgressAge = 0;
    this.intent = null;
    this.divebombActive = false;
    this.switchbackActive = false;
    this.switchbackStage = 'NONE';
    this.contactRubbingActive = false;
    this.ersAttackActive = false;
  }

  /**
   * Evaluate slipstream wake strength, drag reduction, and slingshot pull-out criteria.
   * @param {Object} vehicle
   * @param {Object} target
   * @returns {Object} Slipstream metrics
   */
  evaluateSlipstream(vehicle, target) {
    if (!target || target.delta <= 0 || target.delta > 55) {
      return { wakeStrength: 0, dragReduction: 0, shouldPullOut: false, pullOutVelocityFloor: this.closingSpeedFloor };
    }

    const lateralOffset = Math.abs(target.side);
    const alignment = clamp(1.0 - lateralOffset / 2.8, 0, 1);
    const distanceFactor = clamp(1.0 - target.delta / 55, 0, 1);
    const wakeStrength = alignment * distanceFactor;

    // Up to 38% aerodynamic drag reduction in direct slipstream wake
    const dragReduction = wakeStrength * 0.38;

    // Slingshot pull-out timing: ride tow to accumulate delta, then punch out into clean air
    const closingSpeed = target.relativeLongitudinalVelocity;
    const dynamicPulloutDist = clamp(closingSpeed * 1.15 + 4.8, 6.5, 28.0);
    const shouldPullOut = (target.delta <= dynamicPulloutDist && closingSpeed > 0.4)
      || (target.ttc < 1.8 && target.delta < 32)
      || (target.delta < 8.0);

    return {
      wakeStrength,
      dragReduction,
      shouldPullOut,
      pullOutVelocityFloor: this.closingSpeedFloor
    };
  }

  /**
   * Calculate late-braking inside divebomb feasibility into upcoming corner.
   * Assumes -3.5G peak braking deceleration with ground-effect aero downforce.
   * @param {Object} params
   * @returns {Object} Divebomb evaluation
   */
  evaluateDivebomb({
    vehicle,
    target,
    track,
    nextTurn,
    roadMargin,
    egoGripFactor = 1.0,
    kerbAllowance = 0
  }) {
    if (!target || !nextTurn) return { feasible: false };

    const curvature = Math.abs(finite(nextTurn.curvature, 0));
    if (curvature < 0.0025) return { feasible: false };

    const turnSign = Math.sign(finite(nextTurn.turnSign, 1)) || 1;
    const trackLen = Math.max(1, finite(track?.length, 1000));
    const distToCorner = wrap(nextTurn.s - vehicle.distance + trackLen * 0.5, trackLen) - trackLen * 0.5;

    // Divebomb window is active when approaching braking zone (8m to 90m ahead)
    if (distToCorner < 8 || distToCorner > 90) return { feasible: false };

    // Fearless -3.5G peak braking deceleration
    const diveDecelG = Math.max(3.4, (vehicle.classKey === 'prototype' ? 3.6 : 3.4) * egoGripFactor * (0.95 + this.aggression * 0.20));
    const egoMaxDecel = diveDecelG * 9.81;
    const oppDecelEst = 1.25 * 9.81 * egoGripFactor;

    // Corner speed limit at apex
    const apexRadius = 1.0 / Math.max(1e-4, curvature);
    const apexLateralG = (vehicle.classKey === 'prototype' ? 2.4 : 1.85) * egoGripFactor * (1.0 + this.kerbUsage * 0.15);
    const apexMaxSpeed = Math.sqrt(apexLateralG * 9.81 * apexRadius);

    const egoSpeed = Math.max(5, finite(vehicle.speed, 0));
    const egoBrakingDist = Math.max(0, (egoSpeed * egoSpeed - apexMaxSpeed * apexMaxSpeed) / (2 * egoMaxDecel));

    const oppSpeed = Math.max(5, finite(target.other.speed, 0));
    const oppBrakingDist = Math.max(0, (oppSpeed * oppSpeed - apexMaxSpeed * apexMaxSpeed) / (2 * oppDecelEst));

    const brakingAdvantage = oppBrakingDist - egoBrakingDist;
    const gapToBridge = target.delta;

    // Target inside line for the dive (inside = -turnSign)
    const targetInsideOffset = clamp(
      -turnSign * Math.min(3.4, roadMargin * 0.60),
      -roadMargin + 0.65,
      roadMargin - 0.65
    );

    const diveRange = 28 + this.aggression * 18;
    const speedAdvantage = egoSpeed - oppSpeed;
    const feasible = (gapToBridge < diveRange)
      && (brakingAdvantage + speedAdvantage * 0.95 > gapToBridge * 0.25)
      && (this.aggression >= this.diveMargin * 0.70);

    return {
      feasible,
      insideOffset: targetInsideOffset,
      brakingAdvantage,
      apexSpeed: apexMaxSpeed,
      diveDecelG,
      egoBrakingDist,
      oppBrakingDist,
      distToCorner,
      turnSign
    };
  }

  /**
   * Evaluate switchback / cutback counter-attack against over-defending lead car.
   * Detects when defender overslows on inside and executes sharp late-apex diamond undercut.
   * @param {Object} params
   * @returns {Object} Switchback evaluation
   */
  evaluateSwitchback({
    vehicle,
    target,
    track,
    nextTurn,
    roadMargin,
    kerbAllowance = 0
  }) {
    if (!target || !nextTurn) return { feasible: false };

    const curvature = Math.abs(finite(nextTurn.curvature, 0));
    if (curvature < 0.0035) return { feasible: false };

    const turnSign = Math.sign(finite(nextTurn.turnSign, 1)) || 1;
    const trackLen = Math.max(1, finite(track?.length, 1000));
    const distToCorner = wrap(nextTurn.s - vehicle.distance + trackLen * 0.5, trackLen) - trackLen * 0.5;

    if (distToCorner < 8 || distToCorner > 55) return { feasible: false };

    const leadLateral = finite(target.otherLateral, 0);

    // Defender has committed heavily to the inside line (inside = -turnSign)
    const defenderHuggingInside = (leadLateral * (-turnSign)) > (roadMargin * 0.30);
    const speedAdvantage = vehicle.speed - target.other.speed;
    const closingSpeed = target.relativeLongitudinalVelocity;

    // Detect defender overslowing on inside entry / apex
    const apexRadius = 1.0 / Math.max(1e-4, curvature);
    const estApexSpeed = Math.sqrt(1.5 * 9.81 * apexRadius);
    const defenderOverslowed = target.other.speed < estApexSpeed * 0.88 || (speedAdvantage > 3.0 && closingSpeed > 2.0);

    if (defenderHuggingInside && defenderOverslowed && target.delta < 28) {
      // 1. Setup wide entry on the outside (+turnSign) to square off corner entry
      const outsideEntryOffset = clamp(
        turnSign * Math.min(3.2, roadMargin * 0.55),
        -3.6,
        3.6
      );

      // 2. Sharp late-apex diamond undercut offset cutting across inside exit (-turnSign)
      const undercutApexOffset = clamp(
        -turnSign * Math.min(2.4, roadMargin * 0.42),
        -3.6,
        3.6
      );

      return {
        feasible: true,
        outsideOffset: outsideEntryOffset,
        undercutOffset: undercutApexOffset,
        turnSign,
        exitAdvantage: 0.65 + this.aggression * 0.35,
        isDiamondUndercut: true
      };
    }

    return { feasible: false };
  }

  /**
   * Main tactical attack update loop.
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

    const roadHalfWidth = finite(track?.roadHalfWidth, 6.5);
    const baseRoadMargin = Math.max(2.1, Math.min(5.2, roadHalfWidth - 1.8));
    const kerbAllowance = this.kerbUsage * Math.min(0.80, finite(track?.curbWidth, 0.8) * 0.60);
    const roadMargin = baseRoadMargin + kerbAllowance;
    const currentLateral = finite(traffic?.current?.lateral, 0);
    const baseOffset = clamp(policyLine, -baseRoadMargin, baseRoadMargin);

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

    // Analyze current and upcoming turn geometry across wide horizon
    const turns = [0, 8, 16, 32, 50, 75, 105, 140].map((dist) =>
      track?.atDistance ? track.atDistance(vehicle.distance + dist) : { curvature: 0, turnSign: 1, s: vehicle.distance + dist }
    );
    const turn = turns.sort((a, b) => Math.abs(b.curvature) - Math.abs(a.curvature))[0];
    const turnCurvature = Math.abs(finite(turn?.curvature, 0));
    const inCorner = turnCurvature >= 0.003;
    const distToCorner = Math.max(0, wrap((turn?.s ?? vehicle.distance) - vehicle.distance + (track?.length || 1000) * 0.5, track?.length || 1000) - (track?.length || 1000) * 0.5);
    const turnSign = Math.sign(finite(turn?.turnSign, 1)) || 1;

    // Handle ongoing active attack state
    if (this.attacking) {
      const target = traffic.entries.find((e) => e.other.id === this.targetId) ?? null;
      this.age += dt;

      // Completed pass: target is now cleared behind
      const isPastLeadingEdge = target && target.delta < -2.2;
      const passCompleted = !target || target.delta < -3.5 || (isPastLeadingEdge && this.age > 1.2);
      if (passCompleted) {
        this.passedTargetId = this.targetId;
        this.targetLockTime = 16.0;
        this._clear('RETURN', 0.5);
        return {
          phase: 'RETURN',
          desiredOffset: baseOffset,
          target,
          corridor: null,
          committed: false,
          kerbAllowance
        };
      }

      // Check side-by-side status & rubbing contact acceptance (< 0.4m separation)
      const isSideBySide = Math.abs(target.delta) < 6.5;
      const actualSeparation = Math.abs(currentLateral - finite(target.otherLateral, 0));
      const isRubbingContact = isSideBySide && actualSeparation < 1.8;
      this.contactRubbingActive = isRubbingContact;

      // If switchbacking, progress from wide entry to sharp diamond undercut at apex
      if (this.switchbackActive && distToCorner < 22 && inCorner) {
        const switchbackDecision = this.evaluateSwitchback({
          vehicle,
          target,
          track,
          nextTurn: turn,
          roadMargin,
          kerbAllowance
        });
        if (switchbackDecision.feasible && switchbackDecision.undercutOffset != null) {
          this.targetOffset = switchbackDecision.undercutOffset;
          this.switchbackStage = 'APEX_UNDERCUT';
        }
      }

      // When attacking inside or divebombing through a corner sequence, adapt target offset to the active turn sign
      if ((this.phase === 'ATTACK_INSIDE' || this.divebombActive) && inCorner) {
        const activeTurnSign = Math.sign(finite(turn?.turnSign, 1)) || 1;
        const dynamicInsideOffset = clamp(-activeTurnSign * Math.min(3.2, baseRoadMargin * 0.65), -baseRoadMargin + 0.8, baseRoadMargin - 0.8);
        this.targetOffset = dynamicInsideOffset;
      }

      const targetSpeed = Math.max(vehicle.speed, target.other.speed + 18.0);
      const corridor = awareness.evaluateCorridor({
        vehicle,
        track,
        traffic,
        terminalOffset: this.targetOffset,
        targetId: this.targetId,
        targetSpeed,
        kerbAllowance
      });

      const lateralPassClear = Math.abs(currentLateral - target.otherLateral) >= 2.6;
      if (lateralPassClear && target.relativeLongitudinalVelocity < 0.10) {
        this.noProgressAge += dt;
      } else {
        this.noProgressAge = Math.max(0, this.noProgressAge - dt * 2.5);
      }
      this.lastTargetDelta = target.delta;

      // Aggressive combat contact tolerance: allow doors rubbing / close proximity with battle target
      const targetEnvelopeAllowed = Boolean(this.intent?.straightSend)
        && corridor.legal
        && corridor.minimumClearanceM >= finite(this.intent?.safetyThresholdM, -0.75);

      const staticTargetEscape = target.other.speed < 6.0
        && corridor.legal
        && corridor.blockerId === this.targetId;

      const targetCombatInteraction = (isSideBySide || this.divebombActive || this.switchbackActive)
        && (corridor.blockerId === this.targetId || corridor.blockerId === null);

      const newlyUnsafe = !corridor.legal || (!corridor.collisionFree && !targetEnvelopeAllowed && !staticTargetEscape && !targetCombatInteraction);
      const maxAttackAge = target.other.speed < 3.0 ? 18.0 : (isSideBySide ? 18.0 : 12.0);
      const minCommitmentDuration = 1.4;

      const shouldAbort = (this.age > maxAttackAge)
        || (!isSideBySide && this.noProgressAge > 5.0)
        || (!isSideBySide && !this.divebombActive && this.age > minCommitmentDuration && newlyUnsafe);

      if (shouldAbort) {
        this._clear('RETURN', 0.6);
        return {
          phase: 'RETURN',
          desiredOffset: baseOffset,
          target,
          corridor,
          committed: false,
          abortReason: newlyUnsafe ? 'TRACK_LIMIT_OR_OBSTACLE' : 'NO_PROGRESS',
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
        safetyThresholdM: finite(this.intent?.safetyThresholdM, -0.75),
        predictedTimeGainS: finite(this.intent?.predictedTimeGainS, 1.5),
        divebombing: this.divebombActive,
        switchbacking: this.switchbackActive,
        switchbackStage: this.switchbackStage,
        closingFloor: this.closingSpeedFloor,
        targetClosingSpeed: finite(this.intent?.targetClosingSpeed, this.closingSpeedFloor),
        ersDeployRequested: true,
        ersDeployReason: this.divebombActive ? 'DIVEBOMB_BURST' : 'SLINGSHOT_ATTACK_BURST',
        inCorner,
        distToCorner,
        contactAccepted: true,
        holdingAttackLine: true,
        kerbAllowance
      };
    }

    // Select primary overtake target ahead
    const target = traffic.entries
      .filter((e) => e.other.id !== this.passedTargetId
        && e.delta > 0 && e.delta < 62 && e.longitudinal > -1.5
        && !(e.other.aiTactical?.passTargetId === vehicle.id && e.delta < 8)
        && Math.abs(e.side) < roadMargin * 2 + 1.2)
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

    // 1. Slipstream check
    const slipstream = this.evaluateSlipstream(vehicle, target);
    if (slipstream.wakeStrength > 0.15) {
      this.draftAge += dt;
    } else {
      this.draftAge = Math.max(0, this.draftAge - dt);
    }

    const leadLateral = finite(target.otherLateral, 0);
    const spaceOnRight = roadMargin - leadLateral; // available asphalt on right (+lateral)
    const spaceOnLeft = roadMargin + leadLateral;  // available asphalt on left (-lateral)

    // 2. Dynamic Divebomb & Switchback checks in corners
    const divebomb = inCorner ? this.evaluateDivebomb({
      vehicle,
      target,
      track,
      nextTurn: turn,
      roadMargin,
      egoGripFactor: 1.0 + this.kerbUsage * 0.12,
      kerbAllowance
    }) : { feasible: false };

    const switchback = inCorner ? this.evaluateSwitchback({
      vehicle,
      target,
      track,
      nextTurn: turn,
      roadMargin,
      kerbAllowance
    }) : { feasible: false };

    const targetSpeed = Math.max(vehicle.speed, target.other.speed + 18.0);
    const candidates = [];

    // Lateral separation for slipstream pull-out
    const lateralSwoop = clamp(2.35 + this.aggression * 0.30, 2.35, 2.65);

    // Left Attack Lane Candidate (-lateral, side = -1):
    if (spaceOnLeft >= 2.2) {
      const isLeftInside = inCorner && turnSign < 0;
      const isLeftDive = isLeftInside && divebomb.feasible;
      const isLeftSwitch = !isLeftInside && inCorner && switchback.feasible;

      const leftSwoopOffset = clamp(leadLateral - lateralSwoop, -baseRoadMargin + 0.8, baseRoadMargin - 0.8);
      const leftTargetOffset = isLeftDive
        ? clamp(divebomb.insideOffset, -baseRoadMargin + 0.8, baseRoadMargin - 0.8)
        : (isLeftInside ? clamp(-Math.min(3.2, baseRoadMargin * 0.70), -baseRoadMargin + 0.8, baseRoadMargin - 0.8)
          : (isLeftSwitch ? clamp(switchback.outsideOffset, -baseRoadMargin + 0.8, baseRoadMargin - 0.8) : leftSwoopOffset));

      const leftCorridor = awareness.evaluateCorridor({
        vehicle,
        track,
        traffic,
        terminalOffset: leftTargetOffset,
        targetId: target.other.id,
        targetSpeed,
        kerbAllowance
      });

      if (leftCorridor.legal) {
        let phase = 'ATTACK_LEFT';
        if (inCorner) {
          phase = isLeftDive ? 'DIVEBOMB' : (isLeftInside ? 'ATTACK_INSIDE' : (isLeftSwitch ? 'SWITCHBACK' : 'ATTACK_OUTSIDE'));
        } else if (slipstream.shouldPullOut) {
          phase = 'SLINGSHOT';
        }

        const timeGain = (90 / Math.max(4, target.other.speed)) - (90 / Math.max(4, targetSpeed));
        const spaceAdvantage = (spaceOnLeft - spaceOnRight) * 1.4;
        const isSqueezed = spaceOnLeft < 3.8 && spaceOnRight >= 4.6;
        const isOpenSweep = spaceOnLeft >= 4.5 && spaceOnRight < 3.8;
        const score = leftCorridor.minimumClearanceM * 1.6
          + timeGain * 3.0
          + spaceAdvantage
          + (isLeftDive ? 4.5 + this.aggression * 2.5 : (isLeftInside ? 1.8 : 0.8))
          + (isLeftSwitch ? 3.0 + this.aggression * 1.8 : 0)
          - (isSqueezed ? 2.5 : 0)
          + (isOpenSweep ? 2.8 : 0);

        candidates.push({
          phase,
          side: -1,
          offset: leftTargetOffset,
          corridor: leftCorridor,
          score: leftCorridor.collisionFree ? score : score - 15,
          timeGainS: timeGain,
          isDive: isLeftDive,
          isSwitchback: isLeftSwitch
        });
      }
    }

    // Right Attack Lane Candidate (+lateral, side = 1):
    if (spaceOnRight >= 2.2) {
      const isRightInside = inCorner && turnSign > 0;
      const isRightDive = isRightInside && divebomb.feasible;
      const isRightSwitch = !isRightInside && inCorner && switchback.feasible;

      const rightSwoopOffset = clamp(leadLateral + lateralSwoop, -baseRoadMargin + 0.8, baseRoadMargin - 0.8);
      const rightTargetOffset = isRightDive
        ? clamp(divebomb.insideOffset, -baseRoadMargin + 0.8, baseRoadMargin - 0.8)
        : (isRightInside ? clamp(Math.min(3.2, baseRoadMargin * 0.70), -baseRoadMargin + 0.8, baseRoadMargin - 0.8)
          : (isRightSwitch ? clamp(switchback.outsideOffset, -baseRoadMargin + 0.8, baseRoadMargin - 0.8) : rightSwoopOffset));

      const rightCorridor = awareness.evaluateCorridor({
        vehicle,
        track,
        traffic,
        terminalOffset: rightTargetOffset,
        targetId: target.other.id,
        targetSpeed,
        kerbAllowance
      });

      if (rightCorridor.legal) {
        let phase = 'ATTACK_RIGHT';
        if (inCorner) {
          phase = isRightDive ? 'DIVEBOMB' : (isRightInside ? 'ATTACK_INSIDE' : (isRightSwitch ? 'SWITCHBACK' : 'ATTACK_OUTSIDE'));
        } else if (slipstream.shouldPullOut) {
          phase = 'SLINGSHOT';
        }

        const timeGain = (90 / Math.max(4, target.other.speed)) - (90 / Math.max(4, targetSpeed));
        const spaceAdvantage = (spaceOnRight - spaceOnLeft) * 1.4;
        const isSqueezed = spaceOnRight < 3.8 && spaceOnLeft >= 4.6;
        const isOpenSweep = spaceOnRight >= 4.5 && spaceOnLeft < 3.8;
        const score = rightCorridor.minimumClearanceM * 1.6
          + timeGain * 3.0
          + spaceAdvantage
          + (isRightDive ? 4.5 + this.aggression * 2.5 : (isRightInside ? 1.8 : 0.8))
          + (isRightSwitch ? 3.0 + this.aggression * 1.8 : 0)
          - (isSqueezed ? 2.5 : 0)
          + (isOpenSweep ? 2.8 : 0);

        candidates.push({
          phase,
          side: 1,
          offset: rightTargetOffset,
          corridor: rightCorridor,
          score: rightCorridor.collisionFree ? score : score - 15,
          timeGainS: timeGain,
          isDive: isRightDive,
          isSwitchback: isRightSwitch
        });
      }
    }

    // Lane commitment hysteresis
    if (this.phase !== 'NONE' && this.phase !== 'RETURN' && this.side != null) {
      for (const cand of candidates) {
        if (cand.side === this.side) {
          cand.score += 5.0;
        } else {
          cand.score -= 6.0;
        }
      }
    }

    candidates.sort((a, b) => b.score - a.score);
    const chosen = candidates[0] ?? null;

    const isSlowObstacle = target.other.speed < 12.0 && target.delta < 24.0;
    const attackRange = 28 + aggression * 10;

    const straightSend = !inCorner && (chosen?.timeGainS ?? 0) > 0.02 && target.delta < attackRange;
    const attackTrigger = Boolean(chosen) && (
      (target.delta < attackRange && (chosen.timeGainS ?? 0) > 0.02)
      || slipstream.shouldPullOut
      || isSlowObstacle
      || chosen.isDive
      || chosen.isSwitchback
    );

    if (chosen && attackTrigger) {
      this.phase = chosen.phase;
      this.targetId = target.other.id;
      this.targetOffset = chosen.offset;
      this.side = chosen.side;
      this.age = 0;
      this.noProgressAge = 0;
      this.divebombActive = chosen.isDive;
      this.switchbackActive = chosen.isSwitchback;
      this.switchbackStage = chosen.isSwitchback ? 'ENTRY_WIDE' : 'NONE';
      this.contactRubbingActive = false;
      this.ersAttackActive = true;

      this.intent = {
        targetId: this.targetId,
        side: this.side,
        lane: chosen.phase,
        gapM: target.delta,
        predictedTimeGainS: chosen.timeGainS,
        straightSend,
        safetyThresholdM: vehicle.classKey === 'prototype' ? -0.75 : -0.65,
        targetClosingSpeed: Math.max(this.closingSpeedFloor, this.closingSpeedFloor + target.delta * 0.25),
        commitmentDuration: target.other.speed < 2.5 ? 18.0 : 12.0
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
        switchbackStage: this.switchbackStage,
        closingFloor: this.closingSpeedFloor,
        targetClosingSpeed: this.intent.targetClosingSpeed,
        ersDeployRequested: true,
        ersDeployReason: this.divebombActive ? 'DIVEBOMB_BURST' : 'SLINGSHOT_ATTACK_BURST',
        inCorner,
        distToCorner,
        contactAccepted: true,
        holdingAttackLine: true,
        kerbAllowance
      };
    }

    if (isSlowObstacle) {
      const openSide = spaceOnRight >= spaceOnLeft ? 1 : -1;
      const evasionOffset = openSide > 0
        ? clamp(leadLateral + Math.max(3.8, spaceOnRight * 0.72), -roadMargin + 0.5, roadMargin - 0.5)
        : clamp(leadLateral - Math.max(3.8, spaceOnLeft * 0.72), -roadMargin + 0.5, roadMargin - 0.5);

      this.phase = openSide > 0 ? 'ATTACK_RIGHT' : 'ATTACK_LEFT';
      this.targetId = target.other.id;
      this.targetOffset = evasionOffset;
      this.side = openSide;
      this.age = 0;
      this.noProgressAge = 0;
      this.divebombActive = false;
      this.switchbackActive = false;
      this.switchbackStage = 'NONE';
      this.contactRubbingActive = false;
      this.ersAttackActive = true;
      this.intent = {
        targetId: this.targetId,
        side: this.side,
        lane: this.phase,
        gapM: target.delta,
        predictedTimeGainS: 2.2,
        straightSend: true,
        safetyThresholdM: -0.75,
        targetClosingSpeed: Math.max(this.closingSpeedFloor, 18.0),
        commitmentDuration: 14.0
      };

      return {
        phase: this.phase,
        desiredOffset: this.targetOffset,
        target,
        corridor: null,
        committed: true,
        straightSend: true,
        safetyThresholdM: -0.75,
        predictedTimeGainS: 2.2,
        divebombing: false,
        switchbacking: false,
        switchbackStage: 'NONE',
        closingFloor: this.closingSpeedFloor,
        targetClosingSpeed: this.intent.targetClosingSpeed,
        ersDeployRequested: true,
        ersDeployReason: 'OBSTACLE_EVASION_BURST',
        inCorner,
        distToCorner,
        contactAccepted: true,
        holdingAttackLine: true,
        kerbAllowance,
        reason: 'OBSTACLE_EVASION_OVERTAKE'
      };
    }

    this.phase = 'DRAFT';
    return {
      phase: 'DRAFT',
      desiredOffset: target.otherLateral,
      target,
      corridor: null,
      committed: false,
      closingFloor: this.closingSpeedFloor,
      targetClosingSpeed: this.closingSpeedFloor,
      ersDeployRequested: false,
      inCorner,
      distToCorner,
      kerbAllowance,
      reason: 'DRAFTING_IN_WAKE'
    };
  }
}

export { ATTACK_PHASES };
