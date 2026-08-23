/**
 * ResearchAIController.js
 * Complete AI Research Heuristics Controller for High-Performance Three.js Racing Simulation.
 * Integrates:
 * - TrafficAwareness (predictive perception & swept corridor evaluation)
 * - FrenetLatticePlanner (multi-candidate trajectory lattice & weighted optimization)
 * - TacticalAttackEngine (slipstream, dynamic divebomb, switchback, kerb exploitation)
 * - TacticalDefenseEngine (threat monitoring, FIA one-move inside line protection, tow breaking)
 * - PaceOptimizer (G-G friction circle, trail braking, backward reachable speed, throttle unwind)
 */

import { TrafficAwareness } from './TrafficAwareness.js';
import { FrenetLatticePlanner } from './FrenetLatticePlanner.js';
import { TacticalAttackEngine } from './TacticalAttackEngine.js';
import { TacticalDefenseEngine } from './TacticalDefenseEngine.js';
import { PaceOptimizer } from './PaceOptimizer.js';

const finite = (value, fallback = 0) => (Number.isFinite(value) ? value : fallback);

const clamp = (value, min, max) => Math.max(min, Math.min(max, value));

const wrap = (value, length) => {
  if (length <= 0) return 0;
  return ((value % length) + length) % length;
};

const wrapAngle = (angle) => {
  let result = (angle + Math.PI) % (Math.PI * 2);
  if (result < 0) result += Math.PI * 2;
  return result - Math.PI;
};

const offRoad = (surface) => surface?.zone === 'grass' || surface?.zone === 'runoff';

export class ResearchAIController {
  /**
   * @param {number} [index=1] - Driver slot index
   * @param {Object} [options]
   */
  constructor(index = 1, options = {}) {
    this.index = index;

    // Tunable Heuristic Parameters (Calibrated for aggressive 1:00 flat benchmark pace & stubborn defense)
    this._aggression = clamp(finite(options.aggression, 0.95), 0, 1);
    this._diveMargin = clamp(finite(options.diveMargin, 0.85), 0, 1);
    this._defenseReactivity = clamp(finite(options.defenseReactivity, 0.95), 0, 1);
    this._kerbUsage = clamp(finite(options.kerbUsage, 0.95), 0, 1);
    this._lookahead = clamp(finite(options.lookahead, 24.0), 8, 30);
    this._trailBrakingSkill = clamp(finite(options.trailBrakingSkill, 0.95), 0, 1);
    this._ersAttackMode = Boolean(options.ersAttackMode ?? false);

    this.skill = clamp(finite(options.skill, 0.98), 0.5, 1.0);

    // AI Core Modules
    this.awareness = new TrafficAwareness();
    this.trajectoryPlanner = new FrenetLatticePlanner();
    this.attackEngine = new TacticalAttackEngine({
      index,
      aggression: this._aggression,
      diveMargin: this._diveMargin,
      kerbUsage: this._kerbUsage
    });
    this.defenseEngine = new TacticalDefenseEngine({
      index,
      defenseReactivity: this._defenseReactivity
    });
    this.paceOptimizer = new PaceOptimizer({
      trailBrakingSkill: this._trailBrakingSkill,
      unwindFactor: 0.60
    });

    // Runtime state
    this.trajectoryPlan = null;
    this.debugEnabled = true;
    this.debugState = null;
    this.steerCommand = 0;
    this.lastDistance = null;
    this.stallTime = 0;
    this.recoveryTimer = 0;
    this.marshalRecoveries = 0;
    this.referenceProfile = null;

    // ERS state
    this.ersPlan = {
      previousDistance: null,
      travelledM: 0,
      lapIndex: 0,
      lapStartSoc: null,
      targetSoc: null,
      mode: 'AUTO'
    };
  }

  // --- Tunable Parameter Getters & Setters ---

  get aggression() { return this._aggression; }
  set aggression(val) {
    this._aggression = clamp(finite(val, 0.5), 0, 1);
    this.attackEngine.setParameters({ aggression: this._aggression });
  }

  get diveMargin() { return this._diveMargin; }
  set diveMargin(val) {
    this._diveMargin = clamp(finite(val, 0.6), 0, 1);
    this.attackEngine.setParameters({ diveMargin: this._diveMargin });
  }

  get defenseReactivity() { return this._defenseReactivity; }
  set defenseReactivity(val) {
    this._defenseReactivity = clamp(finite(val, 0.8), 0, 1);
    this.defenseEngine.setParameters({ defenseReactivity: this._defenseReactivity });
  }

  get kerbUsage() { return this._kerbUsage; }
  set kerbUsage(val) {
    this._kerbUsage = clamp(finite(val, 0.8), 0, 1);
    this.attackEngine.setParameters({ kerbUsage: this._kerbUsage });
  }

  get lookahead() { return this._lookahead; }
  set lookahead(val) {
    this._lookahead = clamp(finite(val, 14.0), 8, 30);
  }

  get trailBrakingSkill() { return this._trailBrakingSkill; }
  set trailBrakingSkill(val) {
    this._trailBrakingSkill = clamp(finite(val, 0.85), 0, 1);
    this.paceOptimizer.setParameters({ trailBrakingSkill: this._trailBrakingSkill });
  }

  get ersAttackMode() { return this._ersAttackMode; }
  set ersAttackMode(val) {
    this._ersAttackMode = Boolean(val);
  }

  setReferenceProfile(profile) {
    this.referenceProfile = profile;
  }

  get telemetry() {
    return this.debugState?.telemetry ?? null;
  }

  setHeuristicWeights(heuristics = {}) {
    if (heuristics.aggression != null) this.aggression = heuristics.aggression > 1 ? heuristics.aggression / 100 : heuristics.aggression;
    if (heuristics.diveMargin != null) this.diveMargin = heuristics.diveMargin > 1 ? heuristics.diveMargin / 100 : heuristics.diveMargin;
    if (heuristics.defenseReactivity != null) this.defenseReactivity = heuristics.defenseReactivity > 1 ? heuristics.defenseReactivity / 100 : heuristics.defenseReactivity;
    if (heuristics.kerbUsage != null) this.kerbUsage = heuristics.kerbUsage > 1 ? heuristics.kerbUsage / 100 : heuristics.kerbUsage;
    if (heuristics.lookaheadHorizon != null) this.lookahead = heuristics.lookaheadHorizon;
    if (heuristics.lookahead != null) this.lookahead = heuristics.lookahead;
  }

  setReferenceProfile(profile = null) {
    this.referenceProfile = profile && typeof profile.targetAtDistance === 'function' ? profile : null;
    return Boolean(this.referenceProfile);
  }

  setDebugEnabled(enabled) {
    this.debugEnabled = Boolean(enabled);
    if (!this.debugEnabled) this.debugState = null;
    return this.debugEnabled;
  }

  getDebugState() {
    return this.debugState;
  }

  resetForRace(vehicle = null) {
    this.attackEngine.reset();
    this.defenseEngine.reset();
    this.trajectoryPlan = null;
    this.steerCommand = 0;
    this.lastDistance = null;
    this.stallTime = 0;
    this.recoveryTimer = 0;
    this.ersPlan = {
      previousDistance: null,
      travelledM: 0,
      lapIndex: 0,
      lapStartSoc: null,
      targetSoc: null,
      mode: 'AUTO'
    };
  }

  /**
   * ERS Energy Recovery & Deployment Strategy Planner.
   * @private
   */
  _planERS(vehicle, track, { committed = false, defending = false, defensiveErsRequested = false, straight = false, throttle = 0 } = {}) {
    if (!vehicle.ers?.enabled) return 'OFF';
    const plan = this.ersPlan;
    const currentDist = finite(vehicle.distance, 0);
    const trackLen = Math.max(1, finite(track?.length, 1000));

    if (plan.previousDistance === null) {
      plan.previousDistance = currentDist;
      plan.lapStartSoc = finite(vehicle.ers.soc, 1.0);
    } else {
      const delta = wrap(currentDist - plan.previousDistance + trackLen * 0.5, trackLen) - trackLen * 0.5;
      plan.travelledM += Math.max(0, delta);
      plan.previousDistance = currentDist;
    }

    const lapIndex = Math.floor(plan.travelledM / trackLen);
    if (lapIndex !== plan.lapIndex) {
      plan.lapIndex = lapIndex;
      plan.lapStartSoc = finite(vehicle.ers.soc, 1.0);
    }

    const lapProgress = (plan.travelledM % trackLen) / trackLen;
    const tacticalReserve = (committed || defending) ? 0.06 : 0.14;
    const plannedSpend = Math.min(0.38, Math.max(0, finite(plan.lapStartSoc, 1.0) - tacticalReserve));
    const targetSoc = Math.max(tacticalReserve, finite(plan.lapStartSoc, 1.0) - plannedSpend * lapProgress);
    const surplus = finite(vehicle.ers.soc, 1.0) - targetSoc;

    const minSocCeiling = (vehicle.ers.minSoc ?? 0.04) + 0.03;
    const canDeploy = finite(vehicle.ers.soc, 0) > minSocCeiling;

    const offensiveDeploy = committed || this._ersAttackMode;
    const defensiveDeploy = defending && defensiveErsRequested;
    const paceSurplusDeploy = (straight && throttle > 0.85 && surplus > 0.01)
      || (lapProgress > 0.80 && throttle > 0.88 && surplus > 0.015);

    const shouldDeploy = canDeploy && (offensiveDeploy || defensiveDeploy || paceSurplusDeploy);

    plan.targetSoc = targetSoc;
    plan.mode = shouldDeploy ? 'ATTACK' : 'AUTO';
    return plan.mode;
  }

  /**
   * Main simulation tick update.
   * @param {Object} vehicle - Ego vehicle
   * @param {Array<Object>} vehicles - All vehicles on circuit
   * @param {Object} track - Track geometry model
   * @param {Object} race - Race session state
   * @param {number} dt - Timestep delta in seconds
   */
  update(vehicle, vehicles, track, race = null, dt = 1 / 120) {
    const racePhase = race?.phase ?? 'racing';
    if (racePhase !== 'racing') {
      vehicle.controls = { throttle: 0, brake: 1, steer: 0, handbrake: 0 };
      return;
    }

    if (vehicle.finished) {
      this._cooldown(vehicle, track, dt);
      return;
    }

    // 1. Perception
    const traffic = this.awareness.scan(vehicle, vehicles, track);
    const current = traffic.current;
    const isOffTrack = offRoad(current);

    const roadHalfWidth = finite(track?.roadHalfWidth, 6.5);
    const baseRoadMargin = Math.max(2.1, roadHalfWidth - 1.25);
    const kerbAllowance = this._kerbUsage * Math.min(1.35, finite(track?.curbWidth, 0.8) * 0.9);
    const plannedRoadMargin = baseRoadMargin + kerbAllowance;

    // Track boundary deviation and stall recovery
    if (this.lastDistance === null) this.lastDistance = vehicle.distance;
    const progress = wrap(vehicle.distance - this.lastDistance + (track?.length || 1000) * 0.5, track?.length || 1000) - (track?.length || 1000) * 0.5;
    this.lastDistance = vehicle.distance;

    const queued = traffic.ahead && traffic.ahead.delta < 12;
    this.stallTime = vehicle.speed < 2.2 && progress < 0.25 && !queued
      ? this.stallTime + dt
      : Math.max(0, this.stallTime - dt * 2);

    if (isOffTrack) {
      this.recoveryTimer = 1.2;
    } else {
      this.recoveryTimer = Math.max(0, this.recoveryTimer - dt);
    }

    const recovering = isOffTrack || this.recoveryTimer > 0 || this.stallTime > 0.7;

    // Marshal recovery safeguard if stuck
    if (isOffTrack && this.stallTime > 5.0 && vehicle.marshalRecoverTo) {
      vehicle.marshalRecoverTo(track, vehicle.distance + 10, 0);
      this.marshalRecoveries += 1;
      this.stallTime = 0;
      this.recoveryTimer = 1.0;
      return;
    }

    // 2. Tactical Evaluation (Defense -> Attack -> Pace)
    const referenceLine = this.referenceProfile?.paceAtDistance?.(vehicle.distance + 24)?.lineLateral ?? 0;
    const paceLine = clamp(referenceLine, -baseRoadMargin, baseRoadMargin);

    const defDecision = this.defenseEngine.update({
      vehicle,
      track,
      traffic,
      awareness: this.awareness,
      dt,
      baseLine: paceLine,
      recovering
    });

    const attDecision = this.attackEngine.update({
      vehicle,
      track,
      traffic,
      awareness: this.awareness,
      dt,
      aggression: this._aggression,
      policyLine: paceLine,
      recovering,
      pitIntent: vehicle.pitIntent
    });

    // Tactical Decision arbitration
    let tacticalMode = 'PACE';
    let targetOffset = paceLine;
    let targetId = null;
    let committed = false;
    let defending = false;
    let straightSend = false;
    let safetyThresholdM = 0;
    let tacticalReason = 'OPTIMAL_RACING_LINE';

    if (vehicle.pitIntent?.active) {
      tacticalMode = 'PIT';
      targetOffset = finite(vehicle.pitIntent.targetLateralM, paceLine);
      tacticalReason = 'PIT_LANE_ENTRY';
    } else if (recovering) {
      tacticalMode = 'RECOVER';
      targetOffset = 0;
      tacticalReason = isOffTrack ? 'OFF_TRACK_RECOVERY' : 'STALL_RECOVERY';
    } else if (defDecision.defending) {
      tacticalMode = 'DEFEND';
      targetOffset = defDecision.desiredOffset;
      targetId = defDecision.target?.other?.id ?? null;
      defending = true;
      tacticalReason = defDecision.reason;
    } else if (attDecision.committed || (attDecision.phase !== 'NONE' && attDecision.phase !== 'RETURN')) {
      tacticalMode = 'ATTACK';
      targetOffset = attDecision.desiredOffset;
      targetId = attDecision.target?.other?.id ?? null;
      committed = attDecision.committed;
      straightSend = attDecision.straightSend;
      safetyThresholdM = attDecision.safetyThresholdM;
      tacticalReason = attDecision.divebombing ? 'DIVEBOMB_CORNER_ENTRY'
        : attDecision.switchbacking ? 'SWITCHBACK_LATE_APEX'
        : `${attDecision.phase}_MANEUVER`;
    }

    // 3. Multi-Candidate Frenet Trajectory Planning
    const turns = [18, 36, 56].map((d) =>
      track?.atDistance ? track.atDistance(vehicle.distance + d) : { curvature: 0, turnSign: 1 }
    );
    const turn = turns.sort((a, b) => Math.abs(b.curvature) - Math.abs(a.curvature))[0];
    const turnCurvature = Math.abs(finite(turn?.curvature, 0));
    const turnSign = Math.sign(finite(turn?.turnSign, 1)) || 1;

    // Tactical candidate variations for lattice sampling
    const tacticalCandidates = [
      { offset: targetOffset, intentType: 'TACTICAL_TARGET', transitionScales: [0.75, 1.0, 1.3] },
      { offset: paceLine, intentType: 'RACING_LINE', transitionScales: [1.0, 1.5] },
      { offset: plannedRoadMargin * 0.65, intentType: 'RIGHT_OPEN_LANE', transitionScales: [0.8, 1.1] },
      { offset: -plannedRoadMargin * 0.65, intentType: 'LEFT_OPEN_LANE', transitionScales: [0.8, 1.1] }
    ];

    const lookAheadDist = recovering
      ? clamp(9.0 + vehicle.speed * 0.35, 8.0, 18.0)
      : clamp(this._lookahead + vehicle.speed * 0.55, 10.0, 30.0);

    const maxTireWear = Math.max(0, ...(vehicle.wheels ?? []).map((w) => finite(w.wear, 0)));
    const tireGripFactor = clamp(1.0 - maxTireWear * 0.50, 0.80, 1.0);

    // Compute speed envelope via backward integration with defensive offset and threat parameters
    const physicalTargetSpeed = this.paceOptimizer.computeSpeedEnvelope({
      vehicle,
      track,
      tireGripFactor,
      skill: this.skill,
      aggression: this._aggression,
      defending,
      threatScore: defDecision.threatScore || 0,
      closingSpeed: defDecision.closingSpeed || 0,
      insideLineOffset: targetOffset
    });

    this.trajectoryPlan = this.trajectoryPlanner.plan({
      vehicle,
      track,
      desiredOffset: targetOffset,
      fallbackOffsets: recovering || committed || defending ? [] : [paceLine, finite(current?.lateral, 0)],
      tacticalCandidates: recovering ? [] : tacticalCandidates,
      trafficEntries: traffic.entries,
      targetSpeed: physicalTargetSpeed,
      aggression: this._aggression,
      racecraftPhase: defending ? defDecision.phase : attDecision.phase,
      targetId,
      recovering,
      pitActive: Boolean(vehicle.pitIntent?.active),
      urgent: committed || defending,
      roadMargin: plannedRoadMargin,
      kerbAllowance,
      lookAhead: lookAheadDist
    });

    const trackingPoint = this.trajectoryPlan.trackingPoint ?? this.trajectoryPlan.points.at(-1);
    const plannedTargetOffset = finite(trackingPoint?.lateral, targetOffset);
    const targetPos = {
      x: finite(trackingPoint?.x, vehicle.position.x),
      y: finite(trackingPoint?.y, vehicle.position.y),
      z: finite(trackingPoint?.z, vehicle.position.z),
      lateral: plannedTargetOffset
    };

    // 4. Lateral Pursuit Steering
    const headingError = wrapAngle(
      Math.atan2(targetPos.x - vehicle.position.x, targetPos.z - vehicle.position.z) - vehicle.yaw
    );
    const lateralError = finite(current?.lateral, 0) - plannedTargetOffset;

    this.steerCommand = this.paceOptimizer.computeSteering({
      previous: this.steerCommand,
      headingError,
      lateralError,
      yawRate: vehicle.yawRate,
      dt,
      committed,
      recovering
    });

    // 5. Longitudinal Target Speed & Dynamic Adjustments
    let desiredSpeed = physicalTargetSpeed;

    // Cap speed based on chosen trajectory curvature
    const lateralAccelBudget = (vehicle.classKey === 'prototype' ? 24.0 : vehicle.classKey === 'gt' ? 17.5 : 13.5) * tireGripFactor;
    const trajectorySpeedLimit = this.trajectoryPlan.points.reduce((limit, p) => {
      const curv = Math.max(0, finite(p.curvature, 0));
      if (curv < 1e-5) return limit;
      const cornerSpeed = Math.sqrt(lateralAccelBudget / curv);
      const reachableSpeed = Math.sqrt(cornerSpeed * cornerSpeed + 2.0 * 8.5 * Math.max(0, finite(p.forwardDistance, 0)));
      return Math.min(limit, reachableSpeed);
    }, 95.0);

    desiredSpeed = Math.min(desiredSpeed, trajectorySpeedLimit);

    // Synchronize pace with user baseline reference profile if available
    const refSpeed = this.referenceProfile?.paceAtDistance?.(vehicle.distance)?.speed;
    if (Number.isFinite(refSpeed) && refSpeed > 10.0 && tacticalMode === 'PACE') {
      desiredSpeed = Math.min(desiredSpeed, Math.max(desiredSpeed * 0.92, refSpeed * (1.0 + (this._aggression - 0.5) * 0.08)));
    }

    // Overtake speed adjustments
    const passTarget = attDecision.target;
    const actualSeparation = passTarget
      ? Math.abs(finite(current?.lateral, 0) - finite(passTarget.otherLateral, 0))
      : 99;

    if (committed && passTarget) {
      const closingFloor = straightSend ? 8.0 : 3.5;
      const isSlowObstacle = passTarget.other.speed < 16.0;
      const obstacleFloor = isSlowObstacle ? Math.min(physicalTargetSpeed, Math.max(14.0, passTarget.other.speed + 10.0)) : 0;

      if (straightSend) {
        desiredSpeed = Math.min(physicalTargetSpeed, Math.max(desiredSpeed, passTarget.other.speed + closingFloor));
      } else {
        // In corners / braking zones, cap desiredSpeed to physicalTargetSpeed to ensure staying on legal track!
        desiredSpeed = Math.min(physicalTargetSpeed, Math.max(obstacleFloor, passTarget.other.speed + closingFloor));
      }
    } else if (passTarget && passTarget.delta > 0 && passTarget.delta < 32 && !defending) {
      const isSlowObstacle = passTarget.other.speed < 16.0;
      if (isSlowObstacle) {
        const escapeSpeed = Math.min(physicalTargetSpeed, Math.max(12.0, passTarget.other.speed + 8.0));
        desiredSpeed = Math.min(desiredSpeed, Math.max(escapeSpeed, passTarget.other.speed + 4.0));
      } else {
        const safeGap = clamp(7.0 + passTarget.relativeLongitudinalVelocity ** 2 / 10.0, 8.0, 30.0);
        desiredSpeed = Math.min(
          desiredSpeed,
          Math.max(0, passTarget.other.speed + clamp((passTarget.delta - safeGap) * 0.35, -6.0, 2.5))
        );
      }
    }

    if (recovering) desiredSpeed = isOffTrack ? 7.0 : 14.0;

    // 6. Emergency Hazard Avoidance
    const hazard = this.awareness.forwardHazard(traffic);
    const passTargetClear = committed && actualSeparation >= 2.4 && hazard?.other?.id === passTarget?.other?.id;
    const isEvasiveOvertake = committed && passTarget && passTarget.other.speed < 16.0 && actualSeparation >= 1.8;
    const emergency = Boolean(hazard && !passTargetClear && !isEvasiveOvertake && (hazard.ttc < 2.5 || (hazard.longitudinal < 8.0 && Math.abs(finite(current?.lateral, 0) - finite(hazard.otherLateral, 0)) < 1.8)));

    if (emergency) {
      desiredSpeed = Math.min(desiredSpeed, Math.max(0, hazard.other.speed - 2.5));
    }

    // 7. Low-Level Pedal Control & Trail Braking
    const speedError = desiredSpeed - vehicle.speed;
    const straight = turnCurvature < 0.0035;
    const liveSlip = Math.atan2(
      finite(vehicle.localVelocity?.x, 0),
      Math.max(3.0, Math.abs(finite(vehicle.localVelocity?.z, vehicle.speed)))
    );

    const latAccel = vehicle.speed * vehicle.speed * finite(this.trajectoryPlan.maxCurvaturePerM, 0);

    const pedals = this.paceOptimizer.computePedals({
      vehicle,
      speedError,
      desiredSpeed,
      headingError,
      lateralAccel: latAccel,
      steerAngle: this.steerCommand,
      yawRate: vehicle.yawRate,
      slipAngle: liveSlip,
      straight,
      recovering,
      emergency,
      defending,
      tireGripFactor
    });

    vehicle.controls = {
      steer: clamp(this.steerCommand, -1, 1),
      throttle: pedals.throttle,
      brake: pedals.brake,
      handbrake: 0
    };

    vehicle.aiTarget = { x: targetPos.x, z: targetPos.z, lateral: plannedTargetOffset };

    // 8. ERS Deployment
    const ersMode = vehicle.classKey === 'prototype'
      ? this._planERS(vehicle, track, {
          committed,
          defending,
          defensiveErsRequested: Boolean(defDecision.ersDeployRequested),
          straight,
          throttle: pedals.throttle
        })
      : 'OFF';

    if (vehicle.classKey === 'prototype') vehicle.setERSMode?.(ersMode);

    // 9. Diagnostics and Debug State Recording
    this._recordDebugState(vehicle, {
      tacticalMode,
      tacticalReason,
      desiredSpeed,
      targetOffset: tacticalMode === 'DEFEND' ? finite(defDecision.desiredOffset, targetOffset) : finite(plannedTargetOffset, targetOffset),
      targetPos,
      headingError,
      lateralError,
      traffic,
      hazard,
      recovering,
      attDecision,
      defDecision,
      pedals,
      latAccel,
      ersMode,
      emergency
    });
  }

  /**
   * Build complete tactical diagnostics snapshot for 3D debug rendering and telemetry.
   * @private
   */
  _recordDebugState(vehicle, {
    tacticalMode,
    tacticalReason,
    desiredSpeed,
    targetOffset,
    targetPos,
    headingError,
    lateralError,
    traffic,
    hazard,
    recovering,
    attDecision,
    defDecision,
    pedals,
    latAccel,
    ersMode,
    emergency
  }) {
    if (!this.debugEnabled) return;

    const challenger = traffic.behind;
    const target = attDecision.target || defDecision.target;

    this.debugState = {
      vehicleId: vehicle.id,
      name: vehicle.name,
      classKey: vehicle.classKey,
      mode: tacticalMode,
      racecraftPhase: attDecision.phase,
      reason: tacticalReason,
      targetId: target?.other?.id ?? null,
      desiredOffset: finite(targetOffset),
      desiredSpeed: finite(desiredSpeed),
      targetSpeed: finite(desiredSpeed),
      currentSpeed: finite(vehicle.speed),
      speedKmh: finite(vehicle.speed * 3.6),
      speedError: finite(desiredSpeed - vehicle.speed),
      headingError: finite(headingError),
      lateralError: finite(lateralError),
      recovering: Boolean(recovering),
      
      // Candidate Trajectory Lattice
      candidates: this.trajectoryPlan?.candidates ?? [],
      bestCandidate: this.trajectoryPlan,
      
      // Threat & Challenger Perception
      threat: {
        challengerId: challenger?.other?.id ?? null,
        threatLevel: defDecision.threatLevel ?? 'NONE',
        threatScore: finite(defDecision.threatScore, 0),
        attackerIntent: defDecision.attackerIntent ?? 'NONE',
        gapM: finite(challenger?.delta, 99),
        closingSpeedMps: finite(challenger?.relativeLongitudinalVelocity, 0),
        ttc: finite(challenger?.ttc, 99)
      },

      // Human-readable tactical thought summary
      thought: {
        maneuver: tacticalMode === 'ATTACK' ? attDecision.phase : (tacticalMode === 'DEFEND' ? defDecision.phase : tacticalMode),
        deployedOffsetM: finite(targetOffset),
        targetId: target?.other?.id ?? null,
        committed: Boolean(attDecision.committed),
        defending: Boolean(defDecision.defending),
        defensivePhase: defDecision.phase ?? 'NONE',
        attackerIntent: defDecision.attackerIntent ?? 'NONE',
        straightSend: Boolean(attDecision.straightSend),
        divebombing: Boolean(attDecision.divebombing),
        switchbacking: Boolean(attDecision.switchbacking),
        predictedTimeGainS: finite(attDecision.predictedTimeGainS, 0),
        safetyThresholdM: finite(attDecision.safetyThresholdM, 0),
        kerbAllowanceM: finite(attDecision.kerbAllowance, 0),
        corridorCollisionFree: Boolean(attDecision.corridor?.collisionFree ?? true),
        corridorMinimumClearanceM: finite(attDecision.corridor?.minimumClearanceM, 99),
        trajectoryCollisionFree: Boolean(this.trajectoryPlan?.collisionFree),
        trajectoryRoadLegal: Boolean(this.trajectoryPlan?.roadLegal),
        trajectorySelectedOffsetM: finite(this.trajectoryPlan?.selectedOffset)
      },

      // G-G Friction circle & controls telemetry
      telemetry: {
        lateralAccelMps2: finite(latAccel),
        lateralUtilization: finite(pedals.friction?.latUtilization),
        maxG: finite(pedals.friction?.maxTotalAccel / 9.81, 1.5),
        trailBrakingActive: Boolean(pedals.trailBraking),
        throttle: finite(pedals.throttle),
        brake: finite(pedals.brake),
        steer: finite(this.steerCommand),
        ersMode,
        ersSoc: finite(vehicle.ers?.soc, 0),
        yawRate: finite(vehicle.yawRate, 0),
        emergency: Boolean(emergency)
      }
    };
  }

  /**
   * Post-race cooldown autopilot.
   * @private
   */
  _cooldown(vehicle, track, dt) {
    if (vehicle.despawned) {
      vehicle.controls = { throttle: 0, brake: 1, steer: 0, handbrake: 0 };
      return;
    }
    vehicle.trafficGhost = true;
    vehicle.cooldownTime = (vehicle.cooldownTime || 0) + dt;

    const point = track?.atDistance
      ? track.atDistance(vehicle.distance + 14.0)
      : { x: vehicle.position.x, z: vehicle.position.z };

    const heading = wrapAngle(Math.atan2(point.x - vehicle.position.x, point.z - vehicle.position.z) - vehicle.yaw);
    const desiredSpeed = vehicle.cooldownTime < 4 ? 16 : vehicle.cooldownTime < 10 ? 8 : 0;
    const speedError = desiredSpeed - vehicle.speed;

    vehicle.controls = {
      steer: clamp(heading * 2.0, -1, 1),
      throttle: clamp(speedError * 0.15, 0, 0.5),
      brake: vehicle.cooldownTime > 12 ? 0.9 : clamp((-speedError - 0.5) * 0.2, 0, 0.8),
      handbrake: 0
    };

    if (vehicle.cooldownTime > 16 && vehicle.speed < 0.6) {
      vehicle.despawned = true;
    }
  }
}
