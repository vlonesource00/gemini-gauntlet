/**
 * ResearchAIController.js
 * Complete AI Research Heuristics Controller for High-Performance Three.js Racing Simulation.
 * Integrates:
 * - Layer 1: GlobalTimeOptimalEngine (2D free-boundary time-optimal racing line & velocity envelopes)
 * - TrafficAwareness (predictive perception & swept corridor evaluation)
 * - FrenetLatticePlanner (multi-candidate trajectory lattice & weighted optimization)
 * - TacticalAttackEngine (slipstream, dynamic divebomb, switchback, kerb exploitation)
 * - TacticalDefenseEngine (threat monitoring, FIA one-move inside line protection, tow breaking)
 * - PaceOptimizer (G-G friction circle, trail braking, backward reachable speed, throttle unwind)
 * - CombatDynamicsEngine (elastic contact rubbing equilibrium & slip-slope micro countersteer)
 */

import { TrafficAwareness } from './TrafficAwareness.js';
import { FrenetLatticePlanner } from './FrenetLatticePlanner.js';
import { TacticalAttackEngine } from './TacticalAttackEngine.js';
import { TacticalDefenseEngine } from './TacticalDefenseEngine.js';
import { PaceOptimizer } from './PaceOptimizer.js';
import { CombatDynamicsEngine } from './v2/CombatDynamicsEngine.js';
import { GlobalTimeOptimalEngine } from './v2/GlobalTimeOptimalEngine.js';

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
    this.id = typeof index === 'string' ? index : `ai-${index}`;

    // Tunable Heuristic Parameters (Calibrated for aggressive 1:00 flat benchmark pace & stubborn defense)
    this._aggression = clamp(finite(options.aggression, 0.95), 0, 1);
    this._diveMargin = clamp(finite(options.diveMargin, 0.85), 0, 1);
    this._defenseReactivity = clamp(finite(options.defenseReactivity, 0.95), 0, 1);
    this._kerbUsage = clamp(finite(options.kerbUsage, 0.95), 0, 1);
    this._lookahead = clamp(finite(options.lookahead, 24.0), 8, 30);
    this._trailBrakingSkill = clamp(finite(options.trailBrakingSkill, 0.95), 0, 1);
    this._ersAttackMode = Boolean(options.ersAttackMode ?? false);

    this.skill = clamp(finite(options.skill, 0.98), 0.5, 1.0);

    // AI Core Modules & Hybrid Layers
    this.awareness = new TrafficAwareness();
    this.optimalEngine = null;
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
    this.combatDynamics = new CombatDynamicsEngine();

    // Scenario Engine & Racecraft state handles
    this.draftTargetId = null;
    this.passTargetId = null;
    this.defenseTargetId = null;
    this.passPhase = 'NONE';
    this.racecraft = {
      phase: 'NONE',
      targetId: null,
      defenseTargetId: null,
      side: 0,
      targetOffset: 0
    };

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
    this._lookahead = clamp(finite(val, 24.0), 7.5, 30);
  }

  /**
   * Calculate curvature-adaptive lookahead horizon.
   * Tightens to 7.5m-10m in tight chicanes/hairpins for millimeter-precise apex clipping,
   * expands up to 26.0m on high-speed straights.
   * @param {number} speed - Current velocity in m/s
   * @param {number} kappa - Local / upcoming track curvature
   * @returns {number} Lookahead distance in meters [7.5, 26.0]
   */
  computeLookahead(speed = 0, kappa = 0) {
    const v = Math.max(0, finite(speed, 0));
    const k = Math.abs(finite(kappa, 0));
    return clamp((v * 0.36) / (1.0 + 90.0 * k), 7.5, 26.0);
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
    this.referenceProfile = profile && typeof profile.targetAtDistance === 'function'
      ? profile
      : (profile && typeof profile.paceAtDistance === 'function' ? profile : null);
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
    this.draftTargetId = null;
    this.passTargetId = null;
    this.defenseTargetId = null;
    this.passPhase = 'NONE';
    if (this.racecraft) {
      this.racecraft.phase = 'NONE';
      this.racecraft.targetId = null;
      this.racecraft.defenseTargetId = null;
      this.racecraft.side = 0;
      this.racecraft.targetOffset = 0;
    }
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
    if (!vehicle || !track) return;

    const racePhase = race?.phase ?? 'racing';
    if (racePhase !== 'racing') {
      vehicle.controls = { throttle: 0, brake: 1, steer: 0, handbrake: 0 };
      return;
    }

    if (vehicle.finished) {
      this._cooldown(vehicle, track, dt);
      return;
    }

    // 1. Layer 1 Global Time Optimal Engine instantiation if track changes
    if (!this.optimalEngine || this.optimalEngine.track !== track) {
      this.optimalEngine = new GlobalTimeOptimalEngine({ track });
    }

    // 2. Perception & Multi-Agent Awareness
    const traffic = this.awareness.scan(vehicle, vehicles, track);
    const current = traffic.current;
    const roadHalfWidth = finite(track?.roadHalfWidth, 6.5);
    const isOffTrack = offRoad(current) || offRoad(vehicle.surface) || Math.abs(finite(current?.lateral, 0)) > (roadHalfWidth + 0.35);

    const baseRoadMargin = Math.max(2.1, Math.min(4.8, roadHalfWidth - 2.0));
    const kerbAllowance = this._kerbUsage * Math.min(0.65, finite(track?.curbWidth, 0.8) * 0.5);
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

    // 3. Tactical Evaluation (Defense -> Attack -> Pace)
    const turns = [0, 8, 16, 28, 45, 65].map((d) =>
      track?.atDistance ? track.atDistance(vehicle.distance + d) : { curvature: 0, turnSign: 1 }
    );
    const turn = turns.sort((a, b) => Math.abs(b.curvature) - Math.abs(a.curvature))[0];
    const turnCurvature = Math.abs(finite(turn?.curvature, 0));
    const turnSign = Math.sign(finite(turn?.turnSign, 1)) || 1;

    const currentPoint = track?.atDistance ? track.atDistance(vehicle.distance) : { curvature: 0 };
    const signedCurv = finite(currentPoint?.curvature, 0);
    const currentCurv = Math.abs(signedCurv);

    // Curvature-Adaptive Apex Lookahead Horizon
    const dynamicLookahead = this.computeLookahead(vehicle.speed, currentCurv);

    const upcomingPoint = track?.atDistance ? track.atDistance(vehicle.distance + Math.max(18.0, dynamicLookahead)) : { curvature: 0 };
    const upcomingCurv = finite(upcomingPoint.curvature, 0);
    
    // Globally time-optimal baseline trajectory (Layer 1 GlobalTimeOptimalEngine)
    if (!this.optimalEngine || this.optimalEngine.track !== track) {
      this.optimalEngine = new GlobalTimeOptimalEngine({ track });
    }
    const isMatchingTrack = Boolean(this.referenceProfile) && (
      this.referenceProfile.trackId
        ? this.referenceProfile.trackId === track?.id
        : Math.abs((this.referenceProfile?.trackLength || 3061.7) - (track?.length || 1000)) < 100
    );
    const activeProfile = (this.referenceProfile && isMatchingTrack) ? this.referenceProfile : this.optimalEngine;
    const fallbackGeometricLine = clamp(-Math.sign(upcomingCurv) * Math.min(2.4, Math.abs(upcomingCurv) * 80.0), -2.5, 2.5);
    const referenceLine = (activeProfile && typeof activeProfile.paceAtDistance === 'function')
      ? (activeProfile.paceAtDistance(vehicle.distance + Math.max(18.0, dynamicLookahead))?.lineLateral ?? fallbackGeometricLine)
      : fallbackGeometricLine;
    const paceLine = clamp(referenceLine, -baseRoadMargin, baseRoadMargin);

    const defDecision = this.defenseEngine.update({
      vehicle,
      track,
      traffic,
      awareness: this.awareness,
      dt,
      baseLine: paceLine,
      recovering,
      passedTargetId: this.attackEngine.passedTargetId
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
      if (this.defenseEngine.defending) {
        this.defenseEngine._clear('NONE');
      }
    } else if (defDecision.defending) {
      tacticalMode = 'DEFEND';
      targetOffset = defDecision.desiredOffset;
      targetId = defDecision.target?.other?.id ?? null;
      defending = true;
      tacticalReason = defDecision.reason;
    } else if (attDecision.phase === 'RETURN') {
      tacticalMode = 'PACE';
      targetOffset = paceLine;
      tacticalReason = 'OVERTAKE_COMPLETE_RETURN';
    }

    // Universal continuous targetOffset rate-limiter: guarantees zero lateral jump artifacts
    const currentCarLat = finite(current?.lateral, 0);
    const maxLateralRate = (recovering || isOffTrack) ? 6.0 : (committed ? 4.5 : 3.2);
    const maxShiftThisFrame = maxLateralRate * dt;
    this.smoothedTargetOffset = clamp(
      (this.smoothedTargetOffset != null ? this.smoothedTargetOffset : currentCarLat) +
        clamp(targetOffset - (this.smoothedTargetOffset != null ? this.smoothedTargetOffset : currentCarLat), -maxShiftThisFrame, maxShiftThisFrame),
      -plannedRoadMargin,
      plannedRoadMargin
    );
    targetOffset = this.smoothedTargetOffset;

    // 4. Multi-Candidate Frenet Trajectory Planning
    const tacticalCandidates = (tacticalMode === 'PACE' || recovering) ? [] : [
      { offset: targetOffset, intentType: 'TACTICAL_TARGET', transitionScales: [0.75, 1.0, 1.3] },
      { offset: paceLine, intentType: 'RACING_LINE', transitionScales: [1.0, 1.5] },
      { offset: plannedRoadMargin * 0.65, intentType: 'RIGHT_OPEN_LANE', transitionScales: [0.8, 1.1] },
      { offset: -plannedRoadMargin * 0.65, intentType: 'LEFT_OPEN_LANE', transitionScales: [0.8, 1.1] }
    ];

    const lookAheadDist = recovering
      ? clamp(10.0 + vehicle.speed * 0.42, 10.0, 20.0)
      : clamp(11.0 + vehicle.speed * 0.58, 12.0, 30.0);

    const kappa = Math.abs(finite(currentCurv, 0));
    const trackingDistance = clamp(lookAheadDist * 0.72 / (1.0 + kappa * 8.0), 8.5, 24.0);

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
      fallbackOffsets: recovering ? [] : [paceLine, finite(current?.lateral, 0)],
      tacticalCandidates: recovering ? [] : tacticalCandidates,
      trafficEntries: traffic.entries,
      targetSpeed: physicalTargetSpeed,
      aggression: this._aggression,
      racecraftPhase: defending ? defDecision.phase : attDecision.phase,
      targetId,
      recovering,
      pitActive: Boolean(vehicle.pitIntent?.active),
      urgent: committed || defending || isOffTrack,
      roadMargin: plannedRoadMargin,
      kerbAllowance,
      lookAhead: lookAheadDist,
      trackingDistance,
      referenceLineAtDistance: (s) => {
        if (isMatchingTrack && typeof this.referenceProfile?.paceAtDistance === 'function') {
          return this.referenceProfile.paceAtDistance(s)?.lineLateral ?? 0;
        }
        const pt = track?.atDistance ? track.atDistance(s) : { curvature: 0 };
        const curv = finite(pt.curvature, 0);
        return clamp(-Math.sign(curv) * Math.min(2.4, Math.abs(curv) * 35.0), -2.5, 2.5);
      }
    });

    const trackingPoint = this.trajectoryPlan.trackingPoint ?? this.trajectoryPlan.points.at(-1);
    const plannedTargetOffset = finite(trackingPoint?.lateral, targetOffset);
    const targetPos = {
      x: finite(trackingPoint?.x, vehicle.position.x),
      y: finite(trackingPoint?.y, vehicle.position.y),
      z: finite(trackingPoint?.z, vehicle.position.z),
      lateral: plannedTargetOffset
    };

    // 5. Lateral Pursuit Steering & Orientation-Aware Rejoin
    const trackPointAtCar = track?.atDistance ? track.atDistance(vehicle.distance) : { tangent: { x: 0, z: 1 } };
    const trackHeadingAtCar = Math.atan2(trackPointAtCar.tangent.x, trackPointAtCar.tangent.z);
    const yawAlignment = wrapAngle(vehicle.yaw - trackHeadingAtCar);
    const isFacingBackwards = Math.abs(yawAlignment) > Math.PI * 0.78;

    const currentLateralVal = finite(current?.lateral, 0);
    let headingError = wrapAngle(
      Math.atan2(targetPos.x - vehicle.position.x, targetPos.z - vehicle.position.z) - vehicle.yaw
    );

    if (recovering || isOffTrack) {
      const rejoinPt = track?.atDistance ? track.atDistance(vehicle.distance + 14.0) : trackPointAtCar;
      const toTrackFwdX = finite(rejoinPt.x, 0) - vehicle.position.x;
      const toTrackFwdZ = finite(rejoinPt.z, 0) - vehicle.position.z;
      const targetRejoinAngle = Math.atan2(toTrackFwdX, toTrackFwdZ);
      headingError = wrapAngle(targetRejoinAngle - vehicle.yaw);
    }

    if (isFacingBackwards) {
      headingError = Math.sign(yawAlignment) * -1.2;
    }

    const lateralError = currentLateralVal - plannedTargetOffset;

    const liveSlip = Math.atan2(
      finite(vehicle.localVelocity?.x, 0),
      Math.max(3.0, Math.abs(finite(vehicle.localVelocity?.z, vehicle.speed)))
    );

    this.steerCommand = this.paceOptimizer.computeSteering({
      previous: this.steerCommand,
      headingError,
      lateralError,
      yawRate: vehicle.yawRate,
      slipAngle: liveSlip,
      speed: vehicle.speed,
      currentCurvature: currentCurv,
      dt,
      committed,
      recovering
    });

    // 6. Longitudinal Target Speed & Dynamic Adjustments
    let desiredSpeed = physicalTargetSpeed;

    const speedAero = vehicle.classKey === 'prototype' ? clamp((vehicle.speed - 16.0) / 32.0, 0, 1) : 0;
    const baseClassG = (vehicle.classKey === 'prototype' ? (1.80 + 0.85 * speedAero) : vehicle.classKey === 'gt' ? 1.25 : 0.98) * 9.81;
    const lateralAccelBudget = baseClassG * tireGripFactor * (committed ? 1.08 : 1.0);
    const decelBudget = (vehicle.classKey === 'prototype' ? 14.5 : vehicle.classKey === 'gt' ? 9.8 : 6.8) * tireGripFactor;
    const trajectorySpeedLimit = this.trajectoryPlan.points.reduce((limit, p) => {
      const fwd = Math.max(0, finite(p.forwardDistance, 0));
      if (fwd < 2.0) return limit;
      const curv = Math.max(0, finite(p.curvature, 0));
      if (curv < 1e-4) return limit;
      const cornerSpeed = Math.sqrt(lateralAccelBudget / curv);
      const reachableSpeed = Math.sqrt(cornerSpeed * cornerSpeed + 2.0 * decelBudget * fwd);
      return Math.min(limit, reachableSpeed);
    }, physicalTargetSpeed);

    desiredSpeed = Math.min(desiredSpeed, trajectorySpeedLimit);

    const refData = (activeProfile && typeof activeProfile.paceAtDistance === 'function')
      ? activeProfile.paceAtDistance(vehicle.distance)
      : null;
    const classFactor = (vehicle.classKey === 'gt') ? 0.82 : (vehicle.classKey === 'touring' ? 0.70 : 1.0);
    const rawRefSpeed = refData?.targetSpeed;
    const refSpeed = Number.isFinite(rawRefSpeed) ? rawRefSpeed * classFactor : null;
    const scaledRef = (refSpeed != null && refSpeed > 8.0)
      ? refSpeed * (1.0 + (this._aggression - 0.5) * 0.08)
      : null;

    if (scaledRef != null && tacticalMode === 'PACE') {
      desiredSpeed = Math.max(desiredSpeed, Math.min(physicalTargetSpeed * 1.05, scaledRef));
    }

    const passTarget = attDecision.target;
    const actualSeparation = passTarget
      ? Math.abs(finite(current?.lateral, 0) - finite(passTarget.otherLateral, 0))
      : 99;

    if (committed && passTarget) {
      const isCornerApproach = Boolean(attDecision.inCorner) || (attDecision.distToCorner != null && attDecision.distToCorner < 85) || Math.abs(currentCurv) > 0.003;
      const straightClosingFloor = 14.0 + clamp(this._aggression, 0, 1) * 4.0;
      const cornerClosingFloor = Math.max(4.5, 7.5 * this._aggression);
      const isOverlappingLane = actualSeparation < 1.6 && passTarget.delta < 14.0;
      const closingFloor = (straightSend && !isCornerApproach && !isOverlappingLane) ? straightClosingFloor : cornerClosingFloor;
      const isSlowObstacle = passTarget.other.speed < 16.0;
      const obstacleFloor = isSlowObstacle ? Math.min(physicalTargetSpeed, Math.max(14.0, passTarget.other.speed + 10.0)) : 0;

      if (straightSend && !isCornerApproach && !isOverlappingLane) {
        desiredSpeed = Math.min(physicalTargetSpeed, Math.max(desiredSpeed, passTarget.other.speed + closingFloor));
      } else {
        if (isOverlappingLane) {
          const followFloor = Math.max(obstacleFloor, passTarget.other.speed + clamp((passTarget.delta - 6.5) * 0.75, -6.0, 3.0));
          desiredSpeed = Math.min(desiredSpeed, followFloor);
        } else {
          const cornerFloor = Math.min(physicalTargetSpeed, passTarget.other.speed + cornerClosingFloor);
          desiredSpeed = Math.min(physicalTargetSpeed, Math.max(obstacleFloor, cornerFloor));
        }
      }
    } else if (passTarget && passTarget.delta > 0 && passTarget.delta < 45 && !defending) {
      const isSlowObstacle = passTarget.other.speed < 16.0;
      const isAttackingPhase = attDecision.phase !== 'NONE' && attDecision.phase !== 'RETURN';
      const isCornerApproach = Boolean(attDecision.inCorner) || (attDecision.distToCorner != null && attDecision.distToCorner < 85) || Math.abs(currentCurv) > 0.003;

      if ((isAttackingPhase || this._aggression > 0.8) && !isCornerApproach) {
        const straightClosingFloor = 14.0 + clamp(this._aggression, 0, 1) * 4.0;
        desiredSpeed = Math.min(physicalTargetSpeed, Math.max(desiredSpeed, passTarget.other.speed + straightClosingFloor));
      } else if (isSlowObstacle) {
        const escapeSpeed = Math.min(physicalTargetSpeed, Math.max(14.0, passTarget.other.speed + 10.0));
        desiredSpeed = Math.min(desiredSpeed, Math.max(escapeSpeed, passTarget.other.speed + 6.0));
      } else {
        const safeGap = clamp(7.0 + passTarget.relativeLongitudinalVelocity ** 2 / 10.0, 7.5, 30.0);
        const followSpeedFloor = Math.max(10.0, passTarget.other.speed - 4.5);
        desiredSpeed = Math.min(
          desiredSpeed,
          Math.max(followSpeedFloor, passTarget.other.speed + clamp((passTarget.delta - safeGap) * 0.35, -5.0, 3.0))
        );
      }
    }

    if (recovering) desiredSpeed = isOffTrack ? (isFacingBackwards ? 5.0 : 8.5) : 14.0;

    // 7. Hazard Avoidance
    const hazard = this.awareness.forwardHazard(traffic);
    const passTargetCombat = (committed || defending) && hazard?.other?.id === (passTarget?.other?.id ?? this.defenseEngine.defenseTargetId);
    const emergency = Boolean(hazard && !passTargetCombat && (hazard.ttc < 2.2 || (hazard.longitudinal < 6.0 && Math.abs(finite(current?.lateral, 0) - finite(hazard.otherLateral, 0)) < 1.6)));

    if (emergency) {
      desiredSpeed = Math.min(desiredSpeed, Math.max(0, hazard.other.speed - 2.5));
    }

    // Never exceed physical cornering limit or global braking envelope
    desiredSpeed = Math.min(desiredSpeed, trajectorySpeedLimit);
    if (scaledRef != null) {
      const diveBonus = (committed || attDecision.divebombing || straightSend) ? 1.25 : 1.02;
      const envelopeCap = scaledRef * diveBonus;
      desiredSpeed = Math.min(desiredSpeed, envelopeCap);
    }

    // 8. Low-Level Pedal Control & Trail Braking
    const speedError = desiredSpeed - vehicle.speed;
    const straight = Math.abs(signedCurv) < 0.0030;
    const liveLatAccel = Math.abs(finite(vehicle.speed, 0) * finite(vehicle.yawRate, 0));

    const pedals = this.paceOptimizer.computePedals({
      vehicle,
      speedError,
      desiredSpeed,
      headingError,
      lateralAccel: liveLatAccel,
      steerAngle: this.steerCommand,
      yawRate: vehicle.yawRate,
      slipAngle: liveSlip,
      currentCurvature: signedCurv,
      straight,
      recovering,
      emergency,
      defending,
      tireGripFactor,
      dt
    });

    const dynamicControls = this.combatDynamics.process({
      vehicle,
      traffic,
      controls: {
        steer: clamp(this.steerCommand, -1, 1),
        throttle: pedals.throttle,
        brake: pedals.brake
      },
      dt
    });

    vehicle.controls = {
      steer: dynamicControls.steer,
      throttle: dynamicControls.throttle,
      brake: dynamicControls.brake,
      handbrake: 0
    };
    this.steerCommand = dynamicControls.steer;

    // Set target and tactical state handles on vehicle
    vehicle.aiTarget = { x: targetPos.x, z: targetPos.z, lateral: plannedTargetOffset };
    vehicle.aiTactical = {
      targetLaneOffsetM: plannedTargetOffset,
      racecraftPhase: defending ? defDecision.phase : (attDecision.phase !== 'NONE' ? attDecision.phase : tacticalMode),
      passPhase: attDecision.phase,
      passTargetId: targetId,
      defending,
      defenseTargetId: defDecision.target?.other?.id ?? null,
      desiredSpeed
    };

    // 9. ERS Deployment
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

    // 10. Diagnostics and Debug State Recording
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
      latAccel: liveLatAccel,
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
    const clearance = finite(this.trajectoryPlan?.minimumClearanceM, 99);
    const isSafe = Boolean(this.trajectoryPlan?.collisionFree ?? true);
    const speedError = finite(desiredSpeed - vehicle.speed);
    const defending = Boolean(defDecision.defending);
    const attacking = tacticalMode === 'ATTACK';

    this.debugState = {
      controllerVersion: 'Research-V1-Hybrid',
      vehicleId: vehicle.id,
      name: vehicle.name,
      classKey: vehicle.classKey,
      mode: tacticalMode,
      racecraftPhase: defending ? defDecision.phase : attDecision.phase,
      reason: tacticalReason,
      decisionReason: tacticalReason,
      targetId: target?.other?.id ?? null,
      passTargetId: attDecision.target?.other?.id ?? null,
      draftTargetId: attDecision.draftTarget?.other?.id ?? null,
      defenseTargetId: defDecision.target?.other?.id ?? null,
      desiredOffset: finite(targetOffset),
      targetOffset: finite(targetOffset),
      desiredSpeed: finite(desiredSpeed),
      targetSpeed: finite(desiredSpeed),
      currentSpeed: finite(vehicle.speed),
      speedKmh: finite(vehicle.speed * 3.6),
      speedError,
      headingError: finite(headingError),
      lateralError: finite(lateralError),
      recovering: Boolean(recovering),
      
      // Candidate Trajectory Lattice
      candidates: this.trajectoryPlan?.candidates ?? [],
      bestCandidate: this.trajectoryPlan,
      trajectory: this.trajectoryPlan,
      trajectoryMinimumClearanceM: clearance,
      trajectoryCollisionFree: isSafe,
      trajectoryRoadLegal: Boolean(this.trajectoryPlan?.roadLegal ?? true),
      trajectorySelectedOffsetM: finite(this.trajectoryPlan?.selectedOffset, finite(targetOffset, 0)),
      trajectoryScore: finite(this.trajectoryPlan?.score, 0),

      // Multi-Agent Awareness
      traffic,
      hazard,
      
      // Threat Perception
      threat: {
        challengerId: challenger?.other?.id ?? null,
        threatLevel: defDecision.threatLevel ?? (defending ? 'HIGH' : 'NONE'),
        threatScore: finite(defDecision.threatScore, defending ? 0.85 : 0),
        attackerIntent: defDecision.attackerIntent ?? 'NONE',
        gapM: finite(challenger?.delta, 99),
        closingSpeedMps: finite(challenger?.relativeLongitudinalVelocity, 0),
        ttc: finite(challenger?.ttc, 99)
      },

      // Floating Thought Billboard & Summary
      thought: {
        requestedManeuver: tacticalMode === 'ATTACK' ? attDecision.phase : (tacticalMode === 'DEFEND' ? defDecision.phase : tacticalMode),
        deployedManeuver: tacticalMode === 'ATTACK' ? attDecision.phase : (tacticalMode === 'DEFEND' ? defDecision.phase : tacticalMode),
        deployedOffsetM: finite(targetOffset),
        targetId: target?.other?.id ?? 'CLEAR',
        abortReason: 'CLEAR',
        waitReason: 'CLEAR',
        committed: Boolean(attDecision.committed),
        defending,
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
        trajectoryCollisionFree: isSafe,
        trajectoryRoadLegal: Boolean(this.trajectoryPlan?.roadLegal ?? true),
        trajectorySelectedOffsetM: finite(this.trajectoryPlan?.selectedOffset, finite(targetOffset, 0)),
        trajectoryMinimumClearanceM: clearance
      },

      // G-G Friction Circle & Real-Time Control Telemetry
      telemetry: {
        lateralAccelMps2: finite(latAccel),
        lateralUtilization: finite(pedals.friction?.latUtilization ?? 0),
        maxG: finite(pedals.friction?.maxTotalAccel ? pedals.friction.maxTotalAccel / 9.81 : 2.70, 2.70),
        trailBrakingActive: Boolean(pedals.trailBraking),
        throttle: finite(pedals.throttle),
        brake: finite(pedals.brake),
        steer: finite(this.steerCommand),
        ersMode,
        ersSoc: finite(vehicle.ers?.soc, 0),
        yawRate: finite(vehicle.yawRate, 0),
        emergency: Boolean(emergency),
        rubbingActive: Boolean(this.combatDynamics?.rubbingActive),
        powerSlideActive: Boolean(this.combatDynamics?.powerSlideActive),
        state: tacticalMode,
        action: attacking ? (attDecision.divebombing ? 'DIVEBOMB ATTACK' : 'SLINGSHOT PASS') : (defending ? `DEFEND (${defDecision.phase})` : 'PACE CRUISE'),
        reason: tacticalReason,
        threatLevel: defDecision.threatLevel ?? (defending ? 'HIGH' : 'LOW'),
        tactic: attacking ? 'Iterative Best Response (IBR)' : (defending ? 'Stackelberg Apex Shield' : 'Coupled Friction-Circle Pace'),
        prediction: `Clearance: ${clearance < 90 ? clearance.toFixed(1) + 'm' : 'CLEAR'} // Safe: ${isSafe}`,
        decision: tacticalReason,
        latUtilization: finite(pedals.friction?.latUtilization ?? 0),
        remainingLongBudget: finite(pedals.friction?.remainingLongBudget ?? 1),
        liveLatG: finite(latAccel / 9.81),
        peakLatG: finite(pedals.friction?.peakG ?? 2.70, 2.70)
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
