/**
 * NextGenAIController.js (V2 Main Controller Orchestrator)
 * Unites the 4-Layer Hybrid AI Architecture:
 * - Layer 1: 2D Free-Boundary Optimal Profile Solver (GlobalTimeOptimalEngine)
 * - Layer 2: Stackelberg Leader Defense & IBR Divebomb Attack (GameTheoreticCombatEngine)
 * - Layer 3: Coupled Friction-Circle Trail-Braking & Apex Exit Power (CoupledMPCC + PaceOptimizer + Lattice)
 * - Layer 4: Combat Dynamics: Contact-Tolerant Elastic Rubbing & Slip-Slope Power Sliding (CombatDynamicsEngine)
 * - Perception: Multi-agent spatial awareness & swept corridor hazard scanning (TrafficAwareness)
 * - Telemetry: Full export of candidate lattice matrices, G-G friction states, and thought labels for AIDebugSuiteRenderer
 */

import { TrafficAwareness } from '../TrafficAwareness.js';
import { GlobalTimeOptimalEngine } from './GlobalTimeOptimalEngine.js';
import { GameTheoreticCombatEngine } from './GameTheoreticCombatEngine.js';
import { CoupledMPCCController } from './CoupledMPCCController.js';
import { CombatDynamicsEngine } from './CombatDynamicsEngine.js';
import { FrenetLatticePlanner } from '../FrenetLatticePlanner.js';
import { PaceOptimizer } from '../PaceOptimizer.js';
import { clamp, wrap, wrapAngle, damp } from '../../core/math.js';

const finite = (val, fallback = 0) => (Number.isFinite(val) ? val : fallback);
const offRoad = (c) => c?.zone === 'grass' || c?.zone === 'runoff';

export class NextGenAIController {
  /**
   * @param {number|string} [id=1] - Controller / Car identifier
   * @param {Object} [options]
   */
  constructor(id = 1, options = {}) {
    this.id = id;
    this.index = typeof id === 'number' ? id : 1;

    // Tunable AI Heuristic Parameters
    this._aggression = clamp(finite(options.aggression, 0.90), 0, 1);
    this._diveMargin = clamp(finite(options.diveMargin, 0.85), 0, 1);
    this._defenseReactivity = clamp(finite(options.defenseReactivity, 0.92), 0, 1);
    this._kerbUsage = clamp(finite(options.kerbUsage, 0.90), 0, 1);
    this._lookahead = clamp(finite(options.lookahead, 24.0), 7.5, 30);
    this._trailBrakingSkill = clamp(finite(options.trailBrakingSkill, 0.95), 0, 1);
    this._ersAttackMode = Boolean(options.ersAttackMode ?? false);

    this.skill = clamp(finite(options.skill, 0.98), 0.5, 1.0);

    // Perception & Hybrid Engine Layers
    this.awareness = new TrafficAwareness();
    this.optimalEngine = options.track ? new GlobalTimeOptimalEngine({ track: options.track }) : null;
    this.combatEngine = new GameTheoreticCombatEngine();
    this.coupledMPCC = new CoupledMPCCController({ horizonSeconds: 2.5, nodeCount: 18 });
    this.trajectoryPlanner = new FrenetLatticePlanner({ pointCount: 24, horizonS: 3.2 });
    this.paceOptimizer = new PaceOptimizer({
      trailBrakingSkill: this._trailBrakingSkill,
      unwindFactor: 0.62
    });
    this.dynamicsEngine = new CombatDynamicsEngine();

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

    // Recovery, stall detection, and telemetry tracking
    this.recoveryTimer = 0;
    this.stallTime = 0;
    this.lastDistance = null;
    this.steerCommand = 0;
    this.humanSteer = 0;
    this.smoothedOffset = 0;
    this.supervisor = null;
    this.marshalRecoveries = 0;
    this.referenceProfile = null;
    this.debugEnabled = true;
    this.debugState = null;
    this.trajectoryPlan = null;

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
  }

  get diveMargin() { return this._diveMargin; }
  set diveMargin(val) {
    this._diveMargin = clamp(finite(val, 0.6), 0, 1);
  }

  get defenseReactivity() { return this._defenseReactivity; }
  set defenseReactivity(val) {
    this._defenseReactivity = clamp(finite(val, 0.8), 0, 1);
  }

  get kerbUsage() { return this._kerbUsage; }
  set kerbUsage(val) {
    this._kerbUsage = clamp(finite(val, 0.8), 0, 1);
  }

  get lookahead() { return this._lookahead; }
  set lookahead(val) {
    this._lookahead = clamp(finite(val, 24.0), 7.5, 30);
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

  computeLookahead(speed = 0, kappa = 0) {
    const v = Math.max(0, finite(speed, 0));
    const k = Math.abs(finite(kappa, 0));
    return clamp((v * 0.36) / (1.0 + 90.0 * k), 7.5, 26.0);
  }

  resetForRace(vehicle = null) {
    this.combatEngine = new GameTheoreticCombatEngine({
      roadHalfWidth: finite(this.optimalEngine?.roadHalfWidth, 8.2),
      curbWidth: finite(this.optimalEngine?.curbWidth, 1.25)
    });
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
  _planERS(vehicle, track, { committed = false, defending = false, straight = false, throttle = 0 } = {}) {
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
    const defensiveDeploy = defending;
    const paceSurplusDeploy = (straight && throttle > 0.85 && surplus > 0.01)
      || (lapProgress > 0.80 && throttle > 0.88 && surplus > 0.015);

    const shouldDeploy = canDeploy && (offensiveDeploy || defensiveDeploy || paceSurplusDeploy);

    plan.targetSoc = targetSoc;
    plan.mode = shouldDeploy ? 'ATTACK' : 'AUTO';
    return plan.mode;
  }

  /**
   * Main 120Hz update cycle.
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

    // 1. Initialize Layer 1 Optimal Profile Engine for this circuit if needed
    if (!this.optimalEngine || this.optimalEngine.track !== track) {
      this.optimalEngine = new GlobalTimeOptimalEngine({ track });
    }

    // 2. Scan multi-agent traffic awareness & surface state
    const traffic = this.awareness.scan(vehicle, vehicles, track);
    const current = traffic.current;
    const isOffTrack = offRoad(current) || offRoad(vehicle.surface);

    const nominalHalfWidth = finite(track.roadHalfWidth, 6.5);
    const kerbAllowance = Math.min(0.8, finite(track.curbWidth, 1.05) * 0.65) * this._kerbUsage;
    const baseRoadMargin = Math.max(2.1, Math.min(5.35, nominalHalfWidth - 1.20 + kerbAllowance));
    const currentSurfaceMargin = finite(
      track.planningLateralLimit?.(vehicle.distance, current?.lateral),
      nominalHalfWidth - 1.20
    );
    const edgeDeviation = Math.abs(finite(current?.lateral, 0)) > currentSurfaceMargin + 0.55;

    // Progress & Stall Detection
    if (this.lastDistance === null) this.lastDistance = vehicle.distance;
    const progress = wrap(vehicle.distance - this.lastDistance + track.length * 0.5, track.length) - track.length * 0.5;
    this.lastDistance = vehicle.distance;
    const queued = traffic.ahead && traffic.ahead.delta < 12;
    this.stallTime = vehicle.speed < 2.2 && progress < 0.25 && !queued
      ? this.stallTime + dt
      : Math.max(0, this.stallTime - dt * 2);

    if (isOffTrack) this.recoveryTimer = 1.2;
    else this.recoveryTimer = Math.max(0, this.recoveryTimer - dt);

    const recovering = isOffTrack || this.recoveryTimer > 0 || this.stallTime > 0.7;

    // Reverse recovery maneuver if pinned against barrier or stuck in runoff
    if (isOffTrack && vehicle.speed < 1.6) {
      this.stuckTimer = (this.stuckTimer || 0) + dt;
      if (this.stuckTimer > 1.2 && (this.reverseDuration || 0) <= 0) {
        this.reverseDuration = 2.0;
        this.stuckTimer = 0;
      }
    } else {
      this.stuckTimer = Math.max(0, (this.stuckTimer || 0) - dt * 2);
    }

    if ((this.reverseDuration || 0) > 0) {
      this.reverseDuration -= dt;
      const latSign = Math.sign(finite(current?.lateral, 0)) || 1;
      vehicle.controls = {
        throttle: 0.45,
        brake: 0,
        steer: clamp(latSign * 0.75, -1, 1),
        handbrake: 0,
        reverse: true
      };
      return;
    }

    // Marshal recovery safeguard if stuck off-track
    if (isOffTrack && this.stallTime > 5.0 && vehicle.marshalRecoverTo) {
      vehicle.marshalRecoverTo(track, vehicle.distance + 10, 0);
      this.marshalRecoveries += 1;
      this.stallTime = 0;
      this.recoveryTimer = 1.0;
      return;
    }

    // 3. Layer 2: Game-Theoretic Adversarial Corridor Planning
    const tactical = this.combatEngine.evaluate({
      vehicle,
      track,
      traffic,
      optimalProfile: this.optimalEngine,
      aggression: this._aggression,
      dt
    });

    const optCurrent = this.optimalEngine?.sampleAtDistance?.(vehicle.distance, vehicle.classKey);
    const defending = tactical.role === 'DEFEND' || tactical.role === 'DUAL_COMBAT' || tactical.defenseMode !== 'PACE';
    const attacking = tactical.role === 'ATTACK' || tactical.role === 'DUAL_COMBAT' || tactical.attackMode !== 'NONE';
    let tacticalMode = recovering ? 'RECOVER' : (tactical.role === 'DUAL_COMBAT' ? 'DUAL_COMBAT' : (defending ? 'DEFEND' : (attacking ? 'ATTACK' : 'PACE')));
    let targetOffset = recovering ? 0 : (tactical.role === 'PACE' ? (optCurrent?.lateral ?? 0) : clamp(tactical.targetLateral, -baseRoadMargin, baseRoadMargin));
    let targetId = attacking ? this.combatEngine.attackTargetId : (defending ? this.combatEngine.defenseTargetId : null);
    let tacticalReason = recovering ? (isOffTrack ? 'OFF_TRACK_RECOVERY' : 'STALL_RECOVERY') : tactical.notes;

    if (vehicle.pitIntent?.active) {
      tacticalMode = 'PIT';
      targetOffset = finite(vehicle.pitIntent.targetLateralM, targetOffset);
      tacticalReason = 'PIT_LANE_ENTRY';
    }

    // 4. Multi-Candidate Frenet Lattice Trajectory Planning
    const currentPoint = track?.atDistance ? track.atDistance(vehicle.distance) : { curvature: 0 };
    const signedCurv = finite(currentPoint?.curvature, 0);
    const currentCurv = Math.abs(signedCurv);

    const dynamicLookahead = this.computeLookahead(vehicle.speed, currentCurv);
    const lookAheadDist = recovering
      ? clamp(10.0 + vehicle.speed * 0.42, 10.0, 20.0)
      : clamp(Math.max(dynamicLookahead, 11.0 + vehicle.speed * 0.58), 12.0, 30.0);

    const trackingDistance = clamp(lookAheadDist * 0.72 / (1.0 + currentCurv * 20.0), 5.5, 24.0);

    const maxTireWear = Math.max(0, ...(vehicle.wheels ?? []).map((w) => finite(w.wear, 0)));
    const tireGripFactor = clamp(1.0 - maxTireWear * 0.45, 0.80, 1.0);

    // Layer 1 Free-Boundary Speed Reachability Envelope
    const physicalTargetSpeed = this.paceOptimizer.computeSpeedEnvelope({
      vehicle,
      track,
      tireGripFactor,
      skill: this.skill,
      aggression: this._aggression,
      defending,
      insideLineOffset: targetOffset
    });

    const isMatchingTrack = Boolean(this.referenceProfile) && (
      this.referenceProfile.trackId
        ? this.referenceProfile.trackId === track?.id
        : Math.abs((this.referenceProfile?.trackLength || 3061.7) - (track?.length || 1000)) < 100
    );

    // 4. Multi-Rate Decoupled Trajectory Lattice Evaluation (25Hz / Phase-Triggered)
    this.planTimer = (this.planTimer || 0) + dt;
    const racecraftPhase = defending ? tactical.defenseMode : (attacking ? tactical.attackMode : 'NONE');
    const phaseChanged = (this.lastRacecraftPhase !== racecraftPhase);
    this.lastRacecraftPhase = racecraftPhase;

    const shouldReplan = !this.trajectoryPlan
      || phaseChanged
      || defending
      || attacking
      || isOffTrack
      || this.planTimer >= 0.04;

    if (shouldReplan) {
      this.planTimer = (this.index % 4) * (0.04 / 4); // time-slice phase offset across cars
      this.trajectoryPlan = this.trajectoryPlanner.plan({
        vehicle,
        track,
        desiredOffset: targetOffset,
        fallbackOffsets: (recovering || defending || attacking) ? [] : [targetOffset],
        trafficEntries: traffic.entries,
        targetSpeed: physicalTargetSpeed,
        aggression: this._aggression,
        racecraftPhase,
        targetId,
        recovering,
        pitActive: Boolean(vehicle.pitIntent?.active),
        urgent: defending || attacking || isOffTrack,
        roadMargin: baseRoadMargin,
        kerbAllowance,
        trackingDistance,
        referenceLineAtDistance: (s) => {
          if (isMatchingTrack && typeof this.referenceProfile?.paceAtDistance === 'function') {
            return this.referenceProfile.paceAtDistance(s, vehicle.classKey)?.lineLateral ?? 0;
          }
          const optLat = this.optimalEngine?.sampleAtDistance?.(s, vehicle.classKey)?.lateral;
          return Number.isFinite(optLat) ? clamp(optLat, -baseRoadMargin, baseRoadMargin) : 0;
        }
      });
    }

    const trackingPoint = this.trajectoryPlan.trackingPoint ?? this.trajectoryPlan.points.at(-1);
    const plannedTargetOffset = finite(trackingPoint?.lateral, targetOffset);
    let targetPos = {
      x: finite(trackingPoint?.x, vehicle.position.x),
      y: finite(trackingPoint?.y, vehicle.position.y),
      z: finite(trackingPoint?.z, vehicle.position.z),
      lateral: plannedTargetOffset
    };

    if (recovering) {
      const rejoinDistance = wrap(vehicle.distance + (isOffTrack ? 10.0 : 14.0), track.length);
      const rejoinPoint = track.atDistance ? track.atDistance(rejoinDistance) : trackPointAtCar;
      const rejoinWorld = track.lateralPoint ? track.lateralPoint(rejoinPoint, 0, 0) : rejoinPoint;
      targetPos = {
        x: finite(rejoinWorld?.x, vehicle.position.x),
        y: vehicle.position.y,
        z: finite(rejoinWorld?.z, vehicle.position.z),
        lateral: 0
      };
    }

    // 5. Lateral Pursuit Steering & Orientation-Aware Rejoin
    const trackPointAtCar = track?.atDistance ? track.atDistance(vehicle.distance) : { tangent: { x: 0, z: 1 } };
    const trackHeadingAtCar = Math.atan2(trackPointAtCar.tangent.x, trackPointAtCar.tangent.z);
    const yawAlignment = wrapAngle(vehicle.yaw - trackHeadingAtCar);
    const isFacingBackwards = Math.abs(yawAlignment) > Math.PI * 0.65;

    let headingError = wrapAngle(
      Math.atan2(targetPos.x - vehicle.position.x, targetPos.z - vehicle.position.z) - vehicle.yaw
    );

    if (recovering && isFacingBackwards) {
      headingError = Math.sign(yawAlignment) * -1.2;
    }

    const lateralError = finite(current?.lateral, 0) - plannedTargetOffset;

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
      currentCurvature: signedCurv,
      dt,
      committed: defending || attacking,
      recovering
    });

    // 6. Dynamic Multi-Agent Physics Collision & Track-Edge Supervisor
    const supervisor = (vehicles && vehicles.length > 1)
      ? this._superviseTraffic(vehicle, vehicles, track, current, dt)
      : { maxSpeed: Infinity, emergency: false, reason: 'CLEAR', nearestAheadDist: Infinity, following: false };
    this.supervisor = supervisor;

    let desiredSpeed = physicalTargetSpeed;
    if (tactical.desiredSpeed) {
      desiredSpeed = Math.min(physicalTargetSpeed, Math.max(desiredSpeed, tactical.desiredSpeed));
    }
    desiredSpeed = Math.min(desiredSpeed, supervisor.maxSpeed);

    if (recovering) desiredSpeed = isOffTrack ? (isFacingBackwards ? 5.0 : 8.5) : Math.max(16.0, physicalTargetSpeed * 0.75);

    // 7. Friction-Circle-Coupled Pedal Computation
    const speedError = desiredSpeed - vehicle.speed;
    const straight = Math.abs(signedCurv) < 0.0030;
    const liveLatAccel = Math.abs(finite(vehicle.speed, 0) * finite(vehicle.yawRate, 0));

    const rawPedals = this.paceOptimizer.computePedals({
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
      emergency: supervisor.emergency,
      defending,
      following: supervisor.following && !attacking && !defending,
      tireGripFactor,
      dt
    });

    // 8. Layer 4: Combat Dynamics & Slip-Slope Limit Tracking
    const finalControls = this.dynamicsEngine.process({
      vehicle,
      traffic,
      controls: {
        steer: clamp(this.steerCommand, -1, 1),
        throttle: rawPedals.throttle,
        brake: rawPedals.brake
      },
      dt
    });

    // Apply finalized controls to vehicle
    vehicle.controls = {
      steer: finalControls.steer,
      throttle: finalControls.throttle,
      brake: finalControls.brake,
      handbrake: 0
    };
    this.steerCommand = finalControls.steer;

    // Update target handles on vehicle object for external perception
    vehicle.aiTarget = { x: targetPos.x, z: targetPos.z, lateral: plannedTargetOffset };
    vehicle.aiTactical = {
      targetLaneOffsetM: plannedTargetOffset,
      racecraftPhase: defending ? tactical.defenseMode : (attacking ? tactical.attackMode : 'PACE'),
      passPhase: tactical.attackMode,
      passTargetId: this.combatEngine.attackTargetId,
      defending,
      defenseTargetId: this.combatEngine.defenseTargetId,
      desiredSpeed
    };

    // 9. ERS Deployment Planning
    const ersMode = vehicle.classKey === 'prototype'
      ? this._planERS(vehicle, track, {
          committed: attacking,
          defending,
          straight,
          throttle: finalControls.throttle
        })
      : 'OFF';

    if (vehicle.classKey === 'prototype') vehicle.setERSMode?.(ersMode);

    // 10. Record Complete Diagnostics & Debug Telemetry
    this._recordDebugState(vehicle, {
      tacticalMode,
      tacticalReason,
      tactical,
      desiredSpeed,
      targetOffset: plannedTargetOffset,
      targetPos,
      headingError,
      lateralError,
      traffic,
      recovering,
      pedals: rawPedals,
      finalControls,
      latAccel: liveLatAccel,
      ersMode,
      straight
    });
  }

  /**
   * Build complete tactical diagnostics snapshot for 3D debug rendering and telemetry.
   * @private
   */
  _recordDebugState(vehicle, {
    tacticalMode,
    tacticalReason,
    tactical,
    desiredSpeed,
    targetOffset,
    targetPos,
    headingError,
    lateralError,
    traffic,
    recovering,
    pedals,
    finalControls,
    latAccel,
    ersMode,
    straight
  }) {
    if (!this.debugEnabled) return;

    const challenger = traffic?.behind;
    const targetId = tactical.attackMode !== 'NONE' ? this.combatEngine.attackTargetId : (tactical.defenseMode !== 'PACE' ? this.combatEngine.defenseTargetId : null);
    const defending = tactical.role === 'DEFEND' || tactical.role === 'DUAL_COMBAT' || tactical.defenseMode !== 'PACE';
    const attacking = tactical.role === 'ATTACK' || tactical.role === 'DUAL_COMBAT' || tactical.attackMode !== 'NONE';
    const speedError = finite(desiredSpeed - vehicle.speed);
    const clearance = finite(this.trajectoryPlan?.minimumClearanceM, 99);
    const isSafe = Boolean(this.trajectoryPlan?.collisionFree ?? true);

    this.debugState = {
      controllerVersion: 'V2-NextGen-Hybrid',
      vehicleId: vehicle.id,
      name: vehicle.name,
      classKey: vehicle.classKey,
      mode: tacticalMode,
      racecraftPhase: defending ? tactical.defenseMode : (attacking ? tactical.attackMode : 'OPTIMAL_LINE'),
      reason: tacticalReason,
      decisionReason: tacticalReason,
      targetId,
      passTargetId: this.combatEngine.attackTargetId,
      draftTargetId: this.combatEngine.attackTargetId,
      defenseTargetId: this.combatEngine.defenseTargetId,
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

      // Multi-Agent Traffic Awareness
      traffic,

      // Adversarial Threat Matrix
      threat: {
        challengerId: challenger?.other?.id ?? null,
        threatLevel: defending ? 'HIGH' : (challenger && challenger.delta > -25 ? 'MEDIUM' : 'NONE'),
        threatScore: defending ? 0.88 : (challenger ? 0.45 : 0.0),
        attackerIntent: defending ? tactical.defenseMode : 'NONE',
        gapM: finite(challenger?.delta, 99),
        closingSpeedMps: finite(challenger?.relativeLongitudinalVelocity, 0),
        ttc: finite(challenger?.ttc, 99)
      },

      // Floating Thought Billboard & Diagnostics Summary
      thought: {
        requestedManeuver: defending ? tactical.defenseMode : (attacking ? tactical.attackMode : 'PACE'),
        deployedManeuver: defending ? tactical.defenseMode : (attacking ? tactical.attackMode : 'PACE'),
        deployedOffsetM: finite(targetOffset),
        targetId: targetId ?? 'CLEAR',
        abortReason: 'CLEAR',
        waitReason: 'CLEAR',
        committed: attacking,
        defending,
        defensivePhase: tactical.defenseMode,
        attackerIntent: defending ? 'CHALLENGER_PRESSURE' : 'NONE',
        straightSend: attacking && straight,
        divebombing: tactical.attackMode === 'DIVEBOMB',
        switchbacking: tactical.attackMode === 'SWITCHBACK',
        predictedTimeGainS: attacking ? 0.55 : 0.0,
        safetyThresholdM: 1.2,
        kerbAllowanceM: this._kerbUsage * 0.95,
        corridorCollisionFree: isSafe,
        corridorMinimumClearanceM: clearance,
        trajectoryCollisionFree: isSafe,
        trajectoryRoadLegal: Boolean(this.trajectoryPlan?.roadLegal ?? true),
        trajectorySelectedOffsetM: finite(this.trajectoryPlan?.selectedOffset, finite(targetOffset, 0)),
        trajectoryMinimumClearanceM: clearance
      },

      // Real-Time Control & Friction Circle Telemetry
      telemetry: {
        lateralAccelMps2: finite(latAccel),
        lateralUtilization: finite(pedals?.friction?.latUtilization ?? 0),
        maxG: finite(pedals?.friction?.peakG ?? 2.70, 2.70),
        trailBrakingActive: Boolean(pedals?.trailBraking || (finalControls.brake > 0.05 && Math.abs(finalControls.steer) > 0.1)),
        throttle: finite(finalControls.throttle),
        brake: finite(finalControls.brake),
        steer: finite(finalControls.steer),
        ersMode,
        ersSoc: finite(vehicle.ers?.soc, 0),
        yawRate: finite(vehicle.yawRate, 0),
        emergency: false,
        rubbingActive: Boolean(finalControls.rubbing),
        powerSlideActive: Boolean(finalControls.powerSlide),
        state: tacticalMode,
        action: attacking ? (tactical.attackMode === 'DIVEBOMB' ? 'FEARLESS DIVEBOMB' : 'SLINGSHOT ATTACK') : (defending ? `DEFEND (${tactical.defenseMode})` : 'OPTIMAL TIME PACE'),
        reason: tacticalReason,
        threatLevel: defending ? 'HIGH' : 'LOW',
        tactic: attacking ? 'Dynamic Nash / IBR Game' : (defending ? 'Stackelberg Apex Shield' : 'Coupled MPCC Friction Circle'),
        prediction: `Clearance: ${clearance < 90 ? clearance.toFixed(1) + 'm' : 'CLEAR'} // Safe: ${isSafe}`,
        decision: tacticalReason,
        latUtilization: finite(pedals?.friction?.latUtilization ?? 0),
        remainingLongBudget: finite(pedals?.friction?.remainingLongBudget ?? 1),
        liveLatG: finite(latAccel / 9.81),
        peakLatG: finite(pedals?.friction?.peakG ?? 2.70, 2.70)
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

  /**
   * Continuous Physics-Based Multi-Agent Collision & Track-Edge Supervisor.
   * Models relative closing velocities, stopping distance margins, and side-by-side rubbing.
   * Eliminates rear-end collisions while preserving authoritative door-to-door combat.
   * @private
   */
  _superviseTraffic(vehicle, vehicles, track, current, dt) {
    const vSpeed = finite(vehicle?.speed, 0);
    const forward = vehicle?.forward ?? { x: -Math.sin(vehicle?.yaw || 0), z: Math.cos(vehicle?.yaw || 0) };
    const right = vehicle?.right ?? { x: forward.z, z: -forward.x };
    const vClass = vehicle?.classKey || 'prototype';
    const brakeAcc = vClass === 'prototype' ? 15.0 : (vClass === 'gt' ? 11.5 : 9.8);
    const nominalHalfWidth = finite(track?.roadHalfWidth, 6.5);
    const edgeMargin = Math.max(2.4, nominalHalfWidth - 1.15);

    let maxSpeed = Infinity;
    let emergency = false;
    let reason = 'CLEAR';
    let nearestAheadDist = Infinity;
    let following = false;

    for (const other of vehicles || []) {
      if (!other || other === vehicle || other.finished || other.despawned || other.trafficGhost) continue;
      const dx = finite(other.position.x) - finite(vehicle.position.x);
      const dz = finite(other.position.z) - finite(vehicle.position.z);
      const ahead = dx * forward.x + dz * forward.z;
      const side = dx * right.x + dz * right.z;

      const otherVel = other.velocity ?? { x: 0, z: 0 };
      const otherForwardSpeed = finite(otherVel.x * forward.x + otherVel.z * forward.z, finite(other.speed, 0));
      const closing = Math.max(0, vSpeed - otherForwardSpeed);
      const sideVelocity = (otherVel.x - (vehicle?.velocity?.x || 0)) * right.x + (otherVel.z - (vehicle?.velocity?.z || 0)) * right.z;

      const halfLength = (vehicle?.length || 4.65) * 0.5 + (other.length || 4.65) * 0.5;
      const halfWidth = (vehicle?.width || 2.05) * 0.5 + (other.width || 2.05) * 0.5;

      const lateralClearance = Math.abs(side) - halfWidth;
      // Lateral convergence toward our vehicle centerline:
      // When side > 0, other is to our right; sideVelocity < 0 indicates movement toward us.
      const lateralConvergence = -Math.sign(side) * sideVelocity;

      // 1. Longitudinally overlapping (side-by-side or passing alongside)
      // When |ahead| < halfLength + 0.6, the vehicles are alongside each other.
      // An in-line rear-end collision is physically impossible. Never clamp longitudinal speed!
      const isAlongside = Math.abs(ahead) < halfLength + 0.6;
      if (isAlongside) {
        // If there is positive clearance or a light rub, allow uninhibited racing and passing
        if (lateralClearance > -0.22 && Math.abs(closing) < 6.5 && lateralConvergence < 1.8) {
          continue;
        }
        // Severe converging sideswipe while alongside: ease off gently to relieve the pinch
        if (lateralClearance < -0.15 && lateralConvergence > 1.0) {
          maxSpeed = Math.min(maxSpeed, Math.max(16.0, otherForwardSpeed - 2.0));
        }
        continue;
      }

      // 2. Obstacle ahead in our forward travel corridor
      // In-corridor requires being physically in our lane OR converging directly across our path
      const isInCorridor = (lateralClearance < 0.20)
        || (lateralConvergence > 0.40 && (Math.abs(side) - lateralConvergence * 0.45) < halfWidth + 0.15);

      if (ahead >= halfLength + 0.6 && ahead < 65.0 && isInCorridor) {
        nearestAheadDist = Math.min(nearestAheadDist, ahead);
        const clearance = ahead - halfLength; // Strictly positive bumper-to-bumper distance

        // Dynamic physics stopping distance: d = v_close * tau + v_close^2 / (2 * a_brake)
        const safeMargin = 1.2 + closing * 0.16 + (closing * closing) / (2 * Math.max(4.0, brakeAcc));

        // When obstacle ahead is slow/stopped (< 6.0 m/s), maintain rolling bypass speed floor
        // so the vehicle retains aerodynamic and steering authority to duck out and pass
        const isSlowObstacle = otherForwardSpeed < 6.0;
        const speedLimit = isSlowObstacle
          ? Math.max(clearance > 2.2 ? 11.0 : (clearance > 1.0 ? 5.0 : 0), otherForwardSpeed + (clearance - safeMargin) * 0.85)
          : Math.max(clearance > 2.0 ? 8.0 : 0, otherForwardSpeed + (clearance - safeMargin) * 0.85);

        if (speedLimit < maxSpeed) {
          maxSpeed = speedLimit;
          following = !isSlowObstacle && clearance < safeMargin + 3.0;
          reason = isSlowObstacle ? 'STATIC_OBSTACLE_PACING' : 'TRAFFIC_PACING';
        }

        // True imminent rear-end bumper touch
        if (clearance < Math.max(0.35, closing * 0.22)) {
          emergency = true;
          reason = 'IMMINENT_CONTACT';
        }
      }
    }

    // Outward drift / track limit safeguard: gently ease off if drifting beyond track boundary
    const physicalLimit = nominalHalfWidth - 0.70;
    const trackPoint = track?.atDistance ? track.atDistance(vehicle.distance) : null;
    if (trackPoint?.normal && Math.abs(current?.lateral || 0) > physicalLimit) {
      const outward = ((vehicle?.velocity?.x || 0) * trackPoint.normal.x + (vehicle?.velocity?.z || 0) * trackPoint.normal.z) * Math.sign(current?.lateral || 0);
      if (outward > 0.15) {
        const excess = Math.abs(current?.lateral || 0) - physicalLimit;
        maxSpeed = Math.min(maxSpeed, Math.max(12.0, vSpeed - excess * 5.0 - outward * 2.0));
        if (!emergency) reason = 'TRACK_EDGE_DECEL';
      }
    }

    return { maxSpeed, emergency, reason, nearestAheadDist, following };
  }
}
