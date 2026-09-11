/**
 * NextGenAIController.js (V2 Main Controller Orchestrator)
 * Unites the 4-Layer Hybrid AI Architecture:
 * - Layer 1: Multi-Scale Curvature Profile & Analytical Velocity Solver (GlobalTimeOptimalEngine)
 * - Layer 2: Predictive Adversarial Racecraft Engine (GameTheoreticCombatEngine)
 * - Layer 3: Coupled Physics-Informed Saturated Feedback Controller (CoupledMPCC + PaceOptimizer + Lattice)
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

    this.options = options;
    const defaultClass = options.defaultClass ?? 'prototype';
    const customSpecs = options.customSpecs ?? (options.spec ? { [options.classKey || defaultClass]: options.spec } : null);

    // Perception & Hybrid Engine Layers
    this.awareness = new TrafficAwareness();
    this.optimalEngine = options.track ? new GlobalTimeOptimalEngine({ track: options.track, defaultClass, customSpecs }) : null;
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
    this.previousActiveTrajectory = null;
    this._lastVehicle = null;
    this._lastVehicles = [];
    this._lastTrack = null;
    this._lastTactical = null;
    this._lastTacticalMode = 'PACE';
    this.lapLineBias = 0;

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

  /**
   * Export comprehensive 3D visual debug data for Benchmark and Gauntlet HUD.
   * Completely read-only and non-intrusive.
   * @param {Object} [trackOverride]
   * @returns {Object|null} Canonical visual debug bundle
   */
  getDebugVisuals(trackOverride = null) {
    const track = trackOverride || this._lastTrack || this.optimalEngine?.track;
    const vehicle = this._lastVehicle;
    if (!track || !vehicle || !this.trajectoryPlan) return null;

    const tactical = this._lastTactical || {};
    const tacticalMode = this._lastTacticalMode || this.debugState?.mode || 'PACE';
    const racecraftPhase = this.debugState?.racecraftPhase || 'OPTIMAL_LINE';
    const defending = tactical.role === 'DEFEND' || tactical.role === 'DUAL_COMBAT' || tactical.defenseMode !== 'PACE';
    const attacking = tactical.role === 'ATTACK' || tactical.role === 'DUAL_COMBAT' || tactical.attackMode !== 'NONE';

    // 1. Color mapping based on tactical mode
    let modeColor = '#00ff88'; // PACE (emerald green)
    if (defending) {
      modeColor = '#00d2ff'; // DEFEND (cyan / electric blue)
    } else if (attacking) {
      if (tactical.attackMode === 'DIVEBOMB') modeColor = '#ff2a2a'; // DIVEBOMB (intense red)
      else if (tactical.attackMode === 'SWITCHBACK') modeColor = '#d020d0'; // SWITCHBACK (magenta)
      else modeColor = '#ff8800'; // SLINGSHOT (amber orange)
    } else if (this.debugState?.recovering) {
      modeColor = '#ffbb00';
    }

    // 2. Selected Trajectory Ribbon (0.65m wide, elevated +0.06m)
    const selectedTrajectory = {
      points: this.trajectoryPlan.points || [],
      color: modeColor,
      mode: tacticalMode,
      phase: racecraftPhase,
      score: this.trajectoryPlan.score || 0,
      selectedOffset: this.trajectoryPlan.selectedOffset ?? 0,
      transitionTimeS: this.trajectoryPlan.transitionTimeS ?? 0,
      ribbonWidth: 0.65,
      lift: 0.06
    };

    // 3. Previous Active Trajectory Ghost (Fading out over ~0.8s)
    let previousTrajectory = null;
    if (this.previousActiveTrajectory && Array.isArray(this.previousActiveTrajectory.points)) {
      const ageS = Math.max(0, (this.totalTime || 0) - (this.previousActiveTrajectory.switchedAt || this.previousActiveTrajectory.generatedAt || 0));
      if (ageS < 0.85) {
        const opacity = Math.max(0, 1.0 - (ageS / 0.85));
        previousTrajectory = {
          points: this.previousActiveTrajectory.points,
          selectedOffset: this.previousActiveTrajectory.selectedOffset,
          divergencePoint: this.previousActiveTrajectory.divergencePoint,
          ageS,
          opacity,
          color: '#d8b4fe' // ghostly lilac
        };
      }
    }

    // 4. Alternate Trajectory Candidates (Bounded 24-32 paths)
    const rawCandidates = this.trajectoryPlan.candidates || this.trajectoryPlanner.lastCandidates || [];
    const candidates = rawCandidates.map((cand) => {
      let color = '#a0a0a0'; // HIGHER_COST (gray)
      const reason = cand.rejectionReason || 'HIGHER_COST';
      if (cand.selected || reason === 'SELECTED') {
        color = '#00ff88'; // green
      } else if (reason === 'VIABLE_ALTERNATIVE') {
        color = '#00d2ff'; // cyan
      } else if (reason === 'COLLISION') {
        color = '#ff2222'; // red
      } else if (reason === 'ROAD_LIMIT') {
        color = '#ff8800'; // orange
      } else if (reason === 'DYNAMIC_LIMIT') {
        color = '#ffdd00'; // yellow
      } else if (reason === 'SWITCH_MARGIN' || reason === 'CONTINUITY_COST') {
        color = '#c033ff'; // purple
      }

      return {
        id: cand.id,
        points: cand.points || [],
        terminalLateral: cand.terminalLateral,
        transitionTime: cand.transitionTime,
        score: cand.score,
        selected: Boolean(cand.selected),
        rejectionReason: reason,
        collisionFree: Boolean(cand.collisionFree),
        roadLegal: Boolean(cand.roadLegal),
        conflictPoint: cand.conflictPoint || null,
        conflictStation: cand.conflictStation ?? null,
        conflictTime: cand.conflictTime ?? null,
        color
      };
    });

    // 5. Tactical Corridor [dMin, dMax]
    const dMin = finite(tactical.dMin, -finite(track.roadHalfWidth, 8.2));
    const dMax = finite(tactical.dMax, finite(track.roadHalfWidth, 8.2));
    const corridorLength = 48.0;
    const corridorSteps = 16;
    const corridorLeft = [];
    const corridorRight = [];
    const vDist = finite(vehicle.distance, 0);

    for (let i = 0; i <= corridorSteps; i++) {
      const s = vDist + (corridorLength * i) / corridorSteps;
      const ref = track.atDistance ? track.atDistance(s) : { s, x: 0, y: 0, z: 0 };
      const ptLeft = track.lateralPoint ? track.lateralPoint(ref, dMax, 0.04) : { x: ref.x, y: (ref.y || 0) + 0.04, z: ref.z };
      const ptRight = track.lateralPoint ? track.lateralPoint(ref, dMin, 0.04) : { x: ref.x, y: (ref.y || 0) + 0.04, z: ref.z };
      corridorLeft.push(ptLeft);
      corridorRight.push(ptRight);
    }

    const tacticalCorridor = {
      dMin,
      dMax,
      mode: tacticalMode,
      color: modeColor,
      leftBoundary: corridorLeft,
      rightBoundary: corridorRight
    };

    // 6. Target Vehicle Highlight
    let targetVehicle = null;
    const targetId = this.debugState?.targetId;
    if (targetId != null && Array.isArray(this._lastVehicles)) {
      const targetObj = this._lastVehicles.find((v) => v?.id === targetId || v?.name === targetId);
      if (targetObj && targetObj.position) {
        targetVehicle = {
          id: targetId,
          role: defending ? 'DEFEND_FROM' : 'ATTACK_TARGET',
          position: { x: targetObj.position.x, y: targetObj.position.y, z: targetObj.position.z },
          egoFront: {
            x: vehicle.position.x + Math.sin(vehicle.yaw || 0) * 2.3,
            y: vehicle.position.y + 0.35,
            z: vehicle.position.z + Math.cos(vehicle.yaw || 0) * 2.3
          },
          targetRear: {
            x: targetObj.position.x - Math.sin(targetObj.yaw || 0) * 2.3,
            y: targetObj.position.y + 0.35,
            z: targetObj.position.z - Math.cos(targetObj.yaw || 0) * 2.3
          }
        };
      }
    }

    // 7. Opponent Future Predictions (0.5s, 1.0s, 2.0s, 3.0s)
    const opponentPredictions = [];
    const predictionTimes = [0.5, 1.0, 2.0, 3.0];
    for (const other of (this._lastVehicles || [])) {
      if (!other || other === vehicle || other.finished || other.despawned || other.trafficGhost) continue;
      const oSpeed = Math.max(0, finite(other.speed, 0));
      const oDist = finite(other.distance, 0);
      const oLat = finite(other.surface?.lateral, 0);
      const oYaw = finite(other.yaw, 0);

      const dx = other.position.x - vehicle.position.x;
      const dz = other.position.z - vehicle.position.z;
      if (Math.hypot(dx, dz) > 95) continue;

      const trail = [];
      const boxes = [];

      for (let step = 0; step <= 30; step++) {
        const t = (step / 30) * 3.0;
        const sPred = oDist + oSpeed * t;
        const refPred = track.atDistance ? track.atDistance(sPred) : { s: sPred, x: other.position.x, y: other.position.y, z: other.position.z };
        const pt = track.lateralPoint ? track.lateralPoint(refPred, oLat, 0.1) : { x: refPred.x, y: (refPred.y || 0) + 0.1, z: refPred.z };
        trail.push({ x: pt.x, y: pt.y, z: pt.z, time: t });
      }

      for (const t of predictionTimes) {
        const sPred = oDist + oSpeed * t;
        const refPred = track.atDistance ? track.atDistance(sPred) : { s: sPred, x: other.position.x, y: other.position.y, z: other.position.z };
        const pt = track.lateralPoint ? track.lateralPoint(refPred, oLat, 0.1) : { x: refPred.x, y: (refPred.y || 0) + 0.1, z: refPred.z };

        let boxYaw = oYaw;
        if (track.tangentAtDistance) {
          const tan = track.tangentAtDistance(sPred);
          if (tan) boxYaw = Math.atan2(tan.x, tan.z);
        }

        boxes.push({
          time: t,
          x: pt.x,
          y: pt.y + 0.55,
          z: pt.z,
          yaw: boxYaw,
          length: 4.65,
          width: 2.05,
          height: 1.15
        });
      }

      opponentPredictions.push({
        id: other.id,
        name: other.name,
        current: { x: other.position.x, y: other.position.y, z: other.position.z, yaw: oYaw, speed: oSpeed },
        trail,
        boxes
      });
    }

    // 8. Tracking Point & Lookahead
    const trackingPoint = this.trajectoryPlan.trackingPoint ?? this.trajectoryPlan.points?.[this.trajectoryPlan.trackingIndex ?? 0] ?? this.trajectoryPlan.points?.[0];
    const frontAxle = {
      x: vehicle.position.x + Math.sin(vehicle.yaw || 0) * 1.4,
      y: vehicle.position.y + 0.35,
      z: vehicle.position.z + Math.cos(vehicle.yaw || 0) * 1.4
    };

    // 9. Markers (Braking & Commitment)
    const isBraking = Boolean(vehicle.controls?.brake > 0.05);
    let commitmentType = 'NONE';
    if (tactical.attackMode === 'DIVEBOMB') commitmentType = 'DIVE';
    else if (tactical.attackMode === 'SWITCHBACK') commitmentType = 'SWITCHBACK';
    else if (tactical.attackMode === 'ABORT_HOLD' || tactical.attackMode === 'ABORT_BLEND') commitmentType = 'ABORT';

    const markers = {
      braking: {
        active: isBraking,
        x: vehicle.position.x + Math.sin(vehicle.yaw || 0) * 8.0,
        y: vehicle.position.y + 0.05,
        z: vehicle.position.z + Math.cos(vehicle.yaw || 0) * 8.0,
        yaw: vehicle.yaw || 0,
        width: 4.0
      },
      commitment: {
        type: commitmentType,
        x: vehicle.position.x + Math.sin(vehicle.yaw || 0) * 3.5,
        y: vehicle.position.y + 0.15,
        z: vehicle.position.z + Math.cos(vehicle.yaw || 0) * 3.5,
        yaw: vehicle.yaw || 0
      }
    };

    // 10. Road and Planner Limits ahead of car
    const roadLimits = {
      asphaltLeft: [],
      asphaltRight: [],
      plannerLeft: [],
      plannerRight: []
    };
    const roadSteps = 20;
    const roadDistAhead = 60.0;
    const roadHalfWidth = finite(track.roadHalfWidth, 8.2);
    const curbWidth = finite(track.curbWidth, 1.25);
    for (let i = 0; i <= roadSteps; i++) {
      const s = vDist + (roadDistAhead * i) / roadSteps;
      const ref = track.atDistance ? track.atDistance(s) : { s, x: 0, y: 0, z: 0 };
      const asphaltL = track.lateralPoint ? track.lateralPoint(ref, roadHalfWidth, 0.04) : { x: ref.x, y: 0.04, z: ref.z };
      const asphaltR = track.lateralPoint ? track.lateralPoint(ref, -roadHalfWidth, 0.04) : { x: ref.x, y: 0.04, z: ref.z };
      const plannerL = track.lateralPoint ? track.lateralPoint(ref, roadHalfWidth + curbWidth, 0.04) : { x: ref.x, y: 0.04, z: ref.z };
      const plannerR = track.lateralPoint ? track.lateralPoint(ref, -(roadHalfWidth + curbWidth), 0.04) : { x: ref.x, y: 0.04, z: ref.z };
      roadLimits.asphaltLeft.push(asphaltL);
      roadLimits.asphaltRight.push(asphaltR);
      roadLimits.plannerLeft.push(plannerL);
      roadLimits.plannerRight.push(plannerR);
    }

    return {
      selectedTrajectory,
      previousTrajectory,
      candidates,
      tacticalCorridor,
      targetVehicle,
      opponentPredictions,
      trackingPoint: trackingPoint ? {
        x: trackingPoint.x,
        y: trackingPoint.y,
        z: trackingPoint.z,
        lateral: trackingPoint.lateral,
        forwardDistance: trackingPoint.forwardDistance
      } : null,
      frontAxle,
      markers,
      roadLimits
    };
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
    this.previousActiveTrajectory = null;
    this._lastVehicle = null;
    this._lastVehicles = [];
    this._lastTrack = null;
    this._lastTactical = null;
    this._lastTacticalMode = 'PACE';
    this.steerCommand = 0;
    this.lastDistance = null;
    this.stallTime = 0;
    this.recoveryTimer = 0;
    this.draftTargetId = null;
    this.passTargetId = null;
    this.defenseTargetId = null;
    this.passPhase = 'NONE';
    this.tacticalTimer = 0;
    this.tacticalState = null;
    this.totalTime = 0;
    this.steeringHistory = [];
    this.steeringReversalsLastSecond = 0;
    this.lateralLoadTransferRate = 0;
    this.lastLocalAccelX = 0;
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

    this._lastVehicle = vehicle;
    this._lastVehicles = vehicles;
    this._lastTrack = track;

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
      const defaultClass = this.options?.defaultClass ?? vehicle.classKey ?? 'prototype';
      const customSpecs = this.options?.customSpecs ?? (vehicle.spec ? { [vehicle.classKey || 'gt']: vehicle.spec } : null);
      this.optimalEngine = new GlobalTimeOptimalEngine({ track, defaultClass, customSpecs });
    }

    // 2. Scan multi-agent traffic awareness & surface state
    const traffic = this.awareness.scan(vehicle, vehicles, track);
    const current = traffic.current;
    const isOffTrack = offRoad(current) || offRoad(vehicle.surface);

    const nominalHalfWidth = finite(track.roadHalfWidth, 6.5);
    const kerbAllowance = Math.min(0.8, finite(track.curbWidth, 1.05) * 0.65) * this._kerbUsage;
    const baseRoadMargin = Math.max(2.1, nominalHalfWidth - 0.85 + kerbAllowance);
    const currentSurfaceMargin = finite(
      track.planningLateralLimit?.(vehicle.distance, current?.lateral),
      nominalHalfWidth - 0.85
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

    this.totalTime = (this.totalTime || 0) + dt;
    if (this.nextTacticalTime == null) {
      this.nextTacticalTime = (this.index % 10) * (0.10 / 10);
    }
    if (this.nextPlanTime == null) {
      this.nextPlanTime = (this.index % 4) * (0.04 / 4);
    }

    // 3. Layer 2: Multi-Rate Decoupled Game-Theoretic Corridor Planning (10 Hz)
    const hasCloseTarget = (traffic.challenger && traffic.challenger.delta > -15.0)
      || (traffic.targetAhead && traffic.targetAhead.delta < 20.0 && traffic.targetAhead.delta > 0);
    const targetEnteredCloseRange = hasCloseTarget && !this.wasInProximity;
    this.wasInProximity = hasCloseTarget;

    const shouldEvaluateTactics = !this.tacticalState
      || targetEnteredCloseRange
      || this.totalTime >= this.nextTacticalTime;

    if (shouldEvaluateTactics) {
      const tacticalDt = Math.max(dt, this.totalTime - (this.lastTacticalEvalTime || 0));
      this.lastTacticalEvalTime = this.totalTime;
      this.nextTacticalTime = this.totalTime + 0.10;
      this.tacticalEvalCount = (this.tacticalEvalCount || 0) + 1;
      this.tacticalState = this.combatEngine.evaluate({
        vehicle,
        track,
        traffic,
        optimalProfile: this.optimalEngine,
        aggression: this._aggression,
        dt: tacticalDt
      });
    }
    const tactical = this.tacticalState;

    const currentPoint = track?.atDistance ? track.atDistance(vehicle.distance) : { curvature: 0, turnSign: 0 };
    const rawCurv = Math.abs(finite(currentPoint?.curvature, 0));
    const currentTurnSign = finite(currentPoint?.turnSign, 0);
    // Steering-signed curvature: positive steers right, negative steers left.
    // In Track.js and shadow.js, turnSign is -1 for right turn, +1 for left turn.
    const signedCurv = -currentTurnSign * rawCurv;
    const currentCurv = rawCurv;

    const optCurrent = this.optimalEngine?.sampleAtDistance?.(vehicle.distance, vehicle.classKey);
    const defending = tactical.role === 'DEFEND' || tactical.role === 'DUAL_COMBAT' || tactical.defenseMode !== 'PACE';
    const attacking = tactical.role === 'ATTACK' || tactical.role === 'DUAL_COMBAT' || tactical.attackMode !== 'NONE';
    let tacticalMode = recovering ? 'RECOVER' : (tactical.role === 'DUAL_COMBAT' ? 'DUAL_COMBAT' : (defending ? 'DEFEND' : (attacking ? 'ATTACK' : 'PACE')));

    const basePaceLateral = optCurrent?.lateral ?? 0;
    const adaptedPaceLateral = this._adaptLapLine(vehicle, basePaceLateral, track, signedCurv, dt);
    let targetOffset = recovering ? 0 : (tactical.role === 'PACE' ? clamp(adaptedPaceLateral, -baseRoadMargin, baseRoadMargin) : clamp(tactical.targetLateral, -baseRoadMargin, baseRoadMargin));
    let targetId = attacking ? this.combatEngine.attackTargetId : (defending ? this.combatEngine.defenseTargetId : null);
    let tacticalReason = recovering ? (isOffTrack ? 'OFF_TRACK_RECOVERY' : 'STALL_RECOVERY') : tactical.notes;

    if (vehicle.pitIntent?.active) {
      tacticalMode = 'PIT';
      targetOffset = finite(vehicle.pitIntent.targetLateralM, targetOffset);
      tacticalReason = 'PIT_LANE_ENTRY';
    }

    this._lastTactical = tactical;
    this._lastTacticalMode = tacticalMode;

    // 4. Multi-Candidate Frenet Lattice Trajectory Planning
    const dynamicLookahead = this.computeLookahead(vehicle.speed, currentCurv);
    const lookAheadDist = recovering
      ? clamp(10.0 + vehicle.speed * 0.42, 10.0, 20.0)
      : clamp(Math.max(dynamicLookahead, 11.0 + vehicle.speed * 0.58), 12.0, 30.0);

    const trackingDistance = clamp(lookAheadDist * 0.72 / (1.0 + currentCurv * 12.0), 8.5, 24.0);

    const maxTireWear = Math.max(0, ...(vehicle.wheels ?? []).map((w) => finite(w.wear, 0)));
    const tireGripFactor = clamp(1.0 - maxTireWear * 0.45, 0.80, 1.0);

    // Layer 1 Free-Boundary Speed Reachability Envelope
    const physicalTargetSpeed = this.paceOptimizer.computeSpeedEnvelope({
      vehicle,
      track,
      optimalEngine: this.optimalEngine,
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
    const racecraftPhase = defending ? tactical.defenseMode : (attacking ? tactical.attackMode : 'NONE');
    const phaseChanged = (this.lastRacecraftPhase !== racecraftPhase);
    this.lastRacecraftPhase = racecraftPhase;

    const shouldReplan = !this.trajectoryPlan
      || phaseChanged
      || isOffTrack
      || this.totalTime >= this.nextPlanTime;

    if (shouldReplan) {
      const dtSinceLastPlan = Math.max(dt, this.totalTime - (this.lastPlanTime || 0));
      this.lastPlanTime = this.totalTime;
      this.nextPlanTime = this.totalTime + 0.04;
      const previousPlanSnapshot = this.trajectoryPlan ? {
        points: this.trajectoryPlan.points,
        selectedOffset: this.trajectoryPlan.selectedOffset,
        generatedAt: this.trajectoryPlan.generatedAt ?? this.totalTime,
        replanIndex: this.trajectoryPlan.replanIndex ?? 0
      } : null;

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
        dMin: tactical.dMin ?? -baseRoadMargin,
        dMax: tactical.dMax ?? baseRoadMargin,
        trackingDistance,
        referenceLineAtDistance: (s) => {
          if (isMatchingTrack && typeof this.referenceProfile?.paceAtDistance === 'function') {
            return this.referenceProfile.paceAtDistance(s, vehicle.classKey)?.lineLateral ?? 0;
          }
          const optLat = this.optimalEngine?.sampleAtDistance?.(s, vehicle.classKey)?.lateral;
          return Number.isFinite(optLat) ? clamp(optLat, -baseRoadMargin, baseRoadMargin) : 0;
        },
        previousPlan: this.trajectoryPlan,
        dtSinceLastPlan
      });
      this.replanCount = (this.replanCount || 0) + 1;
      if (this.trajectoryPlan) {
        this.trajectoryPlan.generatedAt = this.totalTime || 0;
        this.trajectoryPlan.replanIndex = this.replanCount;

        if (previousPlanSnapshot && previousPlanSnapshot.points?.length > 0) {
          const offsetDiff = Math.abs((this.trajectoryPlan.selectedOffset ?? 0) - (previousPlanSnapshot.selectedOffset ?? 0));
          if (offsetDiff > 0.15 || !this.previousActiveTrajectory) {
            let divergencePoint = null;
            const currPts = this.trajectoryPlan.points || [];
            const prevPts = previousPlanSnapshot.points || [];
            for (let i = 0; i < Math.min(currPts.length, prevPts.length); i++) {
              if (Math.abs(currPts[i].lateral - prevPts[i].lateral) > 0.15) {
                divergencePoint = { x: currPts[i].x, y: currPts[i].y, z: currPts[i].z };
                break;
              }
            }
            this.previousActiveTrajectory = {
              ...previousPlanSnapshot,
              divergencePoint: divergencePoint ?? (currPts[0] ? { x: currPts[0].x, y: currPts[0].y, z: currPts[0].z } : null),
              switchedAt: this.totalTime || 0
            };
          }
        }
      }
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
    } else {
      headingError = clamp(headingError, -0.75, 0.75);
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
    if (optCurrent?.targetSpeed && Number.isFinite(optCurrent.targetSpeed)) {
      desiredSpeed = Math.min(desiredSpeed, optCurrent.targetSpeed);
    }
    if (Number.isFinite(tactical.desiredSpeed) && tactical.desiredSpeed > 0) {
      // Context-sensitive tactical overspeed schedule:
      // Straights (|curv| < 0.003): up to +12-15% for draft/slingshot
      // Mild bends (0.003 <= |curv| < 0.008): +4-7%
      // Real corners (|curv| >= 0.008): +0-2% (physical target acts as ceiling)
      // Elevated stability risk: 0% or slightly below baseline to prevent overdriving
      const curvAbs = Math.abs(signedCurv);
      const stabilityRisk = this.coupledMPCC?.telemetry?.stabilityRisk ?? 0;
      let overspeedCap = 1.0;
      if (stabilityRisk > 0.35) {
        overspeedCap = 0.96;
      } else if (stabilityRisk > 0.15) {
        overspeedCap = 1.00;
      } else if (curvAbs < 0.0030) {
        overspeedCap = 1.15;
      } else if (curvAbs < 0.0080) {
        overspeedCap = 1.06;
      } else {
        overspeedCap = 1.02;
      }
      desiredSpeed = clamp(tactical.desiredSpeed, physicalTargetSpeed * 0.70, physicalTargetSpeed * overspeedCap);
    }
    desiredSpeed = Math.min(desiredSpeed, supervisor.maxSpeed);

    if (recovering) desiredSpeed = isOffTrack ? (isFacingBackwards ? 5.0 : 8.5) : Math.max(16.0, physicalTargetSpeed * 0.75);

    // 7. Coupled Model Predictive Contouring (MPCC) State Step
    const mpccOut = this.coupledMPCC.step({
      vehicle,
      track,
      tacticalTarget: {
        targetLateral: plannedTargetOffset,
        desiredSpeed,
        headingError,
        targetPos,
        dMin: tactical.dMin ?? -baseRoadMargin,
        dMax: tactical.dMax ?? baseRoadMargin
      },
      dt,
      tireGripFactor,
      aggression: this._aggression,
      recovering,
      defending,
      committed: defending || attacking
    });

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
    // Actuator Authority: In normal/tactical mode, Coupled MPCC provides closed-loop
    // contouring steering, friction-budgeted braking, and launch power.
    // Under recovery or emergency intervention, supervisor/recovery overrides prevail.
    const activeSteer = recovering ? this.steerCommand : (mpccOut?.steer ?? this.steerCommand);
    const activeThrottle = supervisor.emergency
      ? 0
      : (recovering ? rawPedals.throttle : (mpccOut?.throttle ?? rawPedals.throttle));
    const activeBrake = supervisor.emergency
      ? 1.0
      : (recovering ? rawPedals.brake : (mpccOut?.brake ?? rawPedals.brake));

    const finalControls = this.dynamicsEngine.process({
      vehicle,
      traffic,
      controls: {
        steer: clamp(activeSteer, -1, 1),
        throttle: activeThrottle,
        brake: activeBrake
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

    // Track steering reversals & lateral load transfer rate
    this.steeringHistory = this.steeringHistory || [];
    this.steeringHistory.push({ time: this.totalTime, steer: finalControls.steer });
    while (this.steeringHistory.length > 0 && this.totalTime - this.steeringHistory[0].time > 1.0) {
      this.steeringHistory.shift();
    }
    let reversals = 0;
    for (let i = 1; i < this.steeringHistory.length; i++) {
      const s0 = this.steeringHistory[i - 1].steer;
      const s1 = this.steeringHistory[i].steer;
      if (Math.sign(s0) !== 0 && Math.sign(s1) !== 0 && Math.sign(s0) !== Math.sign(s1) && Math.abs(s1 - s0) > 0.08) {
        reversals++;
      }
    }
    this.steeringReversalsLastSecond = reversals;

    const currentAccelX = finite(vehicle.localAcceleration?.x, 0);
    this.lateralLoadTransferRate = Math.abs(currentAccelX - (this.lastLocalAccelX || 0)) / Math.max(1e-4, dt);
    this.lastLocalAccelX = currentAccelX;

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
      
      // Candidate Trajectory Lattice & NMPCC Predictive Horizon
      coupledMPCC: this.coupledMPCC?.telemetry,
      mpccHorizon: this.coupledMPCC?.predPoints,
      candidates: this.trajectoryPlan?.candidates ?? [],
      bestCandidate: this.trajectoryPlan,
      trajectory: this.trajectoryPlan,
      trajectoryMinimumClearanceM: clearance,
      trajectoryCollisionFree: isSafe,
      trajectoryRoadLegal: Boolean(this.trajectoryPlan?.roadLegal ?? true),
      trajectorySelectedOffsetM: finite(this.trajectoryPlan?.selectedOffset, finite(targetOffset, 0)),
      trajectoryScore: finite(this.trajectoryPlan?.score, 0),
      tacticalPhase: tactical.tacticalPhase || (defending ? tactical.defenseMode : (attacking ? tactical.attackMode : 'PACE')),
      commitDwellRemaining: finite(tactical.commitDwellRemaining, 0),
      abortDwellRemaining: finite(tactical.abortDwellRemaining, 0),
      evaluatedTrajectories: finite(this.trajectoryPlan?.candidateCount ?? this.trajectoryPlan?.candidates?.length, 0),
      safeTrajectories: finite(this.trajectoryPlan?.safeTrajectoryCount ?? this.trajectoryPlan?.candidateCount, 0),
      steeringReversalsLastSecond: finite(this.steeringReversalsLastSecond, 0),
      trajectorySwitchBonus: finite(this.trajectoryPlan?.costBreakdown?.hysteresisBonus, 0),
      lateralLoadTransferRate: finite(this.lateralLoadTransferRate, 0),

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
        peakLatG: finite(pedals?.friction?.peakG ?? 2.70, 2.70),
        tacticalPhase: tactical.tacticalPhase || (defending ? tactical.defenseMode : (attacking ? tactical.attackMode : 'PACE')),
        commitDwellRemaining: finite(tactical.commitDwellRemaining, 0),
        abortDwellRemaining: finite(tactical.abortDwellRemaining, 0),
        evaluatedTrajectories: finite(this.trajectoryPlan?.candidateCount ?? this.trajectoryPlan?.candidates?.length, 0),
        safeTrajectories: finite(this.trajectoryPlan?.safeTrajectoryCount ?? this.trajectoryPlan?.candidateCount, 0),
        steeringReversalsLastSecond: finite(this.steeringReversalsLastSecond, 0),
        trajectorySwitchBonus: finite(this.trajectoryPlan?.costBreakdown?.hysteresisBonus, 0),
        lateralLoadTransferRate: finite(this.lateralLoadTransferRate, 0)
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

  /**
   * Human-like lap-by-lap line adaptation & organic dynamic exploration.
   * Dynamically modulates turn-in, apex, and exit lines per lap based on:
   * 1. Lap progression: multi-harmonic organic variation across consecutive laps
   * 2. Tire wear & thermal degradation: widens corner entry and squares off corners to preserve front grip
   * 3. Understeer gradient: adapts line width when front axle scrub is detected
   * @private
   */
  _adaptLapLine(vehicle, baseLateral, track, curvature, dt = 0.016) {
    const trackLength = Math.max(500, finite(track?.totalLength ?? track?.length, 2704.6));
    const dist = finite(vehicle?.distance, 0);
    const lap = Math.floor(dist / trackLength);
    const s = wrap(dist, trackLength);
    const curvMag = Math.abs(finite(curvature, 0));

    // 1. Multi-harmonic organic variation across laps (seeded uniquely per car and lap)
    // Simulates human line exploration (+/- 0.22m) without driving off-track
    const carOffset = (this.index * 137.5) % 1000;
    const lapAngle = (lap * 1.618 + carOffset * 0.01) % (Math.PI * 2);
    const sNorm = s / trackLength;

    const harm1 = Math.sin(sNorm * 6.0 * Math.PI + lapAngle) * 0.18;
    const harm2 = Math.cos(sNorm * 14.0 * Math.PI + lapAngle * 1.414) * 0.10;
    // Taper organic variation in high-curvature apexes where precise line adherence is vital
    const organicDelta = (harm1 + harm2) * (1.0 - clamp(curvMag * 35.0, 0, 0.70));

    // 2. Tire wear & understeer adaptation
    // When tires degrade, human drivers widen corner entries and take a later geometric apex to reduce scrub
    const maxTireWear = Math.max(0, ...(vehicle?.wheels ?? []).map((w) => finite(w.wear, 0)));
    const wearFactor = clamp(maxTireWear, 0, 1.0);
    const understeerFactor = clamp((this.coupledMPCC?.satAvg ?? this.paceOptimizer?.satAvg ?? 0), 0, 1.5);

    let wearDelta = 0;
    if (curvMag > 0.002) {
      // For left turns (curv > 0), apex is to the left (-lat), so entry is to the right (+lat).
      // Widening entry means moving away from the apex direction.
      const turnSign = Math.sign(curvature);
      wearDelta = -turnSign * (wearFactor * 0.24 + understeerFactor * 0.12);
    }

    // 3. Smooth temporal blending to prevent steering jerk
    const targetAdaptation = organicDelta + wearDelta;
    this.lapLineBias = damp(this.lapLineBias || 0, targetAdaptation, 4.0, dt);

    return baseLateral + this.lapLineBias;
  }
}
