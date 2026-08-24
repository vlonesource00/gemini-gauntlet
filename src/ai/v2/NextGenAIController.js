/**
 * NextGenAIController.js (V2 Main Orchestrator)
 * Unites the 3-Layer Hybrid Architecture:
 * - Layer 1: 2D Free-Boundary Optimal Profile Solver (GlobalTimeOptimalEngine)
 * - Layer 2: Stackelberg Leader Defense & IBR Divebomb Attack (GameTheoreticCombatEngine)
 * - Layer 3: Coupled Friction-Circle Trail-Braking & Apex Exit Power (PaceOptimizer + Lattice)
 * - Combat Dynamics: Elastic Contact Rubbing Equilibrium & Slip-Slope Power Sliding (CombatDynamicsEngine)
 */

import { TrafficAwareness } from '../TrafficAwareness.js';
import { GlobalTimeOptimalEngine } from './GlobalTimeOptimalEngine.js';
import { GameTheoreticCombatEngine } from './GameTheoreticCombatEngine.js';
import { CombatDynamicsEngine } from './CombatDynamicsEngine.js';
import { FrenetLatticePlanner } from '../FrenetLatticePlanner.js';
import { PaceOptimizer } from '../PaceOptimizer.js';
import { clamp, wrap, wrapAngle } from '../../core/math.js';

const finite = (val, fallback = 0) => (Number.isFinite(val) ? val : fallback);
const offRoad = (c) => c?.zone === 'grass' || c?.zone === 'runoff';

export class NextGenAIController {
  /**
   * @param {number|string} id - Controller / Car identifier
   * @param {Object} [options]
   */
  constructor(id = 1, options = {}) {
    this.id = id;
    this.skill = clamp(finite(options.skill, 0.95), 0.5, 1.0);
    this._aggression = clamp(finite(options.aggression, 0.90), 0.5, 1.0);
    this._diveMargin = clamp(finite(options.diveMargin, 0.85), 0.4, 1.0);
    this._defenseReactivity = clamp(finite(options.defenseReactivity, 0.92), 0.5, 1.0);
    this._kerbUsage = clamp(finite(options.kerbUsage, 0.90), 0.0, 1.0);

    // Perception & Hybrid Engine Layers
    this.awareness = new TrafficAwareness();
    this.optimalEngine = null;
    this.combatEngine = new GameTheoreticCombatEngine();
    this.trajectoryPlanner = new FrenetLatticePlanner({ pointCount: 24, horizonS: 3.2 });
    this.paceOptimizer = new PaceOptimizer();
    this.dynamicsEngine = new CombatDynamicsEngine();

    // Recovery timers and state tracking
    this.recoveryTimer = 0;
    this.stallTime = 0;
    this.lastDistance = null;
    this.steerCommand = 0;

    // Live debug and telemetry states
    this.debugState = null;
    this.trajectoryPlan = null;
  }

  get aggression() { return this._aggression; }
  set aggression(val) { this._aggression = clamp(finite(val, 0.90), 0.5, 1.0); }
  get diveMargin() { return this._diveMargin; }
  set diveMargin(val) { this._diveMargin = clamp(finite(val, 0.85), 0.4, 1.0); }
  get defenseReactivity() { return this._defenseReactivity; }
  set defenseReactivity(val) { this._defenseReactivity = clamp(finite(val, 0.92), 0.5, 1.0); }
  get kerbUsage() { return this._kerbUsage; }
  set kerbUsage(val) { this._kerbUsage = clamp(finite(val, 0.90), 0.0, 1.0); }

  /**
   * Main 120Hz update cycle.
   */
  update(vehicle, vehicles, track, race, dt = 0.016) {
    if (!vehicle || !track) return;

    if (race && race.phase !== 'racing') {
      vehicle.controls = { throttle: 0, brake: 1, steer: 0, handbrake: 0 };
      return;
    }

    // 1. Initialize Layer 1 Optimal Profile Engine for this circuit if needed
    if (!this.optimalEngine || this.optimalEngine.track !== track) {
      this.optimalEngine = new GlobalTimeOptimalEngine({ track });
    }

    // 2. Scan multi-agent traffic awareness & surface state
    const traffic = this.awareness.scan(vehicle, vehicles, track);
    const current = traffic.current;
    const isOffTrack = offRoad(current);

    const nominalHalfWidth = finite(track.roadHalfWidth, 6.5);
    const kerbAllowance = Math.min(1.2, finite(track.curbWidth, 1.05) * 0.95) * this._kerbUsage;
    const baseRoadMargin = Math.max(2.1, nominalHalfWidth - 1.15 + kerbAllowance);
    const currentSurfaceMargin = finite(
      track.planningLateralLimit?.(vehicle.distance, current?.lateral),
      nominalHalfWidth - 1.15
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

    const recovering = isOffTrack || edgeDeviation || this.recoveryTimer > 0 || this.stallTime > 0.7;

    // 3. Layer 2: Game-Theoretic Adversarial Corridor Planning
    const tactical = this.combatEngine.evaluate({
      vehicle,
      track,
      traffic,
      optimalProfile: this.optimalEngine,
      aggression: this._aggression,
      dt
    });

    const defending = tactical.role === 'DEFEND';
    const attacking = tactical.role === 'ATTACK';
    const targetOffset = clamp(tactical.targetLateral, -baseRoadMargin, baseRoadMargin);

    // 4. Multi-Candidate Frenet Lattice Trajectory Planning
    const currentPoint = track?.atDistance ? track.atDistance(vehicle.distance) : { curvature: 0 };
    const signedCurv = finite(currentPoint?.curvature, 0);
    const currentCurv = Math.abs(signedCurv);

    const lookAheadDist = recovering
      ? clamp(10.0 + vehicle.speed * 0.42, 10.0, 20.0)
      : clamp(11.0 + vehicle.speed * 0.58, 12.0, 30.0);

    const trackingDistance = clamp(lookAheadDist * 0.72 / (1.0 + currentCurv * 20.0), 5.5, 24.0);

    const maxTireWear = Math.max(0, ...(vehicle.wheels ?? []).map((w) => finite(w.wear, 0)));
    const tireGripFactor = clamp(1.0 - maxTireWear * 0.45, 0.80, 1.0);

    // Backward-integrated speed envelope reachability with Layer 1 apex optimization
    const physicalTargetSpeed = this.paceOptimizer.computeSpeedEnvelope({
      vehicle,
      track,
      tireGripFactor,
      skill: this.skill,
      aggression: this._aggression,
      defending,
      insideLineOffset: targetOffset
    });

    this.trajectoryPlan = this.trajectoryPlanner.plan({
      vehicle,
      track,
      desiredOffset: targetOffset,
      fallbackOffsets: recovering || defending ? [] : [targetOffset, finite(current?.lateral, 0)],
      trafficEntries: traffic.entries,
      targetSpeed: physicalTargetSpeed,
      aggression: this._aggression,
      racecraftPhase: defending ? tactical.defenseMode : attacking ? tactical.attackMode : 'NONE',
      targetId: tactical.attackMode !== 'NONE' ? this.combatEngine.attackTargetId : null,
      recovering,
      pitActive: Boolean(vehicle.pitIntent?.active),
      urgent: defending || attacking,
      roadMargin: baseRoadMargin,
      kerbAllowance,
      lookAhead: lookAheadDist,
      trackingDistance: (defending || attacking) ? Math.max(trackingDistance, clamp(vehicle.speed * 0.95, 12.0, 24.0)) : trackingDistance,
      referenceLineAtDistance: (s) => this.optimalEngine?.sampleAtDistance?.(s)?.lateral ?? 0
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
    const isFacingBackwards = Math.abs(yawAlignment) > Math.PI * 0.55;

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

    // 6. Longitudinal Desired Speed Synchronization
    let desiredSpeed = physicalTargetSpeed;
    const optSample = this.optimalEngine?.sampleAtDistance?.(vehicle.distance);
    if (optSample && tactical.role === 'PACE') {
      const scaledRef = optSample.targetSpeed * (0.98 + (this._aggression - 0.5) * 0.08);
      desiredSpeed = Math.min(physicalTargetSpeed, Math.max(desiredSpeed, scaledRef));
    } else if (tactical.desiredSpeed) {
      desiredSpeed = Math.min(physicalTargetSpeed, Math.max(desiredSpeed, tactical.desiredSpeed));
    }

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
      emergency: false,
      defending,
      tireGripFactor
    });

    // 8. Combat Dynamics & Slip-Slope Limit Tracking
    const finalControls = this.dynamicsEngine.process({
      vehicle,
      traffic,
      controls: {
        steer: this.steerCommand,
        throttle: rawPedals.throttle,
        brake: rawPedals.brake
      },
      dt
    });

    // Apply finalized controls to vehicle
    vehicle.controls.steer = finalControls.steer;
    vehicle.controls.throttle = finalControls.throttle;
    vehicle.controls.brake = finalControls.brake;
    vehicle.controls.handbrake = 0;
    this.steerCommand = finalControls.steer;

    // 9. Telemetry & Debug States
    this.debugState = {
      controllerVersion: 'V2-NextGen-Hybrid',
      tacticalMode: tactical.role,
      racecraftPhase: defending ? tactical.defenseMode : attacking ? tactical.attackMode : 'OPTIMAL_LINE',
      decisionReason: tactical.notes,
      targetSpeed: desiredSpeed,
      speedError,
      targetLateral: targetOffset,
      throttle: finalControls.throttle,
      brake: finalControls.brake,
      steer: finalControls.steer,
      latUtilization: rawPedals.friction?.latUtilization ?? 0,
      remainingLongBudget: rawPedals.friction?.remainingLongBudget ?? 1,
      liveLatG: liveLatAccel / 9.81,
      peakLatG: rawPedals.friction?.peakG ?? 2.70,
      rubbingActive: finalControls.rubbing,
      powerSlideActive: finalControls.powerSlide,
      defenseThreatScore: defending ? 0.85 : 0.0,
      attackIntensity: attacking ? 0.95 : 0.0,
      trajectoryScore: this.trajectoryPlan?.score ?? 0,
      trajectoryCollisionFree: this.trajectoryPlan?.collisionFree ?? true,
      trajectoryRoadLegal: this.trajectoryPlan?.roadLegal ?? true,
      trajectorySelectedOffsetM: targetOffset
    };
  }
}
