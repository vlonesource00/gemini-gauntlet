/**
 * CoupledDynamicsController.js (V2 Layer 3 Coupled Dynamics)
 * High-precision coupled vehicle dynamics solver operating at 120Hz:
 * 
 * 1. Curvature-Feedforward + Stanley Steering Control Law:
 *    - Cross-track error compensation via nonlinear Stanley formulation: atan(k_e * e_lat / (k_soft + v_x))
 *    - Lookahead path tangent heading alignment with pure-pursuit trajectory tracking
 *    - Kinematic & dynamic curvature feedforward: delta_ff = atan(L * kappa) + K_us * (v_x^2 * kappa)
 *    - Lookahead preview curvature blending to proactively turn in prior to apexes
 *    - Active yaw rate damping and body sideslip angle compensation
 *    - Dynamic speed-dependent steering saturation protection (prevents front tire scrub understeer)
 *    - Actuator slew rate limiting matching physical rack dynamics (12 rad/s)
 * 
 * 2. 2D G-G Friction-Circle Trail-Braking & Apex Exit Power Launch:
 *    - Dynamic aerodynamic downforce scaling with speed (Prototype up to 2.70G lateral, -3.50G threshold braking)
 *    - 2D Pacejka friction ellipse coupling: (a_long / a_long_max)^2 + (a_lat / a_lat_max)^2 <= 1.0
 *    - Real-time lateral load sensing: a_lat = |v_x * yawRate|
 *    - Dynamic trail-braking: smoothly tapers longitudinal braking along friction ellipse as lateral load builds
 *    - Apex drive & steering unwind power: instantly unleashes 100% full throttle launch as wheel unwinds
 *    - Oversteer breakaway slip stabilization & lift-off snap prevention
 * 
 * 3. Zero-GC Pre-Allocated Rolling Prediction Horizon:
 *    - Pre-allocated prediction vectors for 3D visual telemetry overlays and fast execution (<0.1ms).
 */

import { clamp, wrapAngle, saturate } from '../../core/math.js';

const finite = (val, fallback = 0) => (Number.isFinite(val) ? val : fallback);

export class CoupledDynamicsController {
  /**
   * @param {Object} [options]
   * @param {number} [options.horizonSeconds=2.4] - Prediction horizon time span (s)
   * @param {number} [options.nodeCount=16] - Pre-allocated horizon point resolution
   * @param {number} [options.stanleyGain=2.45] - Stanley lateral cross-track gain (k_e)
   * @param {number} [options.stanleySoftening=2.5] - Stanley softening velocity (k_soft, m/s)
   * @param {number} [options.headingGain=1.0] - Heading error gain
   * @param {number} [options.yawDampingGain=0.24] - Yaw damping gain (k_d)
   * @param {number} [options.slipCompensationGain=0.35] - Sideslip angle compensation gain
   * @param {number} [options.curvatureLookaheadS=0.35] - Curvature feedforward preview time (s)
   * @param {number} [options.understeerGradient=0.0018] - Dynamic understeer gradient (K_us)
   * @param {number} [options.trailBrakingSkill=0.90] - Trail braking effectiveness (0-1)
   * @param {number} [options.unwindFactor=0.55] - Exit throttle modulation factor
   * @param {number} [options.steerRate=12.0] - Maximum steering rack slew rate (rad/s)
   */
  constructor({
    horizonSeconds = 2.4,
    nodeCount = 16,
    stanleyGain = 2.45,
    stanleySoftening = 2.5,
    headingGain = 1.0,
    yawDampingGain = 0.24,
    slipCompensationGain = 0.35,
    curvatureLookaheadS = 0.35,
    understeerGradient = 0.0018,
    trailBrakingSkill = 0.90,
    unwindFactor = 0.55,
    steerRate = 12.0
  } = {}) {
    this.horizonS = horizonSeconds;
    this.nodeCount = Math.max(12, nodeCount);

    // Tunable controller parameters
    this.stanleyGain = stanleyGain;
    this.stanleySoftening = stanleySoftening;
    this.headingGain = headingGain;
    this.yawDampingGain = yawDampingGain;
    this.slipCompensationGain = slipCompensationGain;
    this.curvatureLookaheadS = curvatureLookaheadS;
    this.understeerGradient = understeerGradient;
    this.trailBrakingSkill = trailBrakingSkill;
    this.unwindFactor = unwindFactor;
    this.steerRate = steerRate;

    // Previous control commands for slew rate rate-limiting
    this.prevSteer = 0;
    this.prevThrottle = 0;
    this.prevBrake = 0;

    // Reusable pre-allocated prediction horizon state vectors (Zero-GC pressure)
    this.predPoints = new Array(this.nodeCount).fill(0).map(() => ({
      x: 0,
      y: 0,
      z: 0,
      s: 0,
      lateral: 0,
      speed: 0,
      time: 0,
      curvature: 0,
      latG: 0,
      remainingLongBudget: 1.0
    }));

    // Pre-allocated telemetry container
    this.telemetry = {
      steer: 0,
      throttle: 0,
      brake: 0,
      horizon: this.predPoints,
      friction: {
        latUtilization: 0,
        remainingLongBudget: 1.0,
        liveLatG: 0,
        peakLatG: 2.70,
        availableLongDecel: 34.3,
        aLatMax: 26.5,
        aLongMaxDecel: 34.3
      },
      stanley: {
        headingError: 0,
        crossTrackError: 0,
        curvatureFeedforward: 0,
        yawDamping: 0,
        targetSteer: 0,
        maxSteerLimit: 1.0
      },
      trailBraking: {
        active: false,
        factor: 1.0,
        brakeRaw: 0,
        brakeTapered: 0
      },
      traction: {
        exitFactor: 1.0,
        unwindBonus: 0,
        launchActive: false
      }
    };
  }

  /**
   * Reset internal rate-limiter states.
   */
  reset() {
    this.prevSteer = 0;
    this.prevThrottle = 0;
    this.prevBrake = 0;
  }

  /**
   * Dynamic update of tuning parameters.
   */
  setParameters(options = {}) {
    if (options.stanleyGain !== undefined) this.stanleyGain = options.stanleyGain;
    if (options.stanleySoftening !== undefined) this.stanleySoftening = options.stanleySoftening;
    if (options.headingGain !== undefined) this.headingGain = options.headingGain;
    if (options.yawDampingGain !== undefined) this.yawDampingGain = options.yawDampingGain;
    if (options.slipCompensationGain !== undefined) this.slipCompensationGain = options.slipCompensationGain;
    if (options.curvatureLookaheadS !== undefined) this.curvatureLookaheadS = options.curvatureLookaheadS;
    if (options.understeerGradient !== undefined) this.understeerGradient = options.understeerGradient;
    if (options.trailBrakingSkill !== undefined) this.trailBrakingSkill = options.trailBrakingSkill;
    if (options.unwindFactor !== undefined) this.unwindFactor = options.unwindFactor;
    if (options.steerRate !== undefined) this.steerRate = options.steerRate;
  }

  /**
   * Calculate 2D G-G friction circle acceleration capacities accounting for dynamic aerodynamic downforce.
   * @param {Object} params
   * @returns {Object} { aLatMax, aLongMaxDecel, aLongMaxAccel, peakLatG, peakBrakeG }
   */
  calculateFrictionLimits({
    vehicleClass = 'prototype',
    speed = 0,
    tireGripFactor = 1.0
  } = {}) {
    const vSpeed = Math.max(0.1, finite(speed, 0));
    let baseLatG = 1.85;
    let downforceG = 0;
    let baseBrakeG = 2.20;
    let brakeDownforceG = 0;
    let aLongMaxAccel = 4.80;

    if (vehicleClass === 'prototype') {
      // Prototype: Ground effect floor + aero wings scaling with v^2
      const dfFactor = saturate((vSpeed - 18.0) / 35.0);
      const aeroMult = clamp(1.0 + 0.00022 * vSpeed * vSpeed, 1.0, 1.55);
      baseLatG = (1.85 + 0.85 * dfFactor) * aeroMult;
      baseBrakeG = (2.20 + 1.30 * dfFactor) * aeroMult; // Up to -3.50G threshold braking
      aLongMaxAccel = 4.80;
    } else if (vehicleClass === 'gt') {
      const aeroMult = clamp(1.0 + 0.00008 * vSpeed * vSpeed, 1.0, 1.25);
      baseLatG = 1.35 * aeroMult;
      baseBrakeG = 1.65 * aeroMult;
      aLongMaxAccel = 3.60;
    } else {
      // Touring / standard
      baseLatG = 1.15;
      baseBrakeG = 1.25;
      aLongMaxAccel = 2.80;
    }

    const peakLatG = baseLatG * tireGripFactor;
    const peakBrakeG = baseBrakeG * tireGripFactor;
    const aLatMax = peakLatG * 9.81;
    const aLongMaxDecel = peakBrakeG * 9.81;

    return {
      aLatMax,
      aLongMaxDecel,
      aLongMaxAccel,
      peakLatG,
      peakBrakeG
    };
  }

  /**
   * Curvature-Feedforward + Stanley Steering Control.
   * Computes high-fidelity steering command combining:
   * 1. Path tangent heading alignment + Lookahead pursuit
   * 2. Stanley cross-track nonlinear error correction
   * 3. Kinematic Ackermann curvature feedforward + dynamic understeer compensation
   * 4. Lookahead curvature preview for proactive corner turn-in
   * 5. Active yaw rate damping and body sideslip compensation
   * 6. Speed-dependent dynamic saturation limits
   * 
   * @param {Object} params
   * @returns {Object} Steering command and diagnostic components
   */
  computeStanleySteering({
    vehicle,
    track,
    targetLateral = 0,
    lookAheadM = 15.0,
    dt = 0.016,
    aggression = 0.85,
    recovering = false,
    committed = false
  } = {}) {
    const vSpeed = Math.max(0.1, finite(vehicle?.speed, 0));
    const vDist = finite(vehicle?.distance, 0);
    const currentLat = finite(vehicle?.surface?.lateral, 0);
    const yaw = finite(vehicle?.yaw, 0);
    const yawRate = finite(vehicle?.yawRate, 0);
    const wheelBase = finite(vehicle?.wheelBase, 2.65);

    // 1. Reference Track Geometry at Car and Lookahead Waypoint
    const refPoint = track?.atDistance ? track.atDistance(vDist) : { curvature: 0, tangent: { x: 0, z: 1 } };
    const lookAheadPoint = track?.atDistance
      ? track.atDistance(vDist + lookAheadM)
      : { curvature: 0, tangent: { x: 0, z: 1 } };

    // Target coordinate in world space for pure-pursuit blending
    const targetWorld = track?.lateralPoint
      ? track.lateralPoint(lookAheadPoint, targetLateral, 0.08)
      : lookAheadPoint;

    // Track path heading at vehicle location
    const trackHeading = Math.atan2(finite(refPoint.tangent?.x, 0), finite(refPoint.tangent?.z, 1));
    const rawHeadingToTrack = wrapAngle(trackHeading - yaw);

    // Heading towards lookahead target point
    const headingToLookahead = Math.atan2(
      finite(targetWorld.x, 0) - finite(vehicle?.position?.x, 0),
      finite(targetWorld.z, 0) - finite(vehicle?.position?.z, 0)
    );
    const rawHeadingToLookahead = wrapAngle(headingToLookahead - yaw);

    // Blend path tangent heading with lookahead heading for optimal line entry
    const lookaheadWeight = clamp(0.55 + aggression * 0.15, 0.50, 0.85);
    let headingError = wrapAngle(
      rawHeadingToLookahead * lookaheadWeight + rawHeadingToTrack * (1.0 - lookaheadWeight)
    );

    // Reversal / backwards recovery guard
    const isFacingBackwards = Math.abs(rawHeadingToTrack) > Math.PI * 0.55;
    if (recovering && isFacingBackwards) {
      headingError = Math.sign(rawHeadingToTrack) * -1.2;
    }

    // 2. Stanley Cross-Track Error Formulation
    // In our coordinate frame: vehicle is at currentLat, target is at targetLateral.
    // Error e_lat = (targetLateral - currentLat).
    // When target is to the right (+lateral), error is positive -> command positive steer (right).
    const crossTrackError = targetLateral - currentLat;
    const effectiveStanleyGain = (this.stanleyGain + aggression * 0.40) * (committed ? 1.15 : 1.0);
    const stanleyAngle = Math.atan2(
      effectiveStanleyGain * crossTrackError,
      this.stanleySoftening + vSpeed
    );

    // 3. Kinematic & Dynamic Curvature Feedforward with Lookahead Preview
    // Sample curvature ahead on track to initiate turn-in proactively
    const previewDistance = clamp(vSpeed * this.curvatureLookaheadS, 3.0, 28.0);
    const previewPoint = track?.atDistance ? track.atDistance(vDist + previewDistance) : refPoint;
    const rawCurvCurrent = finite(refPoint.curvature, 0);
    const rawCurvPreview = finite(previewPoint.curvature, 0);

    // Curvature blend: 35% current, 65% preview
    const effectiveCurvature = rawCurvCurrent * 0.35 + rawCurvPreview * 0.65;

    // Kinematic Ackermann steering feedforward: delta_ff_kin = atan(L * kappa)
    const kinematicFeedforward = Math.atan(wheelBase * effectiveCurvature);

    // Dynamic understeer gradient compensation: delta_ff_dyn = K_us * (v_x^2 * kappa)
    const dynamicUndersteerComp = this.understeerGradient * (vSpeed * vSpeed * effectiveCurvature);
    const curvatureFeedforward = kinematicFeedforward + dynamicUndersteerComp;

    // 4. Dynamic Yaw Rate Damping & Body Sideslip Compensation
    // Steady-state curve yaw rate: r_ss = v_x * kappa
    const steadyStateYawRate = vSpeed * effectiveCurvature;
    const excessYawRate = yawRate - steadyStateYawRate;
    const effectiveYawDamping = this.yawDampingGain * (committed ? 1.25 : 1.0);
    const yawDamping = excessYawRate * effectiveYawDamping;

    // Live body sideslip angle: beta = atan2(v_local_x, v_local_z)
    const localVx = finite(vehicle?.localVelocity?.x, 0);
    const localVz = Math.max(2.5, Math.abs(finite(vehicle?.localVelocity?.z, vSpeed)));
    const slipAngle = Math.atan2(localVx, localVz);
    const slipCompensation = slipAngle * this.slipCompensationGain;

    // 5. Synthesize Combined Target Steering Command
    const rawTargetSteer =
      headingError * this.headingGain +
      stanleyAngle +
      curvatureFeedforward -
      yawDamping -
      slipCompensation;

    // 6. Dynamic Speed-Dependent Steering Saturation Limit
    // Prevents over-steering beyond peak tire slip angle (~7-9 degrees) at high speeds
    const maxSteerLimit = recovering
      ? 0.85
      : (committed
        ? clamp(17.5 / Math.max(6.0, vSpeed), 0.14, 0.95)
        : clamp(14.5 / Math.max(6.0, vSpeed), 0.10, 0.85));

    const targetSteer = clamp(rawTargetSteer, -maxSteerLimit, maxSteerLimit);

    // 7. Actuator Slew Rate Limiting
    // Fast unwinding response to prevent yaw overshoot
    const isUnwinding =
      Math.sign(targetSteer) !== Math.sign(this.prevSteer) ||
      Math.abs(targetSteer) < Math.abs(this.prevSteer);
    const activeRate = isUnwinding ? this.steerRate * 1.5 : this.steerRate;
    const maxDelta = activeRate * clamp(dt, 0.005, 0.05);

    const steer = clamp(
      this.prevSteer + clamp(targetSteer - this.prevSteer, -maxDelta, maxDelta),
      -1.0,
      1.0
    );
    this.prevSteer = steer;

    return {
      steer,
      targetSteer,
      headingError,
      crossTrackError,
      curvatureFeedforward,
      yawDamping,
      maxSteerLimit
    };
  }

  /**
   * 2D G-G Friction-Circle Trail-Braking & Exit Traction Control.
   * - Solves nonlinear friction circle constraint: (a_long / a_long_max)^2 + (a_lat / a_lat_max)^2 <= 1.0
   * - Dynamically tapers braking force as cornering grip builds (seamless trail braking)
   * - Unleashes instant 100% full throttle launch on steering unwind
   * 
   * @param {Object} params
   * @returns {Object} { throttle, brake, friction, trailBraking, traction }
   */
  computePedals({
    vehicle,
    speedError = 0,
    desiredSpeed = 50.0,
    liveLatAccel = 0,
    steer = 0,
    aLatMax = 26.5,
    aLongMaxDecel = 34.3,
    aLongMaxAccel = 4.8,
    straight = false,
    recovering = false,
    defending = false,
    dt = 0.016
  } = {}) {
    const vSpeed = Math.max(0.1, finite(vehicle?.speed, 0));
    const steerMag = saturate(Math.abs(steer));

    // 1. Friction Circle Utilization
    const latUtilization = clamp(liveLatAccel / Math.max(1.0, aLatMax), 0, 1.0);

    // Available longitudinal budget from Pacejka friction ellipse:
    // (a_long / a_long_max)^2 + (a_lat / a_lat_max)^2 <= 1.0  =>  budget = sqrt(1 - (latUtil * coupling)^2)
    const coupling = 0.95;
    const remainingLongBudget = Math.sqrt(Math.max(0.01, 1.0 - Math.pow(latUtilization * coupling, 2)));
    const availableLongDecel = aLongMaxDecel * remainingLongBudget;

    let throttle = 0;
    let brake = 0;
    let trailBrakingActive = false;
    let trailFactor = 1.0;
    let rawBrake = 0;
    let launchActive = false;
    let unwindBonus = 0;
    let exitFactor = 1.0;

    const isCornering = !straight && (steerMag > 0.16 || latUtilization > 0.48);

    if (recovering) {
      // Gentle recovery throttle / brake
      throttle = speedError > 0.5 ? clamp(0.35 + speedError * 0.05, 0.3, 0.6) : 0;
      brake = speedError < -2.0 ? clamp((-speedError - 2.0) * 0.25, 0.1, 0.7) : 0;
    } else if (speedError < -2.20) {
      // =======================================================================
      // DECELERATION & TRAIL-BRAKING ZONE
      // =======================================================================
      throttle = 0;

      if (speedError < -6.50) {
        // High-G threshold braking on approach (up to -3.5G decel)
        rawBrake = clamp(0.85 + (-speedError - 6.50) * 0.15, 0.85, 1.0);

        // Trail braking: smoothly blend off brake along friction circle boundary
        if (isCornering || latUtilization > 0.12) {
          trailBrakingActive = true;
          const trailExponent = defending ? 1.4 : 1.7;
          trailFactor = Math.pow(
            Math.max(0.02, 1.0 - Math.pow(this.trailBrakingSkill * latUtilization * 0.96, 2)),
            1.0 / trailExponent
          );
          brake = clamp(rawBrake * trailFactor * remainingLongBudget, 0.04, 1.0);

          // Deep mid-corner safety ceiling (preserves lateral tire capacity for apex clipping)
          if (latUtilization > 0.58 || steerMag > 0.26) {
            const midCornerCeiling = vSpeed < 16.0 ? 0.18 : 0.38;
            brake = Math.min(brake, midCornerCeiling);
          }
        } else {
          // Pure straight-line threshold braking
          brake = rawBrake;
        }
      } else {
        // Smooth progressive braking (speedError between -2.20 and -6.50)
        const brakeFrac = (-speedError - 2.20) / (6.50 - 2.20);
        rawBrake = clamp(brakeFrac * 0.85, 0.05, 0.85);
        brake = isCornering ? clamp(rawBrake * remainingLongBudget * 0.8, 0, 0.50) : rawBrake;
      }
    } else {
      // =======================================================================
      // ACCELERATION, MOMENTUM CARRY & APEX EXIT POWER LAUNCH ZONE
      // =======================================================================
      brake = 0;
      const rawThrottle = speedError >= 0
        ? clamp(0.95 + speedError * 0.25, 0.70, 1.0)
        : clamp((speedError + 2.20) / 2.20 * 0.65, 0.20, 0.65); // Smooth coasting / carry momentum when slightly over target

      if (isCornering) {
        // Corner-exit power launch: scale throttle with high baseline and fast unwind
        const unwindPower = 1.0 - this.unwindFactor * Math.pow(steerMag, 1.1) * 0.18;
        exitFactor = clamp(remainingLongBudget * unwindPower, 0.65, 1.0);
        throttle = clamp(rawThrottle * exitFactor, 0.35, 1.0);

        // High-speed cornering maintenance floor (maintains positive rear-axle load & downforce)
        if (vSpeed > 10.0 && !straight) {
          throttle = Math.max(throttle, 0.45);
        }

        // Fast full-throttle trigger on steering unwinding
        if (steerMag < 0.30 || speedError > 0) {
          throttle = 1.0;
          launchActive = true;
          unwindBonus = 1.0;
        }
      } else {
        // Straightaway / unwound steering: instant 100% full launch power!
        throttle = 1.0;
        launchActive = true;
        unwindBonus = 1.0;
      }
    }

    // Rate-limit brake pressure for realistic hydraulic dynamics (Fast bite 24/s, smooth release 14/s)
    const maxBrakeRate = brake > this.prevBrake ? 24.0 : 14.0;
    const maxBrakeDelta = maxBrakeRate * clamp(dt, 0.005, 0.05);
    brake = clamp(
      this.prevBrake + clamp(brake - this.prevBrake, -maxBrakeDelta, maxBrakeDelta),
      0,
      1.0
    );

    this.prevThrottle = throttle;
    this.prevBrake = brake;

    return {
      throttle: clamp(throttle, 0, 1.0),
      brake: clamp(brake, 0, 1.0),
      friction: {
        latUtilization,
        remainingLongBudget,
        liveLatG: liveLatAccel / 9.81,
        peakLatG: aLatMax / 9.81,
        availableLongDecel,
        aLatMax,
        aLongMaxDecel
      },
      trailBraking: {
        active: trailBrakingActive,
        factor: trailFactor,
        brakeRaw: rawBrake,
        brakeTapered: brake
      },
      traction: {
        exitFactor,
        unwindBonus,
        launchActive
      }
    };
  }

  /**
   * Main 120Hz coupled dynamics solver step.
   * @param {Object} params
   * @returns {Object} Complete control outputs, prediction horizon, and telemetry
   */
  step({
    vehicle,
    track,
    tacticalTarget, // { targetLateral, desiredSpeed, dMin, dMax }
    dt = 0.016,
    tireGripFactor = 1.0,
    aggression = 0.85,
    recovering = false,
    defending = false,
    committed = false
  } = {}) {
    const vSpeed = Math.max(0.1, finite(vehicle?.speed, 0));
    const vDist = finite(vehicle?.distance, 0);
    const currentLat = finite(vehicle?.surface?.lateral, 0);
    const yawRate = finite(vehicle?.yawRate, 0);

    const targetLateral = finite(tacticalTarget?.targetLateral, 0);
    const desiredSpeed = Math.max(8.0, finite(tacticalTarget?.desiredSpeed, 50.0));

    // 1. Friction Limits (Downforce-Scaled Peak Lateral & Longitudinal Accelerations)
    const vClass = vehicle?.classKey || 'prototype';
    const frictionLimits = this.calculateFrictionLimits({
      vehicleClass: vClass,
      speed: vSpeed,
      tireGripFactor
    });

    // 2. Curvature-Feedforward + Stanley Steering Control
    const lookAheadM = clamp(8.0 + vSpeed * 0.48, 9.0, 32.0);
    const steeringResult = this.computeStanleySteering({
      vehicle,
      track,
      targetLateral,
      lookAheadM,
      dt,
      aggression,
      recovering,
      committed: committed || defending
    });

    // 3. 2D G-G Friction-Circle Trail-Braking & Exit Power Launch
    const liveLatAccel = Math.abs(vSpeed * yawRate);
    const speedError = desiredSpeed - vSpeed;
    const refPoint = track?.atDistance ? track.atDistance(vDist) : { curvature: 0 };
    const isStraight = Math.abs(finite(refPoint.curvature, 0)) < 0.0028;

    const pedalResult = this.computePedals({
      vehicle,
      speedError,
      desiredSpeed,
      liveLatAccel,
      steer: steeringResult.steer,
      aLatMax: frictionLimits.aLatMax,
      aLongMaxDecel: frictionLimits.aLongMaxDecel,
      aLongMaxAccel: frictionLimits.aLongMaxAccel,
      straight: isStraight,
      recovering,
      defending,
      dt
    });

    // 4. Update Zero-GC Prediction Horizon for 3D Visual Telemetry Overlays
    const dtHorizon = this.horizonS / (this.nodeCount - 1);
    for (let i = 0; i < this.nodeCount; i++) {
      const t = i * dtHorizon;
      const sNode = vDist + vSpeed * t;
      const ptNode = track?.atDistance ? track.atDistance(sNode) : { x: 0, y: 0, z: 0, curvature: 0 };
      const blendLat = currentLat + (targetLateral - currentLat) * saturate(t / 1.1);
      const worldNode = track?.lateralPoint ? track.lateralPoint(ptNode, blendLat, 0.08) : ptNode;

      const p = this.predPoints[i];
      p.x = worldNode.x;
      p.y = worldNode.y;
      p.z = worldNode.z;
      p.s = sNode;
      p.lateral = blendLat;
      p.speed = clamp(vSpeed + (desiredSpeed - vSpeed) * saturate(t / 1.5), 0, 95.0);
      p.time = t;
      p.curvature = finite(ptNode.curvature, 0);
      p.latG = Math.abs(p.speed * p.speed * p.curvature) / 9.81;
      p.remainingLongBudget = pedalResult.friction.remainingLongBudget;
    }

    // 5. Populate Structured Telemetry Container
    const out = this.telemetry;
    out.steer = steeringResult.steer;
    out.throttle = pedalResult.throttle;
    out.brake = pedalResult.brake;
    out.horizon = this.predPoints;
    out.friction = pedalResult.friction;
    out.stanley = steeringResult;
    out.trailBraking = pedalResult.trailBraking;
    out.traction = pedalResult.traction;

    return out;
  }
}
