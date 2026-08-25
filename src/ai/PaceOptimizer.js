/**
 * PaceOptimizer.js
 * Modular Edge Vehicle Dynamics, Car Control & Pace Optimization Engine.
 * Features:
 * - 2D G-G Friction Circle Tire Load Modeling (Pacejka friction ellipse)
 * - Precision Friction-Circle Trail-Braking Modulation
 * - Backward Reachable Speed Envelope across multi-distance lookaheads
 * - Traction Control System (TCS): progressive friction-circle exit throttle modulation preventing power-oversteer snap
 * - Dynamic Steering Saturation Limiters: preventing front tire scrub understeer and high-speed yaw snap
 * - Dirty Air & Turbulence Downforce Compensation
 * - Smooth Off-Track Rejoin & Low-Grip Surface Recovery Control
 */

import { clamp, wrapAngle, saturate } from '../core/math.js';

const finite = (value, fallback = 0) => (Number.isFinite(value) ? value : fallback);

export class PaceOptimizer {
  /**
   * @param {Object} [options]
   * @param {number} [options.trailBrakingSkill=0.85] - Driver trail braking proficiency (0-1)
   * @param {number} [options.unwindFactor=0.60] - Exit throttle modulation intensity (0-1)
   * @param {number} [options.tcsSensitivity=0.90] - Traction control intervention sensitivity (0-1)
   * @param {number} [options.steerLimitGain=1.0] - Front tire saturation limiter scaling
   */
  constructor({
    trailBrakingSkill = 0.85,
    unwindFactor = 0.60,
    tcsSensitivity = 0.90,
    steerLimitGain = 1.0
  } = {}) {
    this.trailBrakingSkill = clamp(trailBrakingSkill, 0, 1);
    this.unwindFactor = clamp(unwindFactor, 0, 1);
    this.tcsSensitivity = clamp(tcsSensitivity, 0, 1);
    this.steerLimitGain = clamp(steerLimitGain, 0.5, 1.5);
  }

  setParameters({ trailBrakingSkill, unwindFactor, tcsSensitivity, steerLimitGain } = {}) {
    if (Number.isFinite(trailBrakingSkill)) this.trailBrakingSkill = clamp(trailBrakingSkill, 0, 1);
    if (Number.isFinite(unwindFactor)) this.unwindFactor = clamp(unwindFactor, 0, 1);
    if (Number.isFinite(tcsSensitivity)) this.tcsSensitivity = clamp(tcsSensitivity, 0, 1);
    if (Number.isFinite(steerLimitGain)) this.steerLimitGain = clamp(steerLimitGain, 0.5, 1.5);
  }

  /**
   * Compute maximum physical cornering speed based on curvature, banking, aero downforce, and dirty air.
   * Calibrated for Prototype class up to 26.5 m/s² (2.70G) to reflect true ground-effect downforce.
   * @param {Object} params
   * @returns {number} Corner apex speed limit in m/s
   */
  calculateCornerSpeed({
    curvature = 0,
    banking = 0,
    vehicleClass = 'prototype',
    tireGripFactor = 1.0,
    skill = 0.85,
    dirtyAirLoss = 0
  } = {}) {
    const kappa = Math.max(1e-5, Math.abs(finite(curvature, 0)));
    const vEst = Math.sqrt(9.81 * 1.55 / kappa);
    const downforceFactor = vehicleClass === 'prototype' ? saturate((vEst - 16.0) / 38.0) : 0;
    
    // Aerodynamic downforce scaling with speed squared
    const aeroDownforceMultiplier = vehicleClass === 'prototype'
      ? clamp(1.0 + 0.00018 * vEst * vEst, 1.0, 1.50)
      : vehicleClass === 'gt'
        ? clamp(1.0 + 0.00006 * vEst * vEst, 1.0, 1.18)
        : 1.0;

    // Compensate for dirty air front downforce loss
    const aeroEffective = Math.max(0.72, aeroDownforceMultiplier * (1.0 - clamp(dirtyAirLoss, 0, 0.35)));

    // Calibrated realistic mechanical + aero lateral G
    const classBaseG = vehicleClass === 'prototype' ? (1.38 + 0.95 * downforceFactor) : vehicleClass === 'gt' ? 1.15 : 0.95;
    const peakG = classBaseG * tireGripFactor * aeroEffective * (0.86 + skill * 0.14);
    const g = 9.81;

    // Banking bonus: a_lat_eff = g * (peakG * cos(theta) + sin(theta))
    const bankAngle = Math.abs(finite(banking, 0));
    const bankCarry = Math.sin(bankAngle) * (vehicleClass === 'prototype' ? 1.35 : 1.05);
    const effectiveLatAccel = g * (peakG * Math.cos(bankAngle) + bankCarry);

    const classMargin = vehicleClass === 'prototype' ? 0.98 : vehicleClass === 'gt' ? 0.95 : 0.88;
    return Math.sqrt(effectiveLatAccel / kappa) * classMargin;
  }

  /**
   * Compute backward-reachable speed envelope across upcoming lookahead horizons.
   * Ensures the vehicle starts braking at the exact physical threshold for every upcoming corner.
   * @param {Object} params
   * @returns {number} Target physical speed limit at current vehicle position
   */
  computeSpeedEnvelope({
    vehicle,
    track,
    tireGripFactor = 1.0,
    skill = 0.85,
    aggression = 0.80,
    defending = false,
    threatScore = 0,
    closingSpeed = 0,
    insideLineOffset = 0,
    dirtyAirLoss = 0
  } = {}) {
    const vClass = vehicle?.classKey || 'prototype';
    // Calibrated sustained braking deceleration (m/s²) with realistic tire grip & ABS envelope
    const brakingDecel = (vClass === 'prototype' ? 9.5 : vClass === 'gt' ? 7.6 : 5.8) * tireGripFactor * (0.88 + aggression * 0.16);
    const speedEnvelopeDistances = [
      0, 2, 4, 6, 8, 10, 13, 16, 20, 24, 28, 33, 38, 44, 50, 58, 66, 75, 85, 96, 108, 122, 138, 155, 175, 198, 225, 255, 290, 330, 375
    ];

    let speedLimit = 95.0; // max track velocity ceiling
    const vSpeed = finite(vehicle?.speed, 0);
    const previewBuffer = Math.max(0, vSpeed * 0.12);

    for (const dist of speedEnvelopeDistances) {
      const sampleDist = finite(vehicle?.distance, 0) + dist;
      const point = track?.atDistance ? track.atDistance(sampleDist) : { curvature: 0, banking: 0 };
      
      let rawCurvature = Math.abs(finite(point.curvature, 0));
      const roadWidth = finite(track?.roadHalfWidth, 6.5) + finite(track?.curbWidth, 1.05);
      const flattenFactor = (vClass === 'prototype')
        ? clamp(1.0 + 0.35 * roadWidth * Math.min(0.035, rawCurvature), 1.0, 1.25)
        : 1.0;
      let curvature = rawCurvature / flattenFactor;

      if (Math.abs(insideLineOffset) > 0.5 && curvature > 1e-4) {
        const isInsideOffset = (Math.sign(point.curvature) * insideLineOffset) > 0;
        if (isInsideOffset) {
          curvature = curvature / Math.max(0.45, 1.0 - Math.min(0.55, Math.abs(insideLineOffset) * curvature));
        }
      }

      const safetyFactor = aggression > 0.85 ? 0.98 : (defending ? 0.95 : 0.96);
      const physLimit = this.calculateCornerSpeed({
        curvature,
        banking: point.banking,
        vehicleClass: vClass,
        tireGripFactor,
        skill,
        dirtyAirLoss
      }) * safetyFactor;

      const cornerSpeed = Math.max(5.5, physLimit);
      const effectiveDist = Math.max(0, dist - previewBuffer);
      const reachableSpeed = Math.sqrt(cornerSpeed * cornerSpeed + 2.0 * brakingDecel * effectiveDist);
      speedLimit = Math.min(speedLimit, reachableSpeed);
    }

    return speedLimit;
  }

  /**
   * Evaluate G-G friction circle tire utilization and available deceleration.
   * Friction ellipse: (a_long / a_long_max)^2 + (a_lat / a_lat_max)^2 <= 1.0
   * @param {Object} params
   * @returns {Object} Friction circle metrics
   */
  evaluateFrictionCircle({
    vehicle,
    lateralAccel = 0,
    tireGripFactor = 1.0
  }) {
    const vClass = vehicle?.classKey || 'prototype';
    const classBaseG = vClass === 'prototype' ? 2.70 : vClass === 'gt' ? 1.75 : 1.40;
    const maxTotalAccel = classBaseG * 9.81 * tireGripFactor;

    const actualLatAccel = Math.min(Math.abs(finite(lateralAccel, 0)), maxTotalAccel);
    const latUtilization = saturate(actualLatAccel / maxTotalAccel);

    // Available longitudinal deceleration inside friction ellipse
    const availableLongDecel = maxTotalAccel * Math.sqrt(Math.max(0, 1.0 - latUtilization * latUtilization));

    return {
      maxTotalAccel,
      actualLatAccel,
      latUtilization,
      availableLongDecel,
      totalUtilization: latUtilization,
      peakG: classBaseG * tireGripFactor,
      peakLatG: classBaseG * tireGripFactor
    };
  }

  /**
   * Dynamic Steering Limiters & Saturation Prevention:
   * Prevents front tire scrub understeer and high-speed yaw snap with speed-adaptive saturation limits.
   * @param {Object} params
   * @returns {number} Saturated steering command [-1, 1]
   */
  computeSteering({
    previous = 0,
    headingError = 0,
    lateralError = 0,
    yawRate = 0,
    slipAngle = 0,
    speed = 0,
    currentCurvature = 0,
    dt = 0.016,
    committed = false,
    recovering = false,
    yielding = false,
    dirtyAirLoss = 0
  } = {}) {
    const isEmergencyTurnaround = recovering && Math.abs(finite(headingError)) > 1.4;
    const headingGain = isEmergencyTurnaround ? 2.80 : (committed ? 2.60 : 2.25);
    const yawDamping = isEmergencyTurnaround ? 0.18 : (committed ? 0.38 : 0.42);

    const vSpeed = Math.max(3.0, finite(speed, 0));

    // Dynamic speed-dependent steering saturation limit:
    // Prevents over-turning the steering rack at speed which would saturate front slip angles into scrubbing understeer
    const baseLimit = committed
      ? clamp(3.2 / Math.max(3.5, vSpeed) + 0.12, 0.12, 0.48)
      : clamp(2.8 / Math.max(3.5, vSpeed) + 0.10, 0.08, 0.44);

    const maxUsableSteer = isEmergencyTurnaround
      ? 0.85
      : baseLimit * this.steerLimitGain;

    // Curvature feedforward
    const wheelBase = 2.80;
    const curvatureFeedforward = Math.atan(wheelBase * finite(currentCurvature, 0));

    // Counter-steer to catch oversteer breakaway slides
    const liveSlip = finite(slipAngle, 0);
    const counterSteer = Math.abs(liveSlip) > 0.08
      ? -Math.sign(liveSlip) * clamp((Math.abs(liveSlip) - 0.06) * 2.2, 0, 0.55)
      : 0;

    const maxAllowedSteer = Math.abs(counterSteer) > 0.05
      ? Math.max(maxUsableSteer, 0.65)
      : maxUsableSteer;

    // Direct trajectory tracking with curvature feedforward, yaw damping, and active counter-steer
    let target = clamp(
      curvatureFeedforward * 0.85 + finite(headingError) * headingGain - finite(yawRate) * yawDamping + counterSteer,
      -maxAllowedSteer,
      maxAllowedSteer
    );

    if (yielding) target = clamp(target, -0.3, 0.3);

    // Fast unwind rate when returning to center prevents yaw overshoots / snap back
    const isUnwinding = Math.sign(target) !== Math.sign(finite(previous)) || Math.abs(target) < Math.abs(finite(previous));
    const rate = isUnwinding ? 18.0 : (isEmergencyTurnaround ? 16.0 : (committed ? 10.0 : 8.0));
    const maxDelta = rate * clamp(finite(dt, 0.016), 0, 0.1);

    return clamp(
      finite(previous) + clamp(target - finite(previous), -maxDelta, maxDelta),
      -1,
      1
    );
  }

  /**
   * 2D G-G Friction-Circle Trail Braking & Longitudinal Control:
   * Solves progressive braking, corner entry trail braking, and traction control.
   * @param {Object} params
   * @returns {Object} { throttle, brake, friction, trailBraking, instability, tcsActive }
   */
  computePedals({
    vehicle,
    speedError = 0,
    desiredSpeed = 50.0,
    lateralAccel = 0,
    steerAngle = 0,
    headingError = 0,
    slipAngle = 0,
    yawRate = 0,
    currentCurvature = 0,
    straight = false,
    recovering = false,
    emergency = false,
    tireGripFactor = 1.0,
    dirtyAirLoss = 0,
    dt = 0.016
  }) {
    const friction = this.evaluateFrictionCircle({
      vehicle,
      lateralAccel,
      tireGripFactor
    });

    let throttle = 0;
    let brake = 0;

    // 1. Off-Track Rejoin & Recovery Mode
    if (recovering) {
      const vSpeed = finite(vehicle?.speed, 0);
      const rawSlip = finite(slipAngle, 0);
      const rawYawRate = finite(vehicle?.yawRate, 0);
      const isSevereSpin = Math.abs(rawSlip) > 0.40 || Math.abs(rawYawRate) > 0.80 || Math.abs(finite(headingError, 0)) > 2.0;

      if (isSevereSpin || (Math.abs(finite(headingError, 0)) > 0.85 && vSpeed > 4.5)) {
        // Cut throttle completely and apply controlled brake to stop donut spins
        throttle = 0;
        brake = clamp(0.35 + vSpeed * 0.04, 0.25, 0.70);
      } else {
        // Smoothly drive back onto track
        throttle = vSpeed < desiredSpeed ? clamp(0.25 + (desiredSpeed - vSpeed) * 0.04, 0.20, 0.40) : 0;
        brake = vSpeed > desiredSpeed + 2.0 ? 0.30 : 0;
      }
      return { throttle, brake, friction, trailBraking: false, instability: 0, tcsActive: false };
    }

    if (emergency) {
      return { throttle: 0, brake: 1.0, friction, trailBraking: false, instability: 0, tcsActive: false };
    }

    // 2. Dynamic Speed Demand, Smooth Coasting & Progressive Threshold Braking
    const vSpeed = finite(vehicle?.speed, 0);
    const steerMagnitude = saturate(Math.abs(finite(steerAngle, 0)));
    const latUtil = friction.latUtilization;
    const isCornering = !straight && (steerMagnitude > 0.18 || latUtil > 0.48);

    // Smooth momentum-preserving thresholds: prevent brake stabbing and speed stalls
    const coastThreshold = isCornering ? -3.20 : -2.20;
    const fullBrakeThreshold = isCornering ? -7.50 : -6.00;

    if (speedError >= 0) {
      // Need acceleration: ramp throttle smoothly to full power
      const exitBonus = (!straight && steerMagnitude < 0.28) ? 0.15 : 0;
      throttle = clamp((straight ? 1.0 : (0.85 + exitBonus)) + finite(speedError) * 0.35, 0.45, 1.0);
      brake = 0;
    } else if (speedError > coastThreshold) {
      // Momentum Carry & Smooth Coasting Zone: ZERO BRAKES
      // Allows the vehicle to roll naturally through turn entry and apex carrying momentum
      const blend = (speedError - coastThreshold) / Math.max(0.01, -coastThreshold);
      throttle = clamp(blend * (isCornering ? 0.55 : 0.70), 0, 0.70);
      brake = 0;
    } else {
      // Genuine Braking Demand: smoothly progressive up to threshold braking
      throttle = 0;
      const brakeOver = (-speedError - Math.abs(coastThreshold)) / Math.max(0.01, Math.abs(fullBrakeThreshold) - Math.abs(coastThreshold));
      if (speedError <= fullBrakeThreshold) {
        // High-G threshold braking on approach to heavy braking zones
        brake = clamp(0.85 + (-speedError - Math.abs(fullBrakeThreshold)) * 0.15, 0.85, 1.0);
      } else {
        // Smooth progressive entry braking
        brake = clamp(brakeOver * 0.85, 0.05, 0.85);
      }
    }

    // Rate-limit brake changes: smooth hydraulic bite (18/s) and release (18/s)
    const prevBrake = finite(vehicle?.controls?.brake, 0);
    const maxBrakeRate = 18.0;
    const maxBrakeDelta = maxBrakeRate * clamp(finite(dt, 0.016), 0, 0.1);
    brake = clamp(prevBrake + clamp(brake - prevBrake, -maxBrakeDelta, maxBrakeDelta), 0, 1);

    // 3. Friction-Circle-Coupled High-Precision Trail Braking Modulation
    // Seamlessly tapers longitudinal braking along the 2D G-G friction ellipse
    let trailBrakingActive = false;
    if (brake > 0.03 && friction.latUtilization > 0.08) {
      trailBrakingActive = true;
      const latFactor = clamp(this.trailBrakingSkill * friction.latUtilization * 0.90, 0, 0.98);
      const remainingLongitudinal = Math.sqrt(Math.max(0.04, 1.0 - Math.pow(latFactor, 2)));
      brake = Math.min(brake, remainingLongitudinal);
    }

    // 4. Oversteer / Lateral Instability Control (Phase-Aware Yaw & Real Breakaway Slip)
    const rawSlip = finite(slipAngle, 0);
    const kinematicSlip = finite(steerAngle, 0) * 0.42 * saturate((24.0 - vSpeed) / 20.0);
    const dynamicExcessSlip = Math.abs(rawSlip - kinematicSlip);

    const kinYawRate = vSpeed * finite(currentCurvature, 0);
    let excessYaw = 0;
    if (Math.sign(yawRate) === Math.sign(kinYawRate) || Math.abs(kinYawRate) < 0.15) {
      excessYaw = Math.max(0, Math.abs(finite(yawRate, 0)) - Math.abs(kinYawRate) - 0.45);
    } else if (dynamicExcessSlip > 0.12) {
      excessYaw = Math.abs(finite(yawRate, 0));
    }
    
    // Dynamic instability triggers on genuine tire breakaway slides (> 9 deg / 0.16 rad slip)
    const speedWeight = saturate(vSpeed / 8.0);
    const instability = saturate(Math.max(
      (dynamicExcessSlip - 0.160) / 0.10,
      (excessYaw - 1.20) / 1.00
    )) * speedWeight;

    // 5. Traction Control System (TCS) & Aggressive Corner-Exit Power Launch
    let tcsActive = false;
    const isHardCornering = !straight && (steerMagnitude > 0.32 && latUtil > 0.82);

    if (throttle > 0.03 && isHardCornering && brake < 0.05) {
      tcsActive = true;
      const tractionBudget = Math.sqrt(Math.max(0.40, 1.0 - Math.pow(latUtil * 0.78, 2)));
      const lowSpeedBoost = clamp((22.0 - vSpeed) / 10.0, 0, 0.55);
      const unwindGain = clamp(
        tractionBudget * (1.0 - this.unwindFactor * Math.pow(steerMagnitude, 1.1) * 0.18) + lowSpeedBoost,
        0.75,
        1.0
      );
      throttle = clamp(throttle * unwindGain, 0.70, 1.0);
    }

    // Mid-corner apex drive: maintain strong positive throttle floor (60%) for downforce and rear load
    if (throttle > 0.05 && isCornering && vSpeed > 10.0 && brake < 0.05 && instability < 0.25) {
      throttle = Math.max(throttle, 0.60);
    }

    // Aggressive early 100% full throttle launch as soon as steering begins unwinding or on straights
    if (speedError > -0.40 && steerMagnitude < 0.32 && brake < 0.05 && Math.abs(finite(slipAngle, 0)) < 0.12) {
      throttle = 1.0;
    }

    if (instability > 0) {
      tcsActive = true;
      if (brake > 0.45 || speedError < -5.0) {
        throttle = 0;
      } else {
        // High-speed maintenance throttle floor keeps rear axle planted
        throttle = clamp(throttle * (1.0 - instability * 0.45), 0.35, 0.85);
      }
      brake *= Math.max(0.1, 1.0 - instability * 0.45);
    }

    // Explosive power launch out of chicanes and hairpins
    if (speedError > -0.20 && vSpeed < 28.0 && brake < 0.05 && instability === 0) {
      throttle = clamp(0.85 + finite(speedError) * 0.10, 0.85, 1.0);
    }

    return {
      throttle: clamp(throttle, 0, 1),
      brake: clamp(brake, 0, 1),
      friction,
      trailBraking: trailBrakingActive,
      instability,
      tcsActive
    };
  }
}
