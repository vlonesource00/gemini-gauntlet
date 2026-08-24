/**
 * PaceOptimizer.js
 * Physics-based optimal pace and vehicle control engine:
 * - Friction circle (G-G diagram) tire load limit modeling (Prototype up to 26.5 m/s² / 2.70G lateral)
 * - Trail braking modulation during corner entry
 * - Backward reachable speed envelope integration over track curvature & banking
 * - Smooth corner-exit full throttle commitment to maximize acceleration without artificial lag
 * - Deterministic pure pursuit & Stanley steering stabilization
 */

const finite = (value, fallback = 0) => (Number.isFinite(value) ? value : fallback);

const clamp = (value, min, max) => Math.max(min, Math.min(max, value));

const saturate = (value) => clamp(value, 0, 1);

const wrapAngle = (angle) => {
  let result = (angle + Math.PI) % (Math.PI * 2);
  if (result < 0) result += Math.PI * 2;
  return result - Math.PI;
};

export class PaceOptimizer {
  /**
   * @param {Object} options
   * @param {number} [options.trailBrakingSkill=0.85] - Driver trail braking proficiency (0-1)
   * @param {number} [options.unwindFactor=0.60] - Exit throttle modulation intensity (0-1)
   */
  constructor({
    trailBrakingSkill = 0.85,
    unwindFactor = 0.60
  } = {}) {
    this.trailBrakingSkill = clamp(trailBrakingSkill, 0, 1);
    this.unwindFactor = clamp(unwindFactor, 0, 1);
  }

  setParameters({ trailBrakingSkill, unwindFactor } = {}) {
    if (Number.isFinite(trailBrakingSkill)) this.trailBrakingSkill = clamp(trailBrakingSkill, 0, 1);
    if (Number.isFinite(unwindFactor)) this.unwindFactor = clamp(unwindFactor, 0, 1);
  }

  /**
   * Compute maximum physical cornering speed based on curvature, banking, and tire grip.
   * Calibrated for Prototype class up to 26.5 m/s² (2.70G) to reflect true ground-effect downforce at speed.
   * @param {Object} params
   * @returns {number} Corner apex speed limit in m/s
   */
  calculateCornerSpeed({
    curvature = 0,
    banking = 0,
    vehicleClass = 'prototype',
    tireGripFactor = 1.0,
    skill = 0.85
  } = {}) {
    const kappa = Math.max(1e-5, Math.abs(finite(curvature, 0)));
    // Speed-dependent aerodynamic downforce: mechanical grip base (1.85G) + downforce scaling up to 2.70G at speed
    const vEst = Math.sqrt(9.81 * 1.85 / kappa);
    const downforceFactor = vehicleClass === 'prototype' ? saturate((vEst - 18.0) / 35.0) : 0;
    const classBaseG = vehicleClass === 'prototype' ? (1.80 + 0.90 * downforceFactor) : vehicleClass === 'gt' ? 1.30 : 1.10;
    const peakG = classBaseG * tireGripFactor * (0.86 + skill * 0.14);
    const g = 9.81;

    // Banking bonus: a_lat_eff = g * (peakG * cos(theta) + sin(theta))
    // High-speed banking carry: amplify banking support on high-speed sweeps (e.g. Oakland Bowl and high-speed esses)
    const bankAngle = Math.abs(finite(banking, 0));
    const bankCarry = Math.sin(bankAngle) * (vehicleClass === 'prototype' ? 1.35 : 1.05);
    const effectiveLatAccel = g * (peakG * Math.cos(bankAngle) + bankCarry);

    return Math.sqrt(effectiveLatAccel / kappa);
  }

  /**
   * Compute backward-reachable speed envelope across upcoming lookahead horizons.
   * Ensures the vehicle starts braking early enough for every upcoming corner.
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
    insideLineOffset = 0
  } = {}) {
    const vClass = vehicle?.classKey || 'prototype';
    // Calibrated physical sustained deceleration budget ensuring optimal braking point arrival
    // Prototype: ~8.8 m/s²; GT: ~7.0 m/s²; Touring: ~5.2 m/s²
    const brakingDecel = (vClass === 'prototype' ? 8.8 : vClass === 'gt' ? 7.0 : 5.2) * tireGripFactor * (0.92 + (aggression - 0.5) * 0.10);
    const speedEnvelopeDistances = [0, 4, 8, 12, 16, 22, 28, 36, 46, 58, 72, 88, 108, 132, 160, 200, 250, 310];

    let speedLimit = 95.0; // max track velocity ceiling

    for (const dist of speedEnvelopeDistances) {
      const sampleDist = finite(vehicle?.distance, 0) + dist;
      const point = track?.atDistance ? track.atDistance(sampleDist) : { curvature: 0, banking: 0 };
      
      let rawCurvature = Math.abs(finite(point.curvature, 0));
      // Effective racing line curvature: exploiting track width and apex clipping flattens corner radius (e.g. Turns 1-6)
      const roadWidth = finite(track?.roadHalfWidth, 6.5) + finite(track?.curbWidth, 1.05);
      let curvature = rawCurvature / (1.0 + 2.65 * roadWidth * rawCurvature);

      if (Math.abs(insideLineOffset) > 0.5 && curvature > 1e-4) {
        curvature = curvature / Math.max(0.40, 1.0 - Math.abs(insideLineOffset) * curvature);
      }

      const safetyFactor = defending ? 0.94 : (aggression > 0.8 ? 1.00 : 0.97);
      const physLimit = this.calculateCornerSpeed({
        curvature,
        banking: point.banking,
        vehicleClass: vClass,
        tireGripFactor,
        skill
      }) * safetyFactor;

      const trackLimit = track?.targetSpeed
        ? (vClass === 'prototype' ? Math.max(physLimit, track.targetSpeed(sampleDist, skill) * 1.15) : track.targetSpeed(sampleDist, skill))
        : physLimit;

      const cornerSpeed = Math.max(5.5, Math.min(physLimit, trackLimit));
      const reachableSpeed = Math.sqrt(cornerSpeed * cornerSpeed + 2.0 * brakingDecel * dist);
      speedLimit = Math.min(speedLimit, reachableSpeed);
    }

    return speedLimit;
  }

  /**
   * Evaluate G-G friction circle tire utilization and available deceleration.
   * Friction circle: (a_long / a_long_max)^2 + (a_lat / a_lat_max)^2 <= 1.0
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

    // Available longitudinal deceleration inside friction circle
    const availableLongDecel = maxTotalAccel * Math.sqrt(Math.max(0, 1.0 - latUtilization * latUtilization));
    const longUtilization = 0; // will be updated based on brake/throttle

    return {
      maxTotalAccel,
      actualLatAccel,
      latUtilization,
      availableLongDecel,
      totalUtilization: latUtilization
    };
  }

  /**
   * Compute steering command with pure pursuit, lateral offset trim, yaw damping, and active counter-steering.
   * @param {Object} params
   * @returns {number} Steering command [-1, 1]
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
    yielding = false
  } = {}) {
    const headingGain = recovering ? 2.80 : committed ? 3.45 : 2.25;
    const lateralGain = recovering ? 0.085 : committed ? 0.080 : 0.055;
    const yawDamping = committed ? 0.12 : 0.17;

    const vSpeed = finite(speed, 0);
    // Speed-dependent steering limit prevents destructive high-speed front tire saturation scrub
    const maxUsableSteer = recovering ? 1.0 : clamp(15.0 / Math.max(8.0, vSpeed), 0.30, 1.0);

    // Direct pure-pursuit trajectory tracking with lateral error trim and yaw rate damping
    let target = clamp(
      finite(headingError) * headingGain - finite(lateralError) * lateralGain - finite(yawRate) * yawDamping,
      -maxUsableSteer,
      maxUsableSteer
    );

    if (yielding) target = clamp(target, -0.3, 0.3);

    const rate = committed ? 7.5 : recovering ? 6.0 : 5.2;
    const maxDelta = rate * clamp(finite(dt, 0.016), 0, 0.1);

    return clamp(
      finite(previous) + clamp(target - finite(previous), -maxDelta, maxDelta),
      -1,
      1
    );
  }

  /**
   * Calculate throttle and brake pedals using friction-circle trail braking, progressive TCS, and anti-spin modulation.
   * @param {Object} params
   * @returns {Object} Pedals and G-G telemetry
   */
  computePedals({
    vehicle,
    speedError = 0,
    desiredSpeed = 0,
    headingError = 0,
    lateralAccel = 0,
    steerAngle = 0,
    yawRate = 0,
    slipAngle = 0,
    currentCurvature = 0,
    straight = false,
    recovering = false,
    emergency = false,
    defending = false,
    tireGripFactor = 1.0
  }) {
    const friction = this.evaluateFrictionCircle({
      vehicle,
      lateralAccel,
      tireGripFactor
    });

    let throttle = 0;
    let brake = 0;

    if (recovering) {
      const vSpeed = finite(vehicle?.speed, 0);
      const isReversed = Math.abs(finite(headingError, 0)) > 2.0;
      // On recovery / off-track grass, limit throttle to prevent wheelspin donut loops
      throttle = vSpeed < desiredSpeed
        ? (isReversed ? 0.20 : Math.abs(headingError) > 0.9 ? 0.28 : 0.48)
        : 0;
      brake = vSpeed > desiredSpeed + 2.0
        ? clamp(0.20 + (vSpeed - desiredSpeed) * 0.04, 0.20, 0.65)
        : 0;
      return { throttle, brake, friction, trailBraking: false, instability: 0 };
    }

    if (emergency) {
      return { throttle: 0, brake: 1.0, friction, trailBraking: false, instability: 0 };
    }

    // 1. Dynamic Speed Demand & In-Corner Deceleration vs Straight Threshold Braking
    const steerMagnitude = saturate(Math.abs(finite(steerAngle, 0)));
    const latUtil = friction.latUtilization;
    const isCornering = !straight && (steerMagnitude > 0.18 || latUtil > 0.58);
    // In mid-corner, avoid sudden brake stabbing on minor speed errors; lift throttle and coast instead
    const brakeThreshold = isCornering ? -1.80 : -0.70;

    if (speedError > brakeThreshold + 0.30) {
      throttle = clamp((straight ? 1.0 : 0.85) + finite(speedError) * 0.25, 0, 1.0);
      brake = 0;
    } else {
      throttle = 0;
    }

    if (speedError < brakeThreshold) {
      throttle = 0;
      if (speedError < -2.5) {
        // High-G threshold braking on corner approach (-3.5G capacity)
        brake = clamp(0.80 + (-finite(speedError) - 2.5) * 0.30, 0.80, 1.0);
      } else {
        // Progressive corner entry brake modulation
        brake = clamp((-finite(speedError) - Math.abs(brakeThreshold)) * 0.45, 0, 0.75);
      }
    }

    // 2. Friction-Circle-Coupled High-Precision Trail Braking Modulation
    // Seamlessly tapers longitudinal braking force as lateral cornering load rises
    let trailBrakingActive = false;
    if (brake > 0.03 && friction.latUtilization > 0.08) {
      trailBrakingActive = true;
      const trailExp = defending ? 1.3 : 1.6;
      const latFactor = clamp(this.trailBrakingSkill * friction.latUtilization * 0.96, 0, 0.99);
      const trailFactor = Math.pow(Math.max(0.01, 1.0 - Math.pow(latFactor, 2)), 1.0 / trailExp);
      brake *= clamp(trailFactor, 0.05, 1.0);
    }

    // 3. Oversteer / Lateral Instability Control (Phase-Aware Yaw & Real Breakaway Slip)
    const rawSlip = finite(slipAngle, 0);
    const vSpeed = finite(vehicle?.speed, 0);
    // Kinematic geometric body slip from steering lock at low-to-medium speeds
    const kinematicSlip = finite(steerAngle, 0) * 0.42 * saturate((24.0 - vSpeed) / 20.0);
    const dynamicExcessSlip = Math.abs(rawSlip - kinematicSlip);

    const kinYawRate = vSpeed * finite(currentCurvature, 0);
    // Phase-aware yaw rate excess: in high-speed direction changes (esses/chicanes),
    // yaw rate lag behind curvature reversal is normal dynamic response, not a spin.
    let excessYaw = 0;
    if (Math.sign(yawRate) === Math.sign(kinYawRate) || Math.abs(kinYawRate) < 0.15) {
      excessYaw = Math.max(0, Math.abs(finite(yawRate, 0)) - Math.abs(kinYawRate) - 0.45);
    } else if (dynamicExcessSlip > 0.12) {
      // If slipping significantly while yawing against curvature, evaluate counter-spin
      excessYaw = Math.abs(finite(yawRate, 0));
    }
    
    // Dynamic instability triggers ONLY on genuine tire breakaway slides (> 8 deg / 0.13 rad slip)
    const speedWeight = saturate(vSpeed / 8.0);
    const instability = saturate(Math.max(
      (dynamicExcessSlip - 0.130) / 0.09,
      (excessYaw - 0.85) / 0.90
    )) * speedWeight;

    // 4. Traction Control (TCS) & Corner-Exit Throttle Modulation
    // Intervenes ONLY in tight high-load cornering (|steer| > 0.25 AND latUtil > 0.72)
    const isHardCornering = !straight && (steerMagnitude > 0.25 && latUtil > 0.72);

    if (throttle > 0.03 && isHardCornering && brake < 0.05) {
      const tractionBudget = Math.sqrt(Math.max(0.20, 1.0 - Math.pow(latUtil * 0.85, 2)));
      const lowSpeedBoost = clamp((20.0 - vSpeed) / 12.0, 0, 0.45);
      const unwindGain = clamp(
        tractionBudget * (1.0 - this.unwindFactor * Math.pow(steerMagnitude, 1.2) * 0.25) + lowSpeedBoost,
        0.60,
        1.0
      );
      throttle = clamp(throttle * unwindGain, 0, 1);
    }

    // High-speed direction change & apex drive: maintain positive rear axle load to prevent lift-off snap oversteer
    if (throttle > 0.05 && isCornering && vSpeed > 16.0 && brake < 0.05 && instability < 0.15) {
      throttle = Math.max(throttle, 0.42);
    }

    // Instant 100% full throttle pickup as soon as steering unwinds or on straights
    if (speedError > 0.2 && steerMagnitude < 0.18 && brake < 0.05 && Math.abs(finite(slipAngle, 0)) < 0.06) {
      throttle = 1.0;
    }

    if (instability > 0) {
      if (brake > 0.03 || speedError < -0.30) {
        throttle = 0;
      } else {
        // High-speed maintenance throttle floor (25%) prevents lethal lift-off snap spins in esses
        const minFloor = (vSpeed > 18.0 && !straight && dynamicExcessSlip < 0.18) ? 0.25 : 0.0;
        throttle = Math.max(minFloor, throttle * (1.0 - instability * 0.70));
      }
      // Soften brake during oversteer slides to prevent rear lockup
      brake *= Math.max(0.1, 1.0 - instability * 0.55);
    }

    // Prevent low-speed steering scrub stall in tight chicanes and hairpins
    if (speedError > 0.8 && vSpeed < 22.0 && brake < 0.05 && throttle < 0.55 && instability === 0) {
      throttle = clamp(0.55 + finite(speedError) * 0.05, 0.55, 0.90);
    }

    return {
      throttle: clamp(throttle, 0, 1),
      brake: clamp(brake, 0, 1),
      friction,
      trailBraking: trailBrakingActive,
      instability
    };
  }
}
