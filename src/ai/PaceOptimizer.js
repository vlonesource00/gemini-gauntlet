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
    // Calibrated lateral acceleration budget: Prototype class up to 26.5 m/s² (2.70G) reflecting true ground-effect downforce
    const classBaseG = vehicleClass === 'prototype' ? 2.70 : vehicleClass === 'gt' ? 1.30 : 1.10;
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
    lookaheadDistances = [0, 6, 12, 18, 26, 36, 48, 62, 80, 102, 128, 160],
    tireGripFactor = 1.0,
    skill = 0.85,
    aggression = 0.80,
    defending = false,
    threatScore = 0,
    closingSpeed = 0,
    insideLineOffset = 0
  } = {}) {
    const vClass = vehicle?.classKey || 'prototype';
    // Deceleration budget for speed envelope backward reachability (-3.5G threshold capability, 1.85G sustained)
    const classBrakeG = vClass === 'prototype' ? 1.85 : vClass === 'gt' ? 1.25 : 1.0;
    const sustainedDecel = classBrakeG * 9.81 * tireGripFactor * (0.88 + aggression * 0.12);

    // Deep Defensive Braking Point offset (shifts braking threshold deeper into turn-in under threat)
    let deepBrakeOffsetM = 0;
    if (defending && threatScore >= 0.30) {
      const v0 = Math.max(8.0, finite(vehicle?.speed, 0));
      const maxA = sustainedDecel;
      const psi = 0.06 + 0.14 * aggression;
      const omega = saturate(threatScore);
      const gamma = clamp(1.0 + closingSpeed / 8.0, 0.8, 1.4);
      const maxCap = Math.min(16.0, 0.22 * (v0 * v0 / (2.0 * maxA)));
      deepBrakeOffsetM = Math.min(maxCap, (v0 * v0 / (2.0 * maxA)) * psi * omega * gamma);
    }

    let speedLimit = 95.0; // max track velocity ceiling

    for (const dist of lookaheadDistances) {
      const effectiveDist = Math.max(0, dist + deepBrakeOffsetM);
      const sampleDist = finite(vehicle?.distance, 0) + effectiveDist;
      const point = track?.atDistance ? track.atDistance(sampleDist) : { curvature: 0, banking: 0 };
      
      // Account for compressed corner radius if running on the defensive inside lane
      let curvature = Math.abs(finite(point.curvature, 0));
      if (Math.abs(insideLineOffset) > 0.5 && curvature > 1e-4) {
        curvature = curvature / Math.max(0.40, 1.0 - Math.abs(insideLineOffset) * curvature);
      }

      // Boost corner target speed evaluation for Prototype class with 2.70G lateral budget and banking carry
      const physLimit = this.calculateCornerSpeed({
        curvature,
        banking: point.banking,
        vehicleClass: vClass,
        tireGripFactor,
        skill
      });

      const trackLimit = track?.targetSpeed
        ? (vClass === 'prototype' ? Math.max(physLimit, track.targetSpeed(sampleDist, skill) * 1.30) : track.targetSpeed(sampleDist, skill))
        : physLimit;

      const cornerSpeed = Math.max(5.5, trackLimit);
      const reachableSpeed = Math.sqrt(cornerSpeed * cornerSpeed + 2.0 * sustainedDecel * effectiveDist);
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
   * Compute steering command with pure pursuit, lateral offset trim, and yaw damping.
   * @param {Object} params
   * @returns {number} Steering command [-1, 1]
   */
  computeSteering({
    previous = 0,
    headingError = 0,
    lateralError = 0,
    yawRate = 0,
    dt = 0.016,
    committed = false,
    recovering = false,
    yielding = false
  } = {}) {
    const headingGain = recovering ? 2.85 : committed ? 3.5 : 2.3;
    const lateralGain = recovering ? 0.085 : committed ? 0.08 : 0.055;
    const yawDamping = committed ? 0.12 : 0.17;

    let target = clamp(
      finite(headingError) * headingGain - finite(lateralError) * lateralGain - finite(yawRate) * yawDamping,
      -1,
      1
    );

    if (yielding) target = clamp(target, -0.3, 0.3);

    const rate = committed ? 7.8 : recovering ? 6.2 : 5.4;
    const maxDelta = rate * clamp(finite(dt, 0.016), 0, 0.1);

    return clamp(
      finite(previous) + clamp(target - finite(previous), -maxDelta, maxDelta),
      -1,
      1
    );
  }

  /**
   * Calculate throttle and brake pedals using trail braking and throttle unwind modulation.
   * Features:
   * - Threshold braking: when speedError < -1.2 m/s, rapidly ramps brake to 1.0 (-3.5G threshold capability)
   * - Full throttle commitment: aggressive exit drive without artificial lag
   * - Reduced conservative oversteer throttle cuts during high-G cornering
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
      throttle = vSpeed < desiredSpeed ? (Math.abs(headingError) > 1.15 ? 0.35 : 0.68) : 0;
      brake = vSpeed > desiredSpeed + 2.5
        ? clamp(0.25 + (vSpeed - desiredSpeed) * 0.035, 0.25, 0.78)
        : 0;
      return { throttle, brake, friction, trailBraking: false };
    }

    if (emergency) {
      return { throttle: 0, brake: 1.0, friction, trailBraking: false };
    }

    // 1. Speed error response with rapid threshold braking & instant throttle pickup
    if (speedError > -0.3) {
      throttle = clamp((straight ? 1.0 : 0.62) + finite(speedError) * 0.30, 0, 1);
    }

    if (speedError < -0.35) {
      if (speedError < -1.2) {
        // High-G threshold braking (-3.5G deceleration capacity)
        brake = clamp(0.85 + (-finite(speedError) - 1.2) * 1.5, 0.85, 1.0);
      } else {
        brake = clamp((-finite(speedError) - 0.25) * 0.80, 0, 0.85);
      }
    }

    // 2. High-Precision Trail Braking Modulation
    let trailBrakingActive = false;
    if (brake > 0.04 && friction.latUtilization > 0.12 && speedError > -4.5) {
      trailBrakingActive = true;
      const trailExp = defending ? 1.4 : 1.8;
      const latFactor = clamp(this.trailBrakingSkill * friction.latUtilization, 0, 0.98);
      const trailFactor = Math.pow(Math.max(0.01, 1.0 - Math.pow(latFactor, 2)), 1.0 / trailExp);
      brake *= clamp(trailFactor, 0.30, 1.0);
    }

    // 3. Corner-Exit Full Throttle Commitment & Traction Ellipse Controller
    if (throttle > 0.04 && !straight) {
      const steerMagnitude = saturate(Math.abs(finite(steerAngle, 0)));
      const latUtil = friction.latUtilization;
      // High-downforce traction budget: ground effect expands traction envelope at speed
      const tractionBudget = Math.sqrt(Math.max(0.16, 1.0 - Math.pow(latUtil * 0.88, 2)));
      // As steer unwinds, ramp throttle aggressively towards 1.0 without artificial lag
      const unwindGain = clamp(
        tractionBudget * (1.0 - this.unwindFactor * Math.pow(steerMagnitude, 1.8) * 0.40) + (1.0 - steerMagnitude) * 0.35,
        0.35,
        1.0
      );
      throttle = clamp(throttle * unwindGain, 0, 1);
      // Immediate full throttle commitment when steering is mostly unwound on corner exit
      if (speedError > 0 && steerMagnitude < 0.25 && latUtil < 0.85) {
        throttle = Math.max(throttle, 1.0);
      }
    }

    // 4. Oversteer / Lateral Instability Control (Tuned for high-G prototype dynamics without premature throttle cuts)
    const slip = Math.abs(finite(slipAngle, 0));
    const yaw = Math.abs(finite(yawRate, 0));
    // High-G prototype threshold: allow yaw rates up to 1.50 rad/s and slip up to 0.20 rad before intervention
    const instability = saturate(Math.max((slip - 0.20) / 0.16, (yaw - 1.50) / 1.10));

    if (instability > 0) {
      throttle *= (1.0 - instability * 0.50);
      if (speedError > -3.0) {
        brake *= (1.0 - instability * 0.25);
      }
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
