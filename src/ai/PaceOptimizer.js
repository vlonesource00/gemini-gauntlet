/**
 * PaceOptimizer.js
 * Physics-based optimal pace and vehicle control engine:
 * - Friction circle (G-G diagram) tire load limit modeling
 * - Trail braking modulation during corner entry
 * - Backward reachable speed envelope integration over track curvature & banking
 * - Smooth corner-exit throttle unwind controller to prevent snap oversteer
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
   * @param {number} [options.unwindFactor=0.75] - Exit throttle modulation intensity (0-1)
   */
  constructor({
    trailBrakingSkill = 0.85,
    unwindFactor = 0.75
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
   * @param {Object} params
   * @returns {number} Corner apex speed limit in m/s
   */
  calculateCornerSpeed({
    curvature = 0,
    banking = 0,
    vehicleClass = 'gt',
    tireGripFactor = 1.0,
    skill = 0.85
  } = {}) {
    const kappa = Math.max(1e-5, Math.abs(finite(curvature, 0)));
    const classBaseG = vehicleClass === 'prototype' ? 2.1 : vehicleClass === 'gt' ? 1.38 : 1.15;
    const peakG = classBaseG * tireGripFactor * (0.85 + skill * 0.12);
    const g = 9.81;

    // Banking bonus: a_lat_eff = g * (peakG * cos(theta) + sin(theta))
    const bankAngle = finite(banking, 0);
    const effectiveLatAccel = g * (peakG * Math.cos(bankAngle) + Math.sin(bankAngle));

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
    aggression = 0.7,
    defending = false,
    threatScore = 0,
    closingSpeed = 0,
    insideLineOffset = 0
  } = {}) {
    const vClass = vehicle?.classKey || 'gt';
    const classBrakeG = vClass === 'prototype' ? 1.40 : vClass === 'gt' ? 1.15 : 0.95;
    const sustainedDecel = classBrakeG * 9.81 * tireGripFactor * (0.88 + aggression * 0.10);

    // Deep Defensive Braking Point offset (shifts braking threshold deeper into turn-in under threat)
    let deepBrakeOffsetM = 0;
    if (defending && threatScore >= 0.35) {
      const v0 = Math.max(8.0, finite(vehicle?.speed, 0));
      const maxA = sustainedDecel;
      const psi = 0.05 + 0.12 * aggression;
      const omega = saturate(threatScore);
      const gamma = clamp(1.0 + closingSpeed / 8.0, 0.8, 1.4);
      const maxCap = Math.min(14.0, 0.20 * (v0 * v0 / (2.0 * maxA)));
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

      const trackLimit = track?.targetSpeed
        ? track.targetSpeed(sampleDist, skill)
        : this.calculateCornerSpeed({
            curvature,
            banking: point.banking,
            vehicleClass: vClass,
            tireGripFactor,
            skill
          });

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
    const vClass = vehicle?.classKey || 'gt';
    const classBaseG = vClass === 'prototype' ? 2.4 : vClass === 'gt' ? 1.75 : 1.4;
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

    // 1. Raw speed error response
    if (speedError > -0.6) {
      throttle = clamp((straight ? 0.96 : 0.54) + finite(speedError) * 0.15, 0, 1);
    }

    if (speedError < -0.4) {
      brake = clamp((-finite(speedError) - 0.25) * 0.30, 0, 1);
    }

    // 2. High-Precision Trail Braking Modulation
    let trailBrakingActive = false;
    if (brake > 0.04 && friction.latUtilization > 0.12) {
      trailBrakingActive = true;
      const trailExp = defending ? 1.4 : 1.8;
      const latFactor = clamp(this.trailBrakingSkill * friction.latUtilization, 0, 0.98);
      const trailFactor = Math.pow(Math.max(0.01, 1.0 - Math.pow(latFactor, 2)), 1.0 / trailExp);
      brake *= clamp(trailFactor, 0.10, 1.0);
    }

    // 3. Defensive Exit Throttle Unwind & Traction Ellipse Controller
    if (throttle > 0.04 && !straight) {
      const steerMagnitude = saturate(Math.abs(finite(steerAngle, 0)));
      const latUtil = friction.latUtilization;
      const tractionBudget = Math.sqrt(Math.max(0.04, 1.0 - Math.pow(latUtil, 2)));
      const unwindGain = clamp(
        tractionBudget * (1.0 - this.unwindFactor * steerMagnitude * 0.65),
        0.18,
        1.0
      );
      throttle *= unwindGain;
    }

    // 4. Oversteer / Lateral Instability Control
    const slip = Math.abs(finite(slipAngle, 0));
    const yaw = Math.abs(finite(yawRate, 0));
    const instability = saturate(Math.max((slip - 0.12) / 0.18, (yaw - 1.05) / 1.15));

    if (instability > 0) {
      throttle *= (1.0 - instability * 0.80);
      brake *= (1.0 - instability * 0.70);
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
