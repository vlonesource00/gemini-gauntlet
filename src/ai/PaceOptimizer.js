/**
 * PaceOptimizer.js
 * Modular Edge Vehicle Dynamics, Car Control & Pace Optimization Engine.
 * 
 * Features:
 * - 2D G-G Friction Circle Tire Load Modeling (Pacejka friction ellipse)
 * - Exact Forward Stopping Envelope Min-Reduction Braking:
 *     v_allow = min_{d in [0, look]} sqrt(v_t^2 + 2 * a_B * d)
 *     Enables 100% full throttle right up to the exact threshold braking point.
 * - 400Hz/120Hz Physics-Informed Front-Axle Slip Saturation Guard (Anti-Plow / Anti-Scrub):
 *     Monitors front axle slip angle alphaF = 0.5 * (alpha_FL + alpha_FR).
 *     When |alphaF| > alphaPeak, actively backs off steering lock command to maintain peak grip.
 * - Rear-Axle Slip Saturation Guard (Catch & Yaw Damper):
 *     Monitors rear axle slip angle alphaR = 0.5 * (alpha_RL + alpha_RR).
 *     When |alphaR| > alphaPeak, fades path tracking out (hold = 1 - giveUp) and fades yaw damping/countersteer in:
 *     kYaw = K_YAW * (1 + 3.0 * giveUp), kBeta = K_BETA * (1 + 2.0 * giveUp).
 * - Integral Yaw-Rate Understeer Gradient Learner:
 *     eYaw = r_des - r. Integrates yawInt += eYaw * K_YAW_I * dt with anti-windup clamping to learn
 *     the understeer gradient dynamically across prototype, GT, and touring classes.
 * - Extremum-Seeking Pace Trim Observer:
 *     Clean driving without excess slip or off-track advances paceTrim up to 1.0.
 * - Seamless Trail-Braking Modulation along 2D G-G ellipse.
 * - Traction Control System (TCS) & Instant 100% Full-Throttle Power Launch on unwinding.
 */

import { clamp, wrapAngle, saturate } from '../core/math.js';

const G = 9.80665;
const finite = (value, fallback = 0) => (Number.isFinite(value) ? value : fallback);

// Driver / dynamics tuning constants shared across car classes
const K_YAW = 0.145;       // Yaw rate tracking gain
const K_YAW_I = 0.62;      // Integral understeer gradient learning rate
const K_YAW_I_MAX = 0.16;  // Rad, hard anti-windup cap on integral trim
const K_BETA = 0.62;       // Countersteer gain per rad of excess body sideslip

const GIVEUP_YAW = 3.0;    // Yaw-damper gain multiplier at full rear saturation
const GIVEUP_BETA = 2.0;   // Countersteer gain multiplier at full rear saturation
const SAT_RATIO = 1.12;    // Front slip angle / alphaPeak threshold for anti-plow guard
const BETA_SLACK = 1.15;   // Body slip excess slack factor
const BETA_CAP = 0.16;     // Rad, maximum allowed body sideslip reference
const BRAKE_MARGIN = 0.92; // Conservative margin for exact min-reduction scan

const PACE_MIN = 0.80;
const PACE_MAX = 1.00;
const PACE_UP = 0.055;           // Per second of clean driving
const PACE_DOWN_WIDE = 0.030;
const PACE_DOWN_OFF = 0.055;
const PACE_DOWN_SPIN = 0.070;

export class PaceOptimizer {
  /**
   * @param {Object} [options]
   * @param {number} [options.trailBrakingSkill=0.85] - Driver trail braking proficiency (0-1)
   * @param {number} [options.unwindFactor=0.60] - Exit throttle modulation intensity (0-1)
   * @param {number} [options.tcsSensitivity=0.90] - Traction control intervention sensitivity (0-1)
   * @param {number} [options.steerLimitGain=1.0] - Front tire saturation limiter scaling
   * @param {number} [options.paceTrim=0.96] - Initial learned pace trim (0.80 - 1.00)
   */
  constructor({
    trailBrakingSkill = 0.85,
    unwindFactor = 0.60,
    tcsSensitivity = 0.90,
    steerLimitGain = 1.0,
    paceTrim = 0.96
  } = {}) {
    this.trailBrakingSkill = clamp(trailBrakingSkill, 0, 1);
    this.unwindFactor = clamp(unwindFactor, 0, 1);
    this.tcsSensitivity = clamp(tcsSensitivity, 0, 1);
    this.steerLimitGain = clamp(steerLimitGain, 0.5, 1.5);

    // Online adaptive states
    this.yawInt = 0;           // Learned understeer gradient integral (rad)
    this.satAvg = 0;           // Filtered front axle saturation ratio
    this.satR = 0;             // Live rear axle saturation ratio
    this.alphaF = 0;           // Live front slip angle (rad)
    this.alphaR = 0;           // Live rear slip angle (rad)
    this.paceTrim = clamp(paceTrim, PACE_MIN, PACE_MAX);
    this.cleanTimer = 0;       // Duration of clean driving accumulator
    this.prevSteer = 0;
    this.prevBrake = 0;
    this.prevThrottle = 0;

    this.stats = {
      maxLateral: 0,
      offTrackTime: 0,
      spinTime: 0,
      spins: 0,
      minPace: this.paceTrim,
      samples: 0,
      sumPace: 0
    };
    this._wasSpinning = false;
  }

  /**
   * Reset online learned states and rate limiters.
   */
  reset() {
    this.yawInt = 0;
    this.satAvg = 0;
    this.satR = 0;
    this.alphaF = 0;
    this.alphaR = 0;
    this.cleanTimer = 0;
    this.prevSteer = 0;
    this.prevBrake = 0;
    this.prevThrottle = 0;
  }

  setParameters({
    trailBrakingSkill,
    unwindFactor,
    tcsSensitivity,
    steerLimitGain,
    paceTrim
  } = {}) {
    if (Number.isFinite(trailBrakingSkill)) this.trailBrakingSkill = clamp(trailBrakingSkill, 0, 1);
    if (Number.isFinite(unwindFactor)) this.unwindFactor = clamp(unwindFactor, 0, 1);
    if (Number.isFinite(tcsSensitivity)) this.tcsSensitivity = clamp(tcsSensitivity, 0, 1);
    if (Number.isFinite(steerLimitGain)) this.steerLimitGain = clamp(steerLimitGain, 0.5, 1.5);
    if (Number.isFinite(paceTrim)) this.paceTrim = clamp(paceTrim, PACE_MIN, PACE_MAX);
  }

  get pace() {
    return this.paceTrim;
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
    const vEst = Math.sqrt(G * 1.55 / kappa);
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
    const classBaseG = vehicleClass === 'prototype' ? (1.48 + 1.10 * downforceFactor) : vehicleClass === 'gt' ? 1.25 : 1.02;
    const peakG = classBaseG * tireGripFactor * aeroEffective * (0.88 + skill * 0.12);

    // Banking bonus: a_lat_eff = g * (peakG * cos(theta) + sin(theta))
    const bankAngle = Math.abs(finite(banking, 0));
    const bankCarry = Math.sin(bankAngle) * (vehicleClass === 'prototype' ? 1.35 : 1.05);
    const effectiveLatAccel = G * (peakG * Math.cos(bankAngle) + bankCarry);

    const classMargin = vehicleClass === 'prototype' ? 0.99 : vehicleClass === 'gt' ? 0.96 : 0.92;
    return Math.sqrt(effectiveLatAccel / kappa) * classMargin;
  }

  /**
   * Dynamic derating factor: how much to scale read plan speeds when vehicle is misaligned,
   * understeering, or off-line. Modulates read profile speeds rather than chopping the stopping envelope.
   * @private
   */
  _derate({ vehicle, track, targetOffset = 0 } = {}) {
    if (!vehicle) return 1.0;

    let misalign = 0;
    if (track?.atDistance) {
      const pt = track.atDistance(finite(vehicle.distance, 0));
      const trackHeading = Math.atan2(finite(pt.tangent?.x, 0), finite(pt.tangent?.z, 1));
      misalign = Math.abs(wrapAngle(finite(vehicle.yaw, 0) - trackHeading));
    }
    const align = misalign > 0.70 ? clamp(1.0 - (misalign - 0.70) * 1.1, 0.45, 1.0) : 1.0;
    const push = clamp(1.0 - Math.max(0, this.satAvg - 1.0) * 0.40, 0.80, 1.0);

    const currentLat = finite(vehicle.surface?.lateral, 0);
    const eq = Math.max(0, Math.abs(currentLat - targetOffset) - 1.40);
    const rejoin = clamp(1.0 - eq * 0.040, 0.90, 1.0);

    return align * push * rejoin;
  }

  /**
   * Forward Stopping Envelope Min-Reduction Braking.
   * Replaces conservative distance steps with exact min-reduction scan:
   *   v_allow = min_{d in [0, look]} sqrt(v_t^2 + 2 * a_B * d)
   * Allows the vehicle to hold 100% full throttle right up to the exact threshold brake point.
   * 
   * @param {Object} params
   * @returns {number} Target physical speed limit at current vehicle position in m/s
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
    const vSpeed = Math.max(0.1, finite(vehicle?.speed, 0));
    const vDist = finite(vehicle?.distance, 0);
    const vClass = vehicle?.classKey || 'prototype';

    // 1. Calibrate dynamic sustained braking deceleration capacity a_B (m/s²)
    const baseDecel = vClass === 'prototype' ? 10.8 : vClass === 'gt' ? 8.8 : 6.8;
    const brakingDecel = baseDecel * tireGripFactor * (0.85 + aggression * 0.25);

    // 2. Exact multi-distance lookahead scanning distances
    const speedEnvelopeDistances = [
      0, 2, 4, 6, 8, 10, 13, 16, 20, 24, 28, 33, 38, 44, 50, 58, 66, 75, 85, 96, 108, 122, 138, 155, 175, 198, 225, 255, 290, 330, 375
    ];

    let speedLimit = 95.0; // Track velocity ceiling
    const previewBuffer = Math.max(0, vSpeed * 0.18);

    const derate = this._derate({ vehicle, track, targetOffset: insideLineOffset });
    const effectiveSkill = (skill ?? 0.85) * this.paceTrim * derate;

    for (const dist of speedEnvelopeDistances) {
      const sampleDist = vDist + dist;
      const point = track?.atDistance ? track.atDistance(sampleDist) : { curvature: 0, banking: 0 };

      let rawCurvature = Math.abs(finite(point.curvature, 0));
      const roadWidth = finite(track?.roadHalfWidth, 6.5) + finite(track?.curbWidth, 1.05);
      const flattenFactor = (vClass === 'prototype')
        ? clamp(1.0 + 0.35 * roadWidth * Math.min(0.035, rawCurvature), 1.0, 1.25)
        : 1.0;
      let curvature = rawCurvature / flattenFactor;

      const safetyFactor = defending ? 0.95 : (aggression > 0.85 ? 1.03 : 0.96);
      const physLimit = this.calculateCornerSpeed({
        curvature,
        banking: point.bank ?? point.banking ?? 0,
        vehicleClass: vClass,
        tireGripFactor,
        skill: effectiveSkill,
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
    const maxTotalAccel = classBaseG * G * tireGripFactor;

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
   * Physics-Informed Front & Rear Saturation Guards + Integral Understeer Learning Steering Control:
   * 1. Curvature Feedforward + Lookahead Trajectory Tracking
   * 2. Integral Yaw-Rate Understeer Gradient Learner: eYaw = r_des - r, learns gradient dynamically with anti-windup
   * 3. Front-Axle Slip Saturation Guard (Anti-Plow): actively backs off steering lock when |alphaF| > alphaPeak
   * 4. Rear-Axle Slip Saturation Guard (Catch & Yaw Damper): fades path tracking out and fades yaw damping/countersteer in
   * 
   * @param {Object} params
   * @returns {number} Saturated steering command [-1, 1]
   */
  computeSteering({
    vehicle = null,
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
    const vSpeed = Math.max(3.0, finite(speed || vehicle?.speed, 0));
    const safeDt = clamp(finite(dt, 0.016), 0.001, 0.05);
    const vClass = vehicle?.classKey || 'prototype';
    const wheelBase = finite(vehicle?.wheelBase, (vClass === 'prototype' ? 2.65 : 2.70));
    const maxSteerAngle = finite(vehicle?.spec?.steering?.maxAngle, 0.55);
    const alphaPeak = finite(vehicle?.spec?.tire?.alphaPeak, (vClass === 'prototype' ? 0.115 : 0.140));

    // Sign-correct curvature: if currentCurvature is unsigned or opposing headingError
    const rawCurv = finite(currentCurvature, 0);
    const effCurv = (Math.abs(finite(headingError, 0)) > 0.03 && Math.sign(rawCurv) !== Math.sign(finite(headingError, 0)))
      ? -rawCurv
      : rawCurv;

    // --- 1. Extract Contact Patch Slip Angles ---
    let alphaF = 0;
    let alphaR = 0;
    if (vehicle?.wheels && vehicle.wheels.length >= 4) {
      alphaF = (finite(vehicle.wheels[0]?.slipAngle) + finite(vehicle.wheels[1]?.slipAngle)) * 0.5;
      alphaR = (finite(vehicle.wheels[2]?.slipAngle) + finite(vehicle.wheels[3]?.slipAngle)) * 0.5;
    } else if (Array.isArray(vehicle?.slipAngle) && vehicle.slipAngle.length >= 4) {
      alphaF = (finite(vehicle.slipAngle[0]) + finite(vehicle.slipAngle[1])) * 0.5;
      alphaR = (finite(vehicle.slipAngle[2]) + finite(vehicle.slipAngle[3])) * 0.5;
    } else {
      const betaEst = finite(slipAngle, Math.atan2(finite(vehicle?.localVelocity?.x, 0), Math.max(1.0, finite(vehicle?.localVelocity?.z, vSpeed))));
      alphaF = finite(previous, 0) * maxSteerAngle - betaEst - (wheelBase * 0.52 * finite(yawRate, 0)) / vSpeed;
      alphaR = -betaEst + (wheelBase * 0.48 * finite(yawRate, 0)) / vSpeed;
    }

    this.alphaF = alphaF;
    this.alphaR = alphaR;

    // --- 2. Front & Rear Saturation Guards ---
    const satF = Math.abs(alphaF) / (alphaPeak * SAT_RATIO);
    const satDir = Math.sign(alphaF);
    this.satAvg += (satF - this.satAvg) * clamp(safeDt * 6.0, 0, 1);

    // Rear saturation: fade out path tracking and fade in yaw damper/countersteer
    const satR = Math.abs(alphaR) / alphaPeak;
    this.satR = satR;
    const giveUp = clamp((satR - 1.0) / 0.45, 0, 1);
    const hold = 1.0 - giveUp;

    // --- 3. Integral Yaw-Rate Understeer Gradient Learner ---
    const rDes = effCurv * vSpeed;
    const eYaw = rDes - finite(yawRate, 0);

    // Anti-windup: freeze & decay integrator during saturation or near lock limit
    const currentSteerAngle = finite(previous, 0) * maxSteerAngle;
    if (satF < 1.0 && giveUp === 0 && Math.abs(currentSteerAngle) < maxSteerAngle * 0.95 && Math.abs(finite(headingError, 0)) > 0.04) {
      this.yawInt = clamp(this.yawInt + eYaw * K_YAW_I * safeDt, -0.06, 0.06);
    } else {
      this.yawInt *= (1.0 - clamp(safeDt * 6.0, 0, 1));
    }

    // --- 4. Countersteer Excess Body Slip ---
    const liveSlip = finite(slipAngle, Math.atan2(finite(vehicle?.localVelocity?.x, 0), Math.max(1.0, finite(vehicle?.localVelocity?.z, vSpeed))));
    const betaRef = Math.min(Math.abs(alphaR) * BETA_SLACK + 0.035, BETA_CAP) * (1.0 - giveUp);
    const betaExcess = liveSlip > betaRef ? liveSlip - betaRef : (liveSlip < -betaRef ? liveSlip + betaRef : 0);

    // Curvature feedforward (rad)
    const ff = Math.atan(wheelBase * effCurv);

    // Dynamic yaw damping & countersteer gains (ramp up under rear saturation)
    const kYaw = K_YAW * (1.0 + GIVEUP_YAW * giveUp);
    const kBeta = K_BETA * (1.0 + GIVEUP_BETA * giveUp);

    // Total road-wheel steering angle demand (in RADIANS)
    let cmd = ff * (1.0 - giveUp * 0.70)
      + (finite(headingError, 0) * (recovering ? 1.65 : (committed ? 1.35 : 1.15)) + this.yawInt) * hold
      + kYaw * eYaw
      + kBeta * betaExcess;

    // --- 5. Front-Axle Saturation Guard (Anti-Plow / Anti-Scrub Back-Off) ---
    if (satDir !== 0 && Math.sign(cmd) === satDir) {
      const over = clamp((satF - 1.0) / 0.10, 0, 1);
      if (over > 0) {
        const maxOptimalSteer = alphaPeak * 1.15 + (wheelBase * 0.52 * Math.abs(finite(yawRate, 0))) / vSpeed;
        const allow = Math.max(Math.abs(currentSteerAngle) * (1.0 - 0.45 * over), maxOptimalSteer);
        const capped = satDir * Math.min(Math.abs(cmd), allow);
        cmd += (capped - cmd) * over;
      }
    }

    // Convert rad to normalized steer [-1, 1]
    let targetSteer = clamp(cmd / maxSteerAngle, -1.0, 1.0);
    if (yielding) targetSteer = clamp(targetSteer, -0.30, 0.30);

    // Fast unwinding rate prevents yaw overshoots / snap back
    const isUnwinding = Math.sign(targetSteer) !== Math.sign(finite(previous)) || Math.abs(targetSteer) < Math.abs(finite(previous));
    const rate = isUnwinding ? 16.0 : (recovering ? 14.0 : (committed ? 11.0 : 9.0));
    const maxDelta = rate * clamp(safeDt, 0.005, 0.05);

    const steer = clamp(
      finite(previous) + clamp(targetSteer - finite(previous), -maxDelta, maxDelta),
      -1.0,
      1.0
    );
    this.prevSteer = steer;
    return steer;
  }

  /**
   * 2D G-G Friction-Circle Trail Braking, Longitudinal Control & Extremum-Seeking Pace Trim.
   * @param {Object} params
   * @returns {Object} { throttle, brake, friction, trailBraking, instability, tcsActive, paceTrim }
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
    defending = false,
    following = false,
    tireGripFactor = 1.0,
    dirtyAirLoss = 0,
    dt = 0.016
  } = {}) {
    const safeDt = clamp(finite(dt, 0.016), 0.001, 0.05);
    const vSpeed = finite(vehicle?.speed, 0);

    const friction = this.evaluateFrictionCircle({
      vehicle,
      lateralAccel,
      tireGripFactor
    });

    let throttle = 0;
    let brake = 0;

    // 1. Off-Track Rejoin & Recovery Mode
    if (recovering) {
      const rawSlip = finite(slipAngle, 0);
      const rawYawRate = finite(vehicle?.yawRate || yawRate, 0);
      const isSevereSpin = Math.abs(rawSlip) > 0.40 || Math.abs(rawYawRate) > 0.80 || (Math.abs(finite(headingError, 0)) > 2.2 && vSpeed > 8.0);

      if (isSevereSpin) {
        throttle = 0;
        brake = clamp(0.45 + vSpeed * 0.04, 0.35, 0.85);
      } else {
        throttle = vSpeed < desiredSpeed ? clamp(0.45 + (desiredSpeed - vSpeed) * 0.08, 0.35, 0.85) : 0;
        brake = vSpeed > desiredSpeed + 2.5 ? clamp(0.40 + (vSpeed - desiredSpeed) * 0.06, 0.30, 0.80) : 0;
      }
      this._observe({ vehicle, lateralError: 0, slipAngle, dt: safeDt });
      return { throttle, brake, friction, trailBraking: false, instability: 0, tcsActive: false, paceTrim: this.paceTrim };
    }

    if (emergency) {
      return { throttle: 0, brake: 1.0, friction, trailBraking: false, instability: 0, tcsActive: false, paceTrim: this.paceTrim };
    }

    // 2. Dynamic Speed Demand, Smooth Coasting & Progressive Threshold Braking
    const steerMagnitude = saturate(Math.abs(finite(steerAngle, 0)));
    const latUtil = friction.latUtilization;
    const isCornering = !straight && (steerMagnitude > 0.18 || latUtil > 0.48);

    const coastThreshold = isCornering ? -1.80 : -1.20;

    if (speedError >= 0) {
      // Acceleration: ramp throttle smoothly to full power
      const exitBonus = (!straight && steerMagnitude < 0.28) ? 0.15 : 0;
      const baseThrottle = following ? 0.35 : (straight ? 1.0 : (0.85 + exitBonus));
      const minThrottle = following ? 0.12 : 0.45;
      throttle = clamp(baseThrottle + finite(speedError) * 0.35, minThrottle, 1.0);
      brake = 0;
    } else if (speedError > coastThreshold) {
      // Momentum carry & smooth coasting: ZERO BRAKES
      const blend = (speedError - coastThreshold) / Math.max(0.01, -coastThreshold);
      throttle = clamp(blend * (isCornering ? 0.45 : 0.65), 0, 0.65);
      brake = 0;
    } else {
      // Sharp progressive braking demand up to threshold braking
      throttle = 0;
      const rawBrake = clamp((-speedError - Math.abs(coastThreshold)) * 0.38 + 0.15, 0.10, 1.0);
      brake = rawBrake;
    }

    // Rate-limit brake application (Fast bite 24/s, smooth release 18/s)
    const prevBrake = finite(vehicle?.controls?.brake ?? this.prevBrake, 0);
    const maxBrakeRate = brake > prevBrake ? 24.0 : 18.0;
    const maxBrakeDelta = maxBrakeRate * clamp(safeDt, 0.005, 0.05);
    brake = clamp(prevBrake + clamp(brake - prevBrake, -maxBrakeDelta, maxBrakeDelta), 0, 1.0);

    // 3. 2D G-G Friction-Circle Trail Braking Modulation
    let trailBrakingActive = false;
    if (brake > 0.03 && friction.latUtilization > 0.08) {
      trailBrakingActive = true;
      const latFactor = clamp(this.trailBrakingSkill * friction.latUtilization * 0.90, 0, 0.98);
      const remainingLongitudinal = Math.sqrt(Math.max(0.04, 1.0 - Math.pow(latFactor, 2)));
      brake = Math.min(brake, remainingLongitudinal);
    }

    // 4. Rear-Axle Saturation Slip Guard & Slide Stabilization
    const vClass = vehicle?.classKey || 'prototype';
    const alphaPeak = finite(vehicle?.spec?.tire?.alphaPeak, (vClass === 'prototype' ? 0.115 : 0.140));
    const aR = Math.abs(this.alphaR || 0);

    if (aR > alphaPeak * 1.05) {
      const over = aR / alphaPeak - 1.05;
      if (throttle > 0) {
        throttle = Math.max(0.20, throttle * clamp(1.0 - over * 3.2, 0.20, 1.0));
      }
      if (brake > 0 && friction.latUtilization > 0.35) {
        brake *= clamp(1.0 - over * 2.5, 0.25, 1.0);
      }
    }

    // 5. Traction Control System (TCS) & Exit Power Launch
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

    // Mid-corner apex drive: maintain positive throttle floor (60%) for downforce and rear load
    if (throttle > 0.05 && isCornering && vSpeed > 6.0 && brake < 0.05 && this.satR < 1.1 && !following) {
      throttle = Math.max(throttle, 0.60);
    }

    // Instant 100% full throttle launch on steering unwind or straights
    if (!following && speedError > -0.80 && steerMagnitude < 0.38 && brake < 0.05 && Math.abs(finite(slipAngle, 0)) < 0.14) {
      throttle = 1.0;
    }

    this.prevThrottle = throttle;
    this.prevBrake = brake;

    // 6. Extremum-Seeking Pace Trim Observer
    this._observe({
      vehicle,
      lateralError: finite(vehicle?.surface?.lateral, 0),
      slipAngle,
      dt: safeDt
    });

    return {
      throttle: clamp(throttle, 0, 1),
      brake: clamp(brake, 0, 1),
      friction,
      trailBraking: trailBrakingActive,
      instability: this.satR > 1.15 ? saturate((this.satR - 1.15) / 0.5) : 0,
      tcsActive,
      paceTrim: this.paceTrim
    };
  }

  /**
   * Extremum-Seeking Pace Trim Observer:
   * Advances paceTrim up to 1.0 during clean driving, pulls back on excess slip / off-track.
   * @private
   */
  _observe({ vehicle, lateralError = 0, slipAngle = 0, dt = 0.016 } = {}) {
    if (!vehicle) return;

    const vSpeed = finite(vehicle.speed, 0);
    const err = Math.abs(finite(lateralError, 0));
    const st = this.stats;

    if (vSpeed > 8.0) st.maxLateral = Math.max(st.maxLateral, err);
    if (vehicle.offTrack > 0) st.offTrackTime += dt;
    if (vehicle.spinTimer > 0) {
      st.spinTime += dt;
      if (!this._wasSpinning) { st.spins++; this._wasSpinning = true; }
    } else if (vehicle.spinTimer <= 0) {
      this._wasSpinning = false;
    }

    st.samples++;
    st.sumPace += this.paceTrim;
    if (this.paceTrim < st.minPace) st.minPace = this.paceTrim;

    let pay = 0;
    if (vehicle.spinTimer > 0.20) pay = PACE_DOWN_SPIN;
    else if (vehicle.offTrack > 0.35) pay = PACE_DOWN_OFF;
    else if (err > 1.6 && this.satAvg > 0.85 && vSpeed > 12.0) pay = PACE_DOWN_WIDE;
    else if (this.satAvg > 1.06 && vSpeed > 15.0) pay = PACE_DOWN_WIDE * 0.6;

    if (pay > 0) {
      this.paceTrim = Math.max(PACE_MIN, this.paceTrim - pay * dt * 10.0);
      this.cleanTimer = 0;
      return;
    }

    if (err < 1.35 && Math.abs(finite(slipAngle, 0)) < 0.20 && this.satAvg < 0.95 && vSpeed > 10.0 && (vehicle.offTrack || 0) <= 0) {
      this.cleanTimer += dt;
      if (this.cleanTimer > 0.35) {
        this.paceTrim = Math.min(PACE_MAX, this.paceTrim + PACE_UP * dt);
      }
    } else {
      this.cleanTimer = Math.max(0, this.cleanTimer - dt);
    }
  }
}
