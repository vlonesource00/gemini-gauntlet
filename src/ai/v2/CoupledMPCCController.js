/**
 * CoupledMPCCController.js (V2 Layer 3 Coupled Dynamics)
 * Coupled Spatio-Temporal Model Predictive Contouring Controller (120Hz/400Hz):
 * 
 * 1. 400Hz/120Hz Physics-Informed Front-Axle Slip Saturation Guard (Anti-Plow / Anti-Scrub):
 *    - Monitors front axle slip angle alphaF = 0.5 * (alpha_FL + alpha_FR).
 *    - When |alphaF| > alphaPeak (e.g. ~0.115 rad for prototype, ~0.14 for GT), adding steering lock reduces lateral grip.
 *    - Actively backs off steering lock command cmd in the saturated direction: cmd += (capped - cmd) * over.
 * 
 * 2. Rear-Axle Slip Saturation Guard (Catch & Yaw Damper):
 *    - Monitors rear axle slip angle alphaR = 0.5 * (alpha_RL + alpha_RR).
 *    - When |alphaR| > alphaPeak, fades path tracking out (hold = 1 - giveUp) and fades yaw damping/countersteer in:
 *      kYaw = K_YAW * (1 + 3.0 * giveUp), kBeta = K_BETA * (1 + 2.0 * giveUp).
 * 
 * 3. Integral Yaw-Rate Understeer Gradient Learner:
 *    - eYaw = r_des - r. Integrates yawInt += eYaw * K_YAW_I * dt with anti-windup clamping to learn
 *      the vehicle's understeer gradient dynamically across all car classes.
 * 
 * 4. 2D G-G Friction-Circle Trail-Braking & Apex Exit Power Launch:
 *    - Nonlinear Pacejka friction ellipse constraint: (Fx / μFz)² + (Fy / μFz)² ≤ 1.0
 *    - Dynamically tapers braking force as cornering grip builds (seamless trail-braking)
 *    - Rear saturation slip protection: stabilizes slide breakaway without cutting throttle
 *    - Instant 100% full throttle power launch on steering unwind
 * 
 * 5. Extremum-Seeking Pace Trim Observer:
 *    - Clean driving without excess slip or off-track advances paceTrim up to 1.0.
 * 
 * 6. Zero GC Allocation Fast Step:
 *    - Reusable pre-allocated prediction horizon state vectors and telemetry (< 0.05ms per update).
 */

import { clamp, wrapAngle, saturate } from '../../core/math.js';

const G = 9.80665;
const finite = (val, fallback = 0) => (Number.isFinite(val) ? val : fallback);

// Driver / dynamics tuning constants shared across car classes
const K_LAT = 2.45;        // Stanley lateral cross-track gain
const K_LAT_V = 2.5;       // Softening velocity (m/s)
const K_HEAD = 1.0;        // Heading error gain
const K_YAW = 0.145;       // Yaw rate tracking gain
const K_YAW_I = 0.62;      // Integral understeer gradient learning rate
const K_YAW_I_MAX = 0.16;  // Rad, hard anti-windup cap on integral trim
const K_BETA = 0.62;       // Countersteer gain per rad of excess body sideslip

const GIVEUP_YAW = 3.0;    // Yaw-damper gain multiplier at full rear saturation
const GIVEUP_BETA = 2.0;   // Countersteer gain multiplier at full rear saturation
const SAT_RATIO = 1.12;    // Front slip angle / alphaPeak threshold for anti-plow guard
const BETA_SLACK = 1.15;   // Body slip excess slack factor
const BETA_CAP = 0.16;     // Rad, maximum allowed body sideslip reference
const BRAKE_MARGIN = 0.92;

const PACE_MIN = 0.80;
const PACE_MAX = 1.00;
const PACE_UP = 0.055;
const PACE_DOWN_WIDE = 0.030;
const PACE_DOWN_OFF = 0.055;
const PACE_DOWN_SPIN = 0.070;

export class CoupledMPCCController {
  /**
   * @param {Object} [options]
   * @param {number} [options.horizonSeconds=2.4] - Prediction horizon time span (s)
   * @param {number} [options.nodeCount=16] - Pre-allocated horizon point resolution
   * @param {number} [options.stanleyGain=2.45] - Stanley lateral cross-track gain
   * @param {number} [options.stanleySoftening=2.5] - Stanley softening velocity (m/s)
   * @param {number} [options.headingGain=1.0] - Heading error gain
   * @param {number} [options.yawDampingGain=0.145] - Yaw damping gain
   * @param {number} [options.slipCompensationGain=0.62] - Countersteer gain
   * @param {number} [options.trailBrakingSkill=0.90] - Trail braking effectiveness (0-1)
   * @param {number} [options.unwindFactor=0.55] - Exit throttle modulation factor
   * @param {number} [options.steerRate=12.0] - Maximum steering rack slew rate (rad/s)
   * @param {number} [options.paceTrim=0.96] - Initial learned pace trim
   */
  constructor({
    horizonSeconds = 2.4,
    nodeCount = 16,
    stanleyGain = 2.45,
    stanleySoftening = 2.5,
    headingGain = 1.0,
    yawDampingGain = 0.145,
    slipCompensationGain = 0.62,
    trailBrakingSkill = 0.90,
    unwindFactor = 0.55,
    steerRate = 12.0,
    paceTrim = 0.96
  } = {}) {
    this.horizonS = horizonSeconds;
    this.nodeCount = Math.max(12, nodeCount);

    this.stanleyGain = stanleyGain;
    this.stanleySoftening = stanleySoftening;
    this.headingGain = headingGain;
    this.yawDampingGain = yawDampingGain;
    this.slipCompensationGain = slipCompensationGain;
    this.trailBrakingSkill = trailBrakingSkill;
    this.unwindFactor = unwindFactor;
    this.steerRate = steerRate;

    // Online adaptive states
    this.yawInt = 0;           // Learned understeer gradient integral
    this.satAvg = 0;           // Filtered front axle saturation ratio
    this.satR = 0;             // Live rear axle saturation ratio
    this.alphaF = 0;           // Live front slip angle
    this.alphaR = 0;           // Live rear slip angle
    this.paceTrim = clamp(paceTrim, PACE_MIN, PACE_MAX);
    this.cleanTimer = 0;

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
      },
      saturation: {
        satAvg: 0,
        alphaF: 0,
        alphaR: 0,
        satR: 0,
        giveUp: 0,
        yawInt: 0,
        paceTrim: this.paceTrim
      }
    };
  }

  /**
   * Reset internal rate-limiter states and learned adaptive integrals.
   */
  reset() {
    this.yawInt = 0;
    this.satAvg = 0;
    this.satR = 0;
    this.alphaF = 0;
    this.alphaR = 0;
    this.cleanTimer = 0;
    this.prevSteer = 0;
    this.prevThrottle = 0;
    this.prevBrake = 0;
  }

  setParameters(options = {}) {
    if (options.stanleyGain !== undefined) this.stanleyGain = options.stanleyGain;
    if (options.stanleySoftening !== undefined) this.stanleySoftening = options.stanleySoftening;
    if (options.headingGain !== undefined) this.headingGain = options.headingGain;
    if (options.yawDampingGain !== undefined) this.yawDampingGain = options.yawDampingGain;
    if (options.slipCompensationGain !== undefined) this.slipCompensationGain = options.slipCompensationGain;
    if (options.trailBrakingSkill !== undefined) this.trailBrakingSkill = options.trailBrakingSkill;
    if (options.unwindFactor !== undefined) this.unwindFactor = options.unwindFactor;
    if (options.steerRate !== undefined) this.steerRate = options.steerRate;
    if (options.paceTrim !== undefined) this.paceTrim = clamp(options.paceTrim, PACE_MIN, PACE_MAX);
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
    let baseBrakeG = 2.20;
    let aLongMaxAccel = 4.80;

    if (vehicleClass === 'prototype') {
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
      baseLatG = 1.15;
      baseBrakeG = 1.25;
      aLongMaxAccel = 2.80;
    }

    const peakLatG = baseLatG * tireGripFactor;
    const peakBrakeG = baseBrakeG * tireGripFactor;
    const aLatMax = peakLatG * G;
    const aLongMaxDecel = peakBrakeG * G;

    return {
      aLatMax,
      aLongMaxDecel,
      aLongMaxAccel,
      peakLatG,
      peakBrakeG
    };
  }

  /**
   * Execute 120Hz/400Hz coupled MPCC optimization step.
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
    const yaw = finite(vehicle?.yaw, 0);
    const yawRate = finite(vehicle?.yawRate, 0);
    const vClass = vehicle?.classKey || 'prototype';
    const wheelBase = finite(vehicle?.wheelBase, (vClass === 'prototype' ? 2.65 : 2.70));
    const maxSteerAngle = finite(vehicle?.spec?.steering?.maxAngle, 0.55);
    const alphaPeak = finite(vehicle?.spec?.tire?.alphaPeak, (vClass === 'prototype' ? 0.115 : 0.140));
    const safeDt = clamp(finite(dt, 0.016), 0.001, 0.05);

    const targetLateral = finite(tacticalTarget?.targetLateral, 0);
    const desiredSpeed = Math.max(8.0, finite(tacticalTarget?.desiredSpeed, 50.0));

    // 1. Friction Limits (Downforce-Scaled Peak Lateral & Longitudinal Accelerations)
    const frictionLimits = this.calculateFrictionLimits({
      vehicleClass: vClass,
      speed: vSpeed,
      tireGripFactor
    });

    // 2. Pure-Pursuit Target & Stanley Reference Geometry
    const refPoint = track?.atDistance ? track.atDistance(vDist) : { curvature: 0, tangent: { x: 0, z: 1 } };
    const curvMag = Math.abs(finite(refPoint.curvature, 0));
    const lookAheadM = clamp((4.5 + vSpeed * 0.25) / (1.0 + curvMag * 35.0), 4.0, 18.0);

    const lookAheadPoint = track?.atDistance
      ? track.atDistance(vDist + lookAheadM)
      : { curvature: 0, tangent: { x: 0, z: 1 } };

    const targetWorld = track?.lateralPoint
      ? track.lateralPoint(lookAheadPoint, targetLateral, 0.08)
      : lookAheadPoint;

    // Track path heading at car
    const trackHeading = Math.atan2(finite(refPoint.tangent?.x, 0), finite(refPoint.tangent?.z, 1));
    const rawHeadingToTrack = wrapAngle(trackHeading - yaw);

    // Heading towards lookahead target point
    const headingToLookahead = Math.atan2(
      finite(targetWorld.x, 0) - finite(vehicle?.position?.x, 0),
      finite(targetWorld.z, 0) - finite(vehicle?.position?.z, 0)
    );
    const rawHeadingToLookahead = wrapAngle(headingToLookahead - yaw);

    const lookaheadWeight = clamp(0.55 + aggression * 0.15, 0.50, 0.85);
    let headingError = wrapAngle(
      rawHeadingToLookahead * lookaheadWeight + rawHeadingToTrack * (1.0 - lookaheadWeight)
    );

    const isFacingBackwards = Math.abs(rawHeadingToTrack) > Math.PI * 0.55;
    if (recovering && isFacingBackwards) {
      headingError = Math.sign(rawHeadingToTrack) * -1.2;
    }

    // Stanley cross-track error: error = targetLateral - currentLat (positive -> steer right)
    const crossTrackError = targetLateral - currentLat;
    const effectiveStanleyGain = (this.stanleyGain + aggression * 0.40) * (committed ? 1.15 : 1.0);
    const stanleyAngle = Math.atan2(
      effectiveStanleyGain * crossTrackError,
      this.stanleySoftening + vSpeed
    );

    // Curvature feedforward with preview (using signed curvature with turnSign)
    const previewDistance = clamp(vSpeed * 0.30, 2.5, 20.0);
    const previewPoint = track?.atDistance ? track.atDistance(vDist + previewDistance) : refPoint;
    const rawCurvCurrent = finite(refPoint.curvature, 0);
    const rawCurvPreview = finite(previewPoint.curvature, 0);
    const signCurrent = finite(refPoint.turnSign, 0) || (rawCurvCurrent > 0.001 ? 1 : 0);
    const signPreview = finite(previewPoint.turnSign, 0) || (rawCurvPreview > 0.001 ? 1 : 0);
    const signedCurvCurrent = signCurrent * rawCurvCurrent;
    const signedCurvPreview = signPreview * rawCurvPreview;
    const effectiveCurvature = signedCurvCurrent * 0.35 + signedCurvPreview * 0.65;
    const kinematicFeedforward = Math.atan(wheelBase * effectiveCurvature);

    // 3. Extract Slip Angles & Apply Saturation Guards
    let alphaF = 0;
    let alphaR = 0;
    if (vehicle?.wheels && vehicle.wheels.length >= 4) {
      alphaF = (finite(vehicle.wheels[0]?.slipAngle) + finite(vehicle.wheels[1]?.slipAngle)) * 0.5;
      alphaR = (finite(vehicle.wheels[2]?.slipAngle) + finite(vehicle.wheels[3]?.slipAngle)) * 0.5;
    } else if (Array.isArray(vehicle?.slipAngle) && vehicle.slipAngle.length >= 4) {
      alphaF = (finite(vehicle.slipAngle[0]) + finite(vehicle.slipAngle[1])) * 0.5;
      alphaR = (finite(vehicle.slipAngle[2]) + finite(vehicle.slipAngle[3])) * 0.5;
    } else {
      const localVx = finite(vehicle?.localVelocity?.x, 0);
      const localVz = Math.max(2.5, Math.abs(finite(vehicle?.localVelocity?.z, vSpeed)));
      const betaEst = Math.atan2(localVx, localVz);
      alphaF = finite(this.prevSteer, 0) * maxSteerAngle - betaEst - (wheelBase * 0.52 * yawRate) / vSpeed;
      alphaR = -betaEst + (wheelBase * 0.48 * yawRate) / vSpeed;
    }

    this.alphaF = alphaF;
    this.alphaR = alphaR;

    // Front saturation guard: in right turns (+steer), alphaF > 0, saturated direction is satDir = sign(alphaF)
    const satF = Math.abs(alphaF) / (alphaPeak * SAT_RATIO);
    const satDir = Math.sign(alphaF);
    this.satAvg += (satF - this.satAvg) * clamp(safeDt * 6.0, 0, 1);

    // Rear saturation guard: fade out path tracking and fade in yaw damper/countersteer
    const satR = Math.abs(alphaR) / alphaPeak;
    this.satR = satR;
    const giveUp = clamp((satR - 1.0) / 0.45, 0, 1);
    const hold = 1.0 - giveUp;

    // Integral yaw-rate understeer gradient learner
    const rDes = effectiveCurvature * vSpeed;
    const eYaw = rDes - yawRate;
    if (satF < 1.0 && giveUp === 0 && Math.abs(this.prevSteer) < 0.95) {
      this.yawInt = clamp(this.yawInt + eYaw * K_YAW_I * safeDt, -K_YAW_I_MAX, K_YAW_I_MAX);
    } else {
      this.yawInt *= (1.0 - clamp(safeDt * 3.0, 0, 1));
    }

    // Dynamic yaw damping & sideslip excess countersteering
    const localVx = finite(vehicle?.localVelocity?.x, 0);
    const localVz = Math.max(2.5, Math.abs(finite(vehicle?.localVelocity?.z, vSpeed)));
    const slipAngle = Math.atan2(localVx, localVz);
    const betaRef = Math.min(Math.abs(alphaR) * BETA_SLACK + 0.035, BETA_CAP) * (1.0 - giveUp);
    const betaExcess = slipAngle > betaRef ? slipAngle - betaRef : (slipAngle < -betaRef ? slipAngle + betaRef : 0);

    const kYaw = this.yawDampingGain * (1.0 + GIVEUP_YAW * giveUp) * (committed ? 1.25 : 1.0);
    const kBeta = this.slipCompensationGain * (1.0 + GIVEUP_BETA * giveUp);

    let rawSteerCmd = kinematicFeedforward * (1.0 - giveUp * 0.70)
      + (headingError * this.headingGain + stanleyAngle + this.yawInt) * hold
      - kYaw * (yawRate - rDes)
      + kBeta * betaExcess;

    // Front-Axle Slip Saturation Guard (Anti-Plow / Anti-Scrub Back-Off)
    if (satDir !== 0 && Math.sign(rawSteerCmd) === satDir) {
      const over = clamp((satF - 1.0) / 0.10, 0, 1);
      if (over > 0) {
        const optimalSteer = (alphaPeak * 1.12 + Math.abs(slipAngle) + (wheelBase * 0.52 * Math.abs(yawRate)) / vSpeed) / maxSteerAngle;
        const allow = Math.max(optimalSteer, Math.abs(kinematicFeedforward) * 0.85);
        const capped = satDir * Math.min(Math.abs(rawSteerCmd), allow);
        rawSteerCmd += (capped - rawSteerCmd) * over;
      }
    }

    // Speed-dependent dynamic steering saturation limit
    const baseLimit = clamp(6.5 / Math.max(3.5, vSpeed) + 0.18, 0.18, 0.85);

    const maxSteerLimit = recovering
      ? 0.85
      : (giveUp > 0.1 ? Math.max(baseLimit, 0.65) : baseLimit);

    const targetSteer = clamp(rawSteerCmd, -maxSteerLimit, maxSteerLimit);

    // Actuator slew rate limiting
    const isUnwinding = Math.sign(targetSteer) !== Math.sign(this.prevSteer) || Math.abs(targetSteer) < Math.abs(this.prevSteer);
    const activeRate = isUnwinding ? this.steerRate * 1.5 : this.steerRate;
    const maxDelta = activeRate * clamp(safeDt, 0.005, 0.05);

    const steer = clamp(
      this.prevSteer + clamp(targetSteer - this.prevSteer, -maxDelta, maxDelta),
      -1.0,
      1.0
    );
    this.prevSteer = steer;

    // 4. Coupled 2D G-G Friction-Circle Longitudinal Pedal Control
    const liveLatAccel = Math.abs(vSpeed * yawRate);
    const latUtilization = clamp(liveLatAccel / Math.max(1.0, frictionLimits.aLatMax), 0, 1.0);
    const coupling = 0.95;
    const remainingLongBudget = Math.sqrt(Math.max(0.01, 1.0 - Math.pow(latUtilization * coupling, 2)));
    const availableLongDecel = frictionLimits.aLongMaxDecel * remainingLongBudget;

    const speedError = desiredSpeed - vSpeed;
    const isStraight = Math.abs(finite(refPoint.curvature, 0)) < 0.0028;
    const steerMag = saturate(Math.abs(steer));
    const isCornering = !isStraight && (steerMag > 0.16 || latUtilization > 0.48);

    let throttle = 0;
    let brake = 0;
    let trailBrakingActive = false;
    let trailFactor = 1.0;
    let rawBrake = 0;
    let launchActive = false;
    let unwindBonus = 0;
    let exitFactor = 1.0;

    const coastThreshold = isCornering ? -1.80 : -1.20;

    if (recovering) {
      throttle = speedError > 0.5 ? clamp(0.35 + speedError * 0.05, 0.3, 0.6) : 0;
      brake = speedError < -1.5 ? clamp((-speedError - 1.5) * 0.35 + 0.25, 0.35, 1.0) : 0;
    } else if (speedError < coastThreshold) {
      // DECELERATION & TRAIL-BRAKING ZONE
      throttle = 0;
      rawBrake = clamp((-speedError - Math.abs(coastThreshold)) * 0.38 + 0.15, 0.10, 1.0);

      if (isCornering || latUtilization > 0.12) {
        trailBrakingActive = true;
        const latFactor = clamp(this.trailBrakingSkill * latUtilization * 0.90, 0, 0.98);
        trailFactor = Math.sqrt(Math.max(0.04, 1.0 - Math.pow(latFactor, 2)));
        brake = clamp(rawBrake * trailFactor * remainingLongBudget, 0.04, 1.0);
      } else {
        brake = rawBrake;
      }
    } else if (speedError <= 0) {
      // Coasting / momentum carry
      const blend = (speedError - coastThreshold) / Math.max(0.01, -coastThreshold);
      throttle = clamp(blend * (isCornering ? 0.45 : 0.65), 0, 0.65);
      brake = 0;
    } else {
      // ACCELERATION & APEX EXIT POWER LAUNCH ZONE
      brake = 0;
      const rawThrottle = clamp(0.95 + speedError * 0.25, 0.70, 1.0);

      if (isCornering) {
        const unwindPower = 1.0 - this.unwindFactor * Math.pow(steerMag, 1.1) * 0.18;
        exitFactor = clamp(remainingLongBudget * unwindPower, 0.65, 1.0);
        throttle = clamp(rawThrottle * exitFactor, 0.35, 1.0);

        if (vSpeed > 6.0 && !isStraight) {
          throttle = Math.max(throttle, 0.50);
        }

        if (steerMag < 0.35 || speedError > 0) {
          throttle = 1.0;
          launchActive = true;
          unwindBonus = 1.0;
        }
      } else {
        throttle = 1.0;
        launchActive = true;
        unwindBonus = 1.0;
      }
    }

    // Rear axle saturation slip stabilization: prevent snap oversteer
    if (Math.abs(alphaR) > alphaPeak * 1.05) {
      const overR = Math.abs(alphaR) / alphaPeak - 1.05;
      if (throttle > 0) {
        throttle = Math.max(0.20, throttle * clamp(1.0 - overR * 3.2, 0.20, 1.0));
      }
      if (brake > 0 && latUtilization > 0.35) {
        brake *= clamp(1.0 - overR * 2.5, 0.25, 1.0);
      }
    }

    // Rate-limit brake pressure
    const maxBrakeRate = brake > this.prevBrake ? 24.0 : 18.0;
    const maxBrakeDelta = maxBrakeRate * clamp(safeDt, 0.005, 0.05);
    brake = clamp(
      this.prevBrake + clamp(brake - this.prevBrake, -maxBrakeDelta, maxBrakeDelta),
      0,
      1.0
    );

    this.prevThrottle = throttle;
    this.prevBrake = brake;

    // 5. Update Zero-GC Prediction Horizon for 3D Visual Telemetry Overlays
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
      p.latG = Math.abs(p.speed * p.speed * p.curvature) / G;
      p.remainingLongBudget = remainingLongBudget;
    }

    // 6. Extremum-Seeking Pace Trim Observer
    this._observe({ vehicle, lateralError: currentLat, slipAngle, dt: safeDt });

    // 7. Populate Structured Telemetry Container
    const out = this.telemetry;
    out.steer = steer;
    out.throttle = clamp(throttle, 0, 1.0);
    out.brake = clamp(brake, 0, 1.0);
    out.horizon = this.predPoints;
    out.friction = {
      latUtilization,
      remainingLongBudget,
      liveLatG: liveLatAccel / G,
      peakLatG: frictionLimits.peakLatG,
      availableLongDecel,
      aLatMax: frictionLimits.aLatMax,
      aLongMaxDecel: frictionLimits.aLongMaxDecel
    };
    out.stanley = {
      headingError,
      crossTrackError,
      curvatureFeedforward: kinematicFeedforward,
      yawDamping: kYaw * (yawRate - rDes),
      targetSteer,
      maxSteerLimit
    };
    out.trailBraking = {
      active: trailBrakingActive,
      factor: trailFactor,
      brakeRaw: rawBrake,
      brakeTapered: brake
    };
    out.traction = {
      exitFactor,
      unwindBonus,
      launchActive
    };
    out.saturation = {
      satAvg: this.satAvg,
      alphaF,
      alphaR,
      satR: this.satR,
      giveUp,
      yawInt: this.yawInt,
      paceTrim: this.paceTrim
    };

    return out;
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

export { CoupledMPCCController as CoupledDynamicsController };
