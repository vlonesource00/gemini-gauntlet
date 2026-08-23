import { clamp } from '../core/math.js';

const approach = (current, target, rate, dt) => {
  const delta = target - current;
  const step = rate * dt;
  return Math.abs(delta) <= step ? target : current + Math.sign(delta) * step;
};

const filteredPedal = (current, target, attackSeconds, releaseSeconds, dt) => {
  const tau = target > current ? attackSeconds : releaseSeconds;
  return current + (target - current) * (1 - Math.exp(-dt / Math.max(tau, 0.001)));
};

/**
 * Translates binary keyboard inputs into a deterministic virtual driver's
 * controls with speed-sensitive steering lock, bicycle-model curvature scaling,
 * aero lateral budgeting, and yaw stabilization.
 */
export class KeyboardDynamics {
  constructor() {
    this.reset();
  }

  reset() {
    this.throttle = 0;
    this.brake = 0;
    this.steer = 0;
    this.handbrake = 0;
    this.diagnostics = {
      maxLock: 1,
      roadWheelLimit: 0,
      targetLateralAcceleration: 0,
      mechanicalLateralTarget: 0,
      aeroLateralContribution: 0,
      desiredYawRate: 0,
      counterSteer: 0,
      requestedSteer: 0,
      steeringReserve: 1,
      slipEnvelope: 0.105,
      speed: 0
    };
  }

  update(raw = {}, vehicle = null, dt = 1 / 120) {
    const safeDt = clamp(Number.isFinite(dt) ? dt : 1 / 120, 1 / 1000, 1 / 20);
    const speed = Math.max(0, vehicle?.speed ?? 0);
    const requestedSteer = clamp(raw.steer ?? 0, -1, 1);
    const lateralVelocity = vehicle?.localVelocity?.x
      ?? vehicle?.bodyVelocity?.x
      ?? 0;
    const longitudinalVelocity = vehicle?.localVelocity?.z
      ?? Math.max(1, speed);
    const slipAngle = Math.atan2(lateralVelocity, Math.max(3, Math.abs(longitudinalVelocity)));
    const yawRate = vehicle?.yawRate ?? vehicle?.angularVelocity?.y ?? 0;
    const wheelBase = Math.max(1, vehicle?.wheelBase ?? vehicle?.spec?.wheelBase ?? 2.7);
    const steeringLock = Math.max(0.05, vehicle?.spec?.steeringLock ?? 0.51);

    // Dynamic lateral acceleration target based on vehicle class and aero downforce
    const handling = vehicle?.spec?.handling ?? {};
    const mechanicalLateralTarget = Math.max(
      8,
      handling.keyboardLateralTarget
        ?? vehicle?.spec?.keyboardLateralAcceleration
        ?? 11.5
    );

    const mass = Math.max(250, vehicle?.mass ?? vehicle?.spec?.mass ?? 1200);
    const tireMu = Math.max(0.5, vehicle?.spec?.tire?.mu ?? 1.35);
    const aeroUtilisation = clamp(handling.keyboardAeroUtilization ?? 0.68, 0, 1);
    const liveDownforce = Math.max(0, vehicle?.aero?.downforceN ?? 0);
    const aeroLateralContribution = (liveDownforce / mass) * tireMu * aeroUtilisation;
    const maxLateralTarget = Math.max(mechanicalLateralTarget, handling.keyboardMaxLateralTarget ?? 24);
    const targetLateralAcceleration = clamp(
      mechanicalLateralTarget + aeroLateralContribution,
      mechanicalLateralTarget,
      maxLateralTarget
    );

    const lowSpeedBlendStart = handling.keyboardLowSpeedBlendStart ?? 4.5;
    const lowSpeedBlendEnd = Math.max(lowSpeedBlendStart + 2, handling.keyboardLowSpeedBlendEnd ?? 18);
    const lowSpeedBlend = clamp((speed - lowSpeedBlendStart) / (lowSpeedBlendEnd - lowSpeedBlendStart), 0, 1);
    const speedForCurvature = Math.max(4.5, speed);
    
    // δ = atan(L * a_lat / v²) Bicycle model curvature limitation
    const curvatureLimit = Math.atan((wheelBase * targetLateralAcceleration) / (speedForCurvature * speedForCurvature));
    const dynamicRoadWheelLimit = clamp(curvatureLimit, 0, steeringLock);
    const roadWheelLimit = steeringLock * (1 - lowSpeedBlend) + dynamicRoadWheelLimit * lowSpeedBlend;
    const maxLock = clamp(roadWheelLimit / steeringLock, 0, 1);

    const slipEnvelope = vehicle?.spec?.handling?.keyboardSlipEnvelope ?? 0.105;
    const requestedRoadWheel = roadWheelLimit * Math.abs(requestedSteer);
    const desiredYawRate = (speed * Math.tan(requestedRoadWheel)) / wheelBase;

    const yawExcess = desiredYawRate > 0.05
      ? clamp((Math.abs(yawRate) - desiredYawRate * 1.18) / Math.max(0.35, desiredYawRate * 0.85), 0, 1)
      : 0;
    const slipExcess = clamp((Math.abs(slipAngle) - slipEnvelope) / 0.22, 0, 1);
    const steeringReserve = clamp(1 - slipExcess * 0.24 - yawExcess * 0.56, 0.58, 1);
    
    const counterSteer = requestedSteer === 0
      ? clamp(slipAngle * 1.55 - yawRate * 0.105, -0.36, 0.36)
      : clamp(slipAngle * 0.48 - yawRate * 0.055, -0.18, 0.18);

    const targetSteer = clamp(requestedSteer * maxLock * steeringReserve + counterSteer, -maxLock, maxLock);
    const steerRate = requestedSteer === 0 ? 10.5 + speed * 0.04 : 6.5 + speed * 0.025;

    this.steer = approach(this.steer, targetSteer, steerRate, safeDt);
    this.throttle = filteredPedal(this.throttle, clamp(raw.throttle ?? 0, 0, 1), 0.08, 0.11, safeDt);
    this.brake = filteredPedal(this.brake, clamp(raw.brake ?? 0, 0, 1), 0.07, 0.095, safeDt);
    this.handbrake = filteredPedal(this.handbrake, clamp(raw.handbrake ?? 0, 0, 1), 0.045, 0.08, safeDt);

    this.diagnostics.maxLock = maxLock;
    this.diagnostics.roadWheelLimit = roadWheelLimit;
    this.diagnostics.targetLateralAcceleration = targetLateralAcceleration;
    this.diagnostics.mechanicalLateralTarget = mechanicalLateralTarget;
    this.diagnostics.aeroLateralContribution = aeroLateralContribution;
    this.diagnostics.desiredYawRate = desiredYawRate;
    this.diagnostics.counterSteer = counterSteer;
    this.diagnostics.steeringReserve = steeringReserve;
    this.diagnostics.slipEnvelope = slipEnvelope;
    this.diagnostics.requestedSteer = requestedSteer;
    this.diagnostics.speed = speed;

    return { throttle: this.throttle, brake: this.brake, steer: this.steer, handbrake: this.handbrake };
  }
}

export { approach };
