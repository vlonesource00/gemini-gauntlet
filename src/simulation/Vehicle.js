import { clamp, damp, length2, localToWorld, worldToLocal } from '../core/math.js';
import { carSpecFor } from './CarSpecs.js';
import { createTireState, tireForces } from './Tire.js';

const G = 9.81;
const TAU = Math.PI * 2;
const ERS_MODES = ['OFF', 'AUTO', 'ATTACK'];
const signOr = (value, fallback = 1) => Math.sign(value) || fallback;
const finite = (value, fallback = 0) => Number.isFinite(value) ? value : fallback;

const cleanWake = () => ({
  strength: 0,
  wakeStrength: 0,
  dragReduction: 0,
  frontDownforceLoss: 0,
  rearDownforceLoss: 0,
  dragMultiplier: 1,
  frontDownforceMultiplier: 1,
  rearDownforceMultiplier: 1,
  sourceId: null,
  sourceClass: null,
  source: null
});

// Convert a virtual centre steer angle into the angle required at one front
// contact patch.  The instantaneous centre is taken on the rear axle line, so
// the centreline wheel retains the requested curvature while the inner wheel
// gains angle and the outer wheel sheds angle.  Positive steering turns local
// +X; therefore FR (x > 0) is inner for a positive turn and FL for a negative
// turn.
export function ackermannSteerAngle(centerSteer, wheelX, wheelBase) {
  const angle = finite(centerSteer);
  if (Math.abs(angle) < 1e-6) return 0;
  const base = Math.max(0.5, finite(wheelBase, 2.7));
  const side = Math.sign(angle);
  const radius = base / Math.tan(Math.abs(angle));
  // Keep the denominator finite even if a future class receives an extreme
  // steering lock; current presets remain comfortably outside this guard.
  const wheelRadius = Math.max(0.05, radius - side * finite(wheelX));
  const result = Math.atan2(base, wheelRadius);
  return side * result;
}

export class Vehicle {
  constructor({ id = 'vehicle', name = 'VEHICLE', color = '#ffffff', player = false, spec = 'gt', classKey = null } = {}) {
    this.id = id;
    this.name = name;
    this.color = color;
    this.player = player;
    this.position = { x: 0, y: 0, z: 0 };
    this.velocity = { x: 0, y: 0, z: 0 };
    this.acceleration = { x: 0, y: 0, z: 0 };
    this.localAcceleration = { x: 0, z: 0 };
    this.localVelocity = { x: 0, z: 0 };
    this.bodyVelocity = this.localVelocity;
    this.yaw = 0;
    this.yawRate = 0;
    this.roll = 0;
    this.pitch = 0;
    this.roadBank = 0;
    this.roadGrade = 0;
    this._previousRoll = 0;
    this._previousPitch = 0;
    this.angularVelocity = { x: 0, y: 0, z: 0 };
    this.orientation = { x: 0, y: 0, z: 0, w: 1 };
    this.controls = { throttle: 0, brake: 0, steer: 0, handbrake: 0 };
    this.brakePressure = 0;
    this.steering = 0;
    this.gear = 1;
    this.rpm = 1100;
    this.engineOmega = this.rpm * TAU / 60;
    this.engineTorque = 0;
    this.engineBrakeTorque = 0;
    this.speed = 0;
    this.distance = 0;
    this.previousDistance = 0;
    this.surface = null;
    this.finished = false;
    this.cooldownActive = false;
    this.cooldownTime = 0;
    this.trafficGhost = false;
    this.despawned = false;
    this.damage = 0;
    this.impact = 0;
    this.shiftTimer = 0;
    this.transmissionMode = 'automatic';
    this.lastShiftRejected = null;
    this.autoBlip = 0;
    this.rideHeight = 0.068;
    this.aero = {
      downforceN: 0, frontN: 0, rearN: 0, dragN: 0, balance: 0.5, groundEffect: 0,
      wakeStrength: 0, dragReduction: 0, frontDownforceLoss: 0, rearDownforceLoss: 0,
      wakeSource: null, dragMultiplier: 1, frontDownforceMultiplier: 1, rearDownforceMultiplier: 1,
      wakeDragMultiplier: 1,
      wakeFrontDownforceMultiplier: 1, wakeRearDownforceMultiplier: 1
    };
    this.wake = cleanWake();
    this.electronics = {
      tcLevel: 4, absLevel: 5, brakeBias: 0.57,
      tcActivity: 0, absActivity: 0, tcPhase: 0, absPhase: 0,
      tcSlipFiltered: 0, tcIntervention: 0, tcLatched: false
    };
    this.ers = {
      enabled: false, mode: 'OFF', capacityJ: 0, energyJ: 0, soc: 0,
      deployPowerW: 0, deployMechanicalPowerW: 0, regenPowerW: 0, regenMechanicalPowerW: 0,
      regenElectricalPowerW: 0, driveTorqueNm: 0, regenTorqueNm: 0,
      rearAxleOmegaRadS: 0,
      state: 'OFF'
    };
    this.telemetry = {
      longitudinalG: 0, lateralG: 0, verticalG: 0, gDotX: 0, gDotY: 0,
      tyreUtilisation: 0, suspensionTravel: 0,
      wakeStrength: 0, dragReduction: 0, frontDownforceLoss: 0,
      rearDownforceLoss: 0, wakeSource: null,
      dragMultiplier: 1, frontDownforceMultiplier: 1, rearDownforceMultiplier: 1,
      wakeDragMultiplier: 1,
      wakeFrontDownforceMultiplier: 1, wakeRearDownforceMultiplier: 1
    };
    this.aiTarget = null;
    this.setSpec(classKey ?? spec, true);
  }

  setSpec(value, initial = false) {
    this.spec = carSpecFor(value);
    this.classKey = this.spec.key;
    this.mass = this.spec.mass;
    this.inertiaTensor = { ...this.spec.inertia };
    this.inertia = this.inertiaTensor.y;
    this.wheelBase = this.spec.wheelBase;
    this.trackWidth = this.spec.trackWidth;
    this.cgHeight = this.spec.cgHeight;
    this.wheelRadius = this.spec.wheelRadius ?? 0.335;
    this.electronics.brakeBias = this.spec.brakeBias;
    this.rideHeight = this.spec.aero.designRideHeight + 0.006;
    this._resetWake();
    this._resetERS();
    this.wheels = this._makeWheels();
    if (!initial) this.place(this.position.x, this.position.z, this.yaw, this.position.y);
    return this.spec;
  }

  _resetWake() {
    this.wake = cleanWake();
    if (this.aero) {
      this.aero.wakeStrength = 0;
      this.aero.dragReduction = 0;
      this.aero.frontDownforceLoss = 0;
      this.aero.rearDownforceLoss = 0;
      this.aero.wakeSource = null;
      this.aero.dragMultiplier = 1;
      this.aero.frontDownforceMultiplier = 1;
      this.aero.rearDownforceMultiplier = 1;
      this.aero.wakeDragMultiplier = 1;
      this.aero.wakeFrontDownforceMultiplier = 1;
      this.aero.wakeRearDownforceMultiplier = 1;
    }
    if (this.telemetry) {
      this.telemetry.wakeStrength = 0;
      this.telemetry.dragReduction = 0;
      this.telemetry.frontDownforceLoss = 0;
      this.telemetry.rearDownforceLoss = 0;
      this.telemetry.wakeSource = null;
      this.telemetry.dragMultiplier = 1;
      this.telemetry.frontDownforceMultiplier = 1;
      this.telemetry.rearDownforceMultiplier = 1;
      this.telemetry.wakeDragMultiplier = 1;
      this.telemetry.wakeFrontDownforceMultiplier = 1;
      this.telemetry.wakeRearDownforceMultiplier = 1;
    }
  }

  _makeWheels() {
    const frontZ = this.wheelBase * 0.52;
    const rearZ = -this.wheelBase * 0.48;
    const staticFront = this.mass * G * this.spec.weightFront / 2 / this.spec.springRate.front;
    const staticRear = this.mass * G * (1 - this.spec.weightFront) / 2 / this.spec.springRate.rear;
    const build = (name, x, z, front, compression) => {
      const tyre = createTireState(this.spec.tire);
      return {
      name, x, z, front, omega: 0, compression: clamp(compression, 0, this.spec.suspension.travel),
      suspensionVelocity: 0, suspensionForce: 0, normalLoad: 0, contactHeight: 0,
      camber: -0.025, steer: 0, slip: 0, slipRatio: 0, slipAngle: 0, utilisation: 0,
      temperature: tyre.temperatureMiddleC, pressurePa: tyre.pressurePa,
      temperatureInnerC: tyre.temperatureInnerC, temperatureMiddleC: tyre.temperatureMiddleC,
      temperatureOuterC: tyre.temperatureOuterC, carcassTemperatureC: tyre.carcassTemperatureC,
      wear: 0, tyre
    }; };
    return [
      build('FL', -this.trackWidth / 2, frontZ, true, staticFront),
      build('FR', this.trackWidth / 2, frontZ, true, staticFront),
      build('RL', -this.trackWidth / 2, rearZ, false, staticRear),
      build('RR', this.trackWidth / 2, rearZ, false, staticRear)
    ];
  }

  place(x, z, yaw, y = this.position.y) {
    this.position.x = finite(x);
    this.position.y = finite(y);
    this.position.z = finite(z);
    this.velocity.x = this.velocity.y = this.velocity.z = 0;
    this.acceleration.x = this.acceleration.y = this.acceleration.z = 0;
    this.yaw = finite(yaw);
    this.yawRate = 0;
    this.angularVelocity.x = this.angularVelocity.y = this.angularVelocity.z = 0;
    this.brakePressure = 0;
    this.roll = this.pitch = 0;
    this.roadBank = 0;
    this.roadGrade = 0;
    this._previousRoll = 0;
    this._previousPitch = 0;
    this.steering = 0;
    this.rpm = this.spec.idleRpm;
    this.engineOmega = this.rpm * TAU / 60;
    this.gear = 1;
    this.shiftTimer = 0;
    this.lastShiftRejected = null;
    this.autoBlip = 0;
    this.cooldownActive = false;
    this.cooldownTime = 0;
    this.trafficGhost = false;
    this.despawned = false;
    this._resetWake();
    this._resetERS();
    this.electronics.tcActivity = 0;
    this.electronics.tcSlipFiltered = 0;
    this.electronics.tcIntervention = 0;
    this.electronics.tcLatched = false;
    this.wheels.forEach((wheel) => {
      wheel.omega = 0;
      wheel.suspensionVelocity = 0;
      wheel.tyre = createTireState(this.spec.tire);
      wheel.temperature = wheel.tyre.temperatureMiddleC;
      wheel.temperatureInnerC = wheel.tyre.temperatureInnerC;
      wheel.temperatureMiddleC = wheel.tyre.temperatureMiddleC;
      wheel.temperatureOuterC = wheel.tyre.temperatureOuterC;
      wheel.carcassTemperatureC = wheel.tyre.carcassTemperatureC;
      wheel.pressurePa = wheel.tyre.pressurePa;
      wheel.compression = wheel.front
        ? this.mass * G * this.spec.weightFront / 2 / this.spec.springRate.front
        : this.mass * G * (1 - this.spec.weightFront) / 2 / this.spec.springRate.rear;
    });
    this._updateOrientation();
  }

  get forward() { return { x: Math.sin(this.yaw), z: Math.cos(this.yaw) }; }
  get right() { return { x: Math.cos(this.yaw), z: -Math.sin(this.yaw) }; }

  resetTo(track, distance, lateral = 0) {
    const point = track.atDistance(distance);
    const placed = track.lateralPoint ? track.lateralPoint(point, lateral) : { x: point.x + point.normal.x * lateral, y: point.y ?? 0, z: point.z + point.normal.z * lateral };
    this.place(placed.x, placed.z, Math.atan2(point.tangent.x, point.tangent.z), placed.y + this.rideHeight);
    this.distance = point.s;
    this.previousDistance = point.s;
    this.surface = track.surfaceAt(this.position.x, this.position.z);
    this.roadBank = finite(this.surface?.bank);
    this.roadGrade = finite(this.surface?.grade);
  }

  marshalRecoverTo(track, distance = this.distance + 7, lateral = 0) {
    const point = track.atDistance(distance);
    const placed = track.lateralPoint
      ? track.lateralPoint(point, lateral)
      : { x: point.x + point.normal.x * lateral, y: point.y ?? 0, z: point.z + point.normal.z * lateral };
    const recoverySpeed = 4.5;
    this.position.x = finite(placed.x);
    this.position.y = finite(placed.y) + this.rideHeight;
    this.position.z = finite(placed.z);
    this.yaw = Math.atan2(point.tangent.x, point.tangent.z);
    this.velocity.x = point.tangent.x * recoverySpeed;
    this.velocity.y = 0;
    this.velocity.z = point.tangent.z * recoverySpeed;
    this.acceleration.x = this.acceleration.y = this.acceleration.z = 0;
    this.yawRate = 0;
    this.angularVelocity.x = this.angularVelocity.y = this.angularVelocity.z = 0;
    this.steering = 0;
    this.brakePressure = 0;
    this.distance = point.s;
    this.previousDistance = point.s;
    this.surface = track.surfaceAt(this.position.x, this.position.z);
    this.roadBank = finite(this.surface?.bank);
    this.roadGrade = finite(this.surface?.grade);
    for (const wheel of this.wheels) {
      wheel.omega = recoverySpeed / this.wheelRadius;
      wheel.slip = wheel.slipRatio = wheel.slipAngle = 0;
    }
    this._updateOrientation();
  }

  setTCLevel(level) { this.electronics.tcLevel = clamp(Math.round(level), 0, 7); return this.electronics.tcLevel; }
  setABSLevel(level) { this.electronics.absLevel = clamp(Math.round(level), 0, 7); return this.electronics.absLevel; }
  adjustTC(delta) { return this.setTCLevel(this.electronics.tcLevel + delta); }
  adjustABS(delta) { return this.setABSLevel(this.electronics.absLevel + delta); }
  cycleBrakeBias() {
    const next = this.electronics.brakeBias >= 0.66 ? 0.54 : this.electronics.brakeBias + 0.02;
    this.electronics.brakeBias = Number(next.toFixed(2));
    return this.electronics.brakeBias;
  }

  _resetERS() {
    const preset = this.spec?.ers;
    const enabled = Boolean(preset?.enabled && this.classKey === 'prototype');
    const capacityJ = enabled ? Math.max(1, finite(preset.capacityJ, 5.4e6)) : 0;
    const initialSoc = enabled ? clamp(finite(preset.initialSoc, 0.74), 0, 1) : 0;
    this.ers = {
      enabled,
      mode: enabled ? 'AUTO' : 'OFF',
      capacityJ,
      energyJ: capacityJ * initialSoc,
      soc: initialSoc,
      minSoc: enabled ? clamp(finite(preset.minSoc, 0.04), 0, 0.95) : 0,
      maxDeployPowerW: enabled ? Math.max(0, finite(preset.maxDeployPowerW, 50e3)) : 0,
      // AUTO is intentionally a sustainable partial-deployment mode.  Keep
      // maxDeployPowerW as the electrical hard ceiling (and public contract),
      // while this separate ceiling limits ordinary AUTO use.
      autoDeployPowerW: enabled
        ? Math.min(
          Math.max(0, finite(preset.autoDeployPowerW, finite(preset.maxDeployPowerW, 50e3) * 0.6)),
          Math.max(0, finite(preset.maxDeployPowerW, 50e3))
        )
        : 0,
      maxRegenPowerW: enabled ? Math.max(0, finite(preset.maxRegenPowerW, 200e3)) : 0,
      // Lift harvest is deliberately lower than brake regen.  It still enters
      // the rear axle as negative torque, so recovered energy is paid for by
      // a measurable deceleration rather than being granted for free.
      maxLiftRegenPowerW: enabled
        ? Math.min(
          Math.max(0, finite(preset.maxLiftRegenPowerW, finite(preset.maxRegenPowerW, 200e3) * 0.22)),
          Math.max(0, finite(preset.maxRegenPowerW, 200e3))
        )
        : 0,
      liftRegenThrottleThreshold: enabled ? clamp(finite(preset.liftRegenThrottleThreshold, 0.035), 0, 0.2) : 0,
      deployEfficiency: enabled ? clamp(finite(preset.deployEfficiency, 0.91), 0.5, 1) : 1,
      regenEfficiency: enabled ? clamp(finite(preset.regenEfficiency, 0.72), 0.3, 1) : 1,
      maxDeployTorqueNm: enabled ? Math.max(0, finite(preset.maxDeployTorqueNm, 950)) : 0,
      maxRegenTorqueNm: enabled ? Math.max(0, finite(preset.maxRegenTorqueNm, 3000)) : 0,
      maxLiftRegenTorqueNm: enabled
        ? Math.min(
          Math.max(0, finite(preset.maxLiftRegenTorqueNm, finite(preset.maxRegenTorqueNm, 3000) * 0.34)),
          Math.max(0, finite(preset.maxRegenTorqueNm, 3000))
        )
        : 0,
      minDeploySpeed: enabled ? Math.max(0.5, finite(preset.minDeploySpeed, 4.5)) : Infinity,
      maxDeploySpeed: enabled ? Math.max(10, finite(preset.maxDeploySpeed, 94)) : 0,
      deployPowerW: 0, deployMechanicalPowerW: 0, regenPowerW: 0, regenMechanicalPowerW: 0,
      regenElectricalPowerW: 0, driveTorqueNm: 0, regenTorqueNm: 0,
      rearAxleOmegaRadS: 0,
      state: enabled ? 'READY' : 'OFF',
      liftHarvest: false
    };
  }

  setERSMode(mode) {
    if (!this.ers?.enabled) { this.ers.mode = 'OFF'; return this.ers.mode; }
    const normalized = String(mode ?? '').toUpperCase();
    this.ers.mode = ERS_MODES.includes(normalized) ? normalized : this.ers.mode;
    return this.ers.mode;
  }

  cycleERSMode() {
    if (!this.ers?.enabled) return 'OFF';
    const current = Math.max(0, ERS_MODES.indexOf(this.ers.mode));
    this.ers.mode = ERS_MODES[(current + 1) % ERS_MODES.length];
    return this.ers.mode;
  }

  // Short aliases keep the action boundary convenient for tests and future
  // controllers while the explicit names remain the documented API.
  setERS(mode) { return this.setERSMode(mode); }
  cycleERS() { return this.cycleERSMode(); }

  _updateERS(dt, forwardSpeed, throttle, brake, driven) {
    const ers = this.ers;
    ers.energyJ = clamp(finite(ers.energyJ, 0), 0, Math.max(0, ers.capacityJ));
    ers.soc = ers.capacityJ > 0 ? clamp(ers.energyJ / ers.capacityJ, 0, 1) : 0;
    ers.deployPowerW = 0;
    ers.deployMechanicalPowerW = 0;
    ers.regenPowerW = 0;
    ers.regenMechanicalPowerW = 0;
    ers.regenElectricalPowerW = 0;
    ers.driveTorqueNm = 0;
    ers.regenTorqueNm = 0;
    ers.liftHarvest = false;
    // ERS torque is defined at the rear axle.  The wheel angular speed is the
    // only valid divisor for a power-to-torque conversion; vehicle speed would
    // produce a force in newtons and overdrive the rear tyres by 1/radius.
    const rearWheelOmega = driven.includes(2) && driven.includes(3)
      ? (Math.abs(finite(this.wheels[2]?.omega)) + Math.abs(finite(this.wheels[3]?.omega))) * 0.5
      : 0;
    const omegaFloor = ers.enabled
      ? Math.max(1, finite(ers.minDeploySpeed, 4.5) / Math.max(0.05, this.wheelRadius))
      : 0;
    const rearAxleOmega = Math.max(omegaFloor, rearWheelOmega);
    ers.rearAxleOmegaRadS = rearAxleOmega;
    if (!ers.enabled || ers.mode === 'OFF' || this.spec.drive !== 'rear' || !driven.includes(2) || !driven.includes(3)) {
      ers.state = ers.enabled ? 'OFF' : 'OFF';
      return;
    }
    const speed = Math.abs(finite(forwardSpeed));
    const batteryAvailable = Math.max(0, ers.energyJ - ers.capacityJ * ers.minSoc);
    const batteryRoom = Math.max(0, ers.capacityJ - ers.energyJ);
    const braking = brake > 0.035;
    const liftThreshold = clamp(finite(ers.liftRegenThrottleThreshold, 0.12), 0, 0.2);
    const liftDemand = clamp((liftThreshold - throttle) / Math.max(0.001, liftThreshold), 0, 1);
    const canDeploy = !braking && throttle > liftThreshold && speed >= ers.minDeploySpeed && speed <= ers.maxDeploySpeed && batteryAvailable > 0.5;
    if (canDeploy) {
      // AUTO keeps the rear axle predictable during a corner; ATTACK is the
      // driver's explicit override and remains available at full request.
      const cornerReserve = clamp(1 - Math.abs(this.steering) * 10, 0.22, 1);
      const deploymentCeiling = ers.mode === 'ATTACK'
        ? ers.maxDeployPowerW
        : Math.min(ers.maxDeployPowerW, Math.max(0, finite(ers.autoDeployPowerW, ers.maxDeployPowerW * 0.38)));
      const modeDemand = ers.mode === 'ATTACK' ? 1 : clamp((0.2 + throttle * 0.8) * cornerReserve, 0, 1);
      let requestedElectricalPower = deploymentCeiling * modeDemand;
      requestedElectricalPower *= clamp(batteryAvailable / Math.max(1, ers.capacityJ * 0.08), 0, 1);
      requestedElectricalPower = clamp(requestedElectricalPower, 0, ers.maxDeployPowerW);
      const requestedMechanicalPower = requestedElectricalPower * ers.deployEfficiency;
      const requestedTorque = requestedMechanicalPower / rearAxleOmega;
      const axleTorque = clamp(requestedTorque, 0, ers.maxDeployTorqueNm);
      const actualMechanicalPower = axleTorque * rearAxleOmega;
      const actualElectricalPower = Math.min(
        requestedElectricalPower,
        actualMechanicalPower / Math.max(0.5, ers.deployEfficiency)
      );
      const energyDraw = Math.min(batteryAvailable, actualElectricalPower * dt);
      const powerScale = actualElectricalPower > 1e-6
        ? clamp(energyDraw / Math.max(1e-9, actualElectricalPower * dt), 0, 1)
        : 0;
      ers.driveTorqueNm = axleTorque * powerScale;
      ers.deployMechanicalPowerW = ers.driveTorqueNm * rearAxleOmega;
      ers.deployPowerW = ers.deployMechanicalPowerW / Math.max(0.5, ers.deployEfficiency);
      ers.energyJ -= ers.deployPowerW * dt;
    }

    // Regen is available under braking and as a light AUTO lift harvest.  It
    // is represented as rear-axle negative torque; Vehicle.step subtracts an
    // equal amount from friction-brake torque, preventing double braking.
    const liftHarvest = !canDeploy && !braking && (ers.mode === 'AUTO' || ers.mode === 'ATTACK') && liftDemand > 0;
    const canRegen = !canDeploy && speed > 2.5 && batteryRoom > 0.5 && (braking || liftHarvest);
    if (canRegen) {
      const demand = braking ? clamp(brake, 0, 1) : liftDemand;
      const requestedMechanicalPower = braking
        ? Math.min(ers.maxRegenPowerW, ers.maxRegenPowerW * demand)
        : Math.min(ers.maxRegenPowerW, Math.max(0, finite(ers.maxLiftRegenPowerW, ers.maxRegenPowerW * 0.22)) * demand);
      const requestedRearBrakeTorque = brake * this.spec.brakeTorqueNm * (1 - this.electronics.brakeBias);
      const regenTorqueLimit = braking
        ? Math.min(ers.maxRegenTorqueNm, Math.max(0, requestedRearBrakeTorque))
        : Math.min(ers.maxRegenTorqueNm, Math.max(0, finite(ers.maxLiftRegenTorqueNm, ers.maxRegenTorqueNm * 0.34)));
      const requestedTorque = requestedMechanicalPower / rearAxleOmega;
      const axleTorque = clamp(requestedTorque, 0, regenTorqueLimit);
      const actualMechanicalPower = Math.min(ers.maxRegenPowerW, axleTorque * rearAxleOmega);
      const actualElectricalPower = actualMechanicalPower * ers.regenEfficiency;
      const energyGain = Math.min(batteryRoom, actualElectricalPower * dt);
      const powerScale = actualElectricalPower > 1e-6
        ? clamp(energyGain / Math.max(1e-9, actualElectricalPower * dt), 0, 1)
        : 0;
      ers.regenTorqueNm = axleTorque * powerScale;
      ers.regenMechanicalPowerW = ers.regenTorqueNm * rearAxleOmega;
      ers.regenPowerW = ers.regenMechanicalPowerW;
      ers.regenElectricalPowerW = ers.regenMechanicalPowerW * ers.regenEfficiency;
      ers.energyJ += ers.regenElectricalPowerW * dt;
      ers.liftHarvest = liftHarvest && ers.regenMechanicalPowerW > 1;
    }
    ers.energyJ = clamp(finite(ers.energyJ), 0, ers.capacityJ);
    ers.soc = ers.capacityJ > 0 ? clamp(ers.energyJ / ers.capacityJ, 0, 1) : 0;
    ers.state = ers.deployPowerW > 20
      ? 'DEPLOY'
      : ers.liftHarvest
        ? 'LIFT_REGEN'
        : ers.regenPowerW > 20 ? 'REGEN' : 'READY';
  }

  _automaticGear(forwardSpeed, throttle) {
    if (this.transmissionMode !== 'automatic') return;
    if (this.shiftTimer > 0) return;
    const ratios = this.spec.gearRatios;
    if (this.rpm > this.spec.limiterRpm - 120 && this.gear < ratios.length - 1 && forwardSpeed > 8) {
      this.gear += 1;
      this.shiftTimer = 0.09;
    } else if (this.rpm < 2050 && this.gear > 1 && forwardSpeed > 12) {
      this.gear -= 1;
      this.shiftTimer = 0.075;
      this.autoBlip = 1;
    }
    if (forwardSpeed < -1.8 && throttle < 0.1) this.gear = 1;
  }

  toggleTransmissionMode() {
    this.transmissionMode = this.transmissionMode === 'manual' ? 'automatic' : 'manual';
    this.lastShiftRejected = null;
    return this.transmissionMode;
  }

  requestShift(direction) {
    if (this.transmissionMode !== 'manual' || this.shiftTimer > 0) return false;
    const nextGear = clamp(this.gear + Math.sign(finite(direction)), 1, this.spec.gearRatios.length - 1);
    if (nextGear === this.gear) return false;
    const roadOmega = Math.abs(this.localVelocity?.z ?? this.speed) / this.wheelRadius;
    const predictedRpm = roadOmega * this.spec.gearRatios[nextGear] * this.spec.finalDrive * 9.5493;
    if (nextGear < this.gear && predictedRpm > this.spec.limiterRpm + 120) {
      this.lastShiftRejected = 'OVERREV';
      return false;
    }
    const previousGear = this.gear;
    this.gear = nextGear;
    this.shiftTimer = nextGear > previousGear ? 0.09 : 0.075;
    if (direction < 0) this.autoBlip = 1;
    this.lastShiftRejected = null;
    return true;
  }

  _drivenIndices() {
    if (this.spec.drive === 'front') return [0, 1];
    if (this.spec.drive === 'all') return [0, 1, 2, 3];
    return [2, 3];
  }

  ackermannSteer(centerSteer = this.steering, wheelX = 0) {
    return ackermannSteerAngle(centerSteer, wheelX, this.wheelBase);
  }

  _powertrain(dt, forwardSpeed, throttle) {
    const driven = this._drivenIndices();
    const axleOmega = driven.reduce((sum, index) => sum + Math.abs(this.wheels[index].omega), 0) / driven.length;
    const ratio = this.spec.gearRatios[this.gear] * this.spec.finalDrive;
    // Clutch coupling rejects wheel-spin RPM spikes; engine speed follows road
    // speed once coupled, then shifts one ratio at a time.
    const roadOmega = Math.abs(forwardSpeed) / this.wheelRadius;
    const coupledOmega = Math.min(axleOmega, Math.max(26, roadOmega * 1.32 + 8));
    const drivenRpm = coupledOmega * ratio * 9.5493;
    const throttleFloor = this.spec.idleRpm + throttle * 980;
    const targetRpm = clamp(Math.max(throttleFloor, drivenRpm), this.spec.idleRpm, this.spec.limiterRpm + 420);
    this.rpm = damp(this.rpm, targetRpm, this.shiftTimer > 0 ? 5 : 15, dt);
    this._automaticGear(forwardSpeed, throttle);
    const limiter = this.rpm > this.spec.limiterRpm ? clamp((this.spec.limiterRpm + 260 - this.rpm) / 260, 0, 1) : 1;
    const band = clamp((this.rpm - 1500) / Math.max(1000, this.spec.torquePeakRpm - 1500), 0, 1);
    const torqueCurve = 0.7 + 0.3 * Math.sin(band * Math.PI);
    this.engineTorque = throttle * this.spec.maxTorqueNm * torqueCurve * limiter;
    const coastScale = this.spec.handling?.coastEngineBrakeScale ?? 1;
    this.engineBrakeTorque = throttle < 0.07 && forwardSpeed > 3
      ? this.spec.engineBrakeNm * coastScale * clamp((this.rpm - this.spec.idleRpm) / 4000, 0.2, 1)
      : 0;
    this.engineOmega = this.rpm * TAU / 60;
    this.shiftTimer = Math.max(0, this.shiftTimer - dt);
    this.autoBlip = Math.max(0, this.autoBlip - dt * 6);
    const shiftTorqueFactor = this.shiftTimer > 0 ? 0.12 : 1;
    const transmissionTorque = (this.engineTorque - this.engineBrakeTorque) * this.spec.gearRatios[this.gear]
      * this.spec.finalDrive * this.spec.drivetrainEfficiency * shiftTorqueFactor;
    return { driven, transmissionTorque };
  }

  _driveTorques(driven, transmissionTorque, throttle) {
    const torques = [0, 0, 0, 0];
    if (driven.length === 4) {
      const front = this._axleDiffTorque(0, 1, transmissionTorque * 0.45, throttle);
      const rear = this._axleDiffTorque(2, 3, transmissionTorque * 0.55, throttle);
      torques[0] = front[0]; torques[1] = front[1]; torques[2] = rear[0]; torques[3] = rear[1];
    } else {
      const pair = this._axleDiffTorque(driven[0], driven[1], transmissionTorque, throttle);
      torques[driven[0]] = pair[0];
      torques[driven[1]] = pair[1];
    }
    return torques;
  }

  _axleDiffTorque(leftIndex, rightIndex, totalTorque, throttle) {
    const left = this.wheels[leftIndex];
    const right = this.wheels[rightIndex];
    const speedDelta = left.omega - right.omega;
    const ramp = Math.tan((throttle > 0.08 ? this.spec.lsd.driveRampDeg : this.spec.lsd.coastRampDeg) * Math.PI / 180);
    const locking = this.spec.lsd.preloadNm + Math.abs(totalTorque) * clamp(ramp * 0.38, 0.05, 0.85);
    const transfer = clamp(speedDelta * 9.5, -locking, locking);
    return [totalTorque * 0.5 - transfer, totalTorque * 0.5 + transfer];
  }

  _aeroForces(forwardSpeed) {
    const aero = this.spec.aero;
    const speed2 = forwardSpeed * forwardSpeed;
    const ride = clamp(this.rideHeight, 0.016, 0.18);
    const design = aero.designRideHeight;
    let groundEffect;
    if (Number.isFinite(aero.lowRideOnset) && Number.isFinite(aero.highRideOnset)) {
      const smoothstep = (value) => value * value * (3 - 2 * value);
      const lowOnset = Math.min(design, aero.lowRideOnset);
      const stallHeight = Math.min(lowOnset - 0.002, finite(aero.stallHeight, design * 0.4));
      const highOnset = Math.max(design, aero.highRideOnset);
      const lowWindow = smoothstep(clamp((ride - stallHeight) / Math.max(0.006, lowOnset - stallHeight), 0, 1));
      const highWindow = smoothstep(clamp((ride - highOnset) / Math.max(0.035, design * 1.6), 0, 1));
      const stallFloor = clamp(finite(aero.stallFloor, 0.5), 0.25, 0.9);
      const highRideFloor = clamp(finite(aero.highRideFloor, 0.7), 0.4, 0.95);
      const lowEfficiency = stallFloor + (1 - stallFloor) * lowWindow;
      const highEfficiency = 1 - (1 - highRideFloor) * highWindow;
      groundEffect = ride < lowOnset ? lowEfficiency : highEfficiency;
    } else {
      const groundWindow = clamp(1 - Math.abs(ride - design) / Math.max(0.028, design * 1.35), 0.28, 1);
      const stall = ride < aero.stallHeight
        ? clamp(0.42 + ride / Math.max(0.001, aero.stallHeight) * 0.58, 0.28, 1)
        : 1;
      groundEffect = groundWindow * stall;
    }
    const dynamicPressure = 0.5 * 1.225 * speed2 * aero.area;
    const frontGroundShare = clamp(finite(aero.frontGroundShare, 0.42), 0.35, 0.55);
    const cleanFront = dynamicPressure * (aero.frontCl + aero.groundEffect * frontGroundShare * groundEffect);
    const cleanRear = dynamicPressure * (aero.rearCl + aero.groundEffect * (1 - frontGroundShare) * groundEffect);
    const cleanDrag = dynamicPressure * aero.cd;
    // Wake state is written by the simulation interaction pass before the
    // vehicle step.  Sanitise it at this boundary so a malformed external
    // controller can never inject non-finite force or an unbounded gain.
    const wakeStrength = clamp(finite(this.wake?.strength ?? this.wake?.wakeStrength, 0), 0, 1);
    const dragReductionInput = finite(this.wake?.dragReduction, 0);
    const frontLossInput = finite(this.wake?.frontDownforceLoss, 0);
    const rearLossInput = finite(this.wake?.rearDownforceLoss, 0);
    const dragReduction = clamp(
      Math.abs(dragReductionInput) > 1e-9 ? dragReductionInput : 1 - finite(this.wake?.dragMultiplier, 1),
      0, 0.18
    );
    const frontLoss = clamp(
      Math.abs(frontLossInput) > 1e-9 ? frontLossInput : 1 - finite(this.wake?.frontDownforceMultiplier, 1),
      0, 0.38
    );
    const rearLoss = clamp(
      Math.abs(rearLossInput) > 1e-9 ? rearLossInput : 1 - finite(this.wake?.rearDownforceMultiplier, 1),
      0, 0.28
    );
    const front = cleanFront * (1 - frontLoss);
    const rear = cleanRear * (1 - rearLoss);
    const drag = cleanDrag * (1 - dragReduction);
    this.aero.downforceN = front + rear;
    this.aero.frontN = front;
    this.aero.rearN = rear;
    this.aero.dragN = drag;
    this.aero.balance = front / Math.max(1, front + rear);
    this.aero.groundEffect = groundEffect;
    this.aero.wakeStrength = wakeStrength;
    this.aero.dragReduction = dragReduction;
    this.aero.frontDownforceLoss = frontLoss;
    this.aero.rearDownforceLoss = rearLoss;
    this.aero.wakeSource = this.wake?.sourceId ?? null;
    this.aero.dragMultiplier = 1 - dragReduction;
    this.aero.frontDownforceMultiplier = 1 - frontLoss;
    this.aero.rearDownforceMultiplier = 1 - rearLoss;
    this.aero.wakeDragMultiplier = 1 - dragReduction;
    this.aero.wakeFrontDownforceMultiplier = 1 - frontLoss;
    this.aero.wakeRearDownforceMultiplier = 1 - rearLoss;
    this.telemetry.wakeStrength = wakeStrength;
    this.telemetry.dragReduction = dragReduction;
    this.telemetry.frontDownforceLoss = frontLoss;
    this.telemetry.rearDownforceLoss = rearLoss;
    this.telemetry.wakeSource = this.wake?.sourceId ?? null;
    this.telemetry.dragMultiplier = 1 - dragReduction;
    this.telemetry.frontDownforceMultiplier = 1 - frontLoss;
    this.telemetry.rearDownforceMultiplier = 1 - rearLoss;
    this.telemetry.wakeDragMultiplier = 1 - dragReduction;
    this.telemetry.wakeFrontDownforceMultiplier = 1 - frontLoss;
    this.telemetry.wakeRearDownforceMultiplier = 1 - rearLoss;
    return { front, rear, drag };
  }

  _normalLoads(aero) {
    const loads = new Array(4);
    const longTransfer = clamp(-this.localAcceleration.z * this.mass * this.cgHeight / this.wheelBase, -3800, 3800);
    const latTransfer = clamp(this.localAcceleration.x * this.mass * this.cgHeight / this.trackWidth, -3300, 3300);
    for (let i = 0; i < 4; i += 1) {
      const wheel = this.wheels[i];
      const frontShare = wheel.front ? this.spec.weightFront : 1 - this.spec.weightFront;
      const staticLoad = this.mass * G * frontShare / 2;
      const longitudinal = wheel.front ? longTransfer * 0.5 : -longTransfer * 0.5;
      const lateral = (wheel.x < 0 ? 1 : -1) * latTransfer * 0.5;
      const aeroLoad = (wheel.front ? aero.front : aero.rear) * 0.5;
      loads[i] = staticLoad + longitudinal + lateral + aeroLoad;
    }
    // Anti-roll bars move load from the compressed outside corner to its mate.
    for (const [a, b, rate] of [[0, 1, this.spec.arb.front], [2, 3, this.spec.arb.rear]]) {
      const transfer = clamp((this.wheels[a].compression - this.wheels[b].compression) * rate, -1200, 1200);
      loads[a] -= transfer;
      loads[b] += transfer;
    }
    return loads.map((load) => clamp(load, 120, this.mass * G * 1.65));
  }

  _updateSuspension(wheel, targetLoad, surface, dt) {
    const rate = wheel.front ? this.spec.springRate.front : this.spec.springRate.rear;
    const staticLoad = this.mass * G * (wheel.front ? this.spec.weightFront : 1 - this.spec.weightFront) / 2;
    const travel = this.spec.suspension.travel;
    let target = staticLoad / rate + (targetLoad - staticLoad) / rate;
    target = clamp(target, 0.012, travel);
    const previous = wheel.compression;
    const rawVelocity = (target - previous) / Math.max(dt, 0.001);
    const fast = Math.abs(rawVelocity) > 0.45;
    const damping = rawVelocity > 0
      ? (fast ? this.spec.suspension.fastBump : this.spec.suspension.slowBump)
      : (fast ? this.spec.suspension.fastRebound : this.spec.suspension.slowRebound);
    const lambda = clamp(rate / Math.max(600, damping * 7.4), 10, 34);
    wheel.compression = damp(previous, target, lambda, dt);
    if (wheel.compression > this.spec.suspension.bumpStopStart) {
      const excess = wheel.compression - this.spec.suspension.bumpStopStart;
      targetLoad += excess * this.spec.suspension.bumpStopRate;
    }
    wheel.suspensionVelocity = (wheel.compression - previous) / Math.max(dt, 0.001);
    wheel.suspensionForce = targetLoad;
    wheel.normalLoad = clamp(targetLoad, 0, this.mass * G * 1.7);
    wheel.contactHeight = surface.height;
    wheel.camber = clamp(-0.028 - wheel.compression * this.spec.suspension.camberGain, -0.16, 0.03);
  }

  step(dt, track, canDrive = true) {
    const safeDt = clamp(finite(dt, 1 / 120), 1 / 1000, 1 / 20);
    if (this.despawned) {
      this.velocity.x = this.velocity.y = this.velocity.z = 0;
      this.speed = 0;
      return;
    }
    const control = this.controls;
    const handling = this.spec.handling ?? {};
    const throttle = canDrive && (!this.finished || this.cooldownActive) ? clamp(control.throttle, 0, 1) : 0;
    const rawBrakeTarget = canDrive ? clamp(control.brake, 0, 1) : 0.48;
    const brakeBlendInput = clamp(rawBrakeTarget / 0.35, 0, 1);
    const brakeBlend = brakeBlendInput * brakeBlendInput * (3 - 2 * brakeBlendInput);
    const brakeInitialGain = clamp(finite(handling.brakeInitialGain, 1), 0.5, 1);
    const brakeTarget = canDrive
      ? rawBrakeTarget * (brakeInitialGain + (1 - brakeInitialGain) * brakeBlend)
      : rawBrakeTarget;
    const brakeResponse = Math.max(0, finite(handling.brakePressureResponse, 0));
    this.brakePressure = canDrive && brakeResponse > 0
      ? damp(this.brakePressure, brakeTarget, brakeResponse, safeDt)
      : brakeTarget;
    const brake = this.brakePressure;
    const steerInput = clamp(control.steer, -1, 1);
    const handbrake = clamp(control.handbrake, 0, 1);
    this.steering = damp(this.steering, steerInput * this.spec.steeringLock, this.spec.steeringRate, safeDt);
    const previousVelocity = { x: this.velocity.x, y: this.velocity.y, z: this.velocity.z };
    const bodyVelocity = worldToLocal(this.velocity.x, this.velocity.z, this.yaw);
    this.localVelocity.x = bodyVelocity.x;
    this.localVelocity.z = bodyVelocity.z;
    this.speed = length2(this.velocity.x, this.velocity.z);
    const { driven, transmissionTorque } = this._powertrain(safeDt, bodyVelocity.z, throttle);
    const drivenSlip = driven.reduce((max, index) => Math.max(max, Math.max(0, this.wheels[index].slipRatio || 0)), 0);
    const tcLevel = this.electronics.tcLevel;
    const tcTarget = clamp(finite(handling.tcSlipTarget, 0.105) + (4 - tcLevel) * 0.012, 0.045, 0.22);
    const tcEngage = clamp(tcTarget, 0.045, 0.24);
    const tcRelease = tcEngage * 0.64;
    const tcFilterRate = this.speed > 3 ? 16 : 10;
    this.electronics.tcSlipFiltered += (drivenSlip - this.electronics.tcSlipFiltered)
      * (1 - Math.exp(-tcFilterRate * safeDt));
    if (tcLevel <= 0 || this.speed <= 2.5 || throttle < 0.08) {
      this.electronics.tcLatched = false;
    } else if (!this.electronics.tcLatched && this.electronics.tcSlipFiltered > tcEngage) {
      this.electronics.tcLatched = true;
    } else if (this.electronics.tcLatched && this.electronics.tcSlipFiltered < tcRelease) {
      this.electronics.tcLatched = false;
    }
    const interventionRequest = this.electronics.tcLatched
      ? clamp((this.electronics.tcSlipFiltered - tcRelease) / Math.max(0.045, 0.22 - tcRelease), 0, 1)
      : 0;
    this.electronics.tcIntervention += (interventionRequest - this.electronics.tcIntervention)
      * (1 - Math.exp(-((interventionRequest > this.electronics.tcIntervention) ? 19 : 10) * safeDt));
    const maxReduction = clamp(finite(handling.tcMaxTorqueReduction, 0.56), 0.2, 0.82);
    const tractionScale = tcLevel > 0 ? clamp(1 - this.electronics.tcIntervention * maxReduction, 0.18, 1) : 1;
    this.electronics.tcActivity = tcLevel > 0
      ? this.electronics.tcIntervention
      : 0;
    const driveTorques = this._driveTorques(driven, transmissionTorque * tractionScale, throttle);
    this._updateERS(safeDt, bodyVelocity.z, throttle, brake, driven);
    if (this.ers.enabled && this.ers.mode !== 'OFF' && this.spec.drive === 'rear') {
      const perWheelDeploy = this.ers.driveTorqueNm * 0.5;
      const perWheelRegen = this.ers.regenTorqueNm * 0.5;
      driveTorques[2] += perWheelDeploy - perWheelRegen;
      driveTorques[3] += perWheelDeploy - perWheelRegen;
    }
    const aero = this._aeroForces(Math.max(0, bodyVelocity.z));
    const normalLoads = this._normalLoads(aero);
    let totalFx = 0;
    let totalFz = 0;
    let totalTorque = 0;
    let averageCompression = 0;
    let averageUtilisation = 0;
    let absDemand = 0;

    for (let index = 0; index < this.wheels.length; index += 1) {
      const wheel = this.wheels[index];
      const wheelWorld = localToWorld(wheel.x, wheel.z, this.yaw);
      const wheelSurface = track.surfaceAt(this.position.x + wheelWorld.x, this.position.z + wheelWorld.z);
      this._updateSuspension(wheel, normalLoads[index], wheelSurface, safeDt);
      const wheelVelocityX = bodyVelocity.x + this.yawRate * wheel.z;
      const wheelVelocityZ = bodyVelocity.z - this.yawRate * wheel.x;
      const steer = wheel.front ? this.ackermannSteer(this.steering, wheel.x) : 0;
      const tireForwardX = Math.sin(steer);
      const tireForwardZ = Math.cos(steer);
      const tireRightX = Math.cos(steer);
      const tireRightZ = -Math.sin(steer);
      const longitudinalVelocity = wheelVelocityX * tireForwardX + wheelVelocityZ * tireForwardZ;
      const lateralVelocity = wheelVelocityX * tireRightX + wheelVelocityZ * tireRightZ;
      const axle = wheel.front ? 'front' : 'rear';
      const axleGrip = handling.axleGrip?.[axle] ?? 1;
      const axleStiffness = handling.axleStiffness?.[axle] ?? 1;
      const longitudinalStiffness = handling.longitudinalStiffness?.[axle] ?? 1;
      const force = tireForces({
        longitudinalVelocity, lateralVelocity, wheelAngularSpeed: wheel.omega,
        radius: this.wheelRadius, normalLoad: wheel.normalLoad, grip: wheelSurface.grip * axleGrip,
        corneringStiffness: axleStiffness, longitudinalStiffness,
        camber: wheel.camber, spec: this.spec.tire
      }, wheel.tyre, safeDt);
      const fx = force.fx * tireForwardX + force.fy * tireRightX;
      const fz = force.fx * tireForwardZ + force.fy * tireRightZ;
      totalFx += fx;
      totalFz += fz;
      totalTorque += wheel.z * fx - wheel.x * fz + force.mz * 0.24;
      const axleBias = wheel.front ? this.electronics.brakeBias : 1 - this.electronics.brakeBias;
      let wheelBrake = brake * this.spec.brakeTorqueNm * axleBias * 0.5;
      if (!wheel.front && this.ers.enabled && this.ers.regenTorqueNm > 0) {
        // Regen torque already enters through driveTorques as a negative rear
        // axle torque.  Remove the equivalent friction demand so braking
        // remains the driver's requested total rather than being doubled.
        wheelBrake = Math.max(0, wheelBrake - this.ers.regenTorqueNm * 0.5);
      }
      const absThreshold = -0.085 - this.electronics.absLevel * 0.012;
      if (this.electronics.absLevel > 0 && brake > 0.05 && force.slipRatio < absThreshold && this.speed > 3) {
        const modulation = clamp((Math.abs(force.slipRatio) - Math.abs(absThreshold)) * (2.7 + this.electronics.absLevel * 0.15), 0, 0.88);
        wheelBrake *= 1 - modulation;
        absDemand = Math.max(absDemand, modulation);
      }
      if (!wheel.front) wheelBrake += handbrake * 3300;
      const brakeSign = signOr(wheel.omega || longitudinalVelocity, 1);
      const rolling = wheel.omega * (0.72 + wheel.normalLoad * 0.000045);
      wheel.omega += (driveTorques[index] - wheelBrake * brakeSign - force.fx * this.wheelRadius - rolling) / this.spec.wheelInertia * safeDt;
      wheel.omega = clamp(wheel.omega, -360, 360);
      wheel.steer = steer;
      wheel.slipRatio = force.slipRatio;
      wheel.slipAngle = force.slipAngle;
      wheel.slip = Math.max(Math.abs(force.slipRatio), Math.abs(force.slipAngle) * 2.4);
      wheel.utilisation = force.utilisation;
      wheel.temperature = force.temperature;
      wheel.temperatureInnerC = force.temperatureInnerC;
      wheel.temperatureMiddleC = force.temperatureMiddleC;
      wheel.temperatureOuterC = force.temperatureOuterC;
      wheel.carcassTemperatureC = force.carcassTemperatureC;
      wheel.pressurePa = force.pressurePa;
      wheel.wear = force.wear;
      averageCompression += wheel.compression;
      averageUtilisation += force.utilisation;
      // Rubber deposition receives the already-computed wheel load and a
      // bounded slip-energy proxy.  The three-argument Track API remains
      // valid for lightweight fixtures and older callers.
      const slipEnergy = Math.min(
        1.8e5,
        Math.abs(force.fx * force.slipRatio) + Math.abs(force.fy * Math.tan(force.slipAngle))
      );
      track.updateTirePass?.(wheelSurface, force.utilisation, safeDt, wheel.normalLoad, slipEnergy);
    }
    this.electronics.absActivity = this.electronics.absLevel > 0 ? Math.max(absDemand, Math.max(0, this.electronics.absActivity - safeDt * 4.2)) : 0;
    this.electronics.tcPhase = (this.electronics.tcPhase + safeDt * 100) % 1;
    this.electronics.absPhase = (this.electronics.absPhase + safeDt * 100) % 1;
    averageCompression /= this.wheels.length;
    const targetRideHeight = clamp(this.spec.aero.designRideHeight + 0.046 - averageCompression * 0.72, 0.018, 0.15);
    const aeroPlatformResponse = Math.max(0, finite(handling.aeroPlatformResponse, 0));
    this.rideHeight = aeroPlatformResponse > 0
      ? damp(this.rideHeight, targetRideHeight, aeroPlatformResponse, safeDt)
      : targetRideHeight;
    // Chassis drag/rolling terms stay separate from tyre forces for telemetry.
    const dragSign = signOr(bodyVelocity.z, 1);
    totalFz -= aero.drag * dragSign + bodyVelocity.z * 16;
    totalFx -= bodyVelocity.x * (10 + this.speed * 0.08);
    // Speed/aero-dependent yaw damping represents tyre self-aligning and
    // chassis stability.  It is bounded and class-calibrated; the actual yaw
    // still comes exclusively from wheel forces and moments below.
    const aeroLoadRatio = clamp(aero.front + aero.rear, 0, this.mass * G * 2.2) / Math.max(1, this.mass * G);
    const yawDampingScale = clamp(finite(handling.yawDamping, 1), 0.7, 5);
    const yawStability = clamp(finite(handling.yawStability, 1), 0.7, 2.8);
    const lowSpeedYawBoost = 1 + clamp(finite(handling.lowSpeedYawDampingBoost, 0), 0, 0.8)
      * (1 - clamp((this.speed - 8) / 30, 0, 1));
    const yawDamping = clamp(
      (420 + this.speed * 18) * yawDampingScale * lowSpeedYawBoost + aeroLoadRatio * 240 * yawStability,
      260, 3600
    );
    totalTorque -= this.yawRate * yawDamping;

    // -----------------------------------------------------------------------
    // Active Stability Control (ESC) & Anti-Oscillation Damper
    // -----------------------------------------------------------------------
    const escLevel = this.electronics.stabilityAssistLevel ?? 3;
    if (escLevel > 0 && Math.abs(bodyVelocity.z) > 3.5) {
      // Kinematic bicycle model reference yaw rate
      const targetYawRate = (bodyVelocity.z * Math.tan(this.steering)) / Math.max(1.0, this.wheelBase);
      const yawRateError = this.yawRate - targetYawRate;
      const bodySlip = Math.atan2(bodyVelocity.x, Math.max(2.0, bodyVelocity.z));

      // Dynamic ESC restoring moment proportional to yaw error and body slip
      const escGain = (escLevel * 620) * (1 + Math.abs(bodySlip) * 3.2);
      const escTorque = -yawRateError * escGain;
      const maxEscTorque = (3500 + this.speed * 40) * (escLevel / 3);
      totalTorque += clamp(escTorque, -maxEscTorque, maxEscTorque);

      // Oversteer catch assist: prevent high-speed pendulum breakaway
      if (Math.abs(bodySlip) > 0.08) {
        totalFx -= bodyVelocity.x * (140 * escLevel);
      }
    }

    const centreSurface = track.surfaceAt(this.position.x, this.position.z);

    // -----------------------------------------------------------------------
    // Track Edge Virtual Adhesion Cushion (Anti-Stepout Assist)
    // -----------------------------------------------------------------------
    const roadHalf = finite(track.roadHalfWidth, 7.6);
    const curbWidth = finite(track.curbWidth, 1.35);
    const carLateral = finite(centreSurface?.lateral, 0);
    const edgeDist = Math.abs(carLateral) - (roadHalf - 0.7);

    if (edgeDist > 0 && this.speed > 4.0) {
      const movingOutward = (carLateral * bodyVelocity.x > 0);
      const edgeFactor = clamp(edgeDist / (curbWidth + 0.8), 0, 1);
      
      // Inward restorative grip impulse to keep the car on the asphalt
      const restoreMagnitude = this.mass * G * 0.45 * edgeFactor;
      const inwardForce = -Math.sign(carLateral) * restoreMagnitude;
      totalFx += inwardForce * 0.5;
      
      // Inward yaw torque assistance to bring vehicle nose back towards road center
      const yawCentering = -Math.sign(carLateral) * (1200 * edgeFactor);
      totalTorque += yawCentering;
    }

    // Kerb & runoff surface grip protection: prevent sudden grip cliff falloff
    if (centreSurface.zone === 'kerb' || centreSurface.zone === 'runoff') {
      centreSurface.grip = Math.max(0.88, centreSurface.grip || 0.88);
    }

    const worldForce = localToWorld(totalFx, totalFz, this.yaw);
    // Gravity component along circuit grade, expressed in world-plan coordinates.
    worldForce.x -= centreSurface.tangent.x * this.mass * G * Math.sin(centreSurface.grade);
    worldForce.z -= centreSurface.tangent.z * this.mass * G * Math.sin(centreSurface.grade);
    this.velocity.x += worldForce.x / this.mass * safeDt;
    this.velocity.z += worldForce.z / this.mass * safeDt;
    this.yawRate += totalTorque / this.inertiaTensor.y * safeDt;
    // Keep a broad numerical guard for pathological impacts without masking
    // normal handling with a low artificial yaw-rate cap.
    this.yawRate = clamp(this.yawRate, -5.5, 5.5);
    this.yaw += this.yawRate * safeDt;
    this.position.x += this.velocity.x * safeDt;
    this.position.z += this.velocity.z * safeDt;
    const updatedSurface = track.surfaceAt(this.position.x, this.position.z);
    const targetY = updatedSurface.height + this.rideHeight;
    this.velocity.y += ((targetY - this.position.y) * 150 - this.velocity.y * 20) * safeDt;
    this.velocity.y = clamp(this.velocity.y, -8, 8);
    this.position.y += this.velocity.y * safeDt;
    if (Math.abs(targetY - this.position.y) > 0.28) this.position.y = targetY;
    this.acceleration.x = (this.velocity.x - previousVelocity.x) / safeDt;
    this.acceleration.y = (this.velocity.y - previousVelocity.y) / safeDt;
    this.acceleration.z = (this.velocity.z - previousVelocity.z) / safeDt;
    const updatedBodyAcceleration = worldToLocal(this.acceleration.x, this.acceleration.z, this.yaw);
    this.localAcceleration.x = updatedBodyAcceleration.x;
    this.localAcceleration.z = updatedBodyAcceleration.z;
    this.angularVelocity.y = this.yawRate;
    this.roadBank = finite(updatedSurface.bank);
    this.roadGrade = finite(updatedSurface.grade);
    const previousRoll = this.roll;
    const previousPitch = this.pitch;
    // Sprung attitude is relative to the road plane.  Positive lateral
    // acceleration loads the left/outside wheels in this +Z-forward frame, so
    // the body rolls positive (top moves toward local -X).  Braking compresses
    // the front and pitches positive/down; acceleration does the opposite.
    const leftCompression = this.wheels[0].compression + this.wheels[2].compression;
    const rightCompression = this.wheels[1].compression + this.wheels[3].compression;
    const suspensionRoll = (leftCompression - rightCompression) * 0.42;
    const suspensionPitch = (this.wheels[0].compression + this.wheels[1].compression
      - this.wheels[2].compression - this.wheels[3].compression) * 0.34;
    const rollTarget = clamp(this.localAcceleration.x * 0.0024 + suspensionRoll, -0.085, 0.085);
    const pitchTarget = clamp(-this.localAcceleration.z * 0.0018 + suspensionPitch, -0.07, 0.07);
    this.roll = damp(this.roll, rollTarget, 8, safeDt);
    this.pitch = damp(this.pitch, pitchTarget, 8, safeDt);
    this.angularVelocity.x = (this.pitch - previousPitch) / Math.max(safeDt, 0.001);
    this.angularVelocity.z = (this.roll - previousRoll) / Math.max(safeDt, 0.001);
    this.surface = updatedSurface;
    this.previousDistance = this.distance;
    this.distance = updatedSurface.s;
    this.speed = length2(this.velocity.x, this.velocity.z);
    this.telemetry.longitudinalG = this.localAcceleration.z / G;
    this.telemetry.lateralG = this.localAcceleration.x / G;
    this.telemetry.verticalG = this.acceleration.y / G;
    this.telemetry.gDotX = clamp(this.telemetry.lateralG, -2.5, 2.5);
    this.telemetry.gDotY = clamp(-this.telemetry.longitudinalG, -2.5, 2.5);
    this.telemetry.tyreUtilisation = averageUtilisation / this.wheels.length;
    this.telemetry.suspensionTravel = averageCompression / this.spec.suspension.travel;
    this._resolveBarrier(track);
    this._updateOrientation();
    this.impact = Math.max(0, this.impact - safeDt * 2.8);
  }

  _updateOrientation() {
    // Quaternion-like state exposed without importing rendering dependencies.
    const halfYaw = this.yaw * 0.5;
    const halfPitch = this.pitch * 0.5;
    const halfRoll = this.roll * 0.5;
    const cy = Math.cos(halfYaw); const sy = Math.sin(halfYaw);
    const cp = Math.cos(halfPitch); const sp = Math.sin(halfPitch);
    const cr = Math.cos(halfRoll); const sr = Math.sin(halfRoll);
    this.orientation.w = cr * cp * cy + sr * sp * sy;
    this.orientation.x = sr * cp * cy - cr * sp * sy;
    this.orientation.y = cr * sp * cy + sr * cp * sy;
    this.orientation.z = cr * cp * sy - sr * sp * cy;
  }

  _resolveBarrier(track) {
    const surface = this.surface;
    if (!surface || surface.barrierDepth <= 0) return;
    const side = signOr(surface.lateral, 1);
    const excess = surface.barrierDepth;
    this.position.x -= surface.normal.x * side * excess * 0.88;
    this.position.z -= surface.normal.z * side * excess * 0.88;
    const normalSpeed = this.velocity.x * surface.normal.x * side + this.velocity.z * surface.normal.z * side;
    if (normalSpeed > 0) {
      this.velocity.x -= surface.normal.x * side * normalSpeed * 1.55;
      this.velocity.z -= surface.normal.z * side * normalSpeed * 1.55;
      this.yawRate *= 0.72;
      this.damage = clamp(this.damage + normalSpeed * 0.004, 0, 1);
      this.impact = Math.max(this.impact, clamp(normalSpeed / 18, 0, 1));
    }
  }

  _collisionHalfExtents() {
    const collision = this.spec?.collision ?? {};
    const halfLength = Math.max(1.25, finite(
      collision.halfLengthM,
      finite(this.spec?.wheelBase, 2.7) * 0.5 + finite(collision.overhangM, 0.56)
    ));
    const halfWidth = Math.max(0.72, finite(
      collision.halfWidthM,
      finite(this.spec?.trackWidth, 1.6) * 0.5 + finite(collision.bodyMarginM, 0.15)
    ));
    // Expose the resolved dimensions for deterministic diagnostics and race
    // post-solver overlap checks without coupling callers to the preset shape.
    this.collisionHalfLength = halfLength;
    this.collisionHalfWidth = halfWidth;
    return { halfLength, halfWidth };
  }

  // SAT contact in the plan view.  The returned normal always points from
  // this vehicle towards `other`; all fields are plain finite values.
  obbContact(other) {
    if (!other || this.despawned || other.despawned || this.trafficGhost || other.trafficGhost) return null;
    const dx = finite(other.position?.x) - finite(this.position?.x);
    const dz = finite(other.position?.z) - finite(this.position?.z);
    const thisForward = this.forward;
    const thisRight = this.right;
    const otherForward = other.forward;
    const otherRight = other.right;
    const a = this._collisionHalfExtents();
    const b = other._collisionHalfExtents?.() ?? {
      halfLength: Math.max(1.25, finite(other.wheelBase, 2.7) * 0.5 + 0.56),
      halfWidth: Math.max(0.72, finite(other.trackWidth, 1.6) * 0.5 + 0.15)
    };
    const axes = [thisForward, thisRight, otherForward, otherRight];
    let minimum = Infinity;
    let normalX = 1;
    let normalZ = 0;
    let separation = 0;
    for (const axis of axes) {
      const aRadius = a.halfLength * Math.abs(thisForward.x * axis.x + thisForward.z * axis.z)
        + a.halfWidth * Math.abs(thisRight.x * axis.x + thisRight.z * axis.z);
      const bRadius = b.halfLength * Math.abs(otherForward.x * axis.x + otherForward.z * axis.z)
        + b.halfWidth * Math.abs(otherRight.x * axis.x + otherRight.z * axis.z);
      const axisSeparation = dx * axis.x + dz * axis.z;
      const penetration = aRadius + bRadius - Math.abs(axisSeparation);
      if (!(penetration > 0)) return null;
      if (penetration < minimum) {
        minimum = penetration;
        separation = axisSeparation;
        const direction = axisSeparation < -1e-8
          ? -1
          : axisSeparation > 1e-8
            ? 1
            : (String(this.id) <= String(other.id) ? 1 : -1);
        normalX = axis.x * direction;
        normalZ = axis.z * direction;
      }
    }
    if (!Number.isFinite(minimum) || !Number.isFinite(normalX) || !Number.isFinite(normalZ)) return null;
    const normalLength = Math.hypot(normalX, normalZ) || 1;
    normalX /= normalLength;
    normalZ /= normalLength;
    return {
      penetration: minimum,
      normalX, normalZ,
      tangentX: -normalZ, tangentZ: normalX,
      separation,
      distance: Math.hypot(dx, dz),
      halfLengthA: a.halfLength, halfWidthA: a.halfWidth,
      halfLengthB: b.halfLength, halfWidthB: b.halfWidth
    };
  }

  collisionContact(other) { return this.obbContact(other); }
  getCollisionContact(other) { return this.obbContact(other); }

  collide(other) {
    const contact = this.obbContact(other);
    if (!contact) return 0;
    // The positional solver leaves a tiny contact slop so touching cars do
    // not repeatedly receive an impulse on every resolver iteration.
    if (contact.penetration <= 0.012) return 0;
    const inverseMassA = 1 / Math.max(1, finite(this.mass, 1200));
    const inverseMassB = 1 / Math.max(1, finite(other.mass, 1200));
    const inverseMassTotal = inverseMassA + inverseMassB;
    const shareA = inverseMassA / inverseMassTotal;
    const shareB = inverseMassB / inverseMassTotal;
    const correction = Math.max(0, contact.penetration - 0.006);
    this.position.x -= contact.normalX * correction * shareA;
    this.position.z -= contact.normalZ * correction * shareA;
    other.position.x += contact.normalX * correction * shareB;
    other.position.z += contact.normalZ * correction * shareB;

    const relativeX = finite(other.velocity?.x) - finite(this.velocity?.x);
    const relativeZ = finite(other.velocity?.z) - finite(this.velocity?.z);
    const closing = relativeX * contact.normalX + relativeZ * contact.normalZ;
    // The scalar return remains the established velocity-impulse contract,
    // but is capped so a pathological overlap cannot launch a car.
    const impulse = closing < 0 ? clamp(-closing * 0.76, 0, 18) : 0;
    if (impulse > 0) {
      const deltaA = impulse * shareA;
      const deltaB = impulse * shareB;
      this.velocity.x -= contact.normalX * deltaA;
      this.velocity.z -= contact.normalZ * deltaA;
      other.velocity.x += contact.normalX * deltaB;
      other.velocity.z += contact.normalZ * deltaB;
      const yawA = clamp((this.right.x * contact.normalX + this.right.z * contact.normalZ) * impulse * 0.045, -0.32, 0.32);
      const yawB = clamp((other.right.x * contact.normalX + other.right.z * contact.normalZ) * impulse * 0.045, -0.32, 0.32);
      this.yawRate = clamp(this.yawRate + yawA, -5.5, 5.5);
      other.yawRate = clamp(other.yawRate - yawB, -5.5, 5.5);
      this.impact = Math.max(this.impact, clamp(impulse / 14, 0, 1));
      other.impact = Math.max(other.impact, clamp(impulse / 14, 0, 1));
    }
    // A small tangential damping term prevents side-by-side bodies from
    // skating through one another while leaving racing line speed intact.
    const tangentVelocity = relativeX * contact.tangentX + relativeZ * contact.tangentZ;
    const tangentImpulse = clamp(tangentVelocity * 0.022, -0.45, 0.45);
    this.velocity.x += contact.tangentX * tangentImpulse * shareA;
    this.velocity.z += contact.tangentZ * tangentImpulse * shareA;
    other.velocity.x -= contact.tangentX * tangentImpulse * shareB;
    other.velocity.z -= contact.tangentZ * tangentImpulse * shareB;
    return impulse;
  }
}
