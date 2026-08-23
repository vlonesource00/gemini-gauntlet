import * as THREE from 'three';
import { clamp } from '../core/math.js';

export class CameraRig {
  constructor(camera) {
    this.camera = camera;
    this.mode = 'CHASE'; // 'CHASE', 'PURSUIT', 'TACTICAL', 'COCKPIT', 'FREE'
    this.position = new THREE.Vector3(0, 5, -9);
    this.look = new THREE.Vector3(0, 0, 10);
    this.headInertia = new THREE.Vector3();
    this._targetPosition = new THREE.Vector3();
    this._targetLook = new THREE.Vector3();
    this._cockpitForward = new THREE.Vector3();
    this.standardMode = 'CHASE';
    this.freeYaw = 0;
    this.freePitch = 0;
    this._freeForward = new THREE.Vector3();
    this._freeRight = new THREE.Vector3();
    this._freeMove = new THREE.Vector3();
    this._scratchVecA = new THREE.Vector3();
    this._scratchVecB = new THREE.Vector3();
    this._scratchVecC = new THREE.Vector3();
    this._upAxis = new THREE.Vector3(0, 1, 0);
  }

  setMode(mode) {
    const validModes = ['CHASE', 'PURSUIT', 'TACTICAL', 'COCKPIT', 'FREE'];
    if (!validModes.includes(mode)) return this.mode;
    if (mode === 'FREE' && this.mode !== 'FREE') {
      const direction = this.camera.getWorldDirection(this._freeForward);
      this.freeYaw = Math.atan2(direction.x, direction.z);
      this.freePitch = Math.asin(clamp(direction.y, -1, 1));
      this.position.copy(this.camera.position);
    }
    if (mode !== 'FREE') {
      this.standardMode = mode;
    }
    this.mode = mode;
    return this.mode;
  }

  cycleMode() {
    const modes = ['CHASE', 'PURSUIT', 'TACTICAL', 'COCKPIT'];
    const currentIndex = modes.indexOf(this.mode);
    const nextMode = currentIndex >= 0 ? modes[(currentIndex + 1) % modes.length] : 'CHASE';
    return this.setMode(nextMode);
  }

  toggle() {
    if (this.mode === 'FREE' || this.mode === 'PURSUIT' || this.mode === 'TACTICAL') {
      return this.setMode(this.standardMode);
    }
    return this.setMode(this.mode === 'CHASE' ? 'COCKPIT' : 'CHASE');
  }

  setSpectate(enabled = true) {
    if (!enabled || this.mode === 'PURSUIT') return this.setMode(this.standardMode);
    return this.setMode('PURSUIT');
  }

  setFree(enabled = true) {
    if (!enabled || this.mode === 'FREE') return this.setMode(this.standardMode);
    return this.setMode('FREE');
  }

  updateFree(input = {}, dt = 0) {
    if (this.mode !== 'FREE') return;
    const safeDt = clamp(Number.isFinite(dt) ? dt : 0, 0, 0.1);
    this.freeYaw += clamp(input.yaw ?? 0, -1, 1) * 1.65 * safeDt;
    this.freePitch = clamp(this.freePitch + clamp(input.pitch ?? 0, -1, 1) * 1.35 * safeDt, -1.48, 1.48);
    const cosPitch = Math.cos(this.freePitch);
    this._freeForward.set(
      Math.sin(this.freeYaw) * cosPitch,
      Math.sin(this.freePitch),
      Math.cos(this.freeYaw) * cosPitch
    );
    this._freeRight.set(Math.cos(this.freeYaw), 0, -Math.sin(this.freeYaw));
    this._freeMove.set(0, 0, 0)
      .addScaledVector(this._freeForward, clamp(input.forward ?? 0, -1, 1))
      .addScaledVector(this._freeRight, clamp(input.right ?? 0, -1, 1));
    this._freeMove.y += clamp(input.up ?? 0, -1, 1);
    if (this._freeMove.lengthSq() > 1) this._freeMove.normalize();
    const speed = input.boost ? 92 : 24;
    this.position.addScaledVector(this._freeMove, speed * safeDt);
    this.look.copy(this.position).addScaledVector(this._freeForward, 30);
    this.camera.position.copy(this.position);
    this.camera.up.set(0, 1, 0);
    this.camera.lookAt(this.look);
  }

  update(vehicle, dt, cockpitPose = null, opponentVehicle = null) {
    if (this.mode === 'FREE') return;
    const safeDt = clamp(Number.isFinite(dt) ? dt : 0, 0, 0.1);
    const yaw = vehicle?.yaw ?? 0;
    const speed = Math.max(0, vehicle?.speed ?? 0);
    const vehicleY = vehicle?.position?.y ?? 0;
    const forwardX = Math.sin(yaw);
    const forwardZ = Math.cos(yaw);
    const rightX = Math.cos(yaw);
    const rightZ = -Math.sin(yaw);

    const localAcceleration = vehicle?.localAcceleration ?? { x: 0, z: 0 };
    const localVelocity = vehicle?.localVelocity ?? { x: 0, z: 0 };
    const lateralHead = clamp(-localAcceleration.x * 0.012, -0.085, 0.085);
    const longitudinalHead = clamp(-localAcceleration.z * 0.006, -0.045, 0.045);
    const verticalHead = clamp(-(vehicle?.acceleration?.y ?? 0) * 0.003, -0.035, 0.035);
    this._scratchVecA.set(lateralHead, verticalHead, longitudinalHead);
    this.headInertia.lerp(this._scratchVecA, 1 - Math.exp(-9 * safeDt));

    if (this.mode === 'CHASE') {
      const stretch = clamp(speed * 0.052, 0, 3.4);
      this._targetPosition.set(
        (vehicle?.position?.x ?? 0) - forwardX * (8.35 + stretch) + rightX * 0.35,
        vehicleY + 3.5 + Math.min(1.15, speed * 0.024),
        (vehicle?.position?.z ?? 0) - forwardZ * (8.35 + stretch) + rightZ * 0.35
      );
      this._targetLook.set(
        (vehicle?.position?.x ?? 0) + forwardX * (7.7 + stretch * 0.3),
        vehicleY + 0.92,
        (vehicle?.position?.z ?? 0) + forwardZ * (7.7 + stretch * 0.3)
      );
      this.camera.up.set(0, 1, 0);
    } else if (this.mode === 'PURSUIT') {
      // AI Pursuit Camera: follows behind whichever car is pursuing from behind, aiming ahead at the lead car
      const p1 = vehicle;
      const p2 = opponentVehicle || vehicle;
      const p1Dist = p1?.distance ?? 0;
      const p2Dist = p2?.distance ?? 0;
      const pursuer = (p2 && p2Dist < p1Dist) ? p2 : p1;
      const leader = (pursuer === p2) ? p1 : (p2 || p1);

      const pursuerYaw = pursuer?.yaw ?? yaw;
      const tForwardX = Math.sin(pursuerYaw);
      const tForwardZ = Math.cos(pursuerYaw);
      const tRightX = Math.cos(pursuerYaw);
      const tRightZ = -Math.sin(pursuerYaw);
      const stretch = clamp(Math.max(0, pursuer?.speed ?? 0) * 0.07, 0, 5.0);

      this._targetPosition.set(
        (pursuer?.position?.x ?? 0) - tForwardX * (10.5 + stretch) + tRightX * 3.2,
        (pursuer?.position?.y ?? 0) + 4.8 + Math.min(1.8, (pursuer?.speed ?? 0) * 0.025),
        (pursuer?.position?.z ?? 0) - tForwardZ * (10.5 + stretch) + tRightZ * 3.2
      );

      if (leader && leader !== pursuer) {
        this._targetLook.set(
          leader.position?.x ?? 0,
          (leader.position?.y ?? 0) + 0.9,
          leader.position?.z ?? 0
        );
      } else {
        this._targetLook.set(
          (pursuer?.position?.x ?? 0) + tForwardX * (14 + stretch * 0.4),
          (pursuer?.position?.y ?? 0) + 1.1,
          (pursuer?.position?.z ?? 0) + tForwardZ * (14 + stretch * 0.4)
        );
      }
      this.camera.up.set(0, 1, 0);
    } else if (this.mode === 'TACTICAL') {
      // Top-Down Tactical Cam: High-altitude combat view looking down at the duel
      const centerPos = this._scratchVecA.set(vehicle?.position?.x ?? 0, vehicleY, vehicle?.position?.z ?? 0);
      if (opponentVehicle) {
        this._scratchVecB.set(opponentVehicle.position?.x ?? 0, opponentVehicle.position?.y ?? 0, opponentVehicle.position?.z ?? 0);
        centerPos.add(this._scratchVecB).multiplyScalar(0.5);
      }
      const tacticalHeight = 28 + clamp(speed * 0.15, 0, 16);
      this._targetPosition.set(
        centerPos.x - forwardX * 6,
        centerPos.y + tacticalHeight,
        centerPos.z - forwardZ * 6
      );
      this._targetLook.set(
        centerPos.x + forwardX * 8,
        centerPos.y,
        centerPos.z + forwardZ * 8
      );
      this.camera.up.set(forwardX, 0.4, forwardZ).normalize();
    } else if (this.mode === 'COCKPIT' && cockpitPose?.position && cockpitPose?.forward) {
      this._targetPosition.copy(cockpitPose.position);
      this._scratchVecA.set(rightX, 0, rightZ);
      this._targetPosition.addScaledVector(this._scratchVecA, this.headInertia.x);
      this._targetPosition.y += this.headInertia.y;
      this._scratchVecB.set(forwardX, 0, forwardZ);
      this._targetPosition.addScaledVector(this._scratchVecB, this.headInertia.z);
      this._cockpitForward.copy(cockpitPose.forward);
      this._cockpitForward.y = 0;
      if (this._cockpitForward.lengthSq() < 0.0001) this._cockpitForward.set(forwardX, 0, forwardZ);
      else this._cockpitForward.normalize();

      const slip = Math.atan2(localVelocity.x ?? 0, Math.max(4, Math.abs(localVelocity.z ?? speed)));
      const apexBias = clamp((vehicle?.steering ?? 0) * 0.12 - slip * 0.18, -0.12, 0.12);
      this._cockpitForward.applyAxisAngle(this._upAxis, apexBias).normalize();
      this._targetLook.copy(this._targetPosition).addScaledVector(this._cockpitForward, 24);
      this._targetLook.y = this._targetPosition.y + 0.03;
      this.camera.up.set(0, 1, 0);
    } else {
      // Fallback Hood/Cockpit view
      this._targetPosition.set(
        (vehicle?.position?.x ?? 0) + forwardX * 0.12 - rightX * 0.25,
        vehicleY + 1.08,
        (vehicle?.position?.z ?? 0) + forwardZ * 0.12 - rightZ * 0.25
      );
      this._targetLook.set(
        (vehicle?.position?.x ?? 0) + forwardX * 18,
        vehicleY + 1.03,
        (vehicle?.position?.z ?? 0) + forwardZ * 18
      );
      this.camera.up.set(0, 1, 0);
    }

    const smoothingRate = this.mode === 'COCKPIT' ? 22 : this.mode === 'TACTICAL' ? 4.5 : 7.2;
    const smoothing = 1 - Math.exp(-smoothingRate * safeDt);
    this.position.lerp(this._targetPosition, smoothing);
    this.look.lerp(this._targetLook, smoothing);
    this.camera.position.copy(this.position);
    this.camera.lookAt(this.look);
  }
}
