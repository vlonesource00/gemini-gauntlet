import * as THREE from 'three';

const WHEEL_LABELS = ['FL', 'FR', 'RL', 'RR'];
const tireMaterial = new THREE.MeshStandardMaterial({ color: '#090d0b', roughness: 0.94, metalness: 0.01 });
const rimMaterial = new THREE.MeshStandardMaterial({ color: '#b0bcc2', roughness: 0.22, metalness: 0.92 });
const darkMaterial = new THREE.MeshStandardMaterial({ color: '#0d1316', roughness: 0.38, metalness: 0.65 });

function isGlass(material) {
  return material?.name?.startsWith('MAT_GLASS');
}

function shadow(object) {
  object.traverse((child) => {
    if (!child.isMesh) return;
    child.castShadow = true;
    child.receiveShadow = true;
  });
  return object;
}

export class CarVisual {
  constructor(vehicle, { variant = 'gt' } = {}) {
    this.vehicle = vehicle;
    this.variant = variant;
    this.group = new THREE.Group();
    this.group.name = 'CAR_VISUAL_' + (vehicle.id || 'anonymous');
    this.group.rotation.order = 'YXZ';
    this.rollCenterHeight = 0.33;
    this.chassisPivot = new THREE.Group();
    this.chassisPivot.name = 'CHASSIS_ATTITUDE_PIVOT';
    this.chassisPivot.position.y = this.rollCenterHeight;
    this.chassisPivot.rotation.order = 'YXZ';
    this.group.add(this.chassisPivot);

    this.chassis = new THREE.Group();
    this.chassis.name = 'SPRUNG_CHASSIS';
    this.chassis.position.y = -this.rollCenterHeight;
    this.chassisPivot.add(this.chassis);

    this.assetModel = null;
    this.assetWheels = [];
    this.cockpitAnchor = null;
    this.cockpitHiddenNodes = [];
    this.cockpitGlassBindings = [];
    this.cockpitActive = false;
    this._cockpitPose = { position: new THREE.Vector3(), forward: new THREE.Vector3() };

    this.fallback = new THREE.Group();
    this.chassis.add(this.fallback);
    this.fallbackWheels = this._createFallback();
  }

  _createFallback() {
    const carColor = this.vehicle.color || '#00f0ff';
    const paint = new THREE.MeshStandardMaterial({ color: carColor, roughness: 0.28, metalness: 0.55 });
    const glass = new THREE.MeshStandardMaterial({ color: '#041018', roughness: 0.12, metalness: 0.5, transparent: true, opacity: 0.82 });
    const lamp = new THREE.MeshStandardMaterial({ color: '#e8f8ff', emissive: '#00f0ff', emissiveIntensity: 1.2, roughness: 0.2 });

    const body = shadow(new THREE.Mesh(new THREE.BoxGeometry(1.84, 0.42, 4.1), paint));
    body.position.y = 0.55;
    this.fallback.add(body);

    const cabin = shadow(new THREE.Mesh(new THREE.BoxGeometry(1.42, 0.44, 1.58), glass));
    cabin.position.set(0, 0.94, -0.23);
    cabin.rotation.x = -0.13;
    this.fallback.add(cabin);

    const splitter = shadow(new THREE.Mesh(new THREE.BoxGeometry(1.92, 0.08, 0.58), darkMaterial));
    splitter.position.set(0, 0.31, 2.17);
    this.fallback.add(splitter);

    const wing = shadow(new THREE.Mesh(new THREE.BoxGeometry(1.74, 0.07, 0.3), darkMaterial));
    wing.position.set(0, 1.08, -2.02);
    this.fallback.add(wing);

    for (const x of [-0.55, 0.55]) {
      const headlamp = new THREE.Mesh(new THREE.BoxGeometry(0.38, 0.1, 0.08), lamp);
      headlamp.position.set(x, 0.64, 2.08);
      this.fallback.add(headlamp);
    }

    const wheels = this.vehicle.wheels || [
      { x: -0.85, z: 1.35, compression: 0, steer: 0, omega: 0 },
      { x: 0.85, z: 1.35, compression: 0, steer: 0, omega: 0 },
      { x: -0.85, z: -1.35, compression: 0, steer: 0, omega: 0 },
      { x: 0.85, z: -1.35, compression: 0, steer: 0, omega: 0 }
    ];

    return wheels.map((wheel) => {
      const pivot = new THREE.Group();
      pivot.position.set(wheel.x ?? 0, 0.34, wheel.z ?? 0);
      const rubber = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.335, 0.335, 0.245, 16), tireMaterial));
      rubber.geometry.rotateZ(Math.PI / 2);
      pivot.add(rubber);
      const rim = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.196, 0.196, 0.252, 12), rimMaterial));
      rim.geometry.rotateZ(Math.PI / 2);
      pivot.add(rim);
      this.group.add(pivot);
      return { pivot, rubber, rim };
    });
  }

  _bindAssetWheels(model) {
    return WHEEL_LABELS.map((label) => {
      const pivot = model.getObjectByName(label);
      const spin = model.getObjectByName(label + '_SPIN') || pivot;
      if (!pivot || !spin) return null;
      return {
        pivot,
        spin,
        baseY: pivot.position.y,
        baseYaw: pivot.rotation.y
      };
    });
  }

  _bindCockpit(model) {
    this.cockpitAnchor = model.getObjectByName('COCKPIT_CAMERA') || null;
    this.cockpitHiddenNodes = [];
    this.cockpitGlassBindings = [];
    model.traverse((object) => {
      if (object.name.startsWith('COCKPIT_HIDE')) this.cockpitHiddenNodes.push(object);
      if (!object.isMesh || !object.material) return;
      const materials = Array.isArray(object.material) ? object.material : [object.material];
      if (materials.some(isGlass)) this.cockpitGlassBindings.push({ mesh: object, original: object.material });
    });
  }

  attachAsset(assets) {
    if (this.assetModel) return true;
    const model = assets.cloneCar(this.variant, this.vehicle.color);
    if (!model) return false;
    const wheels = this._bindAssetWheels(model);
    if (wheels.some((wheel) => !wheel)) return false;
    shadow(model);
    this.assetModel = model;
    this.assetWheels = wheels;
    this._bindCockpit(model);
    this.chassis.add(model);

    this.group.updateWorldMatrix(true, true);
    model.updateWorldMatrix(true, true);
    wheels.forEach((wheel) => this.group.attach(wheel.pivot));
    this.fallback.visible = false;
    this.fallbackWheels.forEach((wheel) => { wheel.pivot.visible = false; });
    return true;
  }

  setCockpitView(active) {
    if (!this.vehicle.player || !this.assetModel) return false;
    const desired = Boolean(active);
    if (desired === this.cockpitActive) return Boolean(this.cockpitAnchor);
    this.cockpitActive = desired;
    this.cockpitHiddenNodes.forEach((node) => { node.visible = !desired; });
    this.cockpitGlassBindings.forEach((binding) => {
      if (!desired) {
        binding.mesh.material = binding.original;
        return;
      }
      const tune = (material) => {
        if (!isGlass(material)) return material;
        const cockpitGlass = material.clone();
        cockpitGlass.transparent = true;
        cockpitGlass.opacity = Math.min(material.opacity ?? 1, 0.12);
        cockpitGlass.depthWrite = false;
        cockpitGlass.side = THREE.DoubleSide;
        cockpitGlass.needsUpdate = true;
        return cockpitGlass;
      };
      binding.mesh.material = Array.isArray(binding.original) ? binding.original.map(tune) : tune(binding.original);
    });
    return Boolean(this.cockpitAnchor);
  }

  getCockpitPose() {
    if (!this.vehicle.player || !this.assetModel || !this.cockpitAnchor) return null;
    this.group.updateWorldMatrix(true, true);
    this.cockpitAnchor.getWorldPosition(this._cockpitPose.position);
    this.cockpitAnchor.getWorldDirection(this._cockpitPose.forward);
    return this._cockpitPose.forward.lengthSq() > 0.0001 ? this._cockpitPose : null;
  }

  update(dt) {
    const car = this.vehicle;
    if (!car) return;
    this.group.visible = !car.despawned;
    if (car.despawned) return;

    const datumHeight = car.spec?.aero?.designRideHeight != null
      ? car.spec.aero.designRideHeight + 0.006
      : (car.rideHeight ?? 0.068);
    const rootY = (car.position?.y ?? 0) - datumHeight + 0.04;
    const roadBank = Number.isFinite(car.roadBank) ? car.roadBank : (car.surface?.bank ?? 0);
    const roadGrade = Number.isFinite(car.roadGrade) ? car.roadGrade : (car.surface?.grade ?? 0);

    this.group.position.set(car.position?.x ?? 0, rootY, car.position?.z ?? 0);
    this.group.rotation.set(-roadGrade, car.yaw ?? 0, -roadBank);
    this.chassisPivot.rotation.set(car.pitch ?? 0, 0, car.roll ?? 0);
    this.chassis.rotation.set(0, 0, 0);

    const wheels = car.wheels || [];
    this.fallbackWheels.forEach((visual, index) => {
      const wheel = wheels[index];
      if (!wheel) return;
      visual.pivot.position.y = 0.34 - (wheel.compression ?? 0);
      visual.pivot.rotation.y = wheel.steer ?? 0;
      visual.rubber.rotation.x += (wheel.omega ?? 0) * dt;
      visual.rim.rotation.x += (wheel.omega ?? 0) * dt;
    });

    this.assetWheels.forEach((visual, index) => {
      const wheel = wheels[index];
      if (!wheel) return;
      visual.pivot.position.y = visual.baseY - (wheel.compression ?? 0);
      visual.pivot.rotation.y = visual.baseYaw + (wheel.steer ?? 0);
      visual.spin.rotation.x += (wheel.omega ?? 0) * dt;
    });
  }

  dispose() {
    if (this.group) {
      this.group.traverse((node) => {
        if (node.geometry) {
          node.geometry.dispose();
        }
        if (node.material) {
          if (Array.isArray(node.material)) {
            node.material.forEach((mat) => {
              if (mat.map) mat.map.dispose();
              mat.dispose();
            });
          } else {
            if (node.material.map) node.material.map.dispose();
            node.material.dispose();
          }
        }
      });
      this.group.removeFromParent();
    }
  }
}
