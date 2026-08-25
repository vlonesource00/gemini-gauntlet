import * as THREE from 'three';
import { HARBOR_RING } from '../scenarios/HarborRing.js';

const TAU = Math.PI * 2;

const hash01 = (seed) => {
  const value = Math.sin(seed * 12.9898 + 78.233) * 43758.5453;
  return value - Math.floor(value);
};

const mod1 = (value) => ((value % 1) + 1) % 1;

/**
 * Pure placement planning keeps the scenery deterministic.
 */
export function createCircuitPlacementPlan(scenario = HARBOR_RING) {
  const seed = scenario.scenery?.seed ?? 73073;
  const plan = {};
  for (const band of scenario.scenery?.bands ?? []) {
    const placements = [];
    for (let index = 0; index < band.count; index += 1) {
      const a = hash01(seed + index * 31 + band.type.length * 17);
      const b = hash01(seed + index * 59 + band.type.length * 29);
      const c = hash01(seed + index * 83 + band.type.length * 7);
      placements.push({
        type: band.type,
        fraction: mod1((index + 0.20 + a * 0.62) / band.count),
        lateral: (index % 2 ? -1 : 1) * (band.lateralMin + b * (band.lateralMax - band.lateralMin)),
        scale: band.scaleMin + c * (band.scaleMax - band.scaleMin),
        variation: a
      });
    }
    plan[band.type] = placements;
  }
  plan.landmarks = (scenario.landmarks ?? []).map((landmark, index) => ({ ...landmark, variation: hash01(seed + index * 97) }));
  plan.instanceCount = Object.values(plan).reduce((total, list) => total + (Array.isArray(list) ? list.length : 0), 0);
  plan.drawCallBudget = scenario.scenery?.drawCallBudget ?? 35;
  return plan;
}

function vectorFrom(value, fallback = new THREE.Vector3()) {
  if (!value) return fallback;
  return new THREE.Vector3(Number(value.x) || 0, Number(value.y) || 0, Number(value.z) || 0);
}

function trackSample(track, fraction, scenario) {
  const distance = fraction * Math.max(1, Number(track?.length) || 0);
  const raw = typeof track?.atDistance === 'function' ? track.atDistance(distance) : null;
  if (raw) {
    const position = vectorFrom(raw.position ?? raw.point ?? raw);
    const tangent = vectorFrom(raw.tangent ?? raw.forward, new THREE.Vector3(0, 0, 1)).setY(0);
    if (tangent.lengthSq() < 1e-6) tangent.set(0, 0, 1); else tangent.normalize();
    return { position, tangent };
  }
  return {
    position: new THREE.Vector3(),
    tangent: new THREE.Vector3(0, 0, 1)
  };
}

function placementTransform(track, placement, scenario, yOffset = 0) {
  const sample = trackSample(track, placement.fraction ?? 0, scenario);
  const normal = new THREE.Vector3(-sample.tangent.z, 0, sample.tangent.x);
  const position = sample.position.addScaledVector(normal, placement.lateral ?? 0);
  position.y += yOffset;
  return {
    position,
    heading: Math.atan2(sample.tangent.x, sample.tangent.z),
    scale: placement.scale ?? 1
  };
}

function makeMatrix(transform, scale = 1, yScale = scale, headingOffset = 0) {
  const quaternion = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), transform.heading + headingOffset);
  return new THREE.Matrix4().compose(transform.position, quaternion, new THREE.Vector3(scale, yScale, scale));
}

function sharedMaterial(color, options = {}) {
  return new THREE.MeshStandardMaterial({
    color,
    roughness: options.roughness ?? 0.75,
    metalness: options.metalness ?? 0,
    emissive: options.emissive ?? 0,
    emissiveIntensity: options.emissiveIntensity ?? 0
  });
}

/**
 * Browser-efficient circuit scenery with InstancedMesh batches and landmark groups.
 */
export class CircuitScenarioVisuals {
  constructor(scene, track, scenario = HARBOR_RING, assets = null) {
    this.scene = scene ?? null;
    this.track = track ?? null;
    this.scenario = scenario ?? HARBOR_RING;
    this.assets = assets;
    this.root = new THREE.Group();
    this.root.name = 'HARBOR_RING_SCENERY';
    this.plan = createCircuitPlacementPlan(this.scenario);
    this.geometry = [];
    this.materials = [];
    this.instances = [];
    this.animated = [];
    this.time = 0;
    this._build();
    if (this.scene?.add) this.scene.add(this.root);
  }

  get metrics() {
    return {
      plannedInstances: this.plan.instanceCount,
      drawCallBudget: this.plan.drawCallBudget,
      instancedBatches: this.instances.length,
      landmarkCount: this.plan.landmarks.length
    };
  }

  _material(color, options) {
    const material = sharedMaterial(color, options);
    this.materials.push(material);
    return material;
  }

  _geometry(geometry) {
    this.geometry.push(geometry);
    return geometry;
  }

  _batch(name, geometry, material, placements, configure) {
    if (!placements?.length) return null;
    const mesh = new THREE.InstancedMesh(geometry, material, placements.length);
    mesh.name = `CIRCUIT_${name.toUpperCase()}`;
    mesh.castShadow = false;
    mesh.receiveShadow = true;
    const color = new THREE.Color();
    placements.forEach((placement, index) => {
      const transform = placementTransform(this.track, placement, this.scenario, configure?.yOffset ?? 0);
      mesh.setMatrixAt(index, configure?.matrix ? configure.matrix(transform, placement) : makeMatrix(transform, transform.scale));
      if (configure?.color) {
        color.copy(configure.color(placement, index));
        mesh.setColorAt(index, color);
      }
    });
    mesh.instanceMatrix.needsUpdate = true;
    if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
    this.root.add(mesh);
    this.instances.push(mesh);
    return mesh;
  }

  _build() {
    const steel = this._material(0x26313a, { metalness: 0.65, roughness: 0.42 });
    const rubber = this._material(0x151719, { roughness: 0.94 });
    const yellow = this._material(0xe2c31b, { roughness: 0.48 });
    const red = this._material(0xb52d24, { roughness: 0.55 });
    const foliage = this._material(0x285b2f, { roughness: 0.95 });
    const trunk = this._material(0x5b3820, { roughness: 0.92 });
    const concrete = this._material(0x6d716b, { roughness: 0.88 });
    const glass = this._material(0x2d7083, { metalness: 0.25, roughness: 0.24 });
    const crowd = this._material(0xdc8e35, { roughness: 0.78 });
    const lamp = this._material(0xcaf4bb, { emissive: 0x8bea81, emissiveIntensity: 0.75, roughness: 0.3 });

    const treeTrunks = this.plan.tree;
    this._batch('tree-trunks', this._geometry(new THREE.CylinderGeometry(0.20, 0.31, 5.6, 6)), trunk, treeTrunks, {
      yOffset: 2.8,
      matrix: (transform) => makeMatrix(transform, transform.scale, transform.scale * 1.18)
    });
    this._batch('tree-crowns', this._geometry(new THREE.IcosahedronGeometry(2.25, 1)), foliage, treeTrunks, {
      yOffset: 6.4,
      matrix: (transform, placement) => makeMatrix(transform, transform.scale * (0.88 + placement.variation * 0.35), transform.scale * (1.05 + placement.variation * 0.35))
    });
    this._batch('fence', this._geometry(new THREE.BoxGeometry(3.5, 1.65, 0.10)), steel, this.plan.fence, { yOffset: 0.84 });
    this._batch('tires', this._geometry(new THREE.CylinderGeometry(0.54, 0.54, 0.92, 12)), rubber, this.plan.tireStack, {
      yOffset: 0.47,
      matrix: (transform) => makeMatrix(transform, transform.scale, transform.scale * 0.62)
    });
    this._batch('cones', this._geometry(new THREE.ConeGeometry(0.26, 0.62, 8)), red, this.plan.cone, { yOffset: 0.31 });
    this._batch('boards', this._geometry(new THREE.BoxGeometry(0.14, 1.45, 1.05)), yellow, this.plan.brakingBoard, { yOffset: 1.08 });
    this._batch('marshal-posts', this._geometry(new THREE.BoxGeometry(1.7, 2.5, 1.65)), yellow, this.plan.marshal, { yOffset: 1.25 });
    this._batch('camera-masts', this._geometry(new THREE.CylinderGeometry(0.08, 0.11, 5.5, 8)), steel, this.plan.camera, { yOffset: 2.75 });
    this._batch('camera-heads', this._geometry(new THREE.BoxGeometry(0.72, 0.42, 0.95)), glass, this.plan.camera, { yOffset: 5.25 });
    this._batch('crowd', this._geometry(new THREE.BoxGeometry(4.8, 1.2, 1.8)), crowd, this.plan.crowd, {
      yOffset: 0.6,
      color: (placement) => new THREE.Color().setHSL(0.06 + placement.variation * 0.15, 0.65, 0.47)
    });
    this._batch('flag-poles', this._geometry(new THREE.CylinderGeometry(0.045, 0.06, 4.2, 6)), steel, this.plan.flag, { yOffset: 2.1 });
    const flags = this._batch('flags', this._geometry(new THREE.BoxGeometry(1.25, 0.65, 0.05)), red, this.plan.flag, { yOffset: 3.75 });
    if (flags) this.animated.push({ material: red, base: 0.06, phase: 0.0 });
    this._batch('light-masts', this._geometry(new THREE.CylinderGeometry(0.14, 0.18, 10, 8)), steel, this.plan.lightGantry, { yOffset: 5 });
    this._batch('light-arms', this._geometry(new THREE.BoxGeometry(4.3, 0.16, 0.24)), steel, this.plan.lightGantry, { yOffset: 9.5 });
    this._batch('light-lamps', this._geometry(new THREE.BoxGeometry(0.72, 0.36, 0.30)), lamp, this.plan.lightGantry, { yOffset: 9.18 });
    this._batch('service-trucks', this._geometry(new THREE.BoxGeometry(2.45, 1.65, 5.2)), concrete, this.plan.serviceVehicle, { yOffset: 0.85 });
    this._batch('service-campers', this._geometry(new THREE.BoxGeometry(2.7, 2.15, 6.3)), glass, this.plan.camper, { yOffset: 1.08 });

    this._addLandmarks(steel, concrete, yellow, glass, lamp);
  }

  _addLandmarks(steel, concrete, yellow, glass, lamp) {
    for (const landmark of this.plan.landmarks) {
      const authored = this._authoredLandmark(landmark);
      if (authored) continue;
      const transform = placementTransform(this.track, landmark, this.scenario);
      const group = new THREE.Group();
      group.position.copy(transform.position);
      group.rotation.y = transform.heading + (landmark.headingOffset ?? 0);
      group.name = `CIRCUIT_${String(landmark.type).toUpperCase()}`;
      const add = (geometry, material, x, y, z, sx = 1, sy = 1, sz = 1) => {
        const mesh = new THREE.Mesh(geometry, material);
        mesh.position.set(x, y, z);
        mesh.scale.set(sx, sy, sz);
        mesh.castShadow = false;
        mesh.receiveShadow = true;
        group.add(mesh);
      };
      if (landmark.type === 'controlTower') {
        add(this._geometry(new THREE.BoxGeometry(12, 16, 10)), concrete, 0, 8, 0);
        add(this._geometry(new THREE.BoxGeometry(13, 2.2, 11)), glass, 0, 14, -0.3);
        add(this._geometry(new THREE.BoxGeometry(15, 0.35, 12)), steel, 0, 16.4, 0);
      } else if (landmark.type === 'hospitality') {
        add(this._geometry(new THREE.BoxGeometry(28, 6, 12)), glass, 0, 3, 0);
        add(this._geometry(new THREE.BoxGeometry(31, 0.7, 15)), steel, 0, 6.4, 0);
        add(this._geometry(new THREE.BoxGeometry(33, 0.25, 17)), yellow, 0, 7.2, 0);
      } else if (landmark.type === 'footbridge' || landmark.type === 'tunnel') {
        const width = landmark.type === 'footbridge' ? 28 : 22;
        add(this._geometry(new THREE.BoxGeometry(width, 1.2, 3.6)), steel, 0, landmark.type === 'footbridge' ? 7.4 : 5.2, 0);
        add(this._geometry(new THREE.BoxGeometry(1.1, landmark.type === 'footbridge' ? 7.4 : 5.2, 2.6)), concrete, -width * 0.44, (landmark.type === 'footbridge' ? 7.4 : 5.2) * 0.5, 0);
        add(this._geometry(new THREE.BoxGeometry(1.1, landmark.type === 'footbridge' ? 7.4 : 5.2, 2.6)), concrete, width * 0.44, (landmark.type === 'footbridge' ? 7.4 : 5.2) * 0.5, 0);
      } else if (landmark.type === 'crane') {
        add(this._geometry(new THREE.CylinderGeometry(0.38, 0.65, 18, 8)), yellow, 0, 9, 0);
        add(this._geometry(new THREE.BoxGeometry(1.1, 0.38, 18)), yellow, 0, 17.2, -8.4);
        add(this._geometry(new THREE.BoxGeometry(4.4, 0.5, 3.5)), concrete, 0, 0.25, 0);
      } else if (landmark.type === 'serviceRoad') {
        add(this._geometry(new THREE.BoxGeometry(11, 0.08, 105)), concrete, 0, 0.03, 0);
      }
      this.root.add(group);
    }

    const start = placementTransform(this.track, { fraction: this.scenario.start?.finishFraction ?? 0, lateral: 0 }, this.scenario);
    const gantry = new THREE.Group();
    gantry.name = 'HARBOR_START_GANTRY';
    gantry.position.copy(start.position);
    gantry.rotation.y = start.heading;
    const posts = new THREE.InstancedMesh(this._geometry(new THREE.BoxGeometry(0.62, 8.5, 0.75)), steel, 2);
    posts.setMatrixAt(0, new THREE.Matrix4().makeTranslation(-8.9, 4.25, 0));
    posts.setMatrixAt(1, new THREE.Matrix4().makeTranslation(8.9, 4.25, 0));
    posts.instanceMatrix.needsUpdate = true;
    gantry.add(posts);
    const beam = new THREE.Mesh(this._geometry(new THREE.BoxGeometry(18.4, 1.05, 0.9)), yellow);
    beam.position.y = 8.35;
    gantry.add(beam);
    this.root.add(gantry);
  }

  _authoredLandmark(landmark) {
    if (typeof this.assets?.cloneProp !== 'function') return false;
    const roots = {
      controlTower: 'PROP_CONTROL_TOWER', hospitality: 'PROP_HOSPITALITY',
      footbridge: 'PROP_FOOTBRIDGE', crane: 'PROP_CRANE'
    };
    const propName = roots[landmark.type];
    if (!propName) return false;
    const clone = this.assets.cloneProp(propName);
    if (!clone) return false;
    const transform = placementTransform(this.track, landmark, this.scenario);
    clone.position.copy(transform.position);
    clone.rotation.y = transform.heading + (landmark.headingOffset ?? 0);
    clone.scale.setScalar(landmark.type === 'footbridge' ? 1.35 : landmark.type === 'hospitality' ? 1.15 : 1);
    clone.name = `CIRCUIT_AUTHORED_${propName}`;
    this.root.add(clone);
    return true;
  }

  update(dt, vehicles = [], race = null) {
    this.time += Math.max(0, Number(dt) || 0);
    for (const animated of this.animated) {
      animated.material.emissive.setRGB(0.18, 0.015, 0.01);
      animated.material.emissiveIntensity = animated.base + Math.max(0, Math.sin(this.time * 1.65 + animated.phase)) * 0.08;
    }
    void vehicles;
    void race;
  }

  dispose() {
    if (this.root.parent) this.root.parent.remove(this.root);
    this.instances.length = 0;
    this.geometry.forEach((geometry) => geometry.dispose());
    this.materials.forEach((material) => material.dispose());
    this.geometry.length = 0;
    this.materials.length = 0;
    this.animated.length = 0;
    this.root.clear();
  }
}

export default CircuitScenarioVisuals;
