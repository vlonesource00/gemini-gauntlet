import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';

export const ASSET_URLS = Object.freeze({
  cars: Object.freeze({
    gt: '/assets/models/car-gt.glb',
    touring: '/assets/models/car-touring.glb',
    prototype: '/assets/models/car-prototype.glb'
  }),
  props: '/assets/models/circuit-props.glb'
});

export const CAR_VARIANTS = Object.freeze(['gt', 'touring', 'prototype']);
export const PROP_NODES = Object.freeze([
  'PROP_PIT_MODULE',
  'PROP_GRANDSTAND',
  'PROP_MARSHAL_POST',
  'PROP_TIRE_STACK',
  'PROP_TECPRO',
  'PROP_GANTRY',
  'PROP_LIGHT_TOWER',
  'PROP_FENCE_PANEL',
  'PROP_TREE'
]);

function clonePaint(material, color) {
  const copy = material.clone();
  if (color) copy.color.set(color);
  if (copy.emissive) copy.emissive.multiplyScalar(0.22);
  copy.needsUpdate = true;
  return copy;
}

function isPaint(material) {
  return material?.name?.startsWith('MAT_PAINT');
}

function decorateScene(scene) {
  scene.traverse((object) => {
    if (!object.isMesh) return;
    object.castShadow = true;
    object.receiveShadow = true;
    if (object.material) {
      if (Array.isArray(object.material)) {
        object.material.forEach((mat) => { mat.needsUpdate = true; });
      } else {
        object.material.needsUpdate = true;
      }
    }
  });
  return scene;
}

export class AssetLibrary {
  constructor({ onStatus = () => {} } = {}) {
    this.loader = new GLTFLoader();
    this.onStatus = onStatus;
    this.cache = new Map();
    this.templates = new Map();
    this.errors = [];
  }

  _emit(state, key) {
    this.onStatus({ state, key, errors: this.errors.slice() });
  }

  _load(key, url) {
    if (this.cache.has(key)) return this.cache.get(key);
    this._emit('loading', key);
    const request = this.loader.loadAsync(url)
      .then((gltf) => {
        const scene = decorateScene(gltf.scene);
        this.templates.set(key, scene);
        this._emit('ready', key);
        return scene;
      })
      .catch((error) => {
        this.errors.push({ key, message: error?.message || String(error) });
        this._emit('failed', key);
        throw error;
      });
    this.cache.set(key, request);
    return request;
  }

  loadCar(variant) {
    if (!ASSET_URLS.cars[variant]) return Promise.reject(new Error('Unknown car variant: ' + variant));
    return this._load('car:' + variant, ASSET_URLS.cars[variant]);
  }

  loadProps() {
    return this._load('props', ASSET_URLS.props);
  }

  async preload() {
    const requests = [
      ...CAR_VARIANTS.map((variant) => this.loadCar(variant)),
      this.loadProps()
    ];
    const results = await Promise.allSettled(requests);
    return {
      loaded: results.filter((r) => r.status === 'fulfilled').length,
      failed: results.filter((r) => r.status === 'rejected').length,
      errors: this.errors.slice()
    };
  }

  cloneCar(variant, color) {
    const source = this.templates.get('car:' + variant);
    if (!source) return null;
    const clone = source.clone(true);
    clone.traverse((object) => {
      if (!object.isMesh || !object.material) return;
      const tint = (material) => isPaint(material) ? clonePaint(material, color) : material;
      object.material = Array.isArray(object.material) ? object.material.map(tint) : tint(object.material);
      object.castShadow = true;
      object.receiveShadow = true;
    });
    return clone;
  }

  cloneProp(name) {
    const source = this.templates.get('props')?.getObjectByName(name);
    if (!source) return null;
    const clone = source.clone(true);
    clone.traverse((object) => {
      if (!object.isMesh) return;
      object.castShadow = true;
      object.receiveShadow = true;
    });
    return clone;
  }
}
