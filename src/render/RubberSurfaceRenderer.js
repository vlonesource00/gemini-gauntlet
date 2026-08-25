import * as THREE from 'three';

const finite = (value, fallback = 0) => Number.isFinite(value) ? value : fallback;

const pointAt = (point, lateral, lift = 0.044) => {
  const bank = finite(point?.bank);
  return {
    x: finite(point?.x) + finite(point?.normal?.x, -1) * lateral,
    y: finite(point?.y) + Math.sin(bank) * lateral + lift,
    z: finite(point?.z) + finite(point?.normal?.z) * lateral
  };
};

const VERTEX_SHADER = `
  attribute float rubber;
  varying float vRubber;
  void main() {
    vRubber = rubber;
    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
  }
`;

const FRAGMENT_SHADER = `
  varying float vRubber;
  uniform vec3 baseColor;
  uniform float opacity;
  void main() {
    float visibility = smoothstep(0.008, 0.08, vRubber);
    vec3 colour = mix(baseColor, vec3(0.012, 0.016, 0.014), clamp(vRubber * 1.25, 0.0, 1.0));
    gl_FragColor = vec4(colour, visibility * opacity);
  }
`;

export class RubberSurfaceRenderer {
  constructor(parent, track) {
    this.parent = parent;
    this.track = track;
    this.laneCount = Math.max(7, Math.trunc(track?.rubberLaneCount ?? track?.rubberLanes ?? 9));
    this.segmentCount = track?.samples?.length ?? 0;
    this.vertexCount = this.segmentCount * this.laneCount * 4;
    this._lastRevision = -1;
    this._elapsed = 0.1;
    this.updateCount = 0;

    const positions = new Float32Array(this.vertexCount * 3);
    const rubber = new Float32Array(this.vertexCount);
    const indices = new Uint32Array(this.segmentCount * this.laneCount * 6);
    let vertex = 0;
    let index = 0;
    const laneWidth = (track?.roadHalfWidth ?? 6.5) * 2 / this.laneCount;
    for (let segment = 0; segment < this.segmentCount; segment += 1) {
      const a = track.samples[segment];
      const b = track.samples[(segment + 1) % this.segmentCount];
      for (let lane = 0; lane < this.laneCount; lane += 1) {
        const lateralA = -track.roadHalfWidth + lane * laneWidth;
        const lateralB = lateralA + laneWidth;
        const corners = [
          pointAt(a, lateralA), pointAt(b, lateralA),
          pointAt(a, lateralB), pointAt(b, lateralB)
        ];
        for (const corner of corners) {
          positions[vertex * 3] = corner.x;
          positions[vertex * 3 + 1] = corner.y;
          positions[vertex * 3 + 2] = corner.z;
          vertex += 1;
        }
        const base = vertex - 4;
        indices[index++] = base;
        indices[index++] = base + 1;
        indices[index++] = base + 2;
        indices[index++] = base + 2;
        indices[index++] = base + 1;
        indices[index++] = base + 3;
      }
    }
    this.geometry = new THREE.BufferGeometry();
    this.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    this.geometry.setAttribute('rubber', new THREE.BufferAttribute(rubber, 1));
    this.geometry.setIndex(new THREE.BufferAttribute(indices, 1));
    this.material = new THREE.ShaderMaterial({
      uniforms: {
        baseColor: { value: new THREE.Color('#59615c') },
        opacity: { value: 0.62 }
      },
      vertexShader: VERTEX_SHADER,
      fragmentShader: FRAGMENT_SHADER,
      transparent: true,
      depthTest: true,
      depthWrite: false,
      side: THREE.DoubleSide,
      blending: THREE.NormalBlending
    });
    this.mesh = new THREE.Mesh(this.geometry, this.material);
    this.mesh.name = 'DYNAMIC_RUBBER_SURFACE';
    this.mesh.frustumCulled = false;
    this.mesh.renderOrder = 1;
    parent?.add(this.mesh);
    this._updateAttribute(true);
    this._elapsed = 0;
  }

  _updateAttribute(force = false) {
    const revision = Math.trunc(finite(this.track?.rubberRevision ?? this.track?.revision, 0));
    if (!force && revision === this._lastRevision) return false;
    const attribute = this.geometry.getAttribute('rubber');
    const values = attribute.array;
    let offset = 0;
    const laneWidth = (this.track?.roadHalfWidth ?? 6.5) * 2 / this.laneCount;
    for (let segment = 0; segment < this.segmentCount; segment += 1) {
      for (let lane = 0; lane < this.laneCount; lane += 1) {
        const lateralA = -this.track.roadHalfWidth + lane * laneWidth;
        const lateralB = lateralA + laneWidth;
        const a = this.track.rubberAt(segment, lateralA);
        const b = this.track.rubberAt(segment, lateralB);
        const next = (segment + 1) % this.segmentCount;
        const c = this.track.rubberAt(next, lateralA);
        const d = this.track.rubberAt(next, lateralB);
        values[offset++] = finite(a);
        values[offset++] = finite(c);
        values[offset++] = finite(b);
        values[offset++] = finite(d);
      }
    }
    attribute.needsUpdate = true;
    this._lastRevision = revision;
    this.updateCount += 1;
    return true;
  }

  update(dt = 0) {
    this._elapsed += Math.max(0, finite(dt));
    if (this._elapsed < 0.1) return false;
    const changed = this._updateAttribute();
    if (changed) this._elapsed = 0;
    return changed;
  }

  dispose() {
    this.mesh?.removeFromParent();
    this.geometry?.dispose();
    this.material?.dispose();
    this.mesh = null;
    this.geometry = null;
    this.material = null;
    this.parent = null;
    this.track = null;
  }
}

export default RubberSurfaceRenderer;
