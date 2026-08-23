import * as THREE from 'three';

function makeRoadTexture() {
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = 512;
  const context = canvas.getContext('2d');
  context.fillStyle = '#2b3330';
  context.fillRect(0, 0, 512, 512);
  for (let i = 0; i < 4200; i += 1) {
    const shade = 35 + (i * 29) % 32;
    context.fillStyle = `rgba(${shade},${shade + 4},${shade + 2},0.18)`;
    const size = 1 + (i % 4);
    context.fillRect((i * 47) % 512, (i * 91) % 512, size, size);
  }
  context.strokeStyle = 'rgba(218,225,211,0.075)';
  context.lineWidth = 2;
  for (let y = 10; y < 512; y += 23) {
    context.beginPath();
    context.moveTo(0, y);
    context.lineTo(512, y + 8);
    context.stroke();
  }
  const texture = new THREE.CanvasTexture(canvas);
  texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
  texture.repeat.set(2.2, 26);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

function makeGrassTexture() {
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = 256;
  const context = canvas.getContext('2d');
  context.fillStyle = '#223825';
  context.fillRect(0, 0, 256, 256);
  for (let i = 0; i < 2400; i += 1) {
    const dark = i % 3 === 0;
    context.fillStyle = dark ? 'rgba(12,30,16,0.25)' : 'rgba(80,120,55,0.18)';
    context.fillRect((i * 31) % 256, (i * 67) % 256, 1 + (i % 2), 1 + ((i >> 2) % 2));
  }
  const texture = new THREE.CanvasTexture(canvas);
  texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
  texture.repeat.set(45, 45);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

function makeCheckerTexture() {
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = 128;
  const context = canvas.getContext('2d');
  const cell = 16;
  for (let y = 0; y < 8; y += 1) {
    for (let x = 0; x < 8; x += 1) {
      context.fillStyle = (x + y) % 2 ? '#111614' : '#e6ebe2';
      context.fillRect(x * cell, y * cell, cell, cell);
    }
  }
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

function trackPoint(point, lateral = 0, lift = 0) {
  const bank = point?.bank ?? 0;
  const normalX = point?.normal?.x ?? 0;
  const normalZ = point?.normal?.z ?? 0;
  return {
    x: (point?.x ?? 0) + normalX * lateral,
    y: (point?.y ?? 0) + Math.sin(bank) * lateral + lift,
    z: (point?.z ?? 0) + normalZ * lateral
  };
}

const smoothstep = (edge0, edge1, value) => {
  const denominator = edge1 - edge0;
  const t = Math.max(0, Math.min(1, (value - edge0) / (Math.abs(denominator) < 1e-6 ? 1e-6 : denominator)));
  return t * t * (3 - 2 * t);
};

const baseTerrainHeight = (x, z) => -0.42
  + Math.sin(x * 0.052 + z * 0.019) * 0.12
  + Math.cos(z * 0.045) * 0.08;

export function terrainHeightAt(track, x, z) {
  const sampleX = Number.isFinite(x) ? x : 0;
  const sampleZ = Number.isFinite(z) ? z : 0;
  const base = baseTerrainHeight(sampleX, sampleZ);
  if (!track?.closest) return base;
  const closest = track.closest(sampleX, sampleZ);
  if (!closest || !Number.isFinite(closest.lateral)) return base;
  const roadEdge = Number.isFinite(track.roadHalfWidth) ? track.roadHalfWidth : 6.5;
  const curbEdge = roadEdge + (Number.isFinite(track.curbWidth) ? track.curbWidth : 1.05);
  const barrierEdge = curbEdge + (Number.isFinite(track.runoffWidth) ? track.runoffWidth : 8.5);
  const lateralAbs = Math.abs(closest.lateral);
  const blendOutside = barrierEdge + 4.5;
  const corridorInfluence = smoothstep(blendOutside, barrierEdge, lateralAbs);
  if (corridorInfluence <= 0) return base;
  const bankedTrackHeight = (closest.y ?? 0) + Math.sin(closest.bank ?? 0) * closest.lateral;
  const carvedTarget = bankedTrackHeight - 0.24;
  const carved = base * (1 - corridorInfluence) + carvedTarget * corridorInfluence;
  return Math.min(base, Number.isFinite(carved) ? carved : base);
}

function roadGeometry(track) {
  const vertices = [];
  const uvs = [];
  const indices = [];
  const samples = track.samples || [];
  for (let i = 0; i < samples.length; i += 1) {
    const point = samples[i];
    const inner = trackPoint(point, -track.roadHalfWidth, 0.035);
    const outer = trackPoint(point, track.roadHalfWidth, 0.035);
    vertices.push(
      inner.x, inner.y, inner.z,
      outer.x, outer.y, outer.z
    );
    uvs.push(0, point.s / 4.4, 1, point.s / 4.4);
  }
  for (let i = 0; i < samples.length; i += 1) {
    const next = (i + 1) % samples.length;
    indices.push(i * 2, next * 2, i * 2 + 1, i * 2 + 1, next * 2, next * 2 + 1);
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
  geometry.setAttribute('uv', new THREE.Float32BufferAttribute(uvs, 2));
  geometry.setIndex(indices);
  geometry.computeVertexNormals();
  return geometry;
}

function ribbonGeometry(track, inner, outer, height, repeat = 6) {
  const vertices = [];
  const uvs = [];
  const indices = [];
  const samples = track.samples || [];
  for (let i = 0; i < samples.length; i += 1) {
    const point = samples[i];
    for (const side of [-1, 1]) {
      const innerPoint = trackPoint(point, side * inner, height);
      const outerPoint = trackPoint(point, side * outer, height);
      vertices.push(
        innerPoint.x, innerPoint.y, innerPoint.z,
        outerPoint.x, outerPoint.y, outerPoint.z
      );
      uvs.push(0, point.s / repeat, 1, point.s / repeat);
    }
  }
  for (let i = 0; i < samples.length; i += 1) {
    const next = (i + 1) % samples.length;
    for (let side = 0; side < 2; side += 1) {
      const a = i * 4 + side * 2;
      const b = next * 4 + side * 2;
      indices.push(a, b, a + 1, a + 1, b, b + 1);
    }
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
  geometry.setAttribute('uv', new THREE.Float32BufferAttribute(uvs, 2));
  geometry.setIndex(indices);
  geometry.computeVertexNormals();
  return geometry;
}

function segmentGeometry(track, start, end, lateralA, lateralB, y = 0.045, samples = 52) {
  const vertices = [];
  const uvs = [];
  const indices = [];
  for (let i = 0; i <= samples; i += 1) {
    const point = track.atDistance(start + (end - start) * (i / samples));
    const a = trackPoint(point, lateralA, y);
    const b = trackPoint(point, lateralB, y);
    vertices.push(
      a.x, a.y, a.z,
      b.x, b.y, b.z
    );
    uvs.push(0, i / 3, 1, i / 3);
  }
  for (let i = 0; i < samples; i += 1) {
    const a = i * 2;
    indices.push(a, a + 2, a + 1, a + 1, a + 2, a + 3);
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
  geometry.setAttribute('uv', new THREE.Float32BufferAttribute(uvs, 2));
  geometry.setIndex(indices);
  geometry.computeVertexNormals();
  return geometry;
}

function addInstanced(parent, geometry, material, records, { castShadow = false, receiveShadow = true } = {}) {
  if (!records.length) return null;
  const mesh = new THREE.InstancedMesh(geometry, material, records.length);
  const object = new THREE.Object3D();
  records.forEach((record, index) => {
    object.position.copy(record.position);
    object.rotation.set(0, record.yaw, 0);
    object.scale.copy(record.scale || new THREE.Vector3(1, 1, 1));
    object.updateMatrix();
    mesh.setMatrixAt(index, object.matrix);
  });
  mesh.instanceMatrix.needsUpdate = true;
  mesh.castShadow = castShadow;
  mesh.receiveShadow = receiveShadow;
  mesh.computeBoundingSphere();
  parent.add(mesh);
  return mesh;
}

function along(point, side, offset, height = 0) {
  const placed = trackPoint(point, side * offset, height);
  return new THREE.Vector3(placed.x, placed.y, placed.z);
}

function pointYaw(point) {
  return Math.atan2(point?.tangent?.x ?? 0, point?.tangent?.z ?? 1);
}

function makeSign(text, width = 9) {
  const canvas = document.createElement('canvas');
  canvas.width = 1024;
  canvas.height = 256;
  const context = canvas.getContext('2d');
  context.fillStyle = '#0a1017';
  context.fillRect(0, 0, canvas.width, canvas.height);
  context.fillStyle = '#00f0ff';
  context.fillRect(24, 24, canvas.width - 48, 10);
  context.fillStyle = '#f0f6fa';
  context.font = '900 100px Arial, sans-serif';
  context.textAlign = 'center';
  context.textBaseline = 'middle';
  context.fillText(text, canvas.width / 2, 148);
  const map = new THREE.CanvasTexture(canvas);
  map.colorSpace = THREE.SRGBColorSpace;
  const plate = new THREE.Mesh(
    new THREE.PlaneGeometry(width, width * 0.25),
    new THREE.MeshBasicMaterial({ map, transparent: true, toneMapped: false })
  );
  return plate;
}

export class CircuitEnvironment {
  constructor(scene, track) {
    this.scene = scene;
    this.track = track;
    this.root = new THREE.Group();
    this.root.name = 'PROCEDURAL_CIRCUIT_BED';
    this.fallbackProps = new THREE.Group();
    this.assetProps = new THREE.Group();
    this.assetProps.name = 'BLENDER_CIRCUIT_PROPS';
    this.root.add(this.fallbackProps, this.assetProps);
    this.scene.add(this.root);
    this.assetInstalled = false;
    this.pitSide = Math.sign(this.track.scenario?.pit?.lateralM ?? -1) || -1;
    this._createSkyAndTerrain();
    this._createLighting();
    this._createCircuitBed();
    this._createFallbackProps();
  }

  _createSkyAndTerrain() {
    const authored = this.track.scenario?.controlPoints ?? this.track.samples ?? [];
    const extent = Math.max(340, ...authored.map((p) => Math.max(Math.abs(p.x ?? 0), Math.abs(p.z ?? 0)))) + 180;
    this.extent = extent;
    this.scene.fog = new THREE.FogExp2('#142433', 0.00185);

    const sky = new THREE.Mesh(
      new THREE.SphereGeometry(extent * 2.25, 32, 20),
      new THREE.ShaderMaterial({
        side: THREE.BackSide,
        uniforms: {
          topColor: { value: new THREE.Color('#0c2034') },
          horizonColor: { value: new THREE.Color('#1c3a54') },
          bottomColor: { value: new THREE.Color('#081018') }
        },
        vertexShader: `
          varying vec3 vWorld;
          void main() {
            vWorld = (modelMatrix * vec4(position, 1.0)).xyz;
            gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
          }
        `,
        fragmentShader: `
          uniform vec3 topColor;
          uniform vec3 horizonColor;
          uniform vec3 bottomColor;
          varying vec3 vWorld;
          void main() {
            float h = normalize(vWorld).y;
            vec3 c = mix(horizonColor, topColor, smoothstep(-0.04, 0.72, h));
            c = mix(bottomColor, c, smoothstep(-0.25, 0.09, h));
            gl_FragColor = vec4(c, 1.0);
          }
        `
      })
    );
    this.root.add(sky);

    const groundGeometry = new THREE.PlaneGeometry(extent * 2, extent * 2, 176, 176);
    groundGeometry.rotateX(-Math.PI / 2);
    const positions = groundGeometry.getAttribute('position');
    for (let i = 0; i < positions.count; i += 1) {
      const x = positions.getX(i);
      const z = positions.getZ(i);
      positions.setY(i, terrainHeightAt(this.track, x, z));
    }
    positions.needsUpdate = true;
    groundGeometry.computeVertexNormals();

    const ground = new THREE.Mesh(
      groundGeometry,
      new THREE.MeshStandardMaterial({
        map: makeGrassTexture(),
        color: '#344c32',
        roughness: 0.98,
        metalness: 0
      })
    );
    ground.receiveShadow = true;
    this.root.add(ground);
  }

  _createLighting() {
    const hemisphere = new THREE.HemisphereLight('#c8e4ff', '#18281c', 2.2);
    this.root.add(hemisphere);

    const sun = new THREE.DirectionalLight('#fff0d8', 3.3);
    sun.position.set(-90, 150, 45);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    const shadowExtent = Math.min(680, this.extent ?? 510);
    sun.shadow.camera.left = -shadowExtent;
    sun.shadow.camera.right = shadowExtent;
    sun.shadow.camera.top = shadowExtent;
    sun.shadow.camera.bottom = -shadowExtent;
    sun.shadow.bias = -0.00012;
    this.root.add(sun);
  }

  _createCircuitBed() {
    const road = new THREE.Mesh(
      roadGeometry(this.track),
      new THREE.MeshStandardMaterial({
        map: makeRoadTexture(),
        color: '#424a46',
        roughness: 0.91,
        metalness: 0.015,
        side: THREE.DoubleSide
      })
    );
    road.name = 'PROCEDURAL_ROAD_SURFACE';
    road.receiveShadow = true;
    this.root.add(road);

    const shoulder = new THREE.Mesh(
      ribbonGeometry(this.track, this.track.roadHalfWidth, this.track.roadHalfWidth + this.track.curbWidth, 0.026, 2.3),
      new THREE.MeshStandardMaterial({ color: '#cfcbc0', roughness: 0.84, side: THREE.DoubleSide })
    );
    shoulder.name = 'PROCEDURAL_TRACK_SHOULDERS';
    shoulder.receiveShadow = true;
    this.root.add(shoulder);

    const runoff = new THREE.Mesh(
      ribbonGeometry(
        this.track,
        this.track.roadHalfWidth + this.track.curbWidth,
        this.track.roadHalfWidth + this.track.curbWidth + this.track.runoffWidth,
        0.018,
        5.6
      ),
      new THREE.MeshStandardMaterial({ color: '#566168', roughness: 0.95, side: THREE.DoubleSide })
    );
    runoff.name = 'PROCEDURAL_RUNOFF';
    runoff.receiveShadow = true;
    this.root.add(runoff);

    const outerVerge = new THREE.Mesh(
      ribbonGeometry(
        this.track,
        this.track.roadHalfWidth + this.track.curbWidth + this.track.runoffWidth,
        this.track.roadHalfWidth + this.track.curbWidth + this.track.runoffWidth + 2.6,
        0.014,
        4
      ),
      new THREE.MeshStandardMaterial({ color: '#385434', roughness: 1, side: THREE.DoubleSide })
    );
    outerVerge.receiveShadow = true;
    this.root.add(outerVerge);

    this._createCurbs();
    this._createPitAndStart();
    this._createBarriersAndFence();
  }

  _createCurbs() {
    const red = [];
    const white = [];
    const samples = this.track.samples || [];
    const segmentLength = (this.track.length / Math.max(1, samples.length)) * 2.15;
    for (let i = 0; i < samples.length; i += 2) {
      const point = samples[i];
      if (!point?.curbSide) continue;
      const sides = [point.curbSide];
      if (point.turnStrength > 0.6 && i % 4 === 0) sides.push(-point.curbSide);
      for (const side of sides) {
        const record = {
          position: along(point, side, this.track.roadHalfWidth + this.track.curbWidth * 0.5, 0.105),
          yaw: pointYaw(point),
          scale: new THREE.Vector3(1, 1, segmentLength / 2.5)
        };
        ((Math.floor(i / 2) + (side > 0 ? 0 : 1)) % 2 ? red : white).push(record);
      }
    }
    const geometry = new THREE.BoxGeometry(this.track.curbWidth, 0.16, 2.5);
    addInstanced(this.root, geometry, new THREE.MeshStandardMaterial({ color: '#d94032', roughness: 0.78 }), red, { receiveShadow: true });
    addInstanced(this.root, geometry, new THREE.MeshStandardMaterial({ color: '#e9e5d6', roughness: 0.82 }), white, { receiveShadow: true });
  }

  _createPitAndStart() {
    const start = this.track.atDistance(0);
    const startLine = new THREE.Mesh(
      new THREE.BoxGeometry(this.track.roadHalfWidth * 2, 0.022, 1.25),
      new THREE.MeshStandardMaterial({ map: makeCheckerTexture(), roughness: 0.72 })
    );
    startLine.position.set(start.x, start.y + 0.062, start.z);
    startLine.rotation.y = pointYaw(start);
    startLine.receiveShadow = true;
    this.root.add(startLine);

    const lineMaterial = new THREE.MeshBasicMaterial({ color: '#f5f3db', transparent: true, opacity: 0.76 });
    for (let row = 0; row < 7; row += 1) {
      for (const side of [-1, 1]) {
        const marker = this.track.atDistance(-8 - row * 7.2);
        const line = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.018, 3.1), lineMaterial);
        line.position.copy(along(marker, side, 2.2, 0.058));
        line.rotation.y = pointYaw(marker);
        this.root.add(line);
      }
    }

    const pitOuter = this.pitSide * (this.track.roadHalfWidth + this.track.curbWidth + 5.15);
    const pitInner = this.pitSide * (this.track.roadHalfWidth + this.track.curbWidth + 0.18);
    const pitConfig = this.track.scenario?.pit;
    const pitStart = pitConfig ? pitConfig.entryFraction * this.track.length : -51;
    const rawPitEnd = pitConfig ? pitConfig.exitFraction * this.track.length : 78;
    const pitEnd = rawPitEnd <= pitStart ? rawPitEnd + this.track.length : rawPitEnd;
    const pit = new THREE.Mesh(
      segmentGeometry(this.track, pitStart, pitEnd, pitInner, pitOuter, 0.044),
      new THREE.MeshStandardMaterial({ color: '#353c39', roughness: 0.93, side: THREE.DoubleSide })
    );
    pit.name = 'PROCEDURAL_PIT_LANE';
    pit.receiveShadow = true;
    this.root.add(pit);

    const pitLine = new THREE.Mesh(
      segmentGeometry(
        this.track,
        pitStart + 4,
        pitEnd - 3,
        this.pitSide * (this.track.roadHalfWidth + this.track.curbWidth + 1.1),
        this.pitSide * (this.track.roadHalfWidth + this.track.curbWidth + 1.26),
        0.068
      ),
      new THREE.MeshBasicMaterial({ color: '#f0ecdb', side: THREE.DoubleSide })
    );
    this.root.add(pitLine);
  }

  _createBarriersAndFence() {
    const rails = [];
    const fences = [];
    const posts = [];
    const offset = this.track.roadHalfWidth + this.track.curbWidth + this.track.runoffWidth + 0.55;
    const samples = this.track.samples || [];
    const segmentLength = (this.track.length / Math.max(1, samples.length)) * 3.5;
    for (let i = 0; i < samples.length; i += 3) {
      const point = samples[i];
      for (const side of [-1, 1]) {
        const base = along(point, side, offset, 0.58);
        rails.push({ position: base, yaw: pointYaw(point), scale: new THREE.Vector3(1, 1, segmentLength / 3.4) });
        fences.push({ position: along(point, side, offset + 0.22, 1.4), yaw: pointYaw(point), scale: new THREE.Vector3(1, 1, segmentLength / 3.4) });
        posts.push({ position: along(point, side, offset + 0.22, 1.05), yaw: pointYaw(point) });
      }
    }
    addInstanced(this.root, new THREE.BoxGeometry(0.2, 0.76, 3.4), new THREE.MeshStandardMaterial({ color: '#c5d0c8', roughness: 0.4, metalness: 0.7 }), rails, { castShadow: true });
    addInstanced(this.root, new THREE.BoxGeometry(0.045, 1.55, 3.4), new THREE.MeshStandardMaterial({ color: '#253530', roughness: 0.72, metalness: 0.2, transparent: true, opacity: 0.78 }), fences, { receiveShadow: true });
    addInstanced(this.root, new THREE.BoxGeometry(0.13, 2.1, 0.13), new THREE.MeshStandardMaterial({ color: '#4f5c56', roughness: 0.5, metalness: 0.62 }), posts, { castShadow: true });
  }

  _createFallbackProps() {
    const steel = new THREE.MeshStandardMaterial({ color: '#263330', metalness: 0.7, roughness: 0.36 });
    const seat = new THREE.MeshStandardMaterial({ color: '#133e54', roughness: 0.75 });
    const start = this.track.atDistance(18);
    const pit = new THREE.Group();
    pit.position.copy(along(start, this.pitSide, this.track.roadHalfWidth + this.track.curbWidth + 9.8));
    pit.rotation.y = pointYaw(start) + Math.PI;
    const building = new THREE.Mesh(new THREE.BoxGeometry(18, 4.8, 7.2), steel);
    building.position.y = 2.4;
    building.castShadow = building.receiveShadow = true;
    pit.add(building);
    const roof = new THREE.Mesh(new THREE.BoxGeometry(19.2, 0.24, 8.5), steel);
    roof.position.y = 5.05;
    pit.add(roof);
    this.fallbackProps.add(pit);

    for (const distance of [48, 84]) {
      const point = this.track.atDistance(distance);
      const stand = new THREE.Group();
      stand.position.copy(along(point, -this.pitSide, 23));
      stand.rotation.y = pointYaw(point);
      for (let row = 0; row < 5; row += 1) {
        const bench = new THREE.Mesh(new THREE.BoxGeometry(16, 0.22, 0.74), seat);
        bench.position.set(0, 0.72 + row * 0.66, -1.9 + row * 0.63);
        stand.add(bench);
      }
      this.fallbackProps.add(stand);
    }

    const gantryPoint = this.track.atDistance(0);
    const gantry = new THREE.Group();
    gantry.position.set(gantryPoint.x, gantryPoint.y, gantryPoint.z);
    gantry.rotation.y = pointYaw(gantryPoint);
    const cross = new THREE.Mesh(new THREE.BoxGeometry(19, 0.26, 0.36), steel);
    cross.position.y = 6.1;
    gantry.add(cross);
    for (const x of [-8.4, 8.4]) {
      const post = new THREE.Mesh(new THREE.BoxGeometry(0.22, 6.1, 0.22), steel);
      post.position.set(x, 3.05, 0);
      gantry.add(post);
    }
    const sign = makeSign('GEMINI // GAUNTLET', 8.2);
    sign.position.set(0, 4.85, 0.22);
    gantry.add(sign);
    this.fallbackProps.add(gantry);
  }

  installAssets(assets) {
    if (this.assetInstalled) return true;
    const placed = [];
    const place = (name, distance, side, offset, scale = 1, yawOffset = 0) => {
      const model = assets.cloneProp(name);
      if (!model) return;
      const point = this.track.atDistance(distance);
      model.position.copy(along(point, side, offset));
      model.rotation.y = pointYaw(point) + yawOffset;
      model.scale.setScalar(scale);
      this.assetProps.add(model);
      placed.push(name);
    };

    const pitOffset = this.track.roadHalfWidth + this.track.curbWidth + 9.8;
    place('PROP_PIT_MODULE', 17, this.pitSide, pitOffset, 1, Math.PI);
    place('PROP_PIT_MODULE', 38, this.pitSide, pitOffset, 1, Math.PI);
    place('PROP_GRANDSTAND', 47, -this.pitSide, 24, 1, 0);
    place('PROP_GRANDSTAND', 82, -this.pitSide, 24, 0.92, 0);
    place('PROP_GANTRY', 0, 1, 0, 1, 0);

    for (const distance of [21, 70, 142, 217, 302, 379, 451, 525]) {
      const side = Math.floor(distance / 70) % 2 ? -1 : 1;
      place('PROP_LIGHT_TOWER', distance, side, 27, 1, 0);
    }
    for (const distance of [114, 248, 388, 506]) {
      const point = this.track.atDistance(distance);
      place('PROP_MARSHAL_POST', distance, point?.curbSide || 1, 20, 1, 0);
    }
    const samples = this.track.samples || [];
    for (let i = 0; i < samples.length; i += 11) {
      const point = samples[i];
      const side = i % 3 ? -1 : 1;
      place('PROP_TREE', point.s, side, 29 + (i % 5) * 2.6, 0.8 + (i % 4) * 0.1, i * 0.13);
    }
    for (let i = 0; i < samples.length; i += 10) {
      const point = samples[i];
      if (!point?.curbSide || point.turnStrength < 0.4) continue;
      const outside = -point.curbSide;
      const offset = this.track.roadHalfWidth + this.track.curbWidth + this.track.runoffWidth - 0.8;
      place('PROP_TECPRO', point.s, outside, offset, 1, 0);
      if (i % 20 === 0) place('PROP_TIRE_STACK', point.s + 3, outside, offset - 1.2, 1, 0);
    }
    if (!placed.length) return false;
    this.fallbackProps.visible = false;
    this.assetInstalled = true;
    return true;
  }

  update(dt = 0) {
    return true;
  }

  dispose() {
    if (this.root) {
      this.root.traverse((node) => {
        if (node.geometry) {
          node.geometry.dispose();
        }
        if (node.material) {
          if (Array.isArray(node.material)) {
            node.material.forEach((mat) => {
              if (mat.map) mat.map.dispose();
              if (mat.roughnessMap) mat.roughnessMap.dispose();
              if (mat.normalMap) mat.normalMap.dispose();
              mat.dispose();
            });
          } else {
            if (node.material.map) node.material.map.dispose();
            if (node.material.roughnessMap) node.material.roughnessMap.dispose();
            if (node.material.normalMap) node.material.normalMap.dispose();
            node.material.dispose();
          }
        }
      });
      this.root.removeFromParent();
    }
  }
}
