import * as THREE from 'three';

const MAX_OPPONENTS = 12;
const TIME_HORIZONS = [0.5, 1.0, 2.0, 3.0];
const HORIZON_COLORS = [
  new THREE.Color(0xff3344), // t = 0.5s: Urgent/Red
  new THREE.Color(0xff9100), // t = 1.0s: Amber
  new THREE.Color(0xffd600), // t = 2.0s: Gold
  new THREE.Color(0x00e5ff)  // t = 3.0s: Cyan
];

const CORRIDOR_SEGMENTS = 24;
const CORRIDOR_WIDTH = 2.4;

const CORRIDOR_COLORS = {
  ATTACK_LEFT: { base: new THREE.Color(0x00ff88), top: new THREE.Color(0x76ff03) },
  ATTACK_RIGHT: { base: new THREE.Color(0x00e5ff), top: new THREE.Color(0x18ffff) },
  ATTACK_INSIDE: { base: new THREE.Color(0x00ff88), top: new THREE.Color(0x76ff03) },
  ATTACK_OUTSIDE: { base: new THREE.Color(0x00d4ff), top: new THREE.Color(0x80d8ff) },
  DIVEBOMB: { base: new THREE.Color(0xff1744), top: new THREE.Color(0xff5252) },
  SWITCHBACK: { base: new THREE.Color(0xffea00), top: new THREE.Color(0xffff00) },
  APEX_SHIELD: { base: new THREE.Color(0xd500f9), top: new THREE.Color(0xff4081) },
  DIAMOND_DEFENSE: { base: new THREE.Color(0x7c4dff), top: new THREE.Color(0xb388ff) },
  EXIT_SQUEEZE: { base: new THREE.Color(0xff3d00), top: new THREE.Color(0xff6e40) },
  LOCK_DEFENSIVE_LANE: { base: new THREE.Color(0xaa00ff), top: new THREE.Color(0xe040fb) },
  ONE_MOVE_RETURN: { base: new THREE.Color(0x651fff), top: new THREE.Color(0x8c9eff) },
  DEFEND_LEFT: { base: new THREE.Color(0xf05cff), top: new THREE.Color(0xff4081) },
  DEFEND_RIGHT: { base: new THREE.Color(0xd500f9), top: new THREE.Color(0x7c4dff) },
  DEFEND_INSIDE: { base: new THREE.Color(0xaa00ff), top: new THREE.Color(0xe040fb) },
  BREAK_TOW: { base: new THREE.Color(0xff6d00), top: new THREE.Color(0xffab00) },
  DEFEND: { base: new THREE.Color(0xf05cff), top: new THREE.Color(0xe040fb) },
  RETURN: { base: new THREE.Color(0xffab00), top: new THREE.Color(0xffd740) },
  DEFAULT: { base: new THREE.Color(0x00e5ff), top: new THREE.Color(0x18ffff) }
};

const finite = (val, fallback = 0) => (Number.isFinite(val) ? val : fallback);
const clamp = (val, min, max) => Math.max(min, Math.min(max, val));

/**
 * 3D Tactical Zone & Awareness Renderer
 * Renders opponent prediction cones/bounding boxes across time horizons (0.5s, 1.0s, 2.0s, 3.0s),
 * illuminated 3D tactical corridor road strips, and dynamic divebomb/braking trigger markers.
 */
export class TacticalZoneRenderer {
  constructor(scene, options = {}) {
    this.scene = scene;
    this.group = new THREE.Group();
    this.group.name = 'AI_TACTICAL_ZONE';
    this.visible = true;

    this.carDimensions = {
      width: options.carWidth ?? 2.05,
      length: options.carLength ?? 4.75,
      height: options.carHeight ?? 1.22
    };

    // Reusable math structures
    this._vRef = new THREE.Vector3();
    this._vTangent = new THREE.Vector3();
    this._vNormal = new THREE.Vector3(0, 1, 0);
    this._vBinormal = new THREE.Vector3();
    this._vCorner = new THREE.Vector3();
    this._tempColor = new THREE.Color();

    this._initPredictionBoxes();
    this._initTacticalCorridorRibbon();
    this._initBrakingPointMarker();

    if (scene) {
      scene.add(this.group);
    }
  }

  _initPredictionBoxes() {
    // Each box: 12 lines (24 vertices).
    // Plus prediction connecting center rays: 4 segments (8 vertices) per opponent.
    // Total per opponent = 4 boxes * 24 + 8 = 104 vertices.
    const totalVertices = MAX_OPPONENTS * (TIME_HORIZONS.length * 24 + 8);
    const positions = new Float32Array(totalVertices * 3);
    const colors = new Float32Array(totalVertices * 3);

    this.predictionGeometry = new THREE.BufferGeometry();
    this.predictionGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    this.predictionGeometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    this.predictionGeometry.setDrawRange(0, 0);

    this.predictionMaterial = new THREE.LineBasicMaterial({
      vertexColors: true,
      transparent: true,
      opacity: 0.85,
      depthWrite: false,
      depthTest: true
    });

    this.predictionLines = new THREE.LineSegments(this.predictionGeometry, this.predictionMaterial);
    this.predictionLines.name = 'AI_OPPONENT_PREDICTION_BOXES';
    this.predictionLines.frustumCulled = false;
    this.group.add(this.predictionLines);
  }

  _initTacticalCorridorRibbon() {
    // 3D Quad strip ribbon for active tactical corridor
    const maxVertices = (CORRIDOR_SEGMENTS - 1) * 6;
    const positions = new Float32Array(maxVertices * 3);
    const colors = new Float32Array(maxVertices * 3);

    this.corridorGeometry = new THREE.BufferGeometry();
    this.corridorGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    this.corridorGeometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    this.corridorGeometry.setDrawRange(0, 0);

    this.corridorMaterial = new THREE.MeshBasicMaterial({
      vertexColors: true,
      transparent: true,
      opacity: 0.65,
      side: THREE.DoubleSide,
      depthWrite: false,
      depthTest: true
    });

    this.corridorMesh = new THREE.Mesh(this.corridorGeometry, this.corridorMaterial);
    this.corridorMesh.name = 'AI_TACTICAL_CORRIDOR_RIBBON';
    this.corridorMesh.frustumCulled = false;
    this.corridorMesh.visible = false;
    this.group.add(this.corridorMesh);
  }

  _initBrakingPointMarker() {
    this.brakingGroup = new THREE.Group();
    this.brakingGroup.name = 'AI_BRAKING_POINT_MARKER';

    // Transverse road bar across track
    const barGeo = new THREE.PlaneGeometry(3.6, 0.45);
    barGeo.rotateX(-Math.PI / 2);
    const barMat = new THREE.MeshBasicMaterial({
      color: 0xffe15a,
      transparent: true,
      opacity: 0.9,
      side: THREE.DoubleSide,
      depthWrite: false
    });
    this.brakingBar = new THREE.Mesh(barGeo, barMat);
    this.brakingBar.position.y = 0.08;

    // Glowing chevron / pointer mesh
    const pointerGeo = new THREE.ConeGeometry(0.5, 1.2, 4);
    pointerGeo.rotateX(Math.PI);
    const pointerMat = new THREE.MeshBasicMaterial({
      color: 0xff3d00,
      transparent: true,
      opacity: 0.92,
      depthWrite: false
    });
    this.brakingPointer = new THREE.Mesh(pointerGeo, pointerMat);
    this.brakingPointer.position.y = 1.4;

    this.brakingGroup.add(this.brakingBar, this.brakingPointer);
    this.brakingGroup.visible = false;
    this.group.add(this.brakingGroup);
  }

  /**
   * Main update loop for Tactical Visualizations
   */
  update(aiController, aiVehicle, vehicles = [], track = null, now = 0) {
    if (!this.visible || !aiController) {
      this._hideAll();
      return;
    }

    const state = aiController.debugState ?? {};
    const traffic = state.traffic ?? aiController.awareness?.lastScan ?? null;

    // 1. Update Opponent Prediction Cones & Bounding Boxes
    this._updatePredictionCones(aiVehicle, vehicles, traffic, track, now);

    // 2. Update Tactical Corridor Highlight Ribbon
    this._updateTacticalCorridor(aiController, aiVehicle, state, track);

    // 3. Update Dynamic Divebomb / Braking Point Marker
    this._updateBrakingMarker(aiController, aiVehicle, state, track, now);
  }

  _updatePredictionCones(egoVehicle, vehicles, traffic, track, now) {
    const posAttr = this.predictionGeometry.attributes.position;
    const colAttr = this.predictionGeometry.attributes.color;
    const posArr = posAttr.array;
    const colArr = colAttr.array;

    let vertIndex = 0;
    const halfW = this.carDimensions.width * 0.5;
    const halfL = this.carDimensions.length * 0.5;
    const height = this.carDimensions.height;

    // Opponent entries from traffic scan or direct vehicle list
    const trafficEntries = traffic?.entries ?? [];
    let processedCount = 0;

    for (const entry of trafficEntries) {
      if (processedCount >= MAX_OPPONENTS) break;
      const other = entry.other;
      if (!other || other === egoVehicle || other.finished || other.despawned || other.trafficGhost) continue;

      const otherDist = finite(other.distance);
      const otherSpeed = Math.max(0, finite(other.speed));
      const otherLateral = finite(other.surface?.lateral, entry.otherLateral ?? 0);
      const targetLateral = finite(other.aiTactical?.targetLaneOffsetM, finite(other.aiTarget?.lateral, otherLateral));

      let prevX = finite(other.position?.x);
      let prevY = finite(other.position?.y) + 0.1;
      let prevZ = finite(other.position?.z);

      // Render wireframe boxes across time horizons
      for (let h = 0; h < TIME_HORIZONS.length; h++) {
        const timeS = TIME_HORIZONS[h];
        const color = HORIZON_COLORS[h];

        // Projected track distance and lateral offset
        const predDist = otherDist + otherSpeed * timeS;
        const lateralBlend = clamp(timeS / 1.35, 0, 1);
        const predLateral = otherLateral + (targetLateral - otherLateral) * (lateralBlend * lateralBlend * (3 - 2 * lateralBlend));

        let centerX, centerY, centerZ, yaw;

        if (track && typeof track.atDistance === 'function') {
          const ref = track.atDistance(predDist);
          const world = track.lateralPoint ? track.lateralPoint(ref, predLateral, 0.05) : ref;
          centerX = finite(world.x);
          centerY = finite(world.y);
          centerZ = finite(world.z);
          yaw = Math.atan2(finite(ref.tangent?.x), finite(ref.tangent?.z));
        } else {
          // Linear projection fallback
          const heading = finite(other.yaw);
          centerX = prevX + Math.sin(heading) * otherSpeed * (timeS - (h > 0 ? TIME_HORIZONS[h - 1] : 0));
          centerY = prevY;
          centerZ = prevZ + Math.cos(heading) * otherSpeed * (timeS - (h > 0 ? TIME_HORIZONS[h - 1] : 0));
          yaw = heading;
        }

        // Draw prediction connecting line from previous point to current center
        posArr[vertIndex * 3 + 0] = prevX; posArr[vertIndex * 3 + 1] = prevY; posArr[vertIndex * 3 + 2] = prevZ;
        colArr[vertIndex * 3 + 0] = color.r * 0.6; colArr[vertIndex * 3 + 1] = color.g * 0.6; colArr[vertIndex * 3 + 2] = color.b * 0.6;
        vertIndex++;

        posArr[vertIndex * 3 + 0] = centerX; posArr[vertIndex * 3 + 1] = centerY + 0.1; posArr[vertIndex * 3 + 2] = centerZ;
        colArr[vertIndex * 3 + 0] = color.r; colArr[vertIndex * 3 + 1] = color.g; colArr[vertIndex * 3 + 2] = color.b;
        vertIndex++;

        prevX = centerX;
        prevY = centerY + 0.1;
        prevZ = centerZ;

        // Construct 8 corners of the 3D bounding box
        const sinY = Math.sin(yaw);
        const cosY = Math.cos(yaw);

        // Local box corners: [x, y, z]
        const corners = [
          [-halfW, 0, -halfL], [ halfW, 0, -halfL], [ halfW, 0,  halfL], [-halfW, 0,  halfL], // Bottom 0,1,2,3
          [-halfW, height, -halfL], [ halfW, height, -halfL], [ halfW, height,  halfL], [-halfW, height,  halfL]  // Top 4,5,6,7
        ];

        const worldCorners = corners.map(([lx, ly, lz]) => ({
          x: centerX + (lx * cosY + lz * sinY),
          y: centerY + ly,
          z: centerZ + (-lx * sinY + lz * cosY)
        }));

        // 12 edges (24 vertices): 4 bottom, 4 top, 4 vertical pillars
        const edges = [
          [0, 1], [1, 2], [2, 3], [3, 0], // Bottom
          [4, 5], [5, 6], [6, 7], [7, 4], // Top
          [0, 4], [1, 5], [2, 6], [3, 7]  // Vertical
        ];

        for (const [i0, i1] of edges) {
          const p0 = worldCorners[i0];
          const p1 = worldCorners[i1];

          posArr[vertIndex * 3 + 0] = p0.x; posArr[vertIndex * 3 + 1] = p0.y; posArr[vertIndex * 3 + 2] = p0.z;
          colArr[vertIndex * 3 + 0] = color.r; colArr[vertIndex * 3 + 1] = color.g; colArr[vertIndex * 3 + 2] = color.b;
          vertIndex++;

          posArr[vertIndex * 3 + 0] = p1.x; posArr[vertIndex * 3 + 1] = p1.y; posArr[vertIndex * 3 + 2] = p1.z;
          colArr[vertIndex * 3 + 0] = color.r; colArr[vertIndex * 3 + 1] = color.g; colArr[vertIndex * 3 + 2] = color.b;
          vertIndex++;
        }
      }

      processedCount++;
    }

    posAttr.needsUpdate = true;
    colAttr.needsUpdate = true;
    this.predictionGeometry.setDrawRange(0, vertIndex);
    this.predictionLines.visible = vertIndex > 0;
  }

  _updateTacticalCorridor(aiController, egoVehicle, state, track) {
    if (!track || typeof track.atDistance !== 'function') {
      this.corridorMesh.visible = false;
      return;
    }

    const tactical = egoVehicle?.aiTactical ?? {};
    const thought = state.thought ?? {};
    const phase = tactical.racecraftPhase ?? tactical.passPhase ?? state.mode ?? 'PACE';

    const isAttack = ['ATTACK_LEFT', 'ATTACK_RIGHT', 'ATTACK_INSIDE', 'ATTACK_OUTSIDE', 'DIVEBOMB', 'SWITCHBACK'].includes(phase)
      || ['ATTACK_LEFT', 'ATTACK_RIGHT', 'ATTACK_INSIDE', 'ATTACK_OUTSIDE', 'DIVEBOMB', 'SWITCHBACK'].includes(thought.deployedManeuver);
    const isDefend = ['DEFEND_LEFT', 'DEFEND_RIGHT', 'DEFEND_INSIDE', 'BREAK_TOW', 'DEFEND'].includes(phase)
      || tactical.defending || thought.defending;
    const isReturn = phase === 'RETURN';

    if (!isAttack && !isDefend && !isReturn) {
      this.corridorMesh.visible = false;
      return;
    }

    const palette = CORRIDOR_COLORS[phase] || CORRIDOR_COLORS[thought.deployedManeuver] || (isDefend ? CORRIDOR_COLORS.DEFEND : CORRIDOR_COLORS.DEFAULT);
    const targetOffset = finite(tactical.targetLaneOffsetM, finite(state.targetOffset, 0));
    const startDist = finite(egoVehicle.distance);
    const corridorLength = 36.0; // Metres ahead

    const posAttr = this.corridorGeometry.attributes.position;
    const colAttr = this.corridorGeometry.attributes.color;
    const posArr = posAttr.array;
    const colArr = colAttr.array;

    let vertIndex = 0;
    const halfWidth = CORRIDOR_WIDTH * 0.5;

    for (let s = 0; s < CORRIDOR_SEGMENTS - 1; s++) {
      const d0 = startDist + (s / (CORRIDOR_SEGMENTS - 1)) * corridorLength;
      const d1 = startDist + ((s + 1) / (CORRIDOR_SEGMENTS - 1)) * corridorLength;

      const ref0 = track.atDistance(d0);
      const ref1 = track.atDistance(d1);

      const pt0 = track.lateralPoint(ref0, targetOffset, 0.05);
      const pt1 = track.lateralPoint(ref1, targetOffset, 0.05);

      const norm0_x = -finite(ref0.tangent?.z);
      const norm0_z = finite(ref0.tangent?.x);
      const norm1_x = -finite(ref1.tangent?.z);
      const norm1_z = finite(ref1.tangent?.x);

      // Segment start corners
      const p0_L_x = pt0.x - norm0_x * halfWidth;
      const p0_L_y = pt0.y;
      const p0_L_z = pt0.z - norm0_z * halfWidth;

      const p0_R_x = pt0.x + norm0_x * halfWidth;
      const p0_R_y = pt0.y;
      const p0_R_z = pt0.z + norm0_z * halfWidth;

      // Segment end corners
      const p1_L_x = pt1.x - norm1_x * halfWidth;
      const p1_L_y = pt1.y;
      const p1_L_z = pt1.z - norm1_z * halfWidth;

      const p1_R_x = pt1.x + norm1_x * halfWidth;
      const p1_R_y = pt1.y;
      const p1_R_z = pt1.z + norm1_z * halfWidth;

      const alpha0 = 1.0 - (s / CORRIDOR_SEGMENTS);
      const alpha1 = 1.0 - ((s + 1) / CORRIDOR_SEGMENTS);

      // Triangle 1
      posArr[vertIndex * 3 + 0] = p0_L_x; posArr[vertIndex * 3 + 1] = p0_L_y; posArr[vertIndex * 3 + 2] = p0_L_z;
      colArr[vertIndex * 3 + 0] = palette.base.r * alpha0; colArr[vertIndex * 3 + 1] = palette.base.g * alpha0; colArr[vertIndex * 3 + 2] = palette.base.b * alpha0;
      vertIndex++;

      posArr[vertIndex * 3 + 0] = p0_R_x; posArr[vertIndex * 3 + 1] = p0_R_y; posArr[vertIndex * 3 + 2] = p0_R_z;
      colArr[vertIndex * 3 + 0] = palette.base.r * alpha0; colArr[vertIndex * 3 + 1] = palette.base.g * alpha0; colArr[vertIndex * 3 + 2] = palette.base.b * alpha0;
      vertIndex++;

      posArr[vertIndex * 3 + 0] = p1_L_x; posArr[vertIndex * 3 + 1] = p1_L_y; posArr[vertIndex * 3 + 2] = p1_L_z;
      colArr[vertIndex * 3 + 0] = palette.top.r * alpha1; colArr[vertIndex * 3 + 1] = palette.top.g * alpha1; colArr[vertIndex * 3 + 2] = palette.top.b * alpha1;
      vertIndex++;

      // Triangle 2
      posArr[vertIndex * 3 + 0] = p0_R_x; posArr[vertIndex * 3 + 1] = p0_R_y; posArr[vertIndex * 3 + 2] = p0_R_z;
      colArr[vertIndex * 3 + 0] = palette.base.r * alpha0; colArr[vertIndex * 3 + 1] = palette.base.g * alpha0; colArr[vertIndex * 3 + 2] = palette.base.b * alpha0;
      vertIndex++;

      posArr[vertIndex * 3 + 0] = p1_R_x; posArr[vertIndex * 3 + 1] = p1_R_y; posArr[vertIndex * 3 + 2] = p1_R_z;
      colArr[vertIndex * 3 + 0] = palette.top.r * alpha1; colArr[vertIndex * 3 + 1] = palette.top.g * alpha1; colArr[vertIndex * 3 + 2] = palette.top.b * alpha1;
      vertIndex++;

      posArr[vertIndex * 3 + 0] = p1_L_x; posArr[vertIndex * 3 + 1] = p1_L_y; posArr[vertIndex * 3 + 2] = p1_L_z;
      colArr[vertIndex * 3 + 0] = palette.top.r * alpha1; colArr[vertIndex * 3 + 1] = palette.top.g * alpha1; colArr[vertIndex * 3 + 2] = palette.top.b * alpha1;
      vertIndex++;
    }

    posAttr.needsUpdate = true;
    colAttr.needsUpdate = true;
    this.corridorGeometry.setDrawRange(0, vertIndex);
    this.corridorMesh.visible = true;
  }

  _updateBrakingMarker(aiController, egoVehicle, state, track, now) {
    if (!track || typeof track.atDistance !== 'function') {
      this.brakingGroup.visible = false;
      return;
    }

    const currentSpeed = finite(egoVehicle?.speed);
    const tactical = egoVehicle?.aiTactical ?? {};
    const desiredSpeed = finite(tactical.desiredSpeed, finite(state.desiredSpeed, currentSpeed));

    // Deceleration budget ~ 14.5 m/s²
    const aBrake = 14.5;
    const speedDelta = currentSpeed - desiredSpeed;

    // Show braking marker if deceleration is required ahead
    if (speedDelta > 3.5 && currentSpeed > 15) {
      const brakeDistance = (currentSpeed * currentSpeed - desiredSpeed * desiredSpeed) / (2 * aBrake);
      const brakeTriggerDist = finite(egoVehicle.distance) + Math.max(2, brakeDistance);

      const ref = track.atDistance(brakeTriggerDist);
      const lateral = finite(state.targetOffset, 0);
      const world = track.lateralPoint ? track.lateralPoint(ref, lateral, 0.06) : ref;

      this.brakingGroup.position.set(finite(world.x), finite(world.y), finite(world.z));
      const heading = Math.atan2(finite(ref.tangent?.x), finite(ref.tangent?.z));
      this.brakingGroup.rotation.y = heading;

      // Pulse pointer height / intensity
      const pulse = 1.0 + Math.sin(now * 0.012) * 0.25;
      this.brakingPointer.position.y = 1.2 + pulse * 0.3;
      this.brakingBar.material.opacity = 0.75 + Math.sin(now * 0.015) * 0.2;

      this.brakingGroup.visible = true;
    } else {
      this.brakingGroup.visible = false;
    }
  }

  _hideAll() {
    this.predictionLines.visible = false;
    this.corridorMesh.visible = false;
    this.brakingGroup.visible = false;
  }

  setVisible(visible) {
    this.visible = Boolean(visible);
    this.group.visible = this.visible;
    if (!this.visible) {
      this._hideAll();
    }
    return this.visible;
  }

  dispose() {
    this._hideAll();
    this.predictionGeometry.dispose();
    this.predictionMaterial.dispose();
    this.corridorGeometry.dispose();
    this.corridorMaterial.dispose();
    this.brakingBar.geometry.dispose();
    this.brakingBar.material.dispose();
    this.brakingPointer.geometry.dispose();
    this.brakingPointer.material.dispose();
    this.group.removeFromParent();
  }
}
