import * as THREE from 'three';

const MAX_CANDIDATES = 16;
const MAX_POINTS = 32;

const MODE_PALETTES = {
  ATTACK: { primary: new THREE.Color(0x00ff88), secondary: new THREE.Color(0x76ff03), emissive: new THREE.Color(0x00cc66) },
  DEFEND: { primary: new THREE.Color(0xf05cff), secondary: new THREE.Color(0xd020ff), emissive: new THREE.Color(0xaa00cc) },
  PACE: { primary: new THREE.Color(0x00f0ff), secondary: new THREE.Color(0x35e6ed), emissive: new THREE.Color(0x0099cc) },
  BRAKE: { primary: new THREE.Color(0xffe15a), secondary: new THREE.Color(0xff9b3d), emissive: new THREE.Color(0xcc9900) },
  FOLLOW: { primary: new THREE.Color(0x4d8dff), secondary: new THREE.Color(0x82b1ff), emissive: new THREE.Color(0x2962ff) },
  AVOID: { primary: new THREE.Color(0xff6d00), secondary: new THREE.Color(0xffab40), emissive: new THREE.Color(0xd50000) },
  RECOVER: { primary: new THREE.Color(0xff3d71), secondary: new THREE.Color(0xff708d), emissive: new THREE.Color(0xc2185b) },
  DEFAULT: { primary: new THREE.Color(0x00f0ff), secondary: new THREE.Color(0x35e6ed), emissive: new THREE.Color(0x00a0b0) }
};

const ALT_COLORS = {
  VIABLE: new THREE.Color(0x00d4ff),
  BLOCKED: new THREE.Color(0xff3355),
  EDGE_RISK: new THREE.Color(0xffa726),
  DEFAULT: new THREE.Color(0x546e7a)
};

const finite = (val, fallback = 0) => (Number.isFinite(val) ? val : fallback);
const clamp = (val, min, max) => Math.max(min, Math.min(max, val));

/**
 * 3D Multi-Candidate Frenet Trajectory Lattice Renderer
 * Visualizes the chosen trajectory as a glowing 3D ribbon with vertex speed coloring
 * and alternate candidates as semi-transparent lines categorized by viability.
 */
export class CandidateSplineRenderer {
  constructor(scene, options = {}) {
    this.scene = scene;
    this.group = new THREE.Group();
    this.group.name = 'AI_CANDIDATE_SPLINES';
    this.visible = true;
    this.ribbonWidth = options.ribbonWidth ?? 0.65;

    // Temporary vectors to avoid garbage collection
    this._vPrev = new THREE.Vector3();
    this._vCurr = new THREE.Vector3();
    this._vNext = new THREE.Vector3();
    this._vTangent = new THREE.Vector3();
    this._vNormal = new THREE.Vector3(0, 1, 0);
    this._vBinormal = new THREE.Vector3();
    this._tempColor = new THREE.Color();
    this._speedColor = new THREE.Color();

    this._initChosenRibbon();
    this._initChosenCenterLine();
    this._initAlternateCandidates();
    this._initTargetWaypoints();

    if (scene) {
      scene.add(this.group);
    }
  }

  _initChosenRibbon() {
    // 3D Quad strip ribbon: (MAX_POINTS - 1) segments -> 2 triangles (6 vertices) per segment
    const maxVertices = (MAX_POINTS - 1) * 6;
    const positions = new Float32Array(maxVertices * 3);
    const colors = new Float32Array(maxVertices * 3);
    const normals = new Float32Array(maxVertices * 3);

    // Default normals pointing up
    for (let i = 0; i < maxVertices; i++) {
      normals[i * 3 + 0] = 0;
      normals[i * 3 + 1] = 1;
      normals[i * 3 + 2] = 0;
    }

    this.ribbonGeometry = new THREE.BufferGeometry();
    this.ribbonGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    this.ribbonGeometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    this.ribbonGeometry.setAttribute('normal', new THREE.BufferAttribute(normals, 3));
    this.ribbonGeometry.setDrawRange(0, 0);

    this.ribbonMaterial = new THREE.MeshBasicMaterial({
      vertexColors: true,
      transparent: true,
      opacity: 0.88,
      side: THREE.DoubleSide,
      depthWrite: false,
      depthTest: true
    });

    this.ribbonMesh = new THREE.Mesh(this.ribbonGeometry, this.ribbonMaterial);
    this.ribbonMesh.name = 'AI_CHOSEN_RIBBON';
    this.ribbonMesh.frustumCulled = false;
    this.group.add(this.ribbonMesh);
  }

  _initChosenCenterLine() {
    const positions = new Float32Array(MAX_POINTS * 3);
    const colors = new Float32Array(MAX_POINTS * 3);

    this.chosenLineGeometry = new THREE.BufferGeometry();
    this.chosenLineGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    this.chosenLineGeometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    this.chosenLineGeometry.setDrawRange(0, 0);

    this.chosenLineMaterial = new THREE.LineBasicMaterial({
      vertexColors: true,
      transparent: true,
      opacity: 0.98,
      linewidth: 2,
      depthWrite: false
    });

    this.chosenLine = new THREE.Line(this.chosenLineGeometry, this.chosenLineMaterial);
    this.chosenLine.name = 'AI_CHOSEN_CENTER_LINE';
    this.chosenLine.frustumCulled = false;
    this.group.add(this.chosenLine);
  }

  _initAlternateCandidates() {
    this.alternateLines = [];
    for (let c = 0; c < MAX_CANDIDATES; c++) {
      const positions = new Float32Array(MAX_POINTS * 3);
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
      geometry.setDrawRange(0, 0);

      const material = new THREE.LineBasicMaterial({
        color: ALT_COLORS.DEFAULT,
        transparent: true,
        opacity: 0.45,
        depthWrite: false,
        depthTest: true
      });

      const line = new THREE.Line(geometry, material);
      line.name = `AI_ALT_CANDIDATE_${c}`;
      line.frustumCulled = false;
      line.visible = false;
      this.group.add(line);

      this.alternateLines.push({
        line,
        geometry,
        material
      });
    }
  }

  _initTargetWaypoints() {
    // Tracking aim point sphere
    this.trackingGeometry = new THREE.SphereGeometry(0.38, 12, 8);
    this.trackingMaterial = new THREE.MeshBasicMaterial({
      color: 0x00ff88,
      transparent: true,
      opacity: 0.95,
      depthWrite: false
    });
    this.trackingMarker = new THREE.Mesh(this.trackingGeometry, this.trackingMaterial);
    this.trackingMarker.name = 'AI_TRACKING_WAYPOINT';
    this.trackingMarker.visible = false;
    this.trackingMarker.frustumCulled = false;

    // Horizon terminal waypoint ring / sphere
    this.horizonGeometry = new THREE.RingGeometry(0.35, 0.55, 16);
    this.horizonMaterial = new THREE.MeshBasicMaterial({
      color: 0x00f0ff,
      transparent: true,
      opacity: 0.85,
      side: THREE.DoubleSide,
      depthWrite: false
    });
    this.horizonMarker = new THREE.Mesh(this.horizonGeometry, this.horizonMaterial);
    this.horizonMarker.name = 'AI_HORIZON_WAYPOINT';
    this.horizonMarker.rotation.x = -Math.PI / 2;
    this.horizonMarker.visible = false;
    this.horizonMarker.frustumCulled = false;

    this.group.add(this.trackingMarker, this.horizonMarker);
  }

  /**
   * Evaluates vertex speed color for a trajectory point
   */
  _computeSpeedColor(speed, maxSpeed = 75, mode = 'PACE', outColor) {
    const palette = MODE_PALETTES[mode] ?? MODE_PALETTES.DEFAULT;
    const normSpeed = clamp(speed / Math.max(15, maxSpeed), 0, 1);

    if (mode === 'BRAKE' || (speed < 12 && normSpeed < 0.25)) {
      // Yellow to Orange/Red for heavy braking
      outColor.copy(MODE_PALETTES.BRAKE.primary).lerp(MODE_PALETTES.BRAKE.secondary, 1 - normSpeed);
    } else if (mode === 'ATTACK') {
      // Emerald green to lime for attack
      outColor.copy(palette.primary).lerp(palette.secondary, normSpeed);
    } else if (mode === 'DEFEND') {
      // Magenta to purple for defense
      outColor.copy(palette.primary).lerp(palette.secondary, normSpeed);
    } else {
      // Cyan to high-speed turquoise/green
      outColor.copy(palette.primary).lerp(palette.secondary, normSpeed);
    }
    return outColor;
  }

  /**
   * Update all candidate splines and target waypoints
   */
  update(aiController, aiVehicle, track, now = 0) {
    if (!this.visible || !aiController) {
      this._hideAll();
      return;
    }

    const state = aiController.debugState ?? {};
    const trajectory = state.trajectory ?? aiController.trajectoryPlan ?? {};
    const path = trajectory.points ?? state.planPath ?? state.path ?? [];
    const mode = this._resolveMode(state, aiController);

    if (!Array.isArray(path) || path.length < 2) {
      this._hideAll();
      return;
    }

    // 1. Update Chosen Trajectory Ribbon & Center Line
    this._updateChosenRibbon(path, mode, aiVehicle);
    this._updateChosenCenterLine(path, mode);

    // 2. Update Alternate Candidates Lattice
    this._updateAlternateCandidates(aiController, state, path, mode);

    // 3. Update Target Waypoint Markers
    this._updateTargetWaypoints(state, trajectory, path, mode, now);
  }

  _resolveMode(state, controller) {
    const rawMode = state.mode ?? controller.passPhase ?? state.thought?.deployedManeuver ?? 'PACE';
    const upper = String(rawMode).toUpperCase();
    if (upper.includes('ATTACK') || upper.includes('DIVEBOMB') || upper.includes('PASS')) return 'ATTACK';
    if (upper.includes('DEFEND')) return 'DEFEND';
    if (upper.includes('BRAKE')) return 'BRAKE';
    if (upper.includes('FOLLOW')) return 'FOLLOW';
    if (upper.includes('AVOID')) return 'AVOID';
    if (upper.includes('RECOVER')) return 'RECOVER';
    return 'PACE';
  }

  _updateChosenRibbon(path, mode, vehicle) {
    const pointCount = Math.min(MAX_POINTS, path.length);
    if (pointCount < 2) {
      this.ribbonGeometry.setDrawRange(0, 0);
      return;
    }

    const posAttr = this.ribbonGeometry.attributes.position;
    const colAttr = this.ribbonGeometry.attributes.color;
    const posArr = posAttr.array;
    const colArr = colAttr.array;
    const halfWidth = this.ribbonWidth * 0.5;

    let vertIndex = 0;
    const maxSpeed = Math.max(30, finite(vehicle?.speed, 45) * 1.3);

    for (let i = 0; i < pointCount - 1; i++) {
      const p0 = path[i];
      const p1 = path[i + 1];

      // Tangent vector
      this._vCurr.set(finite(p0.x), finite(p0.y) + 0.05, finite(p0.z));
      this._vNext.set(finite(p1.x), finite(p1.y) + 0.05, finite(p1.z));
      this._vTangent.subVectors(this._vNext, this._vCurr).normalize();

      if (this._vTangent.lengthSq() < 1e-4) {
        this._vTangent.set(0, 0, 1);
      }

      // Binormal (perpendicular on horizontal plane)
      this._vBinormal.crossVectors(this._vTangent, this._vNormal).normalize();

      // Ribbon vertices for current segment
      // Segment start: Left & Right
      const p0_L_x = this._vCurr.x - this._vBinormal.x * halfWidth;
      const p0_L_y = this._vCurr.y;
      const p0_L_z = this._vCurr.z - this._vBinormal.z * halfWidth;

      const p0_R_x = this._vCurr.x + this._vBinormal.x * halfWidth;
      const p0_R_y = this._vCurr.y;
      const p0_R_z = this._vCurr.z + this._vBinormal.z * halfWidth;

      // Segment end: Left & Right
      const p1_L_x = this._vNext.x - this._vBinormal.x * halfWidth;
      const p1_L_y = this._vNext.y;
      const p1_L_z = this._vNext.z - this._vBinormal.z * halfWidth;

      const p1_R_x = this._vNext.x + this._vBinormal.x * halfWidth;
      const p1_R_y = this._vNext.y;
      const p1_R_z = this._vNext.z + this._vBinormal.z * halfWidth;

      // Colors for p0 and p1
      const speed0 = finite(p0.predictedSpeed ?? p0.speed, 25);
      const speed1 = finite(p1.predictedSpeed ?? p1.speed, 25);
      this._computeSpeedColor(speed0, maxSpeed, mode, this._speedColor);
      const c0_r = this._speedColor.r;
      const c0_g = this._speedColor.g;
      const c0_b = this._speedColor.b;

      this._computeSpeedColor(speed1, maxSpeed, mode, this._speedColor);
      const c1_r = this._speedColor.r;
      const c1_g = this._speedColor.g;
      const c1_b = this._speedColor.b;

      // Triangle 1: p0_L, p0_R, p1_L
      posArr[vertIndex * 3 + 0] = p0_L_x; posArr[vertIndex * 3 + 1] = p0_L_y; posArr[vertIndex * 3 + 2] = p0_L_z;
      colArr[vertIndex * 3 + 0] = c0_r;   colArr[vertIndex * 3 + 1] = c0_g;   colArr[vertIndex * 3 + 2] = c0_b;
      vertIndex++;

      posArr[vertIndex * 3 + 0] = p0_R_x; posArr[vertIndex * 3 + 1] = p0_R_y; posArr[vertIndex * 3 + 2] = p0_R_z;
      colArr[vertIndex * 3 + 0] = c0_r;   colArr[vertIndex * 3 + 1] = c0_g;   colArr[vertIndex * 3 + 2] = c0_b;
      vertIndex++;

      posArr[vertIndex * 3 + 0] = p1_L_x; posArr[vertIndex * 3 + 1] = p1_L_y; posArr[vertIndex * 3 + 2] = p1_L_z;
      colArr[vertIndex * 3 + 0] = c1_r;   colArr[vertIndex * 3 + 1] = c1_g;   colArr[vertIndex * 3 + 2] = c1_b;
      vertIndex++;

      // Triangle 2: p0_R, p1_R, p1_L
      posArr[vertIndex * 3 + 0] = p0_R_x; posArr[vertIndex * 3 + 1] = p0_R_y; posArr[vertIndex * 3 + 2] = p0_R_z;
      colArr[vertIndex * 3 + 0] = c0_r;   colArr[vertIndex * 3 + 1] = c0_g;   colArr[vertIndex * 3 + 2] = c0_b;
      vertIndex++;

      posArr[vertIndex * 3 + 0] = p1_R_x; posArr[vertIndex * 3 + 1] = p1_R_y; posArr[vertIndex * 3 + 2] = p1_R_z;
      colArr[vertIndex * 3 + 0] = c1_r;   colArr[vertIndex * 3 + 1] = c1_g;   colArr[vertIndex * 3 + 2] = c1_b;
      vertIndex++;

      posArr[vertIndex * 3 + 0] = p1_L_x; posArr[vertIndex * 3 + 1] = p1_L_y; posArr[vertIndex * 3 + 2] = p1_L_z;
      colArr[vertIndex * 3 + 0] = c1_r;   colArr[vertIndex * 3 + 1] = c1_g;   colArr[vertIndex * 3 + 2] = c1_b;
      vertIndex++;
    }

    posAttr.needsUpdate = true;
    colAttr.needsUpdate = true;
    this.ribbonGeometry.setDrawRange(0, vertIndex);
    this.ribbonMesh.visible = true;
  }

  _updateChosenCenterLine(path, mode) {
    const pointCount = Math.min(MAX_POINTS, path.length);
    const posAttr = this.chosenLineGeometry.attributes.position;
    const colAttr = this.chosenLineGeometry.attributes.color;
    const posArr = posAttr.array;
    const colArr = colAttr.array;

    for (let i = 0; i < pointCount; i++) {
      const p = path[i];
      posArr[i * 3 + 0] = finite(p.x);
      posArr[i * 3 + 1] = finite(p.y) + 0.08;
      posArr[i * 3 + 2] = finite(p.z);

      const speed = finite(p.predictedSpeed ?? p.speed, 25);
      this._computeSpeedColor(speed, 75, mode, this._speedColor);
      // Brighten the center line
      this._speedColor.lerp(this._tempColor.set(0xffffff), 0.35);

      colArr[i * 3 + 0] = this._speedColor.r;
      colArr[i * 3 + 1] = this._speedColor.g;
      colArr[i * 3 + 2] = this._speedColor.b;
    }

    posAttr.needsUpdate = true;
    colAttr.needsUpdate = true;
    this.chosenLineGeometry.setDrawRange(0, pointCount);
    this.chosenLine.visible = true;
  }

  _updateAlternateCandidates(controller, state, chosenPath, mode) {
    // Check if candidates array is directly provided
    const candidates = controller.trajectoryPlanner?.lastCandidates
      ?? state.candidates
      ?? state.trajectory?.candidates
      ?? [];

    let renderedCount = 0;

    if (Array.isArray(candidates) && candidates.length > 0) {
      for (const cand of candidates) {
        if (renderedCount >= MAX_CANDIDATES) break;
        const candPoints = cand.points;
        if (!Array.isArray(candPoints) || candPoints.length < 2) continue;

        // Skip if this is the chosen path
        if (cand === state.trajectory || cand.selected) continue;

        const altEntry = this.alternateLines[renderedCount];
        const count = Math.min(MAX_POINTS, candPoints.length);
        const posArr = altEntry.geometry.attributes.position.array;

        for (let i = 0; i < count; i++) {
          const pt = candPoints[i];
          posArr[i * 3 + 0] = finite(pt.x);
          posArr[i * 3 + 1] = finite(pt.y) + 0.04;
          posArr[i * 3 + 2] = finite(pt.z);
        }

        altEntry.geometry.attributes.position.needsUpdate = true;
        altEntry.geometry.setDrawRange(0, count);

        // Viability styling
        if (!cand.collisionFree) {
          altEntry.material.color.copy(ALT_COLORS.BLOCKED);
          altEntry.material.opacity = 0.35;
        } else if (!cand.roadLegal || (cand.edgeRisk && cand.edgeRisk > 0.5)) {
          altEntry.material.color.copy(ALT_COLORS.EDGE_RISK);
          altEntry.material.opacity = 0.40;
        } else {
          altEntry.material.color.copy(ALT_COLORS.VIABLE);
          altEntry.material.opacity = 0.48;
        }

        altEntry.line.visible = true;
        renderedCount++;
      }
    }

    // Hide unused alternate lines
    for (let c = renderedCount; c < MAX_CANDIDATES; c++) {
      this.alternateLines[c].line.visible = false;
    }
  }

  _updateTargetWaypoints(state, trajectory, path, mode, now) {
    const palette = MODE_PALETTES[mode] ?? MODE_PALETTES.DEFAULT;

    // 1. Tracking point (Lookahead pursuit target)
    const trackingPt = trajectory.trackingPoint ?? state.target ?? path[Math.min(path.length - 1, 3)];
    if (trackingPt && Number.isFinite(trackingPt.x) && Number.isFinite(trackingPt.z)) {
      this.trackingMarker.position.set(
        finite(trackingPt.x),
        finite(trackingPt.y) + 0.22,
        finite(trackingPt.z)
      );
      this.trackingMaterial.color.copy(palette.primary);

      // Subtle breath/pulsate effect
      const pulse = 1.0 + Math.sin(now * 0.008) * 0.18;
      this.trackingMarker.scale.setScalar(pulse);
      this.trackingMarker.visible = true;
    } else {
      this.trackingMarker.visible = false;
    }

    // 2. Horizon terminal waypoint (End of trajectory horizon)
    const terminalPt = path[path.length - 1];
    if (terminalPt && Number.isFinite(terminalPt.x) && Number.isFinite(terminalPt.z)) {
      this.horizonMarker.position.set(
        finite(terminalPt.x),
        finite(terminalPt.y) + 0.12,
        finite(terminalPt.z)
      );
      this.horizonMaterial.color.copy(palette.secondary);
      this.horizonMarker.visible = true;
    } else {
      this.horizonMarker.visible = false;
    }
  }

  _hideAll() {
    this.ribbonMesh.visible = false;
    this.chosenLine.visible = false;
    this.trackingMarker.visible = false;
    this.horizonMarker.visible = false;
    for (const alt of this.alternateLines) {
      alt.line.visible = false;
    }
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
    this.ribbonGeometry.dispose();
    this.ribbonMaterial.dispose();
    this.chosenLineGeometry.dispose();
    this.chosenLineMaterial.dispose();

    for (const alt of this.alternateLines) {
      this.group.remove(alt.line);
      alt.geometry.dispose();
      alt.material.dispose();
    }
    this.alternateLines.length = 0;

    this.trackingGeometry.dispose();
    this.trackingMaterial.dispose();
    this.horizonGeometry.dispose();
    this.horizonMaterial.dispose();

    this.group.removeFromParent();
  }
}
