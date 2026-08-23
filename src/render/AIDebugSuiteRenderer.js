import * as THREE from 'three';
import { CandidateSplineRenderer } from './CandidateSplineRenderer.js';
import { TacticalZoneRenderer } from './TacticalZoneRenderer.js';
import { TelemetryHUD } from './TelemetryHUD.js';

const finite = (val, fallback = 0) => (Number.isFinite(val) ? val : fallback);

/**
 * AIDebugSuiteRenderer
 * Master coordinator for the 3D Visual AI Debug Suite & Telemetry Overlays.
 * Unifies Frenet candidate lattice ribbons, tactical opponent prediction cones,
 * track surface corridor highlights, dynamic divebomb/braking markers,
 * 3D floating thought sprites, and live G-G friction circle HUD.
 */
export class AIDebugSuiteRenderer {
  constructor(scene, track = null, options = {}) {
    this.scene = scene;
    this.track = track;
    this.visible = true;

    // Master Three.js Scene Group
    this.group = new THREE.Group();
    this.group.name = 'AI_DEBUG_SUITE_MASTER';

    // Layer Visibility State
    this.layers = {
      candidates: true,
      predictionCones: true,
      tacticalCorridors: true,
      brakingPoint: true,
      thoughtLabel: true,
      ggCircle: true,
      hud: true
    };

    // Sub-components
    this.candidateRenderer = new CandidateSplineRenderer(this.group, options);
    this.tacticalRenderer = new TacticalZoneRenderer(this.group, options);
    this.telemetryHUD = new TelemetryHUD(options);

    // 3D Trackside Incident Hotspot Markers
    this.incidentGroup = new THREE.Group();
    this.incidentGroup.name = 'AI_DEBUG_INCIDENT_MARKERS';
    this.group.add(this.incidentGroup);
    this.incidents = [];
    this.autoIncidentTimer = 0;

    // Connect HUD export / mark triggers
    if (this.telemetryHUD) {
      this.telemetryHUD.onMarkIncident = (note) => {
        if (this._lastVehicle && this._lastController) {
          this.markIncident(this._lastVehicle, this._lastController, this.track, 'MANUAL_FLAG', note);
        }
      };
      this.telemetryHUD.onExportIncidents = () => this.exportIncidentsJSON();
      this.telemetryHUD.onClearIncidents = () => this.clearIncidents();
    }

    // 3D Floating Thought Billboard Sprite
    this._initThoughtSprite();

    if (scene) {
      scene.add(this.group);
    }
  }

  _initThoughtSprite() {
    if (typeof document === 'undefined') {
      this.thoughtSprite = null;
      return;
    }

    const canvas = document.createElement('canvas');
    canvas.width = 640;
    canvas.height = 192;
    const context = canvas.getContext('2d');

    if (!context) {
      this.thoughtSprite = null;
      return;
    }

    const texture = new THREE.CanvasTexture(canvas);
    texture.colorSpace = THREE.SRGBColorSpace;
    texture.minFilter = THREE.LinearFilter;

    const material = new THREE.SpriteMaterial({
      map: texture,
      transparent: true,
      depthTest: false,
      depthWrite: false,
      sizeAttenuation: true
    });

    const sprite = new THREE.Sprite(material);
    sprite.name = 'AI_DEBUG_FLOATING_THOUGHT_SPRITE';
    sprite.renderOrder = 300;
    sprite.visible = false;
    sprite.scale.set(7.5, 2.25, 1);

    this.group.add(sprite);

    this.thought = {
      canvas,
      context,
      texture,
      material,
      sprite,
      signature: '',
      lastUpdateAt: -Infinity
    };
  }

  _updateThoughtSprite(aiVehicle, aiController, now) {
    if (!this.thought || !this.layers.thoughtLabel || !aiVehicle || !aiController) {
      if (this.thought?.sprite) this.thought.sprite.visible = false;
      return;
    }

    const state = aiController.debugState ?? {};
    const thoughtData = state.thought ?? {};
    const requested = thoughtData.requestedManeuver ?? state.racecraftPhase ?? 'PACE';
    const deployed = thoughtData.deployedManeuver && thoughtData.deployedManeuver !== 'NONE'
      ? thoughtData.deployedManeuver
      : state.mode ?? 'WAIT';
    const target = thoughtData.targetId ?? state.passTargetId ?? state.draftTargetId ?? 'CLEAR';
    const reason = thoughtData.abortReason ?? thoughtData.waitReason ?? state.reason ?? 'CLEAR';
    const clearance = finite(thoughtData.trajectoryMinimumClearanceM, finite(state.trajectoryMinimumClearanceM, 99));
    const safe = Boolean(thoughtData.trajectoryCollisionFree ?? state.trajectoryCollisionFree ?? true);

    const signature = `${aiVehicle.name}|${requested}|${deployed}|${target}|${reason}|${safe}|${clearance.toFixed(1)}`;
    const updateIntervalMs = 120; // Throttled texture upload for efficiency

    if (signature !== this.thought.signature && (now - this.thought.lastUpdateAt >= updateIntervalMs)) {
      this.thought.signature = signature;
      this.thought.lastUpdateAt = now;

      const ctx = this.thought.context;
      const w = this.thought.canvas.width;
      const h = this.thought.canvas.height;

      // Clear & Background
      ctx.clearRect(0, 0, w, h);
      ctx.fillStyle = 'rgba(4, 12, 16, 0.92)';
      ctx.strokeStyle = safe ? '#00f0ff' : '#ff3355';
      ctx.lineWidth = 4;
      ctx.fillRect(2, 2, w - 4, h - 4);
      ctx.strokeRect(3, 3, w - 6, h - 6);

      // Header Tag
      ctx.font = '700 28px Consolas, monospace';
      ctx.fillStyle = '#00ff88';
      ctx.fillText(`${aiVehicle.name ?? 'AI VEHICLE'} // ${String(aiVehicle.classKey ?? 'GT').toUpperCase()}`, 18, 38);

      // Tactical Plan
      ctx.font = '700 24px Consolas, monospace';
      ctx.fillStyle = '#ffffff';
      ctx.fillText(`PLAN: ${requested}`, 18, 74);

      // State & Safety Status
      ctx.fillStyle = safe ? '#00f0ff' : '#ff5252';
      ctx.fillText(`STATE: ${deployed} // ${safe ? 'COLLISION FREE' : 'BLOCKED'}`, 18, 108);

      // Target & Reason
      ctx.font = '600 20px Consolas, monospace';
      ctx.fillStyle = '#b0bec5';
      ctx.fillText(`TGT: ${String(target).toUpperCase()} | ${String(reason).slice(0, 32)}`, 18, 144);
      ctx.fillText(`MIN CLEARANCE: ${clearance < 90 ? clearance.toFixed(1) + 'M' : 'CLEAR'}`, 18, 174);

      this.thought.texture.needsUpdate = true;
    }

    // Position sprite above vehicle roof
    const pos = aiVehicle.position ?? { x: 0, y: 0, z: 0 };
    this.thought.sprite.position.set(finite(pos.x), finite(pos.y) + 3.6, finite(pos.z));
    this.thought.sprite.visible = true;
  }

  /**
   * Main update entry point for the AI Debug Suite
   * Flexible parameter handling for both (aiController, aiVehicle, playerOrVehicles, track, now)
   * and (rawDelta, aiVehicle, player, track, aiController).
   */
  update(aiControllerOrDelta, aiVehicle, playerOrVehicles = null, track = null, nowOrController = 0) {
    if (!this.visible) return;

    let aiController = aiControllerOrDelta;
    let time = 0;

    if (typeof aiControllerOrDelta === 'number' && typeof nowOrController === 'object' && nowOrController !== null) {
      aiController = nowOrController;
      time = typeof performance !== 'undefined' ? performance.now() : Date.now();
    } else {
      time = typeof nowOrController === 'number' && nowOrController > 0
        ? nowOrController
        : (typeof performance !== 'undefined' ? performance.now() : Date.now());
    }

    const currentTrack = track ?? this.track;
    const vehiclesList = Array.isArray(playerOrVehicles)
      ? playerOrVehicles
      : (playerOrVehicles ? [playerOrVehicles] : []);

    this._lastVehicle = aiVehicle;
    this._lastController = aiController;

    // 1. Update Candidate Spline Lattice
    if (this.layers.candidates && this.candidateRenderer) {
      this.candidateRenderer.update(aiController, aiVehicle, currentTrack, time);
    } else if (this.candidateRenderer) {
      this.candidateRenderer.setVisible(false);
    }

    // 2. Update Tactical Opponent Predictions, Corridor Strips & Braking Markers
    if (this.tacticalRenderer) {
      const showTactical = this.layers.predictionCones || this.layers.tacticalCorridors || this.layers.brakingPoint;
      this.tacticalRenderer.setVisible(showTactical);
      if (showTactical) {
        this.tacticalRenderer.update(aiController, aiVehicle, vehiclesList, currentTrack, time);
      }
    }

    // 3. Update 3D Floating Billboard Sprite
    this._updateThoughtSprite(aiVehicle, aiController, time);

    // 4. Automatic Hotspot Anomaly Detection (Spins, Hesitations, Track Excursions)
    this._checkAutoIncidents(aiVehicle, aiController, currentTrack, time);

    // 5. Update Telemetry HUD & G-G Friction Circle
    if (this.layers.hud && this.telemetryHUD) {
      this.telemetryHUD.update(aiVehicle, aiController, currentTrack, time);
    }
  }

  _checkAutoIncidents(aiVehicle, aiController, track, time) {
    if (!aiVehicle || !aiController) return;
    const nowS = time * 0.001;
    if (nowS - this.autoIncidentTimer < 1.2) return;

    const debug = aiController.debugState;
    const speed = finite(aiVehicle.speed, 0);
    const slip = Math.abs(finite(aiVehicle.localVelocity?.x, 0)) / Math.max(1, Math.abs(finite(aiVehicle.localVelocity?.z, 1)));
    const isSpin = Math.abs(finite(aiVehicle.yawRate, 0)) > 1.8 || slip > 0.35;
    const isOffTrack = aiVehicle.surface?.zone === 'grass' || aiVehicle.surface?.zone === 'runoff';
    const throttleHesitation = (debug?.speedError ?? 0) > 3.0 && (aiVehicle.controls?.throttle ?? 1) < 0.40 && (aiVehicle.controls?.brake ?? 0) < 0.05 && speed < 45;

    if (isSpin || isOffTrack || throttleHesitation) {
      const type = isSpin ? 'SPIN_LOSS_OF_CONTROL' : isOffTrack ? 'OFF_TRACK_EXCURSION' : 'THROTTLE_HESITATION';
      this.autoIncidentTimer = nowS;
      this.markIncident(aiVehicle, aiController, track, type, 'Auto-detected telemetry anomaly');
    }
  }

  /**
   * Mark and record an incident hotspot with full diagnostic telemetry and drop a 3D trackside beacon
   */
  markIncident(vehicle, controller, track, type = 'MANUAL_FLAG', note = '') {
    if (!vehicle || !controller) return null;

    const debug = controller.debugState;
    const trackLen = finite(track?.length, 3061.7);
    const trackDist = ((finite(vehicle.distance, 0) % trackLen) + trackLen) % trackLen;
    const speedKmh = finite(vehicle.speed, 0) * 3.6;
    const targetSpeedKmh = finite(debug?.targetSpeed, 0) * 3.6;
    const speedError = finite(debug?.speedError, 0);
    const lateral = finite(vehicle.surface?.lateral, 0);
    const zone = vehicle.surface?.zone || 'road';
    const slip = Math.abs(finite(vehicle.localVelocity?.x, 0)) / Math.max(1, Math.abs(finite(vehicle.localVelocity?.z, 1)));

    // Root Cause Diagnosis Classification
    let diagnosis = 'Normal operation anomaly';
    if (type === 'THROTTLE_HESITATION') {
      diagnosis = `TCS / Stability Cut active (Throttle: ${(vehicle.controls?.throttle * 100).toFixed(0)}%, Target: ${targetSpeedKmh.toFixed(0)} km/h)`;
    } else if (type === 'SPIN_LOSS_OF_CONTROL') {
      diagnosis = `Tire breakaway / Yaw snap (YawRate: ${vehicle.yawRate.toFixed(2)} rad/s, Slip: ${(slip * 57.3).toFixed(1)}°)`;
    } else if (type === 'OFF_TRACK_EXCURSION') {
      diagnosis = `Excursion into ${zone} (Lateral offset: ${lateral.toFixed(2)}m)`;
    } else {
      diagnosis = note || `Flagged by user at ${trackDist.toFixed(1)}m mark`;
    }

    const incident = {
      id: `inc-${Date.now()}-${Math.floor(Math.random() * 1000)}`,
      timestamp: new Date().toISOString(),
      type,
      distanceM: Number(trackDist.toFixed(1)),
      lap: Math.floor(vehicle.distance / trackLen) + 1,
      speedKmh: Number(speedKmh.toFixed(1)),
      targetSpeedKmh: Number(targetSpeedKmh.toFixed(1)),
      speedError: Number(speedError.toFixed(2)),
      lateralM: Number(lateral.toFixed(2)),
      zone,
      throttle: Number((vehicle.controls?.throttle ?? 0).toFixed(2)),
      brake: Number((vehicle.controls?.brake ?? 0).toFixed(2)),
      steer: Number((vehicle.controls?.steer ?? 0).toFixed(2)),
      slipAngleRad: Number(slip.toFixed(3)),
      yawRateRps: Number((vehicle.yawRate ?? 0).toFixed(2)),
      mode: debug?.mode || 'PACE',
      racecraftPhase: debug?.racecraftPhase || 'NONE',
      reason: debug?.reason || 'NONE',
      diagnosis,
      note
    };

    this.incidents.push(incident);

    // Create 3D Glowing Trackside Beacon
    this._createIncidentBeacon(vehicle.position, type, incident);

    // Notify Telemetry HUD
    if (this.telemetryHUD?.updateIncidents) {
      this.telemetryHUD.updateIncidents(this.incidents);
    }

    return incident;
  }

  _createIncidentBeacon(position, type, incident) {
    if (!position || typeof THREE === 'undefined') return;

    const color = type === 'SPIN_LOSS_OF_CONTROL' ? 0xff1744
      : type === 'THROTTLE_HESITATION' ? 0xffea00
      : type === 'OFF_TRACK_EXCURSION' ? 0xff9100 : 0x00e5ff;

    const beaconGroup = new THREE.Group();
    beaconGroup.position.set(finite(position.x), finite(position.y), finite(position.z));

    // Vertical Light Beam
    const beamGeo = new THREE.CylinderGeometry(0.12, 0.45, 8.0, 12, 1, true);
    const beamMat = new THREE.MeshBasicMaterial({
      color,
      transparent: true,
      opacity: 0.65,
      side: THREE.DoubleSide,
      depthWrite: false
    });
    const beam = new THREE.Mesh(beamGeo, beamMat);
    beam.position.y = 4.0;
    beaconGroup.add(beam);

    // Floating Diamond Head
    const diamondGeo = new THREE.OctahedronGeometry(0.75);
    const diamondMat = new THREE.MeshBasicMaterial({
      color,
      wireframe: false
    });
    const diamond = new THREE.Mesh(diamondGeo, diamondMat);
    diamond.position.y = 8.5;
    beaconGroup.add(diamond);

    // Ground Ring Pulse
    const ringGeo = new THREE.RingGeometry(0.8, 1.4, 24);
    const ringMat = new THREE.MeshBasicMaterial({
      color,
      transparent: true,
      opacity: 0.8,
      side: THREE.DoubleSide
    });
    const ring = new THREE.Mesh(ringGeo, ringMat);
    ring.rotation.x = -Math.PI * 0.5;
    ring.position.y = 0.08;
    beaconGroup.add(ring);

    this.incidentGroup.add(beaconGroup);
  }

  clearIncidents() {
    this.incidents = [];
    while (this.incidentGroup.children.length > 0) {
      const child = this.incidentGroup.children[0];
      this.incidentGroup.remove(child);
      child.traverse?.((obj) => {
        obj.geometry?.dispose();
        obj.material?.dispose();
      });
    }
    if (this.telemetryHUD?.updateIncidents) {
      this.telemetryHUD.updateIncidents(this.incidents);
    }
  }

  exportIncidentsJSON() {
    const payload = {
      version: 2,
      exportedAt: new Date().toISOString(),
      track: this.track?.name || 'Endurance Park',
      totalIncidents: this.incidents.length,
      incidents: this.incidents
    };

    const blob = new Blob([JSON.stringify(payload, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = `ai_hotspots_diagnostic_${Date.now()}.json`;
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
  }

  /**
   * Set master visibility of the entire debug suite
   */
  setVisible(visible) {
    this.visible = Boolean(visible);
    this.group.visible = this.visible;

    if (this.candidateRenderer) this.candidateRenderer.setVisible(this.visible && this.layers.candidates);
    if (this.tacticalRenderer) this.tacticalRenderer.setVisible(this.visible && (this.layers.predictionCones || this.layers.tacticalCorridors));
    if (this.thought?.sprite) this.thought.sprite.visible = this.visible && this.layers.thoughtLabel;
    if (this.telemetryHUD) this.telemetryHUD.setVisible(this.visible && this.layers.hud);

    return this.visible;
  }

  /**
   * Toggle individual layer visibility
   * @param {'candidates'|'predictionCones'|'tacticalCorridors'|'thoughtLabel'|'ggCircle'|'hud'|'brakingPoint'} layerName
   */
  toggleLayer(layerName) {
    const aliasMap = {
      splines: 'candidates',
      corridors: 'tacticalCorridors',
      prediction: 'predictionCones',
      predictionCones: 'predictionCones',
      thought: 'thoughtLabel',
      thoughtHUD: 'thoughtLabel',
      gg: 'ggCircle',
      ggCircle: 'ggCircle'
    };
    const key = aliasMap[layerName] ?? layerName;
    if (key in this.layers) {
      this.layers[key] = !this.layers[key];

      if (key === 'candidates' && this.candidateRenderer) {
        this.candidateRenderer.setVisible(this.layers.candidates && this.visible);
      } else if (key === 'hud' && this.telemetryHUD) {
        this.telemetryHUD.setVisible(this.layers.hud && this.visible);
      } else if (key === 'thoughtLabel' && this.thought?.sprite) {
        this.thought.sprite.visible = this.layers.thoughtLabel && this.visible;
      }
      return this.layers[key];
    }
    return false;
  }

  setLayerVisible(layerName, visible) {
    const aliasMap = {
      splines: 'candidates',
      corridors: 'tacticalCorridors',
      prediction: 'predictionCones',
      predictionCones: 'predictionCones',
      thought: 'thoughtLabel',
      thoughtHUD: 'thoughtLabel',
      gg: 'ggCircle',
      ggCircle: 'ggCircle'
    };
    const key = aliasMap[layerName] ?? layerName;
    if (key in this.layers) {
      this.layers[key] = Boolean(visible);
      if (key === 'candidates' && this.candidateRenderer) {
        this.candidateRenderer.setVisible(this.layers.candidates && this.visible);
      } else if (key === 'tacticalCorridors' && this.tacticalRenderer) {
        this.tacticalRenderer.setVisible(this.layers.tacticalCorridors && this.visible);
      } else if (key === 'thoughtLabel' && this.thought?.sprite) {
        this.thought.sprite.visible = this.layers.thoughtLabel && this.visible;
      } else if (key === 'hud' && this.telemetryHUD) {
        this.telemetryHUD.setVisible(this.layers.hud && this.visible);
      }
      return this.layers[key];
    }
    return false;
  }

  setLayerVisibility(layerName, visible) {
    return this.setLayerVisible(layerName, visible);
  }

  isLayerActive(layerName) {
    const aliasMap = {
      splines: 'candidates',
      corridors: 'tacticalCorridors',
      prediction: 'predictionCones',
      predictionCones: 'predictionCones',
      thought: 'thoughtLabel',
      thoughtHUD: 'thoughtLabel',
      gg: 'ggCircle',
      ggCircle: 'ggCircle'
    };
    const key = aliasMap[layerName] ?? layerName;
    return Boolean(this.layers[key]);
  }

  /**
   * Complete lifecycle cleanup of all WebGL geometries, materials, textures and DOM elements
   */
  dispose() {
    this.setVisible(false);

    if (this.candidateRenderer) {
      this.candidateRenderer.dispose();
      this.candidateRenderer = null;
    }

    if (this.tacticalRenderer) {
      this.tacticalRenderer.dispose();
      this.tacticalRenderer = null;
    }

    if (this.telemetryHUD) {
      this.telemetryHUD.dispose();
      this.telemetryHUD = null;
    }

    if (this.thought) {
      this.group.remove(this.thought.sprite);
      this.thought.sprite?.removeFromParent();
      this.thought.texture?.dispose();
      this.thought.material?.dispose();
      this.thought = null;
    }

    this.group.removeFromParent();
  }
}
