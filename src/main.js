import * as THREE from 'three';
import './style.css';
import { Circuit } from './simulation/Track.js';
import { Vehicle } from './simulation/Vehicle.js';
import { updateAerodynamicWakes, resolveVehicleCollisions } from './simulation/VehicleInteractions.js';
import { HARBOR_RING } from './scenarios/HarborRing.js';
import { ENDURANCE_PARK } from './scenarios/EndurancePark.js';
import { ScenarioEngine } from './scenarios/ScenarioEngine.js';
import { ResearchAIController } from './ai/ResearchAIController.js';
import { NextGenAIController } from './ai/v2/NextGenAIController.js';
import { CircuitEnvironment } from './render/Environment.js';
import { CarVisual } from './render/CarVisual.js';
import { AssetLibrary } from './render/AssetLibrary.js';
import { CameraRig } from './render/Cameras.js';
import { AIDebugSuiteRenderer } from './render/AIDebugSuiteRenderer.js';
import { InputManager } from './input.js';
import { SynthAudio } from './audio.js';
import { ScenarioDeck, SCENARIO_CATALOG } from './ui/ScenarioDeck.js';
import { ReferenceLapManager } from './simulation/ReferenceLap.js';

// Simulation Consts
const FIXED_TIMESTEP = 1 / 120;
const MAX_STEPS_PER_FRAME = 14;
const finite = (val, fallback = 0) => (Number.isFinite(val) ? val : fallback);

// App Containers
const app = document.querySelector('#app');
const loadingScreen = document.querySelector('#loading-screen');
const loadingStatus = document.querySelector('#loading-status');

// WebGL Renderer
const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.75));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.08;
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.domElement.tabIndex = 0;
renderer.domElement.setAttribute('aria-label', 'GEMINI GAUNTLET 3D Racing Canvas');
app?.append(renderer.domElement);

// Scene & Camera
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(58, window.innerWidth / window.innerHeight, 0.035, 1800);
const cameraRig = new CameraRig(camera);

// Track & Environment: Default to Harbor Ring
const ACTIVE_TRACK_DEF = HARBOR_RING;
const track = new Circuit(ACTIVE_TRACK_DEF);
const environment = new CircuitEnvironment(scene, track);

// Audio & Input
const audio = new SynthAudio();
const input = new InputManager(() => {
  audio.unlock().catch(() => {});
});

// ---------------------------------------------------------------------------
// 4-Car Race Grid Specification & Fleet Initialization
// ---------------------------------------------------------------------------
const GRID_SPECS = [
  { id: 'player', name: 'MARTIM (PLAYER)', spec: 'prototype', color: '#00f0ff', player: true, ai: 'v2', agg: 0.92, skill: 0.95 },
  { id: 'ai-1', name: 'GAUNTLET-AI', spec: 'prototype', color: '#ff9900', player: false, ai: 'v1', agg: 0.88, skill: 0.92 },
  { id: 'ai-2', name: 'Apex Prototype 03', spec: 'prototype', color: '#ff1744', player: false, ai: 'v2', agg: 0.90, skill: 0.90 },
  { id: 'ai-3', name: 'Titan Prototype 04', spec: 'prototype', color: '#76ff03', player: false, ai: 'v1', agg: 0.86, skill: 0.88 }
];

const allVehicles = [];
const allVisuals = [];
const allControllers = [];

for (let i = 0; i < GRID_SPECS.length; i += 1) {
  const spec = GRID_SPECS[i];
  const vehicle = new Vehicle({
    id: spec.id,
    name: spec.name,
    color: spec.color,
    player: spec.player,
    spec: spec.spec
  });
  allVehicles.push(vehicle);

  const visual = new CarVisual(vehicle, { variant: spec.spec === 'prototype' ? 'prototype' : 'gt' });
  scene.add(visual.group);
  allVisuals.push(visual);

  let controller;
  if (i === 0) {
    // Player Autonomous Pace AI
    controller = new ResearchAIController(2, {
      aggression: 0.65,
      diveMargin: 0.35,
      defenseReactivity: 0.7,
      kerbUsage: 0.8,
      lookahead: 24.0
    });
  } else if (i === 1) {
    // Primary GAUNTLET-AI
    controller = new ResearchAIController(1, {
      aggression: spec.agg,
      diveMargin: 0.45,
      defenseReactivity: 0.8,
      kerbUsage: 0.9,
      lookahead: 22.0
    });
  } else if (spec.ai === 'v2') {
    controller = new NextGenAIController(i + 1, {
      aggression: spec.agg,
      skill: spec.skill,
      track
    });
  } else {
    controller = new ResearchAIController(i + 1, {
      aggression: spec.agg,
      skill: spec.skill,
      diveMargin: 0.55,
      kerbUsage: 0.85
    });
  }
  controller.debugEnabled = (i === 1);
  allControllers.push(controller);
}

const player = allVehicles[0];
const aiVehicle = allVehicles[1];
const playerPaceAI = allControllers[0];
const aiController = allControllers[1];
const visuals = allVisuals;

// Active Fleet Subset (dynamically adjusted per scenario)
let activeVehicles = [player, aiVehicle];
let activeControllers = [playerPaceAI, aiController];
let activeGridCount = 2;

// User Reference Lap Recorder & Baseline Engine
const lapRecorder = new ReferenceLapManager(track);
aiController.setReferenceProfile(lapRecorder);
playerPaceAI.setReferenceProfile(lapRecorder);

const scenarioEngine = new ScenarioEngine(track, player, aiVehicle, aiController);

// AI 3D Debug Suite Renderer
const aiDebug = new AIDebugSuiteRenderer(scene, track);

// 3D Model Assets Loader
const assets = new AssetLibrary({
  onStatus: ({ state, key }) => {
    if (!loadingStatus) return;
    loadingStatus.textContent = state === 'ready' ? `LOADED ${key.toUpperCase()}` : `LOADING ${key.toUpperCase()}`;
  }
});

let assetsReady = false;
assets.preload().then((result) => {
  assetsReady = true;
  allVisuals.forEach((v) => v.attachAsset(assets));
  environment.installAssets(assets);
  if (loadingStatus) {
    loadingStatus.textContent = result.failed
      ? 'PROCEDURAL FALLBACK ACTIVE'
      : '3D BLENDER ASSETS ONLINE // 120HZ SIMULATION READY';
  }
});

// Simulation Clock & State
let simTimeScale = 1.0;
let isPaused = false;
let autopilotActive = true;
let sessionTimeS = 0;
const metrics = { fps: 60, physicsHz: 120 };
let accumulator = 0;
let previousTime = performance.now();
let frameCounter = 0;
let physicsCounter = 0;
let metricsAt = previousTime;

/**
 * Configures the active grid (2 cars for tactical duels, 4 for full races)
 */
function configureActiveGrid(scenarioId) {
  let count = 2;
  const upper = String(scenarioId || '').toUpperCase();
  if (upper.startsWith('RACE') || upper === 'FREE') {
    count = 4;
  }

  activeGridCount = count;
  activeVehicles = allVehicles.slice(0, count);
  activeControllers = allControllers.slice(0, count);

  // Position and display cars
  const gridStartDistance = 140.0;
  const gridBoxSpacing = 10.5;

  for (let i = 0; i < allVehicles.length; i += 1) {
    const v = allVehicles[i];
    const vis = allVisuals[i];
    if (i < count) {
      vis.group.visible = true;
      if (count > 2) {
        // 4-car staggered starting grid
        const lateral = (i % 2 === 0) ? 2.0 : -2.0;
        const dist = gridStartDistance - (i * gridBoxSpacing);
        v.resetTo(track, dist, lateral);
        v.speed = 0;
        v.velocity = { x: 0, y: 0, z: 0 };
        v.localVelocity = { x: 0, z: 0 };
      }
    } else {
      vis.group.visible = false;
      v.place(-9999, -9999, 0, -100);
      v.speed = 0;
    }
  }

  // Update Leaderboard visibility
  const leaderboardEl = document.querySelector('#race-leaderboard');
  if (leaderboardEl) {
    leaderboardEl.style.display = count > 2 ? 'block' : 'none';
    const infoEl = document.querySelector('#leaderboard-grid-info');
    if (infoEl) infoEl.textContent = `${count} CARS`;
  }
}

// UI Scenario Control Deck
const scenarioDeck = new ScenarioDeck({
  onScenarioSelect: (scenarioId) => {
    configureActiveGrid(scenarioId);
    if (activeGridCount <= 2) {
      scenarioEngine.loadScenario(scenarioId);
    }
    audio.unlock().catch(() => {});
  },
  onReset: () => {
    if (activeGridCount > 2) {
      configureActiveGrid(scenarioDeck.activeScenarioId);
    } else {
      scenarioEngine.resetScenario();
    }
    lapRecorder.reset();
    input.reset();
  },
  onSpeedChange: (speed) => {
    if (speed === 0) {
      isPaused = true;
    } else {
      isPaused = false;
      simTimeScale = speed;
    }
  },
  onAIHeuristicsChange: (heuristics) => {
    if (aiController.setHeuristicWeights) {
      aiController.setHeuristicWeights(heuristics);
    } else {
      if (heuristics.aggression != null) aiController._aggression = heuristics.aggression / 100;
      if (heuristics.diveMargin != null) aiController._diveMargin = heuristics.diveMargin / 100;
      if (heuristics.defenseReactivity != null) aiController._defenseReactivity = heuristics.defenseReactivity / 100;
      if (heuristics.kerbUsage != null) aiController._kerbUsage = heuristics.kerbUsage / 100;
      if (heuristics.lookaheadHorizon != null) aiController._lookahead = heuristics.lookaheadHorizon;
    }
  },
  onDebugToggle: (layer, enabled) => {
    if (aiDebug.setLayerVisible) {
      aiDebug.setLayerVisible(layer, enabled);
    }
  },
  onCameraChange: (camMode) => {
    cameraRig.setMode(camMode);
    visuals[0].setCockpitView(camMode === 'COCKPIT');
    if (camMode === 'FREE') {
      input.requestPointerLock(renderer.domElement);
    } else {
      input.exitPointerLock();
    }
  },
  onSetBaseline: () => {
    const res = lapRecorder.captureLiveBaseline();
    if (res.success) {
      aiController.setReferenceProfile(lapRecorder);
      playerPaceAI.setReferenceProfile(lapRecorder);
      const notice = document.querySelector('[data-hud="notice"]');
      if (notice) {
        notice.textContent = `USER BASELINE SET: ${ReferenceLapManager.formatTime(res.lapTime)} // AI PACE SYNCHRONIZED`;
        notice.style.display = 'block';
        setTimeout(() => { notice.style.display = 'none'; }, 4000);
      }
    }
  },
  onClearBaseline: () => {
    lapRecorder.clearBaseline();
    aiController.setReferenceProfile(null);
    playerPaceAI.setReferenceProfile(null);
    const notice = document.querySelector('[data-hud="notice"]');
    if (notice) {
      notice.textContent = 'USER BASELINE CLEARED // REVERTED TO GEOMETRIC PACE';
      notice.style.display = 'block';
      setTimeout(() => { notice.style.display = 'none'; }, 3000);
    }
  },
  onExportJSON: () => {
    const ok = lapRecorder.downloadTelemetryFile();
    const notice = document.querySelector('[data-hud="notice"]');
    if (notice) {
      notice.textContent = ok ? 'BUNDLED TELEMETRY JSON (PLAYER + AI) EXPORTED SUCCESSFULLY [DOWNLOAD STARTED]' : 'NO RECORDED LAP DATA TO EXPORT YET';
      notice.style.display = 'block';
      setTimeout(() => { notice.style.display = 'none'; }, 3500);
    }
  },
  onAutopilotToggle: (active) => {
    autopilotActive = active;
    const notice = document.querySelector('[data-hud="notice"]');
    if (notice) {
      notice.textContent = active ? 'AUTOPILOT / SPECTATE ENGAGED [P]' : 'MANUAL DRIVING ACTIVE // USE WASD / ARROWS [P]';
      notice.style.display = 'block';
      setTimeout(() => { notice.style.display = 'none'; }, 2500);
    }
  }
});

// Pointer Lock on canvas click for Noclip camera
renderer.domElement.addEventListener('click', () => {
  if (cameraRig.mode === 'FREE') {
    input.requestPointerLock(renderer.domElement);
  }
});

// Top HUD Autopilot badge click
document.querySelector('#hud-autopilot-badge')?.addEventListener('click', () => {
  scenarioDeck.toggleAutopilot();
});

// Load default initial scenario A1
scenarioEngine.loadScenario('A1_STRAIGHT_SLIPSTREAM');

// HUD DOM Elements Cache
const elSpeed = document.querySelector('[data-hud="speed"]');
const elGear = document.querySelector('[data-hud="gear"]');
const elRpmReadout = document.querySelector('[data-hud="rpm-readout"]');
const elBarRpm = document.querySelector('#bar-rpm');
const elBarThrottle = document.querySelector('#bar-throttle');
const elBarBrake = document.querySelector('#bar-brake');
const elBarSteer = document.querySelector('#bar-steer');
const elThrottleVal = document.querySelector('[data-hud="throttle-val"]');
const elBrakeVal = document.querySelector('[data-hud="brake-val"]');
const elSteerVal = document.querySelector('[data-hud="steer-val"]');
const elSessionTime = document.querySelector('[data-hud="session-time"]');
const elDistance = document.querySelector('[data-hud="distance"]');
const elSimRate = document.querySelector('[data-hud="sim-rate"]');
const elGapAI = document.querySelector('[data-hud="gap-ai"]');
const elLatG = document.querySelector('[data-hud="lat-g"]');
const elLongG = document.querySelector('[data-hud="long-g"]');
const elSlip = document.querySelector('[data-hud="slip-angle"]');
const elGGDot = document.querySelector('#gg-dot');
const elMotecWidthPct = document.querySelector('#motec-width-pct');
const elMotecWidthBar = document.querySelector('#motec-width-bar');
const elMotecLinePhase = document.querySelector('#motec-line-phase');
const elMotecCurbDist = document.querySelector('#motec-curb-dist');
const elBtnMute = document.querySelector('#btn-mute');
const elNoclipHud = document.querySelector('#noclip-hud');
const elLeaderboardRows = document.querySelector('#leaderboard-rows');

// Mute button click
elBtnMute?.addEventListener('click', () => {
  const muted = audio.toggleMute();
  if (elBtnMute) elBtnMute.textContent = muted ? 'AUDIO [MUTED]' : 'AUDIO [M]';
});

// Helper for MM:SS.mmm formatting
function formatTime(seconds) {
  const s = Math.max(0, seconds);
  const mins = Math.floor(s / 60);
  const secs = Math.floor(s % 60);
  const ms = Math.floor((s % 1) * 1000);
  return `${String(mins).padStart(2, '0')}:${String(secs).padStart(2, '0')}.${String(ms).padStart(3, '0')}`;
}

// 120Hz Fixed Physics Step
function fixedStep(dt) {
  if (isPaused) return;

  sessionTimeS += dt;

  const raceState = {
    phase: 'racing',
    raceTime: sessionTimeS,
    elapsed: sessionTimeS,
    statusFor: (v) => {
      const sorted = [...activeVehicles].sort((a, b) => b.distance - a.distance);
      return { position: sorted.indexOf(v) + 1 };
    }
  };

  // Player Controls: manual driving if keys pressed, otherwise autonomous pace along track
  const isInteracting = input.isInteracting();
  if (cameraRig.mode === 'FREE') {
    playerPaceAI.update(player, activeVehicles, track, raceState, dt);
  } else if (!autopilotActive) {
    Object.assign(player.controls, input.controls(player, dt));
  } else if (isInteracting) {
    Object.assign(player.controls, input.controls(player, dt));
  } else {
    playerPaceAI.update(player, activeVehicles, track, raceState, dt);
    const scenario = scenarioEngine.activeScenario;
    if (scenario?.playerConfig?.initialSpeedMps && activeGridCount <= 2) {
      const targetSpeed = scenario.playerConfig.initialSpeedMps;
      const speedError = targetSpeed - player.speed;
      if (speedError < -2) {
        player.controls.throttle = 0;
        player.controls.brake = Math.min(0.5, -speedError * 0.08);
      } else if (speedError > 0) {
        player.controls.throttle = Math.min(1.0, 0.4 + speedError * 0.08);
        player.controls.brake = 0;
      }
    }
  }

  // Update AI controllers for all active AI vehicles
  for (let i = 1; i < activeVehicles.length; i += 1) {
    activeControllers[i].update(activeVehicles[i], activeVehicles, track, raceState, dt);
  }

  // Aerodynamic wake & dirty air matrices across active fleet
  updateAerodynamicWakes(activeVehicles);

  // Physics stepping for all active cars
  for (let i = 0; i < activeVehicles.length; i += 1) {
    activeVehicles[i].step(dt, track, true);
  }

  // Vehicle-vehicle collision resolution
  const collisionStats = resolveVehicleCollisions(activeVehicles, 3);

  // Scenario engine evaluation (for 2-car scenarios)
  if (activeGridCount <= 2) {
    scenarioEngine.update(dt, collisionStats);
  }

  // Reference Lap recording & multi-vehicle telemetry
  lapRecorder.update(activeVehicles, dt);

  physicsCounter += 1;
}

// Keyboard Actions / Shortcuts
function processActions() {
  if (input.consume('KeyR')) {
    if (activeGridCount > 2) {
      configureActiveGrid(scenarioDeck.activeScenarioId);
    } else {
      scenarioEngine.resetScenario();
    }
    input.reset();
  }
  if (input.consume('Space')) {
    scenarioDeck.togglePause();
  }
  if (input.consume('KeyP')) {
    scenarioDeck.toggleAutopilot();
  }
  if (input.consume('KeyC')) {
    const nextMode = cameraRig.cycleMode();
    scenarioDeck.setCameraMode(nextMode);
    visuals[0].setCockpitView(nextMode === 'COCKPIT');
    input.exitPointerLock();
  }
  if (input.consume('KeyV') || input.consume('KeyF')) {
    const isFree = cameraRig.mode === 'FREE';
    const nextMode = isFree ? 'CHASE' : 'FREE';
    cameraRig.setMode(nextMode);
    scenarioDeck.setCameraMode(nextMode);
    visuals[0].setCockpitView(false);
    if (!isFree) {
      input.requestPointerLock(renderer.domElement);
    } else {
      input.exitPointerLock();
    }
  }
  if (input.consume('Tab')) {
    aiDebug.visible = !aiDebug.visible;
    aiDebug.group.visible = aiDebug.visible;
  }
  if (input.consume('KeyM')) {
    if (aiDebug.markIncident) {
      const inc = aiDebug.markIncident(aiVehicle, aiController, track, 'MANUAL_FLAG', 'Flagged via [M] hotkey');
      if (inc) {
        scenarioDeck?.showNotification?.(`🚩 Incident Marked @ ${inc.distanceM}m (Lap ${inc.lap})`);
      }
    }
  }
  if (input.consume('KeyU')) {
    const muted = audio.toggleMute();
    if (elBtnMute) elBtnMute.textContent = muted ? 'AUDIO [MUTED]' : 'AUDIO [U]';
  }
  if (input.consume('KeyJ')) {
    scenarioDeck.onExportJSON();
  }
  if (input.consume('Digit1')) scenarioDeck.selectScenario('A1');
  if (input.consume('Digit2')) scenarioDeck.selectScenario('A2');
  if (input.consume('Digit3')) scenarioDeck.selectScenario('A3');
  if (input.consume('Digit4')) scenarioDeck.selectScenario('A4');
  if (input.consume('Digit5')) scenarioDeck.selectScenario('A5');
}

// Render & Animation Frame
function frame(now) {
  const rawDelta = Math.min(0.1, (now - previousTime) / 1000);
  previousTime = now;

  processActions();

  // Accumulate simulation time with speed scaling
  if (!isPaused) {
    accumulator += rawDelta * simTimeScale;
    let steps = 0;
    while (accumulator >= FIXED_TIMESTEP && steps < MAX_STEPS_PER_FRAME) {
      fixedStep(FIXED_TIMESTEP);
      accumulator -= FIXED_TIMESTEP;
      steps += 1;
    }
    if (steps === MAX_STEPS_PER_FRAME) accumulator = 0;
  }

  // Update visuals for active fleet
  for (let i = 0; i < activeVehicles.length; i += 1) {
    allVisuals[i].update(rawDelta);
  }
  environment.update(rawDelta);

  // Update AI Debug Suite
  if (aiDebug.update) {
    aiDebug.update(aiController, aiVehicle, player, track, now);
  }

  // Update Camera Rig
  if (cameraRig.mode === 'FREE') {
    cameraRig.updateFree(input.freeCameraRaw(), rawDelta);
    if (elNoclipHud) {
      elNoclipHud.style.display = 'block';
      const isLocked = input.isPointerLocked();
      elNoclipHud.textContent = isLocked
        ? `🎥 NOCLIP SPECTATOR LOCKED · WASD+QE FLY · SHIFT BOOST · CTRL SLOW · WHEEL SPD (${input.flightSpeed}m/s) · ESC UNLOCK`
        : `🎥 NOCLIP SPECTATOR ACTIVE · CLICK VIEWPORT TO LOCK MOUSE · [V] TOGGLE CHASE`;
    }
  } else {
    if (elNoclipHud) elNoclipHud.style.display = 'none';
    const cockpitPose = cameraRig.mode === 'COCKPIT' ? visuals[0].getCockpitPose() : null;
    cameraRig.update(player, rawDelta, cockpitPose, aiVehicle);
  }

  // Update Audio
  audio.update(player, rawDelta);

  // Performance Metrics
  frameCounter += 1;
  if (now - metricsAt > 500) {
    const span = (now - metricsAt) / 1000;
    metrics.fps = Math.round(frameCounter / span);
    metrics.physicsHz = Math.round(physicsCounter / span);
    frameCounter = 0;
    physicsCounter = 0;
    metricsAt = now;
  }

  // Update Live HUD Readouts & Multi-Car Leaderboard
  updateHUDReadouts();

  // Render WebGL Scene
  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}

// Update HUD Elements & Race Leaderboard
function updateHUDReadouts() {
  const speedKph = Math.round((player.speed || 0) * 3.6);
  if (elSpeed) elSpeed.textContent = String(speedKph);

  const gearText = (player.gear || 0) === 0 ? 'N' : (player.gear || 0) < 0 ? 'R' : String(player.gear || 1);
  if (elGear) elGear.textContent = gearText;

  const rpm = Math.round(player.rpm || 0);
  const torque = Math.round(player.engineTorque || 0);
  if (elRpmReadout) elRpmReadout.textContent = `${rpm} RPM · ${torque} NM`;

  const rpmFactor = Math.min(1, Math.max(0, (rpm - 1000) / 7500));
  if (elBarRpm) elBarRpm.style.transform = `scaleX(${Math.max(0.04, rpmFactor)})`;

  const thr = player.controls?.throttle || 0;
  const brk = player.controls?.brake || 0;
  const str = Math.abs(player.controls?.steer || 0);

  if (elBarThrottle) elBarThrottle.style.transform = `scaleX(${thr})`;
  if (elBarBrake) elBarBrake.style.transform = `scaleX(${brk})`;
  if (elBarSteer) elBarSteer.style.transform = `scaleX(${str})`;

  if (elThrottleVal) elThrottleVal.textContent = `${Math.round(thr * 100)}%`;
  if (elBrakeVal) elBrakeVal.textContent = `${Math.round(brk * 100)}%`;
  if (elSteerVal) elSteerVal.textContent = `${Math.round(str * 100)}%`;

  if (elSessionTime) elSessionTime.textContent = formatTime(sessionTimeS);
  if (elDistance) elDistance.textContent = `${(player.distance || 0).toFixed(1)} M`;
  if (elSimRate) elSimRate.textContent = `${isPaused ? 'PAUSED' : simTimeScale.toFixed(1) + 'x'} // ${metrics.physicsHz}HZ`;

  // Gap to AI / Leader
  const gapDist = (aiVehicle.distance || 0) - (player.distance || 0);
  const gapSeconds = (gapDist / Math.max(10, player.speed || 20)).toFixed(2);
  if (elGapAI) {
    elGapAI.textContent = `${gapSeconds >= 0 ? '+' : ''}${gapSeconds}s`;
    elGapAI.style.color = gapSeconds >= 0 ? 'var(--lime)' : 'var(--orange)';
  }

  // Telemetry G-forces
  const localAcc = player.localAcceleration || { x: 0, z: 0 };
  const latG = (localAcc.x / 9.81).toFixed(2);
  const longG = (localAcc.z / 9.81).toFixed(2);
  if (elLatG) elLatG.textContent = `${latG} G`;
  if (elLongG) elLongG.textContent = `${longG} G`;

  const localVel = player.localVelocity || { x: 0, z: 1 };
  const slipAngle = ((Math.atan2(localVel.x, Math.max(1, localVel.z)) * 180) / Math.PI).toFixed(1);
  if (elSlip) elSlip.textContent = `${slipAngle}°`;

  // MoTeC G-G circle dot position
  if (elGGDot) {
    const dotX = Math.max(-36, Math.min(36, (-localAcc.x / 9.81 / 2.5) * 36));
    const dotY = Math.max(-36, Math.min(36, (-localAcc.z / 9.81 / 2.5) * 36));
    elGGDot.style.transform = `translate(${dotX}px, ${dotY}px)`;
  }

  // Live Track Width Utilization & Racing Line Telemetry
  const pLateral = player.surface?.lateral || 0;
  const rHalf = track.roadHalfWidth || 7.6;
  const cWidth = track.curbWidth || 1.35;
  const totalMargin = rHalf + cWidth;
  const carHalf = (player.trackWidth || 1.8) * 0.5;
  const widthPct = Math.min(100, Math.max(0, ((Math.abs(pLateral) + carHalf) / totalMargin) * 100));
  const distL = (totalMargin + pLateral).toFixed(1);
  const distR = (totalMargin - pLateral).toFixed(1);

  const curTrackPt = track.atDistance ? track.atDistance(player.distance || 0) : { curvature: 0, turnSign: 0 };
  const linePhase = lapRecorder._classifyRacingLinePhase(
    player.distance || 0,
    pLateral,
    curTrackPt.curvature || 0,
    curTrackPt.turnSign || 0,
    player.controls?.throttle || 0,
    player.controls?.brake || 0
  );

  if (elMotecWidthPct) elMotecWidthPct.textContent = `${widthPct.toFixed(1)}%`;
  if (elMotecWidthBar) elMotecWidthBar.style.width = `${widthPct.toFixed(1)}%`;
  if (elMotecLinePhase) elMotecLinePhase.textContent = linePhase;
  if (elMotecCurbDist) elMotecCurbDist.textContent = `L: ${distL}m · R: ${distR}m`;

  // Update Multi-Car Race Leaderboard
  if (activeGridCount > 2 && elLeaderboardRows) {
    const sorted = [...activeVehicles].sort((a, b) => b.distance - a.distance);
    const leaderDist = sorted[0]?.distance || 1;

    let html = '';
    for (let pos = 0; pos < sorted.length; pos += 1) {
      const v = sorted[pos];
      const isUser = (v === player);
      const gapM = leaderDist - v.distance;
      const gapStr = pos === 0 ? 'LEADER' : `+${(gapM / Math.max(8, v.speed || 20)).toFixed(1)}s`;
      const spdKmh = Math.round(v.speed * 3.6);
      const spec = v.spec || 'gt';
      const badgeClass = spec === 'prototype' ? 'badge-proto' : (spec === 'touring' ? 'badge-tour' : 'badge-gt');
      const badgeText = spec === 'prototype' ? 'LMP' : (spec === 'touring' ? 'TCR' : 'GT');

      html += `
        <div class="leaderboard-row ${isUser ? 'player' : ''}">
          <span class="leaderboard-pos">P${pos + 1}</span>
          <span class="leaderboard-badge ${badgeClass}">${badgeText}</span>
          <span class="leaderboard-name">${v.name}</span>
          <span class="leaderboard-gap">${gapStr}</span>
          <span class="leaderboard-speed">${spdKmh}k</span>
        </div>
      `;
    }
    elLeaderboardRows.innerHTML = html;
  }

  // Update AI Thought HUD from AI Controller telemetry
  const aiTelemetry = aiController.telemetry || aiController.debugState?.telemetry || {};
  const aiDebugState = aiController.debugState || {};
  scenarioDeck.updateAIThoughtHUD({
    state: aiTelemetry.state || aiDebugState.mode || aiTelemetry.maneuver || 'TRAJECTORY_FOLLOW',
    action: aiTelemetry.action || (aiVehicle.speed > 50 ? 'ATTACK DRAFT' : 'CRUISE'),
    reason: aiTelemetry.reason || aiDebugState.reason || 'Optimal corridor tracking at 120Hz',
    threat: aiTelemetry.threatLevel || aiDebugState.threat?.threatLevel || 'LOW',
    aggression: Math.round((aiController._aggression || 0.75) * 100),
    diveMargin: (aiController._diveMargin || 0.45),
    defense: Math.round((aiController._defenseReactivity || 0.8) * 100),
    kerb: Math.round((aiController._kerbUsage || 0.9) * 100),
    lookahead: aiController._lookahead || 22.0,
    throttle: aiVehicle.controls?.throttle || 0,
    brake: aiVehicle.controls?.brake || 0,
    steer: aiVehicle.controls?.steer || 0,
    tactic: aiTelemetry.tactic || 'Dynamic Frenet Lattice Evaluation',
    prediction: aiTelemetry.prediction || `Clearance: ${finite(aiDebugState.trajectoryMinimumClearanceM, 99).toFixed(1)}m`,
    decision: aiTelemetry.decision || aiDebugState.decisionReason || 'Hold tactical spacing'
  });

  // Update Lap Telemetry & User Baseline HUD
  const deltaS = lapRecorder.calculateDelta(player.distance || 0, lapRecorder.currentLapTime);
  scenarioDeck.updateLapTelemetry({
    currentLapTime: lapRecorder.currentLapTime,
    lastLapTime: lapRecorder.lastLapTime,
    bestLapTime: lapRecorder.bestLapTime,
    deltaS,
    baselineLapTime: lapRecorder.userBaseline?.lapTime ?? null,
    hasBaseline: Boolean(lapRecorder.userBaseline),
    formatTime
  });
}

// Window Resizing
window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

// Start Animation Loop
requestAnimationFrame(frame);

// Global Sandbox Export
window.__GEMINI_GAUNTLET__ = {
  renderer,
  scene,
  camera,
  cameraRig,
  track,
  environment,
  allVehicles,
  get activeVehicles() { return activeVehicles; },
  player,
  aiVehicle,
  aiController,
  playerPaceAI,
  ResearchAIController,
  NextGenAIController,
  scenarioEngine,
  aiDebug,
  scenarioDeck,
  assets,
  audio,
  input,
  metrics,
  SCENARIO_CATALOG,
  configureActiveGrid,
  get isPaused() { return isPaused; },
  get simTimeScale() { return simTimeScale; },
  resetScenario: () => {
    if (activeGridCount > 2) configureActiveGrid(scenarioDeck.activeScenarioId);
    else scenarioEngine.resetScenario();
  },
  loadScenario: (id) => {
    configureActiveGrid(id);
    if (activeGridCount <= 2) scenarioEngine.loadScenario(id);
  },
  setSimSpeed: (speed) => scenarioDeck.onSpeedChange(speed)
};

document.documentElement.dataset.ready = 'true';
setTimeout(() => loadingScreen?.classList.add('dismissed'), 500);
