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
import { CircuitScenarioVisuals } from './render/CircuitScenarioVisuals.js';
import { RubberSurfaceRenderer } from './render/RubberSurfaceRenderer.js';
import { InputManager } from './input.js';
import { SynthAudio } from './audio.js';
import { ScenarioDeck, SCENARIO_CATALOG } from './ui/ScenarioDeck.js';
import { HUD } from './ui/HUD.js';
import { ReferenceLapManager } from './simulation/ReferenceLap.js';

// Simulation Constants
const FIXED_TIMESTEP = 1 / 120;
const MAX_STEPS_PER_FRAME = 14;
const TOTAL_RACE_LAPS = 12;
const PIT_WINDOW_OPEN_LAP = 4;
const PIT_WINDOW_CLOSE_LAP = 10;
const PIT_SPEED_LIMIT_MPS = 16.67; // 60 km/h
const PIT_SERVICE_DURATION_S = 3.5;

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
let currentTrackDef = HARBOR_RING;
const track = new Circuit(currentTrackDef);
const environment = new CircuitEnvironment(scene, track);
let scenarioVisuals = new CircuitScenarioVisuals(scene, track, currentTrackDef);
let rubberRenderer = new RubberSurfaceRenderer(scene, track);

// Audio & Input
const audio = new SynthAudio();
const input = new InputManager(() => {
  audio.unlock().catch(() => {});
});

// Pro Motorsport Cockpit HUD
const hud = new HUD(() => audio.toggleMute());

// ---------------------------------------------------------------------------
// 12-Car Full Multi-Class Grid Specification
// ---------------------------------------------------------------------------
const FULL_GRID_SPECS = [
  // LMP2 / Prototype Class (P1 - P4)
  { id: 'player', name: 'MARTIM (PLAYER)', spec: 'prototype', color: '#00f0ff', player: true, ai: 'v2', agg: 0.96, skill: 1.0 },
  { id: 'ai-proto-1', name: 'GAUNTLET-AI', spec: 'prototype', color: '#ff9900', player: false, ai: 'v2', agg: 0.95, skill: 0.98 },
  { id: 'ai-proto-2', name: 'Apex Prototype 03', spec: 'prototype', color: '#ff1744', player: false, ai: 'v2', agg: 0.92, skill: 0.94 },
  { id: 'ai-proto-3', name: 'Titan Prototype 04', spec: 'prototype', color: '#76ff03', player: false, ai: 'v2', agg: 0.90, skill: 0.92 },

  // GT Class (P5 - P8)
  { id: 'ai-gt-1', name: 'GranTurismo 05', spec: 'gt', color: '#00e5ff', player: false, ai: 'v2', agg: 0.88, skill: 0.90 },
  { id: 'ai-gt-2', name: 'GranTurismo 06', spec: 'gt', color: '#d500f9', player: false, ai: 'v2', agg: 0.86, skill: 0.88 },
  { id: 'ai-gt-3', name: 'GranTurismo 07', spec: 'gt', color: '#ff6d00', player: false, ai: 'v2', agg: 0.85, skill: 0.87 },
  { id: 'ai-gt-4', name: 'GranTurismo 08', spec: 'gt', color: '#64dd17', player: false, ai: 'v2', agg: 0.84, skill: 0.86 },

  // Touring Class (P9 - P12)
  { id: 'ai-tour-1', name: 'Touring Racer 09', spec: 'touring', color: '#ffd600', player: false, ai: 'v2', agg: 0.82, skill: 0.85 },
  { id: 'ai-tour-2', name: 'Touring Racer 10', spec: 'touring', color: '#00b0ff', player: false, ai: 'v2', agg: 0.80, skill: 0.84 },
  { id: 'ai-tour-3', name: 'Touring Racer 11', spec: 'touring', color: '#ff4081', player: false, ai: 'v2', agg: 0.79, skill: 0.83 },
  { id: 'ai-tour-4', name: 'Touring Racer 12', spec: 'touring', color: '#aeea00', player: false, ai: 'v2', agg: 0.78, skill: 0.82 }
];

const allVehicles = [];
const allVisuals = [];
const allControllers = [];

for (let i = 0; i < FULL_GRID_SPECS.length; i += 1) {
  const spec = FULL_GRID_SPECS[i];
  const vehicle = new Vehicle({
    id: spec.id,
    name: spec.name,
    color: spec.color,
    player: spec.player,
    spec: spec.spec
  });
  vehicle.classKey = spec.spec;
  vehicle.completedLaps = 0;
  vehicle.pitStopDone = false;
  vehicle.pitRequested = false;
  vehicle.inPitLane = false;
  vehicle.inPitBox = false;
  vehicle.pitServiceRemaining = 0;
  vehicle.pitServiceTotal = PIT_SERVICE_DURATION_S;
  vehicle.lapStartTime = 0;
  vehicle.lastDistance = 0;
  vehicle.bestLapTime = null;
  allVehicles.push(vehicle);

  const visual = new CarVisual(vehicle, { variant: spec.spec });
  scene.add(visual.group);
  allVisuals.push(visual);

  let controller;
  if (spec.ai === 'v2') {
    controller = new NextGenAIController(i + 1, {
      aggression: spec.agg,
      skill: spec.skill,
      track
    });
  } else {
    controller = new ResearchAIController(i + 1, {
      aggression: spec.agg,
      skill: spec.skill,
      track
    });
  }
  controller.debugEnabled = (i === 1);
  allControllers.push(controller);
}

const player = allVehicles[0];
const playerPaceAI = allControllers[0];
const visuals = allVisuals;

// Active Fleet Configuration (defaults to full 12-car grid for Grand Prix)
let activeVehicles = [...allVehicles];
let activeControllers = [...allControllers];
let activeGridCount = 12;

// User Reference Lap Recorder
const lapRecorder = new ReferenceLapManager(track);
allControllers.forEach((c) => c.setReferenceProfile?.(lapRecorder));

const scenarioEngine = new ScenarioEngine(track, player, allVehicles[1], allControllers[1]);

// AI 3D Debug Suite Renderer
const aiDebug = new AIDebugSuiteRenderer(scene, track);
let aiInspectIndex = 1; // Inspect GAUNTLET-AI by default
let aiFieldView = false;
let aiDebugEnabled = false;

// 3D Model Assets Loader
const assets = new AssetLibrary({
  onStatus: ({ state, key }) => {
    if (!loadingStatus) return;
    loadingStatus.textContent = state === 'ready' ? `LOADED ${key.toUpperCase()}` : `LOADING ${key.toUpperCase()}`;
  }
});

const dismissLoadingScreen = () => {
  if (loadingScreen && !loadingScreen.classList.contains('dismissed')) {
    loadingScreen.classList.add('dismissed');
  }
};

let assetsReady = false;
assets.preload().then((result) => {
  assetsReady = true;
  allVisuals.forEach((v) => v.attachAsset(assets));
  environment.installAssets(assets);
  if (loadingStatus) {
    loadingStatus.textContent = result.failed
      ? 'PROCEDURAL FALLBACK ACTIVE'
      : '3D ASSETS ONLINE // 12-LAP GRAND PRIX SIMULATION READY';
  }
  setTimeout(dismissLoadingScreen, 350);
}).catch(() => {
  dismissLoadingScreen();
});

// Safety fallback to ensure loading screen is always dismissed
setTimeout(dismissLoadingScreen, 1200);

// Simulation Clock & Race State
let simTimeScale = 1.0;
let isPaused = false;
let autopilotActive = true;
let sessionTimeS = 0;
let racePhase = 'racing'; // 'grid', 'racing', 'finished'
const metrics = { fps: 60, physicsHz: 120 };
let accumulator = 0;
let previousTime = performance.now();
let frameCounter = 0;
let physicsCounter = 0;
let metricsAt = previousTime;

/**
 * Configure Grand Prix / Tactical Starting Grid
 */
function configureActiveGrid(scenarioId = 'GRAND_PRIX_12_LAPS') {
  const upper = String(scenarioId || '').toUpperCase();
  let count = 12;
  if (upper.startsWith('A') || upper.startsWith('D')) {
    count = 2; // Tactical duel scenarios
  } else if (upper.startsWith('C') || upper === 'FIELD_4') {
    count = 4;
  }

  activeGridCount = count;
  activeVehicles = allVehicles.slice(0, count);
  activeControllers = allControllers.slice(0, count);

  const gridStartDistance = 140.0;
  const gridBoxSpacing = 9.5;

  for (let i = 0; i < allVehicles.length; i += 1) {
    const v = allVehicles[i];
    const vis = allVisuals[i];
    if (i < count) {
      vis.group.visible = true;
      const lateral = (i % 2 === 0) ? 2.2 : -2.2;
      const dist = gridStartDistance - (i * gridBoxSpacing);
      v.resetTo(track, dist, lateral);
      v.speed = 0;
      v.velocity = { x: 0, y: 0, z: 0 };
      v.localVelocity = { x: 0, z: 0 };
      v.completedLaps = 0;
      v.pitStopDone = false;
      v.pitRequested = false;
      v.inPitLane = false;
      v.inPitBox = false;
      v.pitServiceRemaining = 0;
      v.finished = false;
      v.lastDistance = dist;
    } else {
      vis.group.visible = false;
      v.place(-9999, -9999, 0, -100);
      v.speed = 0;
    }
  }

  sessionTimeS = 0;
  racePhase = 'racing';
  hud.setNotice(count === 12 ? 'GRAND PRIX: 12 LAPS · MANDATORY PIT STOP REQUIRED (LAPS 4–10)' : `SCENARIO: ${scenarioId}`, 4.0);
}

// Initialize default 12-Car Grand Prix
configureActiveGrid('GRAND_PRIX_12_LAPS');

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
    configureActiveGrid(scenarioDeck.activeScenarioId);
    lapRecorder.reset();
    input.reset();
  },
  onSpeedChange: (speed) => {
    isPaused = (speed === 0);
    simTimeScale = Math.max(0.1, speed);
  },
  onDebugToggle: (layer, enabled) => {
    aiDebug.setLayerVisible?.(layer, enabled);
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
  onAutopilotToggle: (active) => {
    autopilotActive = active;
    hud.setNotice(active ? 'AUTOPILOT / SPECTATOR ENGAGED [P]' : 'MANUAL DRIVING ENGAGED // WASD / ARROWS [P]', 2.5);
  }
});

// Pointer Lock on canvas click for Noclip camera
renderer.domElement.addEventListener('click', () => {
  if (cameraRig.mode === 'FREE') {
    input.requestPointerLock(renderer.domElement);
  }
});

// ---------------------------------------------------------------------------
// 12-Lap Grand Prix Simulation & Mandatory Pit Stop State Machine
// ---------------------------------------------------------------------------
function updateRaceAndPitLogic(vehicle, controller, dt) {
  const trackLen = track.length;
  const currentDist = vehicle.distance;
  const lastDist = vehicle.lastDistance ?? currentDist;

  // Lap Completion Detection
  if (lastDist > trackLen * 0.80 && currentDist < trackLen * 0.20) {
    const currentLapTime = sessionTimeS - (vehicle.lapStartTime || 0);
    vehicle.completedLaps = (vehicle.completedLaps || 0) + 1;
    vehicle.lapStartTime = sessionTimeS;

    if (!vehicle.bestLapTime || (currentLapTime > 30.0 && currentLapTime < vehicle.bestLapTime)) {
      vehicle.bestLapTime = currentLapTime;
    }

    if (vehicle.completedLaps >= TOTAL_RACE_LAPS) {
      vehicle.finished = true;
      vehicle.totalRaceTime = sessionTimeS;
      if (vehicle === player) racePhase = 'finished';
    }
  }
  vehicle.lastDistance = currentDist;

  // Strategic AI Pit Stop Decision Logic (Laps 4 to 10)
  const currentLap = (vehicle.completedLaps || 0) + 1;
  const isPitWindow = currentLap >= PIT_WINDOW_OPEN_LAP && currentLap <= PIT_WINDOW_CLOSE_LAP;

  if (!vehicle.player && isPitWindow && !vehicle.pitStopDone && !vehicle.pitRequested) {
    const avgTireWear = (vehicle.wheels ?? []).reduce((acc, w) => acc + (w.wear || 0), 0) / 4;
    // Pit if tires worn or strategic lap reached
    const strategicLap = PIT_WINDOW_OPEN_LAP + (vehicle.id.charCodeAt(vehicle.id.length - 1) % 4);
    if (avgTireWear > 0.22 || currentLap >= strategicLap) {
      vehicle.pitRequested = true;
      vehicle.pitIntent = { active: true, targetLateralM: 6.8 };
    }
  }

  // Pit Lane Entry Zone (Near Start/Finish straight on right flank: s > 2500m or s < 180m)
  const isNearPitEntry = currentDist > trackLen - 220 || currentDist < 160;
  const isNearPitLane = Math.abs(vehicle.surface?.lateral || 0) > (track.roadHalfWidth - 1.5) && isNearPitEntry;

  if ((vehicle.pitRequested || vehicle.inPitLane) && isNearPitLane) {
    vehicle.inPitLane = true;
    vehicle.pitLimiterActive = true;

    // Pit Speed Limiter (60 km/h)
    if (vehicle.speed > PIT_SPEED_LIMIT_MPS) {
      vehicle.controls.throttle = 0;
      vehicle.controls.brake = Math.min(0.6, (vehicle.speed - PIT_SPEED_LIMIT_MPS) * 0.15);
    }

    // Pit Box Location (s between 30m and 70m)
    const inBoxZone = currentDist >= 25 && currentDist <= 65;
    if (inBoxZone && !vehicle.pitStopDone) {
      vehicle.inPitBox = true;
      vehicle.controls.throttle = 0;
      vehicle.controls.brake = 1.0;
      vehicle.speed = Math.max(0, vehicle.speed - dt * 12.0);

      if (vehicle.pitServiceRemaining <= 0) {
        vehicle.pitServiceRemaining = PIT_SERVICE_DURATION_S;
      } else {
        vehicle.pitServiceRemaining -= dt;
        if (vehicle.pitServiceRemaining <= 0) {
          // Fresh Tires Fitted & Service Complete
          (vehicle.wheels ?? []).forEach((w) => {
            w.wear = 0;
            w.temperatureInnerC = 85;
            w.temperatureMiddleC = 88;
            w.temperatureOuterC = 85;
          });
          vehicle.pitStopDone = true;
          vehicle.inPitBox = false;
          vehicle.pitRequested = false;
          vehicle.pitIntent = null;
          if (vehicle === player) {
            hud.setNotice('PIT SERVICE COMPLETE: FRESH TIRES FITTED // GO GO GO!', 3.0);
          }
        }
      }
    } else if (currentDist > 160 && currentDist < trackLen * 0.5) {
      // Exit pit lane back to racing line
      vehicle.inPitLane = false;
      vehicle.pitLimiterActive = false;
    }
  }
}

// 120Hz Fixed Physics Step
function fixedStep(dt) {
  if (isPaused) return;

  sessionTimeS += dt;

  const raceState = {
    phase: racePhase,
    raceTime: sessionTimeS,
    elapsed: sessionTimeS,
    totalLaps: TOTAL_RACE_LAPS,
    currentLap: Math.max(1, (player.completedLaps || 0) + 1)
  };

  // 1. Process Controls
  const isInteracting = input.isInteracting();
  if (cameraRig.mode === 'FREE' || autopilotActive) {
    playerPaceAI.update(player, activeVehicles, track, raceState, dt);
  } else {
    Object.assign(player.controls, input.controls(player, dt));
  }

  // 2. Update AI Controllers
  for (let i = 1; i < activeVehicles.length; i += 1) {
    activeControllers[i].update(activeVehicles[i], activeVehicles, track, raceState, dt);
  }

  // 3. Update Race & Pit Logic for All Cars
  for (let i = 0; i < activeVehicles.length; i += 1) {
    updateRaceAndPitLogic(activeVehicles[i], activeControllers[i], dt);
  }

  // 4. Aerodynamic Wakes & Dirty Air
  updateAerodynamicWakes(activeVehicles);

  // 5. Physics Step for Active Cars
  for (let i = 0; i < activeVehicles.length; i += 1) {
    activeVehicles[i].step(dt, track, true);
  }

  // 6. Collision Resolution
  const collisionStats = resolveVehicleCollisions(activeVehicles, 3);

  // 7. Tactical Scenarios (if in 2-car scenario mode)
  if (activeGridCount <= 2) {
    scenarioEngine.update(dt, collisionStats);
  }

  // 8. Reference Lap Recording
  lapRecorder.update(activeVehicles, dt);

  physicsCounter += 1;
}

// Keyboard Actions / Shortcuts
function processActions() {
  // Pit Request [P]
  if (input.consume('KeyP')) {
    player.pitRequested = !player.pitRequested;
    player.pitIntent = player.pitRequested ? { active: true, targetLateralM: 6.8 } : null;
    hud.setNotice(player.pitRequested ? 'PIT REQUESTED: ENTER PIT LANE ON RIGHT FLANK' : 'PIT REQUEST CANCELLED', 3.0);
  }

  // AI Inspect Toggle [F3]
  if (input.consume('F3')) {
    aiDebugEnabled = !aiDebugEnabled;
    aiDebug.setVisible?.(aiDebugEnabled);
    hud.setNotice(aiDebugEnabled ? 'AI DEBUGGER: ACTIVE [F3]' : 'AI DEBUGGER: OFF [F3]', 2.0);
  }

  // AI Field View Toggle [F4]
  if (input.consume('F4')) {
    aiFieldView = !aiFieldView;
    hud.setNotice(aiFieldView ? 'AI FIELD RADAR: ACTIVE [F4]' : 'AI FIELD RADAR: SINGLE CAR [F4]', 2.0);
  }

  // Next AI Car Cycle [N]
  if (input.consume('KeyN')) {
    aiInspectIndex = (aiInspectIndex % (activeVehicles.length - 1)) + 1;
    const inspectCar = activeVehicles[aiInspectIndex];
    hud.setNotice(`AI INSPECT: ${inspectCar.name} (${inspectCar.classKey.toUpperCase()}) [N]`, 2.5);
  }

  // MoTeC Telemetry Toggle [T]
  if (input.consume('KeyT')) {
    hud.toggleTelemetry();
  }

  // Camera Mode Cycle [C]
  if (input.consume('KeyC')) {
    scenarioDeck.cycleCamera();
  }

  // Restart / Reset [R]
  if (input.consume('KeyR')) {
    configureActiveGrid(scenarioDeck.activeScenarioId);
  }
}

// ---------------------------------------------------------------------------
// Main 60FPS / 120Hz Animation Frame Loop
// ---------------------------------------------------------------------------
function animate(time) {
  requestAnimationFrame(animate);

  const rawDelta = Math.min((time - previousTime) / 1000, 0.1);
  previousTime = time;

  processActions();

  // Fixed Physics Sub-stepping
  accumulator += rawDelta * simTimeScale;
  let steps = 0;
  while (accumulator >= FIXED_TIMESTEP && steps < MAX_STEPS_PER_FRAME) {
    fixedStep(FIXED_TIMESTEP);
    accumulator -= FIXED_TIMESTEP;
    steps += 1;
  }

  // Interpolation Alpha
  const alpha = Math.min(1, Math.max(0, accumulator / FIXED_TIMESTEP));

  // Update Visuals
  for (let i = 0; i < activeVehicles.length; i += 1) {
    allVisuals[i].update(alpha, activeVehicles[i].controls, rawDelta);
  }

  // Update Scenery Props, Lighting & Dynamic Rubber
  scenarioVisuals.update(rawDelta, activeVehicles);
  rubberRenderer.update(rawDelta);
  environment.update(camera);

  // Update Camera Rig
  const trackedTarget = cameraRig.mode === 'AI_INSPECT' ? activeVehicles[aiInspectIndex] : player;
  cameraRig.update(trackedTarget, rawDelta, input);

  // Update AI 3D Debug Suite (Lattice ribbons & beacons)
  const inspectedAI = activeVehicles[aiInspectIndex] || activeVehicles[1];
  const inspectedCtrl = activeControllers[aiInspectIndex] || activeControllers[1];
  if (aiDebugEnabled) {
    aiDebug.update(inspectedCtrl, inspectedAI, activeVehicles, track, time);
  }

  // Render 3D Scene
  renderer.render(scene, camera);

  // Build AI Debug Snapshot for HUD
  const aiDebugSnapshot = {
    enabled: aiDebugEnabled || aiFieldView,
    fieldView: aiFieldView,
    selectedName: inspectedAI?.name ?? 'GAUNTLET-AI',
    state: inspectedCtrl?.debugState ?? {
      currentSpeed: inspectedAI?.speed ?? 0,
      desiredSpeed: 45.0,
      targetOffset: inspectedAI?.surface?.lateral ?? 0,
      lineOffset: 0,
      alphaF: inspectedCtrl?.alphaF ?? 0,
      alphaR: inspectedCtrl?.alphaR ?? 0,
      yawInt: inspectedCtrl?.yawInt ?? 0,
      tacticalMode: inspectedAI?.aiTactical?.racecraftPhase ?? 'PACE',
      tacticalReason: '3D DP SPEED ENVELOPE'
    }
  };

  // Update Pro Motorsport Cockpit HUD
  hud.update({
    playerVehicle: player,
    vehicles: activeVehicles,
    race: {
      raceTime: sessionTimeS,
      totalLaps: TOTAL_RACE_LAPS,
      currentLap: Math.max(1, (player.completedLaps || 0) + 1),
      phase: racePhase
    },
    aiDebugSnapshot,
    fps: metrics.fps
  });

  // Calculate Performance FPS
  frameCounter += 1;
  if (time - metricsAt >= 1000) {
    metrics.fps = Math.round((frameCounter * 1000) / (time - metricsAt));
    metrics.physicsHz = Math.round((physicsCounter * 1000) / (time - metricsAt));
    frameCounter = 0;
    physicsCounter = 0;
    metricsAt = time;
  }
}

// Start Render Loop
requestAnimationFrame((time) => {
  previousTime = time;
  metricsAt = time;
  animate(time);
});
