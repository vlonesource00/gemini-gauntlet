import * as THREE from 'three';
import './style.css';
import { Circuit } from './simulation/Track.js';
import { Vehicle } from './simulation/Vehicle.js';
import { updateAerodynamicWakes, resolveVehicleCollisions } from './simulation/VehicleInteractions.js';
import { ENDURANCE_PARK } from './scenarios/EndurancePark.js';
import { ScenarioEngine } from './scenarios/ScenarioEngine.js';
import { ResearchAIController } from './ai/ResearchAIController.js';
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

// Track & Environment
const track = new Circuit(ENDURANCE_PARK);
const environment = new CircuitEnvironment(scene, track);

// Audio & Input
const audio = new SynthAudio();
const input = new InputManager(() => {
  audio.unlock().catch(() => {});
});

// Vehicles Setup
// Player = Prototype Class (Cyan), AI = Prototype / GT Class (Neon Amber / Orange)
const player = new Vehicle({
  id: 'player',
  name: 'MARTIM',
  color: '#00f0ff',
  player: true,
  spec: 'prototype'
});

const aiVehicle = new Vehicle({
  id: 'ai-1',
  name: 'GAUNTLET-AI',
  color: '#ff9900',
  player: false,
  spec: 'prototype'
});

const vehicles = [player, aiVehicle];

// Research AI Controllers & Scenario Engine
const aiController = new ResearchAIController(1, {
  aggression: 0.75,
  diveMargin: 0.45,
  defenseReactivity: 0.8,
  kerbUsage: 0.9,
  lookahead: 22.0
});

const playerPaceAI = new ResearchAIController(2, {
  aggression: 0.65,
  diveMargin: 0.35,
  defenseReactivity: 0.7,
  kerbUsage: 0.8,
  lookahead: 24.0
});

// User Reference Lap Recorder & Baseline Engine
const lapRecorder = new ReferenceLapManager(track);
aiController.setReferenceProfile(lapRecorder);
playerPaceAI.setReferenceProfile(lapRecorder);

const scenarioEngine = new ScenarioEngine(track, player, aiVehicle, aiController);

// Visual Car Models
const visuals = [
  new CarVisual(player, { variant: 'prototype' }),
  new CarVisual(aiVehicle, { variant: 'prototype' })
];
visuals.forEach((v) => scene.add(v.group));

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
  visuals.forEach((v) => v.attachAsset(assets));
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

// UI Scenario Control Deck
const scenarioDeck = new ScenarioDeck({
  onScenarioSelect: (scenarioId) => {
    scenarioEngine.loadScenario(scenarioId);
    audio.unlock().catch(() => {});
  },
  onReset: () => {
    scenarioEngine.resetScenario();
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
    statusFor: (v) => ({ position: v === player ? 1 : 2 })
  };

  // Player Controls: manual driving if keys pressed, otherwise autonomous pace along track
  const isInteracting = input.isInteracting();
  if (cameraRig.mode === 'FREE') {
    Object.assign(player.controls, { throttle: 0, brake: 0, steer: 0, handbrake: 0 });
  } else if (!autopilotActive) {
    // 100% Pure manual driving
    Object.assign(player.controls, input.controls(player, dt));
  } else if (isInteracting) {
    // Autopilot mode with manual user override
    Object.assign(player.controls, input.controls(player, dt));
  } else {
    // Autonomous Pace Cruise for player
    playerPaceAI.update(player, vehicles, track, raceState, dt);
    const scenario = scenarioEngine.activeScenario;
    if (scenario?.playerConfig?.initialSpeedMps) {
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

  // Update AI controller with valid race state
  aiController.update(aiVehicle, vehicles, track, raceState, dt);

  // Aerodynamic wake & slipstream
  updateAerodynamicWakes(vehicles);

  // Physics stepping
  player.step(dt, track, true);
  aiVehicle.step(dt, track, true);

  // Vehicle-vehicle collision resolution
  const collisionStats = resolveVehicleCollisions(vehicles, 3);

  // Scenario engine evaluation
  scenarioEngine.update(dt, collisionStats);

  // Reference Lap recording & multi-vehicle telemetry (Player + AI)
  lapRecorder.update(vehicles, dt);

  physicsCounter += 1;
}

// Keyboard Actions / Shortcuts
function processActions() {
  if (input.consume('KeyR')) {
    scenarioEngine.resetScenario();
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
  }
  if (input.consume('Tab')) {
    aiDebug.visible = !aiDebug.visible;
    aiDebug.group.visible = aiDebug.visible;
  }
  if (input.consume('KeyM')) {
    const muted = audio.toggleMute();
    if (elBtnMute) elBtnMute.textContent = muted ? 'AUDIO [MUTED]' : 'AUDIO [M]';
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

  // Update visuals
  visuals.forEach((v) => v.update(rawDelta));
  environment.update(rawDelta);

  // Update AI Debug Suite
  if (aiDebug.update) {
    aiDebug.update(aiController, aiVehicle, player, track, now);
  }

  // Update Camera Rig
  if (cameraRig.mode === 'FREE') {
    cameraRig.updateFree(input.freeCameraRaw(), rawDelta);
  } else {
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

  // Update Live HUD Readouts
  updateHUDReadouts();

  // Render WebGL Scene
  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}

// Update HUD Elements
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

  // Gap to AI
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

  // MoTeC G-G circle dot position (41px center, +/- 36px range)
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

  // Update AI Thought HUD from AI Controller telemetry
  const aiTelemetry = aiController.telemetry || {};
  scenarioDeck.updateAIThoughtHUD({
    state: aiTelemetry.state || aiTelemetry.maneuver || 'TRAJECTORY_FOLLOW',
    action: aiTelemetry.action || (aiVehicle.speed > 50 ? 'ATTACK DRAFT' : 'CRUISE'),
    reason: aiTelemetry.reason || 'Optimal corridor tracking at 120Hz',
    threat: aiTelemetry.threatLevel || 'LOW',
    aggression: Math.round((aiController._aggression || 0.75) * 100),
    diveMargin: (aiController._diveMargin || 0.45),
    defense: Math.round((aiController._defenseReactivity || 0.8) * 100),
    kerb: Math.round((aiController._kerbUsage || 0.9) * 100),
    lookahead: aiController._lookahead || 22.0,
    throttle: aiVehicle.controls?.throttle || 0,
    brake: aiVehicle.controls?.brake || 0,
    steer: aiVehicle.controls?.steer || 0,
    tactic: aiTelemetry.tactic || 'Dynamic Frenet Lattice Evaluation',
    prediction: aiTelemetry.prediction || 'Clear inside apex window',
    decision: aiTelemetry.decision || 'Hold tactical spacing'
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
  vehicles,
  player,
  aiVehicle,
  aiController,
  scenarioEngine,
  aiDebug,
  scenarioDeck,
  assets,
  audio,
  input,
  metrics,
  SCENARIO_CATALOG,
  get isPaused() { return isPaused; },
  get simTimeScale() { return simTimeScale; },
  resetScenario: () => scenarioEngine.resetScenario(),
  loadScenario: (id) => scenarioEngine.loadScenario(id),
  setSimSpeed: (speed) => scenarioDeck.onSpeedChange(speed)
};

document.documentElement.dataset.ready = 'true';
setTimeout(() => loadingScreen?.classList.add('dismissed'), 500);
