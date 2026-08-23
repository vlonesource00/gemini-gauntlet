import assert from 'node:assert/strict';
import { Circuit } from '../src/simulation/Track.js';
import { ENDURANCE_PARK } from '../src/scenarios/EndurancePark.js';
import { Vehicle } from '../src/simulation/Vehicle.js';
import { carSpecFor } from '../src/simulation/CarSpecs.js';
import { createTireState, tireForces } from '../src/simulation/Tire.js';
import { updateAerodynamicWakes, resolveVehicleCollisions } from '../src/simulation/VehicleInteractions.js';

const DT = 1 / 120;
const finite = (value, label) => assert.ok(Number.isFinite(value), `${label} must be finite`);

console.log('=== [1/5] Running Physics Parity Test Suite ===');

// ---------------------------------------------------------------------------
// 1. Circuit Track Geometry, Elevation, Banking, and Surface Grip
// ---------------------------------------------------------------------------
console.log('  -> Testing Circuit Track Geometry & Surface Zones...');
const track = new Circuit(ENDURANCE_PARK);

// Length should be approximately 2500m-3200m (Endurance Park authored circuit)
assert.ok(track.length >= 2400 && track.length <= 3600, `Track length must be ~2500m (got ${track.length.toFixed(1)}m)`);
assert.equal(track.samples.length, ENDURANCE_PARK.controlPoints.length * ENDURANCE_PARK.sampleDensity, 'Sample count mismatch');

const elevations = track.samples.map((sample) => sample.y);
const banks = track.samples.map((sample) => sample.bank);
const elevationDelta = Math.max(...elevations) - Math.min(...elevations);
const bankRange = Math.max(...banks) - Math.min(...banks);

assert.ok(elevationDelta > 20, `Track must have significant elevation delta (got ${elevationDelta.toFixed(2)}m)`);
assert.ok(bankRange > 0.05, `Track must have authored banking range (got ${bankRange.toFixed(4)} rad)`);

// Surface probe verification at diverse track locations
for (const fraction of [0.0, 0.15, 0.38, 0.65, 0.88]) {
  const distance = track.length * fraction;
  const sample = track.atDistance(distance);
  const roadSurface = track.surfaceAt(sample.x, sample.z);

  finite(roadSurface.height, `roadSurface[${fraction}].height`);
  finite(roadSurface.normal3.x, `roadSurface[${fraction}].normal3.x`);
  finite(roadSurface.normal3.y, `roadSurface[${fraction}].normal3.y`);
  finite(roadSurface.normal3.z, `roadSurface[${fraction}].normal3.z`);

  const normalLength = Math.hypot(roadSurface.normal3.x, roadSurface.normal3.y, roadSurface.normal3.z);
  assert.ok(Math.abs(normalLength - 1.0) < 0.02, `Surface normal must be unit vector (got ${normalLength})`);
  assert.equal(roadSurface.zone, 'road', 'Center line should be road zone');
  assert.ok(roadSurface.grip >= 1.0, 'Road grip should be >= 1.0');
}

// Surface boundary probing across lateral offset zones
const probeDistance = 200;
const probeCenter = track.atDistance(probeDistance);
const roadProbe = track.surfaceAt(probeCenter.x, probeCenter.z);
assert.equal(roadProbe.zone, 'road', 'Center of track must be road');

// Curb zone
const curbPoint = track.lateralPoint(probeCenter, track.roadHalfWidth + track.curbWidth * 0.5);
const curbProbe = track.surfaceAt(curbPoint.x, curbPoint.z);
assert.equal(curbProbe.zone, 'curb', 'Curb offset must evaluate to curb zone');
assert.ok(curbProbe.grip < roadProbe.grip, 'Curb grip must be lower than road grip');

// Runoff zone
const runoffPoint = track.lateralPoint(probeCenter, track.roadHalfWidth + track.curbWidth + 3.0);
const runoffProbe = track.surfaceAt(runoffPoint.x, runoffPoint.z);
assert.equal(runoffProbe.zone, 'runoff', 'Runoff offset must evaluate to runoff zone');
assert.ok(runoffProbe.grip < curbProbe.grip, 'Runoff grip must be lower than curb grip');

// Grass zone
const grassPoint = track.lateralPoint(probeCenter, track.roadHalfWidth + track.curbWidth + track.runoffWidth + 5.0);
const grassProbe = track.surfaceAt(grassPoint.x, grassPoint.z);
assert.equal(grassProbe.zone, 'grass', 'Far lateral offset must evaluate to grass zone');
assert.ok(grassProbe.grip < runoffProbe.grip, 'Grass grip must be lower than runoff grip');

console.log(`    [PASS] Circuit geometry: length=${track.length.toFixed(1)}m, elevation=${elevationDelta.toFixed(1)}m, bankRange=${(bankRange * 57.3).toFixed(1)}°`);

// ---------------------------------------------------------------------------
// 2. Pacejka Tire Forces, Load Sensitivity, Combined Slip, & Thermal Dynamics
// ---------------------------------------------------------------------------
console.log('  -> Testing Pacejka Tire Forces & Thermal Model...');
const settleTire = (args, spec, steps = 90) => {
  const state = createTireState(spec);
  let force;
  for (let i = 0; i < steps; i += 1) force = tireForces({ ...args, spec }, state, DT);
  return { state, force };
};

const gtTire = carSpecFor('gt').tire;

// Load sensitivity test: normal load 2500N vs 5000N
const lowLoad = settleTire({ longitudinalVelocity: 28, lateralVelocity: 0, wheelAngularSpeed: (28 / 0.335) * 1.15, radius: 0.335, normalLoad: 2500, grip: 1 }, gtTire);
const highLoad = settleTire({ longitudinalVelocity: 28, lateralVelocity: 0, wheelAngularSpeed: (28 / 0.335) * 1.15, radius: 0.335, normalLoad: 5000, grip: 1 }, gtTire);

assert.ok(Math.abs(highLoad.force.fx) > Math.abs(lowLoad.force.fx), 'Higher normal load must produce greater longitudinal force');
assert.ok(Math.abs(highLoad.force.fx) < Math.abs(lowLoad.force.fx) * 2, 'Load sensitivity must be non-linear (degressivity)');

// Combined slip reduction test (friction ellipse)
const pureCorner = settleTire({ longitudinalVelocity: 28, lateralVelocity: 3.2, wheelAngularSpeed: 28 / 0.335, radius: 0.335, normalLoad: 3300, grip: 1 }, gtTire);
const combinedCorner = settleTire({ longitudinalVelocity: 28, lateralVelocity: 3.2, wheelAngularSpeed: (28 / 0.335) * 1.22, radius: 0.335, normalLoad: 3300, grip: 1 }, gtTire);

assert.ok(Math.abs(combinedCorner.force.fy) < Math.abs(pureCorner.force.fy), 'Combined longitudinal slip must reduce lateral grip via friction ellipse');

for (const [key, value] of Object.entries(combinedCorner.force)) {
  if (typeof value === 'number') finite(value, `tire.${key}`);
}

// Thermal heating and ideal-gas pressure rise
const heatState = createTireState(gtTire);
const coldPressure = heatState.pressurePa;
for (let i = 0; i < 720; i += 1) {
  tireForces({ longitudinalVelocity: 31, lateralVelocity: 4.8, wheelAngularSpeed: (31 / 0.335) * 1.45, radius: 0.335, normalLoad: 3600, grip: 1, spec: gtTire }, heatState, DT);
}
assert.ok(heatState.carcassTemperatureC > (gtTire.ambientC ?? 22) + 1.0, 'Tire carcass must heat under sustained slip energy');
assert.ok(heatState.pressurePa > coldPressure, 'Ideal-gas pressure must rise with internal carcass temperature');

console.log(`    [PASS] Tire forces: lowLoadFx=${lowLoad.force.fx.toFixed(0)}N, highLoadFx=${highLoad.force.fx.toFixed(0)}N, pureFy=${pureCorner.force.fy.toFixed(0)}N, combinedFy=${combinedCorner.force.fy.toFixed(0)}N, hotTemp=${heatState.carcassTemperatureC.toFixed(1)}°C`);

// ---------------------------------------------------------------------------
// 3. Vehicle Physical Step, Lateral Load Transfer, & Body Roll
// ---------------------------------------------------------------------------
console.log('  -> Testing Vehicle Step, Load Transfer & Roll Attitude...');
const flatTrack = {
  length: 1000,
  atDistance: (distance) => ({
    x: 0, y: 0, z: distance, s: distance, index: 0,
    tangent: { x: 0, z: 1 }, normal: { x: -1, z: 0 },
    normal3: { x: 0, y: 1, z: 0 }, grade: 0, bank: 0,
    curvature: 0, turnSign: 0, turnStrength: 0
  }),
  surfaceAt: (x, z) => ({
    x, y: 0, z, s: 0, index: 0, lateral: -x,
    tangent: { x: 0, z: 1 }, normal: { x: -1, z: 0 },
    normal3: { x: 0, y: 1, z: 0 }, grade: 0, bank: 0, height: 0,
    grip: 1.08, zone: 'road', barrierDepth: 0, curbHeight: 0, roughness: 0,
    roadEdge: 6.5, curbEdge: 7.55, barrierEdge: 16.05, turnStrength: 0
  }),
  updateTirePass() {}
};

const handling = new Vehicle({ id: 'handling-test', spec: 'gt', player: true });
handling.place(0, 0, 0, handling.rideHeight);
handling.velocity.z = 24;
for (const wheel of handling.wheels) wheel.omega = handling.velocity.z / handling.wheelRadius;

let maxSlipAngle = 0;
for (let step = 0; step < 90; step += 1) {
  handling.controls = { throttle: 0, brake: 0, steer: 0.22, handbrake: 0 };
  handling.step(DT, flatTrack, true);
  maxSlipAngle = Math.max(maxSlipAngle, ...handling.wheels.map((w) => Math.abs(w.slipAngle)));
}

const outsideCompression = handling.wheels[0].compression + handling.wheels[2].compression;
const insideCompression = handling.wheels[1].compression + handling.wheels[3].compression;

assert.ok(handling.yaw > 0.1, 'Positive steer must produce positive yaw');
assert.ok(handling.yawRate > 0, 'Positive steer must produce positive yaw rate');
assert.ok(handling.speed > 15, 'Moderate steering should not cause instant spin or stoppage');
assert.ok(outsideCompression > insideCompression, 'Lateral acceleration must compress outside suspension more than inside (load transfer)');
assert.ok(handling.roll > 0, 'Body roll attitude must follow outside loaded side');
assert.ok(Math.abs(handling.roll) < (6 * Math.PI) / 180, 'Body roll must remain bounded within realistic suspension travel (<6 deg)');

for (const [key, value] of Object.entries(handling.position)) finite(value, `handling.position.${key}`);
finite(handling.yaw, 'handling.yaw');
finite(handling.yawRate, 'handling.yawRate');

// TCS & ABS controller level clamping
assert.equal(handling.setTCLevel(99), 7, 'TC level upper limit clamp');
assert.equal(handling.setTCLevel(-4), 0, 'TC level lower limit clamp');
assert.equal(handling.setABSLevel(99), 7, 'ABS level upper limit clamp');
assert.equal(handling.setABSLevel(-4), 0, 'ABS level lower limit clamp');

console.log(`    [PASS] Vehicle handling: yaw=${handling.yaw.toFixed(3)}rad, yawRate=${handling.yawRate.toFixed(3)}rad/s, roll=${(handling.roll * 57.3).toFixed(2)}°, outsideCompression=${outsideCompression.toFixed(3)}m vs inside=${insideCompression.toFixed(3)}m`);

// ---------------------------------------------------------------------------
// 4. Vehicle Class Acceleration & Aerodynamic Downforce Hierarchy
// ---------------------------------------------------------------------------
console.log('  -> Testing Acceleration Hierarchy & Downforce Parity across Car Classes...');
const launchSpeed = (classKey) => {
  const vehicle = new Vehicle({ id: `launch-${classKey}`, spec: classKey });
  vehicle.resetTo(track, 0, 0);
  vehicle.setTCLevel(3);
  for (let i = 0; i < 420; i += 1) {
    vehicle.controls = { throttle: 1, brake: 0, steer: 0.02, handbrake: 0 };
    vehicle.step(DT, track, true);
  }
  return vehicle.speed;
};

const gtLaunch = launchSpeed('gt');
const prototypeLaunch = launchSpeed('prototype');
const touringLaunch = launchSpeed('touring');

assert.ok(prototypeLaunch > gtLaunch * 1.03, `Prototype (${(prototypeLaunch * 3.6).toFixed(1)} km/h) must accelerate faster than GT (${(gtLaunch * 3.6).toFixed(1)} km/h)`);
assert.ok(gtLaunch > touringLaunch * 1.02, `GT (${(gtLaunch * 3.6).toFixed(1)} km/h) must accelerate faster than Touring (${(touringLaunch * 3.6).toFixed(1)} km/h)`);

const protoSpec = carSpecFor('prototype');
const gtSpec = carSpecFor('gt');
const protoDownforce = (protoSpec.aero.frontCl ?? 0) + (protoSpec.aero.rearCl ?? 0);
const gtDownforce = (gtSpec.aero.frontCl ?? 0) + (gtSpec.aero.rearCl ?? 0);
assert.ok(protoDownforce > gtDownforce, `Prototype downforce coefficient (${protoDownforce}) must exceed GT (${gtDownforce})`);

console.log(`    [PASS] Class hierarchy: Prototype=${(prototypeLaunch * 3.6).toFixed(1)} km/h > GT=${(gtLaunch * 3.6).toFixed(1)} km/h > Touring=${(touringLaunch * 3.6).toFixed(1)} km/h`);

// ---------------------------------------------------------------------------
// 5. Aerodynamic Wake & Slipstream Drag Reduction
// ---------------------------------------------------------------------------
console.log('  -> Testing Aerodynamic Wake & Slipstream Drag Reduction...');
const leader = new Vehicle({ id: 'wake-leader', spec: 'prototype' });
const followerInDraft = new Vehicle({ id: 'wake-follower', spec: 'prototype' });
const followerWide = new Vehicle({ id: 'wake-follower-wide', spec: 'prototype' });

leader.resetTo(track, 200, 0.0);
followerInDraft.resetTo(track, 185, 0.0); // 15m directly behind
followerWide.resetTo(track, 185, 8.0);   // 15m behind, 8m lateral offset

const setVehicleVelocity = (vehicle, speed) => {
  const p = track.atDistance(vehicle.distance);
  vehicle.speed = speed;
  vehicle.velocity = { x: p.tangent.x * speed, y: 0, z: p.tangent.z * speed };
};

setVehicleVelocity(leader, 60);
setVehicleVelocity(followerInDraft, 60);
setVehicleVelocity(followerWide, 60);

const wakeStats = updateAerodynamicWakes([leader, followerInDraft, followerWide]);

assert.ok(wakeStats.activeWakes >= 1, 'At least one vehicle must experience active aerodynamic wake');
assert.ok(followerInDraft.wake.strength > 0.2, 'Directly trailing vehicle must experience strong slipstream wake');
assert.ok(followerInDraft.wake.dragReduction > 0.02, 'Slipstream must reduce aerodynamic drag coefficient');
assert.ok(followerInDraft.wake.dragMultiplier < 1.0, 'Drag multiplier in wake must be less than 1.0');
assert.equal(followerInDraft.wake.sourceId, leader.id, 'Wake source must point to the leading vehicle');

assert.ok(followerWide.wake.strength < 0.01, 'Far lateral vehicle outside wake cone must receive negligible wake');
assert.equal(followerWide.wake.dragMultiplier, 1.0, 'Vehicle outside wake must have 1.0 drag multiplier');

console.log(`    [PASS] Wake & Slipstream: draftStrength=${followerInDraft.wake.strength.toFixed(3)}, dragReduction=${(followerInDraft.wake.dragReduction * 100).toFixed(1)}%, dragMultiplier=${followerInDraft.wake.dragMultiplier.toFixed(3)}`);

console.log('=== Physics Parity Test Suite: ALL ASSERTIONS PASSED ===\n');
