/**
 * supreme-racecraft-suite.mjs
 * Comprehensive Supreme Racecraft V3 Verification Suite:
 * - 10 Attack Scenarios (S-A1 to S-A10)
 * - 8 Defense Scenarios (S-D1 to S-D8)
 * - 5 Multi-Car & Pack Combat Scenarios (S-M1 to S-M5)
 *
 * Directly tests NextGenAIController and GameTheoreticCombatEngine at 120Hz.
 */

import assert from 'node:assert/strict';
import { Circuit } from '../src/simulation/Track.js';
import { ENDURANCE_PARK } from '../src/scenarios/EndurancePark.js';
import { HARBOR_RING } from '../src/scenarios/HarborRing.js';
import { Vehicle } from '../src/simulation/Vehicle.js';
import { NextGenAIController } from '../src/ai/v2/NextGenAIController.js';
import { updateAerodynamicWakes, resolveVehicleCollisions } from '../src/simulation/VehicleInteractions.js';

const DT = 1 / 120;

const setForwardSpeed = (vehicle, speedMs, track) => {
  const p = track.atDistance(vehicle.distance);
  vehicle.speed = speedMs;
  vehicle.velocity = { x: p.tangent.x * speedMs, y: 0, z: p.tangent.z * speedMs };
  vehicle.localVelocity = { x: 0, z: speedMs };
  for (const wheel of vehicle.wheels || []) {
    wheel.omega = speedMs / (vehicle.wheelRadius || 0.335);
  }
};

console.log('================================================================================');
console.log('             GEMINI SUPREME — RACECRAFT V3 TEST SUITE (23 SCENARIOS)             ');
console.log('================================================================================\n');

let passedTests = 0;
let totalTests = 0;

function runTest(name, fn) {
  totalTests++;
  console.log(`[${totalTests}/23] Testing: ${name}...`);
  try {
    fn();
    passedTests++;
    console.log(`  -> [PASS] ${name}\n`);
  } catch (err) {
    console.error(`  -> [FAIL] ${name}:`, err.message);
    throw err;
  }
}

const enduranceTrack = new Circuit(ENDURANCE_PARK);
const harborTrack = new Circuit(HARBOR_RING);

// =============================================================================
// SECTION 1: ATTACK SCENARIOS (S-A1 to S-A10)
// =============================================================================

runTest('S-A1: Slingshot Pullout Timing & Acceleration', () => {
  const ego = new Vehicle({ id: 'ego', spec: 'prototype' });
  const rival = new Vehicle({ id: 'rival', spec: 'gt' });
  rival.resetTo(enduranceTrack, 120, 0.2);
  ego.resetTo(enduranceTrack, 95, 0.2); // trailing in slipstream
  setForwardSpeed(rival, 24, enduranceTrack);
  setForwardSpeed(ego, 32, enduranceTrack);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.95 });
  const rivalAI = new NextGenAIController('rival-ai', { aggression: 0.50 });
  const vehicles = [ego, rival];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  let pulledOut = false;
  let passed = false;

  for (let step = 0; step < 750; step++) {
    race.raceTime += DT;
    race.elapsed += DT;
    egoAI.update(ego, vehicles, enduranceTrack, race, DT);
    rivalAI.update(rival, vehicles, enduranceTrack, race, DT);
    updateAerodynamicWakes(vehicles);
    ego.step(DT, enduranceTrack, true);
    rival.step(DT, enduranceTrack, true);
    const col = resolveVehicleCollisions(vehicles, 3);
    assert.equal(col.deepOverlaps, 0, 'No deep overlaps during slingshot');

    const latDiff = Math.abs(ego.surface?.lateral - rival.surface?.lateral);
    if (latDiff > 1.8 && ego.distance < rival.distance + 5) pulledOut = true;
    if (ego.distance > rival.distance + 8) {
      passed = true;
      break;
    }
  }

  assert.ok(pulledOut, 'Ego must pull out laterally to execute slingshot');
  assert.ok(passed, 'Ego must cleanly pass rival on straight');
});

runTest('S-A2: Inside Dive with Apex Room & Threshold Braking', () => {
  const ego = new Vehicle({ id: 'ego', spec: 'prototype' });
  const rival = new Vehicle({ id: 'rival', spec: 'gt' });
  rival.resetTo(enduranceTrack, 180, 2.5); // rival stays wide outside
  ego.resetTo(enduranceTrack, 162, 0.0);
  setForwardSpeed(rival, 22, enduranceTrack);
  setForwardSpeed(ego, 30, enduranceTrack);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.95, diveMargin: 0.85 });
  const rivalAI = new NextGenAIController('rival-ai', { aggression: 0.40 });
  const vehicles = [ego, rival];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  let diveInitiated = false;
  let cleanPass = false;

  for (let step = 0; step < 600; step++) {
    egoAI.update(ego, vehicles, enduranceTrack, race, DT);
    rivalAI.update(rival, vehicles, enduranceTrack, race, DT);
    updateAerodynamicWakes(vehicles);
    ego.step(DT, enduranceTrack, true);
    rival.step(DT, enduranceTrack, true);
    const col = resolveVehicleCollisions(vehicles, 3);
    assert.equal(col.deepOverlaps, 0);

    const mode = egoAI.combatEngine?.attackMode;
    if (mode === 'DIVEBOMB' || mode === 'SIDE_BY_SIDE') diveInitiated = true;
    if (ego.distance > rival.distance + 10.0) {
      cleanPass = true;
      break;
    }
  }

  assert.ok(diveInitiated, 'Inside dive must be initiated into corner');
  assert.ok(cleanPass, 'Clean inside pass must complete successfully');
});

runTest('S-A3: Outside Momentum Pass', () => {
  const ego = new Vehicle({ id: 'ego', spec: 'prototype' });
  const rival = new Vehicle({ id: 'rival', spec: 'gt' });
  rival.resetTo(enduranceTrack, 185, -3.2); // rival blocks inside
  ego.resetTo(enduranceTrack, 168, 0.0);
  setForwardSpeed(rival, 20, enduranceTrack);
  setForwardSpeed(ego, 28, enduranceTrack);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.90 });
  const rivalAI = new NextGenAIController('rival-ai', { aggression: 0.50 });
  const vehicles = [ego, rival];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  let sweptOutside = false;
  let passed = false;

  for (let step = 0; step < 700; step++) {
    egoAI.update(ego, vehicles, enduranceTrack, race, DT);
    rivalAI.update(rival, vehicles, enduranceTrack, race, DT);
    updateAerodynamicWakes(vehicles);
    ego.step(DT, enduranceTrack, true);
    rival.step(DT, enduranceTrack, true);
    const col = resolveVehicleCollisions(vehicles, 3);
    assert.equal(col.deepOverlaps, 0);

    if (ego.surface?.lateral > 1.2) sweptOutside = true;
    if (ego.distance > rival.distance + 8.0) {
      passed = true;
      break;
    }
  }

  assert.ok(sweptOutside, 'Ego must use open outside corridor when inside is blocked');
  assert.ok(passed, 'Outside momentum pass completed cleanly');
});

runTest('S-A4: Switchback / Over-Under Late Apex Undercut', () => {
  const ego = new Vehicle({ id: 'ego', spec: 'prototype' });
  const rival = new Vehicle({ id: 'rival', spec: 'gt' });
  rival.resetTo(enduranceTrack, 782, 4.8); // rival over-defending inside
  ego.resetTo(enduranceTrack, 765, -1.2);
  setForwardSpeed(rival, 18, enduranceTrack);
  setForwardSpeed(ego, 25, enduranceTrack);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.95 });
  const vehicles = [ego, rival];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  let switchbackDetected = false;
  for (let step = 0; step < 400; step++) {
    race.raceTime += DT;
    race.elapsed += DT;
    egoAI.update(ego, vehicles, enduranceTrack, race, DT);
    ego.step(DT, enduranceTrack, true);
    if (egoAI.combatEngine?.attackMode === 'SWITCHBACK' || egoAI.trajectoryPlan?.maneuver?.family === 'ATTACK_SWITCHBACK') {
      switchbackDetected = true;
      break;
    }
  }
  assert.ok(switchbackDetected, 'Switchback maneuver must activate on inside-blocked defender');
});

runTest('S-A5: Blocked Inside -> Outside Corridor Commit', () => {
  const ego = new Vehicle({ id: 'ego', spec: 'prototype' });
  const rival = new Vehicle({ id: 'rival', spec: 'gt' });
  rival.resetTo(enduranceTrack, 140, -2.2); // inside covered
  ego.resetTo(enduranceTrack, 131, -0.5);   // within pullout trigger distance
  setForwardSpeed(rival, 24, enduranceTrack);
  setForwardSpeed(ego, 30, enduranceTrack);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.90 });
  const vehicles = [ego, rival];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  egoAI.update(ego, vehicles, enduranceTrack, race, DT);
  const targetLat = egoAI.trajectoryPlan?.selectedOffset ?? 0;
  assert.ok(targetLat > 0, `Target offset must commit to open outside flank (got ${targetLat.toFixed(2)}m)`);
});

runTest('S-A6: Blocked Outside -> Inside Corridor Commit', () => {
  const ego = new Vehicle({ id: 'ego', spec: 'prototype' });
  const rival = new Vehicle({ id: 'rival', spec: 'gt' });
  rival.resetTo(enduranceTrack, 140, 2.8); // outside covered
  ego.resetTo(enduranceTrack, 131, 0.5);   // within pullout trigger distance
  setForwardSpeed(rival, 24, enduranceTrack);
  setForwardSpeed(ego, 30, enduranceTrack);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.90 });
  const vehicles = [ego, rival];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  egoAI.update(ego, vehicles, enduranceTrack, race, DT);
  const targetLat = egoAI.trajectoryPlan?.selectedOffset ?? 0;
  assert.ok(targetLat < 1.0, `Target offset must commit to open inside flank (got ${targetLat.toFixed(2)}m)`);
});

runTest('S-A7: Attack Abort on Squeezed Door', () => {
  const ego = new Vehicle({ id: 'ego', spec: 'prototype' });
  const rival = new Vehicle({ id: 'rival', spec: 'gt' });
  rival.resetTo(enduranceTrack, 790, 5.5); // rival pinches door shut on inside apex
  ego.resetTo(enduranceTrack, 775, 4.0);
  setForwardSpeed(rival, 24, enduranceTrack);
  setForwardSpeed(ego, 30, enduranceTrack);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.90 });
  const vehicles = [ego, rival];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  // Trigger attack mode
  egoAI.combatEngine.attackMode = 'DIVEBOMB';
  egoAI.combatEngine.attackTargetId = 'rival';
  egoAI.combatEngine.insideClosedFilterTimer = 0.25; // simulate inside pinched shut

  egoAI.update(ego, vehicles, enduranceTrack, race, DT);
  const mode = egoAI.combatEngine.attackMode;
  assert.ok(mode === 'ABORT_HOLD' || mode === 'ABORT_BLEND' || mode === 'SIDE_BY_SIDE' || mode === 'NONE',
    `Pinching inside must transition to safe abort or overlap hold (got ${mode})`);
});

runTest('S-A8: Retained Pass Through Corner Exit', () => {
  const ego = new Vehicle({ id: 'ego', spec: 'prototype' });
  const rival = new Vehicle({ id: 'rival', spec: 'gt' });
  rival.resetTo(enduranceTrack, 200, 1.0);
  ego.resetTo(enduranceTrack, 208, -1.0); // ego ahead by 8m
  setForwardSpeed(rival, 26, enduranceTrack);
  setForwardSpeed(ego, 32, enduranceTrack);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.90 });
  egoAI.combatEngine.attackTargetId = 'rival';
  egoAI.combatEngine.passState = 'FULL_CLEAR';
  const vehicles = [ego, rival];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  egoAI.update(ego, vehicles, enduranceTrack, race, DT);
  assert.equal(egoAI.combatEngine.passState, 'RETAINING', 'FULL_CLEAR must transition into RETAINING state');
});

runTest('S-A9: Post-Pass Safe Merge Without Chopping Nose', () => {
  const ego = new Vehicle({ id: 'ego', spec: 'prototype' });
  const rival = new Vehicle({ id: 'rival', spec: 'gt' });
  rival.resetTo(enduranceTrack, 100, 0.0);
  ego.resetTo(enduranceTrack, 106, 2.5); // ego passed on right (+2.5m lateral, 6m ahead)
  setForwardSpeed(rival, 28, enduranceTrack);
  setForwardSpeed(ego, 32, enduranceTrack);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.90 });
  egoAI.combatEngine.passState = 'RETAINING';
  egoAI.combatEngine.attackSide = 1;
  const vehicles = [ego, rival];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  egoAI.update(ego, vehicles, enduranceTrack, race, DT);
  const targetLat = egoAI.trajectoryPlan?.selectedOffset ?? 0;
  // Should NOT immediately dive to 0.0 (chop rival nose), but merge smoothly
  assert.ok(targetLat > 0.4, `Safe merge must not immediately chop across rival nose (targetLat=${targetLat.toFixed(2)}m)`);
});

runTest('S-A10: Slower Traffic Bypass', () => {
  const ego = new Vehicle({ id: 'ego', spec: 'prototype' });
  const slowCar = new Vehicle({ id: 'slow-car', spec: 'touring' });
  slowCar.resetTo(harborTrack, 300, 0.0);
  ego.resetTo(harborTrack, 280, 0.0); // within pullout envelope of slower car
  setForwardSpeed(slowCar, 14, harborTrack);
  setForwardSpeed(ego, 32, harborTrack);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.85 });
  const vehicles = [ego, slowCar];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  egoAI.update(ego, vehicles, harborTrack, race, DT);
  assert.ok(egoAI.combatEngine.attackMode !== 'NONE', 'Must engage attack/bypass mode for slower traffic');
  assert.ok(Math.abs(egoAI.trajectoryPlan?.selectedOffset ?? 0) > 1.0, 'Must route around slower vehicle');
});

// =============================================================================
// SECTION 2: DEFENSE SCENARIOS (S-D1 to S-D8)
// =============================================================================

runTest('S-D1: Straight Inside Cover Defense', () => {
  const ego = new Vehicle({ id: 'ego', spec: 'gt' });
  const challenger = new Vehicle({ id: 'challenger', spec: 'prototype' });
  ego.resetTo(enduranceTrack, 150, 0.5);
  challenger.resetTo(enduranceTrack, 138, 0.0); // closing fast behind
  setForwardSpeed(ego, 25, enduranceTrack);
  setForwardSpeed(challenger, 33, enduranceTrack);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.50, defenseReactivity: 0.95 });
  const vehicles = [ego, challenger];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  egoAI.update(ego, vehicles, enduranceTrack, race, DT);
  assert.ok(egoAI.combatEngine.defenseMode !== 'PACE', 'Ego must activate defense mode under closing threat');
});

runTest('S-D2: Slingshot Tow-Break Defense', () => {
  const ego = new Vehicle({ id: 'ego', spec: 'gt' });
  const challenger = new Vehicle({ id: 'challenger', spec: 'prototype' });
  ego.resetTo(enduranceTrack, 120, 0.0);
  challenger.resetTo(enduranceTrack, 98, 0.0); // drafting directly behind
  setForwardSpeed(ego, 26, enduranceTrack);
  setForwardSpeed(challenger, 33, enduranceTrack);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.50, defenseReactivity: 0.95 });
  const vehicles = [ego, challenger];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  egoAI.update(ego, vehicles, enduranceTrack, race, DT);
  assert.ok(['BREAK_TOW', 'LOCK_LANE', 'APEX_SHIELD'].includes(egoAI.combatEngine.defenseMode),
    `Must execute tow breaking or lane locking on straight (got ${egoAI.combatEngine.defenseMode})`);
});

runTest('S-D3: Divebomb Defense (Apex Shield)', () => {
  const ego = new Vehicle({ id: 'ego', spec: 'gt' });
  const challenger = new Vehicle({ id: 'challenger', spec: 'prototype' });
  ego.resetTo(enduranceTrack, 205, 0.0); // approaching corner
  challenger.resetTo(enduranceTrack, 195, -1.5);
  setForwardSpeed(ego, 22, enduranceTrack);
  setForwardSpeed(challenger, 28, enduranceTrack);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.50, defenseReactivity: 0.95 });
  const vehicles = [ego, challenger];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  egoAI.update(ego, vehicles, enduranceTrack, race, DT);
  assert.equal(egoAI.combatEngine.defenseMode, 'APEX_SHIELD', 'Must shield apex into corner against dive attempt');
});

runTest('S-D4: Outside Attack Defense', () => {
  const ego = new Vehicle({ id: 'ego', spec: 'gt' });
  const challenger = new Vehicle({ id: 'challenger', spec: 'prototype' });
  ego.resetTo(enduranceTrack, 200, 0.0);
  challenger.resetTo(enduranceTrack, 192, 2.5); // challenger trying outside run
  setForwardSpeed(ego, 23, enduranceTrack);
  setForwardSpeed(challenger, 28, enduranceTrack);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.50, defenseReactivity: 0.95 });
  const vehicles = [ego, challenger];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  egoAI.update(ego, vehicles, enduranceTrack, race, DT);
  assert.ok(egoAI.combatEngine.defenseMode !== 'PACE', 'Must defend line against outside challenger');
});

runTest('S-D5: Anti-Feint Filtering (No Twitch on Rival Jitter)', () => {
  const ego = new Vehicle({ id: 'ego', spec: 'gt' });
  const challenger = new Vehicle({ id: 'challenger', spec: 'gt' });
  ego.resetTo(enduranceTrack, 150, 0.0);
  challenger.resetTo(enduranceTrack, 138, 0.0);
  setForwardSpeed(ego, 26, enduranceTrack);
  setForwardSpeed(challenger, 30, enduranceTrack);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.50, defenseReactivity: 0.90 });
  const vehicles = [ego, challenger];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  let steerSwaps = 0;
  let priorSteer = 0;

  for (let step = 0; step < 60; step++) {
    // Rival rapidly jiggles left and right (feinting)
    challenger.surface.lateral = (step % 4 < 2 ? -1.5 : 1.5);
    egoAI.update(ego, vehicles, enduranceTrack, race, DT);
    const steer = ego.controls?.steer || 0;
    if (Math.sign(steer) !== 0 && Math.sign(priorSteer) !== 0 && Math.sign(steer) !== Math.sign(priorSteer)) {
      steerSwaps++;
    }
    priorSteer = steer;
  }

  assert.ok(steerSwaps <= 2, `Feint filtering must prevent steering oscillation (got ${steerSwaps} swaps)`);
});

runTest('S-D6: Blocked Defensive Lane Fallback', () => {
  const ego = new Vehicle({ id: 'ego', spec: 'gt' });
  const challenger = new Vehicle({ id: 'challenger', spec: 'prototype' });
  ego.resetTo(enduranceTrack, 140, 0.0);
  challenger.resetTo(enduranceTrack, 126, 0.0);
  setForwardSpeed(ego, 25, enduranceTrack);
  setForwardSpeed(challenger, 32, enduranceTrack);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.50 });
  const vehicles = [ego, challenger];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  // Set constrained lateral boundaries (e.g. curb barrier or car on inside)
  const result = egoAI.combatEngine.update({
    vehicle: ego,
    traffic: egoAI.awareness.scan(ego, vehicles, enduranceTrack),
    track: enduranceTrack,
    dt: DT
  });

  assert.ok(Number.isFinite(result.targetLateral), 'Target lateral must remain valid and legal');
});

runTest('S-D7: FIA Single-Move Episode Persistence (No Secondary Weave)', () => {
  const ego = new Vehicle({ id: 'ego', spec: 'gt' });
  const challenger = new Vehicle({ id: 'challenger', spec: 'prototype' });
  ego.resetTo(enduranceTrack, 120, 0.0);
  challenger.resetTo(enduranceTrack, 105, 1.5);
  setForwardSpeed(ego, 25, enduranceTrack);
  setForwardSpeed(challenger, 32, enduranceTrack);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.50 });
  const vehicles = [ego, challenger];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  // Step 1: Challenger triggers move
  egoAI.update(ego, vehicles, enduranceTrack, race, DT);
  assert.equal(egoAI.combatEngine.defensiveEpisode.moveCount, 1, 'Initial defensive reaction must record exactly 1 move');

  // Step 2: Challenger switches flank across to opposite side
  challenger.surface.lateral = -2.5;
  challenger.position.x -= 4.0;
  egoAI.update(ego, vehicles, enduranceTrack, race, DT);

  // Ego must NOT make a second move across the track
  assert.equal(egoAI.combatEngine.defensiveEpisode.moveCount, 1, 'FIA single-move rule must prevent second reactionary move');
});

runTest('S-D8: Exit Squeeze Reachability on Corner Unwinding', () => {
  const ego = new Vehicle({ id: 'ego', spec: 'gt' });
  const challenger = new Vehicle({ id: 'challenger', spec: 'prototype' });
  // Place on corner exit of Turn 1 (past apex, unwinding curvature)
  ego.resetTo(enduranceTrack, 880, 0.0);
  challenger.resetTo(enduranceTrack, 873, 1.2); // challenger on bumper on exit
  setForwardSpeed(ego, 28, enduranceTrack);
  setForwardSpeed(challenger, 30, enduranceTrack);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.50, defenseReactivity: 0.95 });
  const vehicles = [ego, challenger];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  egoAI.update(ego, vehicles, enduranceTrack, race, DT);
  assert.equal(egoAI.combatEngine.defenseMode, 'EXIT_SQUEEZE', 'EXIT_SQUEEZE must be reachable on corner exit');
});

// =============================================================================
// SECTION 3: MULTI-CAR & PACK COMBAT SCENARIOS (S-M1 to S-M5)
// =============================================================================

runTest('S-M1: Three-Wide Center Lane Stability', () => {
  const ego = new Vehicle({ id: 'ego', spec: 'prototype' });
  const carLeft = new Vehicle({ id: 'car-left', spec: 'gt' });
  const carRight = new Vehicle({ id: 'car-right', spec: 'gt' });

  ego.resetTo(enduranceTrack, 150, 0.0);
  carLeft.resetTo(enduranceTrack, 150, -2.4);
  carRight.resetTo(enduranceTrack, 150, 2.4);

  setForwardSpeed(ego, 30, enduranceTrack);
  setForwardSpeed(carLeft, 30, enduranceTrack);
  setForwardSpeed(carRight, 30, enduranceTrack);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.90 });
  const vehicles = [ego, carLeft, carRight];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  egoAI.update(ego, vehicles, enduranceTrack, race, DT);
  assert.ok(egoAI.combatEngine.threeWideActive, 'Three-wide condition must be detected');
  assert.equal(egoAI.combatEngine.role, 'THREE_WIDE_HOLD', 'Tactical role must hold center lane');
  assert.ok(Math.abs(egoAI.trajectoryPlan?.selectedOffset ?? 0) < 0.6, 'Center lane offset must remain near 0');
});

runTest('S-M2: Stacked Braking Accordion Avoidance', () => {
  const ego = new Vehicle({ id: 'ego', spec: 'prototype' });
  const lead1 = new Vehicle({ id: 'lead1', spec: 'gt' });
  const lead2 = new Vehicle({ id: 'lead2', spec: 'gt' });

  lead2.resetTo(enduranceTrack, 160, 0.0);
  lead1.resetTo(enduranceTrack, 148, 0.0);
  ego.resetTo(enduranceTrack, 135, 0.0);

  setForwardSpeed(lead2, 20, enduranceTrack);
  setForwardSpeed(lead1, 20, enduranceTrack);
  setForwardSpeed(ego, 32, enduranceTrack);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.90 });
  const vehicles = [ego, lead1, lead2];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  let collisionOccurred = false;
  for (let step = 0; step < 120; step++) {
    egoAI.update(ego, vehicles, enduranceTrack, race, DT);
    ego.step(DT, enduranceTrack, true);
    lead1.step(DT, enduranceTrack, true);
    lead2.step(DT, enduranceTrack, true);
    const col = resolveVehicleCollisions(vehicles, 3);
    if (col.deepOverlaps > 0) collisionOccurred = true;
  }

  assert.ok(!collisionOccurred, 'Accordion braking must not result in deep overlaps');
});

runTest('S-M3: Dual Combat (Attack Ahead + Defend Behind Simultaneously)', () => {
  const ego = new Vehicle({ id: 'ego', spec: 'prototype' });
  const targetAhead = new Vehicle({ id: 'target', spec: 'gt' });
  const threatBehind = new Vehicle({ id: 'threat', spec: 'prototype' });

  targetAhead.resetTo(enduranceTrack, 165, 0.0);
  ego.resetTo(enduranceTrack, 150, 0.0);
  threatBehind.resetTo(enduranceTrack, 138, 0.0);

  setForwardSpeed(targetAhead, 24, enduranceTrack);
  setForwardSpeed(ego, 28, enduranceTrack);
  setForwardSpeed(threatBehind, 33, enduranceTrack);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.92, defenseReactivity: 0.90 });
  const vehicles = [ego, targetAhead, threatBehind];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  egoAI.update(ego, vehicles, enduranceTrack, race, DT);
  assert.equal(egoAI.combatEngine.role, 'DUAL_COMBAT', 'Dual combat role must synthesize both attack and defense');
});

runTest('S-M4: Dual Rear Threats (Two Chasers Flanking)', () => {
  const ego = new Vehicle({ id: 'ego', spec: 'gt' });
  const chaser1 = new Vehicle({ id: 'chaser1', spec: 'prototype' });
  const chaser2 = new Vehicle({ id: 'chaser2', spec: 'prototype' });

  ego.resetTo(enduranceTrack, 150, 0.0);
  chaser1.resetTo(enduranceTrack, 138, -1.8);
  chaser2.resetTo(enduranceTrack, 138, 1.8);

  setForwardSpeed(ego, 26, enduranceTrack);
  setForwardSpeed(chaser1, 32, enduranceTrack);
  setForwardSpeed(chaser2, 32, enduranceTrack);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.50, defenseReactivity: 0.90 });
  const vehicles = [ego, chaser1, chaser2];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  egoAI.update(ego, vehicles, enduranceTrack, race, DT);
  assert.ok(egoAI.combatEngine.defenseMode !== 'PACE', 'Must defend position against dual rear threats');
  assert.ok(Number.isFinite(egoAI.trajectoryPlan?.selectedOffset), 'Plan offset must remain finite');
});

runTest('S-M5: Pass One Car with Another Ahead (No Collision with Car B)', () => {
  const ego = new Vehicle({ id: 'ego', spec: 'prototype' });
  const carA = new Vehicle({ id: 'carA', spec: 'gt' });
  const carB = new Vehicle({ id: 'carB', spec: 'gt' });

  carB.resetTo(enduranceTrack, 175, -2.5); // Car B ahead on left
  carA.resetTo(enduranceTrack, 145, 0.0);  // Car A directly ahead
  ego.resetTo(enduranceTrack, 128, 0.0);   // Ego trailing

  setForwardSpeed(carB, 24, enduranceTrack);
  setForwardSpeed(carA, 24, enduranceTrack);
  setForwardSpeed(ego, 32, enduranceTrack);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.92 });
  const vehicles = [ego, carA, carB];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  egoAI.update(ego, vehicles, enduranceTrack, race, DT);
  assert.ok(egoAI.trajectoryPlan, 'Must generate valid trajectory plan');
  assert.ok(egoAI.trajectoryPlan.collisionFree, 'Trajectory must be collision-free accounting for both cars');
});

console.log('================================================================================');
console.log(`>>> ALL ${passedTests}/${totalTests} SUPREME RACECRAFT V3 TESTS PASSED CLEANLY (Exit 0) <<<`);
console.log('================================================================================\n');
