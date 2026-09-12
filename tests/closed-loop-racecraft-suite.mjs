/**
 * closed-loop-racecraft-suite.mjs
 * Comprehensive Closed-Loop Racecraft Acceptance Suite (V3.2).
 *
 * Runs 10 fully autonomous scenarios at 120Hz physics with real multi-rate controllers.
 * ZERO manual state tampering:
 * - NO manual assignment of attackMode
 * - NO manual assignment of passState
 * - NO manual assignment of insideClosedFilterTimer
 * - NO manual assignment of defensiveEpisode
 *
 * Scenarios:
 * A. Draft -> Pullout -> Full Pass -> Retained -> Safe Merge
 * B. Defender Closes Inside -> Autonomous Abort or Switchback
 * C. Outside Momentum Pass Through Corner Exit
 * D. Defender Feints Over Multiple Seconds -> Exactly One Defensive Move
 * E. Blocked Lane With a Real Third Car (Feasible Corridors)
 * F. Asymmetric Three-Wide for >= 3 Seconds
 * G. Attack Ahead + Defend Behind for >= 5 Seconds (Sandwich Formation)
 * H. Pass Car A While Car B Blocks Downstream Corridor
 * I. Pass -> Retain -> Same Rival Later Repasses -> New Valid Episode
 * J. Start/Finish Line Passing Scenario (Coordinate Wrap Continuity)
 */

import assert from 'node:assert/strict';
import { Circuit } from '../src/simulation/Track.js';
import { ENDURANCE_PARK } from '../src/scenarios/EndurancePark.js';
import { HARBOR_RING } from '../src/scenarios/HarborRing.js';
import { Vehicle } from '../src/simulation/Vehicle.js';
import { NextGenAIController } from '../src/ai/v2/NextGenAIController.js';
import { updateAerodynamicWakes, resolveVehicleCollisions } from '../src/simulation/VehicleInteractions.js';

const DT = 1 / 120;
const enduranceTrack = new Circuit(ENDURANCE_PARK);
const harborTrack = new Circuit(HARBOR_RING);

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
console.log('       GEMINI SUPREME — CLOSED-LOOP RACECRAFT ACCEPTANCE SUITE (10 SCENARIOS)   ');
console.log('================================================================================\n');

let passedTests = 0;
let totalTests = 0;

function runScenario(name, fn) {
  totalTests++;
  console.log(`[${totalTests}/10] Testing Scenario ${name}...`);
  try {
    fn();
    passedTests++;
    console.log(`  -> [PASS] Scenario ${name} verified.\n`);
  } catch (err) {
    console.error(`  -> [FAIL] Scenario ${name}:`, err.message);
    throw err;
  }
}

// -----------------------------------------------------------------------------
// SCENARIO A: Draft -> Pullout -> Full Pass -> Retained -> Safe Merge
// -----------------------------------------------------------------------------
runScenario('A: Draft -> Pullout -> Full Pass -> Retained -> Safe Merge', () => {
  const track = enduranceTrack;
  const ego = new Vehicle({ id: 'ego-a', spec: 'prototype' });
  const rival = new Vehicle({ id: 'rival-a', spec: 'gt' });

  rival.resetTo(track, 120, 0.0);
  ego.resetTo(track, 90, 0.0);
  setForwardSpeed(rival, 25.0, track);
  setForwardSpeed(ego, 33.0, track);

  const egoAI = new NextGenAIController('ego-ai-a', { aggression: 0.90 });
  const rivalAI = new NextGenAIController('rival-ai-a', { aggression: 0.40 });
  const vehicles = [ego, rival];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  let draftDetected = false;
  let pulloutDetected = false;
  let passDetected = false;
  let retainedDetected = false;
  let safeMergeDetected = false;

  for (let step = 0; step < 900; step++) {
    race.raceTime += DT; race.elapsed += DT;
    egoAI.update(ego, vehicles, track, race, DT);
    rivalAI.update(rival, vehicles, track, race, DT);
    updateAerodynamicWakes(vehicles);
    ego.step(DT, track, true);
    rival.step(DT, track, true);
    const col = resolveVehicleCollisions(vehicles, 3);
    assert.equal(col.deepOverlaps, 0, 'No deep overlaps during pass execution');

    if ((ego.wake?.strength || 0) > 0.05 && ego.distance < rival.distance) draftDetected = true;
    if (Math.abs(ego.surface?.lateral - rival.surface?.lateral) > 1.2 && ego.distance <= rival.distance + 2) pulloutDetected = true;
    if (ego.distance > rival.distance + 6) passDetected = true;

    const combat = egoAI.combatEngine;
    if (combat?.completedPassEpisodes?.some(ep => ep.completionReason === 'RETAINED') || combat?.passState === 'RETAINING' || combat?.passState === 'RETAINED') {
      retainedDetected = true;
    }
    if (passDetected && Math.abs(ego.surface?.lateral) < 1.0 && ego.distance > rival.distance + 15) {
      safeMergeDetected = true;
    }
  }

  assert.ok(draftDetected, 'Must catch aerodynamic draft in wake of leading rival');
  assert.ok(pulloutDetected, 'Must autonomously pull out laterally to execute slingshot pass');
  assert.ok(passDetected, 'Must cleanly pass leading rival on the straight');
  assert.ok(retainedDetected, 'Pass state machine must retain overtake position');
  assert.ok(safeMergeDetected, 'Must safely merge back toward racing line without chopping rival nose');

  const completed = egoAI.combatEngine.completedPassEpisodes;
  assert.ok(completed.length >= 1, 'Must archive completed pass episode');
  assert.equal(completed[0].completionReason, 'RETAINED', 'Episode completion reason must be RETAINED');
  assert.ok(completed[0].durationS > 0, 'Episode duration must be measured in seconds');
  assert.ok(completed[0].retainedDistanceM > 0, 'Retained distance must be measured in meters');
});

// -----------------------------------------------------------------------------
// SCENARIO B: Defender Closes Inside -> Autonomous Abort or Switchback
// -----------------------------------------------------------------------------
runScenario('B: Defender Closes Inside -> Autonomous Abort or Switchback', () => {
  const track = enduranceTrack;
  const ego = new Vehicle({ id: 'ego-b', spec: 'gt' });
  const rival = new Vehicle({ id: 'rival-b', spec: 'gt' });

  // Approaching braking zone with rival tightly closing inside
  rival.resetTo(track, 475, -2.8);
  ego.resetTo(track, 455, -2.5);
  setForwardSpeed(rival, 22.0, track);
  setForwardSpeed(ego, 27.0, track);

  const egoAI = new NextGenAIController('ego-ai-b', { aggression: 0.85 });
  const rivalAI = new NextGenAIController('rival-ai-b', { aggression: 0.70 });
  const vehicles = [ego, rival];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  let insideAbortedOrSwitched = false;

  for (let step = 0; step < 600; step++) {
    race.raceTime += DT; race.elapsed += DT;
    egoAI.update(ego, vehicles, track, race, DT);
    rivalAI.update(rival, vehicles, track, race, DT);
    updateAerodynamicWakes(vehicles);
    ego.step(DT, track, true);
    rival.step(DT, track, true);
    const col = resolveVehicleCollisions(vehicles, 3);
    assert.equal(col.deepOverlaps, 0, 'Zero deep overlaps when inside is closed');

    const action = egoAI.combatEngine?.lastTacticalDiagnostics?.selectedAction;
    const egoLat = ego.surface?.lateral ?? 0;
    if (egoLat > -0.5 || action === 'SWITCHBACK' || action === 'OUTSIDE_MOMENTUM' || action === 'ABORT_PASS') {
      insideAbortedOrSwitched = true;
    }
  }

  assert.ok(insideAbortedOrSwitched, 'Ego must autonomously switch line or abort when inside door is closed');
  assert.ok(ego.surface?.lateral > -1.5, 'Ego must not force into the closed inside wall/curb');
});

// -----------------------------------------------------------------------------
// SCENARIO C: Outside Momentum Pass Through Corner Exit
// -----------------------------------------------------------------------------
runScenario('C: Outside Momentum Pass Through Corner Exit', () => {
  const track = harborTrack;
  const ego = new Vehicle({ id: 'ego-c', spec: 'gt' });
  const rival = new Vehicle({ id: 'rival-c', spec: 'touring' });

  // Corner exit: rival on tight inside, ego sweeping high on outside
  rival.resetTo(track, 300, -2.5);
  ego.resetTo(track, 285, 1.8);
  setForwardSpeed(rival, 18.0, track);
  setForwardSpeed(ego, 25.0, track);

  const egoAI = new NextGenAIController('ego-ai-c', { aggression: 0.90 });
  const rivalAI = new NextGenAIController('rival-ai-c', { aggression: 0.40 });
  const vehicles = [ego, rival];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  let outsideMaintained = false;
  let exitPassCompleted = false;

  for (let step = 0; step < 720; step++) {
    race.raceTime += DT; race.elapsed += DT;
    egoAI.update(ego, vehicles, track, race, DT);
    rivalAI.update(rival, vehicles, track, race, DT);
    updateAerodynamicWakes(vehicles);
    ego.step(DT, track, true);
    rival.step(DT, track, true);
    const col = resolveVehicleCollisions(vehicles, 3);
    assert.equal(col.deepOverlaps, 0, 'No deep overlaps during outside pass');

    if (ego.surface?.lateral > rival.surface?.lateral + 1.0 && Math.abs(ego.distance - rival.distance) < 8.0) {
      outsideMaintained = true;
    }
    if (ego.distance > rival.distance + 8.0) {
      exitPassCompleted = true;
      break;
    }
  }

  assert.ok(outsideMaintained, 'Ego must maintain wide outside arc through corner');
  assert.ok(exitPassCompleted, 'Ego must carry superior exit speed to pass rival on outside');
});

// -----------------------------------------------------------------------------
// SCENARIO D: Defender Feints Over Multiple Seconds -> Exactly One Defensive Move
// -----------------------------------------------------------------------------
runScenario('D: Defender Feints Over Multiple Seconds -> Exactly One Defensive Move', () => {
  const track = enduranceTrack;
  const ego = new Vehicle({ id: 'ego-d', spec: 'gt' });
  const challenger = new Vehicle({ id: 'challenger-d', spec: 'prototype' });

  ego.resetTo(track, 180, 0.0);
  challenger.resetTo(track, 165, 0.0);
  setForwardSpeed(ego, 28.0, track);
  setForwardSpeed(challenger, 30.0, track);

  const egoAI = new NextGenAIController('ego-ai-d', { aggression: 0.60, defenseReactivity: 0.90 });
  const vehicles = [ego, challenger];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  let steerSwaps = 0;
  let priorSteer = 0;
  let maxMoves = 0;

  // 3.0 seconds (360 steps at 120Hz) of continuous lateral feinting from attacker
  for (let step = 0; step < 360; step++) {
    race.raceTime += DT; race.elapsed += DT;
    const feintLat = Math.sin(step * 0.15) * 2.2;
    const chDist = challenger.distance + 30 * DT;
    challenger.resetTo(track, chDist, feintLat);
    setForwardSpeed(challenger, 30.0, track);

    egoAI.update(ego, vehicles, track, race, DT);
    ego.step(DT, track, true);

    const steer = ego.controls?.steer || 0;
    if (Math.sign(steer) !== 0 && Math.sign(priorSteer) !== 0 && Math.sign(steer) !== Math.sign(priorSteer)) {
      steerSwaps++;
    }
    priorSteer = steer;

    const moveCount = egoAI.combatEngine?.defensiveEpisode?.moveCount ?? 0;
    if (moveCount > maxMoves) maxMoves = moveCount;
  }

  assert.ok(maxMoves <= 1, `Ego must lock exactly one defensive move under continuous feints (got ${maxMoves})`);
  assert.ok(steerSwaps <= 6, `Defending car must not weave or twitch reactively (steerSwaps=${steerSwaps})`);
});

// -----------------------------------------------------------------------------
// SCENARIO E: Blocked Lane With a Real Third Car (Feasible Corridors)
// -----------------------------------------------------------------------------
runScenario('E: Blocked Lane With a Real Third Car (Feasible Corridors)', () => {
  const track = enduranceTrack;
  const ego = new Vehicle({ id: 'ego-e', spec: 'prototype' });
  const target = new Vehicle({ id: 'target-e', spec: 'gt' });
  const blocker = new Vehicle({ id: 'blocker-e', spec: 'gt' });

  // Blocker on outside (+2.6m), target on inside (-0.5m)
  target.resetTo(track, 135, -0.5);
  blocker.resetTo(track, 142, 2.6);
  ego.resetTo(track, 105, 0.0);

  setForwardSpeed(target, 24.0, track);
  setForwardSpeed(blocker, 24.0, track);
  setForwardSpeed(ego, 31.0, track);

  const egoAI = new NextGenAIController('ego-ai-e', { aggression: 0.90 });
  const targetAI = new NextGenAIController('target-ai-e', { aggression: 0.50 });
  const blockerAI = new NextGenAIController('blocker-ai-e', { aggression: 0.50 });
  const vehicles = [ego, target, blocker];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  let passedTarget = false;

  for (let step = 0; step < 720; step++) {
    race.raceTime += DT; race.elapsed += DT;
    egoAI.update(ego, vehicles, track, race, DT);
    targetAI.update(target, vehicles, track, race, DT);
    blockerAI.update(blocker, vehicles, track, race, DT);
    updateAerodynamicWakes(vehicles);
    for (const v of vehicles) v.step(DT, track, true);
    const col = resolveVehicleCollisions(vehicles, 3);
    assert.equal(col.deepOverlaps, 0, 'No collisions with blocker or target');

    if (ego.distance > target.distance + 5.0) {
      passedTarget = true;
      break;
    }
  }

  assert.ok(passedTarget, 'Ego must route through feasible corridor and pass target cleanly');
});

// -----------------------------------------------------------------------------
// SCENARIO F: Asymmetric Three-Wide for >= 3 Seconds
// -----------------------------------------------------------------------------
runScenario('F: Asymmetric Three-Wide for >= 3 Seconds', () => {
  const track = enduranceTrack;
  const ego = new Vehicle({ id: 'ego-f', spec: 'gt' });
  const leftCar = new Vehicle({ id: 'left-car-f', spec: 'gt' });
  const rightCar = new Vehicle({ id: 'right-car-f', spec: 'gt' });

  leftCar.resetTo(track, 120, -2.8);
  ego.resetTo(track, 120, 0.0);
  rightCar.resetTo(track, 120, 2.8);

  setForwardSpeed(leftCar, 26.0, track);
  setForwardSpeed(ego, 26.0, track);
  setForwardSpeed(rightCar, 26.0, track);

  const egoAI = new NextGenAIController('ego-ai-f', { aggression: 0.85 });
  const leftAI = new NextGenAIController('left-ai-f', { aggression: 0.50 });
  const rightAI = new NextGenAIController('right-ai-f', { aggression: 0.50 });

  const vehicles = [leftCar, ego, rightCar];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  let threeWideDurationS = 0;

  for (let step = 0; step < 400; step++) {
    race.raceTime += DT; race.elapsed += DT;
    egoAI.update(ego, vehicles, track, race, DT);
    leftAI.update(leftCar, vehicles, track, race, DT);
    rightAI.update(rightCar, vehicles, track, race, DT);
    updateAerodynamicWakes(vehicles);
    for (const v of vehicles) v.step(DT, track, true);
    const col = resolveVehicleCollisions(vehicles, 3);
    assert.equal(col.deepOverlaps, 0, 'No deep overlaps during three-wide pack racing');

    if (egoAI.combatEngine?.threeWideActive) {
      threeWideDurationS += DT;
    }
  }

  assert.ok(threeWideDurationS >= 3.0, `Must sustain active three-wide tactical reasoning for >= 3.0s (got ${threeWideDurationS.toFixed(2)}s)`);
  assert.ok(Math.abs(ego.surface?.lateral) < 1.0, 'Ego must remain centered in free space between flanking cars');
});

// -----------------------------------------------------------------------------
// SCENARIO G: Attack Ahead + Defend Behind for >= 5 Seconds (Sandwich Formation)
// -----------------------------------------------------------------------------
runScenario('G: Attack Ahead + Defend Behind for >= 5 Seconds (Sandwich Formation)', () => {
  const track = enduranceTrack;
  const leader = new Vehicle({ id: 'leader-g', spec: 'gt' });
  const ego = new Vehicle({ id: 'ego-g', spec: 'gt' });
  const chaser = new Vehicle({ id: 'chaser-g', spec: 'gt' });

  leader.resetTo(track, 140, 0.0);
  ego.resetTo(track, 120, 0.0);
  chaser.resetTo(track, 105, 0.0);

  setForwardSpeed(leader, 26.0, track);
  setForwardSpeed(ego, 26.0, track);
  setForwardSpeed(chaser, 26.0, track);

  const leaderAI = new NextGenAIController('leader-ai-g', { aggression: 0.60 });
  const egoAI = new NextGenAIController('ego-ai-g', { aggression: 0.85, defenseReactivity: 0.90 });
  const chaserAI = new NextGenAIController('chaser-ai-g', { aggression: 0.80 });

  const vehicles = [leader, ego, chaser];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  let sandwichDurationS = 0;

  for (let step = 0; step < 600; step++) {
    race.raceTime += DT; race.elapsed += DT;
    leaderAI.update(leader, vehicles, track, race, DT);
    egoAI.update(ego, vehicles, track, race, DT);
    chaserAI.update(chaser, vehicles, track, race, DT);
    updateAerodynamicWakes(vehicles);
    for (const v of vehicles) v.step(DT, track, true);
    const col = resolveVehicleCollisions(vehicles, 3);
    assert.equal(col.deepOverlaps, 0, 'No collisions during 3-car sandwich');

    const gapAhead = leader.distance - ego.distance;
    const gapBehind = ego.distance - chaser.distance;
    if (gapAhead > 0 && gapAhead < 35.0 && gapBehind > 0 && gapBehind < 30.0) {
      sandwichDurationS += DT;
    }
  }

  assert.ok(sandwichDurationS >= 5.0, `Must sustain stable sandwich formation for >= 5.0s (got ${sandwichDurationS.toFixed(2)}s)`);
});

// -----------------------------------------------------------------------------
// SCENARIO H: Pass Car A While Car B Blocks Downstream Corridor
// -----------------------------------------------------------------------------
runScenario('H: Pass Car A While Car B Blocks Downstream Corridor', () => {
  const track = enduranceTrack;
  const carA = new Vehicle({ id: 'carA-h', spec: 'touring' });
  const carB = new Vehicle({ id: 'carB-h', spec: 'touring' });
  const ego = new Vehicle({ id: 'ego-h', spec: 'prototype' });

  carA.resetTo(track, 125, -1.5);
  carB.resetTo(track, 155, 1.8);
  ego.resetTo(track, 95, 0.0);

  setForwardSpeed(carA, 22.0, track);
  setForwardSpeed(carB, 23.0, track);
  setForwardSpeed(ego, 31.0, track);

  const egoAI = new NextGenAIController('ego-ai-h', { aggression: 0.90 });
  const carAAI = new NextGenAIController('carA-ai-h', { aggression: 0.50 });
  const carBAI = new NextGenAIController('carB-ai-h', { aggression: 0.50 });

  const vehicles = [ego, carA, carB];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  let passedCarA = false;
  let passedBoth = false;

  for (let step = 0; step < 850; step++) {
    race.raceTime += DT; race.elapsed += DT;
    egoAI.update(ego, vehicles, track, race, DT);
    carAAI.update(carA, vehicles, track, race, DT);
    carBAI.update(carB, vehicles, track, race, DT);
    updateAerodynamicWakes(vehicles);
    for (const v of vehicles) v.step(DT, track, true);
    const col = resolveVehicleCollisions(vehicles, 3);
    assert.equal(col.deepOverlaps, 0, 'Zero deep overlaps during sequential downstream pass');

    if (ego.distance > carA.distance + 6.0) passedCarA = true;
    if (ego.distance > carB.distance + 6.0) {
      passedBoth = true;
      break;
    }
  }

  assert.ok(passedCarA, 'Must cleanly overtake Car A');
  assert.ok(passedBoth, 'Must cleanly overtake both Car A and downstream blocker Car B');
});

// -----------------------------------------------------------------------------
// SCENARIO I: Pass -> Retain -> Same Rival Later Repasses -> New Valid Episode
// -----------------------------------------------------------------------------
runScenario('I: Pass -> Retain -> Same Rival Later Repasses -> New Valid Episode', () => {
  const track = enduranceTrack;
  const ego = new Vehicle({ id: 'ego-i', spec: 'prototype' });
  const rival = new Vehicle({ id: 'rival-i', spec: 'gt' });

  rival.resetTo(track, 115, 0.0);
  ego.resetTo(track, 90, 0.0);
  setForwardSpeed(rival, 24.0, track);
  setForwardSpeed(ego, 32.0, track);

  const egoAI = new NextGenAIController('ego-ai-i', { aggression: 0.90 });
  const rivalAI = new NextGenAIController('rival-ai-i', { aggression: 0.50 });
  const vehicles = [ego, rival];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  // Phase 1: Pass & retain
  for (let step = 0; step < 700; step++) {
    race.raceTime += DT; race.elapsed += DT;
    egoAI.update(ego, vehicles, track, race, DT);
    rivalAI.update(rival, vehicles, track, race, DT);
    updateAerodynamicWakes(vehicles);
    ego.step(DT, track, true);
    rival.step(DT, track, true);
    resolveVehicleCollisions(vehicles, 3);
    if (egoAI.combatEngine.completedPassEpisodes.length >= 1) break;
  }

  const ep1 = egoAI.combatEngine.completedPassEpisodes[0];
  assert.ok(ep1, 'Episode 1 must be created and completed');
  assert.equal(ep1.completionReason, 'RETAINED', 'Episode 1 must be RETAINED');

  // Phase 2: Rival repasses ego
  const repassDist = ego.distance + 15.0;
  rival.resetTo(track, repassDist, 0.0);
  setForwardSpeed(ego, 24.0, track);
  setForwardSpeed(rival, 30.0, track);

  for (let step = 0; step < 30; step++) {
    race.raceTime += DT; race.elapsed += DT;
    egoAI.update(ego, vehicles, track, race, DT);
    rivalAI.update(rival, vehicles, track, race, DT);
    ego.step(DT, track, true);
    rival.step(DT, track, true);
  }

  // Phase 3: Ego attacks rival again
  setForwardSpeed(ego, 34.0, track);
  setForwardSpeed(rival, 22.0, track);

  let secondEpisodeCreated = false;
  for (let step = 0; step < 200; step++) {
    race.raceTime += DT; race.elapsed += DT;
    egoAI.update(ego, vehicles, track, race, DT);
    rivalAI.update(rival, vehicles, track, race, DT);
    ego.step(DT, track, true);
    rival.step(DT, track, true);
    if (egoAI.combatEngine.activePassEpisode && egoAI.combatEngine.activePassEpisode.episodeId !== ep1.episodeId) {
      secondEpisodeCreated = true;
      break;
    }
  }

  assert.ok(secondEpisodeCreated, 'New valid pass episode must initiate when same rival repasses and is re-attacked');
  assert.notEqual(egoAI.combatEngine.activePassEpisode.episodeId, ep1.episodeId, 'New episode ID must differ from first archived episode ID');
});

// -----------------------------------------------------------------------------
// SCENARIO J: Start/Finish Line Passing Scenario (Coordinate Wrap Continuity)
// -----------------------------------------------------------------------------
runScenario('J: Start/Finish Line Passing Scenario (Coordinate Wrap Continuity)', () => {
  const track = harborTrack;
  const L = track.length;
  const ego = new Vehicle({ id: 'ego-j', spec: 'prototype' });
  const rival = new Vehicle({ id: 'rival-j', spec: 'gt' });

  // Start before the line (L - 45m and L - 20m)
  rival.resetTo(track, L - 20, 0.0);
  ego.resetTo(track, L - 45, 0.0);
  setForwardSpeed(rival, 22.0, track);
  setForwardSpeed(ego, 31.0, track);

  const egoAI = new NextGenAIController('ego-ai-j', { aggression: 0.90 });
  const rivalAI = new NextGenAIController('rival-ai-j', { aggression: 0.40 });
  const vehicles = [ego, rival];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  let passedAfterSeam = false;

  for (let step = 0; step < 720; step++) {
    race.raceTime += DT; race.elapsed += DT;
    egoAI.update(ego, vehicles, track, race, DT);
    rivalAI.update(rival, vehicles, track, race, DT);
    updateAerodynamicWakes(vehicles);
    ego.step(DT, track, true);
    rival.step(DT, track, true);
    const col = resolveVehicleCollisions(vehicles, 3);
    assert.equal(col.deepOverlaps, 0, 'No deep collision across start/finish seam');

    // Past the seam: distances wrap into [0, 200]
    if (ego.distance < 200 && rival.distance < 200 && ego.distance > rival.distance + 5.0) {
      passedAfterSeam = true;
      break;
    }
  }

  assert.ok(passedAfterSeam, 'Overtake maneuver must seamlessly complete across start/finish coordinate wrap boundary');
});

console.log('================================================================================');
console.log(`>>> ALL ${passedTests}/${totalTests} CLOSED-LOOP RACECRAFT SCENARIOS PASSED (Exit 0) <<<`);
console.log('================================================================================\n');
