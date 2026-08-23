import assert from 'node:assert/strict';
import { Circuit } from '../src/simulation/Track.js';
import { ENDURANCE_PARK } from '../src/scenarios/EndurancePark.js';
import { Vehicle } from '../src/simulation/Vehicle.js';
import { ResearchAIController } from '../src/ai/ResearchAIController.js';
import { updateAerodynamicWakes, resolveVehicleCollisions } from '../src/simulation/VehicleInteractions.js';
import { SCENARIOS } from '../src/scenarios/ScenarioDefinitions.js';

const DT = 1 / 120;

console.log('=== [3/5] Running Tactical Attack Scenarios Test Suite (A1 & A2) ===');

const setForwardSpeed = (vehicle, speedMs, track) => {
  const p = track.atDistance(vehicle.distance);
  vehicle.speed = speedMs;
  vehicle.velocity = { x: p.tangent.x * speedMs, y: 0, z: p.tangent.z * speedMs };
  vehicle.localVelocity = { x: 0, z: speedMs };
  for (const wheel of vehicle.wheels || []) {
    wheel.omega = speedMs / (vehicle.wheelRadius || 0.335);
  }
};

// ---------------------------------------------------------------------------
// 1. Attack Scenario A1: Straight Slipstream & ERS Slingshot Divebomb
// ---------------------------------------------------------------------------
console.log('  -> Simulating Attack Scenario A1 (Straight Slipstream Slingshot)...');
{
  const track = new Circuit(ENDURANCE_PARK);
  const player = new Vehicle({ id: 'player-a1', spec: 'gt', player: true });
  const ai = new Vehicle({ id: 'ai-a1', spec: 'prototype' });

  player.resetTo(track, 115, 0.4);
  ai.resetTo(track, 90, -0.4); // trailing in slipstream

  setForwardSpeed(player, 25, track);
  setForwardSpeed(ai, 32, track);

  const playerAI = new ResearchAIController(2, { aggression: 0.60 });
  const aiController = new ResearchAIController(1, {
    aggression: 0.92,
    diveMargin: 0.70,
    ersAttackMode: true
  });
  playerAI.debugEnabled = true;
  aiController.debugEnabled = true;

  const vehicles = [player, ai];
  const race = {
    phase: 'racing',
    raceTime: 10.0,
    elapsed: 10.0,
    statusFor: (v) => ({ position: v === player ? 1 : 2 })
  };

  let contactFrames = 0;
  let deepOverlapFrames = 0;
  let draftSeconds = 0;
  let attackSeconds = 0;
  let offTrackSeconds = 0;
  let completedAt = null;
  let minimumSeparation = Infinity;
  const simDurationS = 15.0;
  const totalSteps = Math.round(simDurationS / DT);

  for (let step = 0; step < totalSteps; step += 1) {
    race.raceTime += DT;
    race.elapsed += DT;

    playerAI.update(player, vehicles, track, race, DT);
    aiController.update(ai, vehicles, track, race, DT);
    updateAerodynamicWakes(vehicles);

    player.step(DT, track, true);
    ai.step(DT, track, true);

    const collision = resolveVehicleCollisions(vehicles, 3);
    if (collision.contacts > 0) contactFrames += 1;
    if (collision.deepOverlaps > 0) deepOverlapFrames += 1;

    const sep = Math.hypot(ai.position.x - player.position.x, ai.position.z - player.position.z);
    minimumSeparation = Math.min(minimumSeparation, sep);

    if (ai.surface?.zone === 'grass' || ai.surface?.zone === 'runoff') {
      offTrackSeconds += DT;
    }

    if (ai.wake?.strength > 0.15) {
      draftSeconds += DT;
    }

    const debug = aiController.debugState;
    if (debug?.mode === 'ATTACK' || aiController.attackEngine.attacking) {
      attackSeconds += DT;
    }

    if (completedAt === null && ai.distance > player.distance + 4.0) {
      completedAt = race.raceTime - 10.0;
    }
  }

  const finalGap = ai.distance - player.distance;
  console.log(`    A1 Result: completedAt=${completedAt ? completedAt.toFixed(2) + 's' : 'N/A'}, draftTime=${draftSeconds.toFixed(2)}s, attackTime=${attackSeconds.toFixed(2)}s, minSep=${minimumSeparation.toFixed(2)}m, finalGap=${finalGap.toFixed(1)}m, contacts=${contactFrames}, deepOverlaps=${deepOverlapFrames}`);

  assert.ok(completedAt !== null, 'AI must complete the overtake in Scenario A1');
  assert.ok(completedAt < simDurationS, 'Overtake must complete within 15 seconds');
  assert.ok(draftSeconds > 0.1, 'AI must draft in the player slipstream before pulling out');
  assert.equal(deepOverlapFrames, 0, 'Overtake must never produce deep overlap');
  assert.ok(offTrackSeconds < 0.25, 'AI must remain on legal track surface during pass');
  assert.ok(finalGap > 4.0, 'AI must establish clear race progress after overtake');
  console.log('    [PASS] Scenario A1: Clean slipstream slingshot pass verified.');
}

// ---------------------------------------------------------------------------
// 2. Attack Scenario A2: Quarry Chicane Late-Braking Inside Dive Attack
// ---------------------------------------------------------------------------
console.log('  -> Simulating Attack Scenario A2 (Quarry Chicane Inside Attack)...');
{
  const track = new Circuit(ENDURANCE_PARK);
  const player = new Vehicle({ id: 'player-a2', spec: 'gt', player: true });
  const ai = new Vehicle({ id: 'ai-a2', spec: 'gt' });

  // Player on outside approaching chicane, AI on inside line
  player.resetTo(track, 655, 2.0);
  ai.resetTo(track, 640, -1.8);

  setForwardSpeed(player, 38, track); // Conservative chicane approach
  setForwardSpeed(ai, 48, track);     // High momentum attack entry

  const playerAI = new ResearchAIController(2, { aggression: 0.50, trailBrakingSkill: 0.60 });
  const aiController = new ResearchAIController(1, {
    aggression: 0.95,
    diveMargin: 0.85,
    trailBrakingSkill: 0.95
  });
  playerAI.debugEnabled = true;
  aiController.debugEnabled = true;

  const vehicles = [player, ai];
  const race = {
    phase: 'racing',
    raceTime: 10.0,
    elapsed: 10.0,
    statusFor: (v) => ({ position: v === player ? 1 : 2 })
  };

  let contactFrames = 0;
  let deepOverlapFrames = 0;
  let offTrackSeconds = 0;
  let completedAt = null;
  let minimumSeparation = Infinity;
  const simDurationS = 15.0;
  const totalSteps = Math.round(simDurationS / DT);

  for (let step = 0; step < totalSteps; step += 1) {
    race.raceTime += DT;
    race.elapsed += DT;

    playerAI.update(player, vehicles, track, race, DT);
    aiController.update(ai, vehicles, track, race, DT);
    updateAerodynamicWakes(vehicles);

    player.step(DT, track, true);
    ai.step(DT, track, true);

    const collision = resolveVehicleCollisions(vehicles, 3);
    if (collision.contacts > 0) contactFrames += 1;
    if (collision.deepOverlaps > 0) deepOverlapFrames += 1;

    const sep = Math.hypot(ai.position.x - player.position.x, ai.position.z - player.position.z);
    minimumSeparation = Math.min(minimumSeparation, sep);

    if (ai.surface?.zone === 'grass' || ai.surface?.zone === 'runoff') {
      offTrackSeconds += DT;
    }

    if (completedAt === null && ai.distance > player.distance + 4.0) {
      completedAt = race.raceTime - 10.0;
    }
  }

  console.log(`    A2 Result: completedAt=${completedAt ? completedAt.toFixed(2) + 's' : 'N/A'}, minSep=${minimumSeparation.toFixed(2)}m, contacts=${contactFrames}, deepOverlaps=${deepOverlapFrames}, offTrack=${offTrackSeconds.toFixed(2)}s`);

  assert.ok(completedAt !== null, 'AI must complete the chicane attack pass in Scenario A2');
  assert.ok(completedAt < simDurationS, 'Chicane attack must complete within scenario duration');
  assert.equal(deepOverlapFrames, 0, 'Chicane attack must never produce deep overlap');
  assert.ok(offTrackSeconds < 0.25, 'AI must stay within legal boundaries through chicane');
  console.log('    [PASS] Scenario A2: Inside chicane dive attack verified.');
}

console.log('=== Tactical Attack Scenarios Test Suite: ALL ASSERTIONS PASSED ===\n');
