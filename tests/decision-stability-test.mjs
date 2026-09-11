import assert from 'node:assert/strict';
import { Circuit } from '../src/simulation/Track.js';
import { ENDURANCE_PARK } from '../src/scenarios/EndurancePark.js';
import { Vehicle } from '../src/simulation/Vehicle.js';
import { NextGenAIController } from '../src/ai/v2/NextGenAIController.js';
import { FrenetLatticePlanner } from '../src/ai/FrenetLatticePlanner.js';
import { updateAerodynamicWakes, resolveVehicleCollisions } from '../src/simulation/VehicleInteractions.js';

const DT = 1 / 120;

console.log('=== Running Decision Stability & Anti-Indecision Test Suite ===');

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
// 1. Multirate & Dwell Verification in Closing Flank Scenario
// ---------------------------------------------------------------------------
console.log('  -> Scenario 1: Corner approach with closing flank (Anti-Twitch & Hysteresis)...');
{
  const track = new Circuit(ENDURANCE_PARK);
  const defender = new Vehicle({ id: 'defender', spec: 'gt' });
  const attacker = new Vehicle({ id: 'attacker', spec: 'gt' });

  defender.resetTo(track, 150, 0.5);
  attacker.resetTo(track, 136, 0.0);

  setForwardSpeed(defender, 26, track);
  setForwardSpeed(attacker, 32, track);

  const defenderAI = new NextGenAIController('defender-ai', { aggression: 0.50, defenseReactivity: 0.90 });
  const attackerAI = new NextGenAIController('attacker-ai', { aggression: 0.95, diveMargin: 0.85 });

  const vehicles = [defender, attacker];
  const race = {
    phase: 'racing',
    raceTime: 10.0,
    elapsed: 10.0,
    statusFor: (v) => ({ position: v === defender ? 1 : 2 })
  };

  let contactFrames = 0;
  let deepOverlapFrames = 0;
  let maxSteeringReversals = 0;
  let maxLateralLoadTransferRate = 0;
  let phaseTransitions = 0;
  let priorPhase = 'PACE';
  let flankFlips = 0;
  let priorTargetSign = 0;
  let tacticalCalls = 0;
  let trajectoryReplanCalls = 0;

  const simDurationS = 2.5;
  const totalSteps = Math.round(simDurationS / DT);

  for (let step = 0; step < totalSteps; step += 1) {
    race.raceTime += DT;
    race.elapsed += DT;

    // Defender moves dynamically to squeeze inside corridor
    if (step > 40 && step < 100) {
      defender.position.x += 0.015;
    }

    const prevAttackerTacticsTime = attackerAI.lastTacticalEvalTime;
    const prevAttackerReplanCount = attackerAI.trajectoryPlan?.generatedAt || 0;

    defenderAI.update(defender, vehicles, track, race, DT);
    attackerAI.update(attacker, vehicles, track, race, DT);
    updateAerodynamicWakes(vehicles);

    if (attackerAI.lastTacticalEvalTime !== prevAttackerTacticsTime) {
      tacticalCalls += 1;
    }
    if ((attackerAI.trajectoryPlan?.generatedAt || 0) !== prevAttackerReplanCount) {
      trajectoryReplanCalls += 1;
    }

    defender.step(DT, track, true);
    attacker.step(DT, track, true);

    const collision = resolveVehicleCollisions(vehicles, 3);
    if (collision.contacts > 0) contactFrames += 1;
    if (collision.deepOverlaps > 0) deepOverlapFrames += 1;

    const telem = attackerAI.debugState?.telemetry || {};
    if (telem.steeringReversalsLastSecond > maxSteeringReversals) {
      maxSteeringReversals = telem.steeringReversalsLastSecond;
    }
    if (telem.lateralLoadTransferRate > maxLateralLoadTransferRate) {
      maxLateralLoadTransferRate = telem.lateralLoadTransferRate;
    }

    const currentPhase = telem.tacticalPhase || attackerAI.combatEngine?.tacticalPhase || 'PACE';
    if (currentPhase !== priorPhase) {
      phaseTransitions += 1;
      priorPhase = currentPhase;
    }

    const targetLat = attackerAI.trajectoryPlan?.targetOffset ?? 0;
    const targetSign = Math.sign(targetLat);
    if (targetSign !== 0 && priorTargetSign !== 0 && targetSign !== priorTargetSign) {
      flankFlips += 1;
    }
    if (targetSign !== 0) priorTargetSign = targetSign;
  }

  const tacticalRateHz = tacticalCalls / simDurationS;
  const replanRateHz = trajectoryReplanCalls / simDurationS;

  console.log(`    Metrics: phaseTransitions=${phaseTransitions}, flankFlips=${flankFlips}, maxSteerRev=${maxSteeringReversals}/s, maxLoadTransferRate=${maxLateralLoadTransferRate.toFixed(2)}/s, tacticalRate=${tacticalRateHz.toFixed(1)}Hz, replanRate=${replanRateHz.toFixed(1)}Hz, contacts=${contactFrames}, deepOverlaps=${deepOverlapFrames}`);

  assert.equal(deepOverlapFrames, 0, 'Must never produce deep overlap');
  assert.ok(flankFlips <= 1, `Target flank must not rapidly oscillate (got ${flankFlips} flips)`);
  assert.ok(maxSteeringReversals <= 1, `Steering reversals must remain <= 1/s to prevent twitching (got ${maxSteeringReversals})`);
  assert.ok(phaseTransitions <= 3, `Tactical phase must not flap back and forth (got ${phaseTransitions} transitions)`);
  assert.ok(tacticalRateHz <= 15, `Tactical evaluation must be rate-limited (~10 Hz, got ${tacticalRateHz.toFixed(1)} Hz)`);
  assert.ok(replanRateHz <= 35, `Trajectory replanning must be rate-limited (~25 Hz, got ${replanRateHz.toFixed(1)} Hz)`);
  console.log('    [PASS] Scenario 1: Indecision mitigation and smooth rate decoupling verified.');
}

// ---------------------------------------------------------------------------
// 2. Trajectory Switching Cost & Solution Hysteresis Unit Evaluation
// ---------------------------------------------------------------------------
console.log('  -> Scenario 2: Trajectory Lattice Switching Cost & Solution Hysteresis...');
{
  const planner = new FrenetLatticePlanner();
  const context = {
    carWidth: 1.9,
    carLength: 4.6,
    speed: 30,
    racingLineLat: 0.0,
    dMin: -2.5,
    dMax: 2.5,
    attackSide: 'inside',
    overtakeStage: 'ATTACK',
    tacticalPhase: 'DIVEBOMB'
  };

  const opponent = {
    distance: 40,
    speed: 28,
    lateralOffset: 1.0,
    halfWidth: 1.0,
    length: 4.6
  };

  const plan1 = planner.plan({
    s0: 0,
    q0: 0.0,
    v0: 30,
    omega0: 0.0,
    targetSpeed: 30,
    opponents: [opponent],
    desiredOffset: -1.2,
    tacticalMode: 'ATTACK',
    tacticalPhase: 'DIVEBOMB',
    context
  });

  assert.ok(plan1, 'Planner must produce a valid plan');
  const chosenOffset1 = plan1.targetOffset;

  const noisyOpponent = { ...opponent, lateralOffset: 1.1 };
  const plan2 = planner.plan({
    s0: 0.5,
    q0: chosenOffset1 * 0.1,
    v0: 30,
    omega0: 0.0,
    targetSpeed: 30,
    opponents: [noisyOpponent],
    desiredOffset: -1.2,
    tacticalMode: 'ATTACK',
    tacticalPhase: 'DIVEBOMB',
    previousPlan: plan1,
    dtSinceLastPlan: 0.04,
    context
  });

  assert.ok(plan2, 'Planner must produce follow-up plan');
  assert.equal(plan2.targetOffset, chosenOffset1, 'Solution hysteresis must retain consistent offset despite minor noise');
  console.log('    [PASS] Scenario 2: Trajectory lattice hysteresis and switching cost verified.');
}

console.log('=== ALL DECISION STABILITY & ANTI-INDECISION TESTS PASSED ===\n');
