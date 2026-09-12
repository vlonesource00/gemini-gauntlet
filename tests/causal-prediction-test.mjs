/**
 * causal-prediction-test.mjs
 * Dedicated test suite verifying Causal Expected Utility Tactical Action Selection.
 *
 * Proves that varying opponent prediction probabilities P(r_i) causally alters
 * the Stackelberg best-response tactical action (e.g., SWITCHBACK vs DIVEBOMB)
 * with identical physical vehicle states.
 */

import assert from 'node:assert/strict';
import { Circuit } from '../src/simulation/Track.js';
import { ENDURANCE_PARK } from '../src/scenarios/EndurancePark.js';
import { Vehicle } from '../src/simulation/Vehicle.js';
import { NextGenAIController } from '../src/ai/v2/NextGenAIController.js';

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
console.log('       GEMINI SUPREME — CAUSAL PREDICTION EXPECTED UTILITY TEST SUITE           ');
console.log('================================================================================\n');

const track = new Circuit(ENDURANCE_PARK);

// ---------------------------------------------------------------------------
// Test 1: High P(DEFEND_INSIDE) causally triggers SWITCHBACK undercut
// ---------------------------------------------------------------------------
console.log('[1/3] Testing: High P(DEFEND_INSIDE) -> SWITCHBACK Undercut Selection...');
{
  const ego = new Vehicle({ id: 'ego', spec: 'prototype' });
  const rival = new Vehicle({ id: 'rival', spec: 'gt' });

  // Approaching Turn 1 (inside is positive lateral on Endurance Park Turn 1)
  rival.resetTo(track, 765, 2.5);
  ego.resetTo(track, 745, 0.0);
  setForwardSpeed(rival, 24, track);
  setForwardSpeed(ego, 31, track);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.90 });
  const vehicles = [ego, rival];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  // Intercept awareness to inject high P(DEFEND_INSIDE)
  const origScan = egoAI.awareness.scan.bind(egoAI.awareness);
  egoAI.awareness.scan = (veh, allVehs, trk) => {
    const traffic = origScan(veh, allVehs, trk);
    if (traffic.primaryAttackTarget) {
      traffic.primaryAttackTarget.predictions = [
        { id: 'DEFEND_INSIDE', probability: 0.80, envelopes: [] },
        { id: 'HOLD_LINE', probability: 0.15, envelopes: [] },
        { id: 'DEFEND_OUTSIDE', probability: 0.05, envelopes: [] }
      ];
    }
    return traffic;
  };

  egoAI.update(ego, vehicles, track, race, DT);

  console.log(`    Selected Mode: ${egoAI.combatEngine.attackMode} | Attack Side: ${egoAI.combatEngine.attackSide}`);
  assert.equal(
    egoAI.combatEngine.attackMode,
    'SWITCHBACK',
    `High P(DEFEND_INSIDE) must causally induce SWITCHBACK action (got ${egoAI.combatEngine.attackMode})`
  );
  console.log('    [PASS] Causal response to DEFEND_INSIDE verified.\n');
}

// ---------------------------------------------------------------------------
// Test 2: High P(DEFEND_OUTSIDE) causally triggers DIVEBOMB inside apex dive
// ---------------------------------------------------------------------------
console.log('[2/3] Testing: High P(DEFEND_OUTSIDE) -> DIVEBOMB Inside Attack Selection...');
{
  const ego = new Vehicle({ id: 'ego', spec: 'prototype' });
  const rival = new Vehicle({ id: 'rival', spec: 'gt' });

  // Same identical physical placement as Test 1
  rival.resetTo(track, 765, 2.5);
  ego.resetTo(track, 745, 0.0);
  setForwardSpeed(rival, 24, track);
  setForwardSpeed(ego, 31, track);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.90 });
  const vehicles = [ego, rival];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  // Intercept awareness with identical physical state but inverted prediction distribution
  const origScan = egoAI.awareness.scan.bind(egoAI.awareness);
  egoAI.awareness.scan = (veh, allVehs, trk) => {
    const traffic = origScan(veh, allVehs, trk);
    if (traffic.primaryAttackTarget) {
      traffic.primaryAttackTarget.predictions = [
        { id: 'DEFEND_OUTSIDE', probability: 0.80, envelopes: [] },
        { id: 'HOLD_LINE', probability: 0.15, envelopes: [] },
        { id: 'DEFEND_INSIDE', probability: 0.05, envelopes: [] }
      ];
    }
    return traffic;
  };

  egoAI.update(ego, vehicles, track, race, DT);

  console.log(`    Selected Mode: ${egoAI.combatEngine.attackMode} | Attack Side: ${egoAI.combatEngine.attackSide}`);
  assert.equal(
    egoAI.combatEngine.attackMode,
    'DIVEBOMB',
    `High P(DEFEND_OUTSIDE) must causally induce DIVEBOMB action (got ${egoAI.combatEngine.attackMode})`
  );
  console.log('    [PASS] Causal response to DEFEND_OUTSIDE verified.\n');
}

// ---------------------------------------------------------------------------
// Test 3: High P(LATE_BRAKE_OVERSHOOT) causally triggers SWITCHBACK undercut
// ---------------------------------------------------------------------------
console.log('[3/3] Testing: High P(LATE_BRAKE_OVERSHOOT) -> SWITCHBACK Undercut Selection...');
{
  const ego = new Vehicle({ id: 'ego', spec: 'prototype' });
  const rival = new Vehicle({ id: 'rival', spec: 'gt' });

  rival.resetTo(track, 765, 2.5);
  ego.resetTo(track, 745, 0.0);
  setForwardSpeed(rival, 24, track);
  setForwardSpeed(ego, 31, track);

  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.90 });
  const vehicles = [ego, rival];
  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };

  const origScan = egoAI.awareness.scan.bind(egoAI.awareness);
  egoAI.awareness.scan = (veh, allVehs, trk) => {
    const traffic = origScan(veh, allVehs, trk);
    if (traffic.primaryAttackTarget) {
      traffic.primaryAttackTarget.predictions = [
        { id: 'LATE_BRAKE_OVERSHOOT', probability: 0.85, envelopes: [] },
        { id: 'HOLD_LINE', probability: 0.10, envelopes: [] },
        { id: 'DEFEND_INSIDE', probability: 0.05, envelopes: [] }
      ];
    }
    return traffic;
  };

  egoAI.update(ego, vehicles, track, race, DT);

  console.log(`    Selected Mode: ${egoAI.combatEngine.attackMode} | Target Lateral: ${egoAI.trajectoryPlan?.selectedOffset?.toFixed(2)}m`);
  assert.equal(
    egoAI.combatEngine.attackMode,
    'SWITCHBACK',
    `High P(LATE_BRAKE_OVERSHOOT) must causally induce SWITCHBACK undercut (got ${egoAI.combatEngine.attackMode})`
  );
  console.log('    [PASS] Causal response to LATE_BRAKE_OVERSHOOT verified.\n');
}

console.log('================================================================================');
console.log('>>> ALL CAUSAL PREDICTION EXPECTED UTILITY TESTS PASSED CLEANLY (Exit 0) <<<');
console.log('================================================================================\n');
