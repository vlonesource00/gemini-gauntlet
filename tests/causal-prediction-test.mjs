/**
 * causal-prediction-test.mjs
 * Dedicated test suite verifying Causal Expected Utility Tactical Action Selection.
 *
 * Section 3 & 4:
 * Proves that varying opponent prediction probabilities are inferred PURELY from
 * observed history without ANY monkeypatching, and that expected-utility margins
 * causally select superior tactical actions with clear positive margins.
 */

import assert from 'node:assert/strict';
import { Circuit } from '../src/simulation/Track.js';
import { ENDURANCE_PARK } from '../src/scenarios/EndurancePark.js';
import { Vehicle } from '../src/simulation/Vehicle.js';
import { TrafficAwareness } from '../src/ai/TrafficAwareness.js';
import { NextGenAIController } from '../src/ai/v2/NextGenAIController.js';

const DT = 1 / 120;

console.log('================================================================================');
console.log('       GEMINI SUPREME — CAUSAL PREDICTION EXPECTED UTILITY TEST SUITE           ');
console.log('================================================================================\n');

const track = new Circuit(ENDURANCE_PARK);

// ===========================================================================
// SECTION 3: Causal Behavioral Prediction from Real Observed History
// ===========================================================================
console.log('--- SECTION 3: PURE CAUSAL PREDICTION INFERENCE (NO MONKEYPATCHING) ---\n');

// ---------------------------------------------------------------------------
// Case A: Rival holds stable line, normal braking -> predicted yield/concede is high
// ---------------------------------------------------------------------------
console.log('[1/4] Testing Case A: Stable Line Observation -> Concede / Hold Line Prediction...');
{
  const awareness = new TrafficAwareness();
  const ego = new Vehicle({ id: 'ego', spec: 'prototype' });
  const rival = new Vehicle({ id: 'rival', spec: 'gt' });

  // Feed 120 frames of steady line
  for (let i = 0; i < 120; i++) {
    ego.resetTo(track, 720 + i * 0.20, 0.0);
    rival.resetTo(track, 745 + i * 0.18, 1.0);
    awareness.scan(ego, [ego, rival], track, DT);
  }

  const traffic = awareness.scan(ego, [ego, rival], track, DT);
  const rivalEntry = traffic.entries.find((e) => e.other.id === 'rival');
  const preds = awareness.predictOpponentResponses(rivalEntry, track, {
    speed: 30,
    distance: ego.distance,
    lateral: 0
  });

  const totalProb = preds.reduce((sum, p) => sum + p.probability, 0);
  assert.ok(Math.abs(totalProb - 1.0) < 1e-4, 'Probabilities must normalize to 1.0');

  const holdLine = preds.find((p) => p.id === 'HOLD_LINE');
  console.log(`    Case A HOLD_LINE probability: ${holdLine.probability.toFixed(3)}`);
  assert.ok(holdLine.probability > 0.60, `Stable line must yield high HOLD_LINE prediction (got ${holdLine.probability})`);
  console.log('    [PASS] Case A inferred purely from observed history.\n');
}

// ---------------------------------------------------------------------------
// Case B: Rival moves inside early, holds inside -> predicted cover/defend inside is high
// ---------------------------------------------------------------------------
console.log('[2/4] Testing Case B: Early Inside Motion -> Defend Inside Prediction...');
{
  const awareness = new TrafficAwareness();
  const ego = new Vehicle({ id: 'ego', spec: 'prototype' });
  const rival = new Vehicle({ id: 'rival', spec: 'gt' });

  // On Endurance Park Turn 1 (s~770), turnSign > 0 (inside is positive lateral)
  for (let i = 0; i < 40; i++) {
    ego.resetTo(track, 720 + i * 0.20, 0.0);
    rival.resetTo(track, 745 + i * 0.18, 0.5 + i * 0.05); // drifts inside to +2.5
    awareness.scan(ego, [ego, rival], track, DT);
  }

  const traffic = awareness.scan(ego, [ego, rival], track, DT);
  const rivalEntry = traffic.entries.find((e) => e.other.id === 'rival');
  const preds = awareness.predictOpponentResponses(rivalEntry, track, {
    speed: 30,
    distance: ego.distance,
    lateral: 0
  });

  const totalProb = preds.reduce((sum, p) => sum + p.probability, 0);
  assert.ok(Math.abs(totalProb - 1.0) < 1e-4, 'Probabilities must normalize to 1.0');

  const defendInside = preds.find((p) => p.id === 'DEFEND_INSIDE');
  console.log(`    Case B DEFEND_INSIDE probability: ${defendInside.probability.toFixed(3)}`);
  assert.ok(defendInside.probability > 0.60, `Inside motion must infer high DEFEND_INSIDE prediction (got ${defendInside.probability})`);
  console.log('    [PASS] Case B inferred purely from observed history.\n');
}

// ---------------------------------------------------------------------------
// Case C: Rival brakes late / divebombs inside -> predicted dive/lunge is high
// ---------------------------------------------------------------------------
console.log('[3/4] Testing Case C: Hard / Late Braking -> High Braking / Overshoot Prediction...');
{
  const awareness = new TrafficAwareness();
  const ego = new Vehicle({ id: 'ego', spec: 'prototype' });
  const rival = new Vehicle({ id: 'rival', spec: 'gt' });

  rival.speed = 34;
  for (let i = 0; i < 30; i++) {
    ego.resetTo(track, 720 + i * 0.20, 0.0);
    rival.resetTo(track, 745 + i * 0.18, 0.0);
    rival.speed = Math.max(14, rival.speed - 0.45);
    rival.controls = { brake: 0.90, throttle: 0, steer: 0 };
    awareness.scan(ego, [ego, rival], track, DT);
  }

  const traffic = awareness.scan(ego, [ego, rival], track, DT);
  const rivalEntry = traffic.entries.find((e) => e.other.id === 'rival');
  const preds = awareness.predictOpponentResponses(rivalEntry, track, {
    speed: 30,
    distance: ego.distance,
    lateral: 0
  });

  const totalProb = preds.reduce((sum, p) => sum + p.probability, 0);
  assert.ok(Math.abs(totalProb - 1.0) < 1e-4, 'Probabilities must normalize to 1.0');

  const brakeEarly = preds.find((p) => p.id === 'BRAKE_EARLY').probability;
  const brakeNormal = preds.find((p) => p.id === 'BRAKE_NORMAL').probability;
  const overshoot = preds.find((p) => p.id === 'LATE_BRAKE_OVERSHOOT').probability;
  const totalBrakingThreat = brakeEarly + brakeNormal + overshoot;

  console.log(`    Case C Total Braking/Overshoot probability: ${totalBrakingThreat.toFixed(3)}`);
  assert.ok(totalBrakingThreat > 0.45, `Late/threshold braking must infer high braking/overshoot probability (got ${totalBrakingThreat})`);
  console.log('    [PASS] Case C inferred purely from observed history.\n');
}

// ---------------------------------------------------------------------------
// Case D: Rival moves outside to set up switchback -> predicted outside is high
// ---------------------------------------------------------------------------
console.log('[4/4] Testing Case D: Outside Motion -> Defend Outside Prediction...');
{
  const awareness = new TrafficAwareness();
  const ego = new Vehicle({ id: 'ego', spec: 'prototype' });
  const rival = new Vehicle({ id: 'rival', spec: 'gt' });

  for (let i = 0; i < 40; i++) {
    ego.resetTo(track, 720 + i * 0.20, 0.0);
    rival.resetTo(track, 745 + i * 0.18, 1.5 - i * 0.05); // drifts outside toward -0.5
    awareness.scan(ego, [ego, rival], track, DT);
  }

  const traffic = awareness.scan(ego, [ego, rival], track, DT);
  const rivalEntry = traffic.entries.find((e) => e.other.id === 'rival');
  const preds = awareness.predictOpponentResponses(rivalEntry, track, {
    speed: 30,
    distance: ego.distance,
    lateral: 0
  });

  const totalProb = preds.reduce((sum, p) => sum + p.probability, 0);
  assert.ok(Math.abs(totalProb - 1.0) < 1e-4, 'Probabilities must normalize to 1.0');

  const defendOutside = preds.find((p) => p.id === 'DEFEND_OUTSIDE');
  console.log(`    Case D DEFEND_OUTSIDE probability: ${defendOutside.probability.toFixed(3)}`);
  assert.ok(defendOutside.probability > 0.40, `Outside motion must infer high DEFEND_OUTSIDE prediction (got ${defendOutside.probability})`);
  console.log('    [PASS] Case D inferred purely from observed history.\n');
}

// ===========================================================================
// SECTION 4: Expected-Utility Margins & Decision Rationality
// ===========================================================================
console.log('--- SECTION 4: EXPECTED-UTILITY MARGINS & DECISION SELECTION ---\n');

// ---------------------------------------------------------------------------
// Case A Tactical Utility: Conceding Opponent -> Inside Pass Utility Exceeds Outside Pass
// ---------------------------------------------------------------------------
console.log('[5/6] Testing: Case A Expected Utilities -> Inside Dive Preferred with Positive Margin...');
{
  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.90 });
  const ego = new Vehicle({ id: 'ego', spec: 'prototype' });
  const rival = new Vehicle({ id: 'rival', spec: 'gt' });

  // Train Case A (steady line)
  for (let i = 0; i < 120; i++) {
    ego.resetTo(track, 720 + i * 0.20, 0.0);
    rival.resetTo(track, 745 + i * 0.18, 1.0);
    egoAI.awareness.scan(ego, [ego, rival], track, DT);
  }

  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };
  egoAI.update(ego, [ego, rival], track, race, DT);

  const diag = egoAI.combatEngine.lastTacticalDiagnostics;
  console.log(`    Selected: ${diag.selectedAction} | 2nd Best: ${diag.secondBestAction} | Margin: ${diag.utilityMargin}`);

  assert.ok(Array.isArray(diag.actionUtilities) && diag.actionUtilities.length > 0, 'actionUtilities must be recorded');
  assert.ok(diag.selectedAction, 'selectedAction must be identified');
  assert.ok(diag.utilityMargin > 0, `utilityMargin must be strictly positive (got ${diag.utilityMargin})`);
  assert.ok(diag.utilityMargin >= 2.0, `utilityMargin must exceed 2.0 threshold for clear corridor (got ${diag.utilityMargin})`);

  const insideUtil = diag.actionUtilities.find((u) => u.action === 'INSIDE_DIVE')?.expectedUtility ?? -999;
  const outsideUtil = diag.actionUtilities.find((u) => u.action === 'OUTSIDE_MOMENTUM')?.expectedUtility ?? -999;

  console.log(`    INSIDE_DIVE utility: ${insideUtil} vs OUTSIDE_MOMENTUM utility: ${outsideUtil}`);
  assert.ok(insideUtil > outsideUtil, `Conceding rival must yield inside utility > outside utility (${insideUtil} > ${outsideUtil})`);
  assert.equal(diag.selectedAction, 'INSIDE_DIVE', 'INSIDE_DIVE must be chosen when opponent concedes');
  console.log('    [PASS] Case A inside pass utility superiority verified.\n');
}

// ---------------------------------------------------------------------------
// Case B Tactical Utility: Inside Defending Opponent -> Outside Pass Utility Exceeds Inside Pass
// ---------------------------------------------------------------------------
console.log('[6/6] Testing: Case B Expected Utilities -> Outside / Switchback Preferred with Positive Margin...');
{
  const egoAI = new NextGenAIController('ego-ai', { aggression: 0.90 });
  const ego = new Vehicle({ id: 'ego', spec: 'prototype' });
  const rival = new Vehicle({ id: 'rival', spec: 'gt' });

  // Train Case B (moving inside)
  for (let i = 0; i < 40; i++) {
    ego.resetTo(track, 720 + i * 0.20, 0.0);
    rival.resetTo(track, 745 + i * 0.18, 0.5 + i * 0.05);
    egoAI.awareness.scan(ego, [ego, rival], track, DT);
  }

  const race = { phase: 'racing', raceTime: 5.0, elapsed: 5.0, statusFor: () => ({ position: 1 }) };
  egoAI.update(ego, [ego, rival], track, race, DT);

  const diag = egoAI.combatEngine.lastTacticalDiagnostics;
  console.log(`    Selected: ${diag.selectedAction} | 2nd Best: ${diag.secondBestAction} | Margin: ${diag.utilityMargin}`);

  assert.ok(diag.utilityMargin > 0, `utilityMargin must be strictly positive (got ${diag.utilityMargin})`);
  assert.ok(diag.utilityMargin >= 2.0, `utilityMargin must exceed 2.0 threshold for clear corridor (got ${diag.utilityMargin})`);

  const insideUtil = diag.actionUtilities.find((u) => u.action === 'INSIDE_DIVE')?.expectedUtility ?? -999;
  const outsideUtil = diag.actionUtilities.find((u) => u.action === 'OUTSIDE_MOMENTUM')?.expectedUtility ?? -999;
  const switchbackUtil = diag.actionUtilities.find((u) => u.action === 'SWITCHBACK')?.expectedUtility ?? -999;

  console.log(`    SWITCHBACK: ${switchbackUtil} | OUTSIDE: ${outsideUtil} vs INSIDE_DIVE: ${insideUtil}`);
  assert.ok(outsideUtil > insideUtil, `Inside defending rival must yield outside utility > inside utility (${outsideUtil} > ${insideUtil})`);
  assert.ok(switchbackUtil > insideUtil, `Inside defending rival must yield switchback utility > inside utility (${switchbackUtil} > ${insideUtil})`);
  assert.notEqual(diag.selectedAction, 'INSIDE_DIVE', 'Must NOT dive inside into an opponent who defends inside');
  console.log('    [PASS] Case B outside pass utility superiority verified.\n');
}

console.log('================================================================================');
console.log('>>> ALL CAUSAL PREDICTION EXPECTED UTILITY TESTS PASSED CLEANLY (Exit 0) <<<');
console.log('================================================================================\n');
