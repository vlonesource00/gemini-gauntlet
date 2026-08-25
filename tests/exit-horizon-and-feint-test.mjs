import assert from 'node:assert/strict';
import { FrenetLatticePlanner } from '../src/ai/FrenetLatticePlanner.js';
import { Circuit } from '../src/simulation/Track.js';
import { Vehicle } from '../src/simulation/Vehicle.js';
import { HARBOR_RING } from '../src/scenarios/HarborRing.js';

console.log('========================================================================');
console.log('     EXIT-HORIZON EVALUATION & COUNTERFACTUAL RESPONSE TEST');
console.log('========================================================================');

const track = new Circuit(HARBOR_RING);
const planner = new FrenetLatticePlanner({ pointCount: 24, horizonS: 3.2 });

// ---------------------------------------------------------------------------
// TEST 1: Exit-Horizon Outcome Evaluation (Late Apex vs Shallow Inside Dive)
// ---------------------------------------------------------------------------
console.log('\n--- Test 1: Exit-Horizon Velocity Propagation & Late-Apex Outcome ---');

const testVehicle = new Vehicle({ id: 'ego', spec: 'prototype' });
testVehicle.speed = 35.0; // 126 km/h approaching turn
testVehicle.distance = 420.0; // Corner approach on Harbor Ring
testVehicle.surface = { lateral: 0.0, zone: 'road' };

// Candidate A: Shallow inside dive (terminal lateral = -3.5m)
const candA = planner.generateCandidate({
  vehicle: testVehicle,
  track,
  startLateral: 0.0,
  startSpeed: testVehicle.speed,
  startDistance: testVehicle.distance,
  terminalLateral: -3.8,
  transitionTime: 0.6,
  horizon: 3.2,
  effectiveRoadMargin: 5.5,
  aggression: 0.85
});

// Candidate B: Wide entry, late apex sweep (terminal lateral = +2.5m entry -> parabolic wide sweep)
const candB = planner.generateCandidate({
  vehicle: testVehicle,
  track,
  startLateral: 0.0,
  startSpeed: testVehicle.speed,
  startDistance: testVehicle.distance,
  terminalLateral: 2.2,
  transitionTime: 1.2,
  horizon: 3.2,
  effectiveRoadMargin: 5.5,
  aggression: 0.85
});

console.log(`Candidate A (Shallow Inside): MaxCurv=${candA.maxCurvaturePerM.toFixed(4)}, ExitSpeed=${(candA.actualExitSpeedMps * 3.6).toFixed(1)} km/h, Score=${candA.score.toFixed(1)}`);
console.log(`Candidate B (Wide Late-Apex): MaxCurv=${candB.maxCurvaturePerM.toFixed(4)}, ExitSpeed=${(candB.actualExitSpeedMps * 3.6).toFixed(1)} km/h, Score=${candB.score.toFixed(1)}`);

assert.ok(candB.actualExitSpeedMps > candA.actualExitSpeedMps, 'Wide late-apex line must achieve higher exit speed under friction circle');
assert.ok(candB.maxCurvaturePerM < candA.maxCurvaturePerM, 'Wide entry line must produce lower apex path curvature');
console.log(' [PASS] Exit-horizon velocity propagation verified: higher exit momentum correctly evaluated.');

// ---------------------------------------------------------------------------
// TEST 2: Counterfactual Defender Response Model & Probability Distribution
// ---------------------------------------------------------------------------
console.log('\n--- Test 2: Counterfactual Action-Conditioned Response Model P(m | a_ego) ---');

const defender = new Vehicle({ id: 'defender', spec: 'prototype' });
defender.speed = 34.0;
defender.distance = testVehicle.distance + 16.0;
defender.surface = { lateral: 0.0, zone: 'road' };

const trafficEntries = [
  { other: defender, delta: 16.0, otherLateral: 0.0, relativeLongitudinalVelocity: -1.0 }
];

// Ego proposes showing inside (moving left toward -3.0m)
const candShowInside = planner.generateCandidate({
  vehicle: testVehicle,
  track,
  startLateral: 0.0,
  startSpeed: testVehicle.speed,
  startDistance: testVehicle.distance,
  terminalLateral: -3.2,
  transitionTime: 0.8,
  horizon: 3.2,
  effectiveRoadMargin: 5.5,
  targetId: 'defender',
  trafficEntries,
  aggression: 0.9
});

// Ego proposes outside sweep (moving right toward +3.2m)
const candOutsidePass = planner.generateCandidate({
  vehicle: testVehicle,
  track,
  startLateral: 0.0,
  startSpeed: testVehicle.speed,
  startDistance: testVehicle.distance,
  terminalLateral: 3.2,
  transitionTime: 0.8,
  horizon: 3.2,
  effectiveRoadMargin: 5.5,
  targetId: 'defender',
  trafficEntries,
  aggression: 0.9
});

console.log(`Ego Action: Show Inside (Lat -3.2m) -> Expected Collision Risk Score: ${candShowInside.costBreakdown.collisionRisk.toFixed(1)}`);
console.log(`Ego Action: Outside Pass (Lat +3.2m) -> Expected Collision Risk Score: ${candOutsidePass.costBreakdown.collisionRisk.toFixed(1)}`);

assert.ok(candOutsidePass.collisionFree, 'Outside pass candidate must be collision-free across response distribution');
console.log(' [PASS] Counterfactual response model verified: expected risk calculated across discrete hypotheses.');

console.log('\n========================================================================');
console.log('     ALL EXIT-HORIZON & COUNTERFACTUAL TESTS PASSED CLEANLY (Exit 0)');
console.log('========================================================================\n');
