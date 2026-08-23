import assert from 'node:assert/strict';
import { Circuit } from '../src/simulation/Track.js';
import { ENDURANCE_PARK } from '../src/scenarios/EndurancePark.js';
import { Vehicle } from '../src/simulation/Vehicle.js';
import { FrenetLatticePlanner, minimumJerk } from '../src/ai/FrenetLatticePlanner.js';

const finite = (value, label) => assert.ok(Number.isFinite(value), `${label} must be finite`);

console.log('=== [2/5] Running Trajectory Lattice & Cost Evaluation Test Suite ===');

// ---------------------------------------------------------------------------
// 1. Quintic Minimum-Jerk Mathematical Properties
// ---------------------------------------------------------------------------
console.log('  -> Testing Minimum-Jerk Polynomial Smoothness & Boundaries...');
assert.equal(minimumJerk(0), 0, 'minimumJerk(0) must equal 0');
assert.equal(minimumJerk(1), 1, 'minimumJerk(1) must equal 1');
assert.equal(minimumJerk(0.5), 0.5, 'minimumJerk(0.5) must equal 0.5 by point symmetry');

// Monotonicity and boundary zero-derivative checks
let previousJerk = -1;
for (let i = 0; i <= 100; i += 1) {
  const u = i / 100;
  const val = minimumJerk(u);
  assert.ok(val >= previousJerk - 1e-7, 'minimumJerk must be monotonically increasing in [0, 1]');
  previousJerk = val;
}

// Flat boundary derivative (C2 continuity: velocity and acceleration start/end at 0)
const initialSlope = (minimumJerk(0.01) - minimumJerk(0)) / 0.01;
const terminalSlope = (minimumJerk(1.0) - minimumJerk(0.99)) / 0.01;
assert.ok(initialSlope < 0.05, `Initial slope must be near zero for jerk-free entry (got ${initialSlope.toFixed(4)})`);
assert.ok(terminalSlope < 0.05, `Terminal slope must be near zero for jerk-free settle (got ${terminalSlope.toFixed(4)})`);

console.log('    [PASS] Minimum-jerk polynomial satisfies C2 boundary and monotonicity contracts.');

// ---------------------------------------------------------------------------
// 2. Frenet Lattice Candidate Generation on Clear Road
// ---------------------------------------------------------------------------
console.log('  -> Testing Multi-Candidate Trajectory Generation across Lateral Offsets & Transition Times...');
const track = new Circuit(ENDURANCE_PARK);
const ego = new Vehicle({ id: 'ego-car', spec: 'prototype' });
ego.resetTo(track, 100, 0.0);
ego.speed = 32;
ego.velocity = { x: ego.forward.x * 32, y: 0, z: ego.forward.z * 32 };

const planner = new FrenetLatticePlanner({ pointCount: 24, horizonS: 3.4 });

const tacticalCandidates = [
  { offset: 3.8, intentType: 'TACTICAL_PASS', transitionScales: [0.8, 1.0, 1.25] },
  { offset: 0.0, intentType: 'HOLD_LANE', transitionScales: [1.0] },
  { offset: -3.5, intentType: 'ALTERNATIVE_LINE', transitionScales: [0.9, 1.2] }
];

const plan = planner.plan({
  vehicle: ego,
  track,
  desiredOffset: 3.8,
  fallbackOffsets: [0.0, -3.5],
  tacticalCandidates,
  targetSpeed: 36,
  aggression: 0.82,
  racecraftPhase: 'ATTACK_INSIDE',
  lookAhead: 24
});

// Trajectory point structure assertions
assert.equal(plan.points.length, 24, 'Planner must publish complete discretized 24-point trajectory');
assert.ok(plan.candidateCount >= 6, `Planner must evaluate a diverse candidate lattice (evaluated ${plan.candidateCount} candidates)`);
assert.ok(plan.collisionFree, 'Clear track trajectory must be collision-free');
assert.ok(plan.roadLegal, 'Selected clear trajectory must stay within road boundaries');
assert.ok(Math.abs(plan.selectedOffset - 3.8) < 0.2, `Clear-road tactical intent must select desired offset 3.8m (got ${plan.selectedOffset.toFixed(2)}m)`);
assert.ok(plan.committed, 'Committed attack phase must set committed flag');

// Check progression of trajectory points
for (let i = 0; i < plan.points.length; i += 1) {
  const p = plan.points[i];
  finite(p.x, `point[${i}].x`);
  finite(p.y, `point[${i}].y`);
  finite(p.z, `point[${i}].z`);
  finite(p.s, `point[${i}].s`);
  finite(p.lateral, `point[${i}].lateral`);
  finite(p.time, `point[${i}].time`);
  finite(p.speed, `point[${i}].speed`);

  if (i > 0) {
    assert.ok(p.time > plan.points[i - 1].time, `Time must be strictly increasing at point ${i}`);
    assert.ok(p.forwardDistance >= plan.points[i - 1].forwardDistance - 1e-4, `Forward distance must be non-decreasing at point ${i}`);
    assert.ok(p.lateral >= plan.points[i - 1].lateral - 1e-4, `Lateral change towards +3.8m must be directionally monotonic at point ${i}`);
  }
}

// Minimum-jerk S-curve step distribution: peak in middle, near-zero at ends
const lateralSteps = plan.points.slice(1).map((pt, idx) => pt.lateral - plan.points[idx].lateral);
const peakStep = Math.max(...lateralSteps);
assert.ok(lateralSteps[0] < peakStep * 0.4, 'Initial lateral rate must be progressive (jerk-minimizing)');
assert.ok(lateralSteps.at(-1) < peakStep * 0.25, 'Terminal lateral rate must flatten smoothly upon lane settlement');

console.log(`    [PASS] Trajectory generation: candidateCount=${plan.candidateCount}, selectedOffset=${plan.selectedOffset.toFixed(2)}m, maxLatAccel=${plan.maxLateralAccelerationMps2.toFixed(2)}m/s²`);

// ---------------------------------------------------------------------------
// 3. Cost Function: Road-Violation Penalty Verification
// ---------------------------------------------------------------------------
console.log('  -> Testing Cost Function Penalties on Road-Violating Candidates...');

// Force an extreme off-track candidate (lateral offset = 18m, well into grass)
const offTrackCandidate = planner._evaluateCandidate({
  vehicle: ego,
  track,
  startLateral: 0,
  terminalLateral: 18.0,
  desiredOffset: 18.0,
  transitionTime: 1.5,
  targetSpeed: 34,
  trafficEntries: [],
  roadMargin: 6.0,
  committed: false,
  aggression: 0.5,
  horizon: 3.4,
  targetId: null,
  kerbAllowance: 0
});

const onTrackCandidate = planner._evaluateCandidate({
  vehicle: ego,
  track,
  startLateral: 0,
  terminalLateral: 2.5,
  desiredOffset: 2.5,
  transitionTime: 1.5,
  targetSpeed: 34,
  trafficEntries: [],
  roadMargin: 6.0,
  committed: false,
  aggression: 0.5,
  horizon: 3.4,
  targetId: null,
  kerbAllowance: 0
});

assert.equal(offTrackCandidate.roadLegal, false, 'Candidate with terminalLateral=18m must be flagged roadLegal=false');
assert.equal(onTrackCandidate.roadLegal, true, 'Candidate with terminalLateral=2.5m must be flagged roadLegal=true');
assert.ok(offTrackCandidate.score > 1e6, `Off-track candidate must receive massive violation score (>1e6, got ${offTrackCandidate.score.toFixed(0)})`);
assert.ok(offTrackCandidate.score > onTrackCandidate.score * 1000, 'Road-violating candidate cost must vastly exceed clean candidate cost');

console.log(`    [PASS] Road violation cost: offTrackScore=${offTrackCandidate.score.toFixed(0)} vs onTrackScore=${onTrackCandidate.score.toFixed(1)}`);

// ---------------------------------------------------------------------------
// 4. Cost Function: Traffic Collision Penalty & Multi-Candidate Evasion
// ---------------------------------------------------------------------------
console.log('  -> Testing Collision Penalties & Automatic Alternative Corridor Selection...');
const blocker = new Vehicle({ id: 'blocker-car', spec: 'gt', player: true });
blocker.resetTo(track, 114, 3.8); // Exactly occupying the desired 3.8m lane, 14m ahead
blocker.speed = 18;
blocker.velocity = { x: blocker.forward.x * 18, y: 0, z: blocker.forward.z * 18 };

const trafficEntry = {
  other: blocker,
  delta: 14,
  lateralDelta: 3.8,
  longitudinal: 14,
  side: 3.8,
  otherLateral: 3.8,
  otherTargetLateral: 3.8,
  relativeLongitudinalVelocity: 14,
  relativeLateralVelocity: 0
};

// Evaluate the colliding candidate directly to verify collision penalty
const collidingCandidate = planner._evaluateCandidate({
  vehicle: ego,
  track,
  startLateral: 0,
  terminalLateral: 3.8,
  desiredOffset: 3.8,
  transitionTime: 1.2,
  targetSpeed: 34,
  trafficEntries: [trafficEntry],
  roadMargin: 6.0,
  committed: false,
  aggression: 0.8,
  horizon: 3.4,
  targetId: blocker.id
});

assert.equal(collidingCandidate.collisionFree, false, 'Candidate steering directly into blocker must be flagged collisionFree=false');
assert.ok(collidingCandidate.score > 20000, `Colliding candidate must receive severe collision penalty (>20,000, got ${collidingCandidate.score.toFixed(0)})`);

// Full planner run with fallback offsets available
const blockedPlan = planner.plan({
  vehicle: ego,
  track,
  desiredOffset: 3.8,
  fallbackOffsets: [-3.2, 0.0],
  tacticalCandidates: [
    { offset: 3.8, intentType: 'PRIMARY_BLOCKED' },
    { offset: -3.2, intentType: 'OPEN_INSIDE' },
    { offset: 0.0, intentType: 'CENTER_HOLD' }
  ],
  trafficEntries: [trafficEntry],
  targetSpeed: 34,
  aggression: 0.8,
  racecraftPhase: 'PACE',
  lookAhead: 24
});

assert.ok(blockedPlan.collisionFree, 'Planner must select a collision-free alternative trajectory');
assert.ok(blockedPlan.selectedOffset < 1.0, `Planner must switch away from occupied corridor 3.8m (selected ${blockedPlan.selectedOffset.toFixed(2)}m)`);
assert.ok(blockedPlan.minimumClearanceM > 0, `Planner must maintain positive safety clearance (got ${blockedPlan.minimumClearanceM.toFixed(2)}m)`);

console.log(`    [PASS] Collision avoidance: collidingScore=${collidingCandidate.score.toFixed(0)}, selectedSafeOffset=${blockedPlan.selectedOffset.toFixed(2)}m, minimumClearance=${blockedPlan.minimumClearanceM.toFixed(2)}m`);

console.log('=== Trajectory Lattice & Cost Evaluation Test Suite: ALL ASSERTIONS PASSED ===\n');
