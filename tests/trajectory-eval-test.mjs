import assert from 'node:assert/strict';
import { Circuit } from '../src/simulation/Track.js';
import { ENDURANCE_PARK } from '../src/scenarios/EndurancePark.js';
import { HARBOR_RING } from '../src/scenarios/HarborRing.js';
import { Vehicle } from '../src/simulation/Vehicle.js';
import { FrenetLatticePlanner, minimumJerk } from '../src/ai/FrenetLatticePlanner.js';
import { GlobalTimeOptimalEngine } from '../src/ai/v2/GlobalTimeOptimalEngine.js';
import { GameTheoreticCombatEngine } from '../src/ai/v2/GameTheoreticCombatEngine.js';
import { CoupledMPCCController } from '../src/ai/v2/CoupledMPCCController.js';
import { CombatDynamicsEngine } from '../src/ai/v2/CombatDynamicsEngine.js';

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

// ---------------------------------------------------------------------------
// 5. Layer 1: Global Time-Optimal 2D Free-Boundary Profile Solver
// ---------------------------------------------------------------------------
console.log('  -> Testing Layer 1: Global Time-Optimal Profile Solver & Chicane Flattening...');
const globalOptEngine = new GlobalTimeOptimalEngine({ track });
assert.ok(globalOptEngine.isSolved, 'GlobalTimeOptimalEngine must solve the complete profile on init');
assert.ok(globalOptEngine.profile.length >= 180, 'Profile must have continuous spatial nodes');

// Sample test across track
const samplePace = globalOptEngine.sampleAtDistance(250);
finite(samplePace.lateral, 'samplePace.lateral');
finite(samplePace.targetSpeed, 'samplePace.targetSpeed');
finite(samplePace.curvature, 'samplePace.curvature');
finite(samplePace.x, 'samplePace.x');
finite(samplePace.z, 'samplePace.z');
assert.ok(samplePace.targetSpeed > 15.0, 'Target speed must be physically viable (>15 m/s)');

// Test chicane / corner radius flattening: effective curvature must be smoothed
const curvedNodes = globalOptEngine.profile.filter((n) => Math.abs(n.curvature) > 0.005);
assert.ok(curvedNodes.length > 0, 'Profile must capture corner zones');
for (const cNode of curvedNodes.slice(0, 5)) {
  assert.ok(cNode.targetSpeed > 0, 'Corner node speed must be strictly positive');
  assert.ok(Math.abs(cNode.lateral) <= track.roadHalfWidth + 0.5, 'Optimal line must obey road boundary constraints');
}

console.log(`    [PASS] Layer 1 Global Time-Optimal Profile solved: nodeCount=${globalOptEngine.profile.length}, sampleSpeed=${samplePace.targetSpeed.toFixed(1)}m/s`);

// ---------------------------------------------------------------------------
// 6. Layer 2: Game-Theoretic Combat Engine (Stackelberg & IBR Tactical Games)
// ---------------------------------------------------------------------------
console.log('  -> Testing Layer 2: Stackelberg Leader Defense & IBR Attack Game Decisions...');
const gtEngine = new GameTheoreticCombatEngine({ roadHalfWidth: track.roadHalfWidth, curbWidth: track.curbWidth });

// Test 6a: Threatening Challenger -> Stackelberg Defense
const challengerCar = new Vehicle({ id: 'threat-challenger', spec: 'prototype' });
challengerCar.speed = 38;
const defResult = gtEngine.evaluate({
  vehicle: ego,
  track,
  traffic: {
    entries: [{
      other: challengerCar,
      delta: -10.0,
      lateralDelta: 0.5,
      otherForwardSpeed: 38
    }],
    egoForwardSpeed: 32
  },
  optimalProfile: globalOptEngine,
  aggression: 0.90,
  dt: 0.016
});

assert.equal(defResult.role, 'DEFEND', 'GT Engine must trigger DEFEND role against fast-closing challenger');
assert.ok(['BREAK_TOW', 'APEX_SHIELD', 'EXIT_SQUEEZE'].includes(defResult.defenseMode), 'Must engage valid defensive mode');
assert.ok(Math.abs(defResult.targetLateral) <= track.roadHalfWidth + 0.5, 'Defensive target must remain within legal track limits');

// Test 6b: Opportunity Ahead -> IBR Slingshot Attack
const targetCar = new Vehicle({ id: 'prey-car', spec: 'gt' });
targetCar.speed = 25;
const atkResult = gtEngine.evaluate({
  vehicle: ego,
  track,
  traffic: {
    entries: [{
      other: targetCar,
      delta: 15.0,
      lateralDelta: 0.0,
      otherLateral: 0.0,
      otherForwardSpeed: 25
    }],
    egoForwardSpeed: 35
  },
  optimalProfile: globalOptEngine,
  aggression: 0.95,
  dt: 0.016
});

assert.equal(atkResult.role, 'ATTACK', 'GT Engine must trigger ATTACK role against vulnerable target ahead');
assert.ok(['SLINGSHOT', 'DIVEBOMB', 'SWITCHBACK', 'SIDE_BY_SIDE'].includes(atkResult.attackMode), 'Must select legal tactical attack mode');
assert.ok(atkResult.desiredSpeed >= 35.0, 'Attack mode must command aggressive overtake closing velocity');

console.log(`    [PASS] Layer 2 Combat Engine: defenseMode=${defResult.defenseMode} (targetLat=${defResult.targetLateral.toFixed(2)}m), attackMode=${atkResult.attackMode} (desiredSpeed=${atkResult.desiredSpeed.toFixed(1)}m/s)`);

// ---------------------------------------------------------------------------
// 7. Layer 3: Coupled MPCC Controller & Pacejka Friction-Circle Constraints
// ---------------------------------------------------------------------------
console.log('  -> Testing Layer 3: Coupled MPCC & Pacejka Friction Circle Budget Allocation...');
const mpcc = new CoupledMPCCController({ horizonSeconds: 2.4, nodeCount: 16 });
assert.equal(mpcc.predPoints.length, 16, 'Prediction horizon points must be pre-allocated');

// Test 7a: Cornering deceleration with high lateral acceleration -> Trail-braking modulation
const corneringEgo = new Vehicle({ id: 'cornering-ego', spec: 'prototype' });
corneringEgo.resetTo(track, 100, 0.0);
corneringEgo.speed = 30;
corneringEgo.yawRate = 0.85;

const stepCornering = mpcc.step({
  vehicle: corneringEgo,
  track,
  tacticalTarget: { targetLateral: 2.0, desiredSpeed: 10.0, dMin: -5, dMax: 5 },
  dt: 0.016,
  tireGripFactor: 1.0,
  aggression: 0.85
});

assert.ok(stepCornering.friction.latUtilization > 0.4, 'Lateral friction utilization must reflect heavy cornering load');
assert.ok(stepCornering.friction.remainingLongBudget < 1.0, 'Longitudinal budget must be throttled under cornering load');
assert.ok(stepCornering.brake > 0, 'Trail-braking must be active');
assert.ok(Number.isFinite(stepCornering.steer) && Math.abs(stepCornering.steer) <= 1.0, 'Steering command must be finite and bounded');

// Test 7b: Straightaway full power acceleration
const straightEgo = new Vehicle({ id: 'straight-ego', spec: 'prototype' });
straightEgo.resetTo(track, 100, 0.0);
straightEgo.speed = 20;
straightEgo.yawRate = 0.0;

const stepStraight = mpcc.step({
  vehicle: straightEgo,
  track,
  tacticalTarget: { targetLateral: 0.0, desiredSpeed: 50.0, dMin: -5, dMax: 5 },
  dt: 0.016,
  tireGripFactor: 1.0,
  aggression: 0.85
});

assert.equal(stepStraight.throttle, 1.0, 'Straight line acceleration must demand 100% full throttle');
assert.equal(stepStraight.brake, 0.0, 'Straight line acceleration must demand zero braking');

console.log(`    [PASS] Layer 3 Coupled MPCC: trailBrakeUtil=${stepCornering.friction.latUtilization.toFixed(2)}, straightThrottle=${stepStraight.throttle.toFixed(2)}`);

// ---------------------------------------------------------------------------
// 8. Combat Dynamics Engine: Rubbing Equilibrium & Power-Slide Recovery
// ---------------------------------------------------------------------------
console.log('  -> Testing Combat Dynamics: Elastic Rubbing Equilibrium & Power-Slide Counter-Steer...');
const combatDynamics = new CombatDynamicsEngine();

// Test 8a: Rubbing contact tolerance (no lift-off)
const rubbingRes = combatDynamics.process({
  vehicle: ego,
  traffic: {
    entries: [{
      other: new Vehicle({ id: 'side-by-side', spec: 'prototype' }),
      delta: 1.0,
      lateralDelta: 1.2
    }]
  },
  controls: { steer: 0, throttle: 0.20, brake: 0 },
  dt: 0.016
});

assert.equal(rubbingRes.rubbing, true, 'Side-by-side contact must be recognized as elastic rubbing');
assert.ok(rubbingRes.throttle >= 0.45, 'Throttle must be maintained (>=45%) during rubbing without cutting power');

// Test 8b: Dynamic slip-slope power slide recovery under oversteer snap
const oversteerEgo = new Vehicle({ id: 'oversteer-ego', spec: 'prototype' });
oversteerEgo.speed = 25;
oversteerEgo.localVelocity = { x: 12.0, z: 25.0 }; // Slip angle ~ 0.446 rad (> 0.32 rad threshold)
oversteerEgo.yawRate = 1.80; // Diverging yaw rate (> 1.25 rad/s)

const slideRes = combatDynamics.process({
  vehicle: oversteerEgo,
  traffic: { entries: [] },
  controls: { steer: 0.10, throttle: 0.80, brake: 0 },
  dt: 0.016
});

assert.equal(slideRes.powerSlide, true, 'High slip angle and yaw rate must trigger active power-slide stabilization');
assert.ok(slideRes.steer < 0, 'Power-slide counter-steer must oppose positive yaw rate');
assert.ok(slideRes.throttle >= 0.25, 'Power-slide recovery must maintain stability drive torque');

console.log(`    [PASS] Combat Dynamics: rubbingDetected=${rubbingRes.rubbing} (throttle=${rubbingRes.throttle.toFixed(2)}), powerSlideSteer=${slideRes.steer.toFixed(3)}`);

console.log('=== Trajectory Lattice & Cost Evaluation Test Suite: ALL ASSERTIONS PASSED ===\n');
