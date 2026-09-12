/**
 * anti-twitch-test.mjs
 * Dedicated verification of Trajectory Continuity and Anti-Twitch Decision Stability.
 *
 * Runs an autonomous GT vehicle through a full corner sequence on Harbor Ring at 120Hz physics
 * (with 25Hz replanning and 10Hz tactical combat cycles), measuring:
 * - geometrySwitchCount (must remain bounded without high-frequency chatter)
 * - flankReversalCount (must be zero: no rapid left/right oscillations)
 * - maxNearHorizonHeadingDiscontinuity (must remain <= 0.08 rad)
 * - maxNearHorizonCurvatureDiscontinuity (must remain <= 0.015 m^-1)
 * - rmsNearHorizonDivergence (must remain <= 0.22 m)
 * - offTrackSeconds (must be 0.00s)
 */

import assert from 'node:assert/strict';
import { Circuit } from '../src/simulation/Track.js';
import { HARBOR_RING } from '../src/scenarios/HarborRing.js';
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
console.log('       GEMINI SUPREME — ANTI-TWITCH & TRAJECTORY CONTINUITY TEST SUITE          ');
console.log('================================================================================\n');

const track = new Circuit(HARBOR_RING);
const ego = new Vehicle({ id: 'ego-gt', spec: 'gt' });

// Approach Turn 1 on Harbor Ring at race pace
ego.resetTo(track, 200, 0.0);
setForwardSpeed(ego, 42, track);

const egoAI = new NextGenAIController('ego-gt-ai', { aggression: 0.85 });
const vehicles = [ego];
const race = { phase: 'racing', raceTime: 10.0, elapsed: 10.0, statusFor: () => ({ position: 1 }) };

let maxHeadingDiscontinuity = 0;
let maxCurvatureDiscontinuity = 0;
let maxRmsDivergence = 0;
let offTrackSeconds = 0;
let maxBetaRad = 0;
const betaSamples = [];

// Simulate 4.0 seconds (480 physics steps, 100 replan cycles at 25Hz) through Turn 1
const simDurationS = 4.0;
const totalSteps = Math.round(simDurationS / DT);

console.log(`[1/1] Simulating Harbor Ring GT Turn 1 (${simDurationS}s, ${totalSteps} steps at 120Hz)...`);

for (let step = 0; step < totalSteps; step++) {
  race.raceTime += DT;
  race.elapsed += DT;

  egoAI.update(ego, vehicles, track, race, DT);
  ego.step(DT, track, true);

  const telem = egoAI.getTelemetry?.() || {};
  if (telem.headingDiscontinuity !== undefined) {
    maxHeadingDiscontinuity = Math.max(maxHeadingDiscontinuity, telem.headingDiscontinuity);
  }
  if (telem.curvatureDiscontinuity !== undefined) {
    maxCurvatureDiscontinuity = Math.max(maxCurvatureDiscontinuity, telem.curvatureDiscontinuity);
  }
  if (telem.rmsNearHorizonDivergence !== undefined) {
    maxRmsDivergence = Math.max(maxRmsDivergence, telem.rmsNearHorizonDivergence);
  }

  const vx = Math.abs(ego.localVelocity?.x || 0);
  const vz = Math.max(1.0, Math.abs(ego.localVelocity?.z || ego.speed || 1.0));
  const beta = Math.atan2(vx, vz);
  maxBetaRad = Math.max(maxBetaRad, beta);
  betaSamples.push(beta);

  if (ego.surface?.zone === 'grass' || ego.surface?.zone === 'runoff') {
    offTrackSeconds += DT;
  }
}

betaSamples.sort((a, b) => a - b);
const p95BetaDeg = (betaSamples[Math.floor(betaSamples.length * 0.95)] * (180 / Math.PI)).toFixed(2);
const maxBetaDeg = (maxBetaRad * (180 / Math.PI)).toFixed(2);

const finalTelem = egoAI.getTelemetry?.() || {};
const geometrySwitches = finalTelem.geometrySwitchCount ?? 0;
const semanticSwitches = finalTelem.semanticSwitchCount ?? 0;
const flankReversals = finalTelem.flankReversalCount ?? 0;
const materialSwitches = finalTelem.materialSwitchCount ?? 0;
const replans = finalTelem.replanCount ?? 0;

console.log('--- Anti-Twitch & Trajectory Continuity Results ---');
console.log(`  Total Replans:                      ${replans}`);
console.log(`  Geometry Material Switches:         ${geometrySwitches}`);
console.log(`  Semantic Maneuver Switches:         ${semanticSwitches}`);
console.log(`  Flank Direction Reversals:          ${flankReversals}`);
console.log(`  Total Material Switches:            ${materialSwitches}`);
console.log(`  Max Heading Discontinuity (Δψ):     ${maxHeadingDiscontinuity.toFixed(4)} rad`);
console.log(`  Max Curvature Discontinuity (Δκ):   ${maxCurvatureDiscontinuity.toFixed(5)} m^-1`);
console.log(`  Max RMS Near-Horizon Divergence:    ${maxRmsDivergence.toFixed(4)} m`);
console.log(`  Peak Sideslip Beta:                 ${maxBetaDeg}° (p95: ${p95BetaDeg}°)`);
console.log(`  Off-Track Time:                     ${offTrackSeconds.toFixed(3)}s`);

assert.equal(flankReversals, 0, `Flank reversal count must be zero (no rapid left/right flipping, got ${flankReversals})`);
assert.ok(geometrySwitches <= 4, `Geometry material switches must be low and bounded (got ${geometrySwitches} out of ${replans} replans)`);
assert.ok(maxHeadingDiscontinuity <= 0.08, `Near-horizon heading discontinuity must remain <= 0.08 rad (got ${maxHeadingDiscontinuity.toFixed(4)} rad)`);
assert.ok(maxCurvatureDiscontinuity <= 0.015, `Near-horizon curvature discontinuity must remain <= 0.015 m^-1 (got ${maxCurvatureDiscontinuity.toFixed(5)} m^-1)`);
assert.ok(maxRmsDivergence <= 0.22, `Near-horizon RMS divergence must remain <= 0.22 m (got ${maxRmsDivergence.toFixed(4)} m)`);
assert.equal(offTrackSeconds, 0, `Must remain 100% on track (got ${offTrackSeconds.toFixed(2)}s off track)`);

console.log('\n================================================================================');
console.log('>>> ALL ANTI-TWITCH & TRAJECTORY CONTINUITY TESTS PASSED CLEANLY (Exit 0) <<<');
console.log('================================================================================\n');
