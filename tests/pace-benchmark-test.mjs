import assert from 'node:assert/strict';
import { Circuit } from '../src/simulation/Track.js';
import { Vehicle } from '../src/simulation/Vehicle.js';
import { NextGenAIController } from '../src/ai/v2/NextGenAIController.js';
import { HARBOR_RING } from '../src/scenarios/HarborRing.js';
import { ENDURANCE_PARK } from '../src/scenarios/EndurancePark.js';

console.log('=== [PACE BENCHMARK] Multi-Class Flying Lap Pace Verification ===');

const DT = 1 / 120;
const mmss = (t) => {
  if (!Number.isFinite(t)) return '  --   ';
  const m = Math.floor(t / 60);
  return `${m}:${(t - m * 60).toFixed(3).padStart(6, '0')}`;
};

async function testCircuitPace(circuitDef, circuitName) {
  console.log(`\n----------------------------------------------------------------------`);
  console.log(`  Benchmarking Circuit: ${circuitName} (Length: ${circuitDef.length || 2704}m)`);
  console.log(`----------------------------------------------------------------------`);
  const track = new Circuit(circuitDef);

  const testClasses = [
    { spec: 'prototype', targetMaxLap: 68.0, name: 'Prototype Class' },
    { spec: 'gt', targetMaxLap: 85.0, name: 'GT Class' },
    { spec: 'touring', targetMaxLap: 98.0, name: 'Touring Class' }
  ];

  for (const { spec, targetMaxLap, name } of testClasses) {
    const vehicle = new Vehicle({ id: `bench-${spec}`, spec, player: false });
    const controller = new NextGenAIController(`bench-${spec}`, { track, aggression: 0.96, skill: 1.0 });

    // Initialize car on the racing line with flying speed and forward velocity vector
    const startPoint = controller.optimalEngine.sampleAtDistance(0, spec);
    vehicle.resetTo(track, 0, startPoint.lateral);
    const forward = { x: Math.sin(vehicle.yaw), z: Math.cos(vehicle.yaw) };
    const flySpeed = startPoint.targetSpeed * 0.95;
    vehicle.velocity.x = forward.x * flySpeed;
    vehicle.velocity.z = forward.z * flySpeed;
    vehicle.speed = flySpeed;
    vehicle.gear = 4;
    vehicle.rpm = 5800;
    vehicle.controls.throttle = 1.0;

    const laps = [];
    let lapTime = 0;
    let lastDistance = 0;
    let offTrackTime = 0;
    let maxLateralError = 0;
    const race = { raceTime: 0, elapsed: 0, phase: 'racing' };

    // Simulate 3 laps
    const maxSimTime = 240.0;
    for (let t = 0; t < maxSimTime; t += DT) {
      race.raceTime = t;
      race.elapsed = t;

      controller.update(vehicle, [vehicle], track, race, DT);
      vehicle.step(DT, track, true);

      lapTime += DT;

      if (vehicle.surface && (vehicle.surface.zone === 'grass' || vehicle.surface.zone === 'runoff')) {
        offTrackTime += DT;
      }

      const opt = controller.optimalEngine.sampleAtDistance(vehicle.distance, spec);
      const latErr = Math.abs((vehicle.surface?.lateral || 0) - opt.lateral);
      if (latErr > maxLateralError) maxLateralError = latErr;

      // Detect lap crossing (require > 30s so first initialization step doesn't false-trigger)
      if (lastDistance > track.length * 0.80 && vehicle.distance < track.length * 0.20 && lapTime > 30.0) {
        laps.push(lapTime);
        lapTime = 0;
        if (laps.length >= 2) break;
      }
      lastDistance = vehicle.distance;
    }

    const flyingLap = laps[0] || Infinity;
    console.log(`  [${name.padEnd(16)}] Flying Lap: ${mmss(flyingLap)} | Max Error: ${maxLateralError.toFixed(2)}m | Off-Track: ${offTrackTime.toFixed(2)}s`);

    assert.ok(Number.isFinite(flyingLap) && flyingLap < targetMaxLap,
      `Pace benchmark failed for ${name}: lap ${flyingLap.toFixed(2)}s > target ${targetMaxLap}s`);
    assert.ok(offTrackTime < 1.0, `Excessive off-track time: ${offTrackTime.toFixed(2)}s`);
  }
}

await testCircuitPace(HARBOR_RING, 'Harbor Ring (Flat Circuit)');
await testCircuitPace(ENDURANCE_PARK, 'Endurance Park (Elevation/Banked)');

console.log('\n>>> SUCCESS: ALL PACE BENCHMARKS PASSED COMPETITIVELY <<<');
