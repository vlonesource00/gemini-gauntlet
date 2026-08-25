import { Circuit } from '../src/simulation/Track.js';
import { Vehicle } from '../src/simulation/Vehicle.js';
import { ResearchAIController } from '../src/ai/ResearchAIController.js';
import { NextGenAIController } from '../src/ai/v2/NextGenAIController.js';
import { HARBOR_RING } from '../src/scenarios/HarborRing.js';
import { ENDURANCE_PARK } from '../src/scenarios/EndurancePark.js';

const DT = 1 / 120;
const mmss = (t) => {
  if (!Number.isFinite(t)) return '  --   ';
  const m = Math.floor(t / 60);
  return `${m}:${(t - m * 60).toFixed(3).padStart(6, '0')}`;
};

function benchmarkControllerPace(circuitDef, ControllerClass, specKey) {
  const track = new Circuit(circuitDef);
  const vehicle = new Vehicle({ id: `bench-${specKey}`, spec: specKey, player: false });
  const controller = new ControllerClass(`bench-${specKey}`, { track, aggression: 0.96, skill: 1.0 });

  const startPoint = controller.optimalEngine?.sampleAtDistance?.(0, specKey) || { lateral: 0, targetSpeed: 45 };
  vehicle.resetTo(track, 0, startPoint.lateral);
  const forward = { x: Math.sin(vehicle.yaw), z: Math.cos(vehicle.yaw) };
  const flySpeed = (startPoint.targetSpeed || 45) * 0.95;
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
  let maxSpeed = 0;
  let minCornerSpeed = 999;
  const race = { raceTime: 0, elapsed: 0, phase: 'racing' };

  for (let t = 0; t < 240.0; t += DT) {
    race.raceTime = t;
    race.elapsed = t;

    controller.update(vehicle, [vehicle], track, race, DT);
    vehicle.step(DT, track, true);

    lapTime += DT;
    if (vehicle.speed > maxSpeed) maxSpeed = vehicle.speed;
    if (vehicle.speed < minCornerSpeed && t > 2.0) minCornerSpeed = vehicle.speed;

    if (vehicle.surface && (vehicle.surface.zone === 'grass' || vehicle.surface.zone === 'runoff')) {
      offTrackTime += DT;
    }

    if (lastDistance > track.length * 0.80 && vehicle.distance < track.length * 0.20 && lapTime > 30.0) {
      laps.push(lapTime);
      lapTime = 0;
      if (laps.length >= 2) break;
    }
    lastDistance = vehicle.distance;
  }

  return {
    flyingLap: laps[0] || Infinity,
    offTrackTime,
    maxSpeedKph: maxSpeed * 3.6,
    minCornerSpeedKph: minCornerSpeed * 3.6
  };
}

function runHeadToHeadRace(circuitDef, specKey) {
  const track = new Circuit(circuitDef);
  // Car 1: V2 NextGen (starting P2 at 25m)
  const v2Car = new Vehicle({ id: 'v2-apex', spec: specKey, player: false });
  const v2Ctrl = new NextGenAIController('v2-apex', { track, aggression: 0.98, skill: 1.0 });

  // Car 2: V1 Original (starting P1 at 40m)
  const v1Car = new Vehicle({ id: 'v1-orig', spec: specKey, player: false });
  const v1Ctrl = new ResearchAIController('v1-orig', { track, aggression: 0.90, skill: 1.0 });

  v2Car.resetTo(track, 25.0, 0);
  v1Car.resetTo(track, 40.0, 0);
  v2Car.speed = 10.0;
  v1Car.speed = 10.0;

  const vehicles = [v2Car, v1Car];
  let v2OvertookAt = null;
  let contacts = 0;
  let deepOverlaps = 0;
  const race = { raceTime: 0, elapsed: 0, phase: 'racing' };

  for (let t = 0; t < 45.0; t += DT) {
    race.raceTime = t;
    race.elapsed = t;

    v2Ctrl.update(v2Car, vehicles, track, race, DT);
    v1Ctrl.update(v1Car, vehicles, track, race, DT);

    v2Car.step(DT, track, true);
    v1Car.step(DT, track, true);

    const distDelta = (v2Car.distance - v1Car.distance + track.length) % track.length;
    const isV2Ahead = distDelta > 0 && distDelta < track.length * 0.5;

    if (isV2Ahead && v2OvertookAt === null && t > 1.0) {
      v2OvertookAt = t;
    }

    const dx = v2Car.position.x - v1Car.position.x;
    const dz = v2Car.position.z - v1Car.position.z;
    const dist2 = dx * dx + dz * dz;
    if (dist2 < 4.0) contacts++;
    if (dist2 < 1.0) deepOverlaps++;
  }

  return {
    v2Distance: v2Car.distance,
    v1Distance: v1Car.distance,
    v2OvertookAt,
    contacts,
    deepOverlaps
  };
}

console.log('========================================================================');
console.log('     GENERATIONAL BENCHMARK: ORIGINAL (V1) VS NEXT-GEN APEX (V3)');
console.log('========================================================================\n');

for (const trackDef of [
  { def: HARBOR_RING, name: 'Harbor Ring (Flat)' },
  { def: ENDURANCE_PARK, name: 'Endurance Park (Banked/Elevated)' }
]) {
  console.log(`--- Circuit: ${trackDef.name} ---`);
  for (const spec of ['prototype', 'gt', 'touring']) {
    const v1Res = benchmarkControllerPace(trackDef.def, ResearchAIController, spec);
    const v2Res = benchmarkControllerPace(trackDef.def, NextGenAIController, spec);
    const deltaS = v1Res.flyingLap - v2Res.flyingLap;
    const pctImprovement = ((deltaS / v1Res.flyingLap) * 100).toFixed(1);

    console.log(`  [${spec.toUpperCase().padEnd(9)}] V1: ${mmss(v1Res.flyingLap)} (Top: ${v1Res.maxSpeedKph.toFixed(0)}kph) | V3: ${mmss(v2Res.flyingLap)} (Top: ${v2Res.maxSpeedKph.toFixed(0)}kph) | Delta: -${deltaS.toFixed(2)}s (-${pctImprovement}%) | Off-Track: ${v2Res.offTrackTime.toFixed(2)}s`);
  }

  const h2h = runHeadToHeadRace(trackDef.def, 'prototype');
  console.log(`  [HEAD-TO-HEAD] V3 vs V1 Race: V3 Overtook V1 at t=${h2h.v2OvertookAt?.toFixed(2)}s | Final Lead: ${(h2h.v2Distance - h2h.v1Distance).toFixed(1)}m | Contacts: ${h2h.contacts} | Deep Overlaps: ${h2h.deepOverlaps}\n`);
}
