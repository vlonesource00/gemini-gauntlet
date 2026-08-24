import assert from 'node:assert/strict';
import { Circuit } from '../src/simulation/Track.js';
import { Vehicle } from '../src/simulation/Vehicle.js';
import { ResearchAIController } from '../src/ai/ResearchAIController.js';
import { HARBOR_RING } from '../src/scenarios/HarborRing.js';
import { estimateScenarioLength } from '../src/scenarios/EndurancePark.js';

const track = new Circuit(HARBOR_RING);
const estimatedLength = estimateScenarioLength(HARBOR_RING, 34);
assert.ok(estimatedLength >= 2400 && estimatedLength <= 3600,
  `Harbor Ring length outside design target: ${estimatedLength.toFixed(1)} m`);
assert.ok(Math.abs(track.length - estimatedLength) < 8, 'Circuit must consume Harbor Ring geometry exactly');
assert.ok(HARBOR_RING.sectors.some((sector) => sector.character === 'heavy-braking'),
  'flat circuit must include a heavy-braking overtaking zone');
assert.ok(HARBOR_RING.sectors.filter((sector) => sector.character === 'technical').length >= 2,
  'flat circuit must include two technical sectors');

for (const [index, point] of track.samples.entries()) {
  assert.ok(Math.abs(point.y) < 1e-9, `sample ${index} has elevation`);
  assert.ok(Math.abs(point.grade) < 1e-9, `sample ${index} has grade`);
  assert.ok(Math.abs(point.bank) < 1e-9, `sample ${index} has banking`);
  const left = track.lateralPoint(point, -track.roadHalfWidth);
  const right = track.lateralPoint(point, track.roadHalfWidth);
  assert.ok(Math.abs(left.y - right.y) < 1e-9, `sample ${index} road surface is not flat`);
}

// Exercise perception, tactics, trajectory selection, steering and pace for
// all three vehicle classes on the new Harbor Ring geometry.
const vehicles = [
  new Vehicle({ id: 'harbor-prototype', spec: 'prototype' }),
  new Vehicle({ id: 'harbor-gt', spec: 'gt' }),
  new Vehicle({ id: 'harbor-touring', spec: 'touring' })
];
const controllers = vehicles.map((_, index) => new ResearchAIController(index + 1));
vehicles.forEach((vehicle, index) => vehicle.resetTo(track, 180 - index * 22, index % 2 ? 2.5 : -2.5));
const startDistances = vehicles.map((vehicle) => vehicle.distance);
const dt = 1 / 120;
const race = { raceTime: 10, elapsed: 10, phase: 'racing' };

for (let step = 0; step < 12 / dt; step += 1) {
  for (let index = 0; index < vehicles.length; index += 1) {
    controllers[index].update(vehicles[index], vehicles, track, race, dt);
  }
  for (const vehicle of vehicles) vehicle.step(dt, track, true);
}

vehicles.forEach((vehicle, index) => {
  const travelled = ((vehicle.distance - startDistances[index]) % track.length + track.length) % track.length;
  assert.ok(travelled > 45, `${vehicle.id} did not make meaningful progress on Harbor Ring`);
  assert.ok(Number.isFinite(vehicle.controls.steer) && Number.isFinite(vehicle.controls.throttle)
    && Number.isFinite(vehicle.controls.brake), `${vehicle.id} produced non-finite controls`);
  assert.equal(controllers[index].trajectoryPlan?.roadLegal, true,
    `${vehicle.id} selected an illegal final trajectory`);
});

console.log(`Harbor Ring contract passed: ${track.length.toFixed(1)} m, zero elevation/bank, ${vehicles.length} AI classes operational.`);
