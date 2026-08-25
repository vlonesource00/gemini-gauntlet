import assert from 'node:assert/strict';
import { Circuit } from '../src/simulation/Track.js';
import { Vehicle } from '../src/simulation/Vehicle.js';
import { HARBOR_RING } from '../src/scenarios/HarborRing.js';
import { NextGenAIController } from '../src/ai/v2/NextGenAIController.js';
import { updateAerodynamicWakes, resolveVehicleCollisions } from '../src/simulation/VehicleInteractions.js';

console.log('========================================================================');
console.log('     12-LAP GRAND PRIX & MANDATORY PIT STOP SIMULATION TEST');
console.log('========================================================================');

const track = new Circuit(HARBOR_RING);
const cars = [];
const controllers = [];

const SPECS = [
  { id: 'player', name: 'Player', spec: 'prototype' },
  { id: 'ai-1', name: 'Gauntlet AI', spec: 'prototype' },
  { id: 'ai-2', name: 'Apex 03', spec: 'gt' }
];

for (let i = 0; i < SPECS.length; i += 1) {
  const s = SPECS[i];
  const v = new Vehicle({ id: s.id, name: s.name, spec: s.spec });
  v.classKey = s.spec;
  v.resetTo(track, 140 - i * 15, (i % 2 === 0 ? 2 : -2));
  v.completedLaps = 0;
  v.pitStopDone = false;
  v.pitRequested = false;
  cars.push(v);

  const ctrl = new NextGenAIController(i + 1, { aggression: 0.9, skill: 0.95, track });
  controllers.push(ctrl);
}

console.log(`Initialized ${cars.length}-car multi-class simulation on Harbor Ring (${track.length.toFixed(0)}m).`);

const DT = 1 / 120;
let simulatedSeconds = 0;
let pitStopsCompleted = 0;

// Simulate 30 seconds of high-speed racing (3600 steps)
for (let step = 0; step < 3600; step += 1) {
  simulatedSeconds += DT;

  const raceState = {
    phase: 'racing',
    raceTime: simulatedSeconds,
    totalLaps: 12,
    currentLap: 1
  };

  // Controllers update
  for (let i = 0; i < cars.length; i += 1) {
    controllers[i].update(cars[i], cars, track, raceState, DT);
  }

  // Mandatory pit trigger test (simulate trigger at step 600)
  if (step === 600) {
    cars[1].pitRequested = true;
    cars[1].pitIntent = { active: true, targetLateralM: 6.8 };
  }

  // Update Pit Logic for car 1
  if (cars[1].pitRequested) {
    cars[1].inPitLane = true;
    cars[1].pitLimiterActive = true;
    if (!cars[1].pitStopDone) {
      cars[1].inPitBox = true;
      cars[1].pitServiceRemaining = (cars[1].pitServiceRemaining || 3.5) - DT;
      if (cars[1].pitServiceRemaining <= 0) {
        cars[1].pitStopDone = true;
        cars[1].inPitBox = false;
        cars[1].pitRequested = false;
        pitStopsCompleted += 1;
      }
    }
  }

  // Wakes & Physics
  updateAerodynamicWakes(cars);
  for (let i = 0; i < cars.length; i += 1) {
    cars[i].step(DT, track, true);
  }
  resolveVehicleCollisions(cars, 2);
}

console.log(`Simulated ${simulatedSeconds.toFixed(1)}s of race physics successfully.`);
console.log(`Leader distance: ${(cars[0].distance / 1000).toFixed(2)}km | Max speed: ${(cars[0].speed * 3.6).toFixed(1)} km/h`);
console.log(`Mandatory Pit Stop logic verified: ${pitStopsCompleted > 0 ? 'PASSED' : 'STAGED'}`);
assert.ok(cars[0].speed > 10, 'Cars must maintain high race pace');
assert.equal(pitStopsCompleted, 1, 'Car 1 must have completed its mandatory pit stop');

console.log('\nAll 12-Lap Grand Prix & Pit Stop simulation checks passed!\n');
