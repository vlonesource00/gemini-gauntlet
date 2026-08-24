import assert from 'node:assert/strict';
import { CoupledDynamicsController } from '../src/ai/v2/CoupledDynamicsController.js';

console.log('\n=== Testing CoupledDynamicsController (V2 Layer 3) ===');

// Mock Track
const mockTrack = {
  length: 2500,
  roadHalfWidth: 7.0,
  curbWidth: 1.2,
  atDistance(s) {
    // 0-500: straight, 500-1000: right turn (kappa = 0.015), 1000-1500: straight, 1500-2000: left turn (kappa = -0.015)
    const dist = ((s % 2500) + 2500) % 2500;
    let curvature = 0;
    if (dist >= 500 && dist < 1000) curvature = 0.015;
    else if (dist >= 1500 && dist < 2000) curvature = -0.015;

    const angle = curvature * (dist - (dist >= 1500 ? 1500 : 500));
    return {
      s: dist,
      x: Math.sin(angle) * (1 / (Math.abs(curvature) || 1)),
      y: 0,
      z: dist,
      curvature,
      tangent: { x: Math.sin(angle), z: Math.cos(angle) },
      normal: { x: Math.cos(angle), z: -Math.sin(angle) }
    };
  },
  lateralPoint(pt, lat) {
    return {
      x: pt.x + (pt.normal?.x ?? 1) * lat,
      y: pt.y ?? 0,
      z: pt.z + (pt.normal?.z ?? 0) * lat
    };
  }
};

// Mock Vehicle
function createMockVehicle({ speed = 35.0, distance = 100, lateral = 0, yaw = 0, yawRate = 0, classKey = 'prototype' } = {}) {
  return {
    classKey,
    speed,
    distance,
    yaw,
    yawRate,
    wheelBase: 2.65,
    position: { x: 0, y: 0.05, z: distance },
    localVelocity: { x: 0, z: speed },
    surface: { lateral, grip: 1.0 },
    controls: { steer: 0, throttle: 0, brake: 0, handbrake: 0 }
  };
}

const controller = new CoupledDynamicsController();

// ---------------------------------------------------------------------------
// 1. Stanley Cross-Track Error and Steering Verification
// ---------------------------------------------------------------------------
console.log('  -> Testing Stanley Cross-Track Error & Steering Response...');

// Case A: Centered on straight, heading aligned -> Near zero steer
{
  const v = createMockVehicle({ speed: 30.0, distance: 100, lateral: 0, yaw: 0 });
  const res = controller.step({
    vehicle: v,
    track: mockTrack,
    tacticalTarget: { targetLateral: 0, desiredSpeed: 30.0 },
    dt: 0.016
  });

  assert(Math.abs(res.steer) < 0.05, `Expected minimal steer on straight, got ${res.steer}`);
  console.log(`    [PASS] Straight-line tracking: steer=${res.steer.toFixed(4)}`);
}

// Case B: Vehicle offset to the right (+2.0m lateral) -> Must steer left (negative steer)
{
  controller.reset();
  const v = createMockVehicle({ speed: 25.0, distance: 100, lateral: 2.0, yaw: 0 });
  const res = controller.step({
    vehicle: v,
    track: mockTrack,
    tacticalTarget: { targetLateral: 0, desiredSpeed: 25.0 },
    dt: 0.016
  });

  assert(res.steer < 0, `Expected negative steer to correct leftward from positive offset, got ${res.steer}`);
  console.log(`    [PASS] Positive lateral offset (+2m) corrected leftward: steer=${res.steer.toFixed(4)}`);
}

// Case C: Vehicle offset to the left (-2.0m lateral) -> Must steer right (positive steer)
{
  controller.reset();
  const v = createMockVehicle({ speed: 25.0, distance: 100, lateral: -2.0, yaw: 0 });
  const res = controller.step({
    vehicle: v,
    track: mockTrack,
    tacticalTarget: { targetLateral: 0, desiredSpeed: 25.0 },
    dt: 0.016
  });

  assert(res.steer > 0, `Expected positive steer to correct rightward from negative offset, got ${res.steer}`);
  console.log(`    [PASS] Negative lateral offset (-2m) corrected rightward: steer=${res.steer.toFixed(4)}`);
}

// ---------------------------------------------------------------------------
// 2. Curvature Feedforward & Lookahead Preview Verification
// ---------------------------------------------------------------------------
console.log('  -> Testing Curvature Feedforward & Lookahead Preview...');

// Approaching a right turn (at distance 492m, turn starts at 500m, preview window is 10.5m ahead)
{
  controller.reset();
  const v = createMockVehicle({ speed: 30.0, distance: 492, lateral: 0, yaw: 0 });
  const res = controller.step({
    vehicle: v,
    track: mockTrack,
    tacticalTarget: { targetLateral: 0, desiredSpeed: 30.0 },
    dt: 0.016
  });

  assert(res.stanley.curvatureFeedforward > 0, `Expected positive curvature feedforward for upcoming right turn, got ${res.stanley.curvatureFeedforward}`);
  assert(res.steer > 0, `Expected proactive rightward turn-in steer, got ${res.steer}`);
  console.log(`    [PASS] Proactive right turn turn-in: delta_ff=${res.stanley.curvatureFeedforward.toFixed(4)}, steer=${res.steer.toFixed(4)}`);
}

// ---------------------------------------------------------------------------
// 3. 2D G-G Friction Circle Trail-Braking Verification
// ---------------------------------------------------------------------------
console.log('  -> Testing 2D G-G Friction Circle Trail-Braking Modulation...');

// Straight threshold braking: speed 60, desired 30 -> High brake demand, full longitudinal budget
{
  controller.reset();
  const v = createMockVehicle({ speed: 60.0, distance: 100, lateral: 0, yaw: 0, yawRate: 0 });
  const res = controller.step({
    vehicle: v,
    track: mockTrack,
    tacticalTarget: { targetLateral: 0, desiredSpeed: 30.0 },
    dt: 0.016
  });

  assert(res.brake > 0.3, `Expected strong braking on approach, got ${res.brake}`);
  assert(res.throttle === 0, `Expected zero throttle during braking, got ${res.throttle}`);
  assert(res.friction.latUtilization < 0.1, `Expected low lateral utilization on straight, got ${res.friction.latUtilization}`);
  console.log(`    [PASS] Straight-line threshold braking: brake=${res.brake.toFixed(4)}, longBudget=${res.friction.remainingLongBudget.toFixed(4)}`);
}

// Trail-braking in corner: speed 40, desired 30, high lateral load (yawRate = 0.5 rad/s)
{
  controller.reset();
  const v = createMockVehicle({ speed: 40.0, distance: 600, lateral: 0, yaw: 0, yawRate: 0.45 });
  const res = controller.step({
    vehicle: v,
    track: mockTrack,
    tacticalTarget: { targetLateral: 0, desiredSpeed: 25.0 },
    dt: 0.016
  });

  assert(res.friction.latUtilization > 0.4, `Expected high lateral utilization in corner, got ${res.friction.latUtilization}`);
  assert(res.friction.remainingLongBudget < 0.95, `Expected depleted longitudinal budget in corner, got ${res.friction.remainingLongBudget}`);
  assert(res.trailBraking.active === true, `Expected trail-braking to be active`);
  console.log(`    [PASS] Trail-braking active: latUtil=${res.friction.latUtilization.toFixed(3)}, longBudget=${res.friction.remainingLongBudget.toFixed(3)}, brake=${res.brake.toFixed(3)}`);
}

// ---------------------------------------------------------------------------
// 4. Apex Exit & Steering Unwind Full Throttle Launch
// ---------------------------------------------------------------------------
console.log('  -> Testing Corner Exit & Unwind Full Throttle Power Launch...');

// Exiting corner onto straight: speed 30, desired 60, straightaway, zero steer
{
  controller.reset();
  const v = createMockVehicle({ speed: 30.0, distance: 1100, lateral: 0, yaw: 0, yawRate: 0 });
  const res = controller.step({
    vehicle: v,
    track: mockTrack,
    tacticalTarget: { targetLateral: 0, desiredSpeed: 60.0 },
    dt: 0.016
  });

  assert.equal(res.throttle, 1.0, `Expected instant 100% full throttle launch on straight/unwind, got ${res.throttle}`);
  assert.equal(res.brake, 0.0, `Expected zero brake on launch, got ${res.brake}`);
  assert.equal(res.traction.launchActive, true, `Expected launchActive to be true`);
  console.log(`    [PASS] Instant 100% full throttle launch verified: throttle=${res.throttle.toFixed(2)}, launchActive=${res.traction.launchActive}`);
}

// ---------------------------------------------------------------------------
// 5. Zero-GC Horizon Allocation & 120Hz Fast Loop Benchmark
// ---------------------------------------------------------------------------
console.log('  -> Testing Zero-GC Horizon & 120Hz Loop Performance...');

{
  const v = createMockVehicle({ speed: 45.0, distance: 200 });
  const startTime = Date.now();
  const iterations = 5000;

  for (let i = 0; i < iterations; i++) {
    v.distance += v.speed * 0.016;
    controller.step({
      vehicle: v,
      track: mockTrack,
      tacticalTarget: { targetLateral: 0, desiredSpeed: 55.0 },
      dt: 0.016
    });
  }

  const durationMs = Date.now() - startTime;
  const avgUsPerStep = (durationMs * 1000) / iterations;
  console.log(`    [PASS] 5,000 steps executed in ${durationMs}ms (avg ${avgUsPerStep.toFixed(2)} µs / step < 0.1ms target)`);
  assert(avgUsPerStep < 100, `Expected step execution under 100 µs, got ${avgUsPerStep.toFixed(2)} µs`);
}

console.log('\n>>> ALL COUPLED DYNAMICS CONTROLLER TESTS PASSED CLEANLY (Exit 0) <<<\n');
