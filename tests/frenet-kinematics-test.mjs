import assert from 'node:assert/strict';
import { computeFrenetKinematics, FrenetLatticePlanner } from '../src/ai/FrenetLatticePlanner.js';

console.log('=== Running Frenet Offset Kinematics & Acceleration Test Suite ===');

const EPS = 1e-9;

// CASE 1: Straight Track (kappa = 0)
{
  console.log('  -> CASE 1: Straight Track (kappa = 0)...');
  const tangent = { x: 0, z: 1 };
  const normal = { x: -1, z: 0 };
  const res = computeFrenetKinematics({
    velocity: { x: -2.5, z: 30.0 },
    acceleration: { x: 1.2, z: 4.0 },
    lateral: 1.5,
    tangent,
    normal,
    curvature: 0,
    turnSign: 1
  });

  const expectedQDot = -2.5 * (-1) + 30.0 * 0; // +2.5
  const expectedQDDot = 1.2 * (-1) + 4.0 * 0;  // -1.2

  assert.ok(Math.abs(res.qDot - expectedQDot) < EPS, `Expected qDot ${expectedQDot}, got ${res.qDot}`);
  assert.ok(Math.abs(res.qDDot - expectedQDDot) < EPS, `Expected qDDot ${expectedQDDot}, got ${res.qDDot}`);
  console.log(`     [PASS] qDot=${res.qDot.toFixed(3)} == v·N, qDDot=${res.qDDot.toFixed(3)} == a·N`);
}

// CASE 2: Vehicle following centerline of constant-radius corner (q = 0)
{
  console.log('  -> CASE 2: Constant-Radius Corner on Centerline (q = 0)...');
  const R = 50.0;
  const kappa = 1.0 / R; // 0.02
  const speed = 25.0;    // m/s
  const tangent = { x: 1, z: 0 };
  const normal = { x: 0, z: 1 }; // centripetal direction

  // Steady cornering: centripetal acceleration a = v^2 / R = kappa * v^2 along normal
  const aNorm = kappa * speed * speed; // 12.5 m/s^2

  const res = computeFrenetKinematics({
    velocity: { x: speed, z: 0 },
    acceleration: { x: 0, z: aNorm },
    lateral: 0.0,
    tangent,
    normal,
    curvature: kappa,
    turnSign: 1
  });

  assert.ok(Math.abs(res.qDot) < EPS, `Expected qDot ≈ 0, got ${res.qDot}`);
  assert.ok(Math.abs(res.qDDot) < EPS, `Expected qDDot ≈ 0, got ${res.qDDot}`);
  console.log(`     [PASS] World centripetal a·N=${res.aDotN.toFixed(2)} m/s², qDDot=${res.qDDot.toExponential(2)} ≈ 0`);
}

// CASE 3: Vehicle following constant parallel offset (q != 0)
{
  console.log('  -> CASE 3: Constant-Radius Corner on Parallel Offset (q = 2.0m)...');
  const R = 50.0;
  const kappa = 1.0 / R; // 0.02
  const q = 2.0;         // inside offset, effective radius R_eff = R - q = 48.0
  const sDot = 25.0;     // centerline speed rate

  // Offset speed v = (1 - kappa * q) * sDot = (1 - 0.04) * 25 = 24.0 m/s
  const vOffset = (1.0 - kappa * q) * sDot;
  // Offset curvature kappa_eff = kappa / (1 - kappa * q) = 1 / 48.0
  // Centripetal acceleration along normal: a = v^2 * kappa_eff = (1 - kappa * q) * kappa * sDot^2
  const aNormOffset = (1.0 - kappa * q) * kappa * sDot * sDot;

  const tangent = { x: 1, z: 0 };
  const normal = { x: 0, z: 1 };

  const res = computeFrenetKinematics({
    velocity: { x: vOffset, z: 0 },
    acceleration: { x: 0, z: aNormOffset },
    lateral: q,
    tangent,
    normal,
    curvature: kappa,
    turnSign: 1
  });

  assert.ok(Math.abs(res.qDot) < EPS, `Expected qDot ≈ 0, got ${res.qDot}`);
  assert.ok(Math.abs(res.qDDot) < EPS, `Expected qDDot < 1e-9, got ${res.qDDot}`);
  console.log(`     [PASS] Offset path (q=2m): a·N=${res.aDotN.toFixed(2)} m/s², qDDot=${res.qDDot.toExponential(2)} ≈ 0 (exact kinematic cancellation)`);
}

// CASE 4: Left / Right Lateral Transition
{
  console.log('  -> CASE 4: Dynamic Lateral Lane Transition...');
  const tangent = { x: 0, z: 1 };
  const normal = { x: -1, z: 0 };

  // Vehicle moving towards positive lateral (+normal, so -X world direction)
  // vx = -2.0 -> qDot = +2.0
  // Accelerating laterally: ax = -3.0 -> aDotN = +3.0 -> qDDot = +3.0
  const resLeft = computeFrenetKinematics({
    velocity: { x: -2.0, z: 30.0 },
    acceleration: { x: -3.0, z: 0.0 },
    lateral: 0.0,
    tangent,
    normal,
    curvature: 0,
    turnSign: 1
  });

  assert.ok(resLeft.qDot > 0, 'qDot must be positive when moving in +N direction');
  assert.ok(resLeft.qDDot > 0, 'qDDot must be positive when accelerating in +N direction');

  // Vehicle moving towards negative lateral (-normal, so +X world direction)
  const resRight = computeFrenetKinematics({
    velocity: { x: 2.0, z: 30.0 },
    acceleration: { x: 3.0, z: 0.0 },
    lateral: 0.0,
    tangent,
    normal,
    curvature: 0,
    turnSign: 1
  });

  assert.ok(resRight.qDot < 0, 'qDot must be negative when moving in -N direction');
  assert.ok(resRight.qDDot < 0, 'qDDot must be negative when accelerating in -N direction');
  console.log(`     [PASS] Directional signs verified: Left lane transition (+qDot=${resLeft.qDot}, +qDDot=${resLeft.qDDot}), Right lane transition (-qDot=${resRight.qDot}, -qDDot=${resRight.qDDot})`);
}

// CASE 5: Live FrenetLatticePlanner.plan() Integration Test
{
  console.log('  -> CASE 5: Live FrenetLatticePlanner.plan() Integration on Parallel Offset...');
  const R = 50.0;
  const kappa = 1.0 / R; // 0.02
  const q = 2.0;
  const sDot = 25.0;
  const vOffset = (1.0 - kappa * q) * sDot; // 24.0 m/s
  const aNormOffset = (1.0 - kappa * q) * kappa * sDot * sDot; // 12.0 m/s^2

  const refPoint = {
    s: 100.0,
    x: 0,
    y: 0,
    z: 100.0,
    tangent: { x: 1, z: 0 },
    normal: { x: 0, z: 1 },
    curvature: kappa,
    turnSign: 1,
    turnStrength: 0.22,
    curbSide: 1,
    grade: 0,
    bank: 0
  };

  const syntheticTrack = {
    length: 2000,
    roadHalfWidth: 10.0,
    curbWidth: 1.5,
    atDistance(s) {
      return { ...refPoint, s };
    },
    lateralPoint(p, lat = 0, lift = 0) {
      return {
        x: p.x + p.normal.x * lat,
        y: lift,
        z: p.z + p.normal.z * lat
      };
    },
    planningLateralLimit(distance, side) {
      return 9.0;
    }
  };

  const syntheticVehicle = {
    distance: 100.0,
    speed: vOffset,
    velocity: { x: vOffset, y: 0, z: 0 },
    acceleration: { x: 0, y: 0, z: aNormOffset },
    yaw: Math.PI / 2, // facing +X along tangent
    yawRate: vOffset / (R - q),
    surface: { lateral: q, zone: 'road' }
  };

  const planner = new FrenetLatticePlanner({ pointCount: 24, horizonS: 3.0 });
  const plan = planner.plan({
    vehicle: syntheticVehicle,
    track: syntheticTrack,
    desiredOffset: q,
    previousPlan: null,
    dtSinceLastPlan: 0.5
  });

  assert.ok(plan != null, 'Plan must not be null');
  assert.ok(typeof plan.startA === 'number', 'plan.startA must be a number');
  assert.ok(Math.abs(plan.startA) < EPS, `Expected planner startA ≈ 0, got ${plan.startA}`);
  assert.ok(Math.abs(plan.startV) < EPS, `Expected planner startV ≈ 0, got ${plan.startV}`);

  console.log(`     [PASS] Live planner verified: startV=${plan.startV.toExponential(2)}, startA=${plan.startA.toExponential(2)} ≈ 0`);
}

console.log('\n>>> ALL FRENET KINEMATIC TESTS PASSED CLEANLY (Exit 0) <<<');
