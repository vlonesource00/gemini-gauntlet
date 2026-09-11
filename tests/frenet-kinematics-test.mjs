import assert from 'node:assert/strict';

console.log('=== Running Frenet Offset Kinematics & Acceleration Test Suite ===');

function computeFrenetDerivatives({ vx, vz, ax, az, currentLateral, trackNormal, trackTangent, curvature, turnSign = 1 }) {
  const qDotMeasured = vx * trackNormal.x + vz * trackNormal.z;
  const sDotMeasured = vx * trackTangent.x + vz * trackTangent.z;

  const kappaTrack = turnSign * curvature;
  const oneMinusKappaQ = 1.0 - kappaTrack * currentLateral;
  const denom = Math.abs(oneMinusKappaQ) < 0.1 ? (oneMinusKappaQ >= 0 ? 0.1 : -0.1) : oneMinusKappaQ;
  const sDot = sDotMeasured / denom;

  const aDotN = ax * trackNormal.x + az * trackNormal.z;
  const qDDotMeasured = aDotN - denom * kappaTrack * (sDot * sDot);

  return { qDot: qDotMeasured, sDot, aDotN, qDDot: qDDotMeasured };
}

const EPS = 1e-9;

// CASE 1: Straight Track (kappa = 0)
{
  console.log('  -> CASE 1: Straight Track (kappa = 0)...');
  const tangent = { x: 0, z: 1 };
  const normal = { x: -1, z: 0 };
  const res = computeFrenetDerivatives({
    vx: -2.5,
    vz: 30.0,
    ax: 1.2,
    az: 4.0,
    currentLateral: 1.5,
    trackNormal: normal,
    trackTangent: tangent,
    curvature: 0,
    turnSign: 1
  });

  const expectedQDot = -2.5 * (-1) + 30.0 * 0; // +2.5
  const expectedQDDot = 1.2 * (-1) + 4.0 * 0;  // -1.2

  assert.ok(Math.abs(res.qDot - expectedQDot) < EPS);
  assert.ok(Math.abs(res.qDDot - expectedQDDot) < EPS);
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

  const res = computeFrenetDerivatives({
    vx: speed,
    vz: 0,
    ax: 0,
    az: aNorm,
    currentLateral: 0.0,
    trackNormal: normal,
    trackTangent: tangent,
    curvature: kappa,
    turnSign: 1
  });

  assert.ok(Math.abs(res.qDot) < EPS);
  assert.ok(Math.abs(res.qDDot) < EPS);
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

  const res = computeFrenetDerivatives({
    vx: vOffset,
    vz: 0,
    ax: 0,
    az: aNormOffset,
    currentLateral: q,
    trackNormal: normal,
    trackTangent: tangent,
    curvature: kappa,
    turnSign: 1
  });

  assert.ok(Math.abs(res.qDot) < EPS);
  assert.ok(Math.abs(res.qDDot) < EPS);
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
  const resLeft = computeFrenetDerivatives({
    vx: -2.0,
    vz: 30.0,
    ax: -3.0,
    az: 0.0,
    currentLateral: 0.0,
    trackNormal: normal,
    trackTangent: tangent,
    curvature: 0,
    turnSign: 1
  });

  assert.ok(resLeft.qDot > 0, 'qDot must be positive when moving in +N direction');
  assert.ok(resLeft.qDDot > 0, 'qDDot must be positive when accelerating in +N direction');

  // Vehicle moving towards negative lateral (-normal, so +X world direction)
  const resRight = computeFrenetDerivatives({
    vx: 2.0,
    vz: 30.0,
    ax: 3.0,
    az: 0.0,
    currentLateral: 0.0,
    trackNormal: normal,
    trackTangent: tangent,
    curvature: 0,
    turnSign: 1
  });

  assert.ok(resRight.qDot < 0, 'qDot must be negative when moving in -N direction');
  assert.ok(resRight.qDDot < 0, 'qDDot must be negative when accelerating in -N direction');
  console.log(`     [PASS] Directional signs verified: Left lane transition (+qDot=${resLeft.qDot}, +qDDot=${resLeft.qDDot}), Right lane transition (-qDot=${resRight.qDot}, -qDDot=${resRight.qDDot})`);
}

console.log('\n>>> ALL FRENET KINEMATIC TESTS PASSED CLEANLY (Exit 0) <<<');
