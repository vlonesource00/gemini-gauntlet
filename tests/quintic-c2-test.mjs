import assert from 'node:assert/strict';
import { QuinticPolynomial, TwoStageSpline } from '../src/ai/FrenetLatticePlanner.js';

console.log('=== [1/3] QuinticPolynomial Exact Boundary & Continuity Tests ===');

const HAND_CASES = [
  { name: 'CASE A (Rest to rest)', q0: 0, v0: 0, a0: 0, q1: 4, v1: 0, a1: 0, T: 1.5 },
  { name: 'CASE B (Nonzero derivatives)', q0: 1.2, v0: 2.0, a0: -3.0, q1: -2.4, v1: -0.5, a1: 1.5, T: 1.2 },
  { name: 'CASE C (Opposing velocity & acceleration)', q0: -3.0, v0: -4.0, a0: 6.0, q1: 2.0, v1: 3.0, a1: -5.0, T: 2.0 }
];

const EPS = 1e-6;

for (const c of HAND_CASES) {
  const p = new QuinticPolynomial(c.q0, c.v0, c.a0, c.q1, c.v1, c.a1, c.T);

  // 1. Exact start boundary
  assert.equal(p.eval(0), c.q0, `${c.name}: eval(0) == q0`);
  assert.equal(p.deriv(0), c.v0, `${c.name}: deriv(0) == v0`);
  assert.equal(p.accel(0), c.a0, `${c.name}: accel(0) == a0`);

  // 2. Exact end boundary
  assert.equal(p.eval(c.T), c.q1, `${c.name}: eval(T) == q1`);
  assert.equal(p.deriv(c.T), c.v1, `${c.name}: deriv(T) == v1`);
  assert.equal(p.accel(c.T), c.a1, `${c.name}: accel(T) == a1`);

  // 3. Immediately before endpoint (T - epsilon)
  const qPre = p.eval(c.T - EPS);
  const vPre = p.deriv(c.T - EPS);
  const aPre = p.accel(c.T - EPS);

  const dq = Math.abs(qPre - c.q1);
  const dv = Math.abs(vPre - c.v1);
  const da = Math.abs(aPre - c.a1);

  assert.ok(dq < 1e-4, `${c.name}: |q(T-eps) - q1| = ${dq} < 1e-4`);
  assert.ok(dv < 1e-3, `${c.name}: |v(T-eps) - v1| = ${dv} < 1e-3`);
  assert.ok(da < 1e-2, `${c.name}: |a(T-eps) - a1| = ${da} < 1e-2`);

  // 4. Central difference consistency check inside (0, T)
  for (let step = 1; step < 10; step++) {
    const t = (c.T * step) / 10;
    const h = 1e-5;
    const vNum = (p.eval(t + h) - p.eval(t - h)) / (2 * h);
    const aNum = (p.eval(t + h) - 2 * p.eval(t) + p.eval(t - h)) / (h * h);

    assert.ok(Math.abs(p.deriv(t) - vNum) < 1e-4, `${c.name} at t=${t.toFixed(2)}: analytical v vs numerical v`);
    assert.ok(Math.abs(p.accel(t) - aNum) < 1e-3, `${c.name} at t=${t.toFixed(2)}: analytical a vs numerical a`);
  }

  console.log(`  [PASS] ${c.name}: dq(T-eps)=${dq.toExponential(2)}, dv(T-eps)=${dv.toExponential(2)}, da(T-eps)=${da.toExponential(2)}`);
}

console.log('\n=== [2/3] TwoStageSpline C2 Joint Continuity Tests ===');

const MANEUVER_CASES = [
  {
    name: 'SLINGSHOT (Lane sweep and settle)',
    s1: { q0: -1.5, v0: 0.8, a0: -2.0, q1: 2.2, v1: 0, a1: 0, T: 1.1 },
    s2: { q0: 2.2, v0: 0, a0: 0, q1: 2.2, v1: 0, a1: 0, T: 1.2 }
  },
  {
    name: 'SWITCHBACK (Cut back across corner exit)',
    s1: { q0: 1.0, v0: -1.2, a0: 3.5, q1: -2.5, v1: 0, a1: 0, T: 1.0 },
    s2: { q0: -2.5, v0: 0, a0: 0, q1: 1.8, v1: 0, a1: 0, T: 1.3 }
  },
  {
    name: 'SOFT_ABORT (Hold current offset then blend back)',
    s1: { q0: 0.5, v0: 1.5, a0: -4.0, q1: 0.5, v1: 0, a1: 0, T: 0.5 },
    s2: { q0: 0.5, v0: 0, a0: 0, q1: -1.0, v1: 0, a1: 0, T: 1.5 }
  }
];

let maxJointDq = 0;
let maxJointDv = 0;
let maxJointDa = 0;

for (const m of MANEUVER_CASES) {
  const p1 = new QuinticPolynomial(m.s1.q0, m.s1.v0, m.s1.a0, m.s1.q1, m.s1.v1, m.s1.a1, m.s1.T);
  const p2 = new QuinticPolynomial(m.s2.q0, m.s2.v0, m.s2.a0, m.s2.q1, m.s2.v1, m.s2.a1, m.s2.T);
  const spline = new TwoStageSpline(p1, p2);

  const T1 = m.s1.T;
  const qLeft = spline.eval(T1 - EPS);
  const qRight = spline.eval(T1 + EPS);
  const vLeft = spline.deriv(T1 - EPS);
  const vRight = spline.deriv(T1 + EPS);
  const aLeft = spline.accel(T1 - EPS);
  const aRight = spline.accel(T1 + EPS);

  const dq = Math.abs(qRight - qLeft);
  const dv = Math.abs(vRight - vLeft);
  const da = Math.abs(aRight - aLeft);

  maxJointDq = Math.max(maxJointDq, dq);
  maxJointDv = Math.max(maxJointDv, dv);
  maxJointDa = Math.max(maxJointDa, da);

  assert.ok(dq < 1e-4, `${m.name}: joint dq = ${dq} < 1e-4`);
  assert.ok(dv < 1e-3, `${m.name}: joint dv = ${dv} < 1e-3`);
  assert.ok(da < 1e-2, `${m.name}: joint da = ${da} < 1e-2`);

  console.log(`  [PASS] ${m.name}: joint Δq=${dq.toExponential(2)}, Δv=${dv.toExponential(2)}, Δa=${da.toExponential(2)}`);
}

console.log('\n=== [3/3] QuinticPolynomial Fuzz Property Suite (5,000 Cases) ===');

let fuzzMaxDq = 0;
let fuzzMaxDv = 0;
let fuzzMaxDa = 0;
const FUZZ_COUNT = 5000;

function seededRandom(seed) {
  let s = seed % 2147483647;
  if (s <= 0) s += 2147483646;
  return () => {
    s = (s * 16807) % 2147483647;
    return (s - 1) / 2147483646;
  };
}

const rand = seededRandom(42);

for (let i = 0; i < FUZZ_COUNT; i++) {
  const q0 = (rand() - 0.5) * 16.0; // [-8, +8] m
  const q1 = (rand() - 0.5) * 16.0;
  const v0 = (rand() - 0.5) * 10.0; // [-5, +5] m/s
  const v1 = (rand() - 0.5) * 10.0;
  const a0 = (rand() - 0.5) * 20.0; // [-10, +10] m/s^2
  const a1 = (rand() - 0.5) * 20.0;
  const T = 0.4 + rand() * 2.6;      // [0.4, 3.0] s

  const p = new QuinticPolynomial(q0, v0, a0, q1, v1, a1, T);

  // Check start
  assert.ok(Number.isFinite(p.eval(0)));
  assert.ok(Number.isFinite(p.deriv(0)));
  assert.ok(Number.isFinite(p.accel(0)));

  // Check end
  const qPre = p.eval(T - EPS);
  const vPre = p.deriv(T - EPS);
  const aPre = p.accel(T - EPS);

  assert.ok(Number.isFinite(qPre), `Fuzz #${i}: qPre finite`);
  assert.ok(Number.isFinite(vPre), `Fuzz #${i}: vPre finite`);
  assert.ok(Number.isFinite(aPre), `Fuzz #${i}: aPre finite`);

  const dq = Math.abs(qPre - q1);
  const dv = Math.abs(vPre - v1);
  const da = Math.abs(aPre - a1);

  fuzzMaxDq = Math.max(fuzzMaxDq, dq);
  fuzzMaxDv = Math.max(fuzzMaxDv, dv);
  fuzzMaxDa = Math.max(fuzzMaxDa, da);

  assert.ok(dq < 1e-4, `Fuzz #${i}: dq ${dq} < 1e-4`);
  assert.ok(dv < 1e-3, `Fuzz #${i}: dv ${dv} < 1e-3`);
  assert.ok(da < 0.02, `Fuzz #${i}: da ${da} < 0.02`);
}

console.log(`  [PASS] 5,000 random boundary sets verified:`);
console.log(`         Max Endpoint |q(T-eps) - q1|: ${fuzzMaxDq.toExponential(3)}`);
console.log(`         Max Endpoint |v(T-eps) - v1|: ${fuzzMaxDv.toExponential(3)}`);
console.log(`         Max Endpoint |a(T-eps) - a1|: ${fuzzMaxDa.toExponential(3)}`);
console.log(`         Zero boundary failures, NaNs, or infinities detected.`);

console.log('\n>>> ALL QUINTIC & TWO-STAGE TESTS PASSED CLEANLY (Exit 0) <<<');
