import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

const TEST_SUITES = [
  {
    id: 'physics-parity',
    name: 'Physics & Surface Parity Suite',
    file: 'physics-parity-test.mjs',
    description: 'Track geometry (~2500m), elevations, banking, Pacejka tire forces, load transfer, and aero wake drag reduction'
  },
  {
    id: 'trajectory-eval',
    name: 'Trajectory Lattice & Cost Evaluation Suite',
    file: 'trajectory-eval-test.mjs',
    description: 'Frenet lattice generation, quintic minimum-jerk curves, GlobalTimeOptimal, GameTheoreticCombat, CoupledMPCC, and CombatDynamics'
  },
  {
    id: 'scenario-attack',
    name: 'Tactical Attack Scenarios (A1, A2, A3)',
    file: 'scenario-attack-test.mjs',
    description: 'Headless simulation of Slipstream Divebomb (A1), Chicane Inside Attack (A2), and NextGen Game-Theoretic Pass (A3)'
  },
  {
    id: 'scenario-defense',
    name: 'Tactical Defense Scenarios (D1, D2, D3)',
    file: 'scenario-defense-test.mjs',
    description: 'Headless simulation of Straight Defense (D1), Chicane Inside Defense (D2), and NextGen Stackelberg Defense (D3)'
  },
  {
    id: 'harbor-ring-contract',
    name: 'Harbor Ring Circuit Contract Suite',
    file: 'harbor-ring-contract.mjs',
    description: 'Purpose-built flat circuit geometry (2704m), zero banking/elevation invariants, and 3-class AI multi-lap execution'
  },
  {
    id: 'coupled-dynamics',
    name: 'Coupled Dynamics & Stanley Suite',
    file: 'coupled-dynamics-test.mjs',
    description: 'Curvature-feedforward, Stanley steering, 2D G-G friction-circle trail braking, and zero-GC horizon benchmark'
  },
  {
    id: 'twelve-car-race',
    name: '12-Car Multi-Class Full Grid Race Suite',
    file: 'twelve-car-race-test.mjs',
    description: 'Full 12-car grid simulation, pack racing emergence, dirty air wake matrices, multi-car passing, and zero deep overlaps'
  },
  {
    id: 'pace-benchmark',
    name: 'Multi-Class Flying Lap Pace Benchmark Suite',
    file: 'pace-benchmark-test.mjs',
    description: 'Autonomous flying lap pace verification across Prototype, GT, and Touring on flat and elevation circuits'
  },
  {
    id: 'decision-stability',
    name: 'Decision Stability & Anti-Indecision Suite',
    file: 'decision-stability-test.mjs',
    description: 'Temporal decision stability, 10Hz/25Hz/120Hz multirate decoupling, C2 trajectory switching hysteresis, and anti-twitching verification'
  },
  {
    id: 'supreme-racecraft',
    name: 'Supreme Racecraft V3 Suite (23 Scenarios)',
    file: 'supreme-racecraft-suite.mjs',
    description: 'Persistent Maneuvers, Predictive Attack, Intelligent Defense, 8-State Pass Machine, and Multi-Car Combat'
  }
];

console.log('\n' + '='.repeat(90));
console.log('            GEMINI GAUNTLET: AUTOMATED VERIFICATION TEST SUITE RUNNER');
console.log('='.repeat(90));
console.log(`Running ${TEST_SUITES.length} verification test suites...\n`);

const results = [];
let allPassed = true;
const globalStartTime = Date.now();

for (let i = 0; i < TEST_SUITES.length; i += 1) {
  const suite = TEST_SUITES[i];
  const suitePath = join(__dirname, suite.file);
  console.log(`\n[${i + 1}/${TEST_SUITES.length}] Executing: ${suite.name} (${suite.file})`);
  console.log(`    Scope: ${suite.description}`);
  console.log('-'.repeat(90));

  const startTime = Date.now();
  const child = spawnSync(process.execPath, [suitePath], {
    cwd: join(__dirname, '..'),
    encoding: 'utf-8',
    stdio: 'inherit'
  });
  const durationMs = Date.now() - startTime;
  const passed = child.status === 0;

  if (!passed) {
    allPassed = false;
  }

  results.push({
    ...suite,
    passed,
    status: child.status,
    durationMs
  });
}

const totalDurationMs = Date.now() - globalStartTime;

// ---------------------------------------------------------------------------
// Formatted Test Results Summary Table
// ---------------------------------------------------------------------------
console.log('\n' + '='.repeat(90));
console.log('                               TEST SUITE RESULTS SUMMARY');
console.log('='.repeat(90));
console.log(
  'Suite'.padEnd(46) +
  'Status'.padEnd(16) +
  'Exit Code'.padEnd(14) +
  'Duration (s)'.padEnd(14)
);
console.log('-'.repeat(90));

for (const res of results) {
  const statusStr = res.passed ? 'PASSED [OK]' : 'FAILED [ERR]';
  const durationStr = (res.durationMs / 1000).toFixed(2) + 's';
  console.log(
    res.name.padEnd(46) +
    statusStr.padEnd(16) +
    String(res.status).padEnd(14) +
    durationStr.padEnd(14)
  );
}

console.log('-'.repeat(90));
const passCount = results.filter((r) => r.passed).length;
const failCount = results.filter((r) => !r.passed).length;
console.log(`Total Suites: ${results.length} | Passed: ${passCount} | Failed: ${failCount} | Total Duration: ${(totalDurationMs / 1000).toFixed(2)}s`);
console.log('='.repeat(90));

if (allPassed) {
  console.log('\n>>> SUCCESS: ALL VERIFICATION TEST SUITES PASSED CLEANLY (Exit 0) <<<\n');
  process.exit(0);
} else {
  console.error('\n>>> FAILURE: ONE OR MORE TEST SUITES FAILED (Exit 1) <<<\n');
  process.exit(1);
}
