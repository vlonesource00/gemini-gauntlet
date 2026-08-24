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
    description: 'Frenet lattice candidate generation, quintic minimum-jerk curves, road-violation penalties, and collision risk scoring'
  },
  {
    id: 'scenario-attack',
    name: 'Tactical Attack Scenarios (A1 & A2)',
    file: 'scenario-attack-test.mjs',
    description: 'Headless simulation of Straight Slipstream Divebomb (A1) and Quarry Chicane Inside Attack (A2) with 0 contacts'
  },
  {
    id: 'scenario-defense',
    name: 'Tactical Defense Scenarios (D1 & D2)',
    file: 'scenario-defense-test.mjs',
    description: 'Headless simulation of Straight Line Defense (D1) and Chicane Inside Line Defense (D2) obeying FIA one-move rule'
  },
  {
    id: 'harbor-ring-contract',
    name: 'Harbor Ring Circuit Contract Suite',
    file: 'harbor-ring-contract.mjs',
    description: 'Purpose-built flat circuit geometry (2704m), zero banking/elevation invariants, and 3-class AI multi-lap execution'
  }
];

console.log('\n' + '='.repeat(80));
console.log('       GEMINI GAUNTLET: AUTOMATED VERIFICATION TEST SUITE RUNNER');
console.log('='.repeat(80));
console.log(`Running ${TEST_SUITES.length} verification test suites...\n`);

const results = [];
let allPassed = true;
const globalStartTime = Date.now();

for (let i = 0; i < TEST_SUITES.length; i += 1) {
  const suite = TEST_SUITES[i];
  const suitePath = join(__dirname, suite.file);
  console.log(`\n[${i + 1}/${TEST_SUITES.length}] Executing: ${suite.name} (${suite.file})`);
  console.log(`    Scope: ${suite.description}`);
  console.log('-'.repeat(80));

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
console.log('\n' + '='.repeat(80));
console.log('                          TEST SUITE RESULTS SUMMARY');
console.log('='.repeat(80));
console.log(
  'Suite'.padEnd(35) +
  'Status'.padEnd(12) +
  'Exit Code'.padEnd(14) +
  'Duration (s)'.padEnd(15)
);
console.log('-'.repeat(80));

for (const res of results) {
  const statusStr = res.passed ? 'PASSED [OK]' : 'FAILED [ERR]';
  const durationStr = (res.durationMs / 1000).toFixed(2) + 's';
  console.log(
    res.name.padEnd(35) +
    statusStr.padEnd(12) +
    String(res.status).padEnd(14) +
    durationStr.padEnd(15)
  );
}

console.log('-'.repeat(80));
const passCount = results.filter((r) => r.passed).length;
const failCount = results.filter((r) => !r.passed).length;
console.log(`Total Suites: ${results.length} | Passed: ${passCount} | Failed: ${failCount} | Total Duration: ${(totalDurationMs / 1000).toFixed(2)}s`);
console.log('='.repeat(80));

if (allPassed) {
  console.log('\n>>> SUCCESS: ALL VERIFICATION TEST SUITES PASSED CLEANLY (Exit 0) <<<\n');
  process.exit(0);
} else {
  console.error('\n>>> FAILURE: ONE OR MORE TEST SUITES FAILED (Exit 1) <<<\n');
  process.exit(1);
}
