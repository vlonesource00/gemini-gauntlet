import fs from 'node:fs';
import { Circuit } from '../src/simulation/Track.js';
import { Vehicle } from '../src/simulation/Vehicle.js';
import { NextGenAIController } from '../src/ai/v2/NextGenAIController.js';
import { ENDURANCE_PARK } from '../src/scenarios/EndurancePark.js';

const DT = 1 / 120;
const track = new Circuit(ENDURANCE_PARK);
const trackLen = track.length;

console.log('================================================================================');
console.log('              GEMINI SUPREME PACE INVESTIGATION: HUMAN VS AI');
console.log('================================================================================');

// 1. Load Human Reference Lap Telemetry
const humanLapData = JSON.parse(fs.readFileSync('public/data/endurance-park-prototype-reference.json', 'utf8'));
const humanSamples = humanLapData.samples;
const humanDuration = humanLapData.durationS;
console.log(`Loaded Human Reference Lap: duration=${humanDuration.toFixed(3)}s, samples=${humanSamples.length}`);

// 2. Setup AI Vehicle and Controller
const spec = 'prototype';
const vehicle = new Vehicle({ id: 'supreme-telemetry', spec, player: false });
const controller = new NextGenAIController('supreme-telemetry', { track, aggression: 1.0, skill: 1.0 });

// Start vehicle on racing line at distance 0 with flying speed
const startPoint = controller.optimalEngine.sampleAtDistance(0, spec);
vehicle.resetTo(track, 0, startPoint.lateral);
const forward = { x: Math.sin(vehicle.yaw), z: Math.cos(vehicle.yaw) };
const flySpeed = startPoint.targetSpeed;
vehicle.velocity.x = forward.x * flySpeed;
vehicle.velocity.z = forward.z * flySpeed;
vehicle.speed = flySpeed;
vehicle.gear = 4;
vehicle.rpm = 5800;
vehicle.controls.throttle = 1.0;

const race = { raceTime: 0, elapsed: 0, phase: 'racing' };
let lapCount = 0;
let lapTime = 0;
let lastDist = 0;
let aiSamples = [];
let recording = false;
let aiFlyingLapTime = null;

const maxSimTime = 250.0;
for (let t = 0; t < maxSimTime; t += DT) {
  race.raceTime = t;
  race.elapsed = t;

  controller.update(vehicle, [vehicle], track, race, DT);
  vehicle.step(DT, track, true);

  lapTime += DT;

  // Detect lap completion
  if (lastDist > trackLen * 0.85 && vehicle.distance < trackLen * 0.15 && lapTime > 30.0) {
    lapCount++;
    console.log(`  -> AI Lap ${lapCount} finished in ${lapTime.toFixed(3)}s`);
    if (lapCount === 1) {
      // Start recording flying lap 2
      recording = true;
      aiSamples = [];
      lapTime = 0;
    } else if (lapCount === 2) {
      aiFlyingLapTime = lapTime;
      break;
    }
  }

  if (Math.floor(t / 10) !== Math.floor((t - DT) / 10)) {
    console.log(`[t=${t.toFixed(1)}s] lap=${lapCount+1} lapTime=${lapTime.toFixed(1)}s dist=${vehicle.distance.toFixed(0)}m speed=${(vehicle.speed*3.6).toFixed(1)}km/h lat=${(vehicle.surface?.lateral||0).toFixed(2)}m offTrack=${vehicle.surface?.offTrack}`);
  }

  if (recording) {
    const opt = controller.optimalEngine.sampleAtDistance(vehicle.distance, spec);
    const dbg = controller.debugState || {};
    const telem = dbg.telemetry || {};
    const liveSlip = Math.atan2(
      vehicle.localVelocity?.x || 0,
      Math.max(3.0, Math.abs(vehicle.localVelocity?.z || vehicle.speed))
    );

    aiSamples.push({
      t: lapTime,
      s: vehicle.distance,
      lateral: vehicle.surface?.lateral || 0,
      speed: vehicle.speed,
      throttle: vehicle.controls.throttle,
      brake: vehicle.controls.brake,
      steer: vehicle.controls.steer,
      gear: vehicle.gear,
      rpm: vehicle.rpm,
      yaw: vehicle.yaw,
      yawRate: vehicle.yawRate,
      bodySlip: liveSlip,
      lateralG: (vehicle.localAcceleration?.x || 0) / 9.80665,
      longitudinalG: (vehicle.localAcceleration?.z || 0) / 9.80665,
      optSpeed: opt?.targetSpeed || 0,
      optLat: opt?.lateral || 0,
      desiredSpeed: vehicle.aiTactical?.desiredSpeed || 0,
      selectedOffset: controller.trajectoryPlan?.selectedOffset ?? 0,
      mpccSteer: dbg.mpccOut?.steer,
      mpccThrottle: dbg.mpccOut?.throttle,
      mpccBrake: dbg.mpccOut?.brake,
      loadTransferRate: telem.lateralLoadTransferRate || 0,
      steerReversals: telem.steeringReversalsLastSecond || 0
    });
  }

  lastDist = vehicle.distance;
}

console.log(`\nAI Flying Lap Completed: ${aiFlyingLapTime.toFixed(3)}s vs Human: ${humanDuration.toFixed(3)}s (Delta: +${(aiFlyingLapTime - humanDuration).toFixed(3)}s)`);

// 3. Distance-Aligned Interpolation (every 5 meters)
const stations = [];
const stepM = 5.0;
const stationCount = Math.floor(trackLen / stepM);

function sampleAtDistance(samples, targetS) {
  if (!samples.length) return null;
  // Find surrounding samples
  let idx = samples.findIndex((s) => s.s >= targetS);
  if (idx === -1) idx = samples.length - 1;
  if (idx === 0) return samples[0];
  const s0 = samples[idx - 1];
  const s1 = samples[idx];
  const span = Math.max(1e-4, s1.s - s0.s);
  const frac = Math.max(0, Math.min(1, (targetS - s0.s) / span));

  const interp = {};
  for (const k of Object.keys(s0)) {
    if (typeof s0[k] === 'number') {
      interp[k] = s0[k] + frac * (s1[k] - s0[k]);
    } else {
      interp[k] = s0[k];
    }
  }
  return interp;
}

const alignedRows = [];
for (let i = 0; i <= stationCount; i++) {
  const s = i * stepM;
  if (s > trackLen) break;
  const h = sampleAtDistance(humanSamples, s);
  const a = sampleAtDistance(aiSamples, s);
  if (!h || !a) continue;

  const dt = a.t - h.t;
  alignedRows.push({
    s,
    hTime: h.t,
    aTime: a.t,
    dt,
    hSpeed: h.speed,
    aSpeed: a.speed,
    dSpeed: a.speed - h.speed,
    hThrottle: h.throttle,
    aThrottle: a.throttle,
    hBrake: h.brake,
    aBrake: a.brake,
    hLat: h.lateral,
    aLat: a.lateral,
    dLat: a.lateral - h.lateral,
    hLatG: h.lateralG,
    aLatG: a.lateralG,
    hLongG: h.longitudinalG,
    aLongG: a.longitudinalG,
    aOptSpeed: a.optSpeed,
    aDesiredSpeed: a.desiredSpeed
  });
}

// 4. Sector Performance Analysis
const s1Dist = 950.0;
const s2Dist = 2100.0;
const s3Dist = trackLen;

const s1Row = alignedRows.find((r) => r.s >= s1Dist) || alignedRows[alignedRows.length - 1];
const s2Row = alignedRows.find((r) => r.s >= s2Dist) || alignedRows[alignedRows.length - 1];
const s3Row = alignedRows[alignedRows.length - 1];

console.log('\n--------------------------------------------------------------------------------');
console.log('                         SECTOR TIME ANALYSIS');
console.log('--------------------------------------------------------------------------------');
console.log('Sector     Human Time     AI Time       Delta (s)     Status');
console.log('--------------------------------------------------------------------------------');
const hS1 = s1Row.hTime;
const aS1 = s1Row.aTime;
const dS1 = aS1 - hS1;
console.log(`Sector 1   ${hS1.toFixed(3).padStart(9)}s   ${aS1.toFixed(3).padStart(9)}s   ${(dS1 >= 0 ? '+' : '') + dS1.toFixed(3).padStart(7)}s   ${dS1 <= 0.2 ? '[PARITY]' : '[TIME LOSS]'}`);

const hS2 = s2Row.hTime - s1Row.hTime;
const aS2 = s2Row.aTime - s1Row.aTime;
const dS2 = aS2 - hS2;
console.log(`Sector 2   ${hS2.toFixed(3).padStart(9)}s   ${aS2.toFixed(3).padStart(9)}s   ${(dS2 >= 0 ? '+' : '') + dS2.toFixed(3).padStart(7)}s   ${dS2 <= 0.2 ? '[PARITY]' : '[TIME LOSS]'}`);

const hS3 = s3Row.hTime - s2Row.hTime;
const aS3 = s3Row.aTime - s2Row.aTime;
const dS3 = aS3 - hS3;
console.log(`Sector 3   ${hS3.toFixed(3).padStart(9)}s   ${aS3.toFixed(3).padStart(9)}s   ${(dS3 >= 0 ? '+' : '') + dS3.toFixed(3).padStart(7)}s   ${dS3 <= 0.2 ? '[PARITY]' : '[TIME LOSS]'}`);
console.log(`TOTAL      ${humanDuration.toFixed(3).padStart(9)}s   ${aiFlyingLapTime.toFixed(3).padStart(9)}s   ${'+' + (aiFlyingLapTime - humanDuration).toFixed(3).padStart(7)}s`);

// 5. Major Corner Analysis & Diagnostic Attribution
const CORNERS = [
  { id: 'T1', name: 'Turn 1 - Quarry Chicane', startM: 680, apexM: 780, exitM: 880 },
  { id: 'T2', name: 'Turn 2 - North Esses', startM: 1060, apexM: 1150, exitM: 1260 },
  { id: 'T3', name: 'Turn 3 - Oakland Bowl Sweep', startM: 1780, apexM: 1940, exitM: 2060 },
  { id: 'T4', name: 'Turn 4 - South Hairpin', startM: 2200, apexM: 2340, exitM: 2480 },
  { id: 'T5', name: 'Turn 5 - Pit Complex Chicane', startM: 2920, apexM: 2990, exitM: 3060 }
];

console.log('\n--------------------------------------------------------------------------------');
console.log('                     CORNER-BY-CORNER TIME LOSS & CAUSE');
console.log('--------------------------------------------------------------------------------');

const cornerReports = [];

for (const c of CORNERS) {
  const startRow = alignedRows.find((r) => r.s >= c.startM);
  const apexRow = alignedRows.find((r) => r.s >= c.apexM);
  const exitRow = alignedRows.find((r) => r.s >= c.exitM);
  const cornerLoss = (exitRow.dt - startRow.dt);

  // Braking analysis
  const cRows = alignedRows.filter((r) => r.s >= c.startM && r.s <= c.exitM);
  const hBrakeRow = cRows.find((r) => r.hBrake > 0.1);
  const aBrakeRow = cRows.find((r) => r.aBrake > 0.1);
  const hBrakeStart = hBrakeRow ? hBrakeRow.s : null;
  const aBrakeStart = aBrakeRow ? aBrakeRow.s : null;

  // Apex speed
  const hApexSpeed = apexRow.hSpeed * 3.6;
  const aApexSpeed = apexRow.aSpeed * 3.6;

  // Track width used
  const maxHLat = Math.max(...cRows.map((r) => Math.abs(r.hLat)));
  const maxALat = Math.max(...cRows.map((r) => Math.abs(r.aLat)));

  // Full throttle recovery point
  const hThrottleExit = cRows.find((r) => r.s >= c.apexM && r.hThrottle > 0.9);
  const aThrottleExit = cRows.find((r) => r.s >= c.apexM && r.aThrottle > 0.9);

  cornerReports.push({
    ...c,
    loss: cornerLoss,
    hBrakeStart,
    aBrakeStart,
    hApexSpeed,
    aApexSpeed,
    maxHLat,
    maxALat,
    hThrottleExit: hThrottleExit ? hThrottleExit.s : null,
    aThrottleExit: aThrottleExit ? aThrottleExit.s : null
  });
}

cornerReports.sort((a, b) => b.loss - a.loss);

for (const cr of cornerReports) {
  console.log(`\n[${cr.id}] ${cr.name} -> Time Delta: ${(cr.loss >= 0 ? '+' : '') + cr.loss.toFixed(3)}s`);
  console.log(`  Braking:     Human @ ${cr.hBrakeStart ? cr.hBrakeStart.toFixed(0) + 'm' : 'N/A'} vs AI @ ${cr.aBrakeStart ? cr.aBrakeStart.toFixed(0) + 'm' : 'N/A'} (Diff: ${cr.aBrakeStart && cr.hBrakeStart ? (cr.aBrakeStart - cr.hBrakeStart).toFixed(1) + 'm' : 'N/A'})`);
  console.log(`  Apex Speed:  Human ${cr.hApexSpeed.toFixed(1)} km/h vs AI ${cr.aApexSpeed.toFixed(1)} km/h (Delta: ${(cr.aApexSpeed - cr.hApexSpeed).toFixed(1)} km/h)`);
  console.log(`  Track Width: Human max |lat|=${cr.maxHLat.toFixed(2)}m vs AI max |lat|=${cr.maxALat.toFixed(2)}m`);
  console.log(`  Full Power:  Human @ ${cr.hThrottleExit ? cr.hThrottleExit.toFixed(0) + 'm' : 'N/A'} vs AI @ ${cr.aThrottleExit ? cr.aThrottleExit.toFixed(0) + 'm' : 'N/A'}`);
}

// 6. Output Table of Top 10 Time Loss Hotspots across the Lap
console.log('\n--------------------------------------------------------------------------------');
console.log('                 TOP 10 SPATIAL TIME LOSS HOTSPOTS (100m BINS)');
console.log('--------------------------------------------------------------------------------');
console.log('Station Window     Segment Loss (s)     Cumulative dt     Primary Subsystem / Bottleneck');
console.log('--------------------------------------------------------------------------------');

const binSize = 100.0;
const bins = [];
for (let s0 = 0; s0 < trackLen; s0 += binSize) {
  const s1 = Math.min(trackLen, s0 + binSize);
  const r0 = alignedRows.find((r) => r.s >= s0) || alignedRows[0];
  const r1 = alignedRows.find((r) => r.s >= s1) || alignedRows[alignedRows.length - 1];
  const binLoss = r1.dt - r0.dt;

  // Identify bottleneck
  let cause = 'SPEED_MATCH';
  if (binLoss > 0.04) {
    if (r0.aBrake > 0.1 && r0.hBrake < 0.1) {
      cause = 'EARLY_BRAKING';
    } else if (r1.hSpeed - r1.aSpeed > 5.0) {
      cause = 'CORNER_EXIT_ACCEL / APEX_SPEED';
    } else if (Math.abs(r1.hLat) - Math.abs(r1.aLat) > 1.2) {
      cause = 'UNDERUSED_ROAD_WIDTH';
    } else {
      cause = 'TRACTION / SPEED_ENVELOPE';
    }
  }
  bins.push({ s0, s1, binLoss, cumDt: r1.dt, cause });
}

bins.sort((a, b) => b.binLoss - a.binLoss);
for (const b of bins.slice(0, 10)) {
  console.log(`${b.s0.toFixed(0).padStart(5)}m - ${b.s1.toFixed(0).padStart(5)}m    ${(b.binLoss >= 0 ? '+' : '') + b.binLoss.toFixed(3).padStart(8)}s        ${b.cumDt.toFixed(3).padStart(7)}s          ${b.cause}`);
}
console.log('================================================================================\n');
