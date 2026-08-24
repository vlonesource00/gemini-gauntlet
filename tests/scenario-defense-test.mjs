import assert from 'node:assert/strict';
import { Circuit } from '../src/simulation/Track.js';
import { ENDURANCE_PARK } from '../src/scenarios/EndurancePark.js';
import { Vehicle } from '../src/simulation/Vehicle.js';
import { ResearchAIController } from '../src/ai/ResearchAIController.js';
import { NextGenAIController } from '../src/ai/v2/NextGenAIController.js';
import { updateAerodynamicWakes, resolveVehicleCollisions } from '../src/simulation/VehicleInteractions.js';

const DT = 1 / 120;

console.log('=== [4/5] Running Tactical Defense Scenarios Test Suite (D1, D2, D3) ===');

const setForwardSpeed = (vehicle, speedMs, track) => {
  const p = track.atDistance(vehicle.distance);
  vehicle.speed = speedMs;
  vehicle.velocity = { x: p.tangent.x * speedMs, y: 0, z: p.tangent.z * speedMs };
  vehicle.localVelocity = { x: 0, z: speedMs };
  for (const wheel of vehicle.wheels || []) {
    wheel.omega = speedMs / (vehicle.wheelRadius || 0.335);
  }
};

function runDefenseScenario({ name, startDistance = 150, gapM = 20, challengerLateral = 0, trafficAhead = false, durationS = 8.0 }) {
  const track = new Circuit(ENDURANCE_PARK);
  const leader = new Vehicle({ id: `${name}-leader`, spec: 'gt' });
  const challenger = new Vehicle({ id: `${name}-challenger`, spec: 'gt' });

  leader.resetTo(track, startDistance, 0.0);
  challenger.resetTo(track, startDistance - gapM, challengerLateral);

  setForwardSpeed(leader, 25, track);
  setForwardSpeed(challenger, 31, track); // Challenger is faster (+6 m/s closing speed)

  const vehicles = [leader, challenger];
  if (trafficAhead) {
    const traffic = new Vehicle({ id: `${name}-traffic`, spec: 'gt' });
    traffic.resetTo(track, startDistance + 22, -1.5);
    setForwardSpeed(traffic, 25.5, track);
    vehicles.push(traffic);
  }

  const controllers = new Map(vehicles.map((v, idx) => [
    v,
    new ResearchAIController(idx + 1, {
      defenseReactivity: 0.85,
      aggression: idx === 1 ? 0.90 : 0.70
    })
  ]));

  for (const controller of controllers.values()) {
    controller.debugEnabled = true;
  }

  const race = {
    phase: 'racing',
    raceTime: 10.0,
    elapsed: 10.0,
    statusFor: (v) => ({ position: vehicles.indexOf(v) + 1 })
  };

  let defenseSeconds = 0;
  let firstDefenseGap = null;
  let contactFrames = 0;
  let deepOverlapFrames = 0;
  let offTrackSeconds = 0;
  let priorDirection = 0;
  let reversals = 0;
  const totalSteps = Math.round(durationS / DT);

  for (let step = 0; step < totalSteps; step += 1) {
    race.raceTime += DT;
    race.elapsed += DT;

    for (const v of vehicles) {
      controllers.get(v).update(v, vehicles, track, race, DT);
    }
    updateAerodynamicWakes(vehicles);

    for (const v of vehicles) {
      v.step(DT, track, true);
    }

    const collision = resolveVehicleCollisions(vehicles, 3);
    contactFrames += collision.contacts ?? 0;
    deepOverlapFrames += collision.deepOverlaps ?? 0;

    if (vehicles.some((v) => v.surface?.zone === 'grass' || v.surface?.zone === 'runoff')) {
      offTrackSeconds += DT;
    }

    const leaderState = controllers.get(leader).debugState;
    const leaderDefending = leaderState?.mode === 'DEFEND' || controllers.get(leader).defenseEngine.defending;

    if (leaderDefending) {
      defenseSeconds += DT;
      firstDefenseGap ??= leader.distance - challenger.distance;
      const targetOffset = leaderState?.desiredOffset ?? leaderState?.targetOffset ?? 0;
      const direction = Math.sign(targetOffset);
      if (priorDirection && direction && priorDirection !== direction) {
        reversals += 1;
      }
      if (direction) priorDirection = direction;
    }
  }

  return {
    name,
    defenseSeconds: Number(defenseSeconds.toFixed(2)),
    firstDefenseGapM: Number((firstDefenseGap ?? -1).toFixed(2)),
    contactFrames,
    deepOverlapFrames,
    offTrackSeconds: Number(offTrackSeconds.toFixed(2)),
    reversals
  };
}

// ---------------------------------------------------------------------------
// 1. Defense Scenario D1: Straight Defense (Centered & Slipstream Pull-out)
// ---------------------------------------------------------------------------
console.log('  -> Simulating Defense Scenario D1 (Straight Line Inside Defense & Tow-Break)...');
const d1Centered = runDefenseScenario({ name: 'D1-centered-20m', startDistance: 150, gapM: 20, challengerLateral: 0.0 });
const d1Pullout = runDefenseScenario({ name: 'D1-slipstream-pullout', startDistance: 150, gapM: 20, challengerLateral: 2.8 });

console.log(`    D1-Centered: defenseTime=${d1Centered.defenseSeconds}s, firstGap=${d1Centered.firstDefenseGapM}m, reversals=${d1Centered.reversals}, overlaps=${d1Centered.deepOverlapFrames}`);
assert.ok(d1Centered.defenseSeconds > 0.2, 'D1 Centered: AI must detect closing challenger and trigger defensive move');
assert.equal(d1Centered.deepOverlapFrames, 0, 'D1 Centered: Defense must never produce deep overlap');
assert.equal(d1Centered.offTrackSeconds, 0, 'D1 Centered: Defender must keep the field on legal track');
assert.equal(d1Centered.reversals, 0, 'D1 Centered: Defender must obey single defensive move rule (zero reversals)');
console.log('    [PASS] Scenario D1 (Centered): Legal defensive inside line held.');

console.log(`    D1-Pullout: defenseTime=${d1Pullout.defenseSeconds}s, firstGap=${d1Pullout.firstDefenseGapM}m, reversals=${d1Pullout.reversals}, overlaps=${d1Pullout.deepOverlapFrames}`);
assert.ok(d1Pullout.defenseSeconds > 0.2, 'D1 Pullout: AI must detect offset challenger and claim defensive line');
assert.equal(d1Pullout.deepOverlapFrames, 0, 'D1 Pullout: Defense must never produce deep overlap');
assert.equal(d1Pullout.offTrackSeconds, 0, 'D1 Pullout: Field must remain on legal track');
assert.equal(d1Pullout.reversals, 0, 'D1 Pullout: Defender must not weave (zero directional reversals)');
console.log('    [PASS] Scenario D1 (Pullout): Slipstream breaking defense verified.');

// ---------------------------------------------------------------------------
// 2. Defense Scenario D2: Chicane Approach Defense (Quarry Chicane Approach)
// ---------------------------------------------------------------------------
console.log('  -> Simulating Defense Scenario D2 (Quarry Chicane Approach Inside Line Defense)...');
const d2Chicane = runDefenseScenario({ name: 'D2-chicane-defense', startDistance: 640, gapM: 18, challengerLateral: 1.5, durationS: 10.0 });

console.log(`    D2-Chicane: defenseTime=${d2Chicane.defenseSeconds}s, firstGap=${d2Chicane.firstDefenseGapM}m, reversals=${d2Chicane.reversals}, overlaps=${d2Chicane.deepOverlapFrames}`);
assert.ok(d2Chicane.defenseSeconds > 0.2, 'D2 Chicane: Approaching chicane, defender must claim inside line defensively');
assert.equal(d2Chicane.deepOverlapFrames, 0, 'D2 Chicane: Chicane defense must never produce deep overlap');
assert.equal(d2Chicane.offTrackSeconds, 0, 'D2 Chicane: Chicane defense must keep vehicles on track');
assert.ok(d2Chicane.reversals <= 1, 'D2 Chicane: Defender must make at most one directional defensive transition');
console.log('    [PASS] Scenario D2: Quarry Chicane inside defense verified.');

// ---------------------------------------------------------------------------
// 3. Blocked Preferred Defense Corridor Fallback
// ---------------------------------------------------------------------------
console.log('  -> Testing Blocked Defensive Corridor Adaptation...');
{
  const track = new Circuit(ENDURANCE_PARK);
  const leader = new Vehicle({ id: 'blocked-defender', spec: 'gt' });
  const challenger = new Vehicle({ id: 'blocked-challenger', spec: 'gt' });
  const blocker = new Vehicle({ id: 'inside-blocker', spec: 'gt' });

  leader.resetTo(track, 420, 0.0);
  challenger.resetTo(track, 404, 0.0); // 16m behind

  // Place blocker in the preferred inside corridor
  const defenseTurn = [24, 42, 64]
    .map((d) => track.atDistance(420 + d))
    .sort((a, b) => Math.abs(b.curvature) - Math.abs(a.curvature))[0];
  const preferredInsideOffset = (Math.sign(defenseTurn?.turnSign) || 1) * 3.5;
  blocker.resetTo(track, 420, preferredInsideOffset);

  setForwardSpeed(leader, 28, track);
  setForwardSpeed(challenger, 34, track);
  setForwardSpeed(blocker, 28, track);

  const controller = new ResearchAIController(1, { defenseReactivity: 0.88 });
  controller.debugEnabled = true;

  controller.update(
    leader,
    [leader, challenger, blocker],
    track,
    { phase: 'racing', raceTime: 10.0, elapsed: 10.0, statusFor: () => ({ position: 1 }) },
    DT
  );

  const defState = controller.debugState;
  assert.ok(
    Math.abs((defState?.desiredOffset ?? 0) - preferredInsideOffset) > 0.8,
    'Defender must adapt to alternative legal corridor rather than steering into inside blocker'
  );
  console.log('    [PASS] Blocked corridor defense adaptation verified.');
}

// ---------------------------------------------------------------------------
// 4. Defense Scenario D3: NextGen AI Game-Theoretic Stackelberg Defense
// ---------------------------------------------------------------------------
console.log('  -> Simulating Defense Scenario D3 (NextGen AI Stackelberg Leader Defense)...');
{
  const track = new Circuit(ENDURANCE_PARK);
  const leader = new Vehicle({ id: 'leader-d3', spec: 'gt' });
  const challenger = new Vehicle({ id: 'challenger-d3', spec: 'gt' });

  leader.resetTo(track, 150, 0.0);
  challenger.resetTo(track, 130, 2.5);

  setForwardSpeed(leader, 25, track);
  setForwardSpeed(challenger, 31, track);

  const leaderAI = new NextGenAIController('leader-ai-d3', { defenseReactivity: 0.92 });
  const challengerAI = new NextGenAIController('challenger-ai-d3', { aggression: 0.90 });

  const vehicles = [leader, challenger];
  const race = {
    phase: 'racing',
    raceTime: 10.0,
    elapsed: 10.0,
    statusFor: (v) => ({ position: v === leader ? 1 : 2 })
  };

  let defenseFrames = 0;
  let deepOverlapFrames = 0;
  let offTrackFrames = 0;
  let reversals = 0;
  let priorSign = 0;
  const simDurationS = 8.0;
  const totalSteps = Math.round(simDurationS / DT);

  for (let step = 0; step < totalSteps; step += 1) {
    race.raceTime += DT;
    race.elapsed += DT;

    leaderAI.update(leader, vehicles, track, race, DT);
    challengerAI.update(challenger, vehicles, track, race, DT);
    updateAerodynamicWakes(vehicles);

    leader.step(DT, track, true);
    challenger.step(DT, track, true);

    const collision = resolveVehicleCollisions(vehicles, 3);
    if (collision.deepOverlaps > 0) deepOverlapFrames += 1;

    if (vehicles.some((v) => v.surface?.zone === 'grass' || v.surface?.zone === 'runoff')) {
      offTrackFrames += 1;
    }

    const isDefending = leaderAI.debugState?.telemetry?.state === 'DEFEND'
      || leaderAI.debugState?.thought?.defending === true
      || leaderAI.combatEngine?.defenseMode !== 'PACE';

    if (isDefending) {
      defenseFrames += 1;
      const targetLat = leaderAI.debugState?.thought?.deployedOffsetM
        ?? leaderAI.debugState?.thought?.trajectorySelectedOffsetM
        ?? leaderAI.debugState?.targetLateral
        ?? 0;
      const sign = Math.sign(targetLat);
      if (priorSign && sign && priorSign !== sign) {
        reversals += 1;
      }
      if (sign) priorSign = sign;
    }
  }

  const defenseTimeS = defenseFrames * DT;
  const offTrackTimeS = offTrackFrames * DT;
  console.log(`    D3 Result: defenseTime=${defenseTimeS.toFixed(2)}s, reversals=${reversals}, deepOverlaps=${deepOverlapFrames}, offTrack=${offTrackTimeS.toFixed(2)}s`);

  assert.ok(defenseTimeS > 0.5, 'NextGen AI must detect challenger and trigger Stackelberg defense');
  assert.equal(deepOverlapFrames, 0, 'NextGen Stackelberg defense must never produce deep overlap');
  assert.equal(offTrackTimeS, 0, 'NextGen defense must keep vehicles on track');
  assert.ok(reversals <= 1, 'NextGen defender must not weave across track');
  console.log('    [PASS] Scenario D3: NextGen Stackelberg defense verified.');
}

console.log('=== Tactical Defense Scenarios Test Suite: ALL ASSERTIONS PASSED ===\n');
