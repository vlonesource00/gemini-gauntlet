/**
 * Endurance Park is deliberately plain data: Track, AI, visuals and race setup
 * can consume it without sharing implementation details. Coordinates are world
 * metres (x/right, y/up, z/forward) and the loop is Catmull-Rom friendly.
 */

const TAU = Math.PI * 2;

export const ENDURANCE_PARK_CONTROL_POINTS = Object.freeze([
  { x: -350, y: 21, z: -190 },
  { x: 380, y: 22, z: -190 },
  { x: 465, y: 29, z: -130 },
  { x: 500, y: 38, z: 10 },
  { x: 455, y: 44, z: 150 },
  { x: 520, y: 42, z: 265 },
  { x: 455, y: 35, z: 390 },
  { x: 280, y: 27, z: 490 },
  { x: 60, y: 18, z: 525 },
  { x: -135, y: 13, z: 490 },
  { x: -270, y: 10, z: 400 },
  { x: -330, y: 12, z: 285 },
  { x: -455, y: 18, z: 175 },
  { x: -555, y: 28, z: 40 },
  { x: -535, y: 35, z: -105 },
  { x: -455, y: 29, z: -180 }
].map((point) => Object.freeze(point)));

const sceneryBands = Object.freeze([
  { type: 'tree', count: 196, lateralMin: 32, lateralMax: 118, scaleMin: 0.75, scaleMax: 1.55 },
  { type: 'fence', count: 164, lateralMin: 15, lateralMax: 21, scaleMin: 0.8, scaleMax: 1.15 },
  { type: 'tireStack', count: 42, lateralMin: 13, lateralMax: 18, scaleMin: 0.85, scaleMax: 1.2 },
  { type: 'cone', count: 56, lateralMin: 9, lateralMax: 13, scaleMin: 0.85, scaleMax: 1.12 },
  { type: 'brakingBoard', count: 18, lateralMin: 18, lateralMax: 23, scaleMin: 0.9, scaleMax: 1.05 },
  { type: 'marshal', count: 20, lateralMin: 17, lateralMax: 24, scaleMin: 0.8, scaleMax: 1.1 },
  { type: 'camera', count: 15, lateralMin: 20, lateralMax: 28, scaleMin: 0.8, scaleMax: 1.1 },
  { type: 'crowd', count: 38, lateralMin: 24, lateralMax: 34, scaleMin: 0.75, scaleMax: 1.25 },
  { type: 'flag', count: 28, lateralMin: 18, lateralMax: 25, scaleMin: 0.85, scaleMax: 1.2 },
  { type: 'lightGantry', count: 8, lateralMin: 19, lateralMax: 27, scaleMin: 0.9, scaleMax: 1.1 },
  { type: 'serviceVehicle', count: 9, lateralMin: 38, lateralMax: 58, scaleMin: 0.85, scaleMax: 1.15 },
  { type: 'camper', count: 13, lateralMin: 45, lateralMax: 68, scaleMin: 0.85, scaleMax: 1.2 }
]);

export const ENDURANCE_PARK = Object.freeze({
  id: 'endurance-park',
  name: 'Endurance Park',
  description: 'A long undulating parkland endurance circuit with a service-heavy main straight.',
  controlPoints: ENDURANCE_PARK_CONTROL_POINTS,
  roadHalfWidth: 7.6,
  curbWidth: 1.35,
  runoffWidth: 11.5,
  // Roughly six metres between physics samples: detailed enough for tyre
  // contact/banking without making Circuit.closest prohibitively expensive.
  sampleDensity: 32,
  elevation: Object.freeze({
    baseM: 10,
    minM: 10,
    maxM: 44,
    authoredDeltaM: 34,
    profile: 'control-point'
  }),
  bankHints: Object.freeze([
    { fromFraction: 0.08, toFraction: 0.17, bankRadians: 0.055, label: 'Quarry Sweep' },
    { fromFraction: 0.25, toFraction: 0.36, bankRadians: -0.045, label: 'North Esses' },
    { fromFraction: 0.60, toFraction: 0.72, bankRadians: 0.065, label: 'Oakland Bowl' }
  ]),
  sectors: Object.freeze([
    { id: 'start-finish', fromFraction: 0.00, toFraction: 0.205, character: 'high-speed', mainStraightM: 730 },
    { id: 'quarry', fromFraction: 0.205, toFraction: 0.385, character: 'technical', complex: 'Quarry Chicane' },
    { id: 'north-esses', fromFraction: 0.385, toFraction: 0.585, character: 'technical', complex: 'North Esses' },
    { id: 'oakland', fromFraction: 0.585, toFraction: 0.785, character: 'technical', complex: 'Oakland Bowl' },
    { id: 'park-run', fromFraction: 0.785, toFraction: 1.00, character: 'high-speed', complex: 'Parkland Kink' }
  ]),
  start: Object.freeze({
    finishFraction: 0.018,
    gridFraction: 0.962,
    direction: 1,
    rows: 20,
    rowSpacingM: 8.8,
    laneOffsetM: 2.65
  }),
  pit: Object.freeze({
    side: 'right',
    entryFraction: 0.842,
    limiterFraction: 0.868,
    boxStartFraction: 0.905,
    boxEndFraction: 0.965,
    exitFraction: 0.036,
    lateralM: -13.2,
    entryLateralM: -10.5,
    exitLateralM: -8.6,
    pitSpeedLimitMps: 16.67,
    entrySpeedLimitMps: 23.5,
    boxSpeedMps: 1.1,
    stoppedSpeedMps: 0.55,
    serviceDurationS: 7.5
  }),
  landmarks: Object.freeze([
    { type: 'controlTower', fraction: 0.042, lateral: -38, headingOffset: 0 },
    { type: 'hospitality', fraction: 0.074, lateral: -48, headingOffset: 0 },
    { type: 'footbridge', fraction: 0.122, lateral: 0, spansTrack: true },
    { type: 'tunnel', fraction: 0.455, lateral: 0, spansTrack: true },
    { type: 'crane', fraction: 0.915, lateral: -43, headingOffset: 0 },
    { type: 'serviceRoad', fromFraction: 0.82, toFraction: 0.06, lateral: -42 }
  ]),
  scenery: Object.freeze({
    seed: 73073,
    bands: sceneryBands,
    expectedInstanceCount: sceneryBands.reduce((total, band) => total + band.count, 0),
    drawCallBudget: 35
  })
});

export function createEnduranceParkScenario() {
  // The source object is immutable; callers that need to annotate it can use a
  // shallow copy without risking cross-race nondeterminism.
  return { ...ENDURANCE_PARK };
}

export function estimatedScenarioInstanceCount(scenario = ENDURANCE_PARK) {
  return scenario.scenery?.bands?.reduce((total, band) => total + (band.count || 0), 0) || 0;
}

function catmull(a, b, c, d, t) {
  const t2 = t * t;
  const t3 = t2 * t;
  return 0.5 * ((2 * b) + (-a + c) * t + (2 * a - 5 * b + 4 * c - d) * t2 + (-a + 3 * b - 3 * c + d) * t3);
}

/** Pure Catmull-Rom sampler useful to Track adapters and contract tests. */
export function sampleScenarioControlPoints(controlPoints = ENDURANCE_PARK_CONTROL_POINTS, samplesPerSpan = 12) {
  const points = [];
  const count = controlPoints.length;
  for (let index = 0; index < count; index += 1) {
    const a = controlPoints[(index - 1 + count) % count];
    const b = controlPoints[index];
    const c = controlPoints[(index + 1) % count];
    const d = controlPoints[(index + 2) % count];
    for (let sample = 0; sample < samplesPerSpan; sample += 1) {
      const t = sample / samplesPerSpan;
      points.push({
        x: catmull(a.x, b.x, c.x, d.x, t),
        y: catmull(a.y, b.y, c.y, d.y, t),
        z: catmull(a.z, b.z, c.z, d.z, t)
      });
    }
  }
  return points;
}

export function estimateScenarioLength(scenario = ENDURANCE_PARK, samplesPerSpan = 24) {
  const samples = sampleScenarioControlPoints(scenario.controlPoints, samplesPerSpan);
  return samples.reduce((length, point, index) => {
    const next = samples[(index + 1) % samples.length];
    return length + Math.hypot(next.x - point.x, next.y - point.y, next.z - point.z);
  }, 0);
}

function pointAtFraction(fraction, scenario) {
  const t = ((fraction % 1) + 1) % 1;
  const scaled = t * scenario.controlPoints.length;
  const index = Math.floor(scaled) % scenario.controlPoints.length;
  const localT = scaled - Math.floor(scaled);
  const count = scenario.controlPoints.length;
  const at = (offset) => scenario.controlPoints[(index + offset + count) % count];
  return {
    x: catmull(at(-1).x, at(0).x, at(1).x, at(2).x, localT),
    y: catmull(at(-1).y, at(0).y, at(1).y, at(2).y, localT),
    z: catmull(at(-1).z, at(0).z, at(1).z, at(2).z, localT)
  };
}

export function sampleEnduranceParkAtFraction(fraction, scenario = ENDURANCE_PARK) {
  const t = ((fraction % 1) + 1) % 1;
  const point = pointAtFraction(t, scenario);
  const epsilon = 1 / (scenario.controlPoints.length * 1000);
  const forward = pointAtFraction((t + epsilon) % 1, scenario);
  const tangentLength = Math.hypot(forward.x - point.x, forward.z - point.z) || 1;
  return {
    ...point,
    tangent: { x: (forward.x - point.x) / tangentLength, y: 0, z: (forward.z - point.z) / tangentLength },
    heading: Math.atan2(forward.x - point.x, forward.z - point.z),
    phase: t * TAU
  };
}
