/**
 * Harbor Ring is a purpose-built, completely flat race circuit. It combines
 * a long passing straight with a heavy-braking hairpin, a technical infield
 * and two wide corner exits so every vehicle class has more than one viable
 * racecraft line.
 */

export const HARBOR_RING_CONTROL_POINTS = Object.freeze([
  { x: -430, y: 0, z: -205 },
  { x: 330, y: 0, z: -205 },
  { x: 440, y: 0, z: -170 },
  { x: 485, y: 0, z: -75 },
  { x: 455, y: 0, z: 35 },
  { x: 365, y: 0, z: 105 },
  { x: 260, y: 0, z: 82 },
  { x: 205, y: 0, z: 15 },
  { x: 125, y: 0, z: 62 },
  { x: 105, y: 0, z: 180 },
  { x: -35, y: 0, z: 255 },
  { x: -215, y: 0, z: 245 },
  { x: -350, y: 0, z: 175 },
  { x: -405, y: 0, z: 80 },
  { x: -330, y: 0, z: 5 },
  { x: -455, y: 0, z: -72 }
].map((point) => Object.freeze(point)));

const sceneryBands = Object.freeze([
  { type: 'fence', count: 150, lateralMin: 16, lateralMax: 22, scaleMin: 0.9, scaleMax: 1.1 },
  { type: 'tireStack', count: 48, lateralMin: 14, lateralMax: 19, scaleMin: 0.9, scaleMax: 1.2 },
  { type: 'cone', count: 48, lateralMin: 10, lateralMax: 14, scaleMin: 0.9, scaleMax: 1.1 },
  { type: 'brakingBoard', count: 20, lateralMin: 18, lateralMax: 24, scaleMin: 0.9, scaleMax: 1.05 },
  { type: 'marshal', count: 18, lateralMin: 18, lateralMax: 25, scaleMin: 0.85, scaleMax: 1.1 },
  { type: 'camera', count: 14, lateralMin: 22, lateralMax: 31, scaleMin: 0.85, scaleMax: 1.15 },
  { type: 'crowd', count: 42, lateralMin: 28, lateralMax: 39, scaleMin: 0.8, scaleMax: 1.25 },
  { type: 'flag', count: 24, lateralMin: 20, lateralMax: 28, scaleMin: 0.9, scaleMax: 1.15 },
  { type: 'lightGantry', count: 8, lateralMin: 20, lateralMax: 28, scaleMin: 0.95, scaleMax: 1.1 },
  { type: 'serviceVehicle', count: 10, lateralMin: 38, lateralMax: 60, scaleMin: 0.9, scaleMax: 1.15 }
]);

export const HARBOR_RING = Object.freeze({
  id: 'harbor-ring',
  name: 'Harbor Ring',
  description: 'A wide, fully flat circuit designed for braking duels, switchbacks and multi-class overtaking.',
  controlPoints: HARBOR_RING_CONTROL_POINTS,
  roadHalfWidth: 8.2,
  curbWidth: 1.25,
  runoffWidth: 13.0,
  sampleDensity: 34,
  elevation: Object.freeze({ baseM: 0, minM: 0, maxM: 0, authoredDeltaM: 0, profile: 'flat' }),
  bankHints: Object.freeze([]),
  sectors: Object.freeze([
    { id: 'harbor-straight', fromFraction: 0.00, toFraction: 0.235, character: 'high-speed', mainStraightM: 760 },
    { id: 'east-hairpin', fromFraction: 0.235, toFraction: 0.405, character: 'heavy-braking', complex: 'Dock Hairpin' },
    { id: 'infield', fromFraction: 0.405, toFraction: 0.690, character: 'technical', complex: 'Container Esses' },
    { id: 'west-loop', fromFraction: 0.690, toFraction: 0.865, character: 'medium-speed', complex: 'Warehouse Loop' },
    { id: 'final-chicane', fromFraction: 0.865, toFraction: 1.00, character: 'technical', complex: 'Harbor Chicane' }
  ]),
  start: Object.freeze({
    finishFraction: 0.020,
    gridFraction: 0.958,
    direction: 1,
    rows: 20,
    rowSpacingM: 8.8,
    laneOffsetM: 2.8
  }),
  pit: Object.freeze({
    side: 'right',
    entryFraction: 0.900,
    limiterFraction: 0.920,
    boxStartFraction: 0.946,
    boxEndFraction: 0.985,
    exitFraction: 0.070,
    lateralM: -13.8,
    entryLateralM: -10.8,
    exitLateralM: -9.0,
    pitSpeedLimitMps: 16.67,
    entrySpeedLimitMps: 23.5,
    boxSpeedMps: 1.1,
    stoppedSpeedMps: 0.55,
    serviceDurationS: 7.5
  }),
  landmarks: Object.freeze([
    { type: 'controlTower', fraction: 0.045, lateral: -40, headingOffset: 0 },
    { type: 'hospitality', fraction: 0.090, lateral: -50, headingOffset: 0 },
    { type: 'footbridge', fraction: 0.155, lateral: 0, spansTrack: true },
    { type: 'crane', fraction: 0.285, lateral: -46, headingOffset: 0 },
    { type: 'serviceRoad', fromFraction: 0.88, toFraction: 0.08, lateral: -44 }
  ]),
  scenery: Object.freeze({
    seed: 73124,
    bands: sceneryBands,
    expectedInstanceCount: sceneryBands.reduce((total, band) => total + band.count, 0),
    drawCallBudget: 32
  })
});

export function createHarborRingScenario() {
  return { ...HARBOR_RING };
}
