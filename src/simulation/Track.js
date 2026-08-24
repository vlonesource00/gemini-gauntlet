import { clamp, dot2, normalize2, wrap } from '../core/math.js';

const finiteNumber = (value, fallback = 0) => Number.isFinite(value) ? value : fallback;
const inWrappedFraction = (fraction, start, end) => {
  const f = wrap(fraction, 1);
  const a = wrap(start, 1);
  const b = wrap(end, 1);
  return a <= b ? f >= a && f <= b : f >= a || f <= b;
};

// Plan footprint remains authored in world metres. Elevation/banking layer is
// deterministic so lap distance and AI contracts retain their planar meaning.
const CONTROL_POINTS = [
  { x: -24, z: 151 }, { x: 42, z: 155 }, { x: 91, z: 128 },
  { x: 118, z: 74 }, { x: 111, z: 16 }, { x: 82, z: -37 },
  { x: 55, z: -94 }, { x: 8, z: -126 }, { x: -54, z: -121 },
  { x: -106, z: -92 }, { x: -132, z: -37 }, { x: -123, z: 18 },
  { x: -91, z: 62 }, { x: -105, z: 109 }, { x: -78, z: 142 }
];

function catmull(p0, p1, p2, p3, t) {
  const t2 = t * t;
  const t3 = t2 * t;
  return {
    x: 0.5 * ((2 * p1.x) + (-p0.x + p2.x) * t + (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 + (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3),
    y: 0.5 * ((2 * (p1.y ?? 0)) + (-(p0.y ?? 0) + (p2.y ?? 0)) * t + (2 * (p0.y ?? 0) - 5 * (p1.y ?? 0) + 4 * (p2.y ?? 0) - (p3.y ?? 0)) * t2 + (-(p0.y ?? 0) + 3 * (p1.y ?? 0) - 3 * (p2.y ?? 0) + (p3.y ?? 0)) * t3),
    z: 0.5 * ((2 * p1.z) + (-p0.z + p2.z) * t + (2 * p0.z - 5 * p1.z + 4 * p2.z - p3.z) * t2 + (-p0.z + 3 * p1.z - 3 * p2.z + p3.z) * t3)
  };
}

const elevationAtFraction = (fraction) => {
  const a = fraction * Math.PI * 2;
  return 2.45 * Math.sin(a * 1.12 - 0.46) + 1.15 * Math.sin(a * 3.25 + 1.1) + 0.55 * Math.cos(a * 5.4 - 0.2);
};

const normal3For = (tangent, normal, grade, bank) => {
  const gradeTan = Math.tan(grade);
  const bankTan = Math.tan(bank);
  const x = -tangent.x * gradeTan - normal.x * bankTan;
  const z = -tangent.z * gradeTan - normal.z * bankTan;
  const length = Math.hypot(x, 1, z) || 1;
  return { x: x / length, y: 1 / length, z: z / length };
};

export class Circuit {
  constructor(scenario = null) {
    this.scenario = scenario;
    this.controlPoints = scenario?.controlPoints ?? CONTROL_POINTS;
    this.sampleSteps = Math.max(8, Math.trunc(scenario?.sampleDensity ?? 18));
    this.roadHalfWidth = finiteNumber(scenario?.roadHalfWidth, 6.5);
    this.curbWidth = finiteNumber(scenario?.curbWidth, 1.05);
    this.runoffWidth = finiteNumber(scenario?.runoffWidth, 8.5);
    this.samples = [];
    this.length = 0;
    this.rubber = null;
    // The aggregate remains public for existing telemetry/tests.  Physics
    // additionally keeps a flat segment-major lane map so separated tyres do
    // not paint the same whole-width strip.
    this.rubberLaneCount = 9;
    this.rubberLanes = this.rubberLaneCount;
    this.rubberLaneWidth = 0;
    this.rubberMap = null;
    this.rubberRevision = 0;
    this.revision = 0;
    this._buildSamples();
  }

  _buildSamples() {
    const controlPoints = this.controlPoints;
    const count = controlPoints.length;
    const steps = this.sampleSteps;
    for (let i = 0; i < count; i += 1) {
      const p0 = controlPoints[(i - 1 + count) % count];
      const p1 = controlPoints[i];
      const p2 = controlPoints[(i + 1) % count];
      const p3 = controlPoints[(i + 2) % count];
      for (let j = 0; j < steps; j += 1) {
        this.samples.push({
          ...catmull(p0, p1, p2, p3, j / steps), s: 0, grade: 0, bank: 0,
          tangent: { x: 0, z: 1 }, normal: { x: -1, z: 0 }, normal3: { x: 0, y: 1, z: 0 },
          curvature: 0, turnSign: 0, curbSide: 0, turnStrength: 0
        });
      }
    }
    const points = this.samples;
    for (let i = 0; i < points.length; i += 1) {
      const a = points[i];
      const b = points[(i + 1) % points.length];
      if (i > 0) a.s = this.length;
      this.length += Math.hypot(b.x - a.x, b.z - a.z);
    }
    for (let i = 0; i < points.length; i += 1) {
      const point = points[i];
      const prev = points[(i - 1 + points.length) % points.length];
      const next = points[(i + 1) % points.length];
      const tangent = normalize2(next.x - prev.x, next.z - prev.z);
      point.tangent = tangent;
      point.normal = { x: -tangent.z, z: tangent.x };
      if (!this.scenario) point.y = elevationAtFraction(point.s / this.length);
    }
    for (let i = 0; i < points.length; i += 1) {
      const point = points[i];
      const prev = points[(i - 1 + points.length) % points.length];
      const next = points[(i + 1) % points.length];
      const dot = clamp(dot2(prev.tangent.x, prev.tangent.z, next.tangent.x, next.tangent.z), -1, 1);
      const angle = Math.acos(dot);
      const span = Math.max(0.1, Math.hypot(next.x - prev.x, next.z - prev.z));
      point.curvature = angle / span;
      const cross = prev.tangent.x * next.tangent.z - prev.tangent.z * next.tangent.x;
      point.turnSign = Math.abs(cross) > 0.001 ? Math.sign(cross) : 0;
      point.curbSide = point.curvature > 0.009 ? point.turnSign : 0;
      point.turnStrength = clamp(point.curvature / 0.09, 0, 1);
      const ds = Math.max(0.1, Math.hypot(next.x - prev.x, next.z - prev.z));
      point.grade = clamp(Math.atan2(next.y - prev.y, ds), -0.22, 0.22);
      const fraction = point.s / this.length;
      const isFlatProfile = this.scenario?.elevation?.profile === 'flat';
      const bankHint = (this.scenario?.bankHints ?? []).find((hint) => fraction >= hint.fromFraction && fraction <= hint.toFraction);
      const authoredBank = bankHint?.bankRadians ?? Math.sin(point.s * 0.026 + 0.6) * 0.045;
      point.bank = isFlatProfile
        ? 0
        : clamp(authoredBank + point.turnSign * point.turnStrength * 0.17, -0.22, 0.22);
      point.normal3 = normal3For(point.tangent, point.normal, point.grade, point.bank);
    }
    this.rubber = new Float32Array(points.length);
    this.rubberLaneWidth = (this.roadHalfWidth * 2) / this.rubberLaneCount;
    this.rubberMap = new Float32Array(points.length * this.rubberLaneCount);
    this.rubberRevision = 0;
    this.revision = 0;
  }

  _segmentFor(distance) {
    const s = wrap(distance, this.length);
    const points = this.samples;
    let index = 0;
    while (index < points.length - 1 && points[index + 1].s <= s) index += 1;
    const a = points[index];
    const b = points[(index + 1) % points.length];
    const nextS = index === points.length - 1 ? this.length : b.s;
    const t = clamp((s - a.s) / Math.max(0.001, nextS - a.s), 0, 1);
    return { a, b, index, t, s };
  }

  atDistance(distance) {
    const { a, b, index, t, s } = this._segmentFor(distance);
    const tangent = normalize2(a.tangent.x + (b.tangent.x - a.tangent.x) * t, a.tangent.z + (b.tangent.z - a.tangent.z) * t);
    const normal = { x: -tangent.z, z: tangent.x };
    const grade = a.grade + (b.grade - a.grade) * t;
    const bank = a.bank + (b.bank - a.bank) * t;
    return {
      x: a.x + (b.x - a.x) * t,
      y: a.y + (b.y - a.y) * t,
      z: a.z + (b.z - a.z) * t,
      tangent, normal, normal3: normal3For(tangent, normal, grade, bank),
      grade, bank,
      curvature: a.curvature + (b.curvature - a.curvature) * t,
      turnSign: a.turnSign, curbSide: a.curbSide,
      turnStrength: a.turnStrength + (b.turnStrength - a.turnStrength) * t,
      rubber: this.rubberAt(index, 0), s, index
    };
  }

  lateralPoint(point, lateral = 0, lift = 0) {
    const bank = point.bank ?? 0;
    return {
      x: point.x + point.normal.x * lateral,
      y: (point.y ?? 0) + Math.sin(bank) * lateral + lift,
      z: point.z + point.normal.z * lateral
    };
  }

  /**
   * Maximum legal centre position for an AI car footprint. Standard kerbs
   * are usable where the authored corner marks them; runoff and grass never
   * are. This keeps geometry and tactics on one surface contract instead of
   * a fixed, unnecessarily narrow corridor.
   */
  planningLateralLimit(distance, side = 0, { halfWidthM = 1.02, safetyM = 0.16 } = {}) {
    const point = this.atDistance(distance);
    const roadLimit = Math.max(1.8, this.roadHalfWidth - halfWidthM - safetyM);
    const signedSide = Math.sign(finiteNumber(side));
    const onAuthoredKerbSide = signedSide !== 0 && signedSide === Math.sign(finiteNumber(point.curbSide));
    const usableKerb = onAuthoredKerbSide
      ? Math.min(0.42, this.curbWidth * 0.32)
      : point.turnStrength < 0.12 ? Math.min(0.12, this.curbWidth * 0.1) : 0;
    return roadLimit + usableKerb;
  }

  closest(x, z) {
    let best = null;
    const points = this.samples;
    for (let i = 0; i < points.length; i += 1) {
      const a = points[i];
      const b = points[(i + 1) % points.length];
      const dx = b.x - a.x;
      const dz = b.z - a.z;
      const span2 = dx * dx + dz * dz || 1;
      const u = clamp(((x - a.x) * dx + (z - a.z) * dz) / span2, 0, 1);
      const px = a.x + dx * u;
      const pz = a.z + dz * u;
      const ex = x - px;
      const ez = z - pz;
      const distance2 = ex * ex + ez * ez;
      if (!best || distance2 < best.distance2) {
        const tangent = normalize2(dx, dz);
        const normal = { x: -tangent.z, z: tangent.x };
        const segmentLength = Math.sqrt(span2);
        const grade = a.grade + (b.grade - a.grade) * u;
        const bank = a.bank + (b.bank - a.bank) * u;
          best = {
          x: px, y: a.y + (b.y - a.y) * u, z: pz, distance2, tangent, normal,
          normal3: normal3For(tangent, normal, grade, bank), grade, bank,
          lateral: ex * normal.x + ez * normal.z,
          s: wrap(a.s + u * segmentLength, this.length), index: i, t: u,
          curvature: a.curvature + (b.curvature - a.curvature) * u,
          turnStrength: a.turnStrength + (b.turnStrength - a.turnStrength) * u,
          curbSide: a.curbSide
        };
      }
    }
    return best;
  }

  surfaceAt(x, z) {
    const closest = this.closest(x, z);
    const lateralAbs = Math.abs(closest.lateral);
    const roadEdge = this.roadHalfWidth;
    const curbEdge = roadEdge + this.curbWidth;
    const barrierEdge = curbEdge + this.runoffWidth;
    let zone = 'road';
    let grip = 1.08;
    let curbHeight = 0;
    if (lateralAbs > roadEdge) { zone = 'curb'; grip = 0.93; curbHeight = 0.055 + Math.sin(closest.s * 2.8) * 0.009; }
    if (lateralAbs > curbEdge) { zone = 'runoff'; grip = 0.62; curbHeight = 0; }
    if (lateralAbs > barrierEdge) { zone = 'grass'; grip = 0.39; curbHeight = -0.035; }
    const pit = this.scenario?.pit;
    const onPitArc = pit && inWrappedFraction(closest.s / this.length, pit.entryFraction, pit.exitFraction);
    const onPitLane = onPitArc && Math.abs(closest.lateral - finiteNumber(pit.lateralM, -13.2)) <= 3.25;
    if (onPitLane) { zone = 'pit'; grip = 1.01; curbHeight = 0; }
    const nextIndex = (closest.index + 1) % this.samples.length;
    const rubberA = this.rubberAt(closest.index, closest.lateral);
    const rubberB = this.rubberAt(nextIndex, closest.lateral);
    const rubber = rubberA + (rubberB - rubberA) * clamp(closest.t ?? 0, 0, 1);
    const roughness = zone === 'road'
      ? 0.003 + Math.sin(closest.s * 0.71 + closest.lateral * 2.3) * 0.002
      : zone === 'curb' ? 0.012 + Math.sin(closest.s * 4.1) * 0.006 : 0.018;
    const lateralHeight = Math.sin(closest.bank) * closest.lateral;
    const rubberGrip = zone === 'road' || zone === 'pit' ? rubber : 0;
    return {
      ...closest,
      zone, grip: clamp(grip + rubberGrip * 0.07, 0.25, 1.18), rubber,
      height: closest.y + lateralHeight + curbHeight + roughness,
      curbHeight, roughness,
      barrierDepth: onPitLane ? 0 : Math.max(0, lateralAbs - barrierEdge), roadEdge, curbEdge, barrierEdge
    };
  }

  rubberAt(index, lateral = 0) {
    if (!this.rubberMap?.length || !Number.isFinite(index)) return 0;
    const segmentCount = this.samples.length;
    const safeIndex = ((Math.trunc(index) % segmentCount) + segmentCount) % segmentCount;
    const lanePosition = clamp(
      (finiteNumber(lateral) + this.roadHalfWidth) / Math.max(0.001, this.rubberLaneWidth) - 0.5,
      0, this.rubberLaneCount - 1
    );
    const laneA = Math.floor(lanePosition);
    const laneB = Math.min(this.rubberLaneCount - 1, laneA + 1);
    const blend = lanePosition - laneA;
    const base = safeIndex * this.rubberLaneCount;
    const a = this.rubberMap[base + laneA] ?? 0;
    const b = this.rubberMap[base + laneB] ?? a;
    return clamp(a + (b - a) * blend, 0, 1);
  }

  updateTirePass(surfaceOrDistance, utilisation = 0, dt = 1 / 120, loadN = 0, slipEnergy = 0) {
    const index = typeof surfaceOrDistance === 'number'
      ? this._segmentFor(surfaceOrDistance).index
      : surfaceOrDistance?.index;
    if (!Number.isInteger(index) || index < 0 || index >= this.rubber.length) return 0;
    const safeUtilisation = clamp(utilisation, 0, 1.5);
    const safeDt = clamp(dt, 0, 0.1);
    const safeLoad = clamp(Number.isFinite(loadN) ? loadN : 0, 0, 12000);
    const safeSlipEnergy = clamp(Number.isFinite(slipEnergy) ? slipEnergy : 0, 0, 1.8e5);
    // Keep the original utilisation-driven gain as the dominant term, with a
    // small load/slip contribution so a loaded, sliding tyre paints more than
    // a lightly loaded rolling tyre without changing the grip cap.
    const energyFactor = clamp(safeSlipEnergy / 120000, 0, 1);
    const loadFactor = clamp(safeLoad / 5000, 0, 1.5);
    const deposit = safeUtilisation * safeDt * 0.008
      * (1 + loadFactor * 0.16 + energyFactor * 0.12);
    const lateral = typeof surfaceOrDistance === 'number' ? 0 : finiteNumber(surfaceOrDistance?.lateral);
    const lanePosition = clamp(
      (lateral + this.roadHalfWidth) / Math.max(0.001, this.rubberLaneWidth) - 0.5,
      0, this.rubberLaneCount - 1
    );
    const centreLane = Math.round(lanePosition);
    const laneBase = index * this.rubberLaneCount;
    const decay = safeDt * 0.0003;
    for (let lane = Math.max(0, centreLane - 2); lane <= Math.min(this.rubberLaneCount - 1, centreLane + 2); lane += 1) {
      const distance = Math.abs(lane - lanePosition);
      const falloff = distance < 0.5 ? 1 : distance < 1.5 ? 0.42 : 0.12;
      const offset = laneBase + lane;
      const current = this.rubberMap[offset] ?? 0;
      this.rubberMap[offset] = clamp(current + deposit * falloff - current * decay, 0, 1);
    }
    // Aggregate is a truthful lane average, preserving the original public
    // longitudinal representation for HUD and older integrations.
    let aggregate = 0;
    for (let lane = 0; lane < this.rubberLaneCount; lane += 1) aggregate += this.rubberMap[laneBase + lane];
    this.rubber[index] = clamp(aggregate / this.rubberLaneCount, 0, 1);
    this.rubberRevision += 1;
    this.revision = this.rubberRevision;
    return this.rubber[index];
  }

  targetSpeed(distance, skill = 1) {
    const current = this.atDistance(distance);
    const ahead = this.atDistance(distance + 25);
    const bend = Math.max(current.curvature, ahead.curvature);
    const banking = 1 + Math.abs(Math.sin(current.bank)) * 0.34;
    const gradeFactor = clamp(1 - current.grade * 0.22, 0.9, 1.08);
    const base = Math.sqrt(1.08 * banking * 9.81 / Math.max(bend, 0.0025));
    // Do not impose an arcade minimum corner speed. The old 20 m/s floor
    // commanded 4g+ through the tightest spline sections and guaranteed an
    // off-track excursion regardless of how good the steering controller was.
    return clamp(base * gradeFactor * (0.88 + skill * 0.16), 8, 68);
  }
}
