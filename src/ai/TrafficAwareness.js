/**
 * TrafficAwareness.js
 * High-performance AI perception and traffic evaluation engine.
 * Computes vehicle relative metrics, Frenet spatial occupancy, predictive swept corridors,
 * and time-to-collision (TTC) hazard assessments.
 */

const finite = (value, fallback = 0) => (Number.isFinite(value) ? value : fallback);

const clamp = (value, min, max) => Math.max(min, Math.min(max, value));

const wrap = (value, length) => {
  if (length <= 0) return 0;
  return ((value % length) + length) % length;
};

const wrapAngle = (angle) => {
  let result = (angle + Math.PI) % (Math.PI * 2);
  if (result < 0) result += Math.PI * 2;
  return result - Math.PI;
};

const smoothstep = (value) => {
  const u = clamp(value, 0, 1);
  return u * u * (3 - 2 * u);
};

export class TrafficAwareness {
  /**
   * @param {Object} options
   * @param {number} [options.longitudinalEnvelope=5.4] - Safety margin in longitudinal direction (meters)
   * @param {number} [options.lateralEnvelope=2.9] - Safety margin in lateral direction (meters)
   * @param {number} [options.bodyLength=4.6] - Standard car body length
   * @param {number} [options.bodyWidth=2.0] - Standard car body width
   */
  constructor({
    longitudinalEnvelope = 5.4,
    lateralEnvelope = 2.9,
    bodyLength = 4.6,
    bodyWidth = 2.0
  } = {}) {
    this.longitudinalEnvelope = longitudinalEnvelope;
    this.lateralEnvelope = lateralEnvelope;
    this.bodyLength = bodyLength;
    this.bodyWidth = bodyWidth;
  }

  /**
   * Scan surrounding vehicles and compute relative kinematic & Frenet metrics.
   * @param {Object} vehicle - Ego vehicle
   * @param {Array<Object>} vehicles - All vehicles on track
   * @param {Object} track - Track geometry and surface model
   * @returns {Object} Traffic perception summary
   */
  scan(vehicle, vehicles, track) {
    const current = vehicle.surface ?? (track?.surfaceAt ? track.surfaceAt(vehicle.position.x, vehicle.position.z) : { lateral: 0, s: vehicle.distance || 0 });
    const forward = vehicle.forward ?? { x: -Math.sin(vehicle.yaw || 0), z: Math.cos(vehicle.yaw || 0) };
    const right = vehicle.right ?? { x: forward.z, z: -forward.x };

    const vel = vehicle.velocity ?? { x: 0, z: 0 };
    const egoForwardSpeed = finite(vel.x * forward.x + vel.z * forward.z, finite(vehicle.speed, 0));
    const egoLateralSpeed = finite(vel.x * right.x + vel.z * right.z, 0);
    const trackLength = finite(track?.length, 1000);
    const entries = [];

    for (const other of vehicles || []) {
      if (!other || other === vehicle || other.finished || other.despawned || other.trafficGhost) {
        continue;
      }

      const dx = finite(other.position.x) - finite(vehicle.position.x);
      const dz = finite(other.position.z) - finite(vehicle.position.z);
      const direct = Math.hypot(dx, dz);
      const longitudinal = dx * forward.x + dz * forward.z;
      const side = dx * right.x + dz * right.z;

      const egoDist = finite(vehicle.distance, 0);
      const otherDist = finite(other.distance, 0);
      const delta = wrap(otherDist - egoDist + trackLength * 0.5, trackLength) - trackLength * 0.5;

      const otherVel = other.velocity ?? { x: 0, z: 0 };
      const otherForwardSpeed = finite(otherVel.x * forward.x + otherVel.z * forward.z, finite(other.speed, 0));
      const otherLateralSpeed = finite(otherVel.x * right.x + otherVel.z * right.z, 0);

      const closingSpeed = egoForwardSpeed - otherForwardSpeed;
      const bodyGap = longitudinal - this.longitudinalEnvelope * 0.5;

      const otherLateral = finite(other.surface?.lateral, side);
      const egoLateral = finite(current?.lateral, 0);
      const lateralDelta = otherLateral - egoLateral;

      const otherTargetLateral = finite(
        other.aiTactical?.targetLaneOffsetM,
        finite(other.aiTarget?.lateral, otherLateral)
      );

      const ttc = closingSpeed > 0.15 && bodyGap > 0
        ? bodyGap / closingSpeed
        : (closingSpeed > 0.15 && bodyGap <= 0 ? 0 : 99);

      const otherForward = other.forward ?? { x: -Math.sin(other.yaw || 0), z: Math.cos(other.yaw || 0) };
      const otherHeading = finite(other.yaw, Math.atan2(-otherForward.x, otherForward.z));
      const egoHeading = finite(vehicle.yaw, Math.atan2(-forward.x, forward.z));
      const relativeHeading = wrapAngle(otherHeading - egoHeading);
      const otherTrackPoint = track?.atDistance ? track.atDistance(otherDist) : null;
      const otherTrackAngle = otherTrackPoint?.tangent ? Math.atan2(otherTrackPoint.tangent.x, otherTrackPoint.tangent.z) : otherHeading;
      const otherNoseTrackDeviation = wrapAngle(otherHeading - otherTrackAngle);

      entries.push({
        other,
        delta,
        longitudinal,
        side,
        direct,
        lateralDelta,
        otherLateral,
        otherTargetLateral,
        otherForward,
        otherHeading,
        relativeHeading,
        otherNoseTrackDeviation,
        otherLateralSpeed,
        egoForwardSpeed,
        otherForwardSpeed,
        relativeSpeed: finite(vehicle.speed) - finite(other.speed),
        relativeLongitudinalVelocity: closingSpeed,
        relativeLateralVelocity: otherLateralSpeed - egoLateralSpeed,
        ttc: finite(ttc, 99)
      });
    }

    // Sort entries by longitudinal delta (closest behind -> closest ahead)
    entries.sort((a, b) => a.delta - b.delta);

    // Filter key reference vehicles
    const ahead = entries
      .filter((e) => e.delta > 0 && e.delta < 60 && e.longitudinal > -1.5 && Math.abs(e.side) < 5.0)
      .sort((a, b) => a.delta - b.delta)[0] ?? null;

    const behind = entries
      .filter((e) => e.delta < 0 && e.delta > -40 && e.longitudinal < 1.5 && Math.abs(e.side) < 5.5)
      .sort((a, b) => b.delta - a.delta)[0] ?? null;

    const alongside = entries
      .filter((e) => e.direct < 6.5 && Math.abs(e.longitudinal) < 5.0)
      .sort((a, b) => a.direct - b.direct)[0] ?? null;

    // Build predictive 8-quadrant spatial occupancy grid
    const occupancy = {
      frontLeft: [],
      frontCenter: [],
      frontRight: [],
      sideLeft: [],
      sideRight: [],
      rearLeft: [],
      rearCenter: [],
      rearRight: []
    };

    const timeHorizons = [0.5, 1.0, 2.0, 3.0];
    for (const entry of entries) {
      const longitudinalBand = entry.longitudinal > 4.5 ? 'front'
        : entry.longitudinal < -4.5 ? 'rear' : 'side';

      const lateralBand = entry.side > 1.5 ? 'Right'
        : entry.side < -1.5 ? 'Left' : 'Center';

      const key = longitudinalBand === 'side'
        ? (lateralBand === 'Left' ? 'sideLeft' : lateralBand === 'Right' ? 'sideRight' : null)
        : `${longitudinalBand}${lateralBand}`;

      if (!key || !occupancy[key]) continue;

      occupancy[key].push({
        id: entry.other.id,
        distanceM: entry.direct,
        deltaM: entry.delta,
        relativeLongitudinalVelocityMps: entry.relativeLongitudinalVelocity,
        predicted: timeHorizons.map((timeS) => ({
          timeS,
          longitudinalM: entry.longitudinal - entry.relativeLongitudinalVelocity * timeS,
          lateralM: entry.side + entry.relativeLateralVelocity * timeS
        }))
      });
    }

    return {
      current,
      entries,
      ahead,
      behind,
      alongside,
      occupancy,
      egoForwardSpeed
    };
  }

  /**
   * Evaluate if a target lateral offset corridor is collision-free and legal over a time horizon.
   * @param {Object} params
   * @returns {Object} Corridor evaluation result
   */
  evaluateCorridor({
    vehicle,
    track,
    traffic,
    terminalOffset,
    targetId = null,
    horizonS = 3.4,
    targetSpeed = vehicle.speed,
    kerbAllowance = 0
  }) {
    const startLateral = finite(traffic?.current?.lateral, 0);
    const halfWidth = finite(track?.roadHalfWidth, 6.5);
    const nominalRoadMargin = Math.max(2.1, halfWidth - 1.18 + kerbAllowance);
    const offset = clamp(finite(terminalOffset), -nominalRoadMargin, nominalRoadMargin);

    let legal = Math.abs(offset) <= nominalRoadMargin + 1e-4;
    let collisionFree = true;
    let minimumClearance = 99;
    let blocker = null;
    const samples = 20;

    const egoSpeed = Math.max(0, finite(vehicle.speed, 0));
    const accelEst = clamp((finite(targetSpeed, egoSpeed) - egoSpeed) * 0.45, -7.0, 5.0);

    for (let i = 0; i < samples; i += 1) {
      const time = (horizonS * i) / (samples - 1);
      const forwardDistance = Math.max(0, egoSpeed * time + 0.5 * accelEst * time * time);
      
      const transitionDuration = Math.max(0.7, Math.min(2.4, 0.7 + Math.abs(offset - startLateral) * 0.22));
      const blend = smoothstep(time / transitionDuration);
      const lateral = startLateral + (offset - startLateral) * blend;

      const egoDistance = finite(vehicle.distance, 0) + forwardDistance;
      const point = track?.atDistance ? track.atDistance(egoDistance) : { s: egoDistance };
      
      const surfaceLimit = finite(
        track?.planningLateralLimit?.(point.s, lateral),
        nominalRoadMargin
      ) + kerbAllowance;

      if (Math.abs(lateral) > surfaceLimit) {
        legal = false;
      }

      for (const entry of traffic?.entries || []) {
        if (!entry?.other || entry.other.finished || entry.other.despawned || entry.other.trafficGhost) {
          continue;
        }

        const opponentProgress = Math.max(0, finite(entry.other.speed, 0) * time);
        const longitudinalGap = entry.delta + opponentProgress - forwardDistance;
        const opponentStart = entry.otherLateral;
        const opponentTarget = entry.otherTargetLateral;
        const opponentLateral = opponentStart + (opponentTarget - opponentStart) * smoothstep(time / 1.3);

        const lateralGap = Math.abs(lateral - opponentLateral);
        const longitudinalClearance = Math.abs(longitudinalGap) - this.longitudinalEnvelope;
        const lateralClearance = lateralGap - this.lateralEnvelope;
        const clearance = Math.max(longitudinalClearance, lateralClearance);

        minimumClearance = Math.min(minimumClearance, clearance);

        const isPassTarget = targetId !== null && entry.other.id === targetId;
        const initialTargetSeparation = Math.abs(startLateral - opponentStart);
        const targetSeparatingOffset = isPassTarget && Math.abs(offset - opponentStart) >= 2.6;
        const separatingFromPassTarget = isPassTarget
          && (targetSeparatingOffset || (Math.abs(longitudinalGap) > 2.0 && lateralGap >= initialTargetSeparation - 0.1));

        const isSlowObstacle = isPassTarget && entry.other.speed < 15.0 && Math.abs(offset - opponentStart) >= 2.6;

        if (longitudinalClearance < 0 && lateralClearance < 0 && !separatingFromPassTarget && !isSlowObstacle) {
          collisionFree = false;
          if (!blocker || clearance < blocker.clearance) {
            blocker = { entry, clearance, time };
          }
        }
      }
    }

    const target = targetId ? traffic?.entries?.find((e) => e.other.id === targetId) : null;
    const targetSeparation = target ? Math.abs(offset - target.otherLateral) : 99;

    return {
      offset,
      legal,
      collisionFree: legal && collisionFree,
      minimumClearanceM: finite(minimumClearance, 99),
      blockerId: blocker?.entry?.other?.id ?? null,
      blockerTimeS: finite(blocker?.time, 99),
      targetSeparationM: targetSeparation
    };
  }

  /**
   * Assess imminent forward collision hazard requiring emergency braking or avoidance.
   * @param {Object} traffic - Result from scan()
   * @param {Object} [options]
   * @returns {Object|null} Most critical forward hazard
   */
  forwardHazard(traffic, { maximumTtc = 5.0, lateralEnvelope = 3.2 } = {}) {
    if (!traffic?.entries?.length) return null;

    return traffic.entries
      .filter((entry) => entry.longitudinal > 0 && entry.longitudinal < 60
        && entry.relativeLongitudinalVelocity > 0.2)
      .map((entry) => {
        const time = clamp(entry.ttc, 0.2, 3.5);
        const predictedSide = Math.abs(entry.side + entry.relativeLateralVelocity * time);
        return { ...entry, predictedSide };
      })
      .filter((entry) => entry.ttc < maximumTtc && entry.predictedSide < lateralEnvelope)
      .sort((a, b) => a.ttc - b.ttc)[0] ?? null;
  }

  /**
   * Assess rear threat closing from behind.
   * @param {Object} traffic - Result from scan()
   * @param {Object} [options]
   * @returns {Object|null} Threatening challenger behind
   */
  rearThreat(traffic, { maximumTtc = 4.0, maxDistance = 45 } = {}) {
    if (!traffic?.entries?.length) return null;

    return traffic.entries
      .filter((entry) => entry.delta < -1.5 && entry.delta > -maxDistance
        && entry.relativeLongitudinalVelocity < -0.4) // opponent is faster
      .map((entry) => {
        const closingSpeed = Math.abs(entry.relativeLongitudinalVelocity);
        const rearGap = Math.abs(entry.longitudinal) - this.longitudinalEnvelope * 0.5;
        const ttc = closingSpeed > 0.2 ? Math.max(0, rearGap) / closingSpeed : 99;
        return { ...entry, ttc, closingSpeed };
      })
      .filter((entry) => entry.ttc < maximumTtc)
      .sort((a, b) => a.ttc - b.ttc)[0] ?? null;
  }
}
