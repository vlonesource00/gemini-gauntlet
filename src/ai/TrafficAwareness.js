/**
 * TrafficAwareness.js
 * Modular High-Performance AI Perception, Pack Racing & Traffic Dynamics Engine.
 * Features:
 * - Multi-Vehicle Kinematics & Frenet Spatial Occupancy
 * - Pack Racing & Multi-Car Cluster Hazard Detection
 * - Dirty Air, Turbulence & Aerodynamic Downforce Deficit Modeling
 * - Opportunistic Dual-Flank (Left vs Right) Corridor Asphalt Evaluation
 * - Time-To-Collision (TTC) & Predictive Swept Corridors
 * - Off-Track Surface Detection & Rejoin Geometry
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
   * @param {Object} [options]
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
   * Scan surrounding vehicles and compute relative kinematic, pack, dirty-air & Frenet metrics.
   * @param {Object} vehicle - Ego vehicle
   * @param {Array<Object>} vehicles - All vehicles on track
   * @param {Object} track - Track geometry and surface model
   * @returns {Object} Comprehensive traffic perception summary
   */
  scan(vehicle, vehicles, track) {
    const current = vehicle.surface ?? (track?.surfaceAt ? track.surfaceAt(vehicle.position.x, vehicle.position.z) : { lateral: 0, s: vehicle.distance || 0 });
    const forward = vehicle.forward ?? { x: Math.sin(vehicle.yaw || 0), z: Math.cos(vehicle.yaw || 0) };
    const right = vehicle.right ?? { x: Math.cos(vehicle.yaw || 0), z: -Math.sin(vehicle.yaw || 0) };

    const vel = vehicle.velocity ?? { x: 0, z: 0 };
    const egoForwardSpeed = finite(vel.x * forward.x + vel.z * forward.z, finite(vehicle.speed, 0));
    const egoLateralSpeed = finite(vel.x * right.x + vel.z * right.z, 0);
    const trackLength = finite(track?.length, 1000);
    const entries = [];

    let totalWakeStrength = 0;
    let maxFrontDownforceLoss = 0;
    let maxRearDownforceLoss = 0;
    let dirtyAirSourceId = null;

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

      // Dirty air wake modeling from cars ahead (within 45m ahead, |lateral| < 3.8m)
      let wakeContribution = 0;
      let frontDfLoss = 0;
      let rearDfLoss = 0;
      if (delta > 1.8 && delta < 45.0 && Math.abs(side) < 4.2) {
        const longDistFactor = clamp(1.0 - delta / 45.0, 0, 1);
        const latAlignFactor = clamp(1.0 - Math.abs(side) / 3.8, 0, 1);
        const speedFactor = clamp((otherForwardSpeed - 6.0) / 35.0, 0, 1);
        wakeContribution = longDistFactor * latAlignFactor * speedFactor;
        
        // Front downforce suffers higher loss in turbulent wake (up to 35% loss) -> causes corner push / understeer
        frontDfLoss = wakeContribution * 0.35;
        rearDfLoss = wakeContribution * 0.22;

        totalWakeStrength = Math.max(totalWakeStrength, wakeContribution);
        if (frontDfLoss > maxFrontDownforceLoss) {
          maxFrontDownforceLoss = frontDfLoss;
          maxRearDownforceLoss = rearDfLoss;
          dirtyAirSourceId = other.id;
        }
      }

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
        ttc: finite(ttc, 99),
        wakeContribution,
        frontDfLoss,
        rearDfLoss
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

    // Pack Racing & Cluster Evaluation
    const aheadCars = entries.filter((e) => e.delta > 0 && e.delta < 75);
    const packCount = aheadCars.length;
    const isPackRacing = packCount >= 2;
    const packLead = aheadCars[aheadCars.length - 1] ?? null;
    const packTail = aheadCars[0] ?? null;
    const avgPackSpeed = packCount > 0
      ? aheadCars.reduce((sum, e) => sum + e.otherForwardSpeed, 0) / packCount
      : egoForwardSpeed;

    // Dirty Air summary
    const dirtyAir = {
      wakeStrength: clamp(totalWakeStrength, 0, 1),
      frontDownforceLoss: clamp(maxFrontDownforceLoss, 0, 0.38),
      rearDownforceLoss: clamp(maxRearDownforceLoss, 0, 0.25),
      sourceId: dirtyAirSourceId,
      isTurbulent: totalWakeStrength > 0.15,
      understeerMultiplier: 1.0 + totalWakeStrength * 0.45 // multiplier on understeer gradient in turns
    };

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

    // Multi-Car Relevance Scoring & Opponent Predictions (Phase 3 & 4)
    for (const entry of entries) {
      entry.attackOpportunity = this.scoreAttackOpportunity(vehicle, entry, track, entries);
      entry.defenseThreat = this.scoreDefenseThreat(vehicle, entry, track);
      entry.compositeRelevance = (entry.delta > 0) ? entry.attackOpportunity : entry.defenseThreat;
    }

    const primaryAttackTarget = entries
      .filter((e) => e.delta > 0.4 && e.delta < 65.0 && e.attackOpportunity > 0.05)
      .sort((a, b) => b.attackOpportunity - a.attackOpportunity)[0] ?? ahead;

    const primaryDefenseThreat = entries
      .filter((e) => e.delta < -0.4 && e.delta > -45.0 && e.defenseThreat > 0.05)
      .sort((a, b) => b.defenseThreat - a.defenseThreat)[0] ?? behind;

    // Top 3-5 relevant multi-car combatants
    const topRelevant = [...entries]
      .filter((e) => Math.abs(e.delta) < 70.0)
      .sort((a, b) => (b.compositeRelevance ?? 0) - (a.compositeRelevance ?? 0))
      .slice(0, 5);

    for (const entry of topRelevant) {
      entry.predictions = this.predictOpponentResponses(entry, track, {
        speed: egoForwardSpeed,
        distance: vehicle.distance,
        lateral: current?.lateral ?? 0
      });
    }

    // Corridor blockers: vehicles ahead within 35m that restrict passing corridors
    const corridorBlockers = entries.filter((e) => {
      if (e.delta <= 0.5 || e.delta > 35.0) return false;
      const latDist = Math.abs(finite(e.otherLateral, 0) - (current?.lateral ?? 0));
      return latDist < 3.2;
    });

    return {
      current,
      entries,
      ahead,
      behind,
      alongside,
      primaryAttackTarget,
      primaryDefenseThreat,
      topRelevant,
      corridorBlockers,
      occupancy,
      egoForwardSpeed,
      egoLateralSpeed,
      pack: {
        isPackRacing,
        count: packCount,
        lead: packLead,
        tail: packTail,
        avgSpeed: avgPackSpeed
      },
      dirtyAir
    };
  }

  /**
   * Opportunistic Dual-Flank Corridor Evaluation:
   * Evaluates both Left and Right corridors against defender positioning, asphalt width, and upcoming turns.
   * @param {Object} params
   * @returns {Object} Dual-flank analysis with recommended attack corridor
   */
  evaluateDualFlanks({
    vehicle,
    track,
    traffic,
    target,
    roadMargin = 5.2,
    kerbAllowance = 0,
    nextTurn = null
  }) {
    if (!target) {
      return {
        bestFlank: 'CENTER',
        recommendedOffset: 0,
        leftScore: 0,
        rightScore: 0,
        leftOffset: 0,
        rightOffset: 0,
        targetLateral: 0,
        insideFlank: 'NONE'
      };
    }

    const targetLateral = finite(target.otherLateral, 0);
    const targetLatVel = finite(target.otherLateralSpeed, 0);
    const halfWidth = Math.max(2.1, roadMargin + kerbAllowance);

    // Game visual coordinate convention: +lateral is RIGHT (toward +halfWidth), -lateral is LEFT (toward -halfWidth)
    // Left Flank: corridor on the left of target (-lateral, down to -halfWidth)
    // Right Flank: corridor on the right of target (+lateral, up to +halfWidth)
    const leftSpace = halfWidth + targetLateral;   // width available on left (from target down to -halfWidth)
    const rightSpace = halfWidth - targetLateral;  // width available on right (from target up to +halfWidth)

    // Minimum 1 car width clearance (~2.4m with safety margin)
    const minPassWidth = 2.4;
    const leftFeasible = leftSpace >= minPassWidth;
    const rightFeasible = rightSpace >= minPassWidth;

    // Desired offsets for both flanks (-lateral is left, +lateral is right)
    const leftOffset = clamp(targetLateral - Math.min(3.8, Math.max(minPassWidth, leftSpace * 0.65)), -halfWidth, halfWidth);
    const rightOffset = clamp(targetLateral + Math.min(3.8, Math.max(minPassWidth, rightSpace * 0.65)), -halfWidth, halfWidth);

    // Turn direction: turnSign < 0 is left turn (inside is -lateral), turnSign > 0 is right turn (inside is +lateral)
    const turnSign = Math.sign(finite(nextTurn?.turnSign, 0));
    const insideFlank = turnSign < 0 ? 'LEFT' : (turnSign > 0 ? 'RIGHT' : 'NONE');

    // Scoring factors:
    // 1. Available width (more width = higher safety and speed)
    let leftScore = leftFeasible ? leftSpace * 1.5 : -100;
    let rightScore = rightFeasible ? rightSpace * 1.5 : -100;

    // 2. Defender momentum (if defender drifting right (+lat), left opens up; if defender drifting left (-lat), right opens up)
    if (targetLatVel > 0.08) {
      leftScore += 2.5;  // defender drifting right (+lat) -> left corridor (-lat) opening
      rightScore -= 2.0;
    } else if (targetLatVel < -0.08) {
      rightScore += 2.5; // defender drifting left (-lat) -> right corridor (+lat) opening
      leftScore -= 2.0;
    }

    // 3. Inside apex preference into upcoming corner (inside line gets massive advantage)
    if (insideFlank === 'LEFT') {
      leftScore += 3.5;
    } else if (insideFlank === 'RIGHT') {
      rightScore += 3.5;
    }

    // 4. Current ego lateral alignment
    const currentLat = finite(traffic?.current?.lateral, 0);
    if (currentLat < targetLateral) {
      leftScore += 1.0; // already on left side (-lat)
    } else {
      rightScore += 1.0; // already on right side (+lat)
    }

    const bestFlank = leftScore >= rightScore ? (leftFeasible ? 'LEFT' : (rightFeasible ? 'RIGHT' : 'CENTER'))
      : (rightFeasible ? 'RIGHT' : (leftFeasible ? 'LEFT' : 'CENTER'));

    const recommendedOffset = bestFlank === 'LEFT' ? leftOffset : (bestFlank === 'RIGHT' ? rightOffset : 0);

    return {
      bestFlank,
      recommendedOffset,
      leftScore,
      rightScore,
      leftOffset,
      rightOffset,
      leftSpace,
      rightSpace,
      leftFeasible,
      rightFeasible,
      targetLateral,
      insideFlank
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

  /**
   * Score tactical attack opportunity for a car ahead.
   * @param {Object} egoVehicle - Ego car
   * @param {Object} entry - Traffic entry of rival ahead
   * @param {Object} track - Circuit
   * @returns {number} Opportunity score in [0, 1]
   */
  scoreAttackOpportunity(egoVehicle, entry, track, allEntries = []) {
    if (!entry || entry.delta <= 0.2 || entry.delta > 65.0) return 0;
    const gap = entry.delta;
    const closingSpeed = finite(entry.relativeLongitudinalVelocity, 0); // ego - other
    const ttc = finite(entry.ttc, 99);

    const fGap = Math.exp(-gap / 22.0);
    const fClose = clamp((closingSpeed + 0.8) / 4.5, 0, 1.5);
    const fTtc = ttc < 5.0 ? Math.pow(1.0 - ttc / 5.0, 1.5) : 0;
    const fDraft = entry.wakeContribution > 0.1 ? 1.0 + entry.wakeContribution * 0.75 : 1.0;

    const otherLat = finite(entry.otherLateral, 0);
    const roadHalfWidth = finite(track?.roadHalfWidth, 8.2);
    const legalMargin = Math.max(3.5, roadHalfWidth - 1.2);
    const leftRoom = Math.max(0, legalMargin + otherLat);
    const rightRoom = Math.max(0, legalMargin - otherLat);

    // Corridor clearance required for ego body
    const requiredRoom = this.bodyWidth + 0.9;
    const leftFeasible = leftRoom >= requiredRoom;
    const rightFeasible = rightRoom >= requiredRoom;
    if (!leftFeasible && !rightFeasible) {
      return 0.02; // Corridor pinched, virtually zero tactical passing opportunity
    }

    // Corner geometry bonus: inside line is tactically privileged
    const egoDist = finite(egoVehicle?.distance, 0);
    const trackPt = track?.atDistance ? track.atDistance(egoDist + gap) : null;
    const turnSign = finite(trackPt?.turnSign, 0);
    let fApex = 1.0;
    if (turnSign > 0) { // Right turn: inside is positive lateral (right)
      if (rightRoom >= requiredRoom) fApex = 1.25;
    } else if (turnSign < 0) { // Left turn: inside is negative lateral (left)
      if (leftRoom >= requiredRoom) fApex = 1.25;
    }

    // Check downstream corridor obstruction by third-party vehicles
    let fBlocker = 1.0;
    if (Array.isArray(allEntries) && allEntries.length > 1) {
      for (const other of allEntries) {
        if (!other || other === entry) continue;
        // Check if third party is downstream of this rival (gap < other.delta < gap + 28)
        if (other.delta > gap && other.delta < gap + 28.0) {
          const lateralOverlap = Math.abs(finite(other.otherLateral, 0) - otherLat) < 2.2;
          if (lateralOverlap) {
            fBlocker = Math.min(fBlocker, 0.45); // Heavy penalty for attacking into a packed blocker
          }
        }
      }
    }

    const bestRoom = Math.max(leftRoom, rightRoom);
    const fRoom = clamp(bestRoom / 3.2, 0.2, 1.2);

    return clamp((fGap * 0.35 + fClose * 0.35 + fTtc * 0.20) * fDraft * fRoom * fApex * fBlocker, 0, 1);
  }

  /**
   * Score defensive threat for a challenger behind.
   * @param {Object} egoVehicle - Ego car
   * @param {Object} entry - Traffic entry of challenger behind
   * @param {Object} track - Circuit
   * @returns {number} Defense threat score in [0, 1]
   */
  scoreDefenseThreat(egoVehicle, entry, track) {
    if (!entry || entry.delta >= -0.3 || entry.delta < -45.0) return 0;
    const gap = Math.abs(entry.delta);
    const closingSpeed = -finite(entry.relativeLongitudinalVelocity, 0); // challenger - ego
    const ttc = closingSpeed > 0.15 ? Math.max(0, gap - this.bodyLength) / closingSpeed : 99.0;

    const fGap = Math.exp(-gap / 16.0);
    const fClose = clamp((closingSpeed + 0.3) / 4.0, 0, 1.5);
    const fTtc = ttc < 4.5 ? Math.pow(1.0 - ttc / 4.5, 2) : 0;

    const trackPt = track?.atDistance ? track.atDistance(egoVehicle?.distance || 0) : null;
    const isCornering = Math.abs(finite(trackPt?.curvature, 0)) > 0.003;
    const fCorner = isCornering ? 1.35 : 1.0;

    return clamp((fGap * 0.35 + fClose * 0.35 + fTtc * 0.30) * fCorner, 0, 1);
  }

  /**
   * Generate opponent response hypotheses with time-dependent Frenet occupancy envelopes.
   * @param {Object} entry - Opponent traffic entry
   * @param {Object} track - Track geometry
   * @param {Object} egoState - Ego state
   * @returns {Array<Object>} Hypotheses with spatio-temporal bounding boxes
   */
  predictOpponentResponses(entry, track, egoState = {}) {
    if (!entry || !entry.other) return [];

    const otherLat = finite(entry.otherLateral, 0);
    const otherLatVel = finite(entry.otherLateralSpeed, 0);
    const otherSpeed = finite(entry.otherForwardSpeed, 25);
    const otherDist = finite(entry.other.distance, 0);
    const trackLength = finite(track?.length, 1000);

    const trackPoint = track?.atDistance ? track.atDistance(otherDist) : null;
    const lookaheadPoint = track?.atDistance ? track.atDistance(otherDist + 30) : null;
    const turnSign = finite(trackPoint?.turnSign, 0);
    const lookaheadTurnSign = finite(lookaheadPoint?.turnSign, 0);
    const insideSign = turnSign !== 0 ? turnSign : (lookaheadTurnSign !== 0 ? lookaheadTurnSign : 0);

    const isApproachingCorner = Math.abs(finite(lookaheadPoint?.curvature, 0)) > 0.003
      || Math.abs(finite(trackPoint?.curvature, 0)) > 0.003;
    const isOtherBraking = Boolean(entry.other?.controls?.brake > 0.08 || entry.other?.brake > 0.08);

    const horizons = [0.4, 0.8, 1.4, 2.2];

    // Helper for piecewise kinematic longitudinal integration: deltaS = int_0^t v(tau) dtau
    const integrateDecel = (v0, accel, t, vMin = 8.0) => {
      if (t <= 0) return 0;
      if (accel >= 0) return v0 * t + 0.5 * accel * t * t;
      const tStop = Math.max(0, (v0 - vMin) / Math.abs(accel));
      if (t <= tStop) {
        return v0 * t + 0.5 * accel * t * t;
      }
      const distDecel = v0 * tStop + 0.5 * accel * tStop * tStop;
      return distDecel + vMin * (t - tStop);
    };

    const makeEnvelopes = (latProfileFn, speedProfileFn, distDeltaFn) => {
      return horizons.map((t) => {
        const predLat = latProfileFn(t);
        const predSpeed = speedProfileFn(t);
        const deltaS = distDeltaFn ? distDeltaFn(t) : predSpeed * t;
        const predDist = wrap(otherDist + deltaS, trackLength);

        const latUncertainty = 0.25 + 0.35 * t;
        const longUncertainty = 0.50 + 0.90 * t;

        return {
          timeS: t,
          sMin: predDist - this.bodyLength * 0.5 - longUncertainty,
          sMax: predDist + this.bodyLength * 0.5 + longUncertainty,
          qMin: predLat - this.bodyWidth * 0.5 - latUncertainty,
          qMax: predLat + this.bodyWidth * 0.5 + latUncertainty,
          centerS: predDist,
          centerQ: predLat,
          speed: predSpeed
        };
      });
    };

    // Calculate dynamic context weights for hypothesis probabilities
    let wHold = 0.35;
    let wReturn = 0.25;
    let wInside = 0.20;
    let wOutside = 0.08;
    let wBrakeEarly = 0.05;
    let wBrakeNormal = 0.05;
    let wOvershoot = 0.02;

    const movingInside = insideSign !== 0 && (
      (insideSign > 0 && otherLatVel > 0.15) ||
      (insideSign < 0 && otherLatVel < -0.15)
    );
    const movingOutside = insideSign !== 0 && (
      (insideSign > 0 && otherLatVel < -0.15) ||
      (insideSign < 0 && otherLatVel > 0.15)
    );
    const stableLane = Math.abs(otherLatVel) < 0.12;

    if (movingInside) {
      wInside *= 3.2;
      wReturn *= 0.4;
      wOutside *= 0.3;
    } else if (movingOutside) {
      wOutside *= 2.8;
      wReturn *= 0.5;
      wInside *= 0.4;
    }

    if (isOtherBraking) {
      wBrakeNormal *= 3.0;
      wBrakeEarly *= 2.5;
      wOvershoot *= 0.25;
      wHold *= 0.6;
    } else if (isApproachingCorner && otherSpeed > 28) {
      wOvershoot *= 2.5;
      wBrakeNormal *= 2.2;
      wHold *= 0.7;
    } else if (stableLane) {
      wHold *= 1.8;
    }

    if (Math.abs(otherLat) > 2.5 && !movingInside) {
      wReturn *= 1.8;
    }

    const hypotheses = [
      {
        id: 'HOLD_LINE',
        probability: wHold,
        description: 'Maintains current lateral position and steady speed',
        envelopes: makeEnvelopes(
          (t) => otherLat + otherLatVel * t * Math.exp(-t / 1.2),
          (t) => otherSpeed,
          (t) => otherSpeed * t
        )
      },
      {
        id: 'RETURN_TO_RACING_LINE',
        probability: wReturn,
        description: 'Drifts smoothly toward nominal racing line',
        envelopes: makeEnvelopes(
          (t) => otherLat * Math.exp(-t / 1.0),
          (t) => otherSpeed,
          (t) => otherSpeed * t
        )
      },
      {
        id: 'DEFEND_INSIDE',
        probability: wInside,
        description: 'Squeezes toward inside apex curb to defend corner',
        envelopes: makeEnvelopes(
          (t) => {
            const targetInside = insideSign !== 0 ? insideSign * 3.5 : (otherLat >= 0 ? 3.0 : -3.0);
            const blend = 1.0 - Math.exp(-t / 0.8);
            return otherLat + (targetInside - otherLat) * blend;
          },
          (t) => Math.max(10, otherSpeed - 2.0 * t),
          (t) => integrateDecel(otherSpeed, -2.0, t, 10.0)
        )
      },
      {
        id: 'DEFEND_OUTSIDE',
        probability: wOutside,
        description: 'Carries momentum on the outside perimeter',
        envelopes: makeEnvelopes(
          (t) => {
            const targetOutside = insideSign !== 0 ? -insideSign * 3.8 : (otherLat >= 0 ? -3.5 : 3.5);
            const blend = 1.0 - Math.exp(-t / 1.1);
            return otherLat + (targetOutside - otherLat) * blend;
          },
          (t) => otherSpeed + 1.0 * t,
          (t) => otherSpeed * t + 0.5 * 1.0 * t * t
        )
      },
      {
        id: 'BRAKE_EARLY',
        probability: wBrakeEarly,
        description: 'Conservative early braking before corner entry',
        envelopes: makeEnvelopes(
          (t) => otherLat,
          (t) => Math.max(8, otherSpeed - 6.5 * t),
          (t) => integrateDecel(otherSpeed, -6.5, t, 8.0)
        )
      },
      {
        id: 'BRAKE_NORMAL',
        probability: wBrakeNormal,
        description: 'Standard threshold braking at baseline marker',
        envelopes: makeEnvelopes(
          (t) => otherLat,
          (t) => (t < 0.6 ? otherSpeed : Math.max(12, otherSpeed - 8.0 * (t - 0.6))),
          (t) => {
            if (t <= 0.6) return otherSpeed * t;
            return otherSpeed * 0.6 + integrateDecel(otherSpeed, -8.0, t - 0.6, 12.0);
          }
        )
      },
      {
        id: 'LATE_BRAKE_OVERSHOOT',
        probability: wOvershoot,
        description: 'Aggressive deep entry with apex overshoot risk',
        envelopes: makeEnvelopes(
          (t) => otherLat + (insideSign > 0 ? -1.5 : 1.5) * clamp(t - 0.8, 0, 1.5),
          (t) => Math.max(14, otherSpeed - 4.0 * t),
          (t) => integrateDecel(otherSpeed, -4.0, t, 14.0)
        )
      }
    ];

    // Normalize probabilities to sum to 1.0
    const totalProb = hypotheses.reduce((sum, h) => sum + h.probability, 0);
    for (const h of hypotheses) {
      h.probability = totalProb > 0 ? (h.probability / totalProb) : (1.0 / hypotheses.length);
    }

    return hypotheses;
  }
}
