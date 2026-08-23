/**
 * ScenarioEngine.js
 *
 * Orchestrates tactical racing scenarios, initial vehicle state deployment,
 * real-time relative telemetry tracking (gap, clearance, slipstream, ERS),
 * pass quality grading, and success/failure resolution.
 */

import { SCENARIOS, SCENARIO_LIST, SCENARIOS_BY_ID, getScenarioById, getScenariosByCategory } from './ScenarioDefinitions.js';

const TAU = Math.PI * 2;

/**
 * Calculates the shortest signed distance from one track distance to another
 * on a closed loop circuit of given length.
 * Positive returned value means `toDist` is ahead of `fromDist`.
 * Negative returned value means `toDist` is behind `fromDist`.
 */
export function signedTrackGap(fromDist, toDist, trackLength) {
  if (!trackLength || trackLength <= 0) return 0;
  const gap = toDist - fromDist;
  const half = trackLength * 0.5;
  return ((gap + half) % trackLength + trackLength) % trackLength - half;
}

export function isLegalSurface(vehicle) {
  if (!vehicle || !vehicle.surface) return true;
  const zone = vehicle.surface.zone;
  return zone === 'road' || zone === 'kerb';
}

/**
 * PassQualityTracker tracks overtaking attempts between vehicle pairs,
 * detecting whether passes are completed cleanly without contact or off-track excursions.
 */
export class PassQualityTracker {
  constructor(track) {
    this.track = track;
    this.reset();
  }

  reset() {
    this.pairs = new Map();
    this.cleanPasses = 0;
    this.rejectedContact = 0;
    this.rejectedOffTrack = 0;
    this.candidates = 0;
    this.latest = null;
  }

  update(vehicles, collisionStats, dt, active = true) {
    if (!active || !vehicles || vehicles.length < 2) return this.snapshot();
    const trackLength = this.track?.length || 3200;
    const contactKeys = new Set((collisionStats?.contactPairs ?? []).map(({ a, b }) => [a, b].sort().join('|')));

    for (let firstIndex = 0; firstIndex < vehicles.length; firstIndex += 1) {
      for (let secondIndex = firstIndex + 1; secondIndex < vehicles.length; secondIndex += 1) {
        const first = vehicles[firstIndex];
        const second = vehicles[secondIndex];
        if (!first || !second || first.finished || second.finished || first.retired || second.retired) continue;

        const key = [first.id, second.id].sort().join('|');
        const gap = signedTrackGap(first.distance, second.distance, trackLength);
        const state = this.pairs.get(key) ?? { previousGap: gap, attempt: null };
        const firstClosing = gap > 0 && gap < 20 && gap < state.previousGap - 0.01;
        const secondClosing = gap < 0 && gap > -20 && gap > state.previousGap + 0.01;

        if (!state.attempt && ((state.previousGap > 0 && gap <= 0) || firstClosing)) {
          state.attempt = {
            attacker: first,
            defender: second,
            taintedContact: contactKeys.has(key),
            attackerOff: !isLegalSurface(first),
            defenderOff: !isLegalSurface(second),
            clearTime: 0,
            crossed: gap <= 0,
            time: 0
          };
          this.candidates += 1;
        } else if (!state.attempt && ((state.previousGap < 0 && gap >= 0) || secondClosing)) {
          state.attempt = {
            attacker: second,
            defender: first,
            taintedContact: contactKeys.has(key),
            attackerOff: !isLegalSurface(second),
            defenderOff: !isLegalSurface(first),
            clearTime: 0,
            crossed: gap >= 0,
            time: 0
          };
          this.candidates += 1;
        }

        const attempt = state.attempt;
        if (attempt) {
          attempt.taintedContact ||= contactKeys.has(key);
          attempt.attackerOff ||= !isLegalSurface(attempt.attacker);
          attempt.defenderOff ||= !isLegalSurface(attempt.defender);
          const clearGap = signedTrackGap(attempt.defender.distance, attempt.attacker.distance, trackLength);
          attempt.time += dt;
          attempt.crossed ||= clearGap >= 0;
          attempt.clearTime = clearGap > 5 ? attempt.clearTime + dt : 0;

          if (attempt.clearTime >= 1) {
            const clean = !attempt.taintedContact && !attempt.attackerOff && !attempt.defenderOff;
            if (clean) this.cleanPasses += 1;
            else if (attempt.taintedContact) this.rejectedContact += 1;
            else this.rejectedOffTrack += 1;

            this.latest = Object.freeze({
              attackerId: attempt.attacker.id,
              defenderId: attempt.defender.id,
              clean,
              reason: clean ? 'CLEAN_DURABLE_PASS' : attempt.taintedContact ? 'CONTACT' : 'OFF_TRACK',
              clearanceM: clearGap
            });
            state.attempt = null;
          } else if ((attempt.crossed && clearGap < -8) || (!attempt.crossed && attempt.time > 20)) {
            state.attempt = null;
          }
        }
        state.previousGap = gap;
        this.pairs.set(key, state);
      }
    }
    return this.snapshot();
  }

  snapshot() {
    return Object.freeze({
      cleanPasses: this.cleanPasses,
      rejectedContact: this.rejectedContact,
      rejectedOffTrack: this.rejectedOffTrack,
      candidates: this.candidates,
      latest: this.latest
    });
  }
}

/**
 * Accurately sets a vehicle's position, heading, velocity vector, wheel angular
 * velocities, matching transmission gear and engine RPM for a given target speed.
 */
function setVehicleState(vehicle, track, { distance = 0, lateral = 0, speedKph = 0, ersMode = null } = {}) {
  if (!vehicle || !track) return;

  // Position and heading alignment
  vehicle.resetTo(track, distance, lateral);

  const speedMs = (Number.isFinite(speedKph) ? speedKph : 0) / 3.6;
  const point = track.atDistance(vehicle.distance);
  const tx = point.tangent?.x ?? 0;
  const ty = point.tangent?.y ?? 0;
  const tz = point.tangent?.z ?? 1;

  vehicle.speed = speedMs;
  vehicle.velocity = {
    x: tx * speedMs,
    y: ty * speedMs,
    z: tz * speedMs
  };
  vehicle.localVelocity = { x: 0, z: speedMs };
  vehicle.acceleration = { x: 0, y: 0, z: 0 };
  vehicle.localAcceleration = { x: 0, z: 0 };
  vehicle.angularVelocity = { x: 0, y: 0, z: 0 };
  vehicle.yawRate = 0;
  vehicle.steering = 0;
  vehicle.brakePressure = 0;

  // Spin wheels to match ground speed
  const wheelRadius = vehicle.wheelRadius || 0.335;
  const wheelOmega = speedMs / wheelRadius;
  if (Array.isArray(vehicle.wheels)) {
    for (const wheel of vehicle.wheels) {
      wheel.omega = wheelOmega;
      wheel.slip = 0;
      wheel.slipRatio = 0;
      wheel.slipAngle = 0;
    }
  }

  // Select appropriate gear and compute engine RPM in optimal powerband
  if (vehicle.spec?.gearRatios && Array.isArray(vehicle.spec.gearRatios)) {
    const ratios = vehicle.spec.gearRatios;
    const finalDrive = vehicle.spec.finalDrive || 3.5;
    const limiter = vehicle.spec.limiterRpm || 8500;
    const targetRpm = vehicle.spec.torquePeakRpm || 5500;
    const idle = vehicle.spec.idleRpm || 1100;
    const roadOmega = speedMs / wheelRadius;

    let bestGear = 1;
    let bestRpm = idle;
    let bestRpmDiff = Infinity;

    if (speedMs > 2) {
      for (let g = 1; g < ratios.length; g += 1) {
        const rpm = roadOmega * ratios[g] * finalDrive * 9.5493; // 60 / (2 * PI)
        if (rpm >= 2200 && rpm <= limiter) {
          const diff = Math.abs(rpm - targetRpm);
          if (diff < bestRpmDiff) {
            bestRpmDiff = diff;
            bestGear = g;
            bestRpm = rpm;
          }
        }
      }

      if (!Number.isFinite(bestRpmDiff) || bestRpmDiff === Infinity) {
        for (let g = ratios.length - 1; g >= 1; g -= 1) {
          const rpm = roadOmega * ratios[g] * finalDrive * 9.5493;
          if (rpm <= limiter + 400) {
            bestGear = g;
            bestRpm = Math.max(idle, rpm);
            break;
          }
        }
      }
    }

    vehicle.gear = bestGear;
    vehicle.rpm = Math.max(idle, bestRpm);
    vehicle.engineOmega = vehicle.rpm * TAU / 60;
  }

  // Set ERS mode if applicable
  if (ersMode && typeof vehicle.setERSMode === 'function') {
    vehicle.setERSMode(ersMode);
  }
}

export class ScenarioEngine {
  constructor(track, player = null, aiVehicle = null, aiController = null) {
    this.track = track;
    this.player = player;
    this.aiVehicle = aiVehicle;
    this.aiController = aiController;

    this.passQualityTracker = track ? new PassQualityTracker(track) : null;
    this.activeScenario = null;
    this.status = 'IDLE'; // 'IDLE' | 'RUNNING' | 'COMPLETED' | 'FAILED' | 'TIMEOUT'
    this.timeElapsed = 0;
    this.completed = false;
    this.passSuccess = false;
    this.finishReason = '';

    this.minClearance = Infinity;
    this.contactCount = 0;
    this.maxClosingSpeed = 0;
    this.avgGap = 0;
    this.currentGap = 0;
    this.initialGapSign = 0;
    this.gapSamples = [];
    this.previousGap = 0;

    this.draftTime = 0;
    this.ersEnergyStart = 0;
    this.ersUsed = 0;
    this.offTrackTime = 0;
    this.leadHoldTimer = 0;
    this.lapDistanceTravelled = 0;

    this.qualityScore = 100;
    this.safetyScore = 100;
    this.eventLog = [];
  }

  /**
   * Loads and initializes a scenario by preset ID or scenario object.
   * Resets vehicles, AI controller, trackers and telemetry.
   */
  loadScenario(scenarioIdOrObject) {
    const scenario = typeof scenarioIdOrObject === 'string'
      ? getScenarioById(scenarioIdOrObject)
      : scenarioIdOrObject;

    if (!scenario) {
      throw new Error(`Scenario not found: ${scenarioIdOrObject}`);
    }

    this.activeScenario = scenario;
    this.status = 'RUNNING';
    this.timeElapsed = 0;
    this.completed = false;
    this.passSuccess = false;
    this.finishReason = '';

    this.minClearance = Infinity;
    this.contactCount = 0;
    this.maxClosingSpeed = 0;
    this.avgGap = 0;
    this.currentGap = 0;
    this.gapSamples = [];
    this.previousGap = 0;

    this.draftTime = 0;
    this.ersUsed = 0;
    this.offTrackTime = 0;
    this.leadHoldTimer = 0;
    this.lapDistanceTravelled = 0;

    this.qualityScore = 100;
    this.safetyScore = 100;
    this.eventLog = [];

    if (this.passQualityTracker) {
      this.passQualityTracker.reset();
    }

    // 1. Configure and place Player vehicle
    if (this.player) {
      if (scenario.playerConfig) {
        if (scenario.playerConfig.spec && typeof this.player.setSpec === 'function') {
          this.player.setSpec(scenario.playerConfig.spec);
        }
        setVehicleState(this.player, this.track, {
          distance: scenario.playerConfig.distance ?? 0,
          lateral: scenario.playerConfig.lateralOffset ?? 0,
          speedKph: scenario.playerConfig.initialSpeedKph ?? 0,
          ersMode: scenario.playerConfig.ersMode ?? null
        });
        this.player.cooldownActive = false;
        this.player.finished = false;
        this.player.retired = false;
      } else {
        // Player not involved in scenario (e.g. solo hotlap)
        this.player.place(-9999, -9999, 0, -100);
        this.player.cooldownActive = true;
      }
    }

    // 2. Configure and place AI vehicle
    if (this.aiVehicle && scenario.aiConfig) {
      if (scenario.aiConfig.spec && typeof this.aiVehicle.setSpec === 'function') {
        this.aiVehicle.setSpec(scenario.aiConfig.spec);
      }
      setVehicleState(this.aiVehicle, this.track, {
        distance: scenario.aiConfig.distance ?? 0,
        lateral: scenario.aiConfig.lateralOffset ?? 0,
        speedKph: scenario.aiConfig.initialSpeedKph ?? 0,
        ersMode: scenario.aiConfig.ersMode ?? null
      });
      this.aiVehicle.cooldownActive = false;
      this.aiVehicle.finished = false;
      this.aiVehicle.retired = false;

      this.ersEnergyStart = this.aiVehicle.ers?.energyJ ?? 0;
    }

    // 3. Configure AI Controller state and tactics
    if (this.aiController && this.aiVehicle) {
      this.aiController.resetForRace(this.aiVehicle);

      if (Number.isFinite(scenario.aiConfig?.initialAggression)) {
        this.aiController.aggression = scenario.aiConfig.initialAggression;
      }

      const maneuver = scenario.aiConfig?.initialManeuver;
      if (maneuver && this.player) {
        this._configureInitialAIManeuver(maneuver);
      }
    }

    // 4. Record initial relative gap
    if (this.player && this.aiVehicle && scenario.playerConfig) {
      const initialGap = signedTrackGap(this.player.distance, this.aiVehicle.distance, this.track.length);
      this.currentGap = initialGap;
      this.previousGap = initialGap;
      this.initialGapSign = Math.sign(initialGap);
      this.gapSamples.push(initialGap);
      this.avgGap = initialGap;
    }

    this._logEvent('SCENARIO_LOADED', { scenarioId: scenario.id, name: scenario.name });
    return this.activeScenario;
  }

  /**
   * Sets initial racecraft maneuver target and intent on the AI controller.
   */
  _configureInitialAIManeuver(maneuver) {
    const ai = this.aiController;
    const player = this.player;
    if (!ai || !player) return;

    switch (maneuver) {
      case 'SLIPSTREAM_DRAFT':
        ai.draftTargetId = player.id;
        ai.passTargetId = player.id;
        ai.passPhase = 'NONE';
        if (ai.racecraft) {
          ai.racecraft.phase = 'DRAFT';
          ai.racecraft.targetId = player.id;
        }
        break;

      case 'ATTACK_DIVE':
        ai.passPhase = 'ATTACK_LEFT';
        ai.passTargetId = player.id;
        if (ai.racecraft) {
          ai.racecraft.phase = 'ATTACK_LEFT';
          ai.racecraft.targetId = player.id;
          ai.racecraft.side = -1;
          ai.racecraft.targetOffset = -2.2;
        }
        break;

      case 'CUTBACK':
        ai.passPhase = 'ATTACK_RIGHT';
        ai.passTargetId = player.id;
        if (ai.racecraft) {
          ai.racecraft.phase = 'ATTACK_RIGHT';
          ai.racecraft.targetId = player.id;
          ai.racecraft.side = 1;
          ai.racecraft.targetOffset = 2.0;
        }
        break;

      case 'OUTSIDE_SWEEP':
        ai.passPhase = 'ATTACK_RIGHT';
        ai.passTargetId = player.id;
        if (ai.racecraft) {
          ai.racecraft.phase = 'ATTACK_RIGHT';
          ai.racecraft.targetId = player.id;
          ai.racecraft.side = 1;
          ai.racecraft.targetOffset = 3.2;
        }
        break;

      case 'OBSTACLE_AVOIDANCE':
        ai.passPhase = 'ATTACK_LEFT';
        ai.passTargetId = player.id;
        if (ai.racecraft) {
          ai.racecraft.phase = 'ATTACK_LEFT';
          ai.racecraft.targetId = player.id;
          ai.racecraft.side = -1;
          ai.racecraft.targetOffset = -2.8;
        }
        break;

      case 'DEFEND_INSIDE':
        ai.passPhase = 'DEFENSE';
        ai.defenseTargetId = player.id;
        if (ai.racecraft) {
          ai.racecraft.phase = 'DEFENSE';
          ai.racecraft.defenseTargetId = player.id;
          ai.racecraft.targetOffset = -2.5;
        }
        break;

      case 'DEFEND_CUTBACK':
      case 'DEFEND_EXIT_DRIVE':
        ai.passPhase = 'DEFENSE';
        ai.defenseTargetId = player.id;
        if (ai.racecraft) {
          ai.racecraft.phase = 'DEFENSE';
          ai.racecraft.defenseTargetId = player.id;
        }
        break;

      case 'PACE_HOTLAP':
        ai.passPhase = 'PACE';
        if (ai.racecraft) ai.racecraft.phase = 'PACE';
        break;

      case 'RACE':
      default:
        ai.passPhase = 'NONE';
        break;
    }
  }

  /**
   * Resets the active scenario to its starting state immediately.
   */
  reset() {
    if (this.activeScenario) {
      return this.loadScenario(this.activeScenario);
    }
    return null;
  }

  resetScenario() {
    return this.reset();
  }

  update(dt, collisionStats = null) {
    return this.step(dt, collisionStats);
  }

  /**
   * Step the scenario simulation: evaluates gaps, clearances, contacts,
   * pass execution, and termination criteria.
   */
  step(dt, collisionStats = null) {
    if (this.status !== 'RUNNING' || !this.activeScenario) {
      return this.getMetrics();
    }

    this.timeElapsed += dt;
    const scenario = this.activeScenario;
    const hasBothVehicles = Boolean(this.player && this.aiVehicle && scenario.playerConfig);

    if (hasBothVehicles) {
      // 1. Distance gap & closing speed
      const gap = signedTrackGap(this.player.distance, this.aiVehicle.distance, this.track.length);
      const gapChangeRate = (this.previousGap - gap) / Math.max(0.0001, dt);
      const speedDiff = this.aiVehicle.speed - this.player.speed;
      const closingSpeed = Math.max(0, gap < 0 ? speedDiff : -speedDiff, Math.abs(gapChangeRate));

      this.currentGap = gap;
      this.previousGap = gap;
      this.maxClosingSpeed = Math.max(this.maxClosingSpeed, closingSpeed);

      this.gapSamples.push(gap);
      if (this.gapSamples.length > 200) this.gapSamples.shift();
      this.avgGap = this.gapSamples.reduce((sum, g) => sum + g, 0) / this.gapSamples.length;

      // 2. 3D clearance calculation
      const dx = this.aiVehicle.position.x - this.player.position.x;
      const dy = this.aiVehicle.position.y - this.player.position.y;
      const dz = this.aiVehicle.position.z - this.player.position.z;
      const centerDist = Math.hypot(dx, dy, dz);
      const boundSum = ((this.aiVehicle.wheelBase || 2.7) + (this.player.wheelBase || 2.7)) * 0.65;
      const clearance = Math.max(0, centerDist - boundSum);
      this.minClearance = Math.min(this.minClearance, clearance);

      // 3. Slipstream / Drafting detection
      const latDiff = Math.abs((this.aiVehicle.surface?.lateral ?? 0) - (this.player.surface?.lateral ?? 0));
      const inDraftCone = gap < -2 && gap > -40 && latDiff < 2.0 && this.aiVehicle.speed > 25;
      if (inDraftCone) {
        this.draftTime += dt;
      }

      // 4. Contact / Collision detection
      let contactOccurred = false;
      if (collisionStats?.contactPairs) {
        contactOccurred = collisionStats.contactPairs.some(
          (p) => (p.a === this.player.id && p.b === this.aiVehicle.id) ||
                 (p.b === this.player.id && p.a === this.aiVehicle.id)
        );
      }
      if (!contactOccurred && centerDist < 1.7) {
        contactOccurred = true;
      }
      if (!contactOccurred && (this.aiVehicle.impact > 0.1 || this.player.impact > 0.1)) {
        contactOccurred = true;
      }

      if (contactOccurred) {
        this.contactCount += 1;
        this.safetyScore = Math.max(0, this.safetyScore - 20);
        this._logEvent('VEHICLE_CONTACT', { clearance, time: this.timeElapsed });
      }

      // 5. Off-track detection
      if (!isLegalSurface(this.aiVehicle)) {
        this.offTrackTime += dt;
        this.qualityScore = Math.max(0, this.qualityScore - dt * 15);
      }

      // 6. Pass quality tracking update
      if (this.passQualityTracker) {
        this.passQualityTracker.update([this.player, this.aiVehicle], collisionStats, dt, true);
      }
    } else if (this.aiVehicle) {
      // Solo vehicle scenario (e.g. Hotlap)
      this.lapDistanceTravelled += this.aiVehicle.speed * dt;
      if (!isLegalSurface(this.aiVehicle)) {
        this.offTrackTime += dt;
        this.qualityScore = Math.max(0, this.qualityScore - dt * 20);
      }
    }

    // 7. Track ERS energy consumption
    if (this.aiVehicle?.ers?.energyJ !== undefined && this.ersEnergyStart > 0) {
      this.ersUsed = Math.max(0, this.ersEnergyStart - this.aiVehicle.ers.energyJ);
    }

    // 8. Evaluate Scenario Criteria
    this._evaluateScenarioRules(dt);

    return this.getMetrics();
  }

  /**
   * Internal scenario rule checking for attack, defense, hotlap, and duel presets.
   */
  _evaluateScenarioRules(dt) {
    const scenario = this.activeScenario;
    const criteria = scenario.successCriteria;
    const category = scenario.category;

    // Contact disqualification check
    const maxContactsAllowed = criteria?.maxContactCount ?? 0;
    if (this.contactCount > maxContactsAllowed) {
      this.completed = true;
      this.passSuccess = false;
      this.status = 'FAILED';
      this.finishReason = `Disqualified: Exceeded allowable contact limit (${this.contactCount} > ${maxContactsAllowed}).`;
      this._logEvent('DISQUALIFIED_CONTACT', { contactCount: this.contactCount });
      return;
    }

    // Category-specific evaluation
    switch (category) {
      case 'attack': {
        // AI is attacker, starts behind player (initialGapSign <= 0).
        // AI executes pass when currentGap > 4.5m ahead.
        const passClearanceM = criteria?.minClearanceM ?? 1.0;
        const requiredHoldTime = criteria?.holdLeadTimeS ?? 1.0;

        if (this.currentGap >= 4.5) {
          this.leadHoldTimer += dt;
          if (this.leadHoldTimer >= requiredHoldTime) {
            const clean = (this.contactCount <= maxContactsAllowed) &&
                          (!criteria?.allowOffTrack ? this.offTrackTime < 0.25 : true) &&
                          (this.minClearance >= passClearanceM * 0.7);

            this.completed = true;
            this.passSuccess = clean;
            this.status = clean ? 'COMPLETED' : 'FAILED';
            this.finishReason = clean
              ? `Clean pass completed! Held lead for ${this.leadHoldTimer.toFixed(2)}s.`
              : `Pass tainted by off-track excursions or tight clearance.`;
            this._logEvent(clean ? 'PASS_SUCCESS' : 'PASS_REJECTED', {
              leadHoldTimer: this.leadHoldTimer,
              minClearance: this.minClearance
            });
            return;
          }
        } else {
          this.leadHoldTimer = 0;
        }

        // Timeout check
        if (this.timeElapsed >= scenario.durationS) {
          this.completed = true;
          this.passSuccess = false;
          this.status = 'TIMEOUT';
          this.finishReason = `Time limit expired (${scenario.durationS}s) before completing pass.`;
          this._logEvent('SCENARIO_TIMEOUT', { timeElapsed: this.timeElapsed });
        }
        break;
      }

      case 'defense': {
        // AI is defender, starts ahead (initialGapSign >= 0).
        // If player overtakes AI and establishes gap < -5.5m, AI fails defense.
        if (this.currentGap < -5.5) {
          this.completed = true;
          this.passSuccess = false;
          this.status = 'FAILED';
          this.finishReason = 'AI lost defensive position; player completed pass.';
          this._logEvent('DEFENSE_BREACHED', { currentGap: this.currentGap });
          return;
        }

        // If time elapsed and AI held the lead
        if (this.timeElapsed >= scenario.durationS) {
          const heldLead = this.currentGap >= 0;
          this.completed = true;
          this.passSuccess = heldLead && (this.contactCount <= maxContactsAllowed);
          this.status = this.passSuccess ? 'COMPLETED' : 'FAILED';
          this.finishReason = this.passSuccess
            ? 'AI successfully defended inside line and held lead for full duration.'
            : 'AI failed to defend lead position.';
          this._logEvent(this.passSuccess ? 'DEFENSE_SUCCESS' : 'DEFENSE_FAILED', { currentGap: this.currentGap });
        }
        break;
      }

      case 'hotlap': {
        const targetLapTime = criteria?.maxLapTimeS ?? 65.0;
        const lapLength = this.track.length || 3200;

        // Check if lap completed
        if (this.lapDistanceTravelled >= lapLength) {
          const valid = (this.timeElapsed <= targetLapTime) &&
                        (!criteria?.allowOffTrack ? this.offTrackTime < 0.2 : true);
          this.completed = true;
          this.passSuccess = valid;
          this.status = valid ? 'COMPLETED' : 'FAILED';
          this.finishReason = valid
            ? `Hotlap benchmark succeeded: ${this.timeElapsed.toFixed(3)}s (Target: <${targetLapTime.toFixed(1)}s).`
            : `Hotlap pace insufficient: ${this.timeElapsed.toFixed(3)}s or off-track infractions.`;
          this._logEvent('HOTLAP_FINISHED', { lapTime: this.timeElapsed, valid });
          return;
        }

        if (this.timeElapsed >= scenario.durationS) {
          this.completed = true;
          this.passSuccess = false;
          this.status = 'TIMEOUT';
          this.finishReason = `Hotlap timed out before completing track distance.`;
          this._logEvent('SCENARIO_TIMEOUT', { timeElapsed: this.timeElapsed });
        }
        break;
      }

      case 'duel': {
        if (this.timeElapsed >= scenario.durationS) {
          this.completed = true;
          this.passSuccess = true;
          this.status = 'COMPLETED';
          this.finishReason = `Duel finished. Final gap: ${this.currentGap.toFixed(1)}m.`;
          this._logEvent('DUEL_COMPLETED', { finalGap: this.currentGap });
        }
        break;
      }

      default:
        if (this.timeElapsed >= scenario.durationS) {
          this.completed = true;
          this.status = 'COMPLETED';
          this.finishReason = 'Scenario completed.';
        }
        break;
    }
  }

  _logEvent(type, data = {}) {
    this.eventLog.push({
      time: Number(this.timeElapsed.toFixed(3)),
      type,
      ...data
    });
  }

  /**
   * Returns a live snapshot of scenario metrics, scoring and status.
   */
  getMetrics() {
    return Object.freeze({
      scenario: this.activeScenario?.id ?? null,
      scenarioName: this.activeScenario?.name ?? '',
      category: this.activeScenario?.category ?? '',
      description: this.activeScenario?.description ?? '',
      timeElapsed: Number(this.timeElapsed.toFixed(3)),
      durationS: this.activeScenario?.durationS ?? 0,
      completed: this.completed,
      passSuccess: this.passSuccess,
      status: this.status,
      finishReason: this.finishReason,
      minClearance: Number.isFinite(this.minClearance) ? Number(this.minClearance.toFixed(2)) : 99.0,
      contactCount: this.contactCount,
      maxClosingSpeed: Number(this.maxClosingSpeed.toFixed(2)),
      maxClosingSpeedKph: Number((this.maxClosingSpeed * 3.6).toFixed(1)),
      avgGap: Number(this.avgGap.toFixed(2)),
      currentGap: Number(this.currentGap.toFixed(2)),
      draftTime: Number(this.draftTime.toFixed(2)),
      ersUsed: Number(this.ersUsed.toFixed(0)),
      offTrackTime: Number(this.offTrackTime.toFixed(2)),
      qualityScore: Math.round(this.qualityScore),
      safetyScore: Math.round(this.safetyScore),
      playerSpeedKph: Number(((this.player?.speed ?? 0) * 3.6).toFixed(1)),
      aiSpeedKph: Number(((this.aiVehicle?.speed ?? 0) * 3.6).toFixed(1)),
      events: [...this.eventLog]
    });
  }

  isCompleted() {
    return this.completed;
  }

  isSuccess() {
    return this.passSuccess;
  }
}

export {
  SCENARIOS,
  SCENARIO_LIST,
  SCENARIOS_BY_ID,
  getScenarioById,
  getScenariosByCategory
};
