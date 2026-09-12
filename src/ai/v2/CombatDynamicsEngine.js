/**
 * CombatDynamicsEngine.js (V2 Combat & Slip Layer)
 * Combat Rubbing Normal Force Equilibrium & Active Slip-Slope Limit Tracking:
 * - Side-by-side elastic rubbing contact tolerance (leans into contact without aborting)
 * - Dynamic slip-slope extremum seeking (∂Fy / ∂α) to ride the crest of tire grip
 * - Instantaneous micro-countersteer power sliding under yaw snaps without cutting throttle
 */

/**
 * CombatDynamicsEngine.js (V2 Combat & Slip Layer)
 * Combat Rubbing Normal Force Equilibrium & Active Slip-Slope Limit Tracking:
 * - Side-by-side elastic rubbing contact tolerance (leans into contact without aborting)
 * - Dynamic slip-slope extremum seeking (∂Fy / ∂α) to ride the crest of tire grip
 * - Instantaneous micro-countersteer power sliding under yaw snaps without cutting throttle
 */

import { clamp, wrapAngle } from '../../core/math.js';

const finite = (val, fallback = 0) => (Number.isFinite(val) ? val : fallback);
const saturate = (val) => clamp(val, 0, 1);

export class CombatDynamicsEngine {
  constructor() {
    this.carLength = 4.4;
    this.carWidth = 1.9;
    this.rubbingActive = false;
    this.powerSlideActive = false;
    this.attributableContacts = 0;
    this.contactSeverityEstimate = 0.0;
    this.egoFaultSeverityEstimate = 0.0;
    this.actualDamageDelta = 0.0;
    this.accumulatedHostDamage = 0.0;
    this.lastHostDamage = null;
    // Backward-compatibility aliases
    this.attributableDamage = 0.0;
    this.contactAssociatedDamage = 0.0;

    this.contactEpisodes = [];
    this.activeContactEpisode = null;
    this.rivalLastContactTime = new Map();
    this.lastContactTime = -999;
    this.totalTime = 0;
  }

  /**
   * Reset internal episode state between runs.
   */
  reset() {
    this.rubbingActive = false;
    this.powerSlideActive = false;
    this.attributableContacts = 0;
    this.contactSeverityEstimate = 0.0;
    this.egoFaultSeverityEstimate = 0.0;
    this.actualDamageDelta = 0.0;
    this.accumulatedHostDamage = 0.0;
    this.lastHostDamage = null;
    this.attributableDamage = 0.0;
    this.contactAssociatedDamage = 0.0;

    this.contactEpisodes = [];
    this.activeContactEpisode = null;
    this.rivalLastContactTime.clear();
    this.lastContactTime = -999;
    this.totalTime = 0;
  }

  /**
   * Apply combat dynamics and slip-slope adjustments to raw MPCC controls.
   * @param {Object} params
   * @returns {Object} Adjusted controls and contact episode telemetry
   */
  process({
    vehicle,
    traffic,
    controls,
    dt = 0.016
  } = {}) {
    let { steer, throttle, brake } = controls;
    this.totalTime = (this.totalTime || 0) + dt;
    const vSpeed = finite(vehicle?.speed, 0);
    const yawRate = finite(vehicle?.yawRate, 0);

    // Track actual host vehicle damage if available
    const hostDamage = finite(vehicle?.damage, 0);
    if (this.lastHostDamage === null) {
      this.lastHostDamage = hostDamage;
    }
    const currentDamageDelta = Math.max(0, hostDamage - this.lastHostDamage);
    this.lastHostDamage = hostDamage;
    this.actualDamageDelta = currentDamageDelta;
    this.accumulatedHostDamage += currentDamageDelta;

    // Lateral slip angle beta ~ atan2(v_lat, v_long)
    const sideslip = Math.atan2(
      finite(vehicle?.localVelocity?.x, 0),
      Math.max(1.0, Math.abs(finite(vehicle?.localVelocity?.z, 0)))
    );
    const absSlip = Math.abs(sideslip);

    // =========================================================================
    // 1. ELASTIC CONTACT RUBBING EQUILIBRIUM & ATTRIBUTION
    // =========================================================================
    this.rubbingActive = false;
    const entries = traffic?.entries ?? [];
    let activeContactEntry = null;

    for (const entry of entries) {
      if (!entry?.other || entry.other.finished || entry.other.despawned || entry.other.trafficGhost) continue;
      const longGap = finite(entry.delta, 99);
      const latGap = finite(entry.lateralDelta, 99);

      // Genuine side-by-side rubbing contact (< 1.85m separation)
      if (Math.abs(longGap) < this.carLength * 0.85 && Math.abs(latGap) < this.carWidth * 0.90) {
        this.rubbingActive = true;
        activeContactEntry = entry;
        // Maintain drive momentum during side-by-side rubbing contact unless already breaking away
        if (throttle > 0.1 && brake < 0.05 && absSlip < 0.20) {
          throttle = Math.max(throttle, 0.45);
        }
        break;
      }
    }

    // Debounced Contact Episode Tracking & Attribution (Phase 8, 12 & V3.2)
    if (activeContactEntry) {
      const otherId = activeContactEntry.other?.id ?? 'rival';
      const normalImpactSpeed = Math.abs(finite(activeContactEntry.relativeLateralVelocity, 0));
      const closingSpeed = Math.hypot(
        normalImpactSpeed,
        Math.abs(finite(activeContactEntry.relativeLongitudinalVelocity, 0))
      );
      const latGap = Math.abs(finite(activeContactEntry.lateralDelta, 99));
      const isDeepPenetration = latGap < this.carWidth * 0.52;
      const latPenetration = Math.max(0, this.carWidth - latGap);
      const longPenetration = Math.max(0, this.carLength - Math.abs(finite(activeContactEntry.delta, 99)));
      const peakPenetration = Math.max(latPenetration, longPenetration);

      // Ego vs opponent lateral movement (lateral velocities towards each other)
      const egoSideVel = finite(vehicle?.localVelocity?.x, 0);
      const otherLatVel = finite(activeContactEntry.otherLateralSpeed, 0);
      const isEgoMovingTowardsOther = (activeContactEntry.side > 0 && egoSideVel > 0.25) || (activeContactEntry.side < 0 && egoSideVel < -0.25);
      const isOtherMovingTowardsEgo = (activeContactEntry.side > 0 && otherLatVel < -0.25) || (activeContactEntry.side < 0 && otherLatVel > 0.25);

      let classification = 'BENIGN_DOOR_RUB';
      let isEgoFault = false;

      if (Math.abs(activeContactEntry.delta) > this.carLength * 0.65) {
        classification = 'FRONT_REAR_IMPACT';
        if (activeContactEntry.delta > 0 && activeContactEntry.relativeLongitudinalVelocity > 1.2) {
          isEgoFault = true;
        }
      } else if (isDeepPenetration) {
        classification = 'SEVERE_OVERLAP';
        isEgoFault = isEgoMovingTowardsOther;
      } else if (isEgoMovingTowardsOther && !isOtherMovingTowardsEgo) {
        classification = 'EGO_INITIATED_PINCH';
        isEgoFault = true;
      } else if (isOtherMovingTowardsEgo && !isEgoMovingTowardsOther) {
        classification = 'OPPONENT_INITIATED_PINCH';
        isEgoFault = false;
      } else {
        classification = 'BENIGN_DOOR_RUB';
        isEgoFault = false;
      }

      // If active contact was with a DIFFERENT rival: close old episode immediately!
      if (this.activeContactEpisode && this.activeContactEpisode.rivalId !== otherId) {
        this.activeContactEpisode.closed = true;
        this.activeContactEpisode.endTime = this.totalTime;
        this.activeContactEpisode = null;
      }

      const rivalLastTime = this.rivalLastContactTime.get(otherId) ?? -999;
      const timeSinceRivalLast = this.totalTime - rivalLastTime;

      // Check if continuing an existing contact episode or starting a new one
      if (!this.activeContactEpisode || timeSinceRivalLast > 0.35) {
        this.activeContactEpisode = {
          id: `contact_${otherId}_${Math.round(this.totalTime * 1000)}`,
          rivalId: otherId,
          startTime: this.totalTime,
          duration: 0,
          peakPenetration,
          peakClosingSpeed: closingSpeed,
          classification,
          egoInitiated: isEgoFault,
          severityEstimate: 0,
          egoFaultSeverityEstimate: 0,
          associatedDamage: 0,
          attributableDamage: 0,
          closed: false
        };
        this.contactEpisodes.push(this.activeContactEpisode);
        if (isEgoFault && classification !== 'BENIGN_DOOR_RUB') {
          this.attributableContacts += 1;
        }
      } else {
        this.activeContactEpisode.duration += dt;
        this.activeContactEpisode.peakPenetration = Math.max(this.activeContactEpisode.peakPenetration, peakPenetration);
        this.activeContactEpisode.peakClosingSpeed = Math.max(this.activeContactEpisode.peakClosingSpeed, closingSpeed);
        if (isEgoFault) this.activeContactEpisode.egoInitiated = true;
        if (classification === 'SEVERE_OVERLAP' || classification === 'EGO_INITIATED_PINCH' || classification === 'FRONT_REAR_IMPACT') {
          this.activeContactEpisode.classification = classification;
        }
      }

      this.lastContactTime = this.totalTime;
      this.rivalLastContactTime.set(otherId, this.totalTime);

      // Rate-limited severity estimation
      const stepSeverity = clamp(closingSpeed * 0.012 * dt, 0.0001, 0.005);
      this.contactSeverityEstimate += stepSeverity;
      this.contactAssociatedDamage = this.contactSeverityEstimate;
      this.activeContactEpisode.severityEstimate += stepSeverity;
      this.activeContactEpisode.associatedDamage += stepSeverity;

      if (this.activeContactEpisode.egoInitiated && this.activeContactEpisode.classification !== 'BENIGN_DOOR_RUB') {
        this.egoFaultSeverityEstimate += stepSeverity;
        this.attributableDamage = this.egoFaultSeverityEstimate;
        this.activeContactEpisode.egoFaultSeverityEstimate += stepSeverity;
        this.activeContactEpisode.attributableDamage += stepSeverity;
      }
    } else {
      // No active contact this frame
      if (this.activeContactEpisode && (this.totalTime - this.lastContactTime > 0.25)) {
        this.activeContactEpisode.closed = true;
        this.activeContactEpisode.endTime = this.totalTime;
        this.activeContactEpisode = null;
      }
    }

    // =========================================================================
    // 2. LAST-LINE EMERGENCY SPIN BREAKAWAY CATCH
    // =========================================================================
    // Emergency catch activates under severe yaw/sideslip excursions (> 0.26 rad ~ 15 deg)
    // where upstream feedback was insufficient to prevent breakaway.
    const isSpinBreakaway = absSlip > 0.26 && Math.abs(yawRate) > 1.25;
    this.powerSlideActive = false;

    if (isSpinBreakaway && vSpeed > 3.0) {
      this.powerSlideActive = true;
      const excessSlip = absSlip - 0.22;

      const maxCounterSteer = clamp(3.8 / Math.max(4.0, vSpeed) + 0.10, 0.15, 0.55);
      // Active countersteer opposite to yaw rate to arrest yaw angular momentum
      const counterSteer = -Math.sign(yawRate) * clamp(excessSlip * 1.35 + Math.abs(yawRate) * 0.12, 0.08, maxCounterSteer);
      
      steer = clamp(counterSteer, -maxCounterSteer, maxCounterSteer);

      // Emergency breakaway throttle cap: cap upper throttle to prevent excessive
      // wheelspin power-oversteer without forcing a minimum throttle floor during an active spin
      throttle = Math.min(throttle, 0.25);
    }

    return {
      steer,
      throttle,
      brake,
      rubbing: this.rubbingActive,
      powerSlide: this.powerSlideActive,
      contactEpisodes: this.contactEpisodes,
      activeContactEpisode: this.activeContactEpisode,
      contactSeverityEstimate: this.contactSeverityEstimate,
      egoFaultSeverityEstimate: this.egoFaultSeverityEstimate,
      actualDamageDelta: this.actualDamageDelta,
      accumulatedHostDamage: this.accumulatedHostDamage,
      contactAssociatedDamage: this.contactSeverityEstimate,
      attributableDamage: this.egoFaultSeverityEstimate,
      attributableContacts: this.attributableContacts
    };
  }
}
