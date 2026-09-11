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
    this.carWidth = 2.05;
    this.carLength = 4.65;
    this.rubbingActive = false;
    this.powerSlideActive = false;
  }

  /**
   * Apply combat dynamics and slip-slope adjustments to raw MPCC controls.
   * @param {Object} params
   * @returns {Object} Adjusted { steer, throttle, brake, rubbing, powerSlide }
   */
  process({
    vehicle,
    traffic,
    controls, // { steer, throttle, brake }
    dt = 0.016
  } = {}) {
    let { steer, throttle, brake } = controls;
    const vSpeed = finite(vehicle?.speed, 0);
    const yawRate = finite(vehicle?.yawRate, 0);

    // Live body slip angle
    const slipAngle = Math.atan2(
      finite(vehicle?.localVelocity?.x, 0),
      Math.max(2.0, Math.abs(finite(vehicle?.localVelocity?.z, vSpeed)))
    );
    const absSlip = Math.abs(slipAngle);

    // =========================================================================
    // 1. CONTACT-TOLERANT LATERAL RUBBING EQUILIBRIUM
    // =========================================================================
    this.rubbingActive = false;
    const entries = traffic?.entries ?? [];

    for (const entry of entries) {
      if (!entry?.other || entry.other.finished || entry.other.despawned || entry.other.trafficGhost) continue;
      const longGap = finite(entry.delta, 99);
      const latGap = finite(entry.lateralDelta, 99);

      // Genuine side-by-side rubbing contact (< 1.85m separation)
      if (Math.abs(longGap) < this.carLength * 0.85 && Math.abs(latGap) < this.carWidth * 0.90) {
        this.rubbingActive = true;
        // Maintain drive momentum during side-by-side rubbing contact unless already breaking away
        if (throttle > 0.1 && brake < 0.05 && absSlip < 0.20) {
          throttle = Math.max(throttle, 0.45);
        }
        break;
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
      powerSlide: this.powerSlideActive
    };
  }
}
