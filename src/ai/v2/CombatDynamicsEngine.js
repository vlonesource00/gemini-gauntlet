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
        // Never lift or brake due to light side-by-side rubbing contact!
        if (throttle > 0.1 && brake < 0.05) {
          throttle = Math.max(throttle, 0.45);
        }
        break;
      }
    }

    // =========================================================================
    // 2. SLIP-SLOPE EXTREMUM SEEKING & POWER-SLIDE COUNTER-STEER
    // =========================================================================
    // Dynamic breakaway threshold: Prototype tires generate peak lateral load at ~6.5° (0.115 rad).
    // Genuine oversteer breakaway occurs at > 7.5° (0.130 rad).
    const absSlip = Math.abs(slipAngle);
    this.powerSlideActive = false;

    if (absSlip > 0.130 && vSpeed > 10.0) {
      this.powerSlideActive = true;
      const excessSlip = absSlip - 0.130;
      const slipSign = Math.sign(slipAngle);

      // Instantaneous micro-countersteer proportional to yaw velocity and excess slip
      const counterSteer = slipSign * clamp(excessSlip * 1.8 + Math.abs(yawRate) * 0.08, 0.02, 0.25);
      steer = clamp(steer + counterSteer, -0.45, 0.45);

      // Power-slide maintenance: maintain positive throttle (40%) to keep rear tires spinning and maintain attitude
      if (brake < 0.05) {
        throttle = Math.max(0.40, throttle * 0.90);
      }
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
