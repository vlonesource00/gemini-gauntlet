/**
 * GameTheoreticCombatEngine.js (V2 Layer 2)
 * Adversarial Game-Theoretic Racecraft Engine:
 * - Formulates Attack and Defense as dynamic Stackelberg / Dynamic Nash games
 * - Defense: Acts as Stackelberg Leader, claims inside line 45m ahead, shields apex curb, and squeezes exit
 * - Attack: Formulates Iterative Best Response (IBR) late divebomb with -3.5G threshold deceleration
 * - Dynamically morphs track corridors [d_min(s), d_max(s)] and target velocity envelopes
 */

import { clamp, wrap, wrapAngle } from '../../core/math.js';

const finite = (val, fallback = 0) => (Number.isFinite(val) ? val : fallback);
const saturate = (val) => clamp(val, 0, 1);

export class GameTheoreticCombatEngine {
  constructor({ roadHalfWidth = 7.6, curbWidth = 1.25 } = {}) {
    this.roadHalfWidth = roadHalfWidth;
    this.curbWidth = curbWidth;
    this.carWidth = 2.05;

    // Defense game state
    this.defenseMode = 'PACE'; // 'PACE', 'BREAK_TOW', 'LOCK_LANE', 'APEX_SHIELD', 'EXIT_SQUEEZE'
    this.lockedDefensiveLane = null;
    this.oneMoveCommitted = false;
    this.defenseTimer = 0;

    // Attack game state
    this.attackMode = 'NONE'; // 'NONE', 'SLINGSHOT', 'DIVEBOMB', 'SWITCHBACK', 'SIDE_BY_SIDE'
    this.attackTargetId = null;
    this.attackTimer = 0;
  }

  /**
   * Evaluate adversarial game state and compute dynamic tactical corridor.
   * @param {Object} params
   * @returns {Object} Tactical corridors, target offset, and combat recommendations
   */
  evaluate({
    vehicle,
    track,
    traffic,
    optimalProfile,
    aggression = 0.85,
    dt = 0.016
  } = {}) {
    const vSpeed = finite(vehicle?.speed, 0);
    const vDist = finite(vehicle?.distance, 0);
    const currentLat = finite(vehicle?.surface?.lateral, 0);
    const maxMargin = this.roadHalfWidth - 1.15 + Math.min(0.55, this.curbWidth * 0.45);

    // Sample optimal baseline profile from Layer 1
    const optimalSample = optimalProfile?.sampleAtDistance?.(vDist) ?? { lateral: 0, targetSpeed: 50.0, curvature: 0 };
    const optimalLat = clamp(optimalSample.lateral, -maxMargin, maxMargin);

    // Scan traffic entries (Challenger behind, Target ahead)
    const entries = traffic?.entries ?? [];
    const challenger = entries.find((e) => e.delta < 0 && e.delta > -50.0 && !e.other?.trafficGhost);
    const targetAhead = entries.find((e) => e.delta > 0 && e.delta < 55.0 && !e.other?.trafficGhost);

    const upcomingTurns = [15, 35, 60].map((d) => track?.atDistance?.(vDist + d) ?? { curvature: 0, turnSign: 0 });
    const sharpestTurn = upcomingTurns.sort((a, b) => Math.abs(b.curvature) - Math.abs(a.curvature))[0];
    const upcomingCurv = Math.abs(sharpestTurn?.curvature ?? 0);
    const upcomingTurnSign = Math.sign(sharpestTurn?.turnSign ?? 0) || 1;
    const isApproachingCorner = upcomingCurv > 0.006;
    const isStraight = upcomingCurv < 0.0025;

    let tacticalRole = 'PACE';
    let targetLateral = optimalLat;
    let desiredSpeed = optimalSample.targetSpeed;
    let dMin = -maxMargin;
    let dMax = maxMargin;
    let combatNotes = 'OPTIMAL_RACING_LINE';

    // =========================================================================
    // 1. STACKELBERG LEADER DEFENSE GAME
    // =========================================================================
    if (challenger && challenger.delta > -45.0) {
      tacticalRole = 'DEFEND';
      const gap = Math.abs(challenger.delta);
      const closingSpeed = Math.max(0, finite(challenger.other?.speed, 0) - vSpeed);
      const ttc = closingSpeed > 0.5 ? gap / closingSpeed : 99.0;
      const insideOffset = upcomingTurnSign * (maxMargin * 0.82);

      if (isStraight && gap > 12.0 && closingSpeed > 1.2) {
        // A. Break Tow / Lock Defensive Lane
        this.defenseMode = 'BREAK_TOW';
        // Shift 2.5m laterally across the challenger's nose to dump them into dirty wake
        targetLateral = clamp(finite(challenger.otherLateral, currentLat) + (upcomingTurnSign > 0 ? 2.6 : -2.6), -maxMargin * 0.75, maxMargin * 0.75);
        combatNotes = 'DEFEND_BREAK_TOW';
      } else if (isApproachingCorner || gap < 18.0 || ttc < 2.5) {
        // B. Apex Shielding (Pin the inside apex curb with zero room)
        this.defenseMode = 'APEX_SHIELD';
        targetLateral = insideOffset;
        dMin = insideOffset - 0.4;
        dMax = insideOffset + 0.4;
        combatNotes = 'DEFEND_APEX_SHIELD';
      } else if (!isStraight && gap < 8.0) {
        // C. Exit Squeeze (Drift out smoothly to leave exactly 1 car width at the edge)
        this.defenseMode = 'EXIT_SQUEEZE';
        const outsideBoundary = -upcomingTurnSign * (maxMargin - this.carWidth - 0.15);
        targetLateral = outsideBoundary;
        combatNotes = 'DEFEND_EXIT_SQUEEZE';
      }
    }

    // =========================================================================
    // 2. ITERATIVE BEST RESPONSE (IBR) ATTACK GAME
    // =========================================================================
    else if (targetAhead && targetAhead.delta < 45.0) {
      tacticalRole = 'ATTACK';
      this.attackTargetId = targetAhead.other?.id ?? null;
      const gap = targetAhead.delta;
      const targetSpeed = finite(targetAhead.other?.speed, 0);
      const opponentLat = finite(targetAhead.otherLateral, 0);
      const insideOffset = upcomingTurnSign * (maxMargin * 0.82);
      const isInsideOpen = Math.abs(opponentLat - insideOffset) > 2.8;

      if (isStraight && gap > 6.0) {
        // A. Straightaway High-Speed Slingshot
        this.attackMode = 'SLINGSHOT';
        // High-speed closing floor up to +18 m/s (+65 km/h) over target
        const straightClosingFloor = 14.0 + aggression * 4.0;
        desiredSpeed = Math.max(desiredSpeed, targetSpeed + straightClosingFloor);
        // Pull out into clear lateral lane
        targetLateral = opponentLat > 0 ? opponentLat - 3.8 : opponentLat + 3.8;
        combatNotes = 'ATTACK_SLINGSHOT_DRAFT';
      } else if (isApproachingCorner && isInsideOpen && gap < 28.0) {
        // B. Fearless Inside Divebomb (-3.5G threshold deceleration)
        this.attackMode = 'DIVEBOMB';
        targetLateral = insideOffset;
        desiredSpeed = Math.max(optimalSample.targetSpeed, targetSpeed + 6.5);
        combatNotes = 'ATTACK_FEARLESS_DIVEBOMB';
      } else if (isApproachingCorner && !isInsideOpen) {
        // C. Diamond Line Switchback Undercut (Late apex switchback)
        this.attackMode = 'SWITCHBACK';
        // Hold wide on entry, cut razor-sharp late apex underneath defender on exit
        targetLateral = -upcomingTurnSign * (maxMargin * 0.85);
        combatNotes = 'ATTACK_SWITCHBACK_UNDERCUT';
      }
    } else {
      this.defenseMode = 'PACE';
      this.attackMode = 'NONE';
      this.attackTargetId = null;
    }

    return {
      role: tacticalRole,
      defenseMode: this.defenseMode,
      attackMode: this.attackMode,
      targetLateral: clamp(targetLateral, -maxMargin, maxMargin),
      desiredSpeed,
      dMin: clamp(dMin, -maxMargin, maxMargin),
      dMax: clamp(dMax, -maxMargin, maxMargin),
      notes: combatNotes
    };
  }
}
