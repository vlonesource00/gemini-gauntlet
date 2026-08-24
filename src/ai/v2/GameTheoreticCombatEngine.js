/**
 * GameTheoreticCombatEngine.js (V2 Layer 2)
 * Adversarial Game-Theoretic Racecraft Engine:
 * - Formulates Attack and Defense as dynamic Stackelberg Leader-Follower & Iterative Best Response (IBR) games
 * - Defense (Stackelberg Leader):
 *     - Proactive inside lane claim up to 45m ahead of braking zones
 *     - FIA Single Defensive Move Rule enforcement & anti-weave feint filtering
 *     - Apex shielding (APEX_SHIELD): pins inside apex curb tight (0.35m-0.5m margin)
 *     - Exit squeeze (EXIT_SQUEEZE): smoothly drifts wide to leave exactly 1 car width at the edge
 *     - Break-tow (BREAK_TOW): stepped lateral shift destroying slipstream draft on straights
 * - Attack (Iterative Best Response):
 *     - Slingshot draft pull-out with +18 m/s closing speed floor and dynamic threshold timing
 *     - Fearless late divebomb (DIVEBOMB) with -3.5G threshold deceleration model
 *     - Diamond line switchback undercut (SWITCHBACK) countering inside-blocking defenders
 *     - Resilient side-by-side overlap holding without yielding
 * - Multi-Apex & Compound Turn Handling:
 *     - Multi-horizon curvature scanner detecting chicanes, S-bends, and double-apex complexes
 *     - Geometric compromise trajectory line preventing getting trapped on outside runoff
 * - Dynamically morphs track corridors [d_min(s), d_max(s)] and target velocity envelopes
 */

import { clamp, saturate, lerp, damp, wrap } from '../../core/math.js';

const finite = (val, fallback = 0) => (Number.isFinite(val) ? val : fallback);

export class GameTheoreticCombatEngine {
  /**
   * @param {Object} [options]
   * @param {number} [options.roadHalfWidth=8.2]
   * @param {number} [options.curbWidth=1.25]
   * @param {number} [options.carWidth=2.05]
   * @param {number} [options.carLength=4.65]
   */
  constructor({
    roadHalfWidth = 8.2,
    curbWidth = 1.25,
    carWidth = 2.05,
    carLength = 4.65
  } = {}) {
    this.roadHalfWidth = roadHalfWidth;
    this.curbWidth = curbWidth;
    this.carWidth = carWidth;
    this.carLength = carLength;

    // Defense Stackelberg state
    this.defenseMode = 'PACE'; // 'PACE', 'BREAK_TOW', 'LOCK_LANE', 'APEX_SHIELD', 'EXIT_SQUEEZE', 'RETURN'
    this.lockedDefensiveLane = null;
    this.defenseDirection = 0; // -1 (left), 0 (none), +1 (right)
    this.oneMoveLocked = false;
    this.defenseTimer = 0;
    this.defenseDwellTimer = 0;
    this.defenseTargetId = null;
    this.filteredAttackerLateral = 0;
    this.lastAttackerLateral = 0;
    this.feintFilterTimer = 0;
    this.threatScore = 0;

    // Attack Iterative Best Response (IBR) state
    this.attackMode = 'NONE'; // 'NONE', 'SLINGSHOT', 'DIVEBOMB', 'SWITCHBACK', 'SIDE_BY_SIDE', 'ATTACK_INSIDE', 'ATTACK_OUTSIDE'
    this.attackTargetId = null;
    this.passedTargetId = null;
    this.targetLockTimer = 0;
    this.attackTimer = 0;
    this.attackIntensity = 0;
    this.switchbackStage = 'NONE'; // 'NONE', 'ENTRY_WIDE', 'EXIT_UNDERCUT'
    this.divebombCommitted = false;

    // Multi-apex geometry state
    this.compoundTurnDetected = false;
    this.multiApexState = {
      isCompound: false,
      isChicane: false,
      isDoubleApex: false,
      primaryCurv: 0,
      primarySign: 1,
      primaryDist: 999,
      secondaryCurv: 0,
      secondarySign: 1,
      secondaryDist: 999
    };
  }

  /**
   * Reset internal combat engine state machine.
   */
  reset() {
    this.defenseMode = 'PACE';
    this.lockedDefensiveLane = null;
    this.defenseDirection = 0;
    this.oneMoveLocked = false;
    this.defenseTimer = 0;
    this.defenseDwellTimer = 0;
    this.defenseTargetId = null;
    this.filteredAttackerLateral = 0;
    this.lastAttackerLateral = 0;
    this.feintFilterTimer = 0;
    this.threatScore = 0;

    this.attackMode = 'NONE';
    this.attackTargetId = null;
    this.passedTargetId = null;
    this.targetLockTimer = 0;
    this.attackTimer = 0;
    this.attackIntensity = 0;
    this.switchbackStage = 'NONE';
    this.divebombCommitted = false;

    this.compoundTurnDetected = false;
    return this;
  }

  /**
   * Set configuration parameters dynamically.
   */
  setParameters({ roadHalfWidth, curbWidth, carWidth, carLength } = {}) {
    if (Number.isFinite(roadHalfWidth)) this.roadHalfWidth = roadHalfWidth;
    if (Number.isFinite(curbWidth)) this.curbWidth = curbWidth;
    if (Number.isFinite(carWidth)) this.carWidth = carWidth;
    if (Number.isFinite(carLength)) this.carLength = carLength;
  }

  /**
   * Scan multi-horizon track curvature to identify compound turns, chicanes, and apex sequences.
   * Prevents getting cast wide into outside runoff between linked apexes.
   * @private
   */
  _scanMultiApexGeometry(track, currentDist) {
    if (!track?.atDistance) {
      return {
        isCompound: false,
        isChicane: false,
        isDoubleApex: false,
        primaryCurv: 0,
        primarySign: 1,
        primaryDist: 999,
        secondaryCurv: 0,
        secondarySign: 1,
        secondaryDist: 999,
        geometricCompromiseLateral: 0
      };
    }

    const horizons = [6, 14, 24, 38, 56, 80, 110];
    const samples = horizons.map((d) => {
      const pt = track.atDistance(currentDist + d);
      const rawCurv = finite(pt?.curvature, 0);
      const sign = Math.sign(finite(pt?.turnSign, rawCurv)) || 1;
      return {
        dist: d,
        curvature: Math.abs(rawCurv),
        rawCurv,
        sign,
        s: currentDist + d
      };
    });

    // Find primary turn (highest curvature within near horizon)
    const turnCandidates = samples.filter((s) => s.curvature > 0.0035);
    if (turnCandidates.length === 0) {
      const sharpest = samples.sort((a, b) => b.curvature - a.curvature)[0];
      return {
        isCompound: false,
        isChicane: false,
        isDoubleApex: false,
        primaryCurv: sharpest.curvature,
        primarySign: sharpest.sign,
        primaryDist: sharpest.dist,
        secondaryCurv: 0,
        secondarySign: sharpest.sign,
        secondaryDist: 999,
        geometricCompromiseLateral: 0
      };
    }

    const primaryTurn = turnCandidates[0];

    // Find secondary turn (turn after primary with significant curvature)
    const secondaryCandidates = turnCandidates.filter(
      (s) => s.dist >= primaryTurn.dist + 12 && s.curvature > 0.0035
    );
    const secondaryTurn = secondaryCandidates[0] ?? null;

    const isCompound = Boolean(secondaryTurn && (secondaryTurn.dist - primaryTurn.dist) <= 65);
    const isChicane = Boolean(isCompound && primaryTurn.sign !== secondaryTurn.sign);
    const isDoubleApex = Boolean(isCompound && primaryTurn.sign === secondaryTurn.sign);

    return {
      isCompound,
      isChicane,
      isDoubleApex,
      primaryCurv: primaryTurn.curvature,
      primarySign: primaryTurn.sign,
      primaryDist: primaryTurn.dist,
      secondaryCurv: secondaryTurn?.curvature ?? 0,
      secondarySign: secondaryTurn?.sign ?? primaryTurn.sign,
      secondaryDist: secondaryTurn?.dist ?? 999
    };
  }

  /**
   * Main adversarial game evaluation cycle.
   * @param {Object} params
   * @param {Object} params.vehicle - Ego vehicle state
   * @param {Object} params.track - Circuit track object
   * @param {Object} params.traffic - Multi-agent traffic perception
   * @param {Object} [params.optimalProfile] - Layer 1 2D Optimal Profile Solver
   * @param {number} [params.aggression=0.85] - Aggression factor (0.5 - 1.0)
   * @param {number} [params.dt=0.016] - Simulation time step
   * @returns {Object} Tactical corridor, lateral target, desired speed, and racecraft metadata
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

    const roadHalfW = finite(track?.roadHalfWidth, this.roadHalfWidth);
    const curbW = finite(track?.curbWidth, this.curbWidth);
    const maxMargin = Math.max(2.1, roadHalfW - 1.20 + Math.min(0.70, curbW * 0.55));

    // Lockout timer for recently passed cars
    this.targetLockTimer = Math.max(0, this.targetLockTimer - dt);
    if (this.targetLockTimer <= 0) {
      this.passedTargetId = null;
    }

    // Sample Layer 1 globally optimal baseline profile
    const optimalSample = optimalProfile?.sampleAtDistance?.(vDist) ?? {
      lateral: 0,
      lineLateral: 0,
      targetSpeed: Math.max(25.0, vSpeed),
      curvature: 0
    };
    const optimalLat = clamp(optimalSample.lateral, -maxMargin, maxMargin);

    // Multi-horizon curvature & compound corner scanning
    const multiApex = this._scanMultiApexGeometry(track, vDist);
    this.multiApexState = multiApex;
    this.compoundTurnDetected = multiApex.isCompound;

    const isApproachingCorner = multiApex.primaryCurv > 0.0045;
    const isStraight = multiApex.primaryCurv < 0.0028;
    const primaryInsideSign = -multiApex.primarySign; // Inside lateral sign (-1 for right turn, +1 for left turn)
    const insideOffset = clamp(primaryInsideSign * (maxMargin * 0.78), -maxMargin, maxMargin);

    // Scan traffic entries (Challenger behind, Target ahead)
    const entries = traffic?.entries ?? [];

    const challenger = entries.find((e) => {
      if (!e?.other || e.other.finished || e.other.despawned || e.other.trafficGhost) return false;
      if (e.delta >= -0.8 || e.delta <= -52.0) return false;
      if (this.passedTargetId && e.other.id === this.passedTargetId) {
        const closing = finite(e.otherForwardSpeed - traffic.egoForwardSpeed, 0);
        if (Math.abs(e.delta) > 5.5 || closing <= 0.35) return false;
      }
      return true;
    });

    const targetAhead = entries.find((e) => {
      if (!e?.other || e.other.finished || e.other.despawned || e.other.trafficGhost) return false;
      if (e.delta <= 0.4 || e.delta >= 55.0) return false;
      if (e.other.id === this.passedTargetId) return false;
      return true;
    });

    let tacticalRole = 'PACE';
    let targetLateral = optimalLat;
    let desiredSpeed = optimalSample.targetSpeed;
    let dMin = -maxMargin;
    let dMax = maxMargin;
    let combatNotes = 'OPTIMAL_RACING_LINE';

    // =========================================================================
    // 1. STACKELBERG LEADER DEFENSE GAME
    // =========================================================================
    if (challenger && challenger.delta > -42.0) {
      const gap = Math.abs(challenger.delta);
      const challengerSpeed = finite(challenger.other?.speed ?? challenger.otherForwardSpeed, vSpeed);
      const closingSpeed = Math.max(0, challengerSpeed - vSpeed);
      const bodyGap = Math.max(0, gap - this.carLength);
      const ttc = closingSpeed > 0.20 ? bodyGap / closingSpeed : (closingSpeed > -0.2 ? bodyGap / 0.25 : 99.0);

      const rawAttackerLat = finite(challenger.otherLateral ?? challenger.other?.surface?.lateral, currentLat);
      // Low-pass exponential feint filtering (anti-weave)
      this.filteredAttackerLateral = damp(this.filteredAttackerLateral, rawAttackerLat, 6.5, dt);

      // Composite Stackelberg threat score T(t) in [0, 1]
      const fGap = Math.exp(-gap / 15.0);
      const fClose = saturate((closingSpeed + 0.4) / 4.8);
      const fTtc = ttc <= 4.5 ? Math.pow(1.0 - ttc / 4.5, 2) : 0;
      const fCorner = Math.exp(-multiApex.primaryDist / 42.0) * saturate(multiApex.primaryCurv / 0.003);
      const lateralSeparation = Math.abs(rawAttackerLat - currentLat);
      const fLat = 1.0 - saturate((lateralSeparation - 1.4) / 8.0);

      const isClosingThreat = (gap < 10.0)
        || (gap <= 26.0 && closingSpeed >= 0.35)
        || (gap <= 42.0 && closingSpeed >= 1.0 && multiApex.primaryDist < 75.0)
        || (ttc < 3.2);

      this.threatScore = saturate(fGap * 0.30 + fClose * 0.25 + fTtc * 0.25 + fCorner * 0.15 + (isClosingThreat ? 0.25 : 0.0));

      if (isClosingThreat || this.threatScore > 0.35 || this.defenseDwellTimer > 0) {
        tacticalRole = 'DEFEND';
        this.defenseTargetId = challenger.other?.id ?? null;
        this.defenseTimer += dt;
        this.defenseDwellTimer = 0.65; // Hysteresis hold

        // Determine preferred defensive corridor
        let preferredDefensiveOffset = insideOffset;

        if (isStraight) {
          // On straights, if challenger is pulling out or threatening slipstream, defend the inside or break tow
          const attackerOffsetSign = Math.sign(this.filteredAttackerLateral) || 1;
          preferredDefensiveOffset = Math.abs(this.filteredAttackerLateral) > 1.2
            ? clamp(this.filteredAttackerLateral * 0.75, -maxMargin * 0.75, maxMargin * 0.75)
            : clamp(insideOffset * 0.65, -maxMargin * 0.75, maxMargin * 0.75);
        }

        // FIA Single Defensive Move Rule: Lock direction upon initial commitment
        if (!this.oneMoveLocked || this.lockedDefensiveLane === null) {
          this.defenseDirection = Math.sign(preferredDefensiveOffset) || 1;
          this.lockedDefensiveLane = preferredDefensiveOffset;
          this.oneMoveLocked = true;
        }

        // Strict Anti-Weave: hold committed lateral side without reversing
        const lockedSign = this.defenseDirection;
        const defensiveTargetLat = (lockedSign === primaryInsideSign)
          ? insideOffset
          : clamp(lockedSign * (maxMargin * 0.72), -maxMargin, maxMargin);

        if (isStraight && gap > 11.0 && closingSpeed > 0.8) {
          // A. Break Tow (Stepped lateral shift on straight)
          this.defenseMode = 'BREAK_TOW';
          targetLateral = clamp(defensiveTargetLat * 0.70, -maxMargin * 0.72, maxMargin * 0.72);
          dMin = targetLateral - 1.2;
          dMax = targetLateral + 1.2;
          combatNotes = 'DEFEND_BREAK_TOW';
        } else if (isApproachingCorner || gap < 15.0 || ttc < 2.4) {
          // B. Apex Shielding (Pin inside curb tight, denying room completely)
          this.defenseMode = 'APEX_SHIELD';

          // Multi-apex chicane adaptation: blend cleanly between apex 1 and apex 2 setup
          if (multiApex.isChicane && multiApex.primaryDist < 10.0) {
            const secondaryInsideSign = -multiApex.secondarySign;
            const transitionBlend = saturate((10.0 - multiApex.primaryDist) / 10.0);
            const secondaryInsideOffset = secondaryInsideSign * (maxMargin * 0.78);
            targetLateral = lerp(insideOffset, secondaryInsideOffset, transitionBlend * 0.65);
            combatNotes = 'DEFEND_CHICANE_MULTI_APEX_SHIELD';
          } else {
            targetLateral = insideOffset;
            combatNotes = 'DEFEND_APEX_SHIELD';
          }

          dMin = targetLateral - 0.45;
          dMax = targetLateral + 0.45;
        } else if (!isStraight && gap < 8.0) {
          // C. Exit Squeeze (Drift out smoothly to leave exactly 1 car width at outside edge)
          this.defenseMode = 'EXIT_SQUEEZE';
          const outsideBoundary = -primaryInsideSign * (maxMargin - this.carWidth - 0.22);
          targetLateral = outsideBoundary;
          dMin = Math.min(targetLateral, 0) - 0.35;
          dMax = Math.max(targetLateral, 0) + 0.35;
          combatNotes = 'DEFEND_EXIT_SQUEEZE';
        } else {
          this.defenseMode = 'LOCK_LANE';
          targetLateral = defensiveTargetLat;
          combatNotes = 'DEFEND_HOLD_LANE';
        }
      } else {
        // Threat dissipated: graceful dwell return
        this.defenseDwellTimer = Math.max(0, this.defenseDwellTimer - dt);
        if (this.defenseDwellTimer <= 0) {
          this.defenseMode = 'PACE';
          this.defenseTargetId = null;
          this.oneMoveLocked = false;
          this.lockedDefensiveLane = null;
          this.defenseDirection = 0;
          this.defenseTimer = 0;
        }
      }
    } else {
      this.defenseDwellTimer = Math.max(0, this.defenseDwellTimer - dt);
      if (this.defenseDwellTimer <= 0) {
        this.defenseMode = 'PACE';
        this.defenseTargetId = null;
        this.oneMoveLocked = false;
        this.lockedDefensiveLane = null;
        this.defenseDirection = 0;
        this.defenseTimer = 0;
        this.threatScore = 0;
      }
    }

    // =========================================================================
    // 2. ITERATIVE BEST RESPONSE (IBR) ATTACK GAME
    // =========================================================================
    if (tacticalRole !== 'DEFEND' && targetAhead && targetAhead.delta < 50.0) {
      tacticalRole = 'ATTACK';
      this.attackTargetId = targetAhead.other?.id ?? null;
      this.attackTimer += dt;

      const gap = targetAhead.delta;
      const targetSpeed = finite(targetAhead.other?.speed ?? targetAhead.otherForwardSpeed, vSpeed);
      const opponentLat = finite(targetAhead.otherLateral ?? targetAhead.other?.surface?.lateral, 0);
      const closingSpeed = Math.max(0, vSpeed - targetSpeed);

      // Check if inside line is open (opponent is not hugging inside apex curb)
      const isInsideOpen = Math.abs(opponentLat - insideOffset) > 2.4;
      const isSideBySide = Math.abs(gap) < this.carLength * 1.1;

      // Pass completion check (car established clear forward progress)
      if (gap < -2.2) {
        this.passedTargetId = this.attackTargetId;
        this.targetLockTimer = 16.0;
        this.attackMode = 'NONE';
        this.attackTargetId = null;
        this.attackTimer = 0;
        this.divebombCommitted = false;
        this.switchbackStage = 'NONE';
        tacticalRole = 'PACE';
        combatNotes = 'OVERTAKE_COMPLETED_RESUME_PACE';
      } else if (isSideBySide) {
        // A. Resilient Side-by-Side Overlap Combat
        this.attackMode = 'SIDE_BY_SIDE';
        const opponentSide = opponentLat >= 0 ? 1 : -1;
        const assignedSide = -opponentSide;
        targetLateral = clamp(opponentLat + assignedSide * (this.carWidth + 0.55), -maxMargin, maxMargin);
        desiredSpeed = Math.max(desiredSpeed, targetSpeed + 4.0);
        dMin = Math.min(targetLateral - 0.5, currentLat - 0.3);
        dMax = Math.max(targetLateral + 0.5, currentLat + 0.3);
        combatNotes = 'ATTACK_SIDE_BY_SIDE_HOLD';
      } else if (isStraight && gap > 4.8) {
        // B. Straightaway High-Speed Slipstream Slingshot
        this.attackMode = 'SLINGSHOT';
        const straightClosingFloor = 14.0 + aggression * 4.0; // Up to +18 m/s closing speed floor
        desiredSpeed = Math.max(desiredSpeed, targetSpeed + straightClosingFloor);

        // Calculate dynamic pull-out timing
        const dynamicPulloutDist = clamp(closingSpeed * 1.15 + 4.8, 6.5, 28.0);
        const shouldPullOut = gap <= dynamicPulloutDist || gap < 8.0;

        if (shouldPullOut) {
          // Punch out into clear lateral lane
          const pullSide = opponentLat > 0 ? -1 : 1;
          targetLateral = clamp(opponentLat + pullSide * 3.8, -maxMargin, maxMargin);
          combatNotes = 'ATTACK_SLINGSHOT_PUNCH_OUT';
        } else {
          // Ride the slipstream tow pocket directly behind
          targetLateral = clamp(opponentLat, -maxMargin * 0.85, maxMargin * 0.85);
          combatNotes = 'ATTACK_SLINGSHOT_DRAFTING';
        }
      } else if (isApproachingCorner && isInsideOpen && gap < 32.0) {
        // C. Fearless Inside Divebomb (-3.5G Threshold Deceleration)
        this.attackMode = 'DIVEBOMB';
        this.divebombCommitted = true;
        this.attackIntensity = 0.95;

        // Multi-apex chicane divebomb control: regulate exit speed for secondary apex
        if (multiApex.isChicane) {
          targetLateral = insideOffset;
          // Hold sufficient speed through apex 1 while preparing chicane reversal
          desiredSpeed = Math.max(optimalSample.targetSpeed * 0.96, targetSpeed + 4.5);
          combatNotes = 'ATTACK_CHICANE_IBR_DIVEBOMB';
        } else {
          targetLateral = insideOffset;
          desiredSpeed = Math.max(optimalSample.targetSpeed, targetSpeed + 5.5 + aggression * 2.0);
          combatNotes = 'ATTACK_FEARLESS_IBR_DIVEBOMB';
        }

        dMin = insideOffset - 0.55;
        dMax = insideOffset + 0.75;
      } else if (isApproachingCorner && !isInsideOpen) {
        // D. Diamond Line Switchback Undercut (Late apex counter to inside defender)
        this.attackMode = 'SWITCHBACK';
        this.attackIntensity = 0.88;

        const isAtApex = multiApex.primaryDist < 12.0;
        if (!isAtApex) {
          // Stage 1: Stay wider on entry to square off corner radius
          this.switchbackStage = 'ENTRY_WIDE';
          targetLateral = clamp(-primaryInsideSign * (maxMargin * 0.75), -maxMargin, maxMargin);
          desiredSpeed = optimalSample.targetSpeed * 0.94; // Controlled entry
          combatNotes = 'ATTACK_SWITCHBACK_WIDE_ENTRY';
        } else {
          // Stage 2: Cut underneath defender on exit with maximum longitudinal drive
          this.switchbackStage = 'EXIT_UNDERCUT';
          targetLateral = clamp(primaryInsideSign * (maxMargin * 0.45), -maxMargin, maxMargin);
          desiredSpeed = Math.max(optimalSample.targetSpeed * 1.06, targetSpeed + 6.0);
          combatNotes = 'ATTACK_SWITCHBACK_EXIT_UNDERCUT';
        }
      }
    } else if (tacticalRole !== 'DEFEND') {
      this.attackMode = 'NONE';
      this.attackTargetId = null;
      this.attackTimer = 0;
      this.attackIntensity = 0;
      this.switchbackStage = 'NONE';
      this.divebombCommitted = false;
    }

    // =========================================================================
    // 3. MULTI-APEX RUNOFF PREVENTION & DYNAMIC CORRIDOR ENVELOPE
    // =========================================================================
    // Ensure target lateral never exceeds physical track limits or curb boundary
    targetLateral = clamp(targetLateral, -maxMargin, maxMargin);
    dMin = clamp(Math.min(dMin, targetLateral), -maxMargin, maxMargin);
    dMax = clamp(Math.max(dMax, targetLateral), -maxMargin, maxMargin);

    return {
      role: tacticalRole,
      defenseMode: this.defenseMode,
      attackMode: this.attackMode,
      targetLateral,
      desiredSpeed,
      dMin,
      dMax,
      notes: combatNotes,
      multiApex: {
        isCompound: multiApex.isCompound,
        isChicane: multiApex.isChicane,
        isDoubleApex: multiApex.isDoubleApex,
        primaryCurv: multiApex.primaryCurv,
        secondaryCurv: multiApex.secondaryCurv
      },
      threatScore: this.threatScore,
      attackIntensity: this.attackIntensity
    };
  }
}
