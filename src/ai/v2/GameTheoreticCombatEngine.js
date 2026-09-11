/**
 * GameTheoreticCombatEngine.js (V2 Layer 2)
 * Predictive Adversarial Racecraft Engine:
 * - Dynamic tactical attack and defense state transitions with multi-agent threat assessment
 * - Defense:
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
    this.attackSide = 0;
    this.attackSideLocked = false;
    this.passClearDwell = 0;
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
    // Utilize full physical track width respecting vehicle half-width
    const maxMargin = Math.max(2.1, Math.min(5.35, roadHalfW - 1.20 + Math.min(0.50, curbW * 0.40)));

    // Lockout timer for recently passed cars
    this.targetLockTimer = Math.max(0, this.targetLockTimer - dt);
    if (this.targetLockTimer <= 0) {
      this.passedTargetId = null;
    }

    // Sample Layer 1 globally optimal baseline profile
    const vClass = vehicle?.classKey || 'gt';
    const optimalSample = optimalProfile?.sampleAtDistance?.(vDist, vClass) ?? {
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

    const isApproachingCorner = multiApex.primaryCurv > 0.0042;
    const isStraight = multiApex.primaryCurv < 0.0028;
    const primaryInsideSign = multiApex.primarySign; // Inside lateral sign (+1 for left turn, -1 for right turn)
    const insideOffset = clamp(primaryInsideSign * (maxMargin * 0.85), -maxMargin, maxMargin);

    // Scan traffic entries (Challenger behind, Target ahead)
    const entries = traffic?.entries ?? [];

    const challenger = entries.find((e) => {
      if (!e?.other || e.other.finished || e.other.despawned || e.other.trafficGhost) return false;
      if (e.delta >= -0.8 || e.delta <= -45.0) return false;
      const closing = finite(e.otherForwardSpeed - traffic.egoForwardSpeed, 0);
      if (this.passedTargetId && e.other.id === this.passedTargetId) {
        if (Math.abs(e.delta) > 6.0 || closing <= 0.35) return false;
      }
      if (closing < -0.3 && Math.abs(e.delta) > 8.0) return false;
      return true;
    });

    const activeAttackEntry = this.attackTargetId
      ? entries.find((e) => e.other?.id === this.attackTargetId)
      : null;

    const requiredPassClearance = this.carLength + 0.65;
    if (activeAttackEntry) {
      if (activeAttackEntry.delta < -requiredPassClearance) {
        this.passClearDwell += dt;
        if (this.passClearDwell >= 0.20) {
          this.passedTargetId = this.attackTargetId;
          this.targetLockTimer = 16.0;
          this.attackMode = 'NONE';
          this.attackTargetId = null;
          this.attackTimer = 0;
          this.divebombCommitted = false;
          this.switchbackStage = 'NONE';
          this.attackSideLocked = false;
          this.attackSide = 0;
          this.passClearDwell = 0;
        }
      } else {
        this.passClearDwell = 0;
      }
    }

    const targetAhead = entries.find((e) => {
      if (!e?.other || e.other.finished || e.other.despawned || e.other.trafficGhost) return false;
      if (e.delta <= 0.4 || e.delta >= 55.0) return false;
      if (e.other.id === this.passedTargetId) return false;
      return true;
    });

    // =========================================================================
    // 1. EVALUATE DEFENSE THREAD (Stackelberg Leader)
    // =========================================================================
    let isDefending = false;
    let defTargetLat = optimalLat;
    let defDesiredSpeed = optimalSample.targetSpeed;
    let defNotes = 'PACE';

    if (challenger && challenger.delta > -45.0) {
      const gap = Math.abs(challenger.delta);
      const challengerSpeed = finite(challenger.other?.speed ?? challenger.otherForwardSpeed, vSpeed);
      const closingSpeed = Math.max(0, challengerSpeed - vSpeed);
      const bodyGap = Math.max(0, gap - this.carLength);
      const ttc = closingSpeed > 0.20 ? bodyGap / closingSpeed : (closingSpeed > -0.2 ? bodyGap / 0.25 : 99.0);

      const rawAttackerLat = finite(challenger.otherLateral ?? challenger.other?.surface?.lateral, currentLat);
      this.filteredAttackerLateral = damp(this.filteredAttackerLateral, rawAttackerLat, 6.5, dt);

      // Composite Stackelberg threat score T(t) in [0, 1]
      const fGap = Math.exp(-gap / 16.0);
      const fClose = saturate((closingSpeed + 0.4) / 4.8);
      const fTtc = ttc <= 4.5 ? Math.pow(1.0 - ttc / 4.5, 2) : 0;
      const fCorner = Math.exp(-multiApex.primaryDist / 45.0) * saturate(multiApex.primaryCurv / 0.003);

      const isClosingThreat = (gap < 10.0 && closingSpeed > 0.20 && vSpeed > 14.0)
        || (gap <= 24.0 && closingSpeed >= 0.40 && vSpeed > 16.0)
        || (gap <= 42.0 && closingSpeed >= 0.80 && multiApex.primaryDist < 85.0 && vSpeed > 20.0)
        || (ttc < 3.0 && closingSpeed > 0.35 && vSpeed > 16.0);

      const isAttackerRealThreat = (closingSpeed > 0.30 && vSpeed > 14.0)
        || (gap < 7.0 && closingSpeed > 0.15 && vSpeed > 12.0)
        || (ttc < 3.0 && gap < 20.0 && vSpeed > 16.0);

      this.threatScore = isAttackerRealThreat
        ? saturate(fGap * 0.30 + fClose * 0.25 + fTtc * 0.25 + fCorner * 0.15 + (isClosingThreat ? 0.30 : 0.0))
        : 0;

      if ((isClosingThreat || this.threatScore > 0.35 || this.defenseDwellTimer > 0) && isAttackerRealThreat) {
        isDefending = true;
        this.defenseTargetId = challenger.other?.id ?? null;
        this.defenseTimer += dt;
        this.defenseDwellTimer = 0.65;

        let preferredDefensiveOffset = insideOffset;
        if (isStraight) {
          const attackerOffsetSign = Math.sign(this.filteredAttackerLateral) || 1;
          preferredDefensiveOffset = Math.abs(this.filteredAttackerLateral) > 1.2
            ? clamp(this.filteredAttackerLateral * 0.75, -maxMargin * 0.75, maxMargin * 0.75)
            : clamp(insideOffset * 0.65, -maxMargin * 0.75, maxMargin * 0.75);
        }

        // FIA Single Defensive Move Rule
        if (!this.oneMoveLocked || this.lockedDefensiveLane === null) {
          this.defenseDirection = Math.sign(preferredDefensiveOffset) || primaryInsideSign;
          this.lockedDefensiveLane = preferredDefensiveOffset;
          this.oneMoveLocked = true;
        }

        const lockedSign = this.defenseDirection;
        const defensiveTargetLat = (lockedSign === primaryInsideSign)
          ? insideOffset
          : clamp(lockedSign * (maxMargin * 0.75), -maxMargin, maxMargin);

        if (isStraight && gap > 11.0 && closingSpeed > 0.6) {
          this.defenseMode = 'BREAK_TOW';
          defTargetLat = clamp(defensiveTargetLat * 0.70, -maxMargin * 0.75, maxMargin * 0.75);
          defNotes = 'DEFEND_BREAK_TOW';
        } else if (isApproachingCorner || gap < 15.0 || ttc < 2.4) {
          this.defenseMode = 'APEX_SHIELD';
          if (multiApex.isChicane && multiApex.primaryDist < 12.0) {
            const secondaryInsideSign = multiApex.secondarySign;
            const transitionBlend = saturate((12.0 - multiApex.primaryDist) / 12.0);
            const secondaryInsideOffset = secondaryInsideSign * (maxMargin * 0.85);
            defTargetLat = lerp(insideOffset, secondaryInsideOffset, transitionBlend * 0.60);
            defNotes = 'DEFEND_CHICANE_MULTI_APEX_SHIELD';
          } else {
            defTargetLat = insideOffset;
            defNotes = 'DEFEND_APEX_SHIELD';
          }
        } else if (!isStraight && gap < 8.0) {
          this.defenseMode = 'EXIT_SQUEEZE';
          const outsideBoundary = clamp(-primaryInsideSign * (maxMargin - this.carWidth - 0.20), -maxMargin, maxMargin);
          defTargetLat = outsideBoundary;
          defNotes = 'DEFEND_EXIT_SQUEEZE';
        } else {
          this.defenseMode = 'LOCK_LANE';
          defTargetLat = defensiveTargetLat;
          defNotes = 'DEFEND_HOLD_LANE';
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
    // 2. EVALUATE ATTACK THREAD (Iterative Best Response & Fearless Divebomb)
    // =========================================================================
    let isAttacking = false;
    let atkTargetLat = optimalLat;
    let atkDesiredSpeed = optimalSample.targetSpeed;
    let atkNotes = 'NONE';

    if (targetAhead && targetAhead.delta < 55.0) {
      isAttacking = true;
      this.attackTargetId = targetAhead.other?.id ?? null;
      this.attackTimer += dt;

      const gap = targetAhead.delta;
      const targetSpeed = finite(targetAhead.other?.speed ?? targetAhead.otherForwardSpeed, vSpeed);
      const opponentLat = finite(targetAhead.otherLateral ?? targetAhead.other?.surface?.lateral, 0);
      const closingSpeed = Math.max(0, vSpeed - targetSpeed);

      // Check if inside line is open (opponent is leaving space on inside curb)
      const isInsideOpen = Math.abs(opponentLat - insideOffset) > 1.6;
      const isSideBySide = Math.abs(gap) < this.carLength * 1.35;

      if (gap < -requiredPassClearance) {
        this.passClearDwell += dt;
        if (this.passClearDwell >= 0.20) {
          this.passedTargetId = this.attackTargetId;
          this.targetLockTimer = 16.0;
          this.attackMode = 'NONE';
          this.attackTargetId = null;
          this.attackTimer = 0;
          this.divebombCommitted = false;
          this.switchbackStage = 'NONE';
          this.attackSideLocked = false;
          this.attackSide = 0;
          this.passClearDwell = 0;
          isAttacking = false;
        }
      } else {
        this.passClearDwell = 0;
      }

      if (isAttacking) {
        if (isSideBySide) {
          // Resilient Side-by-Side Overlap Combat (Keep assigned locked flank with guaranteed daylight)
          this.attackMode = 'SIDE_BY_SIDE';
          if (!this.attackSideLocked || this.attackSide === 0) {
            this.attackSide = currentLat >= opponentLat ? 1 : -1;
            this.attackSideLocked = true;
          }
          const mySide = this.attackSide;
          const minDaylight = 0.35;
          atkTargetLat = clamp(opponentLat + mySide * (this.carWidth + minDaylight), -maxMargin, maxMargin);
          atkDesiredSpeed = Math.min(optimalSample.targetSpeed * 1.04, Math.max(optimalSample.targetSpeed * 0.95, targetSpeed + 2.5));
          atkNotes = 'ATTACK_SIDE_BY_SIDE_HOLD';
        } else if (isStraight && gap > 4.5) {
          // High-Speed Slipstream Slingshot
          this.attackMode = 'SLINGSHOT';
          const straightClosingFloor = 10.0 + aggression * 3.5;
          atkDesiredSpeed = Math.max(optimalSample.targetSpeed, targetSpeed + straightClosingFloor);

          const dynamicPulloutDist = clamp(closingSpeed * 1.2 + 4.5, 6.0, 24.0);
          const shouldPullOut = gap <= dynamicPulloutDist || gap < 15.0;

          if (shouldPullOut) {
            if (!this.attackSideLocked || this.attackSide === 0) {
              this.attackSide = opponentLat >= 0 ? -1 : 1;
            }
            const pullSide = this.attackSide;
            atkTargetLat = clamp(opponentLat + pullSide * 3.2, -maxMargin, maxMargin);
            atkNotes = 'ATTACK_SLINGSHOT_PUNCH_OUT';
          } else {
            // Ride the slipstream tow pocket directly behind
            atkTargetLat = clamp(opponentLat, -maxMargin * 0.88, maxMargin * 0.88);
            atkNotes = 'ATTACK_SLINGSHOT_DRAFTING';
          }
        } else if (isApproachingCorner && isInsideOpen && gap < 28.0 && vSpeed > 22.0 && closingSpeed > 0.5) {
          // Inside Apex Pass
          this.attackMode = 'DIVEBOMB';
          this.divebombCommitted = true;
          this.attackIntensity = 0.96;
          if (!this.attackSideLocked || this.attackSide === 0) {
            this.attackSide = primaryInsideSign;
            this.attackSideLocked = true;
          }
          atkTargetLat = insideOffset;

          if (multiApex.isChicane) {
            atkDesiredSpeed = Math.max(optimalSample.targetSpeed * 0.99, targetSpeed + 3.5);
            atkNotes = 'ATTACK_CHICANE_IBR_DIVEBOMB';
          } else {
            atkDesiredSpeed = Math.max(optimalSample.targetSpeed * 1.02, targetSpeed + 4.0);
            atkNotes = 'ATTACK_FEARLESS_IBR_DIVEBOMB';
          }
        } else if (isApproachingCorner && !isInsideOpen && gap < 28.0 && vSpeed > 22.0) {
          // Diamond Line Switchback Undercut (Late apex counter to inside defender)
          this.attackMode = 'SWITCHBACK';
          this.attackIntensity = 0.90;
          if (!this.attackSideLocked || this.attackSide === 0) {
            this.attackSide = -primaryInsideSign;
            this.attackSideLocked = true;
          }
          const isAtApex = multiApex.primaryDist < 12.0;

          if (!isAtApex) {
            this.switchbackStage = 'ENTRY_WIDE';
            atkTargetLat = clamp(-primaryInsideSign * (maxMargin * 0.82), -maxMargin, maxMargin);
            atkDesiredSpeed = optimalSample.targetSpeed * 0.97;
            atkNotes = 'ATTACK_SWITCHBACK_WIDE_ENTRY';
          } else {
            this.switchbackStage = 'EXIT_UNDERCUT';
            atkTargetLat = clamp(primaryInsideSign * (maxMargin * 0.50), -maxMargin, maxMargin);
            atkDesiredSpeed = Math.max(optimalSample.targetSpeed * 1.04, targetSpeed + 3.5);
            atkNotes = 'ATTACK_SWITCHBACK_EXIT_UNDERCUT';
          }
        } else {
          // Dynamic Overtake Corridor Selection (Dual-Flank Bypass)
          const isSlower = targetSpeed < vSpeed - 1.5 || targetSpeed < 20.0 || gap < 22.0;
          if (isSlower) {
            this.attackMode = 'OVERTAKE';
            const leftSpace = maxMargin + opponentLat;
            const rightSpace = maxMargin - opponentLat;
            const minPassWidth = this.carWidth + 0.65;
            if (!this.attackSideLocked || this.attackSide === 0) {
              if (currentLat > opponentLat + 0.35 && rightSpace >= minPassWidth) {
                this.attackSide = 1;
              } else if (currentLat < opponentLat - 0.35 && leftSpace >= minPassWidth) {
                this.attackSide = -1;
              } else {
                this.attackSide = leftSpace >= rightSpace ? -1 : 1;
              }
            }
            const passSide = this.attackSide;
            const targetPassOffset = opponentLat + passSide * Math.min(3.2, Math.max(minPassWidth, (passSide < 0 ? leftSpace : rightSpace) * 0.55));
            atkTargetLat = clamp(targetPassOffset, -maxMargin, maxMargin);
            atkDesiredSpeed = Math.min(optimalSample.targetSpeed * 1.04, targetSpeed + 3.0 + aggression * 2.0);
            atkNotes = 'ATTACK_OVERTAKE_BYPASS';
          } else {
            atkTargetLat = optimalLat;
            atkDesiredSpeed = optimalSample.targetSpeed;
            atkNotes = 'ATTACK_PURSUIT_LINE';
          }
        }
      }
    } else {
      this.attackMode = 'NONE';
      this.attackTargetId = null;
      this.attackTimer = 0;
      this.attackIntensity = 0;
      this.switchbackStage = 'NONE';
      this.divebombCommitted = false;
      this.attackSideLocked = false;
      this.attackSide = 0;
      this.passClearDwell = 0;
    }

    // =========================================================================
    // 3. DUAL-THREAD TACTICAL SYNTHESIS (Simultaneous Attack & Defense)
    // =========================================================================
    let tacticalRole = 'PACE';
    let targetLateral = optimalLat;
    let desiredSpeed = optimalSample.targetSpeed;
    let dMin = -maxMargin;
    let dMax = maxMargin;
    let combatNotes = 'OPTIMAL_RACING_LINE';

    if (isDefending && isAttacking) {
      tacticalRole = 'DUAL_COMBAT';

      if (this.attackMode === 'DIVEBOMB') {
        // Diving inside target ahead naturally closes the inside on the challenger behind!
        targetLateral = atkTargetLat;
        desiredSpeed = Math.max(atkDesiredSpeed, defDesiredSpeed + 3.0);
        dMin = targetLateral - 0.50;
        dMax = targetLateral + 0.65;
        combatNotes = 'COMBAT_DUAL_DIVE_AND_SHIELD';
      } else if (this.attackMode === 'SWITCHBACK') {
        // Carry diamond entry while maintaining high speed so car behind cannot lunge
        targetLateral = atkTargetLat;
        desiredSpeed = Math.max(atkDesiredSpeed, vSpeed + 1.5);
        dMin = targetLateral - 0.70;
        dMax = targetLateral + 0.70;
        combatNotes = 'COMBAT_DUAL_SWITCHBACK_AND_DEFEND';
      } else if (this.attackMode === 'SLINGSHOT') {
        // Slingshot forward while breaking tow for the car behind
        targetLateral = atkTargetLat;
        desiredSpeed = Math.max(atkDesiredSpeed, defDesiredSpeed + 4.0);
        dMin = targetLateral - 1.0;
        dMax = targetLateral + 1.0;
        combatNotes = 'COMBAT_DUAL_SLINGSHOT_TOW_BREAK';
      } else {
        // Side by side combat: hold assigned flank firmly
        targetLateral = atkTargetLat;
        desiredSpeed = Math.max(atkDesiredSpeed, defDesiredSpeed);
        dMin = targetLateral - 0.50;
        dMax = targetLateral + 0.50;
        combatNotes = 'COMBAT_DUAL_TACTICAL_HOLD';
      }
    } else if (isDefending) {
      tacticalRole = 'DEFEND';
      targetLateral = defTargetLat;
      desiredSpeed = defDesiredSpeed;
      dMin = (this.defenseMode === 'APEX_SHIELD') ? targetLateral - 0.45 : targetLateral - 1.2;
      dMax = (this.defenseMode === 'APEX_SHIELD') ? targetLateral + 0.45 : targetLateral + 1.2;
      combatNotes = defNotes;
    } else if (isAttacking) {
      tacticalRole = 'ATTACK';
      targetLateral = atkTargetLat;
      desiredSpeed = atkDesiredSpeed;
      dMin = (this.attackMode === 'DIVEBOMB') ? targetLateral - 0.55 : targetLateral - 1.2;
      dMax = (this.attackMode === 'DIVEBOMB') ? targetLateral + 0.75 : targetLateral + 1.2;
      combatNotes = atkNotes;
    }

    // Ensure target lateral never exceeds physical track limits or curb boundary
    targetLateral = clamp(targetLateral, -maxMargin, maxMargin);
    dMin = clamp(Math.min(dMin, targetLateral), -maxMargin, maxMargin);
    dMax = clamp(Math.max(dMax, targetLateral), -maxMargin, maxMargin);

    return {
      role: tacticalRole,
      defenseMode: this.defenseMode,
      attackMode: this.attackMode,
      attackTargetId: this.attackTargetId,
      attackSide: this.attackSide,
      attackSideLocked: this.attackSideLocked,
      passedTargetId: this.passedTargetId,
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
