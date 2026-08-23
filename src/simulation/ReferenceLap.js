/**
 * ReferenceLap.js
 * User Reference Lap Recorder & Telemetry Baseline Engine:
 * - Records player driving trajectory (distance, lateral offset, speed, timestamps)
 * - Tracks lap times, sectors, and lap completion
 * - Generates high-density interpolated reference racing lines and target speeds
 * - Calculates live lap delta time (+/- 0.000s) vs user baseline
 * - LocalStorage persistence and AI controller synchronization
 */

const finite = (val, fallback = 0) => (Number.isFinite(val) ? val : fallback);
const clamp = (val, min, max) => Math.max(min, Math.min(max, val));

export class ReferenceLapManager {
  /**
   * @param {Object} track - Circuit track instance
   */
  constructor(track) {
    this.track = track;
    this.trackLength = finite(track?.totalLength ?? track?.length, 3061.7);

    // Live lap tracking
    this.currentLapTime = 0;
    this.currentLapDistance = 0;
    this.previousDistance = 0;
    this.lapCount = 0;
    this.lastLapTime = null;
    this.bestLapTime = null;
    this.isRecording = true;

    // Buffer of samples for the ongoing lap: [{ distance, lateral, speed, timeS }]
    this.liveSamples = [];
    this.sampleIntervalM = 4.0;
    this.lastSampleDistance = -999;

    // Active baseline profile (either user recorded or default optimal)
    this.userBaseline = null;
    this.activeProfile = null;

    this._loadSavedBaseline();
  }

  reset() {
    this.currentLapTime = 0;
    this.currentLapDistance = 0;
    this.previousDistance = 0;
    this.liveSamples = [];
    this.lastSampleDistance = -999;
  }

  /**
   * Update recorder with player vehicle telemetry every physics frame.
   * @param {Object} vehicle - Player vehicle
   * @param {number} dt - Step duration in seconds
   * @returns {Object} Live lap status { currentLapTime, lastLapTime, bestLapTime, deltaS, lapCompleted, newBest }
   */
  update(vehicle, dt) {
    if (!vehicle) return { currentLapTime: 0, deltaS: 0 };

    this.currentLapTime += dt;
    const dist = finite(vehicle.distance, 0);
    const speed = finite(vehicle.speed, 0);
    const lateral = finite(vehicle.surface?.lateral, 0);

    let lapCompleted = false;
    let newBest = false;

    // Detect start/finish line wrap-around (distance wraps from near end to near start)
    const crossedLine = this.previousDistance > (this.trackLength - 50) && dist < 50;

    if (crossedLine && this.currentLapTime > 15.0) {
      // Completed full lap!
      lapCompleted = true;
      this.lapCount += 1;
      this.lastLapTime = this.currentLapTime;

      if (!this.bestLapTime || this.currentLapTime < this.bestLapTime) {
        this.bestLapTime = this.currentLapTime;
        newBest = true;
      }

      // Finalize lap profile
      const completedProfile = this._buildProfileFromSamples(this.liveSamples, this.currentLapTime);

      // If no baseline exists yet, automatically set first valid clean lap as baseline
      if (!this.userBaseline) {
        this.setBaselineFromProfile(completedProfile);
      }

      // Reset for next lap
      this.currentLapTime = 0;
      this.liveSamples = [];
      this.lastSampleDistance = -999;
    }

    this.previousDistance = dist;

    // Record spatial sample every sampleIntervalM
    if (Math.abs(dist - this.lastSampleDistance) >= this.sampleIntervalM || this.lastSampleDistance < 0) {
      this.liveSamples.push({
        distance: dist,
        lateral,
        speed,
        timeS: this.currentLapTime
      });
      this.lastSampleDistance = dist;
    }

    // Compute live delta vs baseline
    const deltaS = this.calculateDelta(dist, this.currentLapTime);

    return {
      currentLapTime: this.currentLapTime,
      lastLapTime: this.lastLapTime,
      bestLapTime: this.bestLapTime,
      deltaS,
      lapCompleted,
      newBest,
      lapCount: this.lapCount,
      hasBaseline: Boolean(this.userBaseline)
    };
  }

  /**
   * Set the current live lap or last completed lap as the user baseline.
   */
  captureLiveBaseline() {
    if (this.liveSamples.length < 20) {
      return { success: false, reason: 'NOT_ENOUGH_DATA' };
    }

    const lapTime = this.lastLapTime ?? this.currentLapTime;
    const profile = this._buildProfileFromSamples(this.liveSamples, lapTime);
    this.setBaselineFromProfile(profile);
    return { success: true, lapTime, sampleCount: profile.samples.length };
  }

  /**
   * Assign a structured profile as the active user baseline.
   */
  setBaselineFromProfile(profile) {
    if (!profile || !profile.samples?.length) return;
    this.userBaseline = profile;
    this.activeProfile = profile;
    this._saveBaseline(profile);
  }

  /**
   * Clear the user baseline and revert to default.
   */
  clearBaseline() {
    this.userBaseline = null;
    this.activeProfile = null;
    try {
      localStorage.removeItem('gemini_gauntlet_user_baseline');
    } catch {
      // Storage quota or unavailable
    }
  }

  /**
   * Get reference racing line and target speed at a specific track distance.
   * @param {number} distance - Track distance in meters
   * @returns {Object} { lineLateral, targetSpeed, referenceTimeS }
   */
  paceAtDistance(distance) {
    const wrappedDist = ((distance % this.trackLength) + this.trackLength) % this.trackLength;

    if (this.activeProfile?.sampleMap) {
      const idx = clamp(Math.round(wrappedDist / this.activeProfile.resolutionM), 0, this.activeProfile.sampleMap.length - 1);
      const sample = this.activeProfile.sampleMap[idx];
      if (sample) {
        return {
          lineLateral: sample.lateral,
          targetSpeed: sample.speed,
          referenceTimeS: sample.timeS
        };
      }
    }

    // Default geometric line fallback
    const trackPoint = this.track?.atDistance ? this.track.atDistance(wrappedDist) : { curvature: 0 };
    const curv = finite(trackPoint.curvature, 0);
    const lineLateral = clamp(-Math.sign(curv) * Math.min(2.5, Math.abs(curv) * 600), -4.5, 4.5);
    const targetSpeed = Math.max(18, Math.sqrt(18.0 / Math.max(1e-4, Math.abs(curv))));

    return {
      lineLateral,
      targetSpeed,
      referenceTimeS: (wrappedDist / 45.0)
    };
  }

  /**
   * Calculate live delta vs baseline in seconds (+ = slower, - = faster).
   */
  calculateDelta(distance, currentLapTime) {
    if (!this.userBaseline) return 0;
    const ref = this.paceAtDistance(distance);
    if (!ref || !Number.isFinite(ref.referenceTimeS)) return 0;
    return currentLapTime - ref.referenceTimeS;
  }

  /**
   * Build a spatial lookup table from recorded discrete samples.
   * @private
   */
  _buildProfileFromSamples(samples, totalLapTime) {
    if (!samples.length) return null;

    // Sort by distance
    const sorted = [...samples].sort((a, b) => a.distance - b.distance);
    const resolutionM = 2.0;
    const totalSteps = Math.ceil(this.trackLength / resolutionM);
    const sampleMap = [];

    let srcIdx = 0;
    for (let step = 0; step < totalSteps; step++) {
      const targetDist = step * resolutionM;

      // Find surrounding samples
      while (srcIdx < sorted.length - 1 && sorted[srcIdx + 1].distance < targetDist) {
        srcIdx++;
      }

      const p0 = sorted[srcIdx];
      const p1 = sorted[Math.min(sorted.length - 1, srcIdx + 1)];

      let lateral = p0.lateral;
      let speed = p0.speed;
      let timeS = p0.timeS;

      if (p1 && p1.distance > p0.distance) {
        const t = clamp((targetDist - p0.distance) / (p1.distance - p0.distance), 0, 1);
        lateral = p0.lateral + (p1.lateral - p0.lateral) * t;
        speed = p0.speed + (p1.speed - p0.speed) * t;
        timeS = p0.timeS + (p1.timeS - p0.timeS) * t;
      }

      sampleMap.push({
        distance: targetDist,
        lateral: clamp(lateral, -6.5, 6.5),
        speed: Math.max(8.0, speed),
        timeS
      });
    }

    return {
      lapTime: totalLapTime,
      date: new Date().toISOString(),
      resolutionM,
      sampleMap,
      samples: sorted
    };
  }

  _saveBaseline(profile) {
    try {
      const summary = {
        lapTime: profile.lapTime,
        date: profile.date,
        resolutionM: profile.resolutionM,
        sampleMap: profile.sampleMap
      };
      localStorage.setItem('gemini_gauntlet_user_baseline', JSON.stringify(summary));
    } catch {
      // Storage quota or unavailable
    }
  }

  _loadSavedBaseline() {
    try {
      const raw = localStorage.getItem('gemini_gauntlet_user_baseline');
      if (raw) {
        const parsed = JSON.parse(raw);
        if (parsed?.sampleMap?.length) {
          this.userBaseline = parsed;
          this.activeProfile = parsed;
        }
      }
    } catch {
      // Ignore
    }
  }

  static formatTime(seconds) {
    if (!Number.isFinite(seconds) || seconds <= 0) return '--:--.---';
    const mins = Math.floor(seconds / 60);
    const secs = Math.floor(seconds % 60);
    const millis = Math.floor((seconds % 1) * 1000);
    return `${String(mins).padStart(2, '0')}:${String(secs).padStart(2, '0')}.${String(millis).padStart(3, '0')}`;
  }
}
