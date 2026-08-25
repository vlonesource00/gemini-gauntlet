const formatTime = (seconds) => {
  if (!Number.isFinite(seconds)) return '--:--.---';
  const minutes = Math.floor(Math.max(0, seconds) / 60);
  const remainder = Math.max(0, seconds) - minutes * 60;
  return `${String(minutes).padStart(2, '0')}:${remainder.toFixed(3).padStart(6, '0')}`;
};

const finite = (value, fallback = 0) => Number.isFinite(value) ? value : fallback;
const fixed = (value, digits = 2) => finite(value).toFixed(digits);
const percent = (value) => `${Math.round(finite(value) * 100)}%`;
const monotonicNow = () => globalThis.performance?.now?.() ?? Date.now();
const escapeHtml = (value) => String(value ?? '')
  .replaceAll('&', '&amp;').replaceAll('<', '&lt;').replaceAll('>', '&gt;').replaceAll('"', '&quot;');

const wheelCard = (name) => `<article class="telemetry-wheel" data-wheel="${name}">
  <div class="wheel-header"><strong>${name}</strong><span data-field="grip-pct">100% GRIP</span></div>
  <div class="wheel-data"><span data-field="temp">-- / -- / --°C</span><span data-field="pressure">--.-- BAR</span></div>
  <div class="wheel-slip"><span data-field="slip">κ --.-- α --.-°</span><i><em data-field="travel"></em></i></div>
</article>`;

const standingsMarkup = (rows, playerId) => rows.map((row) => {
  const gap = finite(row.distanceGapM);
  const gapLabel = Math.abs(gap) < 0.05 ? 'LEADER' : `${gap > 0 ? '+' : ''}${gap.toFixed(1)}m`;
  const selected = row.id === playerId ? ' player' : '';
  const finished = row.finished ? ' finished' : '';
  const pitBadge = row.pitStopDone
    ? '<span class="badge pit-done">PIT OK</span>'
    : '<span class="badge pit-req">PIT REQ</span>';
  return `<div class="standings-row${selected}${finished}">
    <b class="pos">${String(row.position).padStart(2, '0')}</b>
    <span class="name">${escapeHtml(row.name)}</span>
    <small class="class-badge ${row.spec || 'prototype'}">${String(row.spec ?? 'PROTOTYPE').toUpperCase().slice(0, 5)}</small>
    ${pitBadge}
    <em class="gap">${gapLabel}</em>
  </div>`;
}).join('');

export class HUD {
  constructor(onMute) {
    this.telemetryVisible = false;
    this.root = document.createElement('div');
    this.root.className = 'hud';
    this.root.innerHTML = `
      <!-- TOP SESSION BAR -->
      <header class="session-bar">
        <div class="hud-brand">
          <div class="brand-title">GEMINI<i>//</i>GAUNTLET</div>
          <small class="brand-sub">APEX-NMPCC GRAND PRIX SIM</small>
        </div>
        <div class="session-stats">
          <div class="stat-box"><small>POS</small><strong data-role="position">P01</strong></div>
          <div class="stat-box"><small>LAP</small><strong data-role="lap">1/12</strong></div>
          <div class="stat-box"><small>BEST LAP</small><strong data-role="best-lap">--:--.---</strong></div>
          <div class="stat-box"><small>RACE TIME</small><strong data-role="race-time">00:00.000</strong></div>
          <div class="stat-box pit-status-box"><small>MANDATORY PIT</small><strong data-role="mandatory-pit" class="pit-pending">REQUIRED</strong></div>
        </div>
      </header>

      <!-- COUNTDOWN & NOTICES -->
      <div class="countdown hidden" data-role="countdown">GRID</div>
      <div class="notice" data-role="notice">PRESS ENTER OR SPACE TO START GRAND PRIX</div>

      <!-- PIT BOX SERVICE OVERLAY -->
      <div class="pit-overlay hidden" data-role="pit-overlay">
        <div class="pit-banner">
          <div class="pit-title">PIT SERVICE IN PROGRESS</div>
          <div class="pit-countdown"><span data-role="pit-countdown">3.5</span>S</div>
          <div class="pit-desc">CHANGING TIRES & OPTIMIZING BRAKE DUCTS</div>
          <div class="pit-progress-bar"><i data-role="pit-progress-fill"></i></div>
        </div>
      </div>

      <!-- PIT SPEED LIMITER BANNER -->
      <div class="pit-limiter-badge hidden" data-role="pit-limiter-badge">
        <span>PIT LIMITER ACTIVE</span> <b>60 KM/H</b>
      </div>

      <!-- LIVE STANDINGS TIMING TOWER -->
      <section class="hud-panel standings-panel" data-role="standings-panel">
        <header><b>LIVE TIMING // 12 LAPS</b><span>INTERVAL</span></header>
        <div class="standings-rows-container" data-role="standings-rows"></div>
      </section>

      <!-- PRO MOTORSPORT DIGITAL COCKPIT DASHBOARD -->
      <section class="pro-cockpit-dash">
        <!-- SEQUENTIAL LED SHIFT LIGHTS -->
        <div class="shift-light-bar" data-role="shift-light-bar">
          <i class="led green" data-idx="0"></i>
          <i class="led green" data-idx="1"></i>
          <i class="led green" data-idx="2"></i>
          <i class="led yellow" data-idx="3"></i>
          <i class="led yellow" data-idx="4"></i>
          <i class="led yellow" data-idx="5"></i>
          <i class="led red" data-idx="6"></i>
          <i class="led red" data-idx="7"></i>
          <i class="led red" data-idx="8"></i>
          <i class="led blue" data-idx="9"></i>
          <i class="led blue" data-idx="10"></i>
        </div>

        <div class="dash-main-cluster">
          <!-- SPEED & DELTA -->
          <div class="speed-cluster">
            <div class="speed-number"><strong data-role="speed">000</strong><span>KM/H</span></div>
            <div class="delta-number" data-role="delta-pace"><small>Δ</small> <span>+0.000</span></div>
          </div>

          <!-- GEAR & RPM -->
          <div class="gear-cluster">
            <div class="gear-display"><strong data-role="gear">N</strong></div>
            <div class="gear-sub">
              <span data-role="gearbox-mode">AUTO</span>
              <span class="shift-hint" data-role="shift-hint"></span>
            </div>
          </div>

          <!-- PEDAL INPUTS & ERS -->
          <div class="input-cluster">
            <div class="pedal-bar throttle">
              <span>THR</span>
              <div class="meter-track"><em data-role="throttle"></em></div>
              <b data-role="throttle-value">0%</b>
            </div>
            <div class="pedal-bar brake">
              <span>BRK</span>
              <div class="meter-track"><em data-role="brake"></em></div>
              <b data-role="brake-value">0%</b>
            </div>
            <div class="ers-deploy-bar">
              <span>ERS</span>
              <div class="meter-track"><em data-role="ers-fill"></em></div>
              <b data-role="ers-soc">100%</b>
            </div>
          </div>
        </div>

        <!-- RPM PROGRESS BAR -->
        <div class="rpm-horizontal-bar">
          <div class="rpm-fill" data-role="rpm-fill"></div>
          <div class="rpm-labels">
            <span>0</span><span>2K</span><span>4K</span><span>6K</span><span>8K</span>
            <b data-role="rpm-label">1,050 RPM</b>
          </div>
        </div>
      </section>

      <!-- MULTI-AGENT AI DEBUGGER PANEL (F3 INSPECT / F4 FIELD) -->
      <section class="hud-panel ai-panel" data-role="ai-panel">
        <header>
          <b>AI RADAR // [F3] INSPECT · [F4] FIELD · [N] CYCLE</b>
          <span data-role="ai-field-toggle">MODE: SINGLE</span>
        </header>

        <div class="ai-field-roster" data-role="ai-field-roster">
          <div class="ai-field-legend">
            <span>GRID ROSTER</span>
            <span>
              <i class="ai-legend-dot race"></i>PACE 
              <i class="ai-legend-dot pass"></i>ATTACK 
              <i class="ai-legend-dot defend"></i>DEFEND 
              <i class="ai-legend-dot pit"></i>PIT 
              <i class="ai-legend-dot recover"></i>RECOVER
            </span>
          </div>
          <div class="ai-roster-list" data-role="ai-roster-list"></div>
        </div>

        <div class="ai-single-inspect" data-role="ai-single-inspect">
          <div class="ai-driver-header">
            <div><strong data-role="ai-name">GAUNTLET-AI</strong><span data-role="ai-class">PROTOTYPE</span></div>
            <div class="ai-mode-badge" data-role="ai-mode-badge"><b data-role="ai-mode">PACE</b></div>
          </div>
          <div class="ai-intent-reason"><span data-role="ai-reason">OPTIMAL DP SPEED ENVELOPE</span></div>

          <div class="ai-metrics-grid">
            <div><small>SPEED</small><b data-role="ai-speed">0.0 KM/H</b></div>
            <div><small>TARGET SPEED</small><b data-role="ai-target-speed">0.0 KM/H</b></div>
            <div><small>LANE OFFSET</small><b data-role="ai-offset">0.00 M</b></div>
            <div><small>OPTIMAL LINE</small><b data-role="ai-line">0.00 M</b></div>
            <div><small>FRONT SLIP (αF)</small><b data-role="ai-alpha-f">0.00°</b></div>
            <div><small>REAR SLIP (αR)</small><b data-role="ai-alpha-r">0.00°</b></div>
            <div><small>UNDERSTEER INT</small><b data-role="ai-yaw-int">0.000</b></div>
            <div><small>THREAT / TARGET</small><b data-role="ai-threat">CLEAR</b></div>
          </div>

          <div class="ai-thought-box">
            <div><small>TACTICAL MANEUVER</small><b data-role="ai-maneuver">—</b></div>
            <div><small>CLOSING SPEED FLOOR</small><b data-role="ai-closing-floor">—</b></div>
            <div><small>CORRIDOR [dMin, dMax]</small><b data-role="ai-corridor">[-6.5m, +6.5m]</b></div>
            <div><small>COLLISION VERDICT</small><b data-role="ai-safety">FREE (CLEAR)</b></div>
          </div>
        </div>
      </section>

      <!-- MOTEC TELEMETRY PANEL (T KEY TOGGLE) -->
      <section class="hud-panel motec-panel" data-role="motec">
        <header><b>MoTeC M1 // 4-CORNER TELEMETRY</b><span data-role="gg">+0.00 LAT · +0.00 LONG</span></header>
        <div class="motec-layout">
          <div class="gg-container">
            <div class="gg-circle">
              <div class="gg-ring-inner"></div>
              <div class="gg-crosshair-h"></div>
              <div class="gg-crosshair-v"></div>
              <i data-role="gg-dot"></i>
            </div>
            <div class="gg-readout"><b data-role="peak-g">PEAK 0.00G</b></div>
          </div>
          <div class="wheel-grid">${['FL', 'FR', 'RL', 'RR'].map(wheelCard).join('')}</div>
        </div>
      </section>

      <!-- CONTROLS & SHORTCUTS HELP -->
      <footer class="hud-footer-strip">
        <div class="key-hints">
          <b>WASD/ARROWS</b> DRIVE · <b>SPACE</b> HANDBRAKE · <b>C</b> CAMERA · <b>T</b> MoTeC · <b>P</b> PIT REQUEST · <b>F3</b> AI INSPECT · <b>F4</b> AI FIELD · <b>N</b> NEXT CAR · <b>R</b> RESET
        </div>
        <button class="mute-btn" data-role="mute" type="button">AUDIO: ON</button>
      </footer>

      <!-- RACE FINISH BANNER -->
      <div class="finish-overlay hidden" data-role="finish">
        <div class="finish-modal">
          <h1>GRAND PRIX FINISH</h1>
          <div class="finish-results">
            <div><small>FINAL POSITION</small><strong data-role="finish-position">P01</strong></div>
            <div><small>TOTAL RACE TIME</small><strong data-role="finish-time">00:00.000</strong></div>
            <div><small>BEST LAP</small><strong data-role="finish-best-lap">00:00.000</strong></div>
          </div>
          <p>PRESS <b>ENTER</b> TO RESTART GRAND PRIX</p>
        </div>
      </div>
    `;

    document.body.append(this.root);
    this.$ = (role) => this.root.querySelector(`[data-role="${role}"]`);
    this.wheelNodes = new Map([...this.root.querySelectorAll('[data-wheel]')].map((node) => [node.dataset.wheel, node]));
    this.shiftLeds = [...this.root.querySelectorAll('.shift-light-bar .led')];
    this._standingsSignature = null;
    this._standingsLastRenderAt = Number.NEGATIVE_INFINITY;
    this._aiFieldSignature = null;
    this.$('mute')?.addEventListener('click', () => this.setMuted(onMute?.() ?? false));
  }

  setTelemetryVisible(visible) {
    this.telemetryVisible = Boolean(visible);
    this.$('motec')?.classList.toggle('show', this.telemetryVisible);
    return this.telemetryVisible;
  }

  toggleTelemetry() {
    return this.setTelemetryVisible(!this.telemetryVisible);
  }

  setMuted(muted) {
    const btn = this.$('mute');
    if (btn) btn.textContent = muted ? 'AUDIO: OFF' : 'AUDIO: ON';
  }

  setNotice(message, duration = 3.5) {
    const notice = this.$('notice');
    if (!notice) return;
    notice.textContent = message;
    notice.classList.remove('hidden');
    clearTimeout(this._noticeTimer);
    if (duration > 0) {
      this._noticeTimer = setTimeout(() => notice.classList.add('hidden'), duration * 1000);
    }
  }

  setCountdown(text) {
    const el = this.$('countdown');
    if (!el) return;
    if (!text) {
      el.classList.add('hidden');
    } else {
      el.textContent = text;
      el.classList.remove('hidden');
    }
  }

  update({
    playerVehicle,
    vehicles = [],
    race = null,
    aiDebugSnapshot = null,
    fps = 60
  } = {}) {
    if (playerVehicle) {
      this._updateCockpit(playerVehicle);
      if (this.telemetryVisible) this._updateTelemetry(playerVehicle);
    }

    if (race) {
      this._updateRaceStatus(race, playerVehicle);
    }

    if (Array.isArray(vehicles) && vehicles.length) {
      this._updateStandings(vehicles, playerVehicle?.id);
    }

    if (aiDebugSnapshot) {
      this._updateAIDebug(aiDebugSnapshot, vehicles);
    }
  }

  _updateCockpit(v) {
    const speedKph = Math.max(0, finite(v.speed) * 3.6);
    this.$('speed').textContent = Math.round(speedKph).toString().padStart(3, '0');

    // Gear
    let gearText = 'N';
    if (v.gear === 0) gearText = 'N';
    else if (v.gear < 0) gearText = 'R';
    else gearText = String(v.gear);
    this.$('gear').textContent = gearText;

    // RPM & Shift lights
    const rpm = finite(v.rpm, 1000);
    const maxRpm = finite(v.spec?.limiterRpm, 8000);
    const rpmPct = Math.min(1.0, Math.max(0, (rpm - 1000) / (maxRpm - 1000)));
    this.$('rpm-label').textContent = `${Math.round(rpm).toLocaleString()} RPM`;
    this.$('rpm-fill').style.width = `${(rpmPct * 100).toFixed(1)}%`;

    // Shift LED Lights
    const ledCount = this.shiftLeds.length;
    const litLeds = Math.floor(rpmPct * ledCount);
    const isRedline = rpmPct > 0.94;
    this.shiftLeds.forEach((led, idx) => {
      led.classList.toggle('active', idx < litLeds);
      led.classList.toggle('flash', isRedline);
    });

    // Pedals
    const thr = finite(v.controls?.throttle, 0);
    const brk = finite(v.controls?.brake, 0);
    this.$('throttle').style.height = `${(thr * 100).toFixed(0)}%`;
    this.$('throttle-value').textContent = `${Math.round(thr * 100)}%`;
    this.$('brake').style.height = `${(brk * 100).toFixed(0)}%`;
    this.$('brake-value').textContent = `${Math.round(brk * 100)}%`;

    // ERS
    if (v.ers) {
      const soc = finite(v.ers.soc, 1.0);
      this.$('ers-fill').style.height = `${(soc * 100).toFixed(0)}%`;
      this.$('ers-soc').textContent = `${Math.round(soc * 100)}%`;
    }

    // Pit Limiter
    const isPitLimited = Boolean(v.pitLimiterActive || v.inPitLane);
    this.$('pit-limiter-badge')?.classList.toggle('hidden', !isPitLimited);
  }

  _updateRaceStatus(race, player) {
    const raceTime = finite(race.raceTime || race.elapsed, 0);
    this.$('race-time').textContent = formatTime(raceTime);

    const currentLap = Math.max(1, finite(player?.completedLaps ?? race.currentLap ?? 0) + 1);
    const totalLaps = finite(race.totalLaps, 12);
    this.$('lap').textContent = `${currentLap}/${totalLaps}`;

    if (player?.bestLapTime) {
      this.$('best-lap').textContent = formatTime(player.bestLapTime);
    }

    // Mandatory Pit Stop Status
    const mandatoryDone = Boolean(player?.pitStopDone);
    const pitEl = this.$('mandatory-pit');
    if (pitEl) {
      pitEl.textContent = mandatoryDone ? 'COMPLETED' : 'REQUIRED';
      pitEl.className = mandatoryDone ? 'pit-completed' : 'pit-pending';
    }

    // In Pit Service Countdown
    const inBox = Boolean(player?.inPitBox);
    const pitOverlay = this.$('pit-overlay');
    if (pitOverlay) {
      pitOverlay.classList.toggle('hidden', !inBox);
      if (inBox && player?.pitServiceRemaining != null) {
        const rem = Math.max(0, player.pitServiceRemaining);
        this.$('pit-countdown').textContent = rem.toFixed(1);
        const progressPct = 1.0 - (rem / (player.pitServiceTotal || 3.5));
        this.$('pit-progress-fill').style.width = `${(progressPct * 100).toFixed(0)}%`;
      }
    }

    // Finish Modal
    if (race.phase === 'finished' || player?.finished) {
      const finish = this.$('finish');
      if (finish) {
        finish.classList.remove('hidden');
        this.$('finish-position').textContent = `P${String(player?.position || 1).padStart(2, '0')}`;
        this.$('finish-time').textContent = formatTime(player?.totalRaceTime || raceTime);
        this.$('finish-best-lap').textContent = formatTime(player?.bestLapTime || 0);
      }
    }
  }

  _updateStandings(vehicles, playerId) {
    const list = [...vehicles].sort((a, b) => {
      if (b.completedLaps !== a.completedLaps) return b.completedLaps - a.completedLaps;
      return b.distance - a.distance;
    });

    const rows = list.map((v, idx) => {
      v.position = idx + 1;
      const leaderDist = list[0].distance + (list[0].completedLaps || 0) * (v.track?.length || 2704);
      const myDist = v.distance + (v.completedLaps || 0) * (v.track?.length || 2704);
      const gap = idx === 0 ? 0 : -(leaderDist - myDist);
      return {
        position: idx + 1,
        id: v.id,
        name: v.name || `Car ${idx + 1}`,
        spec: v.spec?.key || v.classKey || 'prototype',
        distanceGapM: gap,
        pitStopDone: Boolean(v.pitStopDone),
        finished: Boolean(v.finished)
      };
    });

    const now = monotonicNow();
    if (now - this._standingsLastRenderAt < 100) return;
    this._standingsLastRenderAt = now;

    this.$('standings-rows').innerHTML = standingsMarkup(rows, playerId);
    const pRow = rows.find((r) => r.id === playerId);
    if (pRow) this.$('position').textContent = `P${String(pRow.position).padStart(2, '0')}`;
  }

  _updateTelemetry(v) {
    const telemetry = v.telemetry ?? {};
    const lat = finite(telemetry.lateralG);
    const long = finite(telemetry.longitudinalG);
    this.$('gg').textContent = `${lat >= 0 ? '+' : ''}${lat.toFixed(2)} LAT · ${long >= 0 ? '+' : ''}${long.toFixed(2)} LONG`;
    this.$('gg-dot').style.transform = `translate(${Math.max(-36, Math.min(36, lat * 16))}px, ${Math.max(-36, Math.min(36, -long * 16))}px)`;

    for (const wheel of v.wheels ?? []) {
      const node = this.wheelNodes.get(wheel.name);
      if (!node) continue;
      const tIn = Math.round(finite(wheel.temperatureInnerC, 85));
      const tMid = Math.round(finite(wheel.temperatureMiddleC, 88));
      const tOut = Math.round(finite(wheel.temperatureOuterC, 85));
      node.querySelector('[data-field="temp"]').textContent = `${tIn}/${tMid}/${tOut}°C`;
      node.querySelector('[data-field="pressure"]').textContent = `${(finite(wheel.pressurePa, 185000) / 100000).toFixed(2)} BAR`;
      node.querySelector('[data-field="slip"]').textContent = `κ ${fixed(wheel.slipRatio, 2)} α ${(finite(wheel.slipAngle) * 57.3).toFixed(1)}°`;
      const gripRemaining = Math.max(0, 1.0 - finite(wheel.wear, 0));
      const gripSpan = node.querySelector('[data-field="grip-pct"]');
      if (gripSpan) {
        gripSpan.textContent = `${Math.round(gripRemaining * 100)}% GRIP`;
        gripSpan.className = gripRemaining > 0.8 ? 'grip-good' : (gripRemaining > 0.6 ? 'grip-warn' : 'grip-bad');
      }
    }
  }

  _updateAIDebug(snapshot, vehicles = []) {
    const enabled = Boolean(snapshot?.enabled);
    const fieldView = Boolean(snapshot?.fieldView);
    const aiPanel = this.$('ai-panel');
    if (!aiPanel) return;

    aiPanel.classList.toggle('show', enabled);
    aiPanel.classList.toggle('field-view', fieldView);
    if (!enabled) return;

    this.$('ai-field-toggle').textContent = fieldView ? 'MODE: FULL GRID' : 'MODE: SINGLE INSPECT';

    // Roster view
    if (fieldView && Array.isArray(vehicles)) {
      const rosterList = this.$('ai-roster-list');
      if (rosterList) {
        rosterList.innerHTML = vehicles.map((v, idx) => {
          const mode = v.aiTactical?.racecraftPhase || v.aiState?.mode || 'PACE';
          const dotClass = mode.includes('ATTACK') || mode.includes('PASS') || mode.includes('DIVE') ? 'pass' : (mode.includes('DEFEND') ? 'defend' : (mode.includes('PIT') ? 'pit' : (mode.includes('RECOVER') ? 'recover' : 'race')));
          return `<div class="ai-roster-item">
            <span class="dot ${dotClass}"></span>
            <b>P${String(v.position || idx + 1).padStart(2, '0')}</b>
            <span class="roster-name">${escapeHtml(v.name)}</span>
            <small>${mode}</small>
            <em>${(finite(v.speed) * 3.6).toFixed(0)}kph</em>
          </div>`;
        }).join('');
      }
    }

    // Single Inspect
    const state = snapshot.state || {};
    this.$('ai-name').textContent = snapshot.selectedName ?? state.name ?? 'GAUNTLET-AI';
    this.$('ai-class').textContent = String(state.classKey ?? state.class ?? 'PROTOTYPE').toUpperCase();
    this.$('ai-mode').textContent = state.tacticalMode || state.mode || 'PACE';
    this.$('ai-reason').textContent = state.tacticalReason || state.reason || 'OPTIMAL 3D DP SPEED PROFILE';

    this.$('ai-speed').textContent = `${(finite(state.currentSpeed || state.speed) * 3.6).toFixed(1)} KM/H`;
    this.$('ai-target-speed').textContent = `${(finite(state.desiredSpeed) * 3.6).toFixed(1)} KM/H`;
    this.$('ai-offset').textContent = `${fixed(state.targetOffset || state.plannedTargetOffset)} M`;
    this.$('ai-line').textContent = `${fixed(state.lineOffset || state.optimalLateral)} M`;

    this.$('ai-alpha-f').textContent = `${(finite(state.alphaF) * 57.3).toFixed(1)}°`;
    this.$('ai-alpha-r').textContent = `${(finite(state.alphaR) * 57.3).toFixed(1)}°`;
    this.$('ai-yaw-int').textContent = `${fixed(state.yawInt, 3)}`;
    this.$('ai-threat').textContent = state.targetId ? `LOCKED [${state.targetId}]` : 'CLEAR';

    this.$('ai-maneuver').textContent = state.racecraftPhase || state.tactical?.notes || 'RACING LINE';
    this.$('ai-closing-floor').textContent = state.straight ? '+18.0 M/S DRAFT FLOOR' : 'CORNER COMMIT';
  }
}

export default HUD;
