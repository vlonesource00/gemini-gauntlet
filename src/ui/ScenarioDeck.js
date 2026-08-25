export const SCENARIO_CATALOG = Object.freeze({
  attack: [
    { id: 'A1', name: 'T1 Dive Attack', tag: 'ATTACK SCENARIO A1', objective: 'Execute a clean inside dive bomb before Turn 1 apex without contact.', carClass: 'prototype', defaultSpeed: 1.0 },
    { id: 'A2', name: 'Switchback Exit', tag: 'ATTACK SCENARIO A2', objective: 'Bait outside entry and cut inside for maximum exit traction.', carClass: 'prototype', defaultSpeed: 1.0 },
    { id: 'A3', name: 'Late Braking Send', tag: 'ATTACK SCENARIO A3', objective: 'Outbrake opponent deep into the braking zone with optimal threshold control.', carClass: 'gt', defaultSpeed: 1.0 },
    { id: 'A4', name: 'Chicane Squeeze', tag: 'ATTACK SCENARIO A4', objective: 'Navigate tight chicane side-by-side leaving minimal fair racing room.', carClass: 'prototype', defaultSpeed: 1.0 },
    { id: 'A5', name: 'Slipstream Pass', tag: 'ATTACK SCENARIO A5', objective: 'Harness high-speed aerodynamic wake to pass before the brake point.', carClass: 'prototype', defaultSpeed: 1.0 }
  ],
  defense: [
    { id: 'D1', name: 'Apex Pinch Defense', tag: 'DEFENSE SCENARIO D1', objective: 'Protect the inside line and pinch the attacker away from optimal apex.', carClass: 'prototype', defaultSpeed: 1.0 },
    { id: 'D2', name: 'Inside Line Cover', tag: 'DEFENSE SCENARIO D2', objective: 'Break the draft and hold the defensive corridor into high-speed braking.', carClass: 'gt', defaultSpeed: 1.0 },
    { id: 'D3', name: 'Cross-Over Retaliation', tag: 'DEFENSE SCENARIO D3', objective: 'Counter-attack on exit after opponent dives deep on entry.', carClass: 'prototype', defaultSpeed: 1.0 },
    { id: 'D4', name: 'Low-Grip Squeeze', tag: 'DEFENSE SCENARIO D4', objective: 'Defend on reduced friction surface balancing traction and corridor space.', carClass: 'touring', defaultSpeed: 1.0 }
  ],
  race: [
    { id: 'RACE_4', name: '4-Car Prototype Grand Prix', tag: '4-CAR GRID', objective: 'High-speed 4-car prototype championship race with pack drafting and late-braking divebombs.', carClass: 'prototype', defaultSpeed: 1.0, gridCount: 4 },
    { id: 'RACE_4_GT', name: '4-Car GT Battle', tag: '4-CAR GRID', objective: '4-car wheel-to-wheel GT battle testing slipstream slingshots, trail braking, and cutbacks.', carClass: 'gt', defaultSpeed: 1.0, gridCount: 4 },
    { id: 'RACE_4_MIXED', name: '4-Car Multi-Class Sprint', tag: '4-CAR GRID', objective: 'Mixed Prototype and GT 4-car sprint with dynamic traffic and multi-flank overtakes.', carClass: 'prototype', defaultSpeed: 1.0, gridCount: 4 }
  ],
  hotlap: [
    { id: 'H1', name: 'Ghost Baseline Lap', tag: 'HOTLAP SCENARIO H1', objective: 'Beat the reference human/AI telemetry delta on a clean hot lap.', carClass: 'prototype', defaultSpeed: 1.0 }
  ],
  duel: [
    { id: 'FREE', name: 'Free Combat Duel', tag: 'OPEN DUEL', objective: 'Continuous high-intensity multi-lap autonomous racecraft combat.', carClass: 'prototype', defaultSpeed: 1.0 }
  ]
});

export class ScenarioDeck {
  constructor({
    onScenarioSelect = () => {},
    onReset = () => {},
    onSpeedChange = () => {},
    onAIHeuristicsChange = () => {},
    onDebugToggle = () => {},
    onCameraChange = () => {},
    onSetBaseline = () => {},
    onClearBaseline = () => {},
    onExportJSON = () => {},
    onAutopilotToggle = () => {}
  } = {}) {
    this.container = document.querySelector('#scenario-deck');
    this.hud = document.querySelector('#hud');
    this.onScenarioSelect = onScenarioSelect;
    this.onReset = onReset;
    this.onSpeedChange = onSpeedChange;
    this.onAIHeuristicsChange = onAIHeuristicsChange;
    this.onDebugToggle = onDebugToggle;
    this.onCameraChange = onCameraChange;
    this.onSetBaseline = onSetBaseline;
    this.onClearBaseline = onClearBaseline;
    this.onExportJSON = onExportJSON;
    this.onAutopilotToggle = onAutopilotToggle;

    this.activeCategory = 'attack';
    this.activeScenarioId = 'A1';
    this.simSpeed = 1.0;
    this.isPaused = false;
    this.autopilotActive = true;
    this.cameraMode = 'CHASE';

    this.heuristics = {
      aggression: 95,
      diveMargin: 85,
      defenseReactivity: 95,
      kerbUsage: 95,
      lookaheadHorizon: 24
    };

    this.debugLayers = {
      splines: true,
      predictionCones: true,
      corridors: true,
      thoughtHUD: true,
      ggCircle: false
    };

    this._render();
    this._bindEvents();
  }

  _render() {
    if (!this.container) return;

    this.container.innerHTML = `
      <header class="deck-header">
        <div class="deck-title">
          <b>CONTROL DECK</b>
          <small>SCENARIOS & AI HEURISTICS</small>
        </div>
        <button class="deck-toggle-btn" id="deck-toggle-collapse" type="button" title="Toggle Panel">◧</button>
      </header>

      <div class="deck-content">
        <!-- Scenario Categories -->
        <div class="deck-section">
          <div class="deck-section-title">
            <span>CATEGORY</span>
            <b id="deck-cat-label">ATTACK</b>
          </div>
          <div class="category-tabs" role="tablist">
            <button class="category-tab active" data-category="attack" type="button">ATTACK</button>
            <button class="category-tab" data-category="defense" type="button">DEFENSE</button>
            <button class="category-tab" data-category="race" type="button">4-CAR RACE</button>
            <button class="category-tab" data-category="hotlap" type="button">HOTLAP</button>
            <button class="category-tab" data-category="duel" type="button">DUEL</button>
          </div>

          <div class="scenario-list" id="deck-scenario-list">
            ${this._renderScenarioCards('attack')}
          </div>
        </div>

        <!-- Simulation Controls -->
        <div class="deck-section">
          <div class="deck-section-title">
            <span>SIMULATION & DRIVER CONTROL</span>
            <b id="deck-speed-label">1.0x</b>
          </div>
          <div class="action-grid">
            <button class="deck-btn" data-speed="0.2" type="button">0.2x</button>
            <button class="deck-btn" data-speed="0.5" type="button">0.5x</button>
            <button class="deck-btn active" data-speed="1.0" type="button">1.0x</button>
            <button class="deck-btn" data-speed="2.0" type="button">2.0x</button>
            <button class="deck-btn" data-action="pause" type="button">PAUSE</button>
            <button class="deck-btn deck-btn-reset" data-action="reset" type="button">RESET [R]</button>
          </div>
          <div style="margin-top: 6px;">
            <button class="deck-btn deck-btn-autopilot active" id="btn-toggle-autopilot" type="button" style="width: 100%; border-color: #00f0ff; color: #00f0ff; font-weight: 800; padding: 8px;">
              AUTOPILOT / SPECTATE: ON [P]
            </button>
          </div>
        </div>

        <!-- Camera Selector -->
        <div class="deck-section">
          <div class="deck-section-title">
            <span>TACTICAL CAMERAS</span>
            <b id="deck-cam-label">CHASE</b>
          </div>
          <div class="action-grid">
            <button class="deck-btn active" data-cam="CHASE" type="button">CHASE</button>
            <button class="deck-btn" data-cam="PURSUIT" type="button">PURSUIT</button>
            <button class="deck-btn" data-cam="TACTICAL" type="button">TACTICAL</button>
            <button class="deck-btn" data-cam="COCKPIT" type="button">COCKPIT</button>
            <button class="deck-btn" data-cam="FREE" type="button">NOCLIP [V]</button>
          </div>
        </div>

        <!-- Real-Time AI Heuristics Tuning Sliders -->
        <div class="deck-section">
          <div class="deck-section-title">
            <span>AI HEURISTICS TUNING</span>
            <b>REAL-TIME</b>
          </div>
          <div class="slider-group">
            <div class="slider-row">
              <div class="slider-labels">
                <span>AGGRESSION</span>
                <b id="val-aggression">${this.heuristics.aggression}%</b>
              </div>
              <input class="slider-input" id="slider-aggression" type="range" min="0" max="100" value="${this.heuristics.aggression}" />
            </div>

            <div class="slider-row">
              <div class="slider-labels">
                <span>DIVE MARGIN</span>
                <b id="val-dive-margin">${this.heuristics.diveMargin}%</b>
              </div>
              <input class="slider-input" id="slider-dive-margin" type="range" min="0" max="100" value="${this.heuristics.diveMargin}" />
            </div>

            <div class="slider-row">
              <div class="slider-labels">
                <span>DEFENSE REACTIVITY</span>
                <b id="val-defense">${this.heuristics.defenseReactivity}%</b>
              </div>
              <input class="slider-input" id="slider-defense" type="range" min="0" max="100" value="${this.heuristics.defenseReactivity}" />
            </div>

            <div class="slider-row">
              <div class="slider-labels">
                <span>KERB USAGE</span>
                <b id="val-kerb">${this.heuristics.kerbUsage}%</b>
              </div>
              <input class="slider-input" id="slider-kerb" type="range" min="0" max="100" value="${this.heuristics.kerbUsage}" />
            </div>

            <div class="slider-row">
              <div class="slider-labels">
                <span>LOOKAHEAD HORIZON</span>
                <b id="val-lookahead">${this.heuristics.lookaheadHorizon}m</b>
              </div>
              <input class="slider-input" id="slider-lookahead" type="range" min="10" max="30" value="${this.heuristics.lookaheadHorizon}" />
            </div>
          </div>
        </div>

        <!-- User Reference Baseline Lap -->
        <div class="deck-section">
          <div class="deck-section-title">
            <span>USER REFERENCE BASELINE</span>
            <b id="deck-baseline-status" style="color: #ffd600;">--:--.---</b>
          </div>
          <div class="deck-baseline-panel">
            <div class="baseline-stats-row">
              <div><small>CURRENT LAP</small><strong id="deck-lap-current">00:00.000</strong></div>
              <div><small>LIVE DELTA</small><strong id="deck-lap-delta" style="color: #00e5ff;">+0.000s</strong></div>
              <div><small>LAST LAP</small><strong id="deck-lap-last">--:--.---</strong></div>
              <div><small>BEST LAP</small><strong id="deck-lap-best">--:--.---</strong></div>
            </div>
            <div class="action-grid" style="margin-top: 8px;">
              <button class="deck-btn" id="btn-set-baseline" type="button" style="border-color: #ffd600; color: #ffd600;">SET LIVE LAP AS BASELINE</button>
              <button class="deck-btn" id="btn-clear-baseline" type="button">CLEAR BASELINE</button>
            </div>
            <div style="margin-top: 6px;">
              <button class="deck-btn" id="btn-export-json" type="button" style="width: 100%; border-color: #00ff88; color: #00ff88; font-weight: 700; padding: 6px;">
                EXPORT TELEMETRY JSON [J]
              </button>
            </div>
          </div>
        </div>

        <!-- Debug Visual Layer Toggles -->
        <div class="deck-section">
          <div class="deck-section-title">
            <span>DEBUG VISUAL SUITE</span>
            <b>LAYERS</b>
          </div>
          <div class="toggle-grid">
            <label class="toggle-label">
              <input type="checkbox" id="chk-splines" ${this.debugLayers.splines ? 'checked' : ''} />
              <span>CANDIDATE SPLINES</span>
            </label>
            <label class="toggle-label">
              <input type="checkbox" id="chk-prediction" ${this.debugLayers.predictionCones ? 'checked' : ''} />
              <span>PREDICTION CONES</span>
            </label>
            <label class="toggle-label">
              <input type="checkbox" id="chk-corridors" ${this.debugLayers.corridors ? 'checked' : ''} />
              <span>CORRIDORS</span>
            </label>
            <label class="toggle-label">
              <input type="checkbox" id="chk-thought" ${this.debugLayers.thoughtHUD ? 'checked' : ''} />
              <span>THOUGHT HUD</span>
            </label>
            <label class="toggle-label" style="grid-column: span 2;">
              <input type="checkbox" id="chk-gg" ${this.debugLayers.ggCircle ? 'checked' : ''} />
              <span>MOTEC G-G / WHEEL LOADS</span>
            </label>
          </div>
        </div>
      </div>
    `;
  }

  _renderScenarioCards(category) {
    const list = SCENARIO_CATALOG[category] || [];
    return list
      .map(
        (sc) => `
      <div class="scenario-card ${sc.id === this.activeScenarioId ? 'active' : ''}" data-scenario-id="${sc.id}">
        <div class="scenario-card-info">
          <span class="scenario-card-id">${sc.id}</span>
          <span class="scenario-card-name">${sc.name}</span>
          <span class="scenario-card-tag">${sc.carClass.toUpperCase()}</span>
        </div>
      </div>
    `
      )
      .join('');
  }

  _bindEvents() {
    if (!this.container) return;

    // Toggle drawer collapse
    const toggleBtn = this.container.querySelector('#deck-toggle-collapse');
    toggleBtn?.addEventListener('click', () => {
      this.container.classList.toggle('collapsed');
    });

    // Category Tabs
    const tabs = this.container.querySelectorAll('[data-category]');
    tabs.forEach((tab) => {
      tab.addEventListener('click', () => {
        const cat = tab.dataset.category;
        this.activeCategory = cat;
        tabs.forEach((t) => t.classList.toggle('active', t === tab));
        const listEl = this.container.querySelector('#deck-scenario-list');
        if (listEl) listEl.innerHTML = this._renderScenarioCards(cat);
        const catLabel = this.container.querySelector('#deck-cat-label');
        if (catLabel) catLabel.textContent = cat.toUpperCase();
        this._bindScenarioCardEvents();
        const firstScenario = SCENARIO_CATALOG[cat]?.[0];
        if (firstScenario) {
          this.selectScenario(firstScenario.id);
        }
      });
    });

    this._bindScenarioCardEvents();

    // Speed Controls
    const speedButtons = this.container.querySelectorAll('[data-speed]');
    speedButtons.forEach((btn) => {
      btn.addEventListener('click', () => {
        const speed = parseFloat(btn.dataset.speed);
        this.simSpeed = speed;
        this.isPaused = false;
        speedButtons.forEach((b) => b.classList.toggle('active', b === btn));
        const pauseBtn = this.container.querySelector('[data-action="pause"]');
        if (pauseBtn) {
          pauseBtn.textContent = 'PAUSE';
          pauseBtn.classList.remove('active');
        }
        const speedLabel = this.container.querySelector('#deck-speed-label');
        if (speedLabel) speedLabel.textContent = `${speed.toFixed(1)}x`;
        this.onSpeedChange(speed);
      });
    });

    // Pause / Resume
    const pauseBtn = this.container.querySelector('[data-action="pause"]');
    pauseBtn?.addEventListener('click', () => {
      this.togglePause();
    });

    // Instant Reset
    const resetBtn = this.container.querySelector('[data-action="reset"]');
    resetBtn?.addEventListener('click', () => {
      this.onReset();
    });

    // Autopilot Toggle Button
    const autopilotBtn = this.container.querySelector('#btn-toggle-autopilot');
    autopilotBtn?.addEventListener('click', () => {
      this.toggleAutopilot();
    });

    // Camera Selector
    const camButtons = this.container.querySelectorAll('[data-cam]');
    camButtons.forEach((btn) => {
      btn.addEventListener('click', () => {
        const mode = btn.dataset.cam;
        this.setCameraMode(mode);
      });
    });

    // AI Heuristics Sliders
    const bindSlider = (id, key, suffix = '%') => {
      const slider = this.container.querySelector(`#slider-${id}`);
      const valDisplay = this.container.querySelector(`#val-${id}`);
      slider?.addEventListener('input', (e) => {
        const val = parseFloat(e.target.value);
        this.heuristics[key] = val;
        if (valDisplay) valDisplay.textContent = `${val}${suffix}`;
        this.onAIHeuristicsChange({ ...this.heuristics });
      });
    };

    bindSlider('aggression', 'aggression', '%');
    bindSlider('dive-margin', 'diveMargin', '%');
    bindSlider('defense', 'defenseReactivity', '%');
    bindSlider('kerb', 'kerbUsage', '%');
    bindSlider('lookahead', 'lookaheadHorizon', 'm');

    // Debug Visual Layer Checkboxes
    const bindCheckbox = (id, key) => {
      const chk = this.container.querySelector(`#chk-${id}`);
      chk?.addEventListener('change', (e) => {
        this.debugLayers[key] = e.target.checked;
        if (key === 'thoughtHUD') {
          const aiPanel = document.querySelector('#ai-thought-panel');
          if (aiPanel) aiPanel.style.display = e.target.checked ? 'block' : 'none';
        }
        if (key === 'ggCircle') {
          const motec = document.querySelector('#motec-panel');
          if (motec) motec.classList.toggle('show', e.target.checked);
        }
        this.onDebugToggle(key, e.target.checked);
      });
    };

    bindCheckbox('splines', 'splines');
    bindCheckbox('prediction', 'predictionCones');
    bindCheckbox('corridors', 'corridors');
    bindCheckbox('thought', 'thoughtHUD');
    bindCheckbox('gg', 'ggCircle');

    // Baseline Lap Controls
    const setBaselineBtn = this.container.querySelector('#btn-set-baseline');
    setBaselineBtn?.addEventListener('click', () => {
      this.onSetBaseline();
    });

    const clearBaselineBtn = this.container.querySelector('#btn-clear-baseline');
    clearBaselineBtn?.addEventListener('click', () => {
      this.onClearBaseline();
    });

    const exportJsonBtn = this.container.querySelector('#btn-export-json');
    exportJsonBtn?.addEventListener('click', () => {
      this.onExportJSON();
    });
  }

  _bindScenarioCardEvents() {
    const cards = this.container.querySelectorAll('.scenario-card');
    cards.forEach((card) => {
      card.addEventListener('click', () => {
        const scenarioId = card.dataset.scenarioId;
        this.selectScenario(scenarioId);
      });
    });
  }

  selectScenario(scenarioId) {
    this.activeScenarioId = scenarioId;
    const cards = this.container.querySelectorAll('.scenario-card');
    cards.forEach((c) => c.classList.toggle('active', c.dataset.scenarioId === scenarioId));

    let foundScenario = null;
    for (const cat of Object.values(SCENARIO_CATALOG)) {
      const match = cat.find((s) => s.id === scenarioId);
      if (match) {
        foundScenario = match;
        break;
      }
    }

    if (foundScenario) {
      this.updateBanner({
        tag: foundScenario.tag,
        title: foundScenario.name.toUpperCase(),
        objective: foundScenario.objective,
        status: 'TRIAL ACTIVE'
      });
    }

    this.onScenarioSelect(scenarioId);
  }

  togglePause() {
    this.isPaused = !this.isPaused;
    const pauseBtn = this.container.querySelector('[data-action="pause"]');
    if (pauseBtn) {
      pauseBtn.textContent = this.isPaused ? 'RESUME' : 'PAUSE';
      pauseBtn.classList.toggle('active', this.isPaused);
    }
    const speedLabel = this.container.querySelector('#deck-speed-label');
    if (speedLabel) {
      speedLabel.textContent = this.isPaused ? 'PAUSED' : `${this.simSpeed.toFixed(1)}x`;
    }
    this.onSpeedChange(this.isPaused ? 0 : this.simSpeed);
    return this.isPaused;
  }

  toggleAutopilot() {
    this.autopilotActive = !this.autopilotActive;
    this.setAutopilotActive(this.autopilotActive);
    this.onAutopilotToggle(this.autopilotActive);
    return this.autopilotActive;
  }

  setAutopilotActive(active) {
    this.autopilotActive = Boolean(active);
    const autopilotBtn = this.container?.querySelector('#btn-toggle-autopilot');
    if (autopilotBtn) {
      autopilotBtn.textContent = active ? 'AUTOPILOT / SPECTATE: ON [P]' : 'MANUAL DRIVE: ACTIVE [P]';
      autopilotBtn.classList.toggle('active', active);
      autopilotBtn.style.color = active ? '#00f0ff' : '#ffd600';
      autopilotBtn.style.borderColor = active ? '#00f0ff' : '#ffd600';
    }
    const hudBadge = document.querySelector('#hud-autopilot-badge');
    if (hudBadge) {
      hudBadge.textContent = active ? 'AUTOPILOT [ON]' : 'MANUAL [ACTIVE]';
      hudBadge.style.color = active ? '#00f0ff' : '#ffd600';
      hudBadge.style.borderColor = active ? '#00f0ff55' : '#ffd60055';
    }
  }

  setCameraMode(mode) {
    this.cameraMode = mode;
    const camButtons = this.container.querySelectorAll('[data-cam]');
    camButtons.forEach((b) => b.classList.toggle('active', b.dataset.cam === mode));
    const camLabel = this.container.querySelector('#deck-cam-label');
    if (camLabel) camLabel.textContent = mode;
    const hudCamMode = document.querySelector('[data-hud="cam-mode"]');
    if (hudCamMode) hudCamMode.textContent = mode;
    this.onCameraChange(mode);
  }

  updateBanner({ tag, title, objective, status, statusClass = '' } = {}) {
    const elTag = document.querySelector('[data-hud="scenario-tag"]');
    const elTitle = document.querySelector('[data-hud="scenario-title"]');
    const elObj = document.querySelector('[data-hud="scenario-objective"]');
    const elStatus = document.querySelector('[data-hud="scenario-status"]');

    if (tag && elTag) elTag.textContent = tag;
    if (title && elTitle) elTitle.textContent = title;
    if (objective && elObj) elObj.textContent = objective;
    if (status && elStatus) {
      elStatus.textContent = status;
      elStatus.className = `banner-status ${statusClass}`;
    }
  }

  updateAIThoughtHUD({
    state = 'TRAJECTORY_FOLLOW',
    action = 'CRUISE',
    reason = 'Evaluating optimal racing line',
    threat = 'LOW',
    aggression = 75,
    diveMargin = 0.45,
    defense = 80,
    kerb = 90,
    lookahead = 22.0,
    throttle = 1.0,
    brake = 0.0,
    steer = 0.0,
    tactic = 'Covering inside corridor',
    prediction = 'Clear apex in 1.4s',
    decision = 'Commit to line'
  } = {}) {
    const elState = document.querySelector('[data-hud="ai-state-badge"]');
    const elAction = document.querySelector('[data-hud="ai-intent-action"]');
    const elReason = document.querySelector('[data-hud="ai-intent-reason"]');
    const elThreat = document.querySelector('[data-hud="ai-stat-threat"]');
    const elAgg = document.querySelector('[data-hud="ai-stat-aggression"]');
    const elDive = document.querySelector('[data-hud="ai-stat-dive-margin"]');
    const elDef = document.querySelector('[data-hud="ai-stat-defense"]');
    const elKerb = document.querySelector('[data-hud="ai-stat-kerb"]');
    const elLook = document.querySelector('[data-hud="ai-stat-lookahead"]');
    const elControls = document.querySelector('[data-hud="ai-controls-readout"]');
    const elTactic = document.querySelector('[data-hud="ai-thought-tactic"]');
    const elPred = document.querySelector('[data-hud="ai-thought-pred"]');
    const elDecision = document.querySelector('[data-hud="ai-thought-decision"]');

    if (elState) elState.textContent = state;
    if (elAction) elAction.textContent = action;
    if (elReason) elReason.textContent = reason;
    if (elThreat) elThreat.textContent = threat;
    if (elAgg) elAgg.textContent = `${aggression}%`;
    if (elDive) elDive.textContent = `${diveMargin.toFixed(2)}m`;
    if (elDef) elDef.textContent = `${defense}%`;
    if (elKerb) elKerb.textContent = `${kerb}%`;
    if (elLook) elLook.textContent = `${lookahead.toFixed(1)}m`;
    if (elControls) {
      elControls.textContent = `THROTTLE: ${(throttle * 100).toFixed(0)}% · BRAKE: ${(brake * 100).toFixed(0)}% · STEER: ${steer.toFixed(2)}`;
    }
    if (elTactic) elTactic.textContent = tactic;
    if (elPred) elPred.textContent = prediction;
    if (elDecision) elDecision.textContent = decision;
  }

  updateLapTelemetry({
    currentLapTime = 0,
    lastLapTime = null,
    bestLapTime = null,
    deltaS = 0,
    baselineLapTime = null,
    hasBaseline = false,
    formatTime = (s) => (Number.isFinite(s) && s > 0 ? `${s.toFixed(3)}s` : '--:--.---')
  } = {}) {
    const elCurrent = this.container?.querySelector('#deck-lap-current');
    const elDelta = this.container?.querySelector('#deck-lap-delta');
    const elLast = this.container?.querySelector('#deck-lap-last');
    const elBest = this.container?.querySelector('#deck-lap-best');
    const elStatus = this.container?.querySelector('#deck-baseline-status');

    const hudBaseline = document.querySelector('[data-hud="baseline-lap"]');
    const hudDelta = document.querySelector('[data-hud="baseline-delta"]');

    if (elCurrent) elCurrent.textContent = formatTime(currentLapTime);
    if (elLast) elLast.textContent = formatTime(lastLapTime);
    if (elBest) elBest.textContent = formatTime(bestLapTime);

    const baselineDisplay = hasBaseline && baselineLapTime ? formatTime(baselineLapTime) : 'NOT SET';
    if (elStatus) elStatus.textContent = baselineDisplay;
    if (hudBaseline) hudBaseline.textContent = baselineDisplay;

    const deltaSign = deltaS > 0 ? `+${deltaS.toFixed(3)}s` : `${deltaS.toFixed(3)}s`;
    const deltaColor = !hasBaseline ? '#8899aa' : (deltaS > 0.05 ? '#ff3d00' : (deltaS < -0.05 ? '#00e676' : '#00e5ff'));

    if (elDelta) {
      elDelta.textContent = hasBaseline ? deltaSign : '+0.000s';
      elDelta.style.color = deltaColor;
    }
    if (hudDelta) {
      hudDelta.textContent = hasBaseline ? deltaSign : '+0.000s';
      hudDelta.style.color = deltaColor;
    }
  }
}
