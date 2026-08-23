const finite = (val, fallback = 0) => (Number.isFinite(val) ? val : fallback);
const clamp = (val, min, max) => Math.max(min, Math.min(max, val));

const HUD_STYLES = `
.ai-telemetry-overlay {
  position: fixed;
  bottom: 18px;
  right: 18px;
  width: 440px;
  background: rgba(6, 12, 18, 0.92);
  border: 1px solid rgba(0, 240, 255, 0.35);
  box-shadow: 0 8px 32px rgba(0, 0, 0, 0.75), inset 0 0 16px rgba(0, 240, 255, 0.08);
  border-radius: 8px;
  color: #e0f7fa;
  font-family: 'Consolas', 'Menlo', 'Monaco', monospace;
  font-size: 11px;
  line-height: 1.35;
  z-index: 9999;
  backdrop-filter: blur(8px);
  pointer-events: auto;
  user-select: none;
  display: flex;
  flex-direction: column;
  gap: 8px;
  padding: 12px;
  box-sizing: border-box;
  transition: opacity 0.2s ease, transform 0.2s ease;
}

.ai-telemetry-overlay.hidden {
  display: none !important;
}

.ai-telemetry-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  border-bottom: 1px solid rgba(0, 240, 255, 0.25);
  padding-bottom: 6px;
}

.ai-telemetry-title {
  font-weight: 800;
  font-size: 12px;
  color: #00f0ff;
  letter-spacing: 1px;
  display: flex;
  align-items: center;
  gap: 6px;
}

.ai-telemetry-badge {
  background: rgba(0, 240, 255, 0.15);
  border: 1px solid #00f0ff;
  border-radius: 3px;
  padding: 1px 5px;
  font-size: 9px;
  color: #00f0ff;
}

.ai-telemetry-grid-top {
  display: grid;
  grid-template-columns: 140px 1fr;
  gap: 10px;
}

.ai-gg-panel {
  display: flex;
  flex-direction: column;
  align-items: center;
  background: rgba(0, 0, 0, 0.45);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 6px;
  padding: 6px;
}

.ai-gg-canvas {
  width: 128px;
  height: 128px;
}

.ai-gg-readout {
  font-size: 9.5px;
  color: #80deea;
  margin-top: 4px;
  text-align: center;
  width: 100%;
}

.ai-thought-box {
  background: rgba(0, 0, 0, 0.45);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 6px;
  padding: 6px 8px;
  display: flex;
  flex-direction: column;
  gap: 4px;
}

.ai-mode-row {
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.ai-mode-tag {
  font-weight: 800;
  font-size: 11px;
  padding: 2px 6px;
  border-radius: 3px;
  background: #00f0ff;
  color: #000;
  letter-spacing: 0.5px;
}

.ai-reason-text {
  font-size: 10px;
  color: #ffeb3b;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.ai-thought-metrics {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 3px 8px;
  margin-top: 4px;
  font-size: 9.5px;
}

.ai-thought-metrics span {
  color: #90a4ae;
}

.ai-thought-metrics b {
  color: #fff;
}

.ai-cost-table-panel {
  background: rgba(0, 0, 0, 0.45);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 6px;
  padding: 6px 8px;
}

.ai-table-title {
  font-size: 9.5px;
  font-weight: 700;
  color: #80cbc4;
  margin-bottom: 4px;
  text-transform: uppercase;
}

.ai-cost-table {
  width: 100%;
  border-collapse: collapse;
  font-size: 9px;
  text-align: left;
}

.ai-cost-table th {
  color: #78909c;
  padding: 2px 4px;
  border-bottom: 1px solid rgba(255, 255, 255, 0.12);
}

.ai-cost-table td {
  padding: 2px 4px;
  border-bottom: 1px solid rgba(255, 255, 255, 0.05);
}

.ai-cost-row-chosen {
  background: rgba(0, 255, 136, 0.12);
  color: #00ff88;
  font-weight: 700;
}

.ai-cost-row-alt {
  color: #b0bec5;
}

.ai-cost-row-blocked {
  color: #ff5252;
}

.ai-telemetry-bars-panel {
  background: rgba(0, 0, 0, 0.45);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 6px;
  padding: 6px 8px;
  display: flex;
  flex-direction: column;
  gap: 5px;
}

.ai-bar-group {
  display: flex;
  align-items: center;
  gap: 6px;
  font-size: 9.5px;
}

.ai-bar-label {
  width: 48px;
  color: #90a4ae;
}

.ai-bar-track {
  flex: 1;
  height: 7px;
  background: rgba(255, 255, 255, 0.1);
  border-radius: 3px;
  overflow: hidden;
  position: relative;
}

.ai-bar-fill {
  height: 100%;
  width: 0%;
  border-radius: 3px;
  transition: width 0.05s linear;
}

.ai-bar-fill.throttle { background: #00ff88; }
.ai-bar-fill.brake { background: #ff3355; }
.ai-bar-fill.rpm { background: linear-gradient(90deg, #00f0ff, #ffeb3b, #ff1744); }
.ai-bar-fill.ers { background: linear-gradient(90deg, #ff9100, #00e5ff); }

.ai-bar-val {
  width: 55px;
  text-align: right;
  color: #fff;
  font-weight: 700;
}

.ai-steer-track {
  flex: 1;
  height: 7px;
  background: rgba(255, 255, 255, 0.1);
  border-radius: 3px;
  position: relative;
}

.ai-steer-center {
  position: absolute;
  left: 50%;
  top: 0;
  bottom: 0;
  width: 1px;
  background: rgba(255, 255, 255, 0.4);
}

.ai-steer-marker {
  position: absolute;
  top: 0;
  bottom: 0;
  width: 4px;
  background: #00f0ff;
  border-radius: 2px;
  transform: translateX(-50%);
}
`;

/**
 * High-Performance Telemetry HUD Overlay
 * Features live G-G friction circle with history trail, live thought box with tactical modes,
 * candidate cost table with viability ratings, and full vehicle dynamics / ERS powertrain bars.
 */
export class TelemetryHUD {
  constructor(options = {}) {
    this.visible = true;
    this.historyCapacity = options.historyCapacity ?? 50;
    this.ggHistory = [];
    this.lastDomUpdate = 0;
    this.domUpdateInterval = 50; // 20Hz DOM refresh for text/tables

    this._initDOM();
    this._initCanvas();
  }

  _initDOM() {
    // Inject stylesheet if not already present
    if (typeof document !== 'undefined' && !document.getElementById('ai-telemetry-hud-styles')) {
      const styleEl = document.createElement('style');
      styleEl.id = 'ai-telemetry-hud-styles';
      styleEl.textContent = HUD_STYLES;
      document.head.appendChild(styleEl);
    }

    if (typeof document === 'undefined') return;

    this.root = document.createElement('div');
    this.root.className = 'ai-telemetry-overlay';
    this.root.innerHTML = `
      <div class="ai-telemetry-header">
        <div class="ai-telemetry-title">
          <span>🧠 AI TELEMETRY HUD</span>
          <span class="ai-telemetry-badge" id="ai-hud-vehicle-tag">PROTOTYPE #1</span>
        </div>
        <div style="display: flex; gap: 6px; align-items: center;">
          <button id="ai-hud-flag-btn" style="background: rgba(255, 234, 0, 0.2); border: 1px solid #ffea00; color: #ffea00; border-radius: 4px; padding: 2px 6px; font-size: 9px; cursor: pointer; font-weight: 700;">🚩 Flag (M)</button>
          <button id="ai-hud-incidents-toggle-btn" style="background: rgba(0, 240, 255, 0.2); border: 1px solid #00f0ff; color: #00f0ff; border-radius: 4px; padding: 2px 6px; font-size: 9px; cursor: pointer; font-weight: 700;">Hotspots (<span id="ai-hud-incidents-count">0</span>)</button>
          <div class="ai-telemetry-badge" id="ai-hud-status-badge" style="background: rgba(0, 255, 136, 0.15); border-color: #00ff88; color: #00ff88;">OPTIMIZING</div>
        </div>
      </div>

      <!-- Incident Hotspot Drawer -->
      <div id="ai-incident-drawer" style="display: none; background: rgba(12, 18, 24, 0.96); border: 1px solid #ffea00; border-radius: 6px; padding: 8px; flex-direction: column; gap: 6px; max-height: 180px; overflow-y: auto;">
        <div style="display: flex; justify-content: space-between; align-items: center; border-bottom: 1px solid rgba(255, 234, 0, 0.3); padding-bottom: 4px;">
          <span style="font-weight: 800; color: #ffea00; font-size: 10px;">🚩 FLAGGED INCIDENTS & HOTSPOTS</span>
          <div style="display: flex; gap: 4px;">
            <button id="ai-incident-export-btn" style="background: #00e5ff; color: #000; border: none; border-radius: 3px; font-size: 8.5px; font-weight: 800; padding: 2px 5px; cursor: pointer;">📥 EXPORT JSON</button>
            <button id="ai-incident-clear-btn" style="background: rgba(255, 82, 82, 0.3); color: #ff5252; border: 1px solid #ff5252; border-radius: 3px; font-size: 8.5px; font-weight: 800; padding: 2px 5px; cursor: pointer;">CLEAR</button>
          </div>
        </div>
        <div id="ai-incident-list" style="display: flex; flex-direction: column; gap: 4px; font-size: 9px;">
          <span style="color: #90a4ae;">No incidents flagged yet. Press [M] or click Flag to mark any hesitation/slide.</span>
        </div>
      </div>

      <div class="ai-telemetry-grid-top">
        <div class="ai-gg-panel">
          <canvas class="ai-gg-canvas" id="ai-gg-canvas" width="128" height="128"></canvas>
          <div class="ai-gg-readout" id="ai-gg-readout">0.00G LAT | 0.00G LONG</div>
        </div>

        <div class="ai-thought-box">
          <div class="ai-mode-row">
            <span class="ai-mode-tag" id="ai-thought-mode">PACE</span>
            <span id="ai-thought-target" style="color: #90a4ae; font-size: 9.5px;">TARGET: OPTIMAL_RACING_LINE</span>
          </div>
          <div class="ai-reason-text" id="ai-thought-reason">Tracking racing line baseline with minimum jerk lateral dynamics</div>
          
          <div class="ai-thought-metrics">
            <div><span>TRACK POS:</span> <b id="ai-metric-dist">0.0 m</b></div>
            <div><span>TARGET SPD:</span> <b id="ai-metric-tgt-spd">0 KM/H</b></div>
            <div><span>GAP:</span> <b id="ai-metric-gap">-- m</b></div>
            <div><span>CLOSING:</span> <b id="ai-metric-closing">-- km/h</b></div>
            <div><span>TTC:</span> <b id="ai-metric-ttc">-- s</b></div>
            <div><span>CLEARANCE:</span> <b id="ai-metric-clearance">-- m</b></div>
            <div><span>TCS BUDGET:</span> <b id="ai-metric-tcs" style="color: #00ff88;">100%</b></div>
            <div><span>STABILITY:</span> <b id="ai-metric-stab" style="color: #00f0ff;">100%</b></div>
          </div>
        </div>
      </div>

      <div class="ai-cost-table-panel">
        <div class="ai-table-title">FRENET LATTICE TRAJECTORY COST MATRIX (24 CANDIDATES)</div>
        <table class="ai-cost-table">
          <thead>
            <tr>
              <th>CANDIDATE</th>
              <th>OFFSET</th>
              <th>TIME</th>
              <th>CLEAR</th>
              <th>SCORE</th>
              <th>STATUS</th>
            </tr>
          </thead>
          <tbody id="ai-cost-table-body">
            <tr class="ai-cost-row-chosen">
              <td>#1 CHOSEN</td>
              <td>0.0 m</td>
              <td>1.2 s</td>
              <td>99 m</td>
              <td>0.0</td>
              <td>VALID</td>
            </tr>
          </tbody>
        </table>
      </div>

      <div class="ai-telemetry-bars-panel">
        <div class="ai-bar-group">
          <span class="ai-bar-label">SPD / RPM</span>
          <div class="ai-bar-track">
            <div class="ai-bar-fill rpm" id="ai-bar-rpm" style="width: 0%;"></div>
          </div>
          <span class="ai-bar-val" id="ai-val-spd">0 KM/H</span>
        </div>

        <div class="ai-bar-group">
          <span class="ai-bar-label">THROTTLE</span>
          <div class="ai-bar-track">
            <div class="ai-bar-fill throttle" id="ai-bar-thr" style="width: 0%;"></div>
          </div>
          <span class="ai-bar-val" id="ai-val-thr">0%</span>
        </div>

        <div class="ai-bar-group">
          <span class="ai-bar-label">BRAKE</span>
          <div class="ai-bar-track">
            <div class="ai-bar-fill brake" id="ai-bar-brk" style="width: 0%;"></div>
          </div>
          <span class="ai-bar-val" id="ai-val-brk">0%</span>
        </div>

        <div class="ai-bar-group">
          <span class="ai-bar-label">STEERING</span>
          <div class="ai-steer-track">
            <div class="ai-steer-center"></div>
            <div class="ai-steer-marker" id="ai-steer-marker" style="left: 50%;"></div>
          </div>
          <span class="ai-bar-val" id="ai-val-steer">0.0°</span>
        </div>

        <div class="ai-bar-group">
          <span class="ai-bar-label">ERS SOC</span>
          <div class="ai-bar-track">
            <div class="ai-bar-fill ers" id="ai-bar-ers" style="width: 0%;"></div>
          </div>
          <span class="ai-bar-val" id="ai-val-ers">--%</span>
        </div>
      </div>
    `;

    document.body.appendChild(this.root);

    // Cache DOM references
    this.el = {
      vehicleTag: this.root.querySelector('#ai-hud-vehicle-tag'),
      statusBadge: this.root.querySelector('#ai-hud-status-badge'),
      flagBtn: this.root.querySelector('#ai-hud-flag-btn'),
      incidentsToggleBtn: this.root.querySelector('#ai-hud-incidents-toggle-btn'),
      incidentsCount: this.root.querySelector('#ai-hud-incidents-count'),
      incidentDrawer: this.root.querySelector('#ai-incident-drawer'),
      incidentList: this.root.querySelector('#ai-incident-list'),
      incidentExportBtn: this.root.querySelector('#ai-incident-export-btn'),
      incidentClearBtn: this.root.querySelector('#ai-incident-clear-btn'),
      ggReadout: this.root.querySelector('#ai-gg-readout'),
      modeTag: this.root.querySelector('#ai-thought-mode'),
      targetTag: this.root.querySelector('#ai-thought-target'),
      reasonText: this.root.querySelector('#ai-thought-reason'),
      metricDist: this.root.querySelector('#ai-metric-dist'),
      metricTgtSpd: this.root.querySelector('#ai-metric-tgt-spd'),
      metricGap: this.root.querySelector('#ai-metric-gap'),
      metricClosing: this.root.querySelector('#ai-metric-closing'),
      metricTTC: this.root.querySelector('#ai-metric-ttc'),
      metricClearance: this.root.querySelector('#ai-metric-clearance'),
      metricTcs: this.root.querySelector('#ai-metric-tcs'),
      metricStab: this.root.querySelector('#ai-metric-stab'),
      costTableBody: this.root.querySelector('#ai-cost-table-body'),
      barRpm: this.root.querySelector('#ai-bar-rpm'),
      valSpd: this.root.querySelector('#ai-val-spd'),
      barThr: this.root.querySelector('#ai-bar-thr'),
      valThr: this.root.querySelector('#ai-val-thr'),
      barBrk: this.root.querySelector('#ai-bar-brk'),
      valBrk: this.root.querySelector('#ai-val-brk'),
      steerMarker: this.root.querySelector('#ai-steer-marker'),
      valSteer: this.root.querySelector('#ai-val-steer'),
      barErs: this.root.querySelector('#ai-bar-ers'),
      valErs: this.root.querySelector('#ai-val-ers')
    };

    // Attach incident button listeners
    this.el.flagBtn?.addEventListener('click', () => {
      this.onMarkIncident?.('User clicked Flag button');
    });

    this.el.incidentsToggleBtn?.addEventListener('click', () => {
      if (this.el.incidentDrawer) {
        const isHidden = this.el.incidentDrawer.style.display === 'none';
        this.el.incidentDrawer.style.display = isHidden ? 'flex' : 'none';
      }
    });

    this.el.incidentExportBtn?.addEventListener('click', () => {
      this.onExportIncidents?.();
    });

    this.el.incidentClearBtn?.addEventListener('click', () => {
      this.onClearIncidents?.();
    });
  }

  _initCanvas() {
    if (typeof document === 'undefined') return;
    this.canvas = this.root?.querySelector('#ai-gg-canvas');
    this.ctx = this.canvas?.getContext('2d') ?? null;
  }

  /**
   * Main telemetry update loop
   */
  update(aiVehicle, aiController, track = null, now = 0) {
    if (!this.visible || !aiVehicle) return;

    // 1. Update G-G Canvas at 60fps
    this._updateGGCanvas(aiVehicle);

    // 2. Throttled DOM update for text, tables, and bars
    if (now - this.lastDomUpdate >= this.domUpdateInterval) {
      this.lastDomUpdate = now;
      this._updateDOM(aiVehicle, aiController, track);
    }
  }

  _updateGGCanvas(vehicle) {
    if (!this.ctx || !this.canvas) return;

    const ctx = this.ctx;
    const w = this.canvas.width;
    const h = this.canvas.height;
    const cx = w * 0.5;
    const cy = h * 0.5;
    const maxG = 2.5; // Radius scale in G's
    const scale = (w * 0.44) / maxG;

    // Extract lateral and longitudinal G
    const telem = vehicle.telemetry ?? {};
    const latG = finite(telem.lateralG);
    const longG = finite(telem.longitudinalG);

    // Append to G-G historical trail ring buffer
    this.ggHistory.push({ x: latG, y: longG });
    if (this.ggHistory.length > this.historyCapacity) {
      this.ggHistory.shift();
    }

    // Clear Canvas
    ctx.clearRect(0, 0, w, h);

    // Draw Background Grid & Concentric G Circles
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.18)';
    ctx.lineWidth = 1.5;

    const gRings = [0.5, 1.0, 1.5, 2.0, 2.5];
    for (const g of gRings) {
      const r = g * scale;
      ctx.beginPath();
      ctx.arc(cx, cy, r, 0, Math.PI * 2);
      ctx.stroke();
    }

    // Crosshair axes
    ctx.beginPath();
    ctx.moveTo(cx, cy - maxG * scale);
    ctx.lineTo(cx, cy + maxG * scale);
    ctx.moveTo(cx - maxG * scale, cy);
    ctx.lineTo(cx + maxG * scale, cy);
    ctx.stroke();

    // Ring G Labels
    ctx.fillStyle = 'rgba(0, 240, 255, 0.45)';
    ctx.font = '16px Consolas, monospace';
    ctx.fillText('1.0G', cx + 1.0 * scale + 4, cy - 4);
    ctx.fillText('2.0G', cx + 2.0 * scale + 4, cy - 4);

    // Draw Historical G-G Breadcrumbs Trail
    for (let i = 0; i < this.ggHistory.length; i++) {
      const pt = this.ggHistory[i];
      const alpha = (i + 1) / this.ggHistory.length;
      const px = cx + pt.x * scale;
      const py = cy - pt.y * scale; // Inverted Y: +Long G is acceleration (up) / -Long G is braking (down)

      ctx.fillStyle = `rgba(0, 240, 255, ${alpha * 0.45})`;
      ctx.beginPath();
      ctx.arc(px, py, 2.5, 0, Math.PI * 2);
      ctx.fill();
    }

    // Draw Current Instantaneous G Point
    const curX = cx + clamp(latG, -maxG, maxG) * scale;
    const curY = cy - clamp(longG, -maxG, maxG) * scale;

    // Glowing outer ring
    ctx.strokeStyle = '#00ff88';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.arc(curX, curY, 6.5, 0, Math.PI * 2);
    ctx.stroke();

    // Solid inner dot
    ctx.fillStyle = '#ffffff';
    ctx.beginPath();
    ctx.arc(curX, curY, 3.5, 0, Math.PI * 2);
    ctx.fill();

    // Numerical Readout
    if (this.el?.ggReadout) {
      const latSign = latG >= 0 ? '+' : '';
      const longSign = longG >= 0 ? '+' : '';
      const totalG = Math.hypot(latG, longG);
      this.el.ggReadout.textContent = `${latSign}${latG.toFixed(2)} LAT · ${longSign}${longG.toFixed(2)} LONG (${totalG.toFixed(2)}G)`;
    }
  }

  _updateDOM(vehicle, controller, track) {
    if (!this.el?.modeTag) return;

    const state = controller?.debugState ?? {};
    const thought = state.thought ?? {};
    const tactical = vehicle.aiTactical ?? {};

    // 1. Vehicle Tag & Status
    this.el.vehicleTag.textContent = `${vehicle.name ?? 'AI'} // ${String(vehicle.classKey ?? 'GT').toUpperCase()}`;

    // 2. Active Mode Badge & Color
    const activeMode = String(
      thought.deployedManeuver ?? tactical.racecraftPhase ?? state.mode ?? 'PACE'
    ).toUpperCase();
    this.el.modeTag.textContent = activeMode;

    if (activeMode.includes('ATTACK') || activeMode.includes('PASS') || activeMode.includes('DIVEBOMB') || activeMode.includes('SWITCHBACK')) {
      this.el.modeTag.style.background = '#00ff88';
      this.el.modeTag.style.color = '#000';
    } else if (activeMode.includes('APEX_SHIELD') || activeMode.includes('DIAMOND') || activeMode.includes('DEFEND') || activeMode.includes('SQUEEZE') || activeMode.includes('LOCK_DEFENSIVE') || activeMode.includes('ONE_MOVE')) {
      this.el.modeTag.style.background = '#d500f9';
      this.el.modeTag.style.color = '#fff';
    } else if (activeMode.includes('BREAK_TOW')) {
      this.el.modeTag.style.background = '#ff6d00';
      this.el.modeTag.style.color = '#000';
    } else if (activeMode.includes('BRAKE')) {
      this.el.modeTag.style.background = '#ffe15a';
      this.el.modeTag.style.color = '#000';
    } else if (activeMode.includes('RECOVER')) {
      this.el.modeTag.style.background = '#ff3355';
      this.el.modeTag.style.color = '#fff';
    } else {
      this.el.modeTag.style.background = '#00f0ff';
      this.el.modeTag.style.color = '#000';
    }

    // 3. Target & Reason
    const targetId = thought.targetId ?? state.passTargetId ?? state.draftTargetId ?? 'CLEAR';
    this.el.targetTag.textContent = targetId !== 'CLEAR' ? `TGT: ${targetId}` : 'TGT: CLEAR';

    const reason = thought.abortReason ?? thought.waitReason ?? state.reason ?? 'OPEN_RACING_LINE';
    this.el.reasonText.textContent = String(reason);

    // 4. Metrics
    const trackLen = finite(track?.length, 3061.7);
    const trackDist = ((finite(vehicle.distance, 0) % trackLen) + trackLen) % trackLen;
    if (this.el.metricDist) this.el.metricDist.textContent = `${trackDist.toFixed(1)} m`;
    if (this.el.metricTgtSpd) this.el.metricTgtSpd.textContent = `${Math.round(finite(state.targetSpeed, 0) * 3.6)} KM/H`;

    const traffic = state.traffic ?? controller?.awareness?.lastScan;
    const targetEntry = traffic?.entries?.find?.(e => e.other?.id === targetId) ?? null;

    if (targetEntry) {
      this.el.metricGap.textContent = `${finite(targetEntry.delta).toFixed(1)} m`;
      this.el.metricClosing.textContent = `${(finite(targetEntry.relativeLongitudinalVelocity) * 3.6).toFixed(1)} km/h`;
      this.el.metricTTC.textContent = targetEntry.ttc < 50 ? `${targetEntry.ttc.toFixed(2)} s` : '> 5.0 s';
    } else {
      this.el.metricGap.textContent = '-- m';
      this.el.metricClosing.textContent = '-- km/h';
      this.el.metricTTC.textContent = '-- s';
    }

    const clearance = finite(thought.trajectoryMinimumClearanceM, finite(state.trajectoryMinimumClearanceM, 99));
    this.el.metricClearance.textContent = clearance < 90 ? `${clearance.toFixed(1)} m` : '> 10 m';

    // Control Limiters (TCS & Stability)
    const telem = vehicle.telemetry ?? {};
    const latG = Math.abs(finite(telem.lateralG, 0));
    const tcsPct = Math.round(Math.sqrt(Math.max(0.1, 1.0 - Math.pow(latG / 2.7 * 0.90, 2))) * 100);
    if (this.el.metricTcs) {
      this.el.metricTcs.textContent = `${tcsPct}%`;
      this.el.metricTcs.style.color = tcsPct < 60 ? '#ffea00' : '#00ff88';
    }
    const slipAngle = Math.abs(finite(vehicle.localVelocity?.x, 0)) / Math.max(1, Math.abs(finite(vehicle.localVelocity?.z, 1)));
    const stabPct = Math.round(clamp(1.0 - Math.max(0, slipAngle - 0.04) / 0.12, 0, 1) * 100);
    if (this.el.metricStab) {
      this.el.metricStab.textContent = `${stabPct}%`;
      this.el.metricStab.style.color = stabPct < 70 ? '#ff3355' : '#00f0ff';
    }

    // 5. Candidate Trajectory Cost Table
    this._updateCandidateTable(controller, state);

    // 6. Telemetry Bars
    const speedKmh = Math.round(finite(vehicle.speed) * 3.6);
    const rpm = Math.round(finite(vehicle.rpm, 1000));
    const maxRpm = vehicle.spec?.engine?.maxRPM ?? 9000;
    const gear = vehicle.gear ?? 1;
    const throttle = clamp(finite(vehicle.controls?.throttle), 0, 1);
    const brake = clamp(finite(vehicle.controls?.brake), 0, 1);
    const steer = clamp(finite(vehicle.controls?.steer), -1, 1);

    this.el.valSpd.textContent = `G${gear} · ${speedKmh} KM/H`;
    this.el.barRpm.style.width = `${Math.min(100, (rpm / maxRpm) * 100)}%`;

    this.el.valThr.textContent = `${Math.round(throttle * 100)}%`;
    this.el.barThr.style.width = `${throttle * 100}%`;

    this.el.valBrk.textContent = `${Math.round(brake * 100)}%`;
    this.el.barBrk.style.width = `${brake * 100}%`;

    this.el.valSteer.textContent = `${(steer * 28.5).toFixed(1)}°`;
    this.el.steerMarker.style.left = `${(steer * 0.5 + 0.5) * 100}%`;

    // ERS Telemetry
    if (vehicle.ers?.enabled) {
      const soc = clamp(finite(vehicle.ers.soc) * 100, 0, 100);
      const powerKw = Math.round(finite(vehicle.ers.deployPowerW) / 1000);
      this.el.valErs.textContent = `${Math.round(soc)}% (${powerKw} kW)`;
      this.el.barErs.style.width = `${soc}%`;
    } else {
      this.el.valErs.textContent = 'OFF';
      this.el.barErs.style.width = '0%';
    }
  }

  _updateCandidateTable(controller, state) {
    const candidates = controller?.trajectoryPlanner?.lastCandidates
      ?? state.candidates
      ?? state.trajectory?.candidates
      ?? [];

    if (!Array.isArray(candidates) || candidates.length === 0) {
      // Fallback row if no candidate list is available
      const chosenOffset = finite(state.trajectorySelectedOffsetM, finite(state.targetOffset, 0));
      const chosenClear = finite(state.trajectoryMinimumClearanceM, 99);
      const score = finite(state.trajectoryScore, 0);

      this.el.costTableBody.innerHTML = `
        <tr class="ai-cost-row-chosen">
          <td>#1 CHOSEN</td>
          <td>${chosenOffset > 0 ? '+' : ''}${chosenOffset.toFixed(1)} m</td>
          <td>1.25 s</td>
          <td>${chosenClear < 90 ? chosenClear.toFixed(1) + 'm' : 'CLEAR'}</td>
          <td>${score.toFixed(1)}</td>
          <td>CHOSEN</td>
        </tr>
      `;
      return;
    }

    // Sort candidates by cost/score
    const sorted = [...candidates].sort((a, b) => (a.score ?? 0) - (b.score ?? 0)).slice(0, 3);

    let rowsHtml = '';
    for (let i = 0; i < sorted.length; i++) {
      const cand = sorted[i];
      const isChosen = i === 0;
      const rowClass = isChosen ? 'ai-cost-row-chosen'
        : !cand.collisionFree ? 'ai-cost-row-blocked'
        : 'ai-cost-row-alt';

      const statusTag = isChosen ? 'CHOSEN'
        : !cand.collisionFree ? 'BLOCKED'
        : !cand.roadLegal ? 'EDGE'
        : 'VIABLE';

      const offset = finite(cand.terminalLateral);
      const transTime = finite(cand.transitionTime, 1.2);
      const clearance = finite(cand.minimumClearanceM, 99);
      const score = finite(cand.score, 0);

      rowsHtml += `
        <tr class="${rowClass}">
          <td>#${i + 1} ${isChosen ? 'CHOSEN' : 'ALT'}</td>
          <td>${offset > 0 ? '+' : ''}${offset.toFixed(1)} m</td>
          <td>${transTime.toFixed(2)} s</td>
          <td>${clearance < 90 ? clearance.toFixed(1) + 'm' : 'CLEAR'}</td>
          <td>${score.toFixed(1)}</td>
          <td>${statusTag}</td>
        </tr>
      `;
    }

    this.el.costTableBody.innerHTML = rowsHtml;
  }

  updateIncidents(incidents = []) {
    if (!this.el?.incidentList) return;

    if (this.el.incidentsCount) {
      this.el.incidentsCount.textContent = String(incidents.length);
    }

    if (!Array.isArray(incidents) || incidents.length === 0) {
      this.el.incidentList.innerHTML = `
        <span style="color: #90a4ae;">No incidents flagged yet. Press [M] or click Flag to mark any hesitation/slide.</span>
      `;
      return;
    }

    // Render latest 6 incidents
    const latest = [...incidents].reverse().slice(0, 6);
    let html = '';
    for (const inc of latest) {
      const typeColor = inc.type === 'SPIN_LOSS_OF_CONTROL' ? '#ff1744'
        : inc.type === 'THROTTLE_HESITATION' ? '#ffea00'
        : inc.type === 'OFF_TRACK_EXCURSION' ? '#ff9100' : '#00e5ff';

      html += `
        <div style="background: rgba(255, 255, 255, 0.05); border-left: 3px solid ${typeColor}; padding: 4px 6px; border-radius: 2px;">
          <div style="display: flex; justify-content: space-between; align-items: center;">
            <b style="color: ${typeColor}; font-size: 9px;">${inc.type}</b>
            <span style="color: #80deea; font-size: 8.5px;">@ ${inc.distanceM}m (Lap ${inc.lap})</span>
          </div>
          <div style="color: #cfd8dc; font-size: 8.5px; margin-top: 2px;">${inc.diagnosis}</div>
          <div style="color: #78909c; font-size: 8px; margin-top: 1px;">Spd: ${inc.speedKmh} km/h (Tgt: ${inc.targetSpeedKmh}) | Thr: ${(inc.throttle * 100).toFixed(0)}% | Yaw: ${inc.yawRateRps} r/s</div>
        </div>
      `;
    }

    this.el.incidentList.innerHTML = html;
  }

  setVisible(visible) {
    this.visible = Boolean(visible);
    if (this.root) {
      this.root.classList.toggle('hidden', !this.visible);
    }
    return this.visible;
  }

  toggle() {
    return this.setVisible(!this.visible);
  }

  dispose() {
    if (this.root && this.root.parentNode) {
      this.root.parentNode.removeChild(this.root);
    }
    if (typeof document !== 'undefined') {
      const styleEl = document.getElementById('ai-telemetry-hud-styles');
      if (styleEl && styleEl.parentNode) {
        styleEl.parentNode.removeChild(styleEl);
      }
    }
    this.ggHistory.length = 0;
    this.canvas = null;
    this.ctx = null;
    this.root = null;
  }
}
