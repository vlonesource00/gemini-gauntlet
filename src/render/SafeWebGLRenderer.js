/**
 * SafeWebGLRenderer.js - Resilient Three.js WebGLRenderer Initializer
 * 
 * Provides:
 * 1. Progressive multi-tier fallback context configurations (high-performance -> default -> no-MSAA -> low-power).
 * 2. Canvas DOM recycling to clear latched WebGL creation failure states in Gecko/WebKit.
 * 3. Graceful fallback dummy renderer preventing downstream script crashes.
 * 4. User-facing interactive diagnostic UI detailing exact Firefox (about:config) and Chrome/Edge fixes.
 */

export class DummyWebGLRenderer {
  constructor(canvas) {
    this.domElement = canvas || (typeof document !== 'undefined' ? document.createElement('canvas') : {});
    this.shadowMap = { enabled: false, type: 0 };
    this.toneMapping = 0;
    this.toneMappingExposure = 1.0;
    this.outputColorSpace = '';
    this.isDummy = true;
    this.info = {
      memory: { geometries: 0, textures: 0 },
      render: { calls: 0, triangles: 0, points: 0, lines: 0, frame: 0 }
    };
    this.capabilities = {
      isWebGL2: false,
      maxTextures: 16,
      maxVertexTextures: 16,
      precision: 'highp'
    };
  }

  setPixelRatio() {}
  setSize() {}
  render() {}
  dispose() {}
  clear() {}
  setClearColor() {}
  getPixelRatio() { return 1; }
  getSize(target) {
    if (target) {
      target.width = typeof window !== 'undefined' ? window.innerWidth : 1280;
      target.height = typeof window !== 'undefined' ? window.innerHeight : 720;
    }
    return target;
  }
}

/**
 * Creates a WebGLRenderer with progressive fallback profiles.
 *
 * @param {typeof import('three')} THREE
 * @param {Object} options
 * @param {HTMLCanvasElement} [options.canvas]
 * @param {HTMLElement} [options.container]
 * @param {string} [options.ariaLabel]
 * @param {string} [options.appName='GEMINI GAUNTLET']
 * @param {Function} [options.onCanvasReplaced]
 * @returns {{ renderer: import('three').WebGLRenderer | DummyWebGLRenderer, canvas: HTMLCanvasElement, isDummy: boolean, profileName: string }}
 */
export function createSafeWebGLRenderer(THREE, options = {}) {
  const {
    container = null,
    ariaLabel = '3D Simulation Viewport',
    appName = 'GEMINI GAUNTLET',
    onCanvasReplaced = null,
    ...customRendererOptions
  } = options;

  let currentCanvas = options.canvas || null;

  // Progressive fallback profiles for ANGLE / EGL / WebGL2 negotiation
  const profiles = [
    {
      name: 'High-Performance (MSAA Antialiased)',
      antialias: true,
      powerPreference: 'high-performance',
      failIfMajorPerformanceCaveat: false,
    },
    {
      name: 'Default-Power (MSAA Antialiased)',
      antialias: true,
      powerPreference: 'default',
      failIfMajorPerformanceCaveat: false,
    },
    {
      name: 'Balanced (Unspecified Power Preference)',
      antialias: true,
      failIfMajorPerformanceCaveat: false,
    },
    {
      name: 'Fallback (No MSAA, Default Power)',
      antialias: false,
      powerPreference: 'default',
      failIfMajorPerformanceCaveat: false,
    },
    {
      name: 'Minimal WebGL2 (Basic Driver Configuration)',
      antialias: false,
      failIfMajorPerformanceCaveat: false,
    },
    {
      name: 'Low-Power Compatibility Profile',
      antialias: false,
      powerPreference: 'low-power',
      failIfMajorPerformanceCaveat: false,
    }
  ];

  let lastError = null;
  let statusReason = '';

  for (let i = 0; i < profiles.length; i += 1) {
    const profile = profiles[i];

    // If previous attempt failed on an existing DOM canvas, recycle the DOM node
    // to reset any internal browser failure-latch state on that canvas
    if (i > 0 && currentCanvas && currentCanvas.parentNode) {
      try {
        const freshCanvas = currentCanvas.cloneNode(false);
        currentCanvas.parentNode.replaceChild(freshCanvas, currentCanvas);
        currentCanvas = freshCanvas;
        if (typeof onCanvasReplaced === 'function') {
          onCanvasReplaced(currentCanvas);
        }
      } catch (cloneErr) {
        console.warn('[SafeWebGLRenderer] Canvas node recycling note:', cloneErr.message);
      }
    }

    // Attach listener for webglcontextcreationerror to capture detailed diagnostic messages
    const creationErrorListener = (event) => {
      if (event && event.statusMessage) {
        statusReason = event.statusMessage;
      }
    };

    if (currentCanvas && typeof currentCanvas.addEventListener === 'function') {
      currentCanvas.addEventListener('webglcontextcreationerror', creationErrorListener, { once: true });
    }

    try {
      const initParams = {
        ...profile,
        ...customRendererOptions,
      };
      if (currentCanvas) {
        initParams.canvas = currentCanvas;
      }

      const renderer = new THREE.WebGLRenderer(initParams);
      const activeCanvas = renderer.domElement;

      if (ariaLabel && activeCanvas && typeof activeCanvas.setAttribute === 'function') {
        activeCanvas.setAttribute('aria-label', ariaLabel);
        activeCanvas.tabIndex = activeCanvas.tabIndex || 0;
      }

      if (container && activeCanvas && activeCanvas.parentNode !== container) {
        container.appendChild(activeCanvas);
      }

      if (i > 0) {
        console.warn(`[SafeWebGLRenderer] Initialized using fallback profile [${i + 1}/${profiles.length}]: "${profile.name}"`);
      } else {
        console.info(`[SafeWebGLRenderer] WebGL2 context successfully established (${profile.name}).`);
      }

      return {
        renderer,
        canvas: activeCanvas,
        isDummy: false,
        profileName: profile.name
      };
    } catch (err) {
      lastError = err;
      const detail = statusReason || err.message || String(err);
      console.warn(`[SafeWebGLRenderer] Profile ${i + 1} ("${profile.name}") failed: ${detail}`);
    }
  }

  // All progressive tiers failed: mount user diagnostic modal and return dummy renderer
  console.error('[SafeWebGLRenderer] All WebGLRenderer initialization tiers exhausted.', lastError);

  showWebGLDiagnosticModal({
    appName,
    error: lastError,
    statusReason: statusReason || (lastError ? lastError.message : 'Unknown driver error'),
    targetContainer: container || (typeof document !== 'undefined' ? document.body : null)
  });

  const dummyRenderer = new DummyWebGLRenderer(currentCanvas);
  if (container && dummyRenderer.domElement && dummyRenderer.domElement.parentNode !== container) {
    container.appendChild(dummyRenderer.domElement);
  }

  return {
    renderer: dummyRenderer,
    canvas: dummyRenderer.domElement,
    isDummy: true,
    profileName: 'Fallback-Dummy'
  };
}

/**
 * Displays a styled diagnostic modal when WebGL context creation fails completely.
 */
export function showWebGLDiagnosticModal({ appName, error, statusReason, targetContainer }) {
  if (typeof document === 'undefined') return;

  // Dismiss any loading screens if present so diagnostic modal is front and center
  const loadingScreen = document.querySelector('#loading-screen');
  if (loadingScreen) {
    loadingScreen.classList.add('dismissed');
  }

  // Prevent multiple diagnostic overlays
  const existing = document.getElementById('webgl-diagnostic-overlay');
  if (existing) existing.remove();

  const overlay = document.createElement('div');
  overlay.id = 'webgl-diagnostic-overlay';
  overlay.style.cssText = `
    position: fixed;
    inset: 0;
    z-index: 99999;
    display: flex;
    align-items: center;
    justify-content: center;
    padding: 24px;
    background: rgba(4, 8, 14, 0.94);
    backdrop-filter: blur(10px);
    -webkit-backdrop-filter: blur(10px);
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
    color: #e6edf3;
    overflow-y: auto;
  `;

  const card = document.createElement('div');
  card.style.cssText = `
    max-width: 680px;
    width: 100%;
    background: #0d1520;
    border: 1px solid #ff444466;
    box-shadow: 0 16px 48px rgba(0, 0, 0, 0.7), 0 0 24px rgba(255, 68, 68, 0.2);
    border-radius: 12px;
    padding: 28px;
    box-sizing: border-box;
  `;

  const reasonText = statusReason || (error?.message) || 'FEATURE_FAILURE_EGL_NO_CONFIG (Exhausted GL driver options)';

  card.innerHTML = `
    <div style="display: flex; align-items: center; gap: 14px; margin-bottom: 18px;">
      <div style="width: 44px; height: 44px; border-radius: 8px; background: rgba(255, 68, 68, 0.15); border: 1px solid #ff4444; display: flex; align-items: center; justify-content: center; font-size: 24px; flex-shrink: 0;">
        ⚠️
      </div>
      <div>
        <h2 style="margin: 0; font-size: 20px; font-weight: 700; letter-spacing: -0.02em; color: #fff;">
          WebGL Context Creation Failed
        </h2>
        <p style="margin: 3px 0 0; font-size: 13px; color: #8b949e; font-weight: 500;">
          ${appName} requires WebGL 2 hardware acceleration
        </p>
      </div>
    </div>

    <div style="background: rgba(255, 68, 68, 0.08); border-left: 3px solid #ff4444; padding: 12px 14px; border-radius: 4px; margin-bottom: 20px; font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace; font-size: 12px; color: #ff8b8b; word-break: break-word;">
      <strong>Error:</strong> ${reasonText}
    </div>

    <div style="margin-bottom: 20px; font-size: 14px; line-height: 1.6; color: #c9d1d9;">
      Your browser or graphics driver rejected WebGL initialization. Follow the quick steps below for your browser to enable it:
    </div>

    <div style="display: grid; gap: 16px; margin-bottom: 24px;">
      <!-- Firefox Fix -->
      <div style="background: #162232; border: 1px solid #23354c; border-radius: 8px; padding: 14px 16px;">
        <div style="font-weight: 700; font-size: 14px; color: #58a6ff; margin-bottom: 8px; display: flex; align-items: center; gap: 8px;">
          <span>🦊</span> If you are using Mozilla Firefox:
        </div>
        <ol style="margin: 0; padding-left: 20px; font-size: 13px; line-height: 1.65; color: #8b949e;">
          <li>Open a new tab and navigate to <code style="background: #090e15; padding: 2px 6px; border-radius: 4px; color: #79c0ff;">about:config</code></li>
          <li>Click <em>"Accept the Risk and Continue"</em></li>
          <li>Search for <code style="background: #090e15; padding: 2px 6px; border-radius: 4px; color: #79c0ff;">webgl.force-enabled</code> and double-click to set it to <strong style="color: #7ee787;">true</strong></li>
          <li>Search for <code style="background: #090e15; padding: 2px 6px; border-radius: 4px; color: #79c0ff;">webgl.disabled</code> and ensure it is <strong style="color: #7ee787;">false</strong></li>
          <li>Search for <code style="background: #090e15; padding: 2px 6px; border-radius: 4px; color: #79c0ff;">layers.acceleration.force-enabled</code> and set it to <strong style="color: #7ee787;">true</strong></li>
        </ol>
      </div>

      <!-- Chrome / Edge Fix -->
      <div style="background: #162232; border: 1px solid #23354c; border-radius: 8px; padding: 14px 16px;">
        <div style="font-weight: 700; font-size: 14px; color: #58a6ff; margin-bottom: 8px; display: flex; align-items: center; gap: 8px;">
          <span>🌐</span> If you are using Google Chrome, Microsoft Edge, or Brave:
        </div>
        <ol style="margin: 0; padding-left: 20px; font-size: 13px; line-height: 1.65; color: #8b949e;">
          <li>Go to <code style="background: #090e15; padding: 2px 6px; border-radius: 4px; color: #79c0ff;">chrome://settings/system</code> (or edge://settings/system)</li>
          <li>Turn <strong>ON</strong> <em>"Use graphics acceleration when available"</em></li>
          <li>If still blocked, visit <code style="background: #090e15; padding: 2px 6px; border-radius: 4px; color: #79c0ff;">chrome://flags</code>, search for <em>"Override software rendering list"</em>, and set to <strong style="color: #7ee787;">Enabled</strong></li>
        </ol>
      </div>
    </div>

    <div style="display: flex; gap: 12px; flex-wrap: wrap;">
      <button id="webgl-retry-btn" style="flex: 1; min-width: 160px; padding: 10px 18px; font-size: 14px; font-weight: 600; color: #fff; background: #238636; border: 1px solid #2ea043; border-radius: 6px; cursor: pointer; transition: background 0.2s;">
        🔄 Retry / Reload
      </button>
      <button id="webgl-copy-btn" style="flex: 1; min-width: 160px; padding: 10px 18px; font-size: 14px; font-weight: 600; color: #c9d1d9; background: #21262d; border: 1px solid #30363d; border-radius: 6px; cursor: pointer; transition: background 0.2s;">
        📋 Copy Diagnostics
      </button>
    </div>
  `;

  overlay.appendChild(card);
  document.body.appendChild(overlay);

  const retryBtn = card.querySelector('#webgl-retry-btn');
  if (retryBtn) {
    retryBtn.addEventListener('click', () => {
      window.location.reload();
    });
  }

  const copyBtn = card.querySelector('#webgl-copy-btn');
  if (copyBtn) {
    copyBtn.addEventListener('click', async () => {
      const diag = {
        app: appName,
        statusReason: reasonText,
        userAgent: navigator.userAgent,
        pixelRatio: window.devicePixelRatio,
        screen: `${window.screen?.width}x${window.screen?.height}`,
        timestamp: new Date().toISOString()
      };
      try {
        await navigator.clipboard.writeText(JSON.stringify(diag, null, 2));
        copyBtn.textContent = '✓ Copied to Clipboard!';
        setTimeout(() => { copyBtn.textContent = '📋 Copy Diagnostics'; }, 3000);
      } catch {
        prompt('Copy system diagnostic report:', JSON.stringify(diag));
      }
    });
  }
}
