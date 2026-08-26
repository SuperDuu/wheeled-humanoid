/**
 * FOC Telemetry Studio OSS - Client Controller
 * Features:
 * - Truly Frozen Independent Waveform Pause (locks frozenWriteIndex snapshot)
 * - Per-Scope Independent Controls (Pause/Resume, Pan Left/Right, Live Reset, 100-Level Zoom)
 * - Direct Mouse Wheel Zoom on hover for ANY individual scope canvas
 * - Mouse Drag to smoothly pan waveform history backward/forward
 * - Per-scope vertical amplitude gain and Y offset controls
 * - Deep Ring Buffer (10,000 samples per channel)
 * - Dynamic Light / Dark theme palette
 */

// Oscilloscope with Frozen Buffer Pause, 100-Level Wheel Zoom & Drag Panning
class DeepOscilloscope {
  constructor(scopeId, canvasId, channels, options = {}) {
    this.scopeId = scopeId;
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas ? this.canvas.getContext('2d') : null;
    this.channels = channels; // [{ key, color, lineWidth, lineDash }]
    this.yMin = options.yMin !== undefined ? options.yMin : -10;
    this.yMax = options.yMax !== undefined ? options.yMax : 10;
    this.autoScale = options.autoScale !== undefined ? options.autoScale : true;
    this.symmetricZero = options.symmetricZero !== undefined ? options.symmetricZero : false;
    this.minSymmetricRange = options.minSymmetricRange !== undefined ? options.minSymmetricRange : 60;
    this.alwaysIncludeZero = options.alwaysIncludeZero !== undefined ? options.alwaysIncludeZero : false;
    this.unit = options.unit || '';

    // Independent state
    this.zoomLevel = options.zoomLevel || 25; // 1 to 100
    this.verticalGain = options.verticalGain || 1.0; // 0.25x to 4.00x visual amplitude
    this.verticalOffset = options.verticalOffset || 0.0; // -1.00 to 1.00 screen-height relative offset
    this.windowSize = 250;
    this.panOffset = 0; // 0 = live, >0 = history
    this.isPaused = false;
    this.frozenWriteIndex = 0; // Locked snapshot index when paused

    // Temporary zoom overlay feedback
    this.zoomOverlayTimer = 0;
    this.verticalOverlayTimer = 0;

    // Ring buffers for deep history
    this.buffers = {};
    this.channels.forEach(ch => {
      this.buffers[ch.key] = new Float32Array(this.maxHistory);
    });

    this.writeIndex = 0;
    this.totalSamples = 0;

    this.calculateWindowSize();
    this.updateVerticalControls();
    this.setupInteractions();
    this.resize();
    window.addEventListener('resize', () => this.resize());
  }

  calculateWindowSize() {
    // 100 Zoom Levels:
    // Level 1: 10 samples (Super Magnified Waveform) -> Level 100: 2000 samples (Wide Overview)
    const minSamples = 10;
    const maxSamples = 2000;
    this.windowSize = Math.round(minSamples + (this.zoomLevel - 1) * ((maxSamples - minSamples) / 99));

    // Update UI controls on this scope's header
    const slider = document.querySelector(`.scope-zoom-slider[data-scope="${this.scopeId}"]`);
    if (slider) slider.value = this.zoomLevel;

    const label = document.querySelector(`.scope-zoom-label[data-scope="${this.scopeId}"]`);
    if (label) label.innerText = `L${this.zoomLevel} (${this.windowSize}p)`;
  }

  setZoom(level) {
    this.zoomLevel = Math.max(1, Math.min(100, level));
    this.calculateWindowSize();
    this.zoomOverlayTimer = Date.now() + 1000; // Show zoom overlay on canvas for 1s
  }

  setVerticalGain(percent) {
    this.verticalGain = Math.max(0.25, Math.min(4.0, percent / 100));
    this.updateVerticalControls();
    this.verticalOverlayTimer = Date.now() + 1200;
  }

  setVerticalOffset(percent) {
    this.verticalOffset = Math.max(-1.0, Math.min(1.0, percent / 100));
    this.updateVerticalControls();
    this.verticalOverlayTimer = Date.now() + 1200;
  }

  resetVerticalView() {
    this.verticalGain = 1.0;
    this.verticalOffset = 0.0;
    this.updateVerticalControls();
    this.verticalOverlayTimer = Date.now() + 1200;
  }

  updateVerticalControls() {
    const gainPercent = Math.round(this.verticalGain * 100);
    const gainSlider = document.querySelector(`.scope-gain-slider[data-scope="${this.scopeId}"]`);
    if (gainSlider) gainSlider.value = gainPercent;

    const gainLabel = document.querySelector(`.scope-gain-label[data-scope="${this.scopeId}"]`);
    if (gainLabel) gainLabel.innerText = `${this.verticalGain.toFixed(2)}x`;

    const offsetPercent = Math.round(this.verticalOffset * 100);
    const offsetSlider = document.querySelector(`.scope-offset-slider[data-scope="${this.scopeId}"]`);
    if (offsetSlider) offsetSlider.value = offsetPercent;

    const offsetLabel = document.querySelector(`.scope-offset-label[data-scope="${this.scopeId}"]`);
    if (offsetLabel) {
      const sign = offsetPercent > 0 ? '+' : '';
      offsetLabel.innerText = `${sign}${offsetPercent}%`;
    }

    const resetBtn = document.querySelector(`.btn-y-reset[data-scope="${this.scopeId}"]`);
    if (resetBtn) {
      const isDefault = Math.abs(this.verticalGain - 1.0) < 0.001 && Math.abs(this.verticalOffset) < 0.001;
      resetBtn.classList.toggle('is-active', !isDefault);
    }
  }

  togglePause() {
    this.isPaused = !this.isPaused;
    if (this.isPaused) {
      // Freeze buffer reference at this exact moment
      this.frozenWriteIndex = this.writeIndex;
    } else {
      this.panOffset = 0;
    }
    this.updateControlsUI();
  }

  pause() {
    if (!this.isPaused) {
      this.isPaused = true;
      this.frozenWriteIndex = this.writeIndex;
      this.updateControlsUI();
    }
  }

  resume() {
    this.isPaused = false;
    this.panOffset = 0;
    this.updateControlsUI();
  }

  pan(delta) {
    if (!this.isPaused) {
      this.isPaused = true;
      this.frozenWriteIndex = this.writeIndex;
    }
    this.panOffset = Math.max(0, this.panOffset + delta);
    this.updateControlsUI();
  }

  resetToLive() {
    this.isPaused = false;
    this.panOffset = 0;
    this.updateControlsUI();
  }

  updateControlsUI() {
    const pauseBtn = document.querySelector(`.btn-pause-toggle[data-scope="${this.scopeId}"]`);
    if (pauseBtn) {
      if (this.isPaused || this.panOffset > 0) {
        pauseBtn.innerText = 'Resume';
        pauseBtn.classList.add('is-paused');
      } else {
        pauseBtn.innerText = 'Pause';
        pauseBtn.classList.remove('is-paused');
      }
    }
  }

  setupInteractions() {
    if (!this.canvas) return;

    // 1. MOUSE WHEEL ZOOM (Hover over canvas and roll wheel to zoom immediately!)
    this.canvas.addEventListener('wheel', (e) => {
      e.preventDefault();
      // Scroll Up (deltaY < 0) -> Zoom In (-level, fewer samples)
      // Scroll Down (deltaY > 0) -> Zoom Out (+level, more samples)
      const step = Math.abs(e.deltaY) > 50 ? 3 : 1;
      if (e.deltaY < 0) {
        this.setZoom(this.zoomLevel - step);
      } else if (e.deltaY > 0) {
        this.setZoom(this.zoomLevel + step);
      }
    }, { passive: false });

    // 2. MOUSE DRAG TO PAN HISTORY
    let isDragging = false;
    let dragStartX = 0;
    let dragStartOffset = 0;

    this.canvas.addEventListener('mousedown', (e) => {
      isDragging = true;
      dragStartX = e.clientX;
      dragStartOffset = this.panOffset;
      if (!this.isPaused) {
        this.isPaused = true;
        this.frozenWriteIndex = this.writeIndex;
      }
      this.updateControlsUI();
    });

    window.addEventListener('mousemove', (e) => {
      if (!isDragging) return;
      const dx = e.clientX - dragStartX;
      // Convert pixel movement to sample displacement
      const samplesPerPixel = this.windowSize / (this.width || 400);
      const deltaSamples = Math.round(dx * samplesPerPixel * 1.5);
      this.panOffset = Math.max(0, dragStartOffset + deltaSamples);
      this.updateControlsUI();
    });

    window.addEventListener('mouseup', () => {
      isDragging = false;
    });

    // Touch Support for mobile / touchpads
    let touchStartX = 0;
    this.canvas.addEventListener('touchstart', (e) => {
      if (e.touches.length === 1) {
        touchStartX = e.touches[0].clientX;
        dragStartOffset = this.panOffset;
        if (!this.isPaused) {
          this.isPaused = true;
          this.frozenWriteIndex = this.writeIndex;
        }
        this.updateControlsUI();
      }
    }, { passive: true });

    this.canvas.addEventListener('touchmove', (e) => {
      if (e.touches.length === 1) {
        const dx = e.touches[0].clientX - touchStartX;
        const delta = Math.round(dx * (this.windowSize / 400));
        this.panOffset = Math.max(0, dragStartOffset + delta);
        this.updateControlsUI();
      }
    }, { passive: true });
  }

  resize() {
    if (!this.canvas) return;
    const rect = this.canvas.getBoundingClientRect();
    this.width = this.canvas.width = rect.width * (window.devicePixelRatio || 1);
    this.height = this.canvas.height = rect.height * (window.devicePixelRatio || 1);
  }

  push(data) {
    this.channels.forEach(ch => {
      const val = data[ch.key] !== undefined ? data[ch.key] : 0;
      this.buffers[ch.key][this.writeIndex] = val;
    });
    this.writeIndex = (this.writeIndex + 1) % this.maxHistory;
    this.totalSamples++;

    // When running live, keep frozen reference aligned with live write index
    if (!this.isPaused && this.panOffset === 0) {
      this.frozenWriteIndex = this.writeIndex;
    }
  }

  render(isDarkTheme = true) {
    if (!this.ctx || this.totalSamples === 0) return;
    const ctx = this.ctx;
    const w = this.width;
    const h = this.height;

    // Palette based on theme
    const bgColor = isDarkTheme ? '#0b0e11' : '#ffffff';
    const gridColor = isDarkTheme ? '#252d36' : '#e1e7ee';
    const zeroColor = isDarkTheme ? '#586575' : '#9aa8b7';

    // Clear background
    ctx.fillStyle = bgColor;
    ctx.fillRect(0, 0, w, h);

    // Determine visible samples based on windowSize & available samples
    const availableSamples = Math.min(this.totalSamples, this.maxHistory);
    const viewSize = Math.min(this.windowSize, availableSamples);
    if (viewSize < 2) return;

    // Determine baseline write pointer:
    // If paused or looking back into history, use locked frozenWriteIndex
    // If running live, use active writeIndex
    const baseIndex = (this.isPaused || this.panOffset > 0) ? this.frozenWriteIndex : this.writeIndex;

    // Clamp pan offset
    const maxOffset = Math.max(0, availableSamples - viewSize);
    const clampedOffset = Math.min(this.panOffset, maxOffset);

    // Calculate start index in circular ring buffer
    let startIdx = (baseIndex - clampedOffset - viewSize) % this.maxHistory;
    if (startIdx < 0) startIdx += this.maxHistory;

    // Auto-scale calculation across visible window
    let min = this.yMin;
    let max = this.yMax;
    if (this.symmetricZero) {
      let absMax = 0;
      for (const ch of this.channels) {
        const buf = this.buffers[ch.key];
        for (let i = 0; i < viewSize; i++) {
          const idx = (startIdx + i) % this.maxHistory;
          const v = buf[idx];
          const a = Math.abs(v);
          if (a > absMax) absMax = a;
        }
      }
      const minRange = this.minSymmetricRange !== undefined ? this.minSymmetricRange : 60;
      let bound = Math.max(minRange, absMax * 1.25);
      if (bound <= 100) bound = Math.ceil(bound / 10) * 10;
      else if (bound <= 500) bound = Math.ceil(bound / 25) * 25;
      else bound = Math.ceil(bound / 50) * 50;

      min = -bound;
      max = bound;
    } else if (this.autoScale) {
      let localMin = Infinity;
      let localMax = -Infinity;
      for (const ch of this.channels) {
        const buf = this.buffers[ch.key];
        for (let i = 0; i < viewSize; i++) {
          const idx = (startIdx + i) % this.maxHistory;
          const v = buf[idx];
          if (v < localMin) localMin = v;
          if (v > localMax) localMax = v;
        }
      }
      if (localMin !== Infinity && localMax !== -Infinity) {
        if (this.alwaysIncludeZero) {
          if (localMin > 0) localMin = 0;
          if (localMax < 0) localMax = 0;
        }
        const margin = Math.max(0.3, (localMax - localMin) * 0.15);
        min = localMin - margin;
        max = localMax + margin;
      }
    }

    const baseRange = (max - min) || 1;
    const baseCenter = (min + max) / 2;
    const halfRange = Math.max(baseRange / 2 / this.verticalGain, 0.000001);
    const center = baseCenter - (this.verticalOffset * halfRange);
    min = center - halfRange;
    max = center + halfRange;

    const range = (max - min) || 1;

    // Draw Grid Lines (Horizontal & Vertical)
    ctx.strokeStyle = gridColor;
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let i = 1; i <= 4; i++) {
      const y = (h / 5) * i;
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
    }
    for (let i = 1; i <= 6; i++) {
      const x = (w / 7) * i;
      ctx.moveTo(x, 0);
      ctx.lineTo(x, h);
    }
    ctx.stroke();

    // Zero Reference Line
    if (min <= 0 && max >= 0) {
      const zeroY = h - ((0 - min) / range) * h;
      ctx.strokeStyle = isDarkTheme ? '#475569' : '#94a3b8';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([5, 4]);
      ctx.beginPath();
      ctx.moveTo(0, zeroY);
      ctx.lineTo(w, zeroY);
      ctx.stroke();
      ctx.setLineDash([]);

      // Zero text badge on right
      ctx.fillStyle = isDarkTheme ? 'rgba(148, 163, 184, 0.85)' : 'rgba(100, 116, 139, 0.85)';
      ctx.font = 'bold 10px monospace';
      ctx.textAlign = 'right';
      ctx.fillText('0' + (this.unit ? this.unit.trim() : ''), w - 10, Math.max(12, Math.min(h - 6, zeroY - 4)));
    }

    // Top and bottom boundary scale labels
    ctx.fillStyle = isDarkTheme ? 'rgba(148, 163, 184, 0.65)' : 'rgba(100, 116, 139, 0.75)';
    ctx.font = '10px monospace';
    ctx.textAlign = 'right';
    const fmtVal = (val) => Math.abs(val) >= 10 ? val.toFixed(0) : val.toFixed(1);
    ctx.fillText(`+${fmtVal(max)}${this.unit || ''}`, w - 10, 14);
    ctx.fillText(`${fmtVal(min)}${this.unit || ''}`, w - 10, h - 6);
    ctx.textAlign = 'start';

    // Plot Channels
    const showPoints = viewSize <= 60; // Show discrete sample point dots when zoomed in
    this.channels.forEach(ch => {
      const buf = this.buffers[ch.key];
      ctx.strokeStyle = ch.color;
      ctx.lineWidth = (ch.lineWidth || 2) * (window.devicePixelRatio || 1);
      if (ch.lineDash) ctx.setLineDash(ch.lineDash);
      else ctx.setLineDash([]);

      ctx.beginPath();
      for (let i = 0; i < viewSize; i++) {
        const sampleIdx = (startIdx + i) % this.maxHistory;
        const val = buf[sampleIdx];
        const x = (i / (viewSize - 1)) * w;
        const y = h - ((val - min) / range) * h;

        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
      ctx.setLineDash([]);

      // Draw point markers when zoomed in close
      if (showPoints) {
        ctx.fillStyle = ch.color;
        for (let i = 0; i < viewSize; i++) {
          const sampleIdx = (startIdx + i) % this.maxHistory;
          const val = buf[sampleIdx];
          const x = (i / (viewSize - 1)) * w;
          const y = h - ((val - min) / range) * h;
          ctx.beginPath();
          ctx.arc(x, y, 2.5 * (window.devicePixelRatio || 1), 0, Math.PI * 2);
          ctx.fill();
        }
      }
    });

    // Paused / History Status Badge Overlay on Canvas
    if (this.isPaused || clampedOffset > 0) {
      ctx.fillStyle = 'rgba(234, 179, 8, 0.18)';
      ctx.fillRect(8, 8, 130, 20);
      ctx.strokeStyle = '#eab308';
      ctx.lineWidth = 1;
      ctx.strokeRect(8, 8, 130, 20);
      ctx.fillStyle = '#eab308';
      ctx.font = 'bold 10px monospace';
      ctx.fillText(clampedOffset > 0 ? `PAN: -${clampedOffset} pts` : 'PAUSED (FROZEN)', 14, 22);
    }

    // Zoom level transient tooltip overlay (shown when rolling wheel)
    if (Date.now() < this.zoomOverlayTimer) {
      ctx.fillStyle = 'rgba(59, 130, 246, 0.9)';
      ctx.fillRect(w / 2 - 80, 8, 160, 24);
      ctx.fillStyle = '#ffffff';
      ctx.font = 'bold 11px monospace';
      ctx.textAlign = 'center';
      ctx.fillText(`ZOOM: L${this.zoomLevel}/100 (${this.windowSize}p)`, w / 2, 24);
      ctx.textAlign = 'start';
    }

    // Vertical controls feedback overlay
    if (Date.now() < this.verticalOverlayTimer || Math.abs(this.verticalGain - 1.0) > 0.001 || Math.abs(this.verticalOffset) > 0.001) {
      const labelY = (this.isPaused || clampedOffset > 0) ? 36 : 8;
      const offsetPercent = Math.round(this.verticalOffset * 100);
      const sign = offsetPercent > 0 ? '+' : '';
      ctx.fillStyle = isDarkTheme ? 'rgba(11, 14, 17, 0.78)' : 'rgba(255, 255, 255, 0.88)';
      ctx.fillRect(8, labelY, 156, 22);
      ctx.strokeStyle = isDarkTheme ? '#45515f' : '#b8c2ce';
      ctx.strokeRect(8, labelY, 156, 22);
      ctx.fillStyle = isDarkTheme ? '#4ea1ff' : '#185abc';
      ctx.font = 'bold 10px monospace';
      ctx.fillText(`AMP ${this.verticalGain.toFixed(2)}x  Y ${sign}${offsetPercent}%`, 14, labelY + 15);
    }
  }
}

// Master Application Controller
const App = {
  ws: null,
  connected: false,
  isSimulation: false,
  isDarkTheme: true,
  scopes: {},
  lastLogTime: 0,
  recordedSamples: 0,
  isRecording: false,

  init() {
    this.loadThemePreference();
    this.setupOscilloscopes();
    this.setupEventListeners();
    this.setupPerScopeButtonHandlers();
    this.fetchPorts();
    this.initWebSocket();
    this.startAnimationLoop();
  },

  loadThemePreference() {
    const savedTheme = localStorage.getItem('foc_theme') || 'dark';
    this.setTheme(savedTheme);
  },

  setTheme(theme) {
    this.isDarkTheme = theme === 'dark';
    document.documentElement.setAttribute('data-theme', theme);
    document.documentElement.setAttribute('data-bs-theme', theme);
    localStorage.setItem('foc_theme', theme);

    const btn = document.getElementById('btn-theme-toggle');
    if (btn) {
      const label = btn.querySelector('.theme-toggle-text');
      if (label) {
        label.innerText = this.isDarkTheme ? 'Dark' : 'Light';
      } else {
        btn.innerText = this.isDarkTheme ? 'Dark' : 'Light';
      }
      btn.setAttribute('aria-label', this.isDarkTheme ? 'Switch to light theme' : 'Switch to dark theme');
    }
  },

  toggleTheme() {
    this.setTheme(this.isDarkTheme ? 'light' : 'dark');
  },

  setupOscilloscopes() {
    // Scope 1: 3-Phase Sinusoids
    this.scopes.phase = new DeepOscilloscope('phase', 'canvas-scope-phase', [
      { key: 'i_a', color: '#e5484d', lineWidth: 2 },
      { key: 'i_b', color: '#2f9e44', lineWidth: 2 },
      { key: 'i_c', color: '#1c7ed6', lineWidth: 2 }
    ], { symmetricZero: true, minSymmetricRange: 2.0, unit: ' A', zoomLevel: 25 });

    // Scope 2: Space Vector D-Q
    this.scopes.dq = new DeepOscilloscope('dq', 'canvas-scope-dq', [
      { key: 'i_d', color: '#d97706', lineWidth: 2 },
      { key: 'i_q', color: '#7c3aed', lineWidth: 2 },
      { key: 'i_q_target', color: '#0891b2', lineWidth: 1.5, lineDash: [4, 4] }
    ], { symmetricZero: true, minSymmetricRange: 2.0, unit: ' A', zoomLevel: 25 });

    // Scope 3: Electrical & Mechanical Angle
    this.scopes.angle = new DeepOscilloscope('angle', 'canvas-scope-angle', [
      { key: 'phase_elec', color: '#db2777', lineWidth: 2 },
      { key: 'mech_angle', color: '#5b6ee1', lineWidth: 1.5 }
    ], { autoScale: false, yMin: -3.5, yMax: 3.5, unit: ' rad', zoomLevel: 25 });

    // Scope 4: Speed Response (RPM) - Centered at 0, displaying both positive and negative speeds
    this.scopes.speed = new DeepOscilloscope('speed', 'canvas-scope-speed', [
      { key: 'speed_rpm', color: '#0f9f6e', lineWidth: 2 },
      { key: 'speed_target_rpm', color: '#0891b2', lineWidth: 1.5, lineDash: [4, 4] }
    ], {
      symmetricZero: true,
      minSymmetricRange: 60, // Minimum span: -60 RPM to +60 RPM anchored at 0 baseline
      unit: ' RPM',
      zoomLevel: 25
    });
  },

  setupPerScopeButtonHandlers() {
    // Pause / Resume Toggle for each scope
    document.querySelectorAll('.btn-pause-toggle').forEach(btn => {
      btn.addEventListener('click', (e) => {
        const scopeKey = e.target.dataset.scope;
        if (this.scopes[scopeKey]) {
          this.scopes[scopeKey].togglePause();
        }
      });
    });

    // Pan Left for each scope
    document.querySelectorAll('.btn-pan-left').forEach(btn => {
      btn.addEventListener('click', (e) => {
        const scopeKey = e.target.dataset.scope;
        if (this.scopes[scopeKey]) {
          this.scopes[scopeKey].pan(50);
        }
      });
    });

    // Pan Right for each scope
    document.querySelectorAll('.btn-pan-right').forEach(btn => {
      btn.addEventListener('click', (e) => {
        const scopeKey = e.target.dataset.scope;
        if (this.scopes[scopeKey]) {
          this.scopes[scopeKey].pan(-50);
        }
      });
    });

    // Live button for each scope
    document.querySelectorAll('.btn-pan-live').forEach(btn => {
      btn.addEventListener('click', (e) => {
        const scopeKey = e.target.dataset.scope;
        if (this.scopes[scopeKey]) {
          this.scopes[scopeKey].resetToLive();
        }
      });
    });

    // Zoom slider for each scope (1 to 100 levels)
    document.querySelectorAll('.scope-zoom-slider').forEach(slider => {
      slider.addEventListener('input', (e) => {
        const scopeKey = e.target.dataset.scope;
        if (this.scopes[scopeKey]) {
          this.scopes[scopeKey].setZoom(parseInt(e.target.value));
        }
      });
    });

    // Vertical amplitude gain for each scope
    document.querySelectorAll('.scope-gain-slider').forEach(slider => {
      slider.addEventListener('input', (e) => {
        const scopeKey = e.target.dataset.scope;
        if (this.scopes[scopeKey]) {
          this.scopes[scopeKey].setVerticalGain(parseInt(e.target.value));
        }
      });
    });

    // Vertical offset for each scope (+ moves trace up, - moves trace down)
    document.querySelectorAll('.scope-offset-slider').forEach(slider => {
      slider.addEventListener('input', (e) => {
        const scopeKey = e.target.dataset.scope;
        if (this.scopes[scopeKey]) {
          this.scopes[scopeKey].setVerticalOffset(parseInt(e.target.value));
        }
      });
    });

    // Reset vertical gain and offset for each scope
    document.querySelectorAll('.btn-y-reset').forEach(btn => {
      btn.addEventListener('click', (e) => {
        const scopeKey = e.target.dataset.scope;
        if (this.scopes[scopeKey]) {
          this.scopes[scopeKey].resetVerticalView();
        }
      });
    });

    // Scale Range selector dropdown for each scope
    document.querySelectorAll('.scope-range-select').forEach(sel => {
      sel.addEventListener('change', (e) => {
        const scopeKey = e.target.dataset.scope;
        const val = e.target.value;
        if (this.scopes[scopeKey]) {
          if (val === 'auto_sym') {
            this.scopes[scopeKey].symmetricZero = true;
            this.scopes[scopeKey].autoScale = true;
            this.scopes[scopeKey].minSymmetricRange = 60;
          } else {
            const num = parseFloat(val);
            if (!isNaN(num)) {
              this.scopes[scopeKey].symmetricZero = false;
              this.scopes[scopeKey].autoScale = false;
              this.scopes[scopeKey].yMin = -num;
              this.scopes[scopeKey].yMax = num;
            }
          }
        }
      });
    });

    // Master Pause All / Resume All
    const masterPauseBtn = document.getElementById('btn-master-pause');
    if (masterPauseBtn) {
      masterPauseBtn.addEventListener('click', () => {
        const anyRunning = Object.values(this.scopes).some(s => !s.isPaused);
        Object.values(this.scopes).forEach(s => {
          if (anyRunning) {
            s.pause();
          } else {
            s.resume();
          }
        });
        masterPauseBtn.innerText = anyRunning ? 'Resume all' : 'Pause all';
        masterPauseBtn.classList.toggle('success-outline', anyRunning);
      });
    }
  },

  setupEventListeners() {
    // Theme Toggle
    document.getElementById('btn-theme-toggle').addEventListener('click', () => this.toggleTheme());

    // Refresh ports
    document.getElementById('btn-refresh-ports').addEventListener('click', () => this.fetchPorts());

    // Connect button
    document.getElementById('btn-connect').addEventListener('click', () => this.toggleConnect());

    // Mode Buttons
    document.querySelectorAll('.btn-mode').forEach(btn => {
      btn.addEventListener('click', (e) => {
        const mode = parseInt(e.target.dataset.mode);
        this.setMotorMode(mode);
      });
    });

    // Speed Slider
    const slider = document.getElementById('speed-slider');
    slider.addEventListener('input', (e) => {
      document.getElementById('slider-val-text').innerText = e.target.value;
    });
    slider.addEventListener('change', (e) => {
      this.sendSpeedCommand(parseFloat(e.target.value));
    });

    // Emergency Stop
    document.getElementById('btn-estop').addEventListener('click', () => this.emergencyStop());

    // Calibration & Align Buttons (MIT Mini Cheetah)
    const btnCalib = document.getElementById('btn-calib-lut');
    if (btnCalib) {
      btnCalib.addEventListener('click', () => this.sendCustomCommand('CALIB'));
    }
    const btnAlign = document.getElementById('btn-align-zero');
    if (btnAlign) {
      btnAlign.addEventListener('click', () => this.sendCustomCommand('ALIGN'));
    }

    // Recording Controls
    document.getElementById('btn-record-toggle').addEventListener('click', () => this.toggleRecording());
    document.getElementById('btn-download-csv').addEventListener('click', () => this.downloadCsv());

    this.setupDriverFeatureHandlers();

    // Copy Log
    document.getElementById('btn-copy-log')?.addEventListener('click', () => this.copyLogToClipboard());

    // Clear Log
    document.getElementById('btn-clear-log').addEventListener('click', () => {
      document.getElementById('log-terminal').innerHTML = '<div class="log-line system">[SYSTEM] Log cleared.</div>';
    });
  },

  setupDriverFeatureHandlers() {
    document.querySelectorAll('[data-cmd]').forEach(btn => {
      btn.addEventListener('click', () => {
        const cmd = btn.getAttribute('data-cmd');
        if (cmd) this.sendCustomCommand(cmd);
      });
    });

    const cliInput = document.getElementById('input-cli');
    const sendCli = () => {
      if (!cliInput) return;
      const cmd = cliInput.value.trim();
      if (!cmd) return;
      this.sendCustomCommand(cmd);
      cliInput.value = '';
    };
    document.getElementById('btn-send-cli')?.addEventListener('click', sendCli);
    cliInput?.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') {
        e.preventDefault();
        sendCli();
      }
    });

    const setSpeed = (val) => {
      const rpm = parseFloat(val) || 0;
      document.getElementById('input-speed').value = rpm;
      document.getElementById('slider-speed').value = rpm;
      document.getElementById('speed-display').innerText = `${rpm} RPM`;
      document.getElementById('speed-slider').value = rpm;
      document.getElementById('slider-val-text').innerText = rpm;
      this.sendCustomCommand(`SPEED ${rpm}`);
      this.markModeActive(3);
    };
    document.getElementById('btn-set-speed')?.addEventListener('click', () => setSpeed(document.getElementById('input-speed').value));
    document.getElementById('input-speed')?.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') setSpeed(e.target.value);
    });
    document.getElementById('slider-speed')?.addEventListener('input', (e) => {
      document.getElementById('speed-display').innerText = `${e.target.value} RPM`;
      document.getElementById('input-speed').value = e.target.value;
    });
    document.getElementById('slider-speed')?.addEventListener('change', (e) => setSpeed(e.target.value));
    document.querySelectorAll('.btn-preset').forEach(btn => {
      btn.addEventListener('click', () => setSpeed(btn.dataset.speed));
    });

    const runOpenLoop = (rpm, volt) => {
      const targetRpm = parseFloat(rpm) || 0;
      const targetVolt = parseFloat(volt) || 0;
      document.getElementById('input-openloop-rpm').value = targetRpm;
      document.getElementById('input-openloop-volt').value = targetVolt;
      document.getElementById('slider-openloop').value = targetRpm;
      document.getElementById('openloop-display').innerText = `${targetRpm} RPM @ ${targetVolt}V`;
      this.sendCustomCommand(Math.abs(targetRpm) < 0.1 ? 'STOP' : `OPENLOOP ${targetRpm} ${targetVolt}`);
    };
    document.getElementById('btn-run-openloop')?.addEventListener('click', () => {
      runOpenLoop(document.getElementById('input-openloop-rpm').value, document.getElementById('input-openloop-volt').value);
    });
    document.getElementById('slider-openloop')?.addEventListener('input', (e) => {
      const volt = document.getElementById('input-openloop-volt').value;
      document.getElementById('input-openloop-rpm').value = e.target.value;
      document.getElementById('openloop-display').innerText = `${e.target.value} RPM @ ${volt}V`;
    });
    document.getElementById('slider-openloop')?.addEventListener('change', (e) => {
      runOpenLoop(e.target.value, document.getElementById('input-openloop-volt').value);
    });
    document.querySelectorAll('.btn-open-preset').forEach(btn => {
      btn.addEventListener('click', () => runOpenLoop(btn.dataset.rpm, btn.dataset.volt || '9.0'));
    });

    const setIq = (val) => {
      const iq = parseFloat(val) || 0;
      document.getElementById('input-iq').value = iq.toFixed(2);
      document.getElementById('slider-iq').value = iq;
      document.getElementById('iq-display').innerText = `${iq.toFixed(2)} A`;
      this.sendCustomCommand(`IQ ${iq.toFixed(2)}`);
      this.markModeActive(1);
    };
    document.getElementById('btn-set-iq')?.addEventListener('click', () => setIq(document.getElementById('input-iq').value));
    document.getElementById('input-iq')?.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') setIq(e.target.value);
    });
    document.getElementById('slider-iq')?.addEventListener('input', (e) => {
      const iq = parseFloat(e.target.value) || 0;
      document.getElementById('input-iq').value = iq.toFixed(2);
      document.getElementById('iq-display').innerText = `${iq.toFixed(2)} A`;
    });
    document.getElementById('slider-iq')?.addEventListener('change', (e) => setIq(e.target.value));
    document.querySelectorAll('.btn-iq-preset').forEach(btn => {
      btn.addEventListener('click', () => setIq(btn.dataset.iq));
    });

    const setPosition = (val) => {
      const deg = parseFloat(val) || 0;
      const rad = deg * Math.PI / 180;
      document.getElementById('input-pos').value = deg;
      document.getElementById('pos-display').innerText = `${deg.toFixed(1)} deg (${rad.toFixed(2)} rad)`;
      this.sendCustomCommand(`POS ${deg}`);
      this.markModeActive(4);
    };
    document.getElementById('btn-set-pos')?.addEventListener('click', () => setPosition(document.getElementById('input-pos').value));
    document.getElementById('input-pos')?.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') setPosition(e.target.value);
    });
    document.querySelectorAll('.btn-pos-preset').forEach(btn => {
      btn.addEventListener('click', () => setPosition(btn.dataset.deg));
    });

    document.getElementById('btn-quick-sethome')?.addEventListener('click', () => this.sendCustomCommand('SETHOME'));
    document.getElementById('btn-quick-gohome')?.addEventListener('click', () => this.sendCustomCommand('GOHOME 2.0'));

    document.getElementById('btn-apply-speed-pid')?.addEventListener('click', () => {
      const kp = parseFloat(document.getElementById('tune-skp').value || '0');
      const ki = parseFloat(document.getElementById('tune-ski').value || '0');
      const ramp = parseFloat(document.getElementById('tune-sramp').value || '0');
      this.sendCustomCommand(`SET_SPEED_PID ${kp} ${ki} ${ramp}`);
    });
    document.querySelectorAll('.btn-ramp-preset').forEach(btn => {
      btn.addEventListener('click', () => {
        const ramp = btn.dataset.ramp;
        document.getElementById('tune-sramp').value = ramp;
        this.sendCustomCommand(`RAMP ${ramp}`);
      });
    });
    document.getElementById('btn-apply-pos-pid')?.addEventListener('click', () => {
      const kp = parseFloat(document.getElementById('tune-pkp').value || '0');
      const kd = parseFloat(document.getElementById('tune-pkd').value || '0');
      this.sendCustomCommand(`SET_POS_PID ${kp} ${kd}`);
    });

    this.currentDir = 1;
    this.currentSwap = false;
    this.currentOffset = 0;
    document.getElementById('btn-toggle-dir')?.addEventListener('click', () => {
      this.currentDir = this.currentDir === 1 ? -1 : 1;
      document.getElementById('display-dir').innerText = this.currentDir > 0 ? '+1' : '-1';
      this.sendCustomCommand(`DIR ${this.currentDir}`);
    });
    document.getElementById('btn-toggle-swap')?.addEventListener('click', () => {
      this.currentSwap = !this.currentSwap;
      document.getElementById('display-swap').innerText = this.currentSwap ? 'ON' : 'OFF';
      this.sendCustomCommand(`SWAP ${this.currentSwap ? 1 : 0}`);
    });
    document.querySelectorAll('.btn-offset-nudge').forEach(btn => {
      btn.addEventListener('click', () => {
        this.currentOffset += parseFloat(btn.dataset.nudge || '0');
        while (this.currentOffset > Math.PI) this.currentOffset -= 2 * Math.PI;
        while (this.currentOffset < -Math.PI) this.currentOffset += 2 * Math.PI;
        document.getElementById('display-offset').innerText = `${this.currentOffset.toFixed(2)} rad`;
        this.sendCustomCommand(`OFFSET ${this.currentOffset.toFixed(3)}`);
      });
    });

    document.getElementById('btn-auto-tune')?.addEventListener('click', async () => this.runAutoTune());
  },

  async fetchPorts() {
    const select = document.getElementById('port-select');
    select.innerHTML = '<option value="">Scanning ports...</option>';
    try {
      const res = await fetch('/api/ports');
      const ports = await res.json();
      select.innerHTML = '';
      ports.forEach(p => {
        const opt = document.createElement('option');
        opt.value = p.device;
        opt.innerText = (p.description || p.device).replace(/[\u26a1\u{1f3af}]/gu, '').trim();
        select.appendChild(opt);
      });
      const preferred = ports.find(p => p.device !== 'SIMULATION');
      if (preferred) {
        select.value = preferred.device;
      }
    } catch (e) {
      select.innerHTML = '<option value="">Error scanning ports</option>';
    }
  },

  async toggleConnect() {
    const btn = document.getElementById('btn-connect');
    const port = document.getElementById('port-select').value;
    const baud = parseInt(document.getElementById('baud-select').value);

    if (this.connected) {
      try {
        await fetch('/api/disconnect', { method: 'POST' });
        this.updateConnectionStatus(false);
      } catch (e) {
        this.log(`Disconnect error: ${e.message}`, 'error');
      }
    } else {
      if (!port) {
        alert('Please select a valid USB port or Simulation Mode.');
        return;
      }
      try {
        btn.disabled = true;
        const res = await fetch('/api/connect', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ port, baudrate: baud })
        });
        const data = await res.json();
        if (res.ok) {
          this.updateConnectionStatus(true, data.is_simulation, port);
        } else {
          alert(`Connection failed: ${data.detail}`);
        }
      } catch (e) {
        alert(`Connection error: ${e.message}`);
      } finally {
        btn.disabled = false;
      }
    }
  },

  updateConnectionStatus(connected, isSim = false, portName = '') {
    this.connected = connected;
    this.isSimulation = isSim;
    const dot = document.getElementById('conn-dot');
    const text = document.getElementById('conn-text');
    const btn = document.getElementById('btn-connect');

    dot.className = 'status-dot';
    if (connected) {
      dot.classList.add(isSim ? 'simulating' : 'connected');
      text.innerText = isSim ? 'Simulating' : `Connected (${portName})`;
      btn.innerText = 'Disconnect';
      btn.classList.add('is-disconnect');
      this.log(`Connected successfully to ${portName}`, 'system');
    } else {
      text.innerText = 'Disconnected';
      btn.innerText = 'Connect';
      btn.classList.remove('is-disconnect');
      document.getElementById('telemetry-hz').innerText = '0 Hz';
      this.log('Disconnected from device.', 'system');
    }
  },

  markModeActive(mode) {
    document.querySelectorAll('.btn-mode').forEach(b => {
      b.classList.toggle('active', parseInt(b.dataset.mode) === mode);
    });
    const modeNames = ['IDLE (0)', 'CURRENT (1)', 'BRAKE (2)', 'SPEED (3)', 'POSITION (4)', 'VOLTAGE (5)'];
    const badge = document.getElementById('active-mode-badge');
    if (badge) badge.innerText = modeNames[mode] || `MODE ${mode}`;
    const modeText = document.getElementById('val-mode');
    if (modeText) modeText.innerText = modeNames[mode] || `MODE ${mode}`;
  },

  async setMotorMode(mode) {
    const modeNames = ['IDLE (0)', 'CURRENT (1)', 'BRAKE (2)', 'SPEED (3)', 'POSITION (4)'];
    this.markModeActive(mode);

    const targetVal = parseFloat(document.getElementById('speed-slider').value);
    try {
      await fetch('/api/control', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ control_mode: mode, target_value: targetVal })
      });
      this.log(`Motor Mode changed to ${modeNames[mode]}`, 'system');
    } catch (e) {
      this.log(`Error setting mode: ${e.message}`, 'error');
    }
  },

  async sendSpeedCommand(rpm) {
    try {
      await this.sendCustomCommand(`SPEED ${rpm}`);
    } catch (e) {
      this.log(`Error sending speed: ${e.message}`, 'error');
    }
  },

  async emergencyStop() {
    try {
      await this.sendCustomCommand('STOP');
      this.markModeActive(0);
      const speedSlider = document.getElementById('speed-slider');
      const speedText = document.getElementById('slider-val-text');
      if (speedSlider) speedSlider.value = 0;
      if (speedText) speedText.innerText = '0';
      this.log('EMERGENCY STOP ACTIVATED! Motor set to IDLE.', 'error');
    } catch (e) {
      this.log(`E-Stop failed: ${e.message}`, 'error');
    }
  },

  async sendCustomCommand(cmd) {
    this.log(`Sending command: ${cmd}...`, 'system');
    try {
      const res = await fetch('/api/command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ command: cmd })
      });
      const data = await res.json();
      if (res.ok && data.success !== false) {
        this.log(`Command [${cmd}] executed successfully.`, 'system');
        return true;
      } else {
        this.log(`Command [${cmd}] failed: ${data.detail || data.message || 'no acknowledgement'}`, 'error');
      }
    } catch (e) {
      this.log(`Error sending [${cmd}]: ${e.message}`, 'error');
    }
    return false;
  },

  async runAutoTune() {
    const btn = document.getElementById('btn-auto-tune');
    if (!btn) return;
    if (!confirm('Run motor auto-tune? The motor may move during calibration.')) return;
    btn.disabled = true;
    const oldText = btn.innerText;
    btn.innerText = 'Tuning...';
    this.log('Auto-tune started. Keep the motor clear.', 'warn');
    try {
      const res = await fetch('/api/auto_tune', { method: 'POST' });
      const data = await res.json();
      if (data.success) {
        const profile = data.profile || {};
        if (profile.speed_pi) {
          document.getElementById('tune-skp').value = profile.speed_pi.kp?.toFixed?.(5) || profile.speed_pi.kp || '';
          document.getElementById('tune-ski').value = profile.speed_pi.ki?.toFixed?.(5) || profile.speed_pi.ki || '';
          document.getElementById('tune-sramp').value = profile.speed_pi.ramp || document.getElementById('tune-sramp').value;
        }
        if (profile.pos_pd) {
          document.getElementById('tune-pkp').value = profile.pos_pd.kp?.toFixed?.(2) || profile.pos_pd.kp || '';
          document.getElementById('tune-pkd').value = profile.pos_pd.kd?.toFixed?.(3) || profile.pos_pd.kd || '';
        }
        this.log('Auto-tune completed and tuning fields were updated.', 'system');
      } else {
        this.log(`Auto-tune failed: ${data.message || 'unknown error'}`, 'error');
      }
    } catch (e) {
      this.log(`Auto-tune error: ${e.message}`, 'error');
    } finally {
      btn.disabled = false;
      btn.innerText = oldText;
    }
  },

  async toggleRecording() {
    const btn = document.getElementById('btn-record-toggle');
    const dlBtn = document.getElementById('btn-download-csv');
    const indicator = document.getElementById('recording-indicator');

    if (!this.isRecording) {
      await fetch('/api/record/start', { method: 'POST' });
      this.isRecording = true;
      this.recordedSamples = 0;
      document.getElementById('recorded-count').innerText = this.recordedSamples;
      btn.innerText = 'Stop recording';
      btn.classList.add('is-recording');
      dlBtn.disabled = true;
      indicator.classList.remove('d-none');
      this.log('Telemetry recording started (Pandas DataFrame buffer active).', 'warn');
    } else {
      const res = await fetch('/api/record/stop', { method: 'POST' });
      const data = await res.json();
      this.isRecording = false;
      btn.innerText = 'Start recording';
      btn.classList.remove('is-recording');
      dlBtn.disabled = false;
      indicator.classList.add('d-none');
      this.log(`Recording stopped. Captured ${data.total_samples} samples. Ready to export CSV.`, 'system');
    }
  },

  downloadCsv() {
    window.location.href = '/api/record/export';
  },

  async copyLogToClipboard() {
    const term = document.getElementById('log-terminal');
    const btn = document.getElementById('btn-copy-log');
    if (!term || !btn) return;

    const text = term.innerText.trim();
    if (!text) return;

    try {
      btn.disabled = true;
      await this.copyTextToClipboard(text);

      const oldText = btn.innerText;
      btn.innerText = 'Copied';
      setTimeout(() => { btn.innerText = oldText; }, 1000);
    } catch (e) {
      this.log(`Copy log failed: ${e.message}`, 'error');
    } finally {
      btn.disabled = false;
    }
  },

  async copyTextToClipboard(text) {
    let clipboardError = null;

    if (navigator.clipboard && window.isSecureContext) {
      try {
        await navigator.clipboard.writeText(text);
        return;
      } catch (e) {
        clipboardError = e;
      }
    }

    const ta = document.createElement('textarea');
    ta.value = text;
    ta.style.position = 'fixed';
    ta.style.top = '-1000px';
    ta.style.left = '-1000px';
    ta.style.width = '1px';
    ta.style.height = '1px';
    ta.style.whiteSpace = 'pre';
    ta.setAttribute('aria-hidden', 'true');

    const active = document.activeElement;
    document.body.appendChild(ta);
    ta.focus({ preventScroll: true });
    ta.select();
    ta.setSelectionRange(0, ta.value.length);

    let copied = false;
    try {
      copied = document.execCommand('copy');
    } catch (e) {
      copied = false;
      clipboardError = clipboardError || e;
    } finally {
      document.body.removeChild(ta);
      if (active && typeof active.focus === 'function') {
        active.focus({ preventScroll: true });
      }
    }

    if (copied) return;

    if (clipboardError) throw clipboardError;

    throw new Error('Clipboard copy is not available in this browser context.');
  },

  initWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws/telemetry`;
    this.ws = new WebSocket(wsUrl);

    this.ws.onopen = () => {
      console.log('WebSocket stream connected.');
    };

    this.ws.onmessage = (event) => {
      try {
        const frame = JSON.parse(event.data);
        this.handleTelemetryFrame(frame);
      } catch (e) {
        console.error('Error parsing frame', e);
      }
    };

    this.ws.onclose = () => {
      setTimeout(() => this.initWebSocket(), 2000);
    };
  },

  handleTelemetryFrame(data) {
    // Push incoming live telemetry into oscilloscope circular buffers
    this.scopes.phase.push(data);
    this.scopes.dq.push(data);
    this.scopes.angle.push(data);
    this.scopes.speed.push(data);

    // Update Space Vector polar plot
    this.drawSpaceVector(data.i_d, data.i_q, data.phase_elec, data.i_vector_mag);

    // Update Overview Metric Cards
    document.getElementById('val-vbus').innerText = data.v_bus.toFixed(1);
    document.getElementById('val-iq').innerText = data.i_q.toFixed(2);
    document.getElementById('val-ipeak').innerText = data.i_peak.toFixed(2);
    document.getElementById('val-rpm').innerText = Math.round(data.speed_rpm);
    document.getElementById('val-power').innerText = data.power_watts.toFixed(1);
    document.getElementById('val-temp').innerText = data.temp_fet.toFixed(1);

    // Update Badges on scopes
    document.getElementById('val-ia').innerText = `${data.i_a.toFixed(2)}A`;
    document.getElementById('val-ib').innerText = `${data.i_b.toFixed(2)}A`;
    document.getElementById('val-ic').innerText = `${data.i_c.toFixed(2)}A`;
    document.getElementById('val-isum').innerText = `${data.i_sum.toFixed(2)}A`;
    document.getElementById('val-id').innerText = `${data.i_d.toFixed(2)}A`;
    document.getElementById('val-iq-badge').innerText = `${data.i_q.toFixed(2)}A`;
    document.getElementById('val-iq-target').innerText = `${data.i_q_target.toFixed(2)}A`;
    document.getElementById('val-theta').innerText = `${data.phase_elec.toFixed(2)} rad`;
    document.getElementById('val-mech').innerText = `${data.mech_angle.toFixed(2)} rad`;
    document.getElementById('val-rpm-actual').innerText = `${Math.round(data.speed_rpm)} RPM`;
    document.getElementById('val-rpm-target').innerText = `${Math.round(data.speed_target_rpm)} RPM`;

    // Space vector magnitude badge
    document.getElementById('vector-mag').innerText = `|I| = ${data.i_vector_mag.toFixed(2)}A`;

    const setTxt = (id, value) => {
      const el = document.getElementById(id);
      if (el) el.innerText = value;
    };
    const jointDeg = ((data.joint_angle || 0) * 180.0 / Math.PI);
    const mechDeg = ((data.mech_angle || 0) * 180.0 / Math.PI);
    setTxt('val-joint-deg', `${jointDeg >= 0 ? '+' : ''}${jointDeg.toFixed(1)} deg`);
    setTxt('val-joint-rad', `${(data.joint_angle || 0).toFixed(3)} rad (shaft ${mechDeg.toFixed(1)} deg)`);
    setTxt('val-rpm-disp', data.speed_rpm.toFixed(1));
    setTxt('val-iq-disp', data.i_q.toFixed(2));
    const modeNames = ['IDLE', 'CURRENT', 'BRAKE', 'SPEED', 'POSITION', 'VOLTAGE', 'HANDBRAKE', 'OPENLOOP'];
    setTxt('val-mode', modeNames[data.control_mode] || `MODE ${data.control_mode}`);

    const faultEl = document.getElementById('val-fault');
    if (faultEl) {
      if ((data.fault_code || 0) === 0) {
        faultEl.innerText = 'NO FAULT';
        faultEl.classList.remove('fault-bad');
        faultEl.classList.add('fault-ok');
      } else {
        faultEl.innerText = `FAULT ${data.fault_code}`;
        faultEl.classList.remove('fault-ok');
        faultEl.classList.add('fault-bad');
      }
    }
    if (data.encoder_dir !== undefined) {
      setTxt('display-dir', data.encoder_dir > 0 ? '+1' : '-1');
      this.currentDir = data.encoder_dir > 0 ? 1 : -1;
    }
    if (data.zero_elec_angle !== undefined) {
      setTxt('display-offset', `${data.zero_elec_angle.toFixed(2)} rad`);
      this.currentOffset = data.zero_elec_angle;
    }
    if (data.encoder_lut_enabled !== undefined) {
      setTxt('display-lut', data.encoder_lut_enabled ? 'ON' : 'OFF');
    }

    // Throttled Log Output (every 600ms)
    const now = Date.now();
    if (now - this.lastLogTime > 600) {
      this.lastLogTime = now;
      const vq = Number(data.vq || 0);
      const vd = Number(data.vd || 0);
      const da = Number(data.duty_a || 0) * 100;
      const db = Number(data.duty_b || 0) * 100;
      const dc = Number(data.duty_c || 0) * 100;
      this.log(`[FOC] RPM: ${Math.round(data.speed_rpm)} / ${Math.round(data.speed_target_rpm)} | Ia: ${data.i_a.toFixed(2)}A | Ib: ${data.i_b.toFixed(2)}A | Ic: ${data.i_c.toFixed(2)}A | Id: ${data.i_d.toFixed(2)}A | Iq: ${data.i_q.toFixed(2)}A | IqT: ${data.i_q_target.toFixed(2)}A | Vq: ${vq.toFixed(2)}V | Vd: ${vd.toFixed(2)}V | D: ${da.toFixed(0)}/${db.toFixed(0)}/${dc.toFixed(0)}% | Vbus: ${data.v_bus.toFixed(1)}V | LUT:${data.encoder_lut_enabled || 0} C:${data.calibration_result || 0} | S:${data.motor_state} F:${data.fault_code}`, 'telemetry');
    }

    if (this.isRecording) {
      this.recordedSamples++;
      document.getElementById('recorded-count').innerText = this.recordedSamples;
    }
  },

  drawSpaceVector(id, iq, theta, mag) {
    const canvas = document.getElementById('canvas-vector-polar');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const w = canvas.width;
    const h = canvas.height;
    const cx = w / 2;
    const cy = h / 2;
    const r = (w / 2) - 15;

    const isDark = this.isDarkTheme;
    ctx.clearRect(0, 0, w, h);

    // Background circles
    ctx.strokeStyle = isDark ? '#252d36' : '#e1e7ee';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(cx, cy, r * 0.33, 0, Math.PI * 2);
    ctx.arc(cx, cy, r * 0.66, 0, Math.PI * 2);
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.stroke();

    // Cross axes (D and Q axes)
    ctx.beginPath();
    ctx.moveTo(cx, 15); ctx.lineTo(cx, h - 15);
    ctx.moveTo(15, cy); ctx.lineTo(w - 15, cy);
    ctx.stroke();

    // Rotating Current Vector Arrow
    const scale = Math.min(1.0, mag / 10.0);
    const arrowLen = r * Math.max(0.15, scale);
    const vx = cx + arrowLen * Math.cos(theta);
    const vy = cy - arrowLen * Math.sin(theta);

    // Vector Trace
    ctx.strokeStyle = '#1f6feb';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(vx, vy);
    ctx.stroke();

    // Arrow tip
    ctx.fillStyle = '#4ea1ff';
    ctx.beginPath();
    ctx.arc(vx, vy, 4, 0, Math.PI * 2);
    ctx.fill();
  },

  log(msg, type = 'system') {
    const term = document.getElementById('log-terminal');
    if (!term) return;
    const line = document.createElement('div');
    line.className = `log-line ${type}`;
    const ts = new Date().toISOString().substring(11, 19);
    line.innerText = `[${ts}] ${msg}`;
    term.appendChild(line);

    while (term.children.length > 50) {
      term.removeChild(term.firstChild);
    }
    term.scrollTop = term.scrollHeight;
  },

  startAnimationLoop() {
    const loop = () => {
      const dark = this.isDarkTheme;

      for (const key in this.scopes) {
        this.scopes[key].render(dark);
      }
      requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);

    // Periodic status poll (Hz rate, connection state)
    setInterval(async () => {
      try {
        const res = await fetch('/api/status');
        if (res.ok) {
          const s = await res.json();
          document.getElementById('telemetry-hz').innerText = `${s.telemetry_hz} Hz`;
          if (s.connected !== this.connected) {
            this.updateConnectionStatus(s.connected, s.is_simulation, s.port);
          }
        }
      } catch (e) {}
    }, 1000);
  }
};

window.setQuickRpm = (rpm) => {
  document.getElementById('speed-slider').value = rpm;
  document.getElementById('slider-val-text').innerText = rpm;
  App.sendSpeedCommand(rpm);
};

window.addEventListener('DOMContentLoaded', () => App.init());
