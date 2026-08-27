/**
 * FOC Telemetry Studio OSS - Client Controller
 * Features:
 * - Truly Frozen Independent Waveform Pause (locks frozenWriteIndex snapshot)
 * - Per-Scope Independent Controls (Pause/Resume, Pan Left/Right, Live Reset, 100-Level Zoom)
 * - Direct Mouse Wheel Zoom on hover for ANY individual scope canvas
 * - Mouse Drag to smoothly pan waveform history backward/forward
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
    this.maxHistory = options.maxHistory || 10000;
    this.yMin = options.yMin !== undefined ? options.yMin : -10;
    this.yMax = options.yMax !== undefined ? options.yMax : 10;
    this.autoScale = options.autoScale !== undefined ? options.autoScale : true;

    // Independent state
    this.zoomLevel = options.zoomLevel || 25; // 1 to 100
    this.windowSize = 250;
    this.panOffset = 0; // 0 = live, >0 = history
    this.isPaused = false;
    this.frozenWriteIndex = 0; // Locked snapshot index when paused

    // Temporary zoom overlay feedback
    this.zoomOverlayTimer = 0;

    // Ring buffers for deep history
    this.buffers = {};
    this.channels.forEach(ch => {
      this.buffers[ch.key] = new Float32Array(this.maxHistory);
    });

    this.writeIndex = 0;
    this.totalSamples = 0;

    this.calculateWindowSize();
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
        pauseBtn.className = 'btn btn-outline-success btn-scope-tool btn-pause-toggle';
      } else {
        pauseBtn.innerText = 'Pause';
        pauseBtn.className = 'btn btn-outline-warning btn-scope-tool btn-pause-toggle';
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
    const bgColor = isDarkTheme ? '#070b12' : '#ffffff';
    const gridColor = isDarkTheme ? '#1e293b' : '#e2e8f0';
    const zeroColor = isDarkTheme ? '#334155' : '#94a3b8';

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
    if (this.autoScale) {
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
        const margin = Math.max(0.3, (localMax - localMin) * 0.15);
        min = localMin - margin;
        max = localMax + margin;
      }
    }

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
    if (min < 0 && max > 0) {
      const zeroY = h - ((0 - min) / range) * h;
      ctx.strokeStyle = zeroColor;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(0, zeroY);
      ctx.lineTo(w, zeroY);
      ctx.stroke();
      ctx.setLineDash([]);
    }

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
    localStorage.setItem('foc_theme', theme);

    const btn = document.getElementById('btn-theme-toggle');
    if (btn) {
      btn.innerText = this.isDarkTheme ? 'Theme: Dark' : 'Theme: Light';
    }
  },

  toggleTheme() {
    this.setTheme(this.isDarkTheme ? 'light' : 'dark');
  },

  setupOscilloscopes() {
    // Scope 1: 3-Phase Sinusoids
    this.scopes.phase = new DeepOscilloscope('phase', 'canvas-scope-phase', [
      { key: 'i_a', color: '#ef4444', lineWidth: 2 },
      { key: 'i_b', color: '#10b981', lineWidth: 2 },
      { key: 'i_c', color: '#3b82f6', lineWidth: 2 }
    ], { zoomLevel: 25 });

    // Scope 2: Space Vector D-Q
    this.scopes.dq = new DeepOscilloscope('dq', 'canvas-scope-dq', [
      { key: 'i_d', color: '#f59e0b', lineWidth: 2 },
      { key: 'i_q', color: '#8b5cf6', lineWidth: 2 },
      { key: 'i_q_target', color: '#06b6d4', lineWidth: 1.5, lineDash: [4, 4] }
    ], { zoomLevel: 25 });

    // Scope 3: Electrical & Mechanical Angle
    this.scopes.angle = new DeepOscilloscope('angle', 'canvas-scope-angle', [
      { key: 'phase_elec', color: '#ec4899', lineWidth: 2 },
      { key: 'mech_angle', color: '#818cf8', lineWidth: 1.5 }
    ], { autoScale: false, yMin: -3.5, yMax: 3.5, zoomLevel: 25 });

    // Scope 4: Speed Response (RPM)
    this.scopes.speed = new DeepOscilloscope('speed', 'canvas-scope-speed', [
      { key: 'speed_rpm', color: '#10b981', lineWidth: 2 },
      { key: 'speed_target_rpm', color: '#06b6d4', lineWidth: 1.5, lineDash: [4, 4] }
    ], { zoomLevel: 25 });
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
        masterPauseBtn.innerText = anyRunning ? 'Resume All' : 'Pause All';
        masterPauseBtn.className = anyRunning ? 'btn btn-outline-success btn-sm' : 'btn btn-outline-warning btn-sm';
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

    // Recording Controls
    document.getElementById('btn-record-toggle').addEventListener('click', () => this.toggleRecording());
    document.getElementById('btn-download-csv').addEventListener('click', () => this.downloadCsv());

    // Clear Log
    document.getElementById('btn-clear-log').addEventListener('click', () => {
      document.getElementById('log-terminal').innerHTML = '<div class="log-line system">[SYSTEM] Log cleared.</div>';
    });
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
        opt.innerText = p.description;
        select.appendChild(opt);
      });
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
      text.classList.remove('text-secondary');
      text.classList.add('text-success');
      btn.innerText = 'Disconnect';
      btn.className = 'btn btn-outline-danger btn-sm px-3 fw-semibold';
      this.log(`Connected successfully to ${portName}`, 'system');
    } else {
      text.innerText = 'Disconnected';
      text.classList.add('text-secondary');
      text.classList.remove('text-success');
      btn.innerText = 'Connect';
      btn.className = 'btn btn-primary btn-sm px-3 fw-semibold';
      document.getElementById('telemetry-hz').innerText = '0 Hz';
      this.log('Disconnected from device.', 'system');
    }
  },

  async setMotorMode(mode) {
    document.querySelectorAll('.btn-mode').forEach(b => {
      b.classList.toggle('active', parseInt(b.dataset.mode) === mode);
    });
    const modeNames = ['IDLE (0)', 'CURRENT (1)', 'BRAKE (2)', 'SPEED (3)', 'POSITION (4)'];
    document.getElementById('active-mode-badge').innerText = modeNames[mode] || `MODE ${mode}`;

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
      await fetch('/api/control', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ control_mode: 3, target_value: rpm })
      });
    } catch (e) {
      this.log(`Error sending speed: ${e.message}`, 'error');
    }
  },

  async emergencyStop() {
    try {
      await fetch('/api/control', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ control_mode: 0, target_value: 0.0 })
      });
      this.setMotorMode(0);
      this.log('EMERGENCY STOP ACTIVATED! Motor set to IDLE.', 'error');
    } catch (e) {
      this.log(`E-Stop failed: ${e.message}`, 'error');
    }
  },

  async toggleRecording() {
    const btn = document.getElementById('btn-record-toggle');
    const dlBtn = document.getElementById('btn-download-csv');
    const indicator = document.getElementById('recording-indicator');

    if (!this.isRecording) {
      await fetch('/api/record/start', { method: 'POST' });
      this.isRecording = true;
      btn.innerText = 'Stop Recording';
      btn.className = 'btn btn-danger btn-sm w-50 fw-semibold';
      dlBtn.disabled = true;
      indicator.classList.remove('d-none');
      this.log('Telemetry recording started (Pandas DataFrame buffer active).', 'warn');
    } else {
      const res = await fetch('/api/record/stop', { method: 'POST' });
      const data = await res.json();
      this.isRecording = false;
      btn.innerText = 'Start Recording';
      btn.className = 'btn btn-outline-danger btn-sm w-50 fw-semibold';
      dlBtn.disabled = false;
      indicator.classList.add('d-none');
      this.log(`Recording stopped. Captured ${data.total_samples} samples. Ready to export CSV.`, 'system');
    }
  },

  downloadCsv() {
    window.location.href = '/api/record/export';
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

    // Throttled Log Output (every 600ms)
    const now = Date.now();
    if (now - this.lastLogTime > 600) {
      this.lastLogTime = now;
      this.log(`[FOC] RPM: ${Math.round(data.speed_rpm)} | Ia: ${data.i_a.toFixed(2)}A | Ib: ${data.i_b.toFixed(2)}A | Ic: ${data.i_c.toFixed(2)}A | Iq: ${data.i_q.toFixed(2)}A | Vbus: ${data.v_bus.toFixed(1)}V`, 'telemetry');
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
    ctx.strokeStyle = isDark ? '#1e293b' : '#e2e8f0';
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
    ctx.strokeStyle = '#3b82f6';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(vx, vy);
    ctx.stroke();

    // Arrow tip
    ctx.fillStyle = '#60a5fa';
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
