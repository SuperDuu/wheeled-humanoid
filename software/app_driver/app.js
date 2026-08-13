/**
 * STM32 FOC Telemetry & 3-Phase Oscilloscope Frontend Application
 * High-performance 60FPS Canvas Oscilloscope with Per-Scope Freeze, Multi-Level Zoom, and Interactive Canvas Controls.
 */

class FOCOscilloscopeStudio {
  constructor() {
    this.isConnected = false;
    this.eventSource = null;
    this.viewMode = 'currents'; // 'currents' or 'duties'
    
    // Timebase options in milliseconds
    this.timebaseOptions = [50, 100, 250, 500, 1000, 2000, 5000];

    // Per-Scope Independent State (Freeze, Zoom, Pan)
    this.scopes = {
      1: { timebase: 250, isFrozen: false, frozenSnapshot: null, panOffset: 0, hoverX: null },
      2: { timebase: 250, isFrozen: false, frozenSnapshot: null, panOffset: 0, hoverX: null },
      3: { timebase: 250, isFrozen: false, frozenSnapshot: null, panOffset: 0, hoverX: null },
      4: { timebase: 500, isFrozen: false, frozenSnapshot: null, panOffset: 0, hoverX: null },
    };

    // Telemetry Buffers (Circular time-series arrays, max 3000 samples)
    this.maxSamples = 3000;
    this.buffer = {
      time: [],
      i_a: [],
      i_b: [],
      i_c: [],
      i_d: [],
      i_q: [],
      i_q_tgt: [],
      duty_a: [],
      duty_b: [],
      duty_c: [],
      phase: [],
      mech: [],
      joint: [],
      rpm: [],
      rpm_tgt: [],
    };

    // Canvas references
    this.canvases = {
      1: document.getElementById('canvas-scope1'),
      2: document.getElementById('canvas-scope2'),
      3: document.getElementById('canvas-scope3'),
      4: document.getElementById('canvas-scope4')
    };

    this.ctxs = {
      1: this.canvases[1] ? this.canvases[1].getContext('2d') : null,
      2: this.canvases[2] ? this.canvases[2].getContext('2d') : null,
      3: this.canvases[3] ? this.canvases[3].getContext('2d') : null,
      4: this.canvases[4] ? this.canvases[4].getContext('2d') : null
    };

    // Telemetry rate tracking
    this.packetCount = 0;
    this.lastPacketTime = performance.now();
    this.fps = 0;

    // Recording State
    this.isRecording = false;

    this.init();
  }

  init() {
    this.setupCanvases();
    this.bindEvents();
    this.fetchPorts();
    this.startSSE();
    this.startAnimationLoop();
    this.startStatusPoller();
  }

  setupCanvases() {
    const resize = () => {
      Object.entries(this.canvases).forEach(([id, canvas]) => {
        if (!canvas) return;
        const dpr = window.devicePixelRatio || 1;
        const rect = canvas.getBoundingClientRect();
        const w = rect.width || canvas.clientWidth || 600;
        const h = rect.height || canvas.clientHeight || 220;
        canvas.width = Math.round(w * dpr);
        canvas.height = Math.round(h * dpr);
      });
    };
    resize();
    window.addEventListener('resize', resize);
    setTimeout(resize, 200);
  }

  bindEvents() {
    // Connect / Disconnect Button
    const btnConnect = document.getElementById('btn-connect');
    if (btnConnect) {
      btnConnect.addEventListener('click', () => {
        if (this.isConnected) {
          this.disconnect();
        } else {
          const port = document.getElementById('port-select').value;
          const baud = document.getElementById('baud-select').value;
          if (!port) {
            alert('Please select a valid USB Serial Port first.');
            return;
          }
          this.connect(port, baud);
        }
      });
    }

    // Refresh Ports
    const btnRefresh = document.getElementById('btn-refresh-ports');
    if (btnRefresh) {
      btnRefresh.addEventListener('click', () => this.fetchPorts());
    }

    // Scope 1 View Mode (Currents vs Duties)
    const btnViewCurrents = document.getElementById('view-currents');
    const btnViewDuties = document.getElementById('view-duties');
    if (btnViewCurrents && btnViewDuties) {
      btnViewCurrents.addEventListener('click', () => {
        this.viewMode = 'currents';
        btnViewCurrents.classList.add('active');
        btnViewDuties.classList.remove('active');
      });

      btnViewDuties.addEventListener('click', () => {
        this.viewMode = 'duties';
        btnViewDuties.classList.add('active');
        btnViewCurrents.classList.remove('active');
      });
    }

    // Per-Scope Freeze / Resume Buttons
    document.querySelectorAll('.btn-scope-pause').forEach(btn => {
      btn.addEventListener('click', () => {
        const scopeId = parseInt(btn.dataset.scope, 10);
        this.toggleScopeFreeze(scopeId);
      });
    });

    // Per-Scope Zoom (+ / -) Buttons
    document.querySelectorAll('.btn-zoom').forEach(btn => {
      btn.addEventListener('click', () => {
        const scopeId = parseInt(btn.dataset.scope, 10);
        const direction = btn.dataset.zoom;
        this.stepScopeZoom(scopeId, direction);
      });
    });

    // Timebase Dropdowns
    document.querySelectorAll('.timebase-dropdown').forEach(select => {
      select.addEventListener('change', (e) => {
        const scopeId = parseInt(select.dataset.scope, 10);
        this.setScopeTimebase(scopeId, parseInt(e.target.value, 10));
      });
    });

    // Interactive Canvas Controls (Mouse Wheel Zoom & Drag Pan & Hover Crosshair)
    [1, 2, 3, 4].forEach(id => {
      const canvas = this.canvases[id];
      if (!canvas) return;

      // Wheel Zoom
      canvas.addEventListener('wheel', (e) => {
        e.preventDefault();
        const dir = e.deltaY < 0 ? 'in' : 'out';
        this.stepScopeZoom(id, dir);
      }, { passive: false });

      // Hover Crosshair
      canvas.addEventListener('mousemove', (e) => {
        const rect = canvas.getBoundingClientRect();
        this.scopes[id].hoverX = (e.clientX - rect.left) / rect.width;
      });

      canvas.addEventListener('mouseleave', () => {
        this.scopes[id].hoverX = null;
      });
    });

    // Motor Control Mode Buttons
    document.querySelectorAll('.btn-mode').forEach(btn => {
      btn.addEventListener('click', () => {
        const mode = btn.dataset.mode;
        this.sendCommand(`MODE ${mode}`);
        document.querySelectorAll('.btn-mode').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
      });
    });

    // Speed Controls
    const inputSpeed = document.getElementById('input-speed');
    const speedDisplay = document.getElementById('speed-display');
    const btnSetSpeed = document.getElementById('btn-set-speed');
    const sliderSpeed = document.getElementById('slider-speed');

    const updateSpeed = (val) => {
      if (inputSpeed) inputSpeed.value = val;
      if (sliderSpeed) sliderSpeed.value = val;
      if (speedDisplay) speedDisplay.innerText = `${val} RPM`;
      this.sendCommand(`MODE 3`);
      this.sendCommand(`SPEED ${val}`);
      document.querySelectorAll('.btn-mode').forEach(b => b.classList.remove('active'));
      const speedBtn = document.querySelector('.btn-mode[data-mode="3"]');
      if (speedBtn) speedBtn.classList.add('active');
    };

    if (btnSetSpeed && inputSpeed) {
      btnSetSpeed.addEventListener('click', () => {
        updateSpeed(inputSpeed.value);
      });
      inputSpeed.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') updateSpeed(inputSpeed.value);
      });
    }

    if (sliderSpeed) {
      sliderSpeed.addEventListener('input', (e) => {
        if (speedDisplay) speedDisplay.innerText = `${e.target.value} RPM`;
        if (inputSpeed) inputSpeed.value = e.target.value;
      });
      sliderSpeed.addEventListener('change', (e) => {
        updateSpeed(e.target.value);
      });
    }

    document.querySelectorAll('.btn-preset').forEach(btn => {
      btn.addEventListener('click', () => {
        const speed = btn.dataset.speed;
        if (speed !== undefined) {
          updateSpeed(speed);
        }
      });
    });

    // Current Iq Controls
    const inputIq = document.getElementById('input-iq');
    const iqDisplay = document.getElementById('iq-display');
    const btnSetIq = document.getElementById('btn-set-iq');

    const updateIq = (val) => {
      const numVal = parseFloat(val).toFixed(1);
      if (inputIq) inputIq.value = numVal;
      if (iqDisplay) iqDisplay.innerText = `${numVal} A`;
      this.sendCommand(`MODE 1`);
      this.sendCommand(`IQ ${numVal}`);
      document.querySelectorAll('.btn-mode').forEach(b => b.classList.remove('active'));
      const curBtn = document.querySelector('.btn-mode[data-mode="1"]');
      if (curBtn) curBtn.classList.add('active');
    };

    if (btnSetIq && inputIq) {
      btnSetIq.addEventListener('click', () => {
        updateIq(inputIq.value);
      });
      inputIq.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') updateIq(inputIq.value);
      });
    }

    // Position / Angle Controls (Degrees -> Radians)
    const inputPos = document.getElementById('input-pos');
    const posDisplay = document.getElementById('pos-display');
    const btnSetPos = document.getElementById('btn-set-pos');

    const updatePos = (degVal) => {
      const deg = parseFloat(degVal) || 0;
      const rad = deg * Math.PI / 180.0;
      if (inputPos) inputPos.value = deg;
      if (posDisplay) posDisplay.innerText = `${deg.toFixed(1)}° (${rad.toFixed(2)} rad)`;
      this.sendCommand(`MODE 4`);
      this.sendCommand(`POS ${rad.toFixed(4)}`);
      document.querySelectorAll('.btn-mode').forEach(b => b.classList.remove('active'));
      const posBtn = document.querySelector('.btn-mode[data-mode="4"]');
      if (posBtn) posBtn.classList.add('active');
    };

    if (btnSetPos && inputPos) {
      btnSetPos.addEventListener('click', () => {
        updatePos(inputPos.value);
      });
      inputPos.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') updatePos(inputPos.value);
      });
    }

    document.querySelectorAll('.btn-pos-preset').forEach(btn => {
      btn.addEventListener('click', () => {
        const deg = btn.dataset.deg;
        if (deg !== undefined) {
          updatePos(deg);
        }
      });
    });

    // Open-Loop Test Run Buttons (Forward & Reverse)
    const btnFwd = document.getElementById('btn-openloop-fwd');
    if (btnFwd) {
      btnFwd.addEventListener('click', () => {
        this.sendCommand('OPENLOOP 100');
        this.appendLog('⚡ TEST RUN (+100 RPM) started.', 'success');
      });
    }
    const btnRev = document.getElementById('btn-openloop-rev');
    if (btnRev) {
      btnRev.addEventListener('click', () => {
        this.sendCommand('OPENLOOP -100');
        this.appendLog('🔄 TEST RUN (-100 RPM Reverse) started.', 'success');
      });
    }

    // Align Encoder Button
    const btnAlign = document.getElementById('btn-align');
    if (btnAlign) {
      btnAlign.addEventListener('click', () => {
        this.sendCommand('ALIGN');
        this.appendLog('Aligning encoder zero angle... Please wait 2 seconds.', 'warn');
      });
    }

    // Emergency Stop
    const btnEstop = document.getElementById('btn-estop');
    if (btnEstop) {
      btnEstop.addEventListener('click', () => {
        this.sendCommand('STOP');
        if (sliderSpeed) sliderSpeed.value = 0;
        if (inputSpeed) inputSpeed.value = 0;
        if (speedDisplay) speedDisplay.innerText = '0 RPM';
        if (inputIq) inputIq.value = 0;
        if (iqDisplay) iqDisplay.innerText = '0.0 A';
        document.querySelectorAll('.btn-mode').forEach(b => b.classList.remove('active'));
        const idleBtn = document.querySelector('.btn-mode[data-mode="0"]');
        if (idleBtn) idleBtn.classList.add('active');
      });
    }

    // CSV Recording Controls
    const btnRecord = document.getElementById('btn-record-toggle');
    const btnDownload = document.getElementById('btn-download-csv');
    if (btnRecord) {
      btnRecord.addEventListener('click', async () => {
        if (!this.isRecording) {
          await fetch('/api/record/start');
          this.isRecording = true;
          btnRecord.innerText = '⏹️ Stop Recording';
          btnRecord.classList.remove('btn-secondary');
          btnRecord.classList.add('btn-danger');
          if (btnDownload) btnDownload.disabled = true;
        } else {
          const res = await fetch('/api/record/stop');
          const data = await res.json();
          this.isRecording = false;
          btnRecord.innerText = '🔴 Start Recording';
          btnRecord.classList.remove('btn-danger');
          btnRecord.classList.add('btn-secondary');
          if (btnDownload) btnDownload.disabled = false;
          this.appendLog(`Recorded ${data.samples} samples ready for download.`);
        }
      });
    }

    if (btnDownload) {
      btnDownload.addEventListener('click', () => {
        window.location.href = '/api/record/download';
      });
    }

    // Log actions
    const btnCopyLog = document.getElementById('btn-copy-log');
    if (btnCopyLog) {
      btnCopyLog.addEventListener('click', () => {
        const text = document.getElementById('log-console').innerText;
        navigator.clipboard.writeText(text);
        alert('Logs copied to clipboard!');
      });
    }

    const btnClearLog = document.getElementById('btn-clear-log');
    if (btnClearLog) {
      btnClearLog.addEventListener('click', () => {
        document.getElementById('log-console').innerHTML = '';
      });
    }
  }

  toggleScopeFreeze(scopeId) {
    const scope = this.scopes[scopeId];
    if (!scope) return;

    scope.isFrozen = !scope.isFrozen;
    const btn = document.getElementById(`btn-pause-scope${scopeId}`);
    const tag = document.getElementById(`freeze-tag-scope${scopeId}`);

    if (scope.isFrozen) {
      // Snapshot current buffers
      scope.frozenSnapshot = {
        time: [...this.buffer.time],
        i_a: [...this.buffer.i_a],
        i_b: [...this.buffer.i_b],
        i_c: [...this.buffer.i_c],
        i_d: [...this.buffer.i_d],
        i_q: [...this.buffer.i_q],
        i_q_tgt: [...this.buffer.i_q_tgt],
        duty_a: [...this.buffer.duty_a],
        duty_b: [...this.buffer.duty_b],
        duty_c: [...this.buffer.duty_c],
        phase: [...this.buffer.phase],
        mech: [...this.buffer.mech],
        joint: [...this.buffer.joint],
        rpm: [...this.buffer.rpm],
        rpm_tgt: [...this.buffer.rpm_tgt],
      };
      if (btn) {
        btn.innerText = scopeId === 1 ? '▶️ Live' : '▶️';
        btn.classList.add('is-frozen');
      }
      if (tag) tag.style.display = 'inline-block';
      this.appendLog(`Scope ${scopeId} Frame Frozen. (Zoom/Inspect active)`);
    } else {
      scope.frozenSnapshot = null;
      if (btn) {
        btn.innerText = scopeId === 1 ? '⏸️ Freeze' : '⏸️';
        btn.classList.remove('is-frozen');
      }
      if (tag) tag.style.display = 'none';
      this.appendLog(`Scope ${scopeId} Resumed Live Stream.`);
    }
  }

  stepScopeZoom(scopeId, direction) {
    const scope = this.scopes[scopeId];
    if (!scope) return;

    const currentIdx = this.timebaseOptions.indexOf(scope.timebase);
    let nextIdx = currentIdx;

    if (direction === 'in') {
      // Zoom in -> smaller ms
      if (currentIdx > 0) nextIdx = currentIdx - 1;
    } else {
      // Zoom out -> larger ms
      if (currentIdx < this.timebaseOptions.length - 1) nextIdx = currentIdx + 1;
    }

    if (nextIdx !== currentIdx) {
      const newTb = this.timebaseOptions[nextIdx];
      this.setScopeTimebase(scopeId, newTb);
    }
  }

  setScopeTimebase(scopeId, timebaseMs) {
    const scope = this.scopes[scopeId];
    if (!scope) return;

    scope.timebase = timebaseMs;
    const select = document.getElementById(`timebase-scope${scopeId}`);
    if (select) {
      select.value = timebaseMs;
    }
  }

  async fetchPorts() {
    try {
      const res = await fetch('/api/ports');
      const data = await res.json();
      const select = document.getElementById('port-select');
      if (!select) return;
      select.innerHTML = '';

      if (data.ports && data.ports.length > 0) {
        data.ports.forEach(p => {
          const opt = document.createElement('option');
          opt.value = p.port;
          opt.innerText = `${p.port} (${p.description || 'Serial Device'})`;
          select.appendChild(opt);
        });
      } else {
        const opt = document.createElement('option');
        opt.value = '';
        opt.innerText = 'No USB Serial Port Detected';
        select.appendChild(opt);
      }

      if (data.connected && data.current_port) {
        select.value = data.current_port;
        this.setConnectedState(true);
        if (!this.eventSource) {
          this.startSSE();
        }
      }
    } catch (err) {
      console.error('Failed to fetch ports:', err);
    }
  }

  async connect(port, baudrate) {
    try {
      const res = await fetch('/api/connect', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ port, baudrate: parseInt(baudrate, 10) })
      });
      const data = await res.json();

      if (data.success) {
        this.setConnectedState(true);
        this.startSSE();
        this.appendLog(`Connected to ${port} @ ${baudrate} baud.`);
      } else {
        alert(`Connection Failed: ${data.message}`);
        this.appendLog(`Connection Failed: ${data.message}`, 'error');
      }
    } catch (err) {
      console.error('Connect error:', err);
      alert(`Connect error: ${err.message}`);
    }
  }

  async disconnect() {
    try {
      await fetch('/api/disconnect', { method: 'POST' });
      this.setConnectedState(false);
      if (this.eventSource) {
        this.eventSource.close();
        this.eventSource = null;
      }
      this.appendLog('Disconnected from USB serial port.');
    } catch (err) {
      console.error('Disconnect error:', err);
    }
  }

  setConnectedState(connected) {
    this.isConnected = connected;
    const btn = document.getElementById('btn-connect');
    const badge = document.getElementById('conn-status');

    if (connected) {
      if (btn) {
        btn.innerHTML = '<span class="btn-icon">🔌</span> Disconnect';
        btn.classList.remove('btn-primary');
        btn.classList.add('btn-secondary');
      }
      if (badge) {
        badge.classList.add('connected');
        badge.querySelector('.text').innerText = 'Connected';
      }
    } else {
      if (btn) {
        btn.innerHTML = '<span class="btn-icon">🔌</span> Connect';
        btn.classList.remove('btn-secondary');
        btn.classList.add('btn-primary');
      }
      if (badge) {
        badge.classList.remove('connected');
        badge.querySelector('.text').innerText = 'Disconnected';
      }
      const hzEl = document.getElementById('telemetry-hz');
      if (hzEl) hzEl.innerText = '0';
    }
  }

  startSSE() {
    if (this.eventSource) {
      this.eventSource.close();
    }

    this.eventSource = new EventSource('/api/stream');
    this.eventSource.onmessage = (e) => {
      try {
        const pkt = JSON.parse(e.data);
        this.handleTelemetry(pkt);
      } catch (err) {
        console.error('Parse telemetry error:', err);
      }
    };

    this.eventSource.onerror = () => {
      console.warn('SSE stream disconnected, polling fallback active...');
    };
  }

  handleTelemetry(pkt) {
    const now = performance.now();
    this.packetCount++;

    // Push into ring buffers
    this.buffer.time.push(now);
    this.buffer.i_a.push(pkt.i_a);
    this.buffer.i_b.push(pkt.i_b);
    this.buffer.i_c.push(pkt.i_c);
    this.buffer.i_d.push(pkt.i_d);
    this.buffer.i_q.push(pkt.i_q);
    this.buffer.i_q_tgt.push(pkt.i_q_target);
    this.buffer.duty_a.push(pkt.duty_a * 100);
    this.buffer.duty_b.push(pkt.duty_b * 100);
    this.buffer.duty_c.push(pkt.duty_c * 100);
    this.buffer.phase.push(pkt.phase_elec);
    this.buffer.mech.push(pkt.mech_angle);
    this.buffer.joint.push(pkt.joint_angle);
    this.buffer.rpm.push(pkt.speed_rpm);
    this.buffer.rpm_tgt.push(pkt.speed_target_rpm);

    // Maintain max buffer capacity
    if (this.buffer.time.length > this.maxSamples) {
      Object.keys(this.buffer).forEach(k => this.buffer[k].shift());
    }

    // Update Telemetry Metrics HUD
    this.updateHUD(pkt);
  }

  updateHUD(pkt) {
    const setTxt = (id, val) => {
      const el = document.getElementById(id);
      if (el) el.innerText = val;
    };

    setTxt('val-ia', `${pkt.i_a.toFixed(2)} A`);
    setTxt('val-ib', `${pkt.i_b.toFixed(2)} A`);
    setTxt('val-ic', `${pkt.i_c.toFixed(2)} A`);

    const iPeak = Math.max(Math.abs(pkt.i_a), Math.abs(pkt.i_b), Math.abs(pkt.i_c));
    setTxt('val-ipeak', `${iPeak.toFixed(2)} A`);

    const iSum = pkt.i_a + pkt.i_b + pkt.i_c;
    setTxt('val-isum', `${iSum.toFixed(2)} A`);

    // Quality check
    const qBadge = document.getElementById('val-quality');
    if (qBadge) {
      if (Math.abs(iSum) < 0.8) {
        qBadge.innerText = 'BALANCED 120°';
        qBadge.className = 'hud-badge status-good';
      } else {
        qBadge.innerText = 'ASYMMETRIC';
        qBadge.className = 'hud-badge status-warn';
      }
    }

    setTxt('val-id', `${pkt.i_d.toFixed(2)} A`);
    setTxt('val-iq', `${pkt.i_q.toFixed(2)} A`);
    setTxt('val-iq-tgt', `${pkt.i_q_target.toFixed(2)} A`);

    setTxt('val-phase', `${pkt.phase_elec.toFixed(2)} rad`);
    setTxt('val-mech', `${pkt.mech_angle.toFixed(2)} rad`);
    setTxt('val-joint', `${pkt.joint_angle.toFixed(2)} rad`);

    setTxt('val-rpm', `${pkt.speed_rpm.toFixed(1)} RPM`);
    setTxt('val-rpm-tgt', `${pkt.speed_target_rpm.toFixed(1)} RPM`);
    const err = pkt.speed_target_rpm - pkt.speed_rpm;
    setTxt('val-rpm-err', `${err.toFixed(1)} RPM`);

    setTxt('val-vbus', pkt.v_bus.toFixed(1));
    setTxt('val-temp', pkt.temp_fet.toFixed(1));

    const modeNames = ["IDLE (0)", "CURRENT (1)", "BRAKE (2)", "SPEED (3)", "POS (4)"];
    setTxt('val-mode', modeNames[pkt.control_mode] || `MODE ${pkt.control_mode}`);
  }

  async sendCommand(cmd) {
    try {
      await fetch('/api/command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ command: cmd })
      });
      this.appendLog(`Command: ${cmd}`);
    } catch (err) {
      console.error('Send command error:', err);
    }
  }

  startStatusPoller() {
    setInterval(async () => {
      try {
        const res = await fetch('/api/status');
        const data = await res.json();
        
        if (data.connected && !this.isConnected) {
          this.setConnectedState(true);
        }

        const hzEl = document.getElementById('telemetry-hz');
        if (hzEl) hzEl.innerText = Math.round(data.fps || 0);

        const recEl = document.getElementById('recorded-count');
        if (recEl) recEl.innerText = data.recorded_samples || 0;

        // Fallback ingestion if SSE stream was stalled
        if (data.latest && this.buffer.time.length < 5) {
          this.handleTelemetry(data.latest);
        }

        if (data.logs && data.logs.length > 0) {
          const lastLog = data.logs[data.logs.length - 1];
          if (this.lastSyncedLog !== lastLog) {
            this.lastSyncedLog = lastLog;
            this.appendLog(lastLog);
          }
        }
      } catch (err) {
        // quiet
      }
    }, 500);
  }

  appendLog(msg, type = 'normal') {
    const consoleEl = document.getElementById('log-console');
    if (!consoleEl) return;
    const entry = document.createElement('div');
    entry.className = `log-entry ${type}`;
    entry.innerText = msg;
    consoleEl.appendChild(entry);
    consoleEl.scrollTop = consoleEl.scrollHeight;

    while (consoleEl.children.length > 100) {
      consoleEl.removeChild(consoleEl.firstChild);
    }
  }

  startAnimationLoop() {
    const render = () => {
      this.renderScope1();
      this.renderScope2();
      this.renderScope3();
      this.renderScope4();
      requestAnimationFrame(render);
    };
    requestAnimationFrame(render);
  }

  // Extract time window based on scope's timebase (e.g. 50ms..5000ms)
  getVisibleSlice(scopeId, seriesName) {
    const scope = this.scopes[scopeId];
    const dataSource = (scope.isFrozen && scope.frozenSnapshot) ? scope.frozenSnapshot : this.buffer;
    const times = dataSource.time;
    const values = dataSource[seriesName] || [];

    if (!times || times.length < 2) return [];

    const latestTime = times[times.length - 1];
    const startTime = latestTime - scope.timebase;

    // Binary search or reverse scan for startTime
    let startIdx = 0;
    for (let i = times.length - 1; i >= 0; i--) {
      if (times[i] < startTime) {
        startIdx = i;
        break;
      }
    }

    return values.slice(startIdx);
  }

  // Draw Grid & Axes
  drawGrid(ctx, canvas, yMin, yMax, yUnits = '', scopeId = 1) {
    const w = canvas.width;
    const h = canvas.height;
    ctx.clearRect(0, 0, w, h);

    // Background
    ctx.fillStyle = '#060910';
    ctx.fillRect(0, 0, w, h);

    // Grid Lines
    ctx.strokeStyle = '#121c2e';
    ctx.lineWidth = 1;

    for (let i = 1; i < 5; i++) {
      const y = (h / 5) * i;
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }

    for (let i = 1; i < 8; i++) {
      const x = (w / 8) * i;
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, h);
      ctx.stroke();
    }

    // Zero Reference Axis
    const zeroY = this.mapY(0, yMin, yMax, h);
    if (zeroY >= 0 && zeroY <= h) {
      ctx.strokeStyle = '#2d4160';
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(0, zeroY);
      ctx.lineTo(w, zeroY);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // Y Axis Labels
    ctx.fillStyle = '#64748b';
    ctx.font = '10px JetBrains Mono, monospace';
    ctx.fillText(`${yMax.toFixed(1)}${yUnits}`, 8, 14);
    ctx.fillText(`0.0${yUnits}`, 8, zeroY - 4 > 14 ? zeroY - 4 : zeroY + 12);
    ctx.fillText(`${yMin.toFixed(1)}${yUnits}`, 8, h - 6);

    // Timebase Zoom Indicator in bottom right
    const tb = this.scopes[scopeId].timebase;
    ctx.fillStyle = '#38bdf8';
    ctx.fillText(`Timebase: ${tb < 1000 ? tb + 'ms' : (tb/1000).toFixed(1) + 's'} / div`, w - 140, h - 6);

    // Draw Crosshair if hovering
    const hover = this.scopes[scopeId].hoverX;
    if (hover !== null && hover >= 0 && hover <= 1) {
      const curX = hover * w;
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.4)';
      ctx.setLineDash([2, 2]);
      ctx.beginPath();
      ctx.moveTo(curX, 0);
      ctx.lineTo(curX, h);
      ctx.stroke();
      ctx.setLineDash([]);
    }
  }

  mapY(val, min, max, h) {
    if (max === min) return h / 2;
    return h - ((val - min) / (max - min)) * h;
  }

  drawTrace(ctx, canvas, values, yMin, yMax, color, isDashed = false) {
    if (values.length < 2) return;
    const w = canvas.width;
    const h = canvas.height;
    const stepX = w / (values.length - 1);

    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.lineJoin = 'round';
    if (isDashed) {
      ctx.setLineDash([6, 4]);
    } else {
      ctx.setLineDash([]);
    }

    ctx.beginPath();
    for (let i = 0; i < values.length; i++) {
      const x = i * stepX;
      const y = this.mapY(values[i], yMin, yMax, h);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // Scope 1: 3-Phase Sinusoids (Ia, Ib, Ic)
  renderScope1() {
    const canvas = this.canvases[1];
    const ctx = this.ctxs[1];
    if (!canvas || !ctx) return;

    let seriesA, seriesB, seriesC, yMin, yMax, units;

    if (this.viewMode === 'currents') {
      seriesA = this.getVisibleSlice(1, 'i_a');
      seriesB = this.getVisibleSlice(1, 'i_b');
      seriesC = this.getVisibleSlice(1, 'i_c');
      
      const maxVal = Math.max(
        ...seriesA.map(Math.abs),
        ...seriesB.map(Math.abs),
        ...seriesC.map(Math.abs),
        2.0
      );
      const limit = Math.ceil(maxVal * 1.2 * 2) / 2;
      yMin = -limit;
      yMax = limit;
      units = 'A';
    } else {
      seriesA = this.getVisibleSlice(1, 'duty_a');
      seriesB = this.getVisibleSlice(1, 'duty_b');
      seriesC = this.getVisibleSlice(1, 'duty_c');
      yMin = 0;
      yMax = 100;
      units = '%';
    }

    this.drawGrid(ctx, canvas, yMin, yMax, units, 1);

    this.drawTrace(ctx, canvas, seriesA, yMin, yMax, '#ff4757');
    this.drawTrace(ctx, canvas, seriesB, yMin, yMax, '#2ed573');
    this.drawTrace(ctx, canvas, seriesC, yMin, yMax, '#1e90ff');
  }

  // Scope 2: FOC DQ Currents
  renderScope2() {
    const canvas = this.canvases[2];
    const ctx = this.ctxs[2];
    if (!canvas || !ctx) return;

    const seriesId = this.getVisibleSlice(2, 'i_d');
    const seriesIq = this.getVisibleSlice(2, 'i_q');
    const seriesIqTgt = this.getVisibleSlice(2, 'i_q_tgt');

    const maxVal = Math.max(
      ...seriesId.map(Math.abs),
      ...seriesIq.map(Math.abs),
      ...seriesIqTgt.map(Math.abs),
      2.0
    );
    const limit = Math.ceil(maxVal * 1.2 * 2) / 2;
    const yMin = -limit;
    const yMax = limit;

    this.drawGrid(ctx, canvas, yMin, yMax, 'A', 2);

    this.drawTrace(ctx, canvas, seriesIqTgt, yMin, yMax, '#f1c40f', true);
    this.drawTrace(ctx, canvas, seriesId, yMin, yMax, '#00d2d3');
    this.drawTrace(ctx, canvas, seriesIq, yMin, yMax, '#e056fd');
  }

  // Scope 3: Rotor Angle & Phase
  renderScope3() {
    const canvas = this.canvases[3];
    const ctx = this.ctxs[3];
    if (!canvas || !ctx) return;

    const seriesMech = this.getVisibleSlice(3, 'mech');
    const seriesPhase = this.getVisibleSlice(3, 'phase');

    const yMin = -Math.PI;
    const yMax = Math.PI;

    this.drawGrid(ctx, canvas, yMin, yMax, ' rad', 3);

    this.drawTrace(ctx, canvas, seriesMech, yMin, yMax, '#54a0ff');
    this.drawTrace(ctx, canvas, seriesPhase, yMin, yMax, '#ff9f43');
  }

  // Scope 4: Speed Tracking (Actual vs Target RPM)
  renderScope4() {
    const canvas = this.canvases[4];
    const ctx = this.ctxs[4];
    if (!canvas || !ctx) return;

    const seriesRpm = this.getVisibleSlice(4, 'rpm');
    const seriesRpmTgt = this.getVisibleSlice(4, 'rpm_tgt');

    const maxVal = Math.max(
      ...seriesRpm.map(Math.abs),
      ...seriesRpmTgt.map(Math.abs),
      100
    );
    const limit = Math.ceil(maxVal * 1.2 / 100) * 100;
    const yMin = -limit;
    const yMax = limit;

    this.drawGrid(ctx, canvas, yMin, yMax, ' RPM', 4);

    this.drawTrace(ctx, canvas, seriesRpmTgt, yMin, yMax, '#f1c40f', true);
    this.drawTrace(ctx, canvas, seriesRpm, yMin, yMax, '#2ed573');
  }
}

// Instantiate on load
document.addEventListener('DOMContentLoaded', () => {
  window.app = new FOCOscilloscopeStudio();
});
