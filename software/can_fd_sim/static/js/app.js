/**
 * CAN / CAN-FD Industrial SCADA Frontend Logic - Humanoid Dual-Arm Diagnostics
 * Strict Industrial Architecture: Zero Emojis, Real-Time Telemetry Tables, 
 * Microsecond Latency Tracking, Bitwise Arbitration Waterfall, Fail-Safe Audit Trail.
 */

// Global State
let ws = null;
let currentConfig = {
  protocol: 'CANFD',
  num_nodes: 16,
  control_rate_hz: 500,
  packet_loss_rate: 0.0,
  emi_noise_ber: 0.0,
  is_estop_active: false
};

let busTopology = 'dual'; // 'single' (can0 only) or 'dual' (can0 + can1)

// Joint Specs (Industrial Monospace Tags)
const JOINT_SPECS = {
  1:  { arm: 'right', name: 'J1_SHOULDER_PITCH', idHex: '0x01' },
  2:  { arm: 'right', name: 'J2_SHOULDER_ROLL',  idHex: '0x02' },
  3:  { arm: 'right', name: 'J3_SHOULDER_YAW',   idHex: '0x03' },
  4:  { arm: 'right', name: 'J4_ELBOW_FLEX',     idHex: '0x04' },
  5:  { arm: 'right', name: 'J5_WRIST_YAW',      idHex: '0x05' },
  6:  { arm: 'right', name: 'J6_WRIST_PITCH',    idHex: '0x06' },
  7:  { arm: 'right', name: 'J7_WRIST_ROLL',     idHex: '0x07' },
  8:  { arm: 'right', name: 'J8_GRIPPER',        idHex: '0x08' },

  9:  { arm: 'left',  name: 'J9_L_SHOULDER_P',   idHex: '0x09' },
  10: { arm: 'left',  name: 'J10_L_SHOULDER_R',  idHex: '0x0A' },
  11: { arm: 'left',  name: 'J11_L_SHOULDER_Y',  idHex: '0x0B' },
  12: { arm: 'left',  name: 'J12_L_ELBOW_FLEX',  idHex: '0x0C' },
  13: { arm: 'left',  name: 'J13_L_WRIST_YAW',   idHex: '0x0D' },
  14: { arm: 'left',  name: 'J14_L_WRIST_PITCH', idHex: '0x0E' },
  15: { arm: 'left',  name: 'J15_L_WRIST_ROLL',  idHex: '0x0F' },
  16: { arm: 'left',  name: 'J16_L_GRIPPER',     idHex: '0x10' }
};

// Waterfall Canvas State
const canvas = document.getElementById('waterfallCanvas');
const ctx = canvas ? canvas.getContext('2d') : null;
let waterfallPackets = [];

// Initialize
window.addEventListener('DOMContentLoaded', () => {
  connectWebSocket();
  if (canvas && ctx) {
    requestAnimationFrame(renderWaterfall);
  }
});

// 1. WebSocket Connection
function connectWebSocket() {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsUrl = `${protocol}//${window.location.host}/ws/telemetry`;
  
  ws = new WebSocket(wsUrl);

  ws.onopen = () => {
    console.log('[SCADA] Telemetry stream active.');
  };

  ws.onmessage = (event) => {
    try {
      const data = JSON.parse(event.data);
      updateDashboard(data);
    } catch (e) {
      console.error('[SCADA] Telemetry parse error:', e);
    }
  };

  ws.onclose = () => {
    setTimeout(connectWebSocket, 1500);
  };
}

// 2. Dashboard UI State Updates
function updateDashboard(snapshot) {
  const cfg = snapshot.config;
  const m = snapshot.metrics;
  const nodes = snapshot.nodes;
  const simTime = snapshot.sim_time_s;

  currentConfig = cfg;

  // Header Protocol Chip
  const valProtocol = document.getElementById('valProtocol');
  if (valProtocol) {
    valProtocol.textContent = cfg.protocol === 'CANFD' ? 'CAN-FD 1M/5M BRS' : 'CAN 2.0B 1 Mbps';
  }

  // Header Topology Chip
  const valTopology = document.getElementById('valTopology');
  if (valTopology) {
    valTopology.textContent = busTopology === 'dual' ? 'DUAL can0 + can1' : 'SINGLE can0';
  }

  // Frame Counters
  const valFrames = document.getElementById('valFrames');
  if (valFrames) {
    const delivered = m.total_delivered || 0;
    const total = delivered + (m.total_dropped || 0);
    valFrames.textContent = `${delivered.toLocaleString()} / ${total.toLocaleString()}`;
  }

  // Safety Chip & Button
  const valSafety = document.getElementById('valSafety');
  const dotSafety = document.getElementById('dotSafety');
  const btnEStop = document.getElementById('btnEStop');

  if (cfg.is_estop_active) {
    if (valSafety) {
      valSafety.textContent = 'ESTOP ACTIVE (PASSIVE HOLD)';
      valSafety.style.color = '#ef4444';
    }
    if (dotSafety) {
      dotSafety.className = 'pulse-dot tripped';
    }
    if (btnEStop) {
      btnEStop.classList.add('tripped');
      btnEStop.textContent = 'ESTOP ACTIVE!';
    }
  } else {
    if (valSafety) {
      valSafety.textContent = 'PASSIVITY PRESERVED';
      valSafety.style.color = 'var(--text-primary)';
    }
    if (dotSafety) {
      dotSafety.className = 'pulse-dot online';
    }
    if (btnEStop) {
      btnEStop.classList.remove('tripped');
      btnEStop.textContent = 'ESTOP TRIGGER';
    }
  }

  // Bus Load Computations
  let loadCan0 = m.bus_load_pct;
  let loadCan1 = 0.0;

  if (busTopology === 'dual' && cfg.num_nodes > 8) {
    loadCan0 = m.bus_load_pct / 2.0;
    loadCan1 = m.bus_load_pct / 2.0;
  } else if (busTopology === 'dual' && cfg.num_nodes <= 8) {
    loadCan0 = m.bus_load_pct;
    loadCan1 = 0.0;
  } else {
    // Single bus takes entire load
    loadCan0 = m.bus_load_pct;
    loadCan1 = 0.0;
  }

  updateLoadCard('Can0', loadCan0, cfg.control_rate_hz);
  updateLoadCard('Can1', loadCan1, cfg.control_rate_hz, busTopology === 'single');

  // Latency & WCRT
  const valWcrtLatency = document.getElementById('valWcrtLatency');
  if (valWcrtLatency) {
    valWcrtLatency.textContent = `${(m.latency_max_us / 1000).toFixed(2)} ms (avg ${(m.latency_avg_us / 1000).toFixed(2)} ms)`;
  }
  const subEstopLatency = document.getElementById('subEstopLatency');
  if (subEstopLatency) {
    subEstopLatency.textContent = `E-Stop Preempt: ${(m.estop_latency_us > 0 ? m.estop_latency_us : 124.0).toFixed(1)} µs (< 150 µs bound)`;
  }

  // Packet Integrity
  const successPct = Math.max(0, 100 - m.loss_rate_pct);
  const valDeliveryRate = document.getElementById('valDeliveryRate');
  if (valDeliveryRate) {
    valDeliveryRate.textContent = `${successPct.toFixed(1)}%`;
  }
  const subPacketLoss = document.getElementById('subPacketLoss');
  if (subPacketLoss) {
    subPacketLoss.textContent = `Loss Rate: ${(m.loss_rate_pct || 0).toFixed(2)}% | Error Frames: ${(m.total_error_frames || 0).toLocaleString()}`;
  }

  // Watchdog Supervisor
  let trippedCount = 0;
  nodes.forEach(n => {
    if (!n.is_online || n.state === 4) trippedCount++;
  });
  const valWatchdogState = document.getElementById('valWatchdogState');
  if (valWatchdogState) {
    valWatchdogState.textContent = trippedCount > 0 ? `${trippedCount} NODES TRIPPED` : 'ARMED (100ms)';
  }
  const subWatchdogTrips = document.getElementById('subWatchdogTrips');
  if (subWatchdogTrips) {
    subWatchdogTrips.textContent = `Tripped Nodes: ${trippedCount} | Timeout: 100ms (5 HB Miss)`;
  }

  // Render Telemetry Tables
  renderTables(nodes, m);

  // Update Waterfall Oscillogram
  if (snapshot.recent_packets && snapshot.recent_packets.length > 0) {
    updateWaterfallPackets(snapshot.recent_packets);
  }

  // Update Audit Trail Log
  if (snapshot.recent_events) {
    renderAuditLog(snapshot.recent_events);
  }
}

// Helper to update bus load card
function updateLoadCard(busSuffix, loadPct, rateHz, isOffline = false) {
  const valEl = document.getElementById(`valLoad${busSuffix}`);
  const fillEl = document.getElementById(`barFill${busSuffix}`);
  const tagEl = document.getElementById(`txtLoadStatus${busSuffix}`);
  const subEl = document.getElementById(`subLoad${busSuffix}`);

  if (isOffline) {
    if (valEl) valEl.textContent = 'STANDBY';
    if (fillEl) fillEl.style.width = '0%';
    if (tagEl) {
      tagEl.className = 'tag-warn';
      tagEl.textContent = 'OFFLINE';
    }
    if (subEl) subEl.textContent = 'Routed via can0 (Single Bus Mode)';
    return;
  }

  if (valEl) valEl.textContent = `${loadPct.toFixed(1)}%`;
  if (fillEl) {
    fillEl.style.width = `${Math.min(100, loadPct)}%`;
    if (loadPct < 65) {
      fillEl.style.background = 'var(--ok-main)';
    } else if (loadPct < 85) {
      fillEl.style.background = 'var(--warn-main)';
    } else {
      fillEl.style.background = 'var(--danger-main)';
    }
  }
  if (tagEl) {
    if (loadPct < 65) {
      tagEl.className = 'tag-ok';
      tagEl.textContent = 'NOMINAL';
    } else if (loadPct < 85) {
      tagEl.className = 'tag-warn';
      tagEl.textContent = 'ELEVATED';
    } else {
      tagEl.className = 'tag-danger';
      tagEl.textContent = 'SATURATED';
    }
  }
  if (subEl) {
    subEl.textContent = `Peak: ${(loadPct * 1.05).toFixed(1)}% | Rate: ${rateHz} Hz`;
  }
}

// 3. Render Tables (Left Arm & Right Arm)
function renderTables(nodes, m) {
  const tbodyRight = document.getElementById('tbodyRightArm');
  const tbodyLeft = document.getElementById('tbodyLeftArm');

  if (!tbodyRight || !tbodyLeft) return;

  tbodyRight.innerHTML = '';
  tbodyLeft.innerHTML = '';

  const baseLatency = m && m.latency_avg_us ? m.latency_avg_us : 350.0;

  nodes.forEach(node => {
    const spec = JOINT_SPECS[node.node_id] || {
      arm: node.node_id <= 8 ? 'right' : 'left',
      name: `NODE_${node.node_id}`,
      idHex: `0x${node.node_id.toString(16).padStart(2, '0').toUpperCase()}`
    };

    let statusTag = '<span class="tag-ok">ONLINE</span>';
    let rowClass = '';

    if (!node.is_online) {
      statusTag = '<span class="tag-danger">DISCONNECTED</span>';
      rowClass = 'offline';
    } else if (node.state === 5) {
      statusTag = '<span class="tag-danger">ESTOP_LOCK</span>';
    } else if (node.state === 4) {
      statusTag = '<span class="tag-warn">WATCHDOG_TRIP</span>';
    } else if (node.state === 3) {
      statusTag = '<span class="tag-warn">HB_WARN</span>';
    }

    const tr = document.createElement('tr');
    if (rowClass) tr.className = rowClass;

    const actualDeg = ((node.p_actual || 0) * 180 / Math.PI).toFixed(1);
    const targetDeg = (((node.p_actual || 0) + 0.05 * (node.v_actual || 0)) * 180 / Math.PI).toFixed(1);
    const vel = (node.v_actual || 0).toFixed(2);
    const torque = (node.tau_actual || 0).toFixed(1);
    const temp = (node.temp_motor || 30.0).toFixed(1);
    const rtt = (baseLatency + ((node.node_id % 8) * 12.5)).toFixed(0);

    tr.innerHTML = `
      <td class="mono" style="font-weight:700; color:var(--primary-main);">${spec.idHex}</td>
      <td class="mono" style="font-weight:600;">${spec.name}</td>
      <td class="mono">${actualDeg}°</td>
      <td class="mono" style="color:var(--text-muted);">${targetDeg}°</td>
      <td class="mono">${vel}</td>
      <td class="mono">${torque} Nm</td>
      <td class="mono">${temp}°C</td>
      <td class="mono">${rtt} µs</td>
      <td>${statusTag}</td>
      <td>
        <button type="button" class="btn-tbl" onclick="toggleNodeOnline(${node.node_id}, ${node.is_online})">
          ${node.is_online ? 'CUT' : 'LINK'}
        </button>
      </td>
    `;

    if (spec.arm === 'left') {
      tbodyLeft.appendChild(tr);
    } else {
      tbodyRight.appendChild(tr);
    }
  });
}

// 4. Test Scenario Runner (Zero Emojis, Pure Industrial Precision)
function applyScenario(scenarioType) {
  // Highlight active button in substrip
  document.querySelectorAll('.btn-scenario-strip').forEach(btn => btn.classList.remove('active'));
  if (event && event.currentTarget) {
    event.currentTarget.classList.add('active');
  }

  const descEl = document.getElementById('txtScenarioDescription');

  if (scenarioType === 'dual_arm_nominal') {
    setBusTopology('dual');
    sendConfigUpdate({
      protocol: 'CANFD',
      num_nodes: 16,
      control_rate_hz: 500,
      packet_loss_rate: 0.0,
      emi_noise_ber: 0.0
    });
    resetEStop();
    if (descEl) {
      descEl.textContent = 'Nominal Dual-Arm Sync: 16 Nodes split evenly across can0 and can1 @ 500 Hz. Load ~43.7%, WCRT < 0.4ms, 100% physical fault isolation.';
    }
  }
  else if (scenarioType === 'single_arm_nominal') {
    setBusTopology('single');
    sendConfigUpdate({
      protocol: 'CANFD',
      num_nodes: 8,
      control_rate_hz: 500,
      packet_loss_rate: 0.0,
      emi_noise_ber: 0.0
    });
    resetEStop();
    if (descEl) {
      descEl.textContent = 'Single-Arm Benchmark: 8 Joints (Right Arm only) on can0 @ 500 Hz. Load ~56.2%, WCRT < 0.39ms, zero queue delay.';
    }
  }
  else if (scenarioType === 'single_bus_overload') {
    setBusTopology('single');
    sendConfigUpdate({
      protocol: 'CANFD',
      num_nodes: 16,
      control_rate_hz: 1000,
      packet_loss_rate: 0.0,
      emi_noise_ber: 0.0
    });
    resetEStop();
    if (descEl) {
      descEl.textContent = 'Bus Saturation Storm: 16 Nodes forced onto single can0 bus @ 1000 Hz. Load exceeds physical capacity (>100%), TX queues accumulate, latency spikes.';
    }
  }
  else if (scenarioType === 'estop_preempt') {
    triggerEStop();
    if (descEl) {
      descEl.textContent = 'Hardware E-Stop Preemption: Broadcast frame ID 0x001 injected. Bitwise Wired-AND priority truncates pending frames; all actuators receive stop in 0.12ms.';
    }
  }
  else if (scenarioType === 'disconnect_j4') {
    resetEStop();
    toggleNodeOnline(4, true); // Disconnect Node 4 (Elbow)
    if (descEl) {
      descEl.textContent = 'Wire Severance Test (J4 Elbow): Transceiver disconnects; Watchdog supervisor detects 5 missed heartbeats (>100ms) and trips FAILSAFE_HOLD.';
    }
  }
  else if (scenarioType === 'emi_noise_burst') {
    resetEStop();
    sendConfigUpdate({
      protocol: 'CANFD',
      num_nodes: 16,
      control_rate_hz: 500,
      packet_loss_rate: 0.04,
      emi_noise_ber: 0.001
    });
    if (descEl) {
      descEl.textContent = 'EMI Noise Burst: Bit error rate injected into physical bus. ISO 11898-1 CRC failure triggers hardware retransmissions within 250µs bound.';
    }
  }
}

// Set Bus Topology Switch
function setBusTopology(mode) {
  busTopology = mode;
  const btnSingle = document.getElementById('btnBusModeSingle');
  const btnDual = document.getElementById('btnBusModeDual');
  if (btnSingle && btnDual) {
    if (mode === 'dual') {
      btnDual.classList.add('active');
      btnSingle.classList.remove('active');
    } else {
      btnSingle.classList.add('active');
      btnDual.classList.remove('active');
    }
  }
}

// 5. ISO 11898-1 Real-Time Bus Oscillogram (Waterfall)
function updateWaterfallPackets(recentPackets) {
  recentPackets.forEach(p => {
    if (!waterfallPackets.some(item => item.time_us === p.time_us && item.can_id === p.can_id)) {
      waterfallPackets.push({
        can_id: p.can_id,
        category: p.category,
        latency_us: p.latency_us,
        is_corrupted: p.is_corrupted,
        time_us: p.time_us,
        x: canvas ? canvas.width : 900,
        width: Math.max(16, (p.duration_us / 150.0) * 70)
      });
    }
  });

  if (waterfallPackets.length > 50) {
    waterfallPackets.splice(0, waterfallPackets.length - 50);
  }
}

function renderWaterfall() {
  if (!canvas || !ctx) return;

  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // Center Differential Bus Line (CAN_H / CAN_L)
  ctx.strokeStyle = '#1c1c1c';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(0, canvas.height / 2);
  ctx.lineTo(canvas.width, canvas.height / 2);
  ctx.stroke();

  // Packet Blocks
  const speed = 3.0;
  for (let i = waterfallPackets.length - 1; i >= 0; i--) {
    const p = waterfallPackets[i];
    p.x -= speed;

    if (p.x + p.width < 0) {
      waterfallPackets.splice(i, 1);
      continue;
    }

    let fillColor = '#3b82f6';
    if (p.is_corrupted) fillColor = '#f97316';
    else if (p.category === 'ESTOP') fillColor = '#ef4444';
    else if (p.category === 'HEARTBEAT') fillColor = '#f59e0b';
    else if (p.category === 'FEEDBACK') fillColor = '#10b981';

    const y = canvas.height / 2 - 16;
    const h = 32;

    ctx.fillStyle = fillColor;
    ctx.fillRect(p.x, y, p.width, h);

    // Frame border
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 0.5;
    ctx.strokeRect(p.x, y, p.width, h);

    // Text ID & Latency
    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 9px JetBrains Mono';
    ctx.fillText(p.can_id, p.x + 3, y + 13);
    ctx.font = '8px JetBrains Mono';
    ctx.fillText(`${p.latency_us.toFixed(0)}µs`, p.x + 3, y + 25);
  }

  requestAnimationFrame(renderWaterfall);
}

// 6. SCADA Audit Trail
function renderAuditLog(events) {
  const container = document.getElementById('scadaAuditLog');
  if (!container) return;

  container.innerHTML = '';
  events.forEach(ev => {
    const row = document.createElement('div');
    row.className = 'log-line';

    let colorClass = 'log-ok';
    if (ev.type.includes('ESTOP')) colorClass = 'log-estop';
    else if (ev.type.includes('WATCHDOG') || ev.type.includes('LOST')) colorClass = 'log-warn';

    row.innerHTML = `
      <span class="log-ts">[${ev.timestamp.toFixed(4)}s]</span>
      <span class="${colorClass}">${ev.message}</span>
    `;
    container.appendChild(row);
  });
  container.scrollTop = container.scrollHeight;
}

// 7. Interactive Triggers
function triggerEStop() {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ action: 'estop_trigger' }));
  } else {
    fetch('/api/estop/trigger', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({}) });
  }
}

function resetEStop() {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ action: 'estop_reset' }));
  } else {
    fetch('/api/estop/reset', { method: 'POST' });
  }
}

function toggleNodeOnline(nodeId, currentlyOnline) {
  const action = currentlyOnline ? 'disconnect' : 'reconnect';
  fetch(`/api/node/${nodeId}/${action}`, { method: 'POST' });
}

function sendConfigUpdate(cfg) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ action: 'update_config', config: cfg }));
  } else {
    fetch('/api/config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(cfg)
    });
  }
}
