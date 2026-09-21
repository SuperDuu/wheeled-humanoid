/**
 * CAN / CAN-FD Humanoid Robotics Bus Simulator Frontend Logic
 * Real-Time WebSocket Streaming, Canvas Waterfall Oscillogram, Topology & Charts
 */

// Global State
let ws = null;
let currentConfig = {
  protocol: 'CANFD',
  num_nodes: 8,
  control_rate_hz: 1000,
  packet_loss_rate: 0.0,
  emi_noise_ber: 0.0,
  is_estop_active: false
};

let busChart = null;
const chartDataPoints = 30;
const chartLabels = [];
const dataBusLoad = [];
const dataLatencyAvg = [];
const dataLatencyP99 = [];

// Waterfall Canvas State
const canvas = document.getElementById('waterfallCanvas');
const ctx = canvas.getContext('2d');
let waterfallPackets = [];

// Initialize
window.addEventListener('DOMContentLoaded', () => {
  initChart();
  connectWebSocket();
  requestAnimationFrame(renderWaterfall);
});

// 1. WebSocket Connection
function connectWebSocket() {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsUrl = `${protocol}//${window.location.host}/ws/telemetry`;
  
  ws = new WebSocket(wsUrl);

  ws.onopen = () => {
    console.log('✅ WebSocket Connected to Simulator backend.');
  };

  ws.onmessage = (event) => {
    try {
      const data = JSON.parse(event.data);
      updateDashboard(data);
    } catch (e) {
      console.error('Error parsing snapshot:', e);
    }
  };

  ws.onclose = () => {
    console.warn('⚠️ WebSocket disconnected, retrying in 1.5s...');
    setTimeout(connectWebSocket, 1500);
  };
}

// 2. Dashboard UI Updates
function updateDashboard(snapshot) {
  const cfg = snapshot.config;
  const m = snapshot.metrics;
  const nodes = snapshot.nodes;
  const simTime = snapshot.sim_time_s;

  currentConfig = cfg;

  // Header Sim Time & Protocol status
  document.getElementById('txtSimTime').textContent = `Sim Time: ${simTime.toFixed(3)}s`;
  updateProtocolButtons(cfg.protocol);
  
  // E-Stop Button State
  const btnEStop = document.getElementById('btnEStop');
  if (cfg.is_estop_active) {
    btnEStop.classList.add('tripped');
    btnEStop.innerHTML = '🛑 E-STOP ACTIVE (TRIPPED)';
  } else {
    btnEStop.classList.remove('tripped');
    btnEStop.innerHTML = '🚨 EMERGENCY STOP';
  }

  // KPIs
  document.getElementById('valBusLoad').textContent = `${m.bus_load_pct.toFixed(1)} %`;
  document.getElementById('fillBusLoad').style.width = `${Math.min(100, m.bus_load_pct)}%`;
  
  // Color zone for bus load
  const cardBusLoad = document.getElementById('cardBusLoad');
  const fillBusLoad = document.getElementById('fillBusLoad');
  if (m.bus_load_pct < 65) {
    cardBusLoad.className = 'kpi-card glass-panel success';
    fillBusLoad.style.background = 'linear-gradient(90deg, #10b981, #00f0ff)';
    document.getElementById('txtBusLoadPeak').textContent = `Peak: ${m.bus_load_peak_pct}% | Status: SAFE (Optimal)`;
  } else if (m.bus_load_pct < 85) {
    cardBusLoad.className = 'kpi-card glass-panel warning';
    fillBusLoad.style.background = 'linear-gradient(90deg, #f59e0b, #ef4444)';
    document.getElementById('txtBusLoadPeak').textContent = `Peak: ${m.bus_load_peak_pct}% | Status: HIGH LOAD (Jitter Risk)`;
  } else {
    cardBusLoad.className = 'kpi-card glass-panel danger';
    fillBusLoad.style.background = 'linear-gradient(90deg, #ef4444, #7f1d1d)';
    document.getElementById('txtBusLoadPeak').textContent = `Peak: ${m.bus_load_peak_pct}% | Status: SATURATED (Packet Drops)`;
  }

  document.getElementById('valLatencyAvg').textContent = `${m.latency_avg_us.toFixed(1)} µs`;
  document.getElementById('txtJitter').textContent = `Jitter (σ): ${m.latency_jitter_us.toFixed(1)} µs`;

  document.getElementById('valLatencyP99').textContent = `${m.latency_p99_us.toFixed(1)} µs`;
  document.getElementById('txtLatencyMax').textContent = `Max Seen: ${m.latency_max_us.toFixed(1)} µs`;

  if (m.estop_latency_us > 0) {
    document.getElementById('valEStopLat').textContent = `${m.estop_latency_us.toFixed(1)} µs`;
  }

  document.getElementById('valThroughput').textContent = `${m.throughput_kbps.toFixed(1)} kbps`;
  document.getElementById('valDeliveredCount').textContent = m.total_delivered.toLocaleString();
  document.getElementById('txtLossInfo').textContent = `Errors: ${m.total_error_frames} | Loss: ${m.loss_rate_pct.toFixed(2)}%`;

  // Baud text
  if (cfg.protocol === 'CANFD') {
    document.getElementById('txtBaudInfo').textContent = 'CAN-FD: 1M Nom / 5M Data';
  } else {
    document.getElementById('txtBaudInfo').textContent = 'Classical CAN: 1M Fixed';
  }

  // Update Nodes Grid
  renderNodesGrid(nodes);

  // Update Waterfall Packets
  if (snapshot.recent_packets && snapshot.recent_packets.length > 0) {
    updateWaterfallPackets(snapshot.recent_packets);
  }

  // Update Event Log
  if (snapshot.recent_events) {
    renderEventLog(snapshot.recent_events);
  }

  // Update Real-Time Charts
  pushChartData(simTime, m.bus_load_pct, m.latency_avg_us, m.latency_p99_us);
}

// 3. Render Nodes Topology Grid
function renderNodesGrid(nodes) {
  const container = document.getElementById('nodesGrid');
  if (!container) return;

  // Render cards if count changed
  if (container.children.length !== nodes.length) {
    container.innerHTML = '';
    nodes.forEach(node => {
      const card = document.createElement('div');
      card.id = `nodeCard_${node.node_id}`;
      card.className = 'node-card';
      container.appendChild(card);
    });
  }

  nodes.forEach(node => {
    const card = document.getElementById(`nodeCard_${node.node_id}`);
    if (!card) return;

    let badgeClass = 'badge-running';
    let ledClass = 'heartbeat-led';
    if (!node.is_online) {
      badgeClass = 'badge-offline';
      ledClass = 'heartbeat-led tripped';
      card.className = 'node-card offline';
    } else if (node.state === 5) { // ESTOP_TRIPPED
      badgeClass = 'badge-failsafe';
      ledClass = 'heartbeat-led tripped';
      card.className = 'node-card';
    } else if (node.state === 4) { // FAILSAFE_HOLD
      badgeClass = 'badge-failsafe';
      ledClass = 'heartbeat-led warning';
      card.className = 'node-card';
    } else if (node.state === 3) { // WARNING
      badgeClass = 'badge-warning';
      ledClass = 'heartbeat-led warning';
      card.className = 'node-card';
    } else {
      card.className = 'node-card';
    }

    card.innerHTML = `
      <div class="node-card-header">
        <span class="node-name">${node.name}</span>
        <span class="${ledClass}" title="Last HB: ${node.time_since_hb_ms}ms ago"></span>
      </div>
      <span class="node-status-badge ${badgeClass}">${node.state_name}</span>
      <div class="node-metrics">
        <span>Pos: <b>${node.p_actual.toFixed(2)}</b>r</span>
        <span>Vel: <b>${node.v_actual.toFixed(1)}</b>r/s</span>
        <span>Tau: <b>${node.tau_actual.toFixed(1)}</b>Nm</span>
        <span>Iq: <b>${node.iq_actual.toFixed(1)}</b>A</span>
        <span>FET: <b>${node.temp_fet.toFixed(0)}</b>°C</span>
        <span>HB: <b>${node.time_since_hb_ms.toFixed(0)}</b>ms</span>
      </div>
      <div class="node-actions">
        <button class="btn-node-toggle" onclick="toggleNodeOnline(${node.node_id}, ${node.is_online})">
          ${node.is_online ? '💥 Disconnect' : '🟢 Reconnect'}
        </button>
      </div>
    `;
  });
}

// 4. Animated Waterfall Oscillogram
function updateWaterfallPackets(recentPackets) {
  recentPackets.forEach(p => {
    // Add unique by timestamp
    if (!waterfallPackets.some(item => item.time_us === p.time_us && item.can_id === p.can_id)) {
      waterfallPackets.push({
        can_id: p.can_id,
        category: p.category,
        sender_id: p.sender_id,
        dlc: p.dlc,
        duration_us: p.duration_us,
        latency_us: p.latency_us,
        is_corrupted: p.is_corrupted,
        time_us: p.time_us,
        x: canvas.width,
        width: Math.max(12, (p.duration_us / 150.0) * 80)
      });
    }
  });

  if (waterfallPackets.length > 50) {
    waterfallPackets.splice(0, waterfallPackets.length - 50);
  }
}

function renderWaterfall() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // Background Grid Lines
  ctx.strokeStyle = 'rgba(64, 93, 138, 0.15)';
  ctx.lineWidth = 1;
  for (let x = 0; x < canvas.width; x += 40) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, canvas.height);
    ctx.stroke();
  }

  // Center bus channel wire
  ctx.strokeStyle = 'rgba(0, 240, 255, 0.3)';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(0, canvas.height / 2);
  ctx.lineTo(canvas.width, canvas.height / 2);
  ctx.stroke();

  // Draw packet blocks moving from right to left
  const speed = 2.5;
  for (let i = waterfallPackets.length - 1; i >= 0; i--) {
    const p = waterfallPackets[i];
    p.x -= speed;

    if (p.x + p.width < 0) {
      waterfallPackets.splice(i, 1);
      continue;
    }

    // Color by category
    let fillColor = '#3b82f6';
    if (p.is_corrupted) {
      fillColor = '#f97316';
    } else if (p.category === 'ESTOP') {
      fillColor = '#ef4444';
    } else if (p.category === 'HEARTBEAT') {
      fillColor = '#f59e0b';
    } else if (p.category === 'FEEDBACK') {
      fillColor = '#10b981';
    }

    const y = canvas.height / 2 - 25;
    const h = 50;

    // Packet Glow & Rectangle
    ctx.fillStyle = fillColor;
    ctx.shadowColor = fillColor;
    ctx.shadowBlur = p.category === 'ESTOP' ? 15 : 6;
    ctx.beginPath();
    ctx.roundRect(p.x, y, p.width, h, 4);
    ctx.fill();
    ctx.shadowBlur = 0;

    // Border
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1;
    ctx.stroke();

    // Packet Label
    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 10px JetBrains Mono';
    ctx.fillText(p.can_id, p.x + 4, y + 20);
    ctx.font = '9px JetBrains Mono';
    ctx.fillText(`${p.latency_us.toFixed(0)}µs`, p.x + 4, y + 36);
  }

  requestAnimationFrame(renderWaterfall);
}

// 5. Chart.js Real-Time History
function initChart() {
  const chartCtx = document.getElementById('busChart').getContext('2d');
  busChart = new Chart(chartCtx, {
    type: 'line',
    data: {
      labels: chartLabels,
      datasets: [
        {
          label: 'Bus Load (%)',
          data: dataBusLoad,
          borderColor: '#00f0ff',
          backgroundColor: 'rgba(0, 240, 255, 0.1)',
          borderWidth: 2,
          tension: 0.3,
          fill: true,
          yAxisID: 'yLoad'
        },
        {
          label: 'Avg Latency (µs)',
          data: dataLatencyAvg,
          borderColor: '#10b981',
          borderWidth: 1.5,
          tension: 0.2,
          fill: false,
          yAxisID: 'yLat'
        },
        {
          label: 'P99 Latency (µs)',
          data: dataLatencyP99,
          borderColor: '#f59e0b',
          borderWidth: 1.5,
          borderDash: [4, 4],
          tension: 0.2,
          fill: false,
          yAxisID: 'yLat'
        }
      ]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      scales: {
        x: {
          display: false
        },
        yLoad: {
          type: 'linear',
          position: 'left',
          min: 0,
          max: 100,
          grid: { color: 'rgba(64, 93, 138, 0.15)' },
          ticks: { color: '#94a3b8', callback: v => v + '%' }
        },
        yLat: {
          type: 'linear',
          position: 'right',
          min: 0,
          grid: { drawOnChartArea: false },
          ticks: { color: '#94a3b8', callback: v => v + 'µs' }
        }
      },
      plugins: {
        legend: {
          labels: { color: '#f1f5f9', font: { family: 'JetBrains Mono', size: 11 } }
        }
      }
    }
  });
}

function pushChartData(time_s, load_pct, lat_avg, lat_p99) {
  if (!busChart) return;

  chartLabels.push(time_s.toFixed(1));
  dataBusLoad.push(load_pct);
  dataLatencyAvg.push(lat_avg);
  dataLatencyP99.push(lat_p99);

  if (chartLabels.length > chartDataPoints) {
    chartLabels.shift();
    dataBusLoad.shift();
    dataLatencyAvg.shift();
    dataLatencyP99.shift();
  }

  busChart.update('none');
}

// 6. Terminal Event Log
function renderEventLog(events) {
  const container = document.getElementById('eventLogContainer');
  if (!container) return;

  container.innerHTML = '';
  events.forEach(ev => {
    const row = document.createElement('div');
    row.className = 'log-entry';
    let typeClass = 'log-info';
    if (ev.type.includes('ESTOP')) typeClass = 'log-estop';
    else if (ev.type.includes('WATCHDOG') || ev.type.includes('BUS_OFF')) typeClass = 'log-warning';

    row.innerHTML = `
      <span class="log-time">[${ev.timestamp.toFixed(3)}s]</span>
      <span class="${typeClass}">${ev.message}</span>
    `;
    container.appendChild(row);
  });
  container.scrollTop = container.scrollHeight;
}

// 7. Interactive Controls & API Calls
function setProtocol(protocol) {
  updateProtocolButtons(protocol);
  sendConfigUpdate({ protocol });
}

function updateProtocolButtons(protocol) {
  const btnCAN20 = document.getElementById('btnProtoCAN20');
  const btnCANFD = document.getElementById('btnProtoCANFD');
  if (protocol === 'CANFD') {
    btnCANFD.classList.add('active');
    btnCAN20.classList.remove('active');
  } else {
    btnCAN20.classList.add('active');
    btnCANFD.classList.remove('active');
  }
}

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

function updateControls() {
  const num_nodes = parseInt(document.getElementById('sliderNodes').value);
  const control_rate_hz = parseInt(document.getElementById('sliderRate').value);
  const packet_loss_rate = parseFloat(document.getElementById('sliderLoss').value);
  const emi_noise_ber = parseFloat(document.getElementById('sliderBER').value);

  document.getElementById('lblNumNodes').textContent = `${num_nodes} Nodes`;
  document.getElementById('lblControlHz').textContent = `${control_rate_hz} Hz`;
  document.getElementById('lblLossRate').textContent = `${(packet_loss_rate * 100).toFixed(1)} %`;
  document.getElementById('lblBER').textContent = emi_noise_ber.toExponential(1);

  sendConfigUpdate({
    num_nodes,
    control_rate_hz,
    packet_loss_rate,
    emi_noise_ber
  });
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

function injectBurstNoise() {
  const sliderLoss = document.getElementById('sliderLoss');
  sliderLoss.value = 0.05;
  updateControls();
  setTimeout(() => {
    sliderLoss.value = 0.0;
    updateControls();
  }, 2500);
}

function disconnectRandomNode() {
  const targetId = Math.floor(Math.random() * currentConfig.num_nodes) + 1;
  toggleNodeOnline(targetId, true);
}

// 8. Benchmark Modal
function openBenchmarkModal() {
  const modal = document.getElementById('benchmarkModal');
  const loading = document.getElementById('benchmarkLoading');
  const content = document.getElementById('benchmarkContent');
  modal.style.display = 'flex';
  loading.style.display = 'block';
  content.style.display = 'none';

  fetch('/api/benchmark')
    .then(r => r.json())
    .then(data => {
      loading.style.display = 'none';
      content.style.display = 'block';

      const tbody = document.getElementById('benchmarkTableBody');
      tbody.innerHTML = '';
      data.benchmark_results.forEach(row => {
        const tr = document.createElement('tr');
        let statusBadge = `<span style="color: #34d399; font-weight: bold;">${row.status}</span>`;
        if (row.status.includes('UNSTABLE')) {
          statusBadge = `<span style="color: #f87171; font-weight: bold;">${row.status}</span>`;
        } else if (row.status.includes('MARGINAL')) {
          statusBadge = `<span style="color: #fbbf24; font-weight: bold;">${row.status}</span>`;
        }

        tr.innerHTML = `
          <td><b>${row.protocol}</b></td>
          <td>${row.nodes} joints</td>
          <td>${row.rate_hz} Hz</td>
          <td>${row.bus_load_pct.toFixed(1)}%</td>
          <td>${row.throughput_kbps.toFixed(1)} kbps</td>
          <td>${row.latency_avg_us.toFixed(1)} µs</td>
          <td>${row.latency_p99_us.toFixed(1)} µs</td>
          <td>${statusBadge}</td>
        `;
        tbody.appendChild(tr);
      });

      const recList = document.getElementById('recommendationList');
      recList.innerHTML = `
        <li><b>Phần cứng đề xuất:</b> ${data.recommendation.real_hardware_mode} (${data.recommendation.nominal_baud} / ${data.recommendation.data_baud}).</li>
        <li><b>Cấu hình tối ưu cho 1 cánh tay (8 khớp):</b> ${data.recommendation.optimal_rate_single_arm_8_nodes}.</li>
        <li><b>Giới hạn Classical CAN 2.0B:</b> ${data.recommendation.classical_can_limit}.</li>
      `;
    })
    .catch(err => {
      loading.textContent = '❌ Lỗi khi chạy benchmark: ' + err.message;
    });
}

function closeBenchmarkModal() {
  document.getElementById('benchmarkModal').style.display = 'none';
}
