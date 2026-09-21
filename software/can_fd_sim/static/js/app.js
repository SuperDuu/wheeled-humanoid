/**
 * CAN / CAN-FD Humanoid Robotics Bus Simulator Frontend Logic
 * Intuitive Visual Robot Anatomy, Traffic Metaphors, 1-Click Scenarios, and Waterfall
 */

// Global State
let ws = null;
let currentConfig = {
  protocol: 'CANFD',
  num_nodes: 8,
  control_rate_hz: 500,
  packet_loss_rate: 0.0,
  emi_noise_ber: 0.0,
  is_estop_active: false
};

let busTopology = 'single'; // 'single' (1 bus) or 'dual' (2 buses)

// Waterfall Canvas State
const canvas = document.getElementById('waterfallCanvas');
const ctx = canvas.getContext('2d');
let waterfallPackets = [];

// Joint Anatomy Mapping
const JOINT_SPECS = {
  1: { arm: 'right', name: 'Khớp Vai 1 (Pitch)', icon: '🦾', idHex: '0x01' },
  2: { arm: 'right', name: 'Khớp Vai 2 (Roll)',  icon: '🦾', idHex: '0x02' },
  3: { arm: 'right', name: 'Khớp Vai 3 (Yaw)',   icon: '🦾', idHex: '0x03' },
  4: { arm: 'right', name: 'Khớp Khuỷu (Elbow)', icon: '🔄', idHex: '0x04' },
  5: { arm: 'right', name: 'Khớp Cổ Tay 1 (Yaw)',icon: '🖐️', idHex: '0x05' },
  6: { arm: 'right', name: 'Khớp Cổ Tay 2 (Pitch)',icon:'🖐️', idHex: '0x06' },
  7: { arm: 'right', name: 'Khớp Cổ Tay 3 (Roll)',icon:'🖐️', idHex: '0x07' },
  8: { arm: 'right', name: 'Bàn Tay Kẹp (Gripper)',icon:'🗜️', idHex: '0x08' },
  
  9: { arm: 'left', name: 'Khớp Vai Trái 1 (Pitch)', icon: '🦾', idHex: '0x09' },
  10: { arm: 'left', name: 'Khớp Vai Trái 2 (Roll)',  icon: '🦾', idHex: '0x0A' },
  11: { arm: 'left', name: 'Khớp Vai Trái 3 (Yaw)',   icon: '🦾', idHex: '0x0B' },
  12: { arm: 'left', name: 'Khớp Khuỷu Trái (Elbow)', icon: '🔄', idHex: '0x0C' },
  13: { arm: 'left', name: 'Khớp Cổ Tay Trái 1',      icon: '🖐️', idHex: '0x0D' },
  14: { arm: 'left', name: 'Khớp Cổ Tay Trái 2',      icon: '🖐️', idHex: '0x0E' },
  15: { arm: 'left', name: 'Khớp Cổ Tay Trái 3',      icon: '🖐️', idHex: '0x0F' },
  16: { arm: 'left', name: 'Kẹp Tay Trái (Gripper)',  icon: '🗜️', idHex: '0x10' },
};

// Initialize
window.addEventListener('DOMContentLoaded', () => {
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

  // Header E-Stop Button State
  const btnEStop = document.getElementById('btnEStop');
  if (cfg.is_estop_active) {
    btnEStop.classList.add('tripped');
    btnEStop.innerHTML = '🛑 ĐANG DỪNG KHẨN CẤP!';
  } else {
    btnEStop.classList.remove('tripped');
    btnEStop.innerHTML = '🚨 DỪNG KHẨN CẤP (E-STOP)';
  }

  // Rate Display
  document.getElementById('lblJetsonRate').textContent = `${cfg.control_rate_hz} Hz (${(1000/cfg.control_rate_hz).toFixed(1)} ms)`;

  // Traffic Metaphor Load & Progress Bar
  let effectiveLoad = m.bus_load_pct;
  if (busTopology === 'dual' && cfg.num_nodes > 8) {
    effectiveLoad = m.bus_load_pct / 2.0; // Split load across 2 buses!
  }

  document.getElementById('txtTrafficPct').textContent = `${effectiveLoad.toFixed(1)} %`;
  document.getElementById('trafficBarFill').style.width = `${Math.min(100, effectiveLoad)}%`;

  const badge = document.getElementById('trafficBadgeText');
  const title = document.getElementById('txtTrafficTitle');
  const detail = document.getElementById('txtTrafficDetail');
  const bar = document.getElementById('trafficBarFill');

  if (effectiveLoad < 65) {
    badge.className = 'traffic-badge-green';
    badge.textContent = '🟢 THÔNG THOÁNG';
    title.textContent = 'Đường truyền cực kỳ ổn định';
    detail.textContent = `Băng thông còn trống ${(100 - effectiveLoad).toFixed(0)}%. Các gói tin không bị xếp hàng, mô-men motor phát mượt mà không rung lắc.`;
    bar.style.background = 'linear-gradient(90deg, #10b981, #00f0ff)';
  } else if (effectiveLoad < 85) {
    badge.className = 'traffic-badge-yellow';
    badge.textContent = '🟡 BẮT ĐẦU ĐÔNG XE';
    title.textContent = 'Tải cao - Bắt đầu xuất hiện Jitter';
    detail.textContent = 'Các gói tin bắt đầu phải chờ nhau trên bus. Nếu có xung nhiễu EMI, nguy cơ rơi gói tin sẽ tăng.';
    bar.style.background = 'linear-gradient(90deg, #f59e0b, #ef4444)';
  } else {
    badge.className = 'traffic-badge-red';
    badge.textContent = '🔴 TẮC ĐƯỜNG NẶNG';
    title.textContent = 'BÃO HÒA ĐƯỜNG TRUYỀN (Quá Tải)';
    detail.textContent = 'Lưu lượng gói tin vượt quá khả năng vật lý của dây bus! Các khớp bị trễ lệnh, robot có nguy cơ giật hoặc trip bảo vệ.';
    bar.style.background = 'linear-gradient(90deg, #ef4444, #7f1d1d)';
  }

  // Human KPIs
  document.getElementById('humanLatAvg').textContent = `${(m.latency_avg_us / 1000).toFixed(2)} ms`;
  if (m.estop_latency_us > 0) {
    document.getElementById('humanEstopLat').textContent = `${(m.estop_latency_us / 1000).toFixed(2)} ms`;
  }
  const lossPct = m.loss_rate_pct;
  const successPct = Math.max(0, 100 - lossPct);
  document.getElementById('humanDelivered').textContent = `${successPct.toFixed(1)}%`;

  // Protocol Text
  document.getElementById('lblProtocolMode').textContent = cfg.protocol === 'CANFD' ? 'CAN-FD (1M Nominal / 5M Data BRS)' : 'Classical CAN 2.0B (1 Mbps)';

  // Render Visual Robot Anatomy
  renderRobotAnatomy(nodes);

  // Update Waterfall Packets
  if (snapshot.recent_packets && snapshot.recent_packets.length > 0) {
    updateWaterfallPackets(snapshot.recent_packets);
  }

  // Update Event Log
  if (snapshot.recent_events) {
    renderEventLog(snapshot.recent_events);
  }
}

// 3. Render Visual Robot Anatomy
function renderRobotAnatomy(nodes) {
  const rightList = document.getElementById('rightJointsList');
  const leftList = document.getElementById('leftJointsList');
  const leftBranch = document.getElementById('leftArmBranch');
  const indCan1 = document.getElementById('indCan1Line');
  const tagLeftBus = document.getElementById('tagLeftBus');
  const txtTopology = document.getElementById('txtTopologyNotice');

  const hasLeftArm = (currentConfig.num_nodes > 8);
  if (hasLeftArm) {
    leftBranch.style.opacity = '1.0';
    indCan1.style.opacity = '1.0';
    txtTopology.textContent = `Đang chạy: 2 Cánh Tay (${currentConfig.num_nodes} Khớp)`;
    if (busTopology === 'dual') {
      tagLeftBus.textContent = 'Tuyến can1 riêng biệt';
      tagLeftBus.style.color = '#34d399';
    } else {
      tagLeftBus.textContent = 'Chung 1 dây can0 (Dễ nghẽn)';
      tagLeftBus.style.color = '#f87171';
    }
  } else {
    leftBranch.style.opacity = '0.35';
    indCan1.style.opacity = '0.3';
    txtTopology.textContent = 'Đang chạy: 1 Cánh Tay Phải (8 Khớp)';
  }

  // Clear & Rebuild
  rightList.innerHTML = '';
  leftList.innerHTML = '';

  nodes.forEach(node => {
    const spec = JOINT_SPECS[node.node_id] || { arm: 'right', name: `Khớp ${node.node_id}`, icon: '🦾', idHex: `0x${node.node_id.toString(16)}` };
    
    let badgeHtml = '<span class="joint-status-badge badge-green">Hoạt Động</span>';
    let cardClass = 'joint-anatomy-card';

    if (!node.is_online) {
      badgeHtml = '<span class="joint-status-badge badge-red">MẤT KẾT NỐI</span>';
      cardClass += ' offline';
    } else if (node.state === 5) { // ESTOP_TRIPPED
      badgeHtml = '<span class="joint-status-badge badge-red">E-STOP KHÓA</span>';
    } else if (node.state === 4) { // FAILSAFE_HOLD
      badgeHtml = '<span class="joint-status-badge badge-yellow">TỰ HÃM SAFE</span>';
    } else if (node.state === 3) {
      badgeHtml = '<span class="joint-status-badge badge-yellow">CẢNH BÁO HB</span>';
    }

    const card = document.createElement('div');
    card.className = cardClass;
    card.innerHTML = `
      <div class="joint-left-part">
        <div class="joint-art-icon">${spec.icon}</div>
        <div class="joint-labels">
          <b>${spec.name}</b>
          <span>CAN ID: ${spec.idHex} | HB: ${node.time_since_hb_ms.toFixed(0)}ms</span>
        </div>
      </div>
      <div class="joint-right-part">
        <div class="joint-telemetry-pill">
          <div>Góc: <b>${node.p_actual.toFixed(2)}</b> rad</div>
          <div>Dòng: <b>${node.iq_actual.toFixed(1)}</b> A</div>
        </div>
        ${badgeHtml}
        <button class="btn-small-toggle" onclick="toggleNodeOnline(${node.node_id}, ${node.is_online})" title="Cắt dây thử nghiệm">
          ${node.is_online ? '✂️ Cắt' : '🔌 Nối'}
        </button>
      </div>
    `;

    if (spec.arm === 'left') {
      leftList.appendChild(card);
    } else {
      rightList.appendChild(card);
    }
  });
}

// 4. Five 1-Click Interactive Scenarios
function applyScenario(scenarioType) {
  // Highlight active button
  document.querySelectorAll('.btn-scenario').forEach(btn => btn.classList.remove('active'));
  event.currentTarget.classList.add('active');

  const story = document.getElementById('storyContent');

  if (scenarioType === 'normal_1arm') {
    busTopology = 'single';
    setBusTopology('single');
    sendConfigUpdate({
      protocol: 'CANFD',
      num_nodes: 8,
      control_rate_hz: 500,
      packet_loss_rate: 0.0,
      emi_noise_ber: 0.0
    });
    resetEStop();
    story.innerHTML = `
      <b>🕊️ Kịch bản 1: Hoạt động chuẩn 1 cánh tay (8 khớp @ 500Hz)</b><br>
      • Máy tính Jetson gửi lệnh qua FDCAN 1M/5M BRS.<br>
      • Tải bus chỉ ~56.2%, cực kỳ êm ái, trễ trung bình &lt; 0.4ms.<br>
      • Băng thông còn trống &gt;40% sẵn sàng chống chọi xung nhiễu điện từ.
    `;
  }
  else if (scenarioType === 'test_estop') {
    triggerEStop();
    story.innerHTML = `
      <b>🚑 Kịch bản 2: Dừng Khẩn Cấp E-Stop (Ưu tiên xe cứu thương)</b><br>
      • Lệnh E-Stop mang ID 0x001 (nhỏ nhất toàn mạng). Nhờ cơ chế Wired-AND vật lý, bit 0 dập tắt toàn bộ các bit khác.<br>
      • Toàn bộ 8 khớp nhận lệnh dừng trong <b>0.12 ms</b> (nhanh gấp 1000 lần một cái chớp mắt)!<br>
      • Motor lập tức triệt tiêu dòng điện $I_q=0$, tránh văng cơ khí hoặc va đập người.
    `;
  }
  else if (scenarioType === 'cut_wire') {
    resetEStop();
    toggleNodeOnline(4, true); // Cắt dây khớp 4 (Elbow)
    story.innerHTML = `
      <b>💥 Kịch bản 3: Thử cắt đứt dây kết nối Khớp Khuỷu (J4)</b><br>
      • Khớp 4 bị mất kết nối vật lý, không còn phát nhịp tim (Heartbeat).<br>
      • Sau đúng 100ms (5 nhịp tim bị thiếu), mạch bảo vệ Watchdog tự ngắt kích hoạt <code>FAILSAFE_HOLD</code>.<br>
      • Khớp chuyển sang hãm mềm (active damping), motor dừng êm ái, <b>hoàn toàn không bị rơi tự do hay mất kiểm soát nguy hiểm</b>!
    `;
  }
  else if (scenarioType === 'overload_2arms') {
    busTopology = 'single';
    setBusTopology('single');
    sendConfigUpdate({
      protocol: 'CANFD',
      num_nodes: 16,
      control_rate_hz: 1000,
      packet_loss_rate: 0.0,
      emi_noise_ber: 0.0
    });
    resetEStop();
    story.innerHTML = `
      <b>🚨 Kịch bản 4: Nhồi 2 cánh tay (16 khớp) vào CHUNG 1 DÂY BUS @ 1000Hz</b><br>
      • Tải đường truyền tăng vọt lên <b>&gt;100% (Quá tải vật lý)</b>!<br>
      • 32,800 gói tin/giây tranh chấp trên cùng 1 sợi cáp, hàng đợi bùng nổ, độ trễ tăng vọt.<br>
      • Minh chứng thực tế: <b>Tuyệt đối không nên nối 2 cánh tay vào chung 1 tuyến bus!</b>
    `;
  }
  else if (scenarioType === 'dual_bus_optimal') {
    busTopology = 'dual';
    setBusTopology('dual');
    sendConfigUpdate({
      protocol: 'CANFD',
      num_nodes: 16,
      control_rate_hz: 500,
      packet_loss_rate: 0.0,
      emi_noise_ber: 0.0
    });
    resetEStop();
    story.innerHTML = `
      <b>🌟 Kịch bản 5: Tách 2 DÂY RIÊNG BIỆT (can0 cho Tay Phải, can1 cho Tay Trái)</b><br>
      • Tận dụng 2 cổng FDCAN phần cứng sẵn có trên Jetson Orin.<br>
      • Tải bus mỗi bên giảm về mức lý tưởng <b>~43.7%</b>, đường truyền xanh mượt.<br>
      • <b>Cách ly lỗi 100%:</b> Nếu tay trái gặp tai nạn chập cáp, tay phải vẫn hoạt động bình thường để chống đỡ robot!
    `;
  }
}

function setBusTopology(mode) {
  busTopology = mode;
  const btnSingle = document.getElementById('btnBusModeSingle');
  const btnDual = document.getElementById('btnBusModeDual');
  if (mode === 'dual') {
    btnDual.classList.add('active');
    btnSingle.classList.remove('active');
  } else {
    btnSingle.classList.add('active');
    btnDual.classList.remove('active');
  }
}

// 5. Animated Waterfall Oscillogram
function updateWaterfallPackets(recentPackets) {
  recentPackets.forEach(p => {
    if (!waterfallPackets.some(item => item.time_us === p.time_us && item.can_id === p.can_id)) {
      waterfallPackets.push({
        can_id: p.can_id,
        category: p.category,
        latency_us: p.latency_us,
        is_corrupted: p.is_corrupted,
        time_us: p.time_us,
        x: canvas.width,
        width: Math.max(14, (p.duration_us / 150.0) * 80)
      });
    }
  });

  if (waterfallPackets.length > 40) {
    waterfallPackets.splice(0, waterfallPackets.length - 40);
  }
}

function renderWaterfall() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // Center bus wire
  ctx.strokeStyle = 'rgba(0, 240, 255, 0.4)';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(0, canvas.height / 2);
  ctx.lineTo(canvas.width, canvas.height / 2);
  ctx.stroke();

  // Packet Blocks
  const speed = 2.5;
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

    const y = canvas.height / 2 - 18;
    const h = 36;

    ctx.fillStyle = fillColor;
    ctx.beginPath();
    ctx.roundRect(p.x, y, p.width, h, 4);
    ctx.fill();

    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 9px JetBrains Mono';
    ctx.fillText(p.can_id, p.x + 3, y + 14);
    ctx.font = '8px JetBrains Mono';
    ctx.fillText(`${p.latency_us.toFixed(0)}µs`, p.x + 3, y + 27);
  }

  requestAnimationFrame(renderWaterfall);
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
    else if (ev.type.includes('WATCHDOG') || ev.type.includes('LOST')) typeClass = 'log-warning';

    row.innerHTML = `
      <span class="log-time">[${ev.timestamp.toFixed(3)}s]</span>
      <span class="${typeClass}">${ev.message}</span>
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
