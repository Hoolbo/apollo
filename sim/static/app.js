// ═══════════════════════════════════════════════════════════
//  铰接车实时可视化 - Canvas 渲染引擎
// ═══════════════════════════════════════════════════════════

// ── 全局状态 ──
let ws = null;
let vehicleParams = null;
let mapData = null;
let mapImageData = null;  // 缓存的地图 ImageData

// 车辆数据
let vehicle = { x: 0, y: 0, theta: 0, gamma: 0, speed: 0 };
let trail = [];
let cilqrTraj = [];
let globalPath = [];

// 控制指令 & CAN 反馈
let controlCmd = { v_front: 0, delta_front_deg: 0, v_rear: 0, delta_rear_deg: 0, timestamp: 0 };
let canFeedback = {
  front: { speed_kmh: 0, eps_angle_deg: 0, eps_enable: false, motor_enable: false, gear: 'N', driving_mode: 'MANUAL', vcu_error: false },
  rear: { speed_mps: 0, steer_angle_rad: 0, vehicle_state: 'UNKNOWN', control_mode: 'UNKNOWN', battery_v: 0, fault_high: 0, fault_low: 0, motor1_rpm: 0, motor1_current_a: 0 }
};
let hasChassisDetail = false;
let moduleStatus = {};  // { gnss: true/false, localization: true/false, ... }

// 视图状态
let camera = { x: 0, y: 0, zoom: 5.0 };  // zoom = pixels per meter
let isDragging = false;
let dragStart = { x: 0, y: 0 };
let cameraStart = { x: 0, y: 0 };
let followVehicle = true;
let goalMode = false;
let startMode = false;
let goalPos = null;

// 帧率统计
let frameCount = 0;
let lastFpsTime = Date.now();
let currentFps = 0;
let wsMessageCount = 0;
let lastWsCountTime = Date.now();
let wsHz = 0;
let lastGlobalPathLen = 0;

// Toast 通知
function showToast(msg, duration = 2000) {
  let container = document.getElementById('toast-container');
  if (!container) {
    container = document.createElement('div');
    container.id = 'toast-container';
    container.style.cssText = 'position:fixed;top:60px;left:50%;transform:translateX(-50%);z-index:9999;pointer-events:none;';
    document.body.appendChild(container);
  }
  const toast = document.createElement('div');
  toast.textContent = msg;
  toast.style.cssText = 'background:rgba(0,200,100,0.9);color:#fff;padding:10px 24px;border-radius:8px;font:600 14px Inter,sans-serif;margin-bottom:8px;opacity:0;transition:opacity 0.3s;box-shadow:0 4px 12px rgba(0,0,0,0.3);';
  container.appendChild(toast);
  requestAnimationFrame(() => { toast.style.opacity = '1'; });
  setTimeout(() => {
    toast.style.opacity = '0';
    setTimeout(() => toast.remove(), 300);
  }, duration);
}

// Canvas
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');


// ═══════════════════════════════════════════════════════════
//  坐标变换
// ═══════════════════════════════════════════════════════════

function worldToScreen(wx, wy) {
  const sx = (wx - camera.x) * camera.zoom + canvas.width / 2;
  const sy = -(wy - camera.y) * camera.zoom + canvas.height / 2;
  return [sx, sy];
}

function screenToWorld(sx, sy) {
  const wx = (sx - canvas.width / 2) / camera.zoom + camera.x;
  const wy = -(sy - canvas.height / 2) / camera.zoom + camera.y;
  return [wx, wy];
}


// ═══════════════════════════════════════════════════════════
//  地图渲染
// ═══════════════════════════════════════════════════════════

function loadMap() {
  fetch('/api/map')
    .then(r => {
      if (!r.ok) throw new Error('map not found');
      return r.json();
    })
    .then(data => {
      mapData = data;
      buildMapImage();
      console.log(`Map loaded: ${data.width}×${data.height}`);
    })
    .catch(e => console.warn('Map:', e.message));
}

function buildMapImage() {
  if (!mapData) return;
  const w = mapData.width;
  const h = mapData.height;
  const offscreen = document.createElement('canvas');
  offscreen.width = w;
  offscreen.height = h;
  const octx = offscreen.getContext('2d');
  const img = octx.createImageData(w, h);

  for (let row = 0; row < h; row++) {
    for (let col = 0; col < w; col++) {
      const val = mapData.data[row][col];
      const idx = ((h - 1 - row) * w + col) * 4;  // flip Y
      if (val < 0) {
        // 障碍物 - 黑色
        img.data[idx] = 0;
        img.data[idx + 1] = 0;
        img.data[idx + 2] = 0;
        img.data[idx + 3] = 120;
      } else {
        // 可通行 - 白色
        img.data[idx] = 255;
        img.data[idx + 1] = 255;
        img.data[idx + 2] = 255;
        img.data[idx + 3] = 200;
      }
    }
  }
  octx.putImageData(img, 0, 0);
  mapImageData = offscreen;
}

function drawMap() {
  if (!mapData || !mapImageData) return;

  const res = mapData.resolution;
  const ox = mapData.origin[0];
  const oy = mapData.origin[1];
  const w = mapData.width;
  const h = mapData.height;

  // 地图世界坐标范围
  const [sx1, sy1] = worldToScreen(ox, oy);
  const [sx2, sy2] = worldToScreen(ox + w * res, oy + h * res);

  const drawX = Math.min(sx1, sx2);
  const drawY = Math.min(sy1, sy2);
  const drawW = Math.abs(sx2 - sx1);
  const drawH = Math.abs(sy2 - sy1);

  // 避免绘制过小
  if (drawW < 2 || drawH < 2) return;

  ctx.save();
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(mapImageData, drawX, drawY, drawW, drawH);
  ctx.restore();
}


// ═══════════════════════════════════════════════════════════
//  车体渲染
// ═══════════════════════════════════════════════════════════

function drawVehicle() {
  if (!vehicleParams) {
    // 无参数时画简单三角形
    const [sx, sy] = worldToScreen(vehicle.x, vehicle.y);
    ctx.save();
    ctx.translate(sx, sy);
    ctx.rotate(-vehicle.theta);
    ctx.beginPath();
    ctx.moveTo(15, 0);
    ctx.lineTo(-8, -8);
    ctx.lineTo(-8, 8);
    ctx.closePath();
    ctx.fillStyle = '#ffcc00';
    ctx.fill();
    ctx.restore();
    return;
  }

  const p = vehicleParams;
  const x = vehicle.x, y = vehicle.y;
  const tf = vehicle.theta;
  const tr = tf - vehicle.gamma;

  // 前车体中心
  const cosF = Math.cos(tf), sinF = Math.sin(tf);
  const cosR = Math.cos(tr), sinR = Math.sin(tr);

  // 铰接点
  const px = x - p.L_f * cosF;
  const py = y - p.L_f * sinF;

  // 后车体中心
  const rx = px - p.L_r * cosR;
  const ry = py - p.L_r * sinR;

  // 画前车体框
  drawBodyRect(x, y, tf, p.L_f_body, p.W_f_body, '#ffcc00', 2);
  // 画后车体框
  drawBodyRect(rx, ry, tr, p.L_r_body, p.W_r_body, '#ffaa00', 2);

  // 中心连线 (前车→铰接→后车)
  drawDashedLine(x, y, px, py, '#ffcc00', 1);
  drawDashedLine(px, py, rx, ry, '#ffaa00', 1);

  // 铰接点
  const [spx, spy] = worldToScreen(px, py);
  ctx.beginPath();
  ctx.arc(spx, spy, 4, 0, Math.PI * 2);
  ctx.fillStyle = 'white';
  ctx.fill();
  ctx.strokeStyle = '#ffcc00';
  ctx.lineWidth = 1.5;
  ctx.stroke();

  // 航向箭头
  const arrowLen = Math.max(1.5, 20 / camera.zoom);
  const ax = x + arrowLen * cosF;
  const ay = y + arrowLen * sinF;
  drawArrow(x, y, ax, ay, '#ffcc00', 2);
}

function drawBodyRect(cx, cy, angle, length, width, color, lineWidth) {
  const hl = length / 2, hw = width / 2;
  const cos = Math.cos(angle), sin = Math.sin(angle);

  const corners = [
    [cx + hl * cos - hw * sin, cy + hl * sin + hw * cos],
    [cx + hl * cos + hw * sin, cy + hl * sin - hw * cos],
    [cx - hl * cos + hw * sin, cy - hl * sin - hw * cos],
    [cx - hl * cos - hw * sin, cy - hl * sin + hw * cos],
  ];

  ctx.beginPath();
  corners.forEach((c, i) => {
    const [sx, sy] = worldToScreen(c[0], c[1]);
    if (i === 0) ctx.moveTo(sx, sy);
    else ctx.lineTo(sx, sy);
  });
  ctx.closePath();
  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.stroke();

  // 半透明填充
  ctx.fillStyle = color.replace(')', ', 0.08)').replace('rgb', 'rgba').replace('#', '');
  // 简单填充
  ctx.save();
  ctx.globalAlpha = 0.08;
  ctx.fillStyle = color;
  ctx.fill();
  ctx.restore();
}

function drawDashedLine(x1, y1, x2, y2, color, width) {
  const [sx1, sy1] = worldToScreen(x1, y1);
  const [sx2, sy2] = worldToScreen(x2, y2);
  ctx.save();
  ctx.setLineDash([4, 4]);
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.beginPath();
  ctx.moveTo(sx1, sy1);
  ctx.lineTo(sx2, sy2);
  ctx.stroke();
  ctx.restore();
}

function drawArrow(x1, y1, x2, y2, color, width) {
  const [sx1, sy1] = worldToScreen(x1, y1);
  const [sx2, sy2] = worldToScreen(x2, y2);
  const angle = Math.atan2(sy2 - sy1, sx2 - sx1);
  const headLen = 8;

  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.beginPath();
  ctx.moveTo(sx1, sy1);
  ctx.lineTo(sx2, sy2);
  ctx.stroke();

  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.moveTo(sx2, sy2);
  ctx.lineTo(sx2 - headLen * Math.cos(angle - 0.4), sy2 - headLen * Math.sin(angle - 0.4));
  ctx.lineTo(sx2 - headLen * Math.cos(angle + 0.4), sy2 - headLen * Math.sin(angle + 0.4));
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}


// ═══════════════════════════════════════════════════════════
//  轨迹渲染
// ═══════════════════════════════════════════════════════════

function drawTrail() {
  if (trail.length < 2) return;
  ctx.save();
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.4)';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  for (let i = 0; i < trail.length; i++) {
    const [sx, sy] = worldToScreen(trail[i][0], trail[i][1]);
    if (i === 0) ctx.moveTo(sx, sy);
    else ctx.lineTo(sx, sy);
  }
  ctx.stroke();
  ctx.restore();
}

function drawPolyline(pts, color, width, dashed) {
  if (pts.length < 2) return;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  if (dashed) ctx.setLineDash([6, 4]);
  ctx.beginPath();
  for (let i = 0; i < pts.length; i++) {
    const [sx, sy] = worldToScreen(pts[i][0], pts[i][1]);
    if (i === 0) ctx.moveTo(sx, sy);
    else ctx.lineTo(sx, sy);
  }
  ctx.stroke();
  ctx.restore();
}

function drawCilqrBand(pts) {
  if (pts.length < 2) return;
  // 车体宽度作为带宽
  const halfW = vehicleParams ? vehicleParams.W_f_body / 2 : 0.33;

  // 计算左右边界点
  const leftPts = [];
  const rightPts = [];

  for (let i = 0; i < pts.length; i++) {
    let dx, dy;
    if (i === 0) {
      dx = pts[1][0] - pts[0][0];
      dy = pts[1][1] - pts[0][1];
    } else if (i === pts.length - 1) {
      dx = pts[i][0] - pts[i - 1][0];
      dy = pts[i][1] - pts[i - 1][1];
    } else {
      dx = pts[i + 1][0] - pts[i - 1][0];
      dy = pts[i + 1][1] - pts[i - 1][1];
    }
    const len = Math.sqrt(dx * dx + dy * dy) || 1;
    // 法向量
    const nx = -dy / len * halfW;
    const ny = dx / len * halfW;

    leftPts.push([pts[i][0] + nx, pts[i][1] + ny]);
    rightPts.push([pts[i][0] - nx, pts[i][1] - ny]);
  }

  // 画填充带
  ctx.save();
  ctx.globalAlpha = 0.25;
  ctx.fillStyle = '#00cc66';
  ctx.beginPath();
  const [sx0, sy0] = worldToScreen(leftPts[0][0], leftPts[0][1]);
  ctx.moveTo(sx0, sy0);
  for (let i = 1; i < leftPts.length; i++) {
    const [sx, sy] = worldToScreen(leftPts[i][0], leftPts[i][1]);
    ctx.lineTo(sx, sy);
  }
  for (let i = rightPts.length - 1; i >= 0; i--) {
    const [sx, sy] = worldToScreen(rightPts[i][0], rightPts[i][1]);
    ctx.lineTo(sx, sy);
  }
  ctx.closePath();
  ctx.fill();
  ctx.restore();

  // 画中心线
  ctx.save();
  ctx.strokeStyle = '#00ff88';
  ctx.lineWidth = 1.5;
  ctx.globalAlpha = 0.7;
  ctx.beginPath();
  for (let i = 0; i < pts.length; i++) {
    const [sx, sy] = worldToScreen(pts[i][0], pts[i][1]);
    if (i === 0) ctx.moveTo(sx, sy);
    else ctx.lineTo(sx, sy);
  }
  ctx.stroke();
  ctx.restore();
}

function drawGoal() {
  if (!goalPos) return;
  const [sx, sy] = worldToScreen(goalPos.x, goalPos.y);

  // 外圈脉冲动画
  const pulse = 0.5 + 0.5 * Math.sin(Date.now() / 300);
  const r = 10 + pulse * 6;

  ctx.save();
  ctx.beginPath();
  ctx.arc(sx, sy, r, 0, Math.PI * 2);
  ctx.strokeStyle = `rgba(255, 68, 68, ${0.3 + pulse * 0.3})`;
  ctx.lineWidth = 2;
  ctx.stroke();

  ctx.beginPath();
  ctx.arc(sx, sy, 5, 0, Math.PI * 2);
  ctx.fillStyle = '#ff4444';
  ctx.fill();
  ctx.strokeStyle = 'white';
  ctx.lineWidth = 2;
  ctx.stroke();

  // 标签
  ctx.fillStyle = 'rgba(255, 68, 68, 0.9)';
  ctx.font = '11px Inter';
  ctx.textAlign = 'center';
  ctx.fillText(`(${goalPos.x.toFixed(1)}, ${goalPos.y.toFixed(1)})`, sx, sy - 18);
  ctx.restore();
}


// ═══════════════════════════════════════════════════════════
//  网格
// ═══════════════════════════════════════════════════════════

function drawGrid() {
  const [wl, wt] = screenToWorld(0, 0);
  const [wr, wb] = screenToWorld(canvas.width, canvas.height);

  // 自适应网格间距
  let gridSize = 1;
  const pixPerMeter = camera.zoom;
  if (pixPerMeter < 2) gridSize = 50;
  else if (pixPerMeter < 5) gridSize = 20;
  else if (pixPerMeter < 10) gridSize = 10;
  else if (pixPerMeter < 30) gridSize = 5;
  else if (pixPerMeter < 80) gridSize = 2;

  const xMin = Math.floor(Math.min(wl, wr) / gridSize) * gridSize;
  const xMax = Math.ceil(Math.max(wl, wr) / gridSize) * gridSize;
  const yMin = Math.floor(Math.min(wt, wb) / gridSize) * gridSize;
  const yMax = Math.ceil(Math.max(wt, wb) / gridSize) * gridSize;

  ctx.save();
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.06)';
  ctx.lineWidth = 0.5;

  for (let x = xMin; x <= xMax; x += gridSize) {
    const [sx] = worldToScreen(x, 0);
    ctx.beginPath();
    ctx.moveTo(sx, 0);
    ctx.lineTo(sx, canvas.height);
    ctx.stroke();
  }
  for (let y = yMin; y <= yMax; y += gridSize) {
    const [, sy] = worldToScreen(0, y);
    ctx.beginPath();
    ctx.moveTo(0, sy);
    ctx.lineTo(canvas.width, sy);
    ctx.stroke();
  }

  // 坐标轴标签
  ctx.fillStyle = 'rgba(255, 255, 255, 0.15)';
  ctx.font = '10px JetBrains Mono';
  ctx.textAlign = 'center';
  for (let x = xMin; x <= xMax; x += gridSize) {
    const [sx, sy] = worldToScreen(x, 0);
    if (sy > 10 && sy < canvas.height - 5) {
      ctx.fillText(x.toString(), sx, Math.min(sy + 14, canvas.height - 4));
    }
  }
  ctx.textAlign = 'right';
  for (let y = yMin; y <= yMax; y += gridSize) {
    const [sx, sy] = worldToScreen(0, y);
    if (sx > 5 && sx < canvas.width - 5) {
      ctx.fillText(y.toString(), Math.max(sx - 4, 30), sy + 4);
    }
  }

  ctx.restore();
}


// ═══════════════════════════════════════════════════════════
//  主渲染循环
// ═══════════════════════════════════════════════════════════

function render() {
  // 自适应 Canvas 尺寸
  const rect = canvas.getBoundingClientRect();
  if (canvas.width !== rect.width || canvas.height !== rect.height) {
    canvas.width = rect.width;
    canvas.height = rect.height;
  }

  // 跟随车辆
  if (followVehicle && (vehicle.x !== 0 || vehicle.y !== 0)) {
    camera.x += (vehicle.x - camera.x) * 0.1;
    camera.y += (vehicle.y - camera.y) * 0.1;
  }

  // 清空
  ctx.fillStyle = '#0a0a1a';
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  // 渲染层次
  drawGrid();
  drawMap();
  drawTrail();
  drawPolyline(globalPath, '#ff9900', 2, true);
  drawCilqrBand(cilqrTraj);
  drawGoal();
  drawVehicle();

  // 拖拽设方向时画箭头预览
  if (placingPoint) {
    const [sx, sy] = worldToScreen(placingPoint.wx, placingPoint.wy);
    const dx = placingPoint.endSx - placingPoint.sx;
    const dy = placingPoint.endSy - placingPoint.sy;
    const dist = Math.sqrt(dx * dx + dy * dy);

    // 放置点标记
    ctx.save();
    ctx.beginPath();
    ctx.arc(sx, sy, 6, 0, Math.PI * 2);
    ctx.fillStyle = placingPoint.mode === 'goal' ? '#ff4444' : '#44aaff';
    ctx.fill();
    ctx.strokeStyle = 'white';
    ctx.lineWidth = 2;
    ctx.stroke();

    // 方向箭头
    if (dist > 10) {
      const angle = Math.atan2(dy, dx);
      const arrowLen = Math.max(30, dist);
      const ex = sx + arrowLen * Math.cos(angle);
      const ey = sy + arrowLen * Math.sin(angle);
      const color = placingPoint.mode === 'goal' ? '#ff6666' : '#66ccff';

      ctx.strokeStyle = color;
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.moveTo(sx, sy);
      ctx.lineTo(ex, ey);
      ctx.stroke();

      // 箭头头部
      const headLen = 12;
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.moveTo(ex, ey);
      ctx.lineTo(ex - headLen * Math.cos(angle - 0.4), ey - headLen * Math.sin(angle - 0.4));
      ctx.lineTo(ex - headLen * Math.cos(angle + 0.4), ey - headLen * Math.sin(angle + 0.4));
      ctx.closePath();
      ctx.fill();

      // 角度标签
      const worldTheta = Math.atan2(-dy, dx);  // 世界坐标角度
      ctx.fillStyle = 'white';
      ctx.font = '12px Inter';
      ctx.textAlign = 'left';
      ctx.fillText(`${(worldTheta * 180 / Math.PI).toFixed(1)}°`, ex + 8, ey - 4);
    }
    ctx.restore();
  }

  // 帧率
  frameCount++;
  const now = Date.now();
  if (now - lastFpsTime > 1000) {
    currentFps = Math.round(frameCount * 1000 / (now - lastFpsTime));
    frameCount = 0;
    lastFpsTime = now;
  }
  if (now - lastWsCountTime > 1000) {
    wsHz = wsMessageCount;
    wsMessageCount = 0;
    lastWsCountTime = now;
  }

  requestAnimationFrame(render);
}


// ═══════════════════════════════════════════════════════════
//  状态面板更新
// ═══════════════════════════════════════════════════════════

function updatePanel() {
  document.getElementById('val-x').textContent = vehicle.x.toFixed(2);
  document.getElementById('val-y').textContent = vehicle.y.toFixed(2);
  document.getElementById('val-theta').textContent = (vehicle.theta * 180 / Math.PI).toFixed(1);
  document.getElementById('val-gamma').textContent = (vehicle.gamma * 180 / Math.PI).toFixed(1);
  document.getElementById('val-speed').textContent = vehicle.speed.toFixed(2);
  document.getElementById('val-fps').textContent = `${currentFps}/${wsHz}`;

  // GNSS 状态
  const gnssLabel = (status, prefix) => {
    const pos = status % 10;
    const names = {0:'无解',1:'单点',2:'差分',4:'RTK固定',5:'RTK浮'};
    const colors = {0:'#f44',1:'#f44',2:'#fa0',4:'#0f0',5:'#8f0'};
    const name = names[pos] || `?${pos}`;
    const color = colors[pos] || '#888';
    return `<span style="color:${color}">${prefix}:${name}(${status})</span>`;
  };
  document.getElementById('val-gnss-front').innerHTML = gnssLabel(vehicle.front_gnss_status || 0, '前');
  document.getElementById('val-gnss-rear').innerHTML = gnssLabel(vehicle.rear_gnss_status || 0, '后');

  if (goalPos) {
    document.getElementById('val-goal-x').textContent = goalPos.x.toFixed(1);
    document.getElementById('val-goal-y').textContent = goalPos.y.toFixed(1);
    const dist = Math.sqrt((vehicle.x - goalPos.x) ** 2 + (vehicle.y - goalPos.y) ** 2);
    document.getElementById('val-dist').textContent = dist.toFixed(1);
  }

  updateControlPanel();
  updateModuleButtons();
}

function updateModuleButtons() {
  document.querySelectorAll('.module-btn[data-module]').forEach(btn => {
    const name = btn.dataset.module;
    const alive = moduleStatus[name] || false;
    btn.classList.toggle('active', alive);
  });
}

function updateControlPanel() {
  // MPC 控制指令
  const el = (id) => document.getElementById(id);
  el('val-cmd-vf').textContent = controlCmd.v_front.toFixed(2);
  el('val-cmd-df').textContent = controlCmd.delta_front_deg.toFixed(1);
  el('val-cmd-vr').textContent = controlCmd.v_rear.toFixed(2);
  el('val-cmd-dr').textContent = controlCmd.delta_rear_deg.toFixed(1);

  // 前车 CAN 反馈
  const f = canFeedback.front;
  el('val-f-speed').textContent = f.speed_kmh.toFixed(1);
  el('val-f-eps').textContent = f.eps_angle_deg.toFixed(1);

  const gearEl = el('val-f-gear');
  gearEl.textContent = f.gear;
  gearEl.className = 'tag' + (f.gear === 'D' || f.gear === 'R' ? ' tag-ok' : '');

  const modeEl = el('val-f-mode');
  modeEl.textContent = f.driving_mode;
  modeEl.className = 'tag' + (f.driving_mode === 'AUTO' ? ' tag-ok' : f.driving_mode === 'REMOTE' ? ' tag-warn' : '');

  const enableEl = el('val-f-enable');
  const bothEnable = f.eps_enable && f.motor_enable;
  enableEl.textContent = bothEnable ? 'ON' : 'OFF';
  enableEl.className = 'tag' + (bothEnable ? ' tag-ok' : ' tag-warn');

  // 后车 CAN 反馈
  const r = canFeedback.rear;
  el('val-r-speed').textContent = r.speed_mps.toFixed(3);
  el('val-r-steer').textContent = r.steer_angle_rad.toFixed(3);
  el('val-r-batt').textContent = r.battery_v.toFixed(1);
  el('val-r-m1').textContent = `${r.motor1_rpm}`;

  const stateEl = el('val-r-state');
  stateEl.textContent = r.vehicle_state;
  stateEl.className = 'tag' + (r.vehicle_state === 'NORMAL' ? ' tag-ok' : r.vehicle_state === 'E-STOP' ? ' tag-warn' : r.vehicle_state === 'FAULT' ? ' tag-err' : '');

  const rModeEl = el('val-r-mode');
  rModeEl.textContent = r.control_mode;
  rModeEl.className = 'tag' + (r.control_mode === 'CAN' ? ' tag-ok' : r.control_mode === 'REMOTE' ? ' tag-warn' : '');

  const faultEl = el('val-r-fault');
  const hasFault = r.fault_high !== 0 || r.fault_low !== 0;
  faultEl.textContent = hasFault ? `H:${r.fault_high} L:${r.fault_low}` : 'OK';
  faultEl.className = 'tag' + (hasFault ? ' tag-err' : ' tag-ok');
}

setInterval(updatePanel, 200);


// ═══════════════════════════════════════════════════════════
//  数据连接（WebSocket 优先，HTTP 轮询后备）
// ═══════════════════════════════════════════════════════════

let wsFailCount = 0;
let usePolling = false;
let pollTimer = null;

function handleStateData(data) {
  if (data.type === 'init') {
    vehicleParams = data.vehicle_params;
    hasChassisDetail = data.has_chassis_detail || false;
    console.log('Vehicle params:', vehicleParams, 'chassis_detail:', hasChassisDetail);
    return;
  }
  wsMessageCount++;
  if (data.vehicle) vehicle = data.vehicle;
  if (data.trail) trail = data.trail;
  if (data.cilqr_traj) cilqrTraj = data.cilqr_traj;
  if (data.global_path && data.global_path.length > 0) {
    if (data.global_path.length !== lastGlobalPathLen) {
      showToast(`✅ 全局路径规划成功 (${data.global_path.length} 个点)`);
      lastGlobalPathLen = data.global_path.length;
    }
    globalPath = data.global_path;
  }
  if (data.control_cmd) controlCmd = data.control_cmd;
  if (data.can_feedback) canFeedback = data.can_feedback;
  if (data.module_status) moduleStatus = data.module_status;
}

function connectWS() {
  if (usePolling) return;  // 已切换到轮询模式

  const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
  ws = new WebSocket(`${protocol}//${location.host}/ws`);

  ws.onopen = () => {
    wsFailCount = 0;
    document.getElementById('ws-status').className = 'ws-badge connected';
    document.getElementById('ws-status').textContent = '● WS已连接';
    console.log('WebSocket connected');
  };

  ws.onmessage = (e) => {
    handleStateData(JSON.parse(e.data));
  };

  ws.onclose = () => {
    wsFailCount++;
    console.log(`WebSocket disconnected (fail #${wsFailCount})`);
    if (wsFailCount >= 3) {
      console.log('WebSocket failed 3 times, switching to HTTP polling');
      startPolling();
    } else {
      document.getElementById('ws-status').className = 'ws-badge disconnected';
      document.getElementById('ws-status').textContent = '● 重连中...';
      setTimeout(connectWS, 2000);
    }
  };

  ws.onerror = () => {
    ws.close();
  };
}

function startPolling() {
  usePolling = true;
  document.getElementById('ws-status').className = 'ws-badge connected';
  document.getElementById('ws-status').textContent = '● HTTP轮询';
  console.log('HTTP polling started (200ms interval)');

  if (pollTimer) clearInterval(pollTimer);
  pollTimer = setInterval(() => {
    fetch('/api/state')
      .then(r => r.json())
      .then(data => handleStateData(data))
      .catch(() => {});
  }, 200);  // 5Hz
}


// ═══════════════════════════════════════════════════════════
//  交互事件
// ═══════════════════════════════════════════════════════════

// 鼠标拖拽平移 & 起点/目标点拖拽设方向
let placingPoint = null;  // { mode: 'goal'|'start', wx, wy, sx, sy }

canvas.addEventListener('mousedown', (e) => {
  if ((goalMode || startMode) && e.button === 0) {
    const [wx, wy] = screenToWorld(e.offsetX, e.offsetY);
    placingPoint = {
      mode: goalMode ? 'goal' : 'start',
      wx, wy,
      sx: e.offsetX, sy: e.offsetY,
      endSx: e.offsetX, endSy: e.offsetY,
    };
    return;
  }

  isDragging = true;
  dragStart = { x: e.clientX, y: e.clientY };
  cameraStart = { x: camera.x, y: camera.y };
  followVehicle = false;
  document.getElementById('btn-follow').classList.remove('active');
});

canvas.addEventListener('mousemove', (e) => {
  if (placingPoint) {
    placingPoint.endSx = e.offsetX;
    placingPoint.endSy = e.offsetY;
    return;
  }
  if (!isDragging) return;
  const dx = (e.clientX - dragStart.x) / camera.zoom;
  const dy = (e.clientY - dragStart.y) / camera.zoom;
  camera.x = cameraStart.x - dx;
  camera.y = cameraStart.y + dy;
});

canvas.addEventListener('mouseup', (e) => {
  if (placingPoint) {
    const dx = placingPoint.endSx - placingPoint.sx;
    const dy = placingPoint.endSy - placingPoint.sy;
    // 拖拽距离 > 10px 则用拖拽角度，否则用默认 0
    let theta = 0;
    if (Math.sqrt(dx * dx + dy * dy) > 10) {
      theta = Math.atan2(-dy, dx);  // 屏幕Y翻转
    }
    const { wx, wy, mode } = placingPoint;

    if (mode === 'goal') {
      goalPos = { x: wx, y: wy, theta };
      fetch('/api/goal', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ x: wx, y: wy, theta }),
      }).then(r => r.json()).then(d => console.log('Goal set:', d));
      goalMode = false;
      canvas.classList.remove('goal-mode');
      document.getElementById('btn-goal').classList.remove('active');
    } else {
      fetch('/api/start', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ x: wx, y: wy, theta }),
      }).then(r => r.json()).then(d => console.log('Start set:', d));
      trail = [];
      startMode = false;
      canvas.classList.remove('goal-mode');
      document.getElementById('btn-start').classList.remove('active');
    }
    placingPoint = null;
    return;
  }
  isDragging = false;
});
canvas.addEventListener('mouseleave', () => { isDragging = false; placingPoint = null; });

// 滚轮缩放
canvas.addEventListener('wheel', (e) => {
  e.preventDefault();
  const [wx, wy] = screenToWorld(e.offsetX, e.offsetY);
  const factor = e.deltaY > 0 ? 0.85 : 1.18;
  camera.zoom = Math.max(0.5, Math.min(200, camera.zoom * factor));
  // 缩放后保持鼠标下的世界坐标不变
  const [newWx, newWy] = screenToWorld(e.offsetX, e.offsetY);
  camera.x += wx - newWx;
  camera.y += wy - newWy;
}, { passive: false });

// 键盘快捷键
document.addEventListener('keydown', (e) => {
  if (e.key === 'f' || e.key === 'F') toggleFollow();
  if (e.key === 'g' || e.key === 'G') toggleGoalMode();
  if (e.key === 'r' || e.key === 'R') resetView();
  if (e.key === 'Escape') {
    goalMode = false;
    canvas.classList.remove('goal-mode');
    document.getElementById('btn-goal').classList.remove('active');
  }
});


// ═══════════════════════════════════════════════════════════
//  工具栏方法
// ═══════════════════════════════════════════════════════════

function toggleModule(name) {
  const btn = document.querySelector(`[data-module="${name}"]`);
  const isActive = btn.classList.contains('active');
  const action = isActive ? 'stop' : 'start';

  fetch('/api/module', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name, action }),
  }).then(r => r.json()).then(d => {
    if (d.status === 'ok') {
      showToast(`${name}: ${action === 'start' ? '启动中...' : '已停止'}`);
    }
  });
}

function toggleGoalMode() {
  goalMode = !goalMode;
  startMode = false;
  canvas.classList.toggle('goal-mode', goalMode);
  document.getElementById('btn-goal').classList.toggle('active', goalMode);
  document.getElementById('btn-start').classList.remove('active');
}

function toggleStartMode() {
  startMode = !startMode;
  goalMode = false;
  canvas.classList.toggle('goal-mode', startMode);
  document.getElementById('btn-start').classList.toggle('active', startMode);
  document.getElementById('btn-goal').classList.remove('active');
}

function toggleFollow() {
  followVehicle = !followVehicle;
  document.getElementById('btn-follow').classList.toggle('active', followVehicle);
}

function resetView() {
  camera.zoom = 5.0;
  followVehicle = true;
  document.getElementById('btn-follow').classList.add('active');
}

// 急停
function emergencyStop() {
  if (!confirm('确认发送急停指令？')) return;
  fetch('/api/emergency', { method: 'POST' })
    .then(r => r.json())
    .then(d => {
      showToast('🛑 急停指令已发送!', 3000);
      console.log('Emergency stop:', d);
    })
    .catch(e => showToast('❌ 急停发送失败', 3000));
}

// 暴露到全局
window.toggleModule = toggleModule;
window.toggleGoalMode = toggleGoalMode;
window.toggleStartMode = toggleStartMode;
window.toggleFollow = toggleFollow;
window.resetView = resetView;
window.emergencyStop = emergencyStop;


// ═══════════════════════════════════════════════════════════
//  录包 & 回放
// ═══════════════════════════════════════════════════════════

let isRecording = false;
let recordStartTime = 0;

function toggleRecord() {
  if (!isRecording) {
    fetch('/api/record/start', {method: 'POST'})
      .then(r => r.json())
      .then(d => {
        isRecording = true;
        recordStartTime = Date.now();
        document.getElementById('btn-record').classList.add('recording');
        document.getElementById('btn-record').innerHTML = '⏹ 停止录包';
        showToast('⏺ 开始录包...');
      });
  } else {
    fetch('/api/record/stop', {method: 'POST'})
      .then(r => r.json())
      .then(d => {
        isRecording = false;
        document.getElementById('btn-record').classList.remove('recording');
        document.getElementById('btn-record').innerHTML = '⏺ 录包';
        if (d.frames > 0) {
          showToast(`⏹ 录包完成: ${d.frames}帧, ${d.duration}秒`, 3000);
        }
      });
  }
}

// 回放状态
let pbData = null;    // {frames: [...]}
let pbIndex = 0;
let pbPlaying = false;
let pbSpeed = 1.0;
let pbTimer = null;
let pbLivePaused = false;

function showRecordings() {
  fetch('/api/records')
    .then(r => r.json())
    .then(files => {
      if (files.length === 0) {
        showToast('没有录包文件', 2000);
        return;
      }
      const items = files.map(f =>
        `<div style="display:flex;justify-content:space-between;align-items:center;padding:6px 0;border-bottom:1px solid #334">
          <span style="color:#8cf;cursor:pointer" onclick="loadRecording('${f.name}')">${f.name}</span>
          <span style="color:#888">${f.size_kb}KB</span>
        </div>`
      ).join('');
      // 简单弹窗
      let dialog = document.getElementById('rec-dialog');
      if (!dialog) {
        dialog = document.createElement('div');
        dialog.id = 'rec-dialog';
        dialog.style.cssText = 'position:fixed;top:50%;left:50%;transform:translate(-50%,-50%);background:#1a2035;border:1px solid #445;border-radius:8px;padding:20px;z-index:200;min-width:320px;max-height:400px;overflow:auto;box-shadow:0 8px 32px rgba(0,0,0,0.5)';
        document.body.appendChild(dialog);
      }
      dialog.innerHTML = `
        <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:12px">
          <h3 style="margin:0;color:#fff">📂 录包列表</h3>
          <button onclick="document.getElementById('rec-dialog').style.display='none'" style="background:none;border:none;color:#888;font-size:1.2rem;cursor:pointer">✕</button>
        </div>
        ${items}
      `;
      dialog.style.display = 'block';
    });
}

function loadRecording(filename) {
  const dialog = document.getElementById('rec-dialog');
  if (dialog) dialog.style.display = 'none';
  showToast('加载录包中...', 2000);

  fetch(`/api/record/${filename}`)
    .then(r => r.json())
    .then(data => {
      pbData = data;
      pbIndex = 0;
      pbPlaying = false;
      pbSpeed = 1.0;

      // 暂停实时数据
      pbLivePaused = true;
      if (pollTimer) { clearInterval(pollTimer); pollTimer = null; }

      // 显示回放条
      const bar = document.getElementById('playback-bar');
      bar.style.display = 'flex';
      document.getElementById('pb-slider').max = data.frames.length - 1;
      document.getElementById('pb-play').textContent = '▶';
      updatePlaybackTime();

      // 加载第一帧
      applyPlaybackFrame(0);
      showToast(`已加载: ${data.frame_count}帧, ${data.duration}秒`, 2000);
    })
    .catch(() => showToast('加载失败', 2000));
}

function applyPlaybackFrame(idx) {
  if (!pbData || idx < 0 || idx >= pbData.frames.length) return;
  pbIndex = idx;
  const frame = pbData.frames[idx];
  handleStateData(frame);
  document.getElementById('pb-slider').value = idx;
  updatePlaybackTime();
}

function updatePlaybackTime() {
  if (!pbData) return;
  const current = pbData.frames[pbIndex]?.t || 0;
  const total = pbData.duration || 0;
  const fmt = (s) => `${Math.floor(s/60)}:${String(Math.floor(s%60)).padStart(2,'0')}`;
  document.getElementById('pb-time').textContent = `${fmt(current)} / ${fmt(total)}`;
}

function playbackToggle() {
  if (!pbData) return;
  pbPlaying = !pbPlaying;
  document.getElementById('pb-play').textContent = pbPlaying ? '⏸' : '▶';

  if (pbPlaying) {
    pbTimer = setInterval(() => {
      if (pbIndex < pbData.frames.length - 1) {
        applyPlaybackFrame(pbIndex + 1);
      } else {
        pbPlaying = false;
        document.getElementById('pb-play').textContent = '▶';
        clearInterval(pbTimer);
      }
    }, 200 / pbSpeed);  // 5fps * speed
  } else {
    if (pbTimer) { clearInterval(pbTimer); pbTimer = null; }
  }
}

function playbackSeek(val) {
  applyPlaybackFrame(parseInt(val));
}

function playbackSpeed(val) {
  pbSpeed = parseFloat(val);
  if (pbPlaying) {
    if (pbTimer) clearInterval(pbTimer);
    pbTimer = setInterval(() => {
      if (pbIndex < pbData.frames.length - 1) {
        applyPlaybackFrame(pbIndex + 1);
      } else {
        pbPlaying = false;
        document.getElementById('pb-play').textContent = '▶';
        clearInterval(pbTimer);
      }
    }, 200 / pbSpeed);
  }
}

function playbackStop() {
  pbPlaying = false;
  pbData = null;
  if (pbTimer) { clearInterval(pbTimer); pbTimer = null; }
  document.getElementById('playback-bar').style.display = 'none';

  // 恢复实时数据
  pbLivePaused = false;
  if (usePolling) {
    startPolling();
  } else {
    wsFailCount = 0;
    connectWS();
  }
  showToast('已退出回放，恢复实时数据');
}

window.toggleRecord = toggleRecord;
window.showRecordings = showRecordings;
window.loadRecording = loadRecording;
window.playbackToggle = playbackToggle;
window.playbackSeek = playbackSeek;
window.playbackSpeed = playbackSpeed;
window.playbackStop = playbackStop;


// ═══════════════════════════════════════════════════════════
//  初始化
// ═══════════════════════════════════════════════════════════

function init() {
  // 调整 canvas
  const rect = canvas.getBoundingClientRect();
  canvas.width = rect.width;
  canvas.height = rect.height;

  // 加载地图
  loadMap();

  // 连接 WebSocket
  connectWS();

  // 启动渲染
  requestAnimationFrame(render);

  console.log('Articulated Vehicle Visualizer initialized');
}

init();
