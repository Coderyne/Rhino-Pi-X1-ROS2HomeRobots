/**
 * 轮足机器人 — 仪表盘核心逻辑
 * roslibjs 实现实时数据可视化、地图渲染、建图导航控制
 *
 * 功能模块:
 *   - rosbridge WebSocket 连接管理
 *   - 地图渲染 (OccupancyGrid / LaserScan / Path / TF)
 *   - 导航工具 (设置目标 / 初始位姿 / 人体跟随 / 区域管理)
 *   - 控制面板 (使能 / 急停 / 跳跃 / 自起 / 姿态滑块)
 *   - 虚拟摇杆 (Canvas 触控 + 鼠标拖拽)
 *   - WASD 键盘控制 (含加速度模拟)
 *   - 指令帧实时监控面板
 *   - 摄像头 MJPEG 流预览
 */
(function () {
  'use strict';

  const $ = (id) => document.getElementById(id);

  /* ═══════════════════════════════════════════════════
   *  主题切换 (暗色 / 亮色)
   ═══════════════════════════════════════════════════ */
  var THEME_KEY = 'dashboard-theme';
  var html = document.documentElement;

  function applyTheme(theme) {
    html.dataset.theme = theme;
    var icon = document.querySelector('#btnTheme .md-icon');
    if (icon) icon.textContent = theme === 'dark' ? 'dark_mode' : 'light_mode';
    try { localStorage.setItem(THEME_KEY, theme); } catch(e) {}
  }

  var saved = null;
  try { saved = localStorage.getItem(THEME_KEY); } catch(e) {}
  applyTheme(saved || 'dark');

  var btnTheme = $('btnTheme');
  btnTheme.addEventListener('click', function () {
    var next = html.dataset.theme === 'dark' ? 'light' : 'dark';
    applyTheme(next);
  });

  /* ═══════════════════════════════════════════════════
   *  地图工具栏提示 (tooltip 实现)
   ═══════════════════════════════════════════════════ */
  function updateBtnTitle(btn, text) {
    btn.title = text;
    if (btn._tooltip) btn._tooltip.textContent = text;
  }

  function initMapTooltips() {
    var bar = document.querySelector('.map-toolbar');
    if (!bar) return;
    var btns = bar.querySelectorAll('.md-btn[title]');
    for (var i = 0; i < btns.length; i++) {
      (function (btn) {
        var tip = document.createElement('div');
        tip.className = 'map-tooltip';
        tip.textContent = btn.getAttribute('title');
        btn._tooltip = tip;
        btn.appendChild(tip);

        btn.addEventListener('mouseenter', function () {
          if (btn._tooltip) btn._tooltip.classList.add('show');
        });
        btn.addEventListener('mouseleave', function () {
          if (btn._tooltip) btn._tooltip.classList.remove('show');
        });
      })(btns[i]);
    }
  }
  initMapTooltips();

  function togglePanel(bodyId, iconId) {
    const body = $(bodyId);
    const icon = $(iconId);
    if (!body || !icon) return;
    if (body.classList.contains('expanded-false')) {
      body.classList.remove('expanded-false');
      icon.classList.remove('collapsed');
    } else {
      body.classList.add('expanded-false');
      icon.classList.add('collapsed');
    }
  }
  window.togglePanel = togglePanel;
  const connStatus = $('connStatus');

  if (typeof ROSLIB === 'undefined') {
    console.error('[dashboard] ROSLIB 未加载, 无法建立 WebSocket 连接');
    const icon = connStatus.querySelector('.md-icon');
    if (icon) icon.textContent = 'cloud_off';
    connStatus.querySelector('span:last-child').textContent = '依赖加载失败';
    return;
  }

  const WS_URL = 'ws://' + window.location.hostname + ':9091';
  console.log('[dashboard] 连接 rosbridge:', WS_URL);
  const ros = new ROSLIB.Ros({ url: WS_URL });

  ros.on('connection', () => {
    console.log('[dashboard] rosbridge 已连接');
    const icon = connStatus.querySelector('.md-icon');
    if (icon) icon.textContent = 'cloud_done';
    connStatus.className = 'connection-status connected';
    connStatus.querySelector('span:last-child').textContent = '已连接';
    try { initPublishers(); } catch (e) { console.error('[dashboard] initPublishers 失败:', e); }
    try { initSubscribers(); } catch (e) { console.error('[dashboard] initSubscribers 失败:', e); }
  });

  ros.on('error', (err) => {
    console.error('[dashboard] rosbridge 连接错误:', err);
    const icon = connStatus.querySelector('.md-icon');
    if (icon) icon.textContent = 'cloud_off';
    connStatus.className = 'connection-status';
    connStatus.querySelector('span:last-child').textContent = '连接失败';
  });

  ros.on('close', () => {
    console.warn('[dashboard] rosbridge 连接断开');
    const icon = connStatus.querySelector('.md-icon');
    if (icon) icon.textContent = 'cloud_sync';
    connStatus.className = 'connection-status';
    connStatus.querySelector('span:last-child').textContent = '已断开, 重连中...';
  });

  /* ═══════════════════════════════════════════════════
   *  ROS Topic 发布者初始化 (连接成功后调用)
   *  所有控制指令通过此处发布
   ═══════════════════════════════════════════════════ */
  let pubCmdVel, pubCmdAtt, pubEnable, pubEstop, pubJump, pubRecover;
  let pubGoalPose, pubInitPose, pubKeepAlive, pubFollowTarget, pubFollowActive, pubFollowRadius;
  let pubRegionSave, pubRegionDelete, pubRegionNavigate;

  function initPublishers() {
    pubCmdVel  = new ROSLIB.Topic({ ros: ros, name: '/cmd_vel',      messageType: 'geometry_msgs/Twist' });
    pubCmdAtt  = new ROSLIB.Topic({ ros: ros, name: '/cmd_attitude', messageType: 'std_msgs/Float32MultiArray' });
    pubEnable  = new ROSLIB.Topic({ ros: ros, name: '/cmd_enable',   messageType: 'std_msgs/Bool' });
    pubEstop   = new ROSLIB.Topic({ ros: ros, name: '/cmd_estop',    messageType: 'std_msgs/Bool' });
    pubJump    = new ROSLIB.Topic({ ros: ros, name: '/cmd_jump',     messageType: 'std_msgs/Bool' });
    pubRecover = new ROSLIB.Topic({ ros: ros, name: '/cmd_recover',  messageType: 'std_msgs/Bool' });
    pubGoalPose = new ROSLIB.Topic({ ros: ros, name: '/goal_pose',   messageType: 'geometry_msgs/PoseStamped' });
    pubInitPose = new ROSLIB.Topic({ ros: ros, name: '/initialpose', messageType: 'geometry_msgs/PoseWithCovarianceStamped' });
    pubKeepAlive = new ROSLIB.Topic({ ros: ros, name: '/cmd_keep_alive', messageType: 'std_msgs/Bool' });
    pubFollowTarget = new ROSLIB.Topic({ ros: ros, name: '/follow_target', messageType: 'geometry_msgs/PointStamped' });
    pubFollowActive = new ROSLIB.Topic({ ros: ros, name: '/follow_active',  messageType: 'std_msgs/Bool' });
    pubFollowRadius = new ROSLIB.Topic({ ros: ros, name: '/follow_radius', messageType: 'std_msgs/Float32' });
    pubRegionSave = new ROSLIB.Topic({ ros: ros, name: '/region_manager/save', messageType: 'std_msgs/String' });
    pubRegionDelete = new ROSLIB.Topic({ ros: ros, name: '/region_manager/delete', messageType: 'std_msgs/String' });
    pubRegionNavigate = new ROSLIB.Topic({ ros: ros, name: '/region_manager/navigate', messageType: 'std_msgs/String' });
    // 同步 keep-alive 初始状态到 bridge
    pubKeepAlive.publish({ data: $('chkKeepAlive').checked });
  }

  /* ═══════════════════════════════════════════════════
   *  DOM 元素引用 (状态显示面板)
   ═══════════════════════════════════════════════════ */
  const valYaw    = $('valYaw');
  const valRoll   = $('valRoll');
  const valPitch  = $('valPitch');
  const valVel    = $('valVel');
  const valGyroZ  = $('valGyroZ');
  const valBattery = $('valBattery');
  const badgeStart = $('badgeStart');
  const badgeEnable = $('badgeEnable');
  const mapMode   = $('mapMode');

  let isEnabled = false;


  /* ═══════════════════════════════════════════════════
   *  Map Renderer
   *   ─ OccupancyGrid → Canvas 灰度图
   *   ─ LaserScan overlay (绿色扫描点)
   *   ─ Path overlay (蓝色路径线)
   *   ─ Robot marker (红色三角形)
   *   ─ Nav tools (Goal / InitPose via click)
   ═══════════════════════════════════════════════════ */

  const mapCanvas = $('mapCanvas');
  const mapCtx = mapCanvas.getContext('2d');

  // Map state
  let mapInfo = null;
  let mapData = null;
  let mapImageData = null;
  let mapDirty = true;
  let overlayDirty = false;
  let scanPoints = [];
  let pathPoints = [];
  let robotX = 0, robotY = 0, robotYaw = 0;
  let robotVisible = false;

  // Tool state
  let activeTool = null;
  let initPoseX = 0, initPoseY = 0;
  let initPosePrevX = 0, initPosePrevY = 0;  // mouse preview for 2nd step
  let goalPoseX = 0, goalPoseY = 0;
  let goalPosePrevX = 0, goalPosePrevY = 0;
  let followTargetX = 0, followTargetY = 0;   // current follow target
  let followRectStep = 0;                    // 0=wait corner1, 1=wait corner2
  let followRectX1 = 0, followRectY1 = 0;    // corner 1 world coords
  let followRectX2 = 0, followRectY2 = 0;    // corner 2 world coords (preview)
  let followRadius = 0.3;                     // current target radius
  let followRectW = 0, followRectH = 0;      // finalized rectangle world size

  // Region tool state
  let regionToolStep = 0;                  // 0=wait anchor, 1=drag preview
  let regionAnchorX = 0, regionAnchorY = 0; // fixed corner world coords
  let regionMouseX = 0, regionMouseY = 0;   // moving corner world coords (preview)
  let regions = [];                         // loaded regions from ROS2
  let regionHoverIdx = -1;                  // -1 = none, >=0 = hovered region index

  // Canvas sizing & view state
  let cssW = 0, cssH = 0;
  let zoomLevel = 1.0;
  let panX = 0, panY = 0;
  const ZOOM_MIN = 0.3, ZOOM_MAX = 5.0, ZOOM_STEP = 0.1;

  function resizeCanvas() {
    var wrap = mapCanvas.parentElement;
    if (!wrap) return;
    var rect = wrap.getBoundingClientRect();
    var w = rect.width;
    var h = rect.height;
    if (w <= 0 || h <= 0) return;
    if (cssW !== w || cssH !== h) {
      cssW = w; cssH = h;
      mapCanvas.width = w;
      mapCanvas.height = h;
      mapCanvas.style.width = w + 'px';
      mapCanvas.style.height = h + 'px';
      mapDirty = true;
    }
  }

  // ── 地图坐标系转换 ────────────────────────────
  // OccupancyGrid: data[row * width + col], row 0 = lowest Y (origin.y), Y increases upward
  // Canvas ImageData: Y increases downward
  // We Y-flip during ImageData creation so canvas row y = OccupancyGrid row (height-1-y)
  // World → Canvas: worldToCanvas(wx, wy) → {x, y} in canvas pixels
  // Canvas → World: canvasToWorld(px, py) → {wx, wy} in world meters

  function buildMapImageData() {
    if (!mapData || !mapInfo) return;
    const w = mapInfo.width, h = mapInfo.height;
    mapImageData = mapCtx.createImageData(w, h);

    for (let row = 0; row < h; row++) {
      for (let col = 0; col < w; col++) {
        // Y-flip: canvas row 'row' = OccupancyGrid row 'h - 1 - row'
        const src = (h - 1 - row) * w + col;
        const dst = (row * w + col) * 4;
        const v = mapData[src];
        if (v < 0) {
          mapImageData.data[dst] = 68;     mapImageData.data[dst + 1] = 71;
          mapImageData.data[dst + 2] = 73; mapImageData.data[dst + 3] = 255;
        } else if (v === 0) {
          mapImageData.data[dst] = 226;    mapImageData.data[dst + 1] = 226;
          mapImageData.data[dst + 2] = 233; mapImageData.data[dst + 3] = 255;
        } else {
          const c = Math.floor(26 + (100 - v) * 2.29);
          mapImageData.data[dst] = c;      mapImageData.data[dst + 1] = c;
          mapImageData.data[dst + 2] = c;  mapImageData.data[dst + 3] = 255;
        }
      }
    }
    mapDirty = false;
  }

  // Offscreen canvas holding the rendered map image (Y-up)
  let _mapOffCanvas = null;
  function getMapOffCanvas() {
    if (!mapData || !mapInfo) return null;
    if (!_mapOffCanvas || _mapOffCanvas.width !== mapInfo.width || _mapOffCanvas.height !== mapInfo.height) {
      _mapOffCanvas = document.createElement('canvas');
      _mapOffCanvas.width = mapInfo.width;
      _mapOffCanvas.height = mapInfo.height;
    }
    return _mapOffCanvas;
  }

  // Current world-to-canvas transform state
  let _mt = { imgX: 0, imgY: 0, imgW: 0, imgH: 0, imgScale: 1 };

  function updateMapTransform() {
    if (!mapInfo || !cssW || !cssH) return;
    const baseS = Math.min(cssW / mapInfo.width, cssH / mapInfo.height);
    const s = baseS * zoomLevel;
    _mt.imgScale = s;
    _mt.imgW = mapInfo.width * s;
    _mt.imgH = mapInfo.height * s;
    _mt.imgX = (cssW - _mt.imgW) / 2 + panX;
    _mt.imgY = (cssH - _mt.imgH) / 2 + panY;
  }

  function worldToCanvas(wx, wy) {
    if (!mapInfo) return { x: 0, y: 0 };
    const rpm = mapInfo.resolution;
    const col = (wx - mapInfo.origin.x) / rpm;
    const row = (wy - mapInfo.origin.y) / rpm;
    const canvasRow = mapInfo.height - 1 - row;  // Y-flip
    return {
      x: _mt.imgX + col * _mt.imgScale,
      y: _mt.imgY + canvasRow * _mt.imgScale
    };
  }

  function canvasToWorld(px, py) {
    if (!mapInfo) return { wx: 0, wy: 0 };
    const rpm = mapInfo.resolution;
    const col = (px - _mt.imgX) / _mt.imgScale;
    const canvasRow = (py - _mt.imgY) / _mt.imgScale;
    const row = mapInfo.height - 1 - canvasRow;  // Reverse Y-flip
    return {
      wx: col * rpm + mapInfo.origin.x,
      wy: row * rpm + mapInfo.origin.y
    };
  }

  // ── 渲染地图背景 (Y-flip: OccupancyGrid Y↑ = Canvas Y↓) ─
  function renderMap() {
    resizeCanvas();
    if (!mapData || !mapInfo) return;

    if (mapDirty || !mapImageData) buildMapImageData();
    updateMapTransform();

    const off = getMapOffCanvas();
    if (!off) return;
    off.getContext('2d').putImageData(mapImageData, 0, 0);

    // Draw in canvas pixel space (image is already Y-flipped)
    mapCtx.drawImage(off, _mt.imgX, _mt.imgY, _mt.imgW, _mt.imgH);
  }

  function drawOverlays() {
    if (!mapInfo) return;
    const z = Math.max(0.5, zoomLevel);

    // Path
    if (pathPoints.length > 1) {
      mapCtx.beginPath();
      mapCtx.strokeStyle = 'rgba(88, 166, 255, 0.8)';
      mapCtx.lineWidth = Math.max(1, 2 * z);
      const p0 = worldToCanvas(pathPoints[0].x, pathPoints[0].y);
      mapCtx.moveTo(p0.x, p0.y);
      for (let i = 1; i < pathPoints.length; i++) {
        const p = worldToCanvas(pathPoints[i].x, pathPoints[i].y);
        mapCtx.lineTo(p.x, p.y);
      }
      mapCtx.stroke();
    }

    // Laser scan
    if (scanPoints.length > 0) {
      const r = Math.max(1, 1.5 * z);
      const size = 2 * r;
      mapCtx.fillStyle = 'rgba(63, 185, 80, 0.4)';
      for (const pt of scanPoints) {
        const p = worldToCanvas(pt.x, pt.y);
        mapCtx.fillRect(p.x - r, p.y - r, size, size);
      }
    }

    // Robot marker
    if (robotVisible) {
      const r = worldToCanvas(robotX, robotY);
      mapCtx.save();
      mapCtx.translate(r.x, r.y);
      mapCtx.rotate(-robotYaw);
      mapCtx.fillStyle = '#f85149';
      mapCtx.beginPath();
      const s = z;
      mapCtx.moveTo(10 * s, 0);
      mapCtx.lineTo(-6 * s, -6 * s);
      mapCtx.lineTo(-4 * s, 0);
      mapCtx.lineTo(-6 * s, 6 * s);
      mapCtx.closePath();
      mapCtx.fill();
      mapCtx.strokeStyle = 'rgba(255,255,255,0.6)';
      mapCtx.lineWidth = Math.max(1, 1 * s);
      mapCtx.stroke();
      mapCtx.restore();
    }

    // Init-pose visual guide
    if (activeTool === 'initpose') {
      const mk = worldToCanvas(initPoseX, initPoseY);

      if (initPoseX || initPoseY) {
        const r = 8 * z;
        mapCtx.beginPath();
        mapCtx.arc(mk.x, mk.y, r, 0, Math.PI * 2);
        mapCtx.fillStyle = 'rgba(176, 198, 255, 0.35)';
        mapCtx.fill();
        mapCtx.strokeStyle = 'rgba(176, 198, 255, 0.8)';
        mapCtx.lineWidth = Math.max(1, 2 * z);
        mapCtx.setLineDash([]);
        mapCtx.stroke();
      }

      if ((initPoseX || initPoseY) && (initPosePrevX || initPosePrevY)) {
        const pp = worldToCanvas(initPosePrevX, initPosePrevY);
        const dx = pp.x - mk.x, dy = pp.y - mk.y;
        const dist = Math.hypot(dx, dy);
        if (dist > 4) {
          const ux = dx / dist, uy = dy / dist;

          mapCtx.beginPath();
          mapCtx.moveTo(mk.x, mk.y);
          mapCtx.lineTo(pp.x, pp.y);
          mapCtx.strokeStyle = 'rgba(176, 198, 255, 0.7)';
          mapCtx.lineWidth = Math.max(1, 2 * z);
          mapCtx.setLineDash([5 * z, 4 * z]);
          mapCtx.stroke();
          mapCtx.setLineDash([]);

          const h = 8 * z;
          mapCtx.save();
          mapCtx.translate(pp.x, pp.y);
          mapCtx.rotate(Math.atan2(dy, dx));
          mapCtx.fillStyle = 'rgba(176, 198, 255, 0.7)';
          mapCtx.beginPath();
          mapCtx.moveTo(0, 0);
          mapCtx.lineTo(-h, -h * 0.5);
          mapCtx.lineTo(-h, h * 0.5);
          mapCtx.closePath();
          mapCtx.fill();
          mapCtx.restore();
        }
      }
    }

    // Goal-pose visual guide (same style, green tone)
    if (activeTool === 'goal') {
      const mk = worldToCanvas(goalPoseX, goalPoseY);

      if (goalPoseX || goalPoseY) {
        const r = 8 * z;
        mapCtx.beginPath();
        mapCtx.arc(mk.x, mk.y, r, 0, Math.PI * 2);
        mapCtx.fillStyle = 'rgba(63, 185, 80, 0.3)';
        mapCtx.fill();
        mapCtx.strokeStyle = 'rgba(63, 185, 80, 0.75)';
        mapCtx.lineWidth = Math.max(1, 2 * z);
        mapCtx.setLineDash([]);
        mapCtx.stroke();
      }

      if ((goalPoseX || goalPoseY) && (goalPosePrevX || goalPosePrevY)) {
        const pp = worldToCanvas(goalPosePrevX, goalPosePrevY);
        const dx = pp.x - mk.x, dy = pp.y - mk.y;
        const dist = Math.hypot(dx, dy);
        if (dist > 4) {
          mapCtx.beginPath();
          mapCtx.moveTo(mk.x, mk.y);
          mapCtx.lineTo(pp.x, pp.y);
          mapCtx.strokeStyle = 'rgba(63, 185, 80, 0.65)';
          mapCtx.lineWidth = Math.max(1, 2 * z);
          mapCtx.setLineDash([5 * z, 4 * z]);
          mapCtx.stroke();
          mapCtx.setLineDash([]);

          const h = 8 * z;
          mapCtx.save();
          mapCtx.translate(pp.x, pp.y);
          mapCtx.rotate(Math.atan2(dy, dx));
          mapCtx.fillStyle = 'rgba(63, 185, 80, 0.65)';
          mapCtx.beginPath();
          mapCtx.moveTo(0, 0);
          mapCtx.lineTo(-h, -h * 0.5);
          mapCtx.lineTo(-h, h * 0.5);
          mapCtx.closePath();
          mapCtx.fill();
          mapCtx.restore();
        }
      }
    }

    // Follow target marker + rectangle preview
    if (activeTool === 'follow' || followTargetX || followTargetY) {
      // Rectangle preview during step 1
      if (activeTool === 'follow' && followRectStep === 1) {
        const c1 = worldToCanvas(followRectX1, followRectY1);
        const c2 = worldToCanvas(followRectX2, followRectY2);
        const rx = Math.min(c1.x, c2.x), ry = Math.min(c1.y, c2.y);
        const rw = Math.abs(c2.x - c1.x), rh = Math.abs(c2.y - c1.y);
        if (rw > 2 && rh > 2) {
          mapCtx.fillStyle = 'rgba(239, 159, 59, 0.12)';
          mapCtx.fillRect(rx, ry, rw, rh);
          mapCtx.strokeStyle = 'rgba(250, 176, 56, 0.85)';
          mapCtx.lineWidth = Math.max(1.5, 2.5 * z);
          mapCtx.setLineDash([]);
          mapCtx.strokeRect(rx, ry, rw, rh);
          const corners = [[c1.x, c1.y], [c2.x, c1.y], [c2.x, c2.y], [c1.x, c2.y]];
          mapCtx.fillStyle = 'rgba(250, 176, 56, 0.9)';
          for (const [cx, cy] of corners) {
            mapCtx.beginPath();
            mapCtx.arc(cx, cy, 4 * z, 0, Math.PI * 2);
            mapCtx.fill();
          }
          const wM = (Math.abs(followRectX2 - followRectX1)).toFixed(1);
          const hM = (Math.abs(followRectY2 - followRectY1)).toFixed(1);
          const lbl = wM + '\u00d7' + hM + 'm';
          mapCtx.font = (11 * z) + 'px monospace';
          mapCtx.fillStyle = 'rgba(250, 176, 56, 0.9)';
          mapCtx.fillText(lbl, rx + 4, ry - 6);
        }
      }
      if (followTargetX || followTargetY) {
        if (followRectW > 0 && followRectH > 0) {
          const tl = worldToCanvas(followTargetX - followRectW / 2, followTargetY + followRectH / 2);
          const br = worldToCanvas(followTargetX + followRectW / 2, followTargetY - followRectH / 2);
          const rx = Math.min(tl.x, br.x), ry = Math.min(tl.y, br.y);
          const rw = Math.abs(br.x - tl.x), rh = Math.abs(br.y - tl.y);
          if (rw > 2 && rh > 2) {
            mapCtx.fillStyle = 'rgba(239, 159, 59, 0.15)';
            mapCtx.fillRect(rx, ry, rw, rh);
            mapCtx.strokeStyle = 'rgba(250, 176, 56, 0.75)';
            mapCtx.lineWidth = Math.max(1, 2 * z);
            mapCtx.setLineDash([]);
            mapCtx.strokeRect(rx, ry, rw, rh);
          }
        } else {
          const f = worldToCanvas(followTargetX, followTargetY);
          if (f) {
            const r = Math.max(4, followRadius / mapInfo.resolution * _mt.imgScale);
            mapCtx.beginPath();
            mapCtx.arc(f.x, f.y, r, 0, Math.PI * 2);
            mapCtx.fillStyle = 'rgba(255, 167, 38, 0.2)';
            mapCtx.fill();
            mapCtx.strokeStyle = 'rgba(255, 167, 38, 0.7)';
            mapCtx.lineWidth = Math.max(1, 2 * z);
            mapCtx.setLineDash([]);
            mapCtx.stroke();
          }
        }
      }
    }

    // Region rectangles (rotated)
    for (let i = regions.length - 1; i >= 0; i--) {
      const rgn = regions[i];
      const c = worldToCanvas(rgn.cx, rgn.cy);
      const w = (rgn.width || Math.abs(rgn.x2 - rgn.x1)) * _mt.imgScale;
      const h = (rgn.height || Math.abs(rgn.y2 - rgn.y1)) * _mt.imgScale;
      if (w < 1 || h < 1) continue;
      const rot = -(rgn.rotation || 0);
      const colorHex = rgn.color || '#4A90D9';
      const hovered = (i === regionHoverIdx);

      mapCtx.save();
      mapCtx.translate(c.x, c.y);
      mapCtx.rotate(rot);

      mapCtx.fillStyle = hexToRGBA(colorHex, hovered ? 0.25 : 0.12);
      mapCtx.fillRect(-w / 2, -h / 2, w, h);
      mapCtx.strokeStyle = hexToRGBA(colorHex, hovered ? 0.9 : 0.55);
      mapCtx.lineWidth = Math.max(1.5, (hovered ? 3 : 2) * z);
      mapCtx.setLineDash([]);
      mapCtx.strokeRect(-w / 2, -h / 2, w, h);

      mapCtx.restore();

      const fontSize = Math.max(10, 13 * z);
      mapCtx.font = '500 ' + fontSize + 'px Roboto, sans-serif';
      mapCtx.textAlign = 'center';
      mapCtx.textBaseline = 'middle';
      mapCtx.fillStyle = hexToRGBA(colorHex, 0.85);
      mapCtx.fillText(rgn.name, c.x, c.y);
      mapCtx.textAlign = 'start';
      mapCtx.textBaseline = 'alphabetic';
    }

    // Region tool preview — anchor corner + diagonally opposite mouse corner
    if (activeTool === 'region' && regionToolStep === 1) {
      const c1 = worldToCanvas(regionAnchorX, regionAnchorY);
      const c2 = worldToCanvas(regionMouseX, regionMouseY);
      const dx = c2.x - c1.x, dy = c2.y - c1.y;
      const dist = Math.hypot(dx, dy);
      if (dist > 3) {
        const diagAngle = Math.atan2(dy, dx);
        const sideCanvas = dist / Math.SQRT2;
        const rot = Math.PI / 4 - diagAngle;

        const centerCanvasX = (c1.x + c2.x) / 2;
        const centerCanvasY = (c1.y + c2.y) / 2;

        mapCtx.save();
        mapCtx.translate(centerCanvasX, centerCanvasY);
        mapCtx.rotate(-rot);

        mapCtx.fillStyle = 'rgba(99, 148, 255, 0.1)';
        mapCtx.fillRect(-sideCanvas / 2, -sideCanvas / 2, sideCanvas, sideCanvas);
        mapCtx.strokeStyle = 'rgba(99, 148, 255, 0.8)';
        mapCtx.lineWidth = Math.max(1.5, 2.5 * z);
        mapCtx.setLineDash([]);
        mapCtx.strokeRect(-sideCanvas / 2, -sideCanvas / 2, sideCanvas, sideCanvas);

        mapCtx.restore();

        mapCtx.fillStyle = 'rgba(99, 148, 255, 0.85)';
        [c1, c2].forEach(p => {
          mapCtx.beginPath();
          mapCtx.arc(p.x, p.y, 4 * z, 0, Math.PI * 2);
          mapCtx.fill();
        });

        mapCtx.beginPath();
        mapCtx.setLineDash([4 * z, 4 * z]);
        mapCtx.moveTo(c1.x, c1.y);
        mapCtx.lineTo(c2.x, c2.y);
        mapCtx.strokeStyle = 'rgba(99, 148, 255, 0.5)';
        mapCtx.lineWidth = 1.5 * z;
        mapCtx.stroke();
        mapCtx.setLineDash([]);

        const worldDist = Math.hypot(regionMouseX - regionAnchorX, regionMouseY - regionAnchorY);
        const sideM = (worldDist / Math.SQRT2).toFixed(1);
        const lbl = sideM + '\u00d7' + sideM + 'm';
        mapCtx.font = (11 * z) + 'px monospace';
        mapCtx.fillStyle = 'rgba(99, 148, 255, 0.85)';
        mapCtx.fillText(lbl, c1.x + 4, c1.y - 6);
      }
    }
  }

  function repaint() {
    resizeCanvas();
    mapCtx.clearRect(0, 0, cssW, cssH);
    renderMap();
    drawOverlays();
  }

  // ── 地图模式检测 (3秒无变化 → 导航模式) ──
  let mapDataPrev = null;      // previous map data for change detection
  let mapStableTimer = null;   // timer to mark map as stable (nav mode)
  const MAP_STABLE_MS = 3000;  // no map changes for 3s = nav mode

  // ── Subscribers ──────────────────────────────

  function initSubscribers() {
    // /amcl_pose — 定位精度 (放在最前确保注册)
    const amclTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/amcl_pose',
      messageType: 'geometry_msgs/PoseWithCovarianceStamped',
      throttle_rate: 200
    });
    amclTopic.subscribe((msg) => {
      const t0 = performance.now();
      try {
        const cov = msg.pose.covariance;
        const el = $('covValue'), badge = $('covBadge');
        if (!el || !badge) { console.warn('cov elements missing'); return; }
        if (!cov || !Array.isArray(cov) || cov.length < 36) {
          el.textContent = cov ? 'len' + cov.length : 'null';
          return;
        }
        const posErr = Math.sqrt(Math.max(0, cov[0] + cov[7]));
        el.textContent = posErr.toFixed(3) + 'm';
        badge.style.color = posErr <= 0.15 ? '#3fb950' : posErr <= 0.5 ? '#d29922' : '#f85149';
      } catch(e) { console.error('amcl_pose cb error:', e); }
    });

    // /odom — 30Hz
    new ROSLIB.Topic({ ros: ros, name: '/odom', messageType: 'nav_msgs/Odometry', throttle_rate: 33, queue_length: 1 })
      .subscribe((msg) => {
        const v = msg.twist.twist.linear.x;
        const wz = msg.twist.twist.angular.z;
        const yaw = 2 * Math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
        valVel.textContent   = v.toFixed(2) + ' m/s';
        valGyroZ.textContent = wz.toFixed(3) + ' rad/s';
        valYaw.textContent   = (yaw * 180 / Math.PI).toFixed(1) + '°';
      });

    // /imu/data — 30Hz
    new ROSLIB.Topic({ ros: ros, name: '/imu/data', messageType: 'sensor_msgs/Imu', throttle_rate: 33, queue_length: 1 })
      .subscribe((msg) => {
        const q = msg.orientation;
        const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        const yaw  = Math.atan2(siny_cosp, cosy_cosp);
        const sinp = 2 * (q.w * q.y - q.z * q.x);
        const pitch = Math.asin(Math.max(-1, Math.min(1, sinp)));
        const sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
        const cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
        const roll = Math.atan2(sinr_cosp, cosr_cosp);
        valRoll.textContent  = (roll * 180 / Math.PI).toFixed(1) + '°';
        valPitch.textContent = (pitch * 180 / Math.PI).toFixed(1) + '°';
      });

    // /joint_states — 30Hz
    new ROSLIB.Topic({ ros: ros, name: '/joint_states', messageType: 'sensor_msgs/JointState', throttle_rate: 33, queue_length: 1 })
      .subscribe((msg) => {
        if (msg.position.length < 4) return;
        const thetaL = msg.position[0];
        const L0L = msg.position[1];
        const thetaR = msg.position[2];
        const L0R = msg.position[3];
        const twL = (msg.effort.length > 1) ? msg.effort[1] : 0;
        const twR = (msg.effort.length > 3) ? msg.effort[3] : 0;
        const pThetaL = ((thetaL + 1) / 2 * 100).toFixed(0);
        const pThetaR = ((thetaR + 1) / 2 * 100).toFixed(0);
        $('barThetaL').style.width = Math.max(0, Math.min(100, pThetaL)) + '%';
        $('txtThetaL').textContent = thetaL.toFixed(2) + ' rad';
        $('barThetaR').style.width = Math.max(0, Math.min(100, pThetaR)) + '%';
        $('txtThetaR').textContent = thetaR.toFixed(2) + ' rad';
        const pL0L = ((L0L - 0.11) / (0.22 - 0.11) * 100).toFixed(0);
        const pL0R = ((L0R - 0.11) / (0.22 - 0.11) * 100).toFixed(0);
        $('barL0L').style.width = Math.max(0, Math.min(100, pL0L)) + '%';
        $('txtL0L').textContent = L0L.toFixed(3) + ' m';
        $('barL0R').style.width = Math.max(0, Math.min(100, pL0R)) + '%';
        $('txtL0R').textContent = L0R.toFixed(3) + ' m';
        const pTwL = ((twL + 5) / 10 * 100).toFixed(0);
        const pTwR = ((twR + 5) / 10 * 100).toFixed(0);
        $('barTwL').style.width = Math.max(0, Math.min(100, pTwL)) + '%';
        $('txtTwL').textContent = twL.toFixed(2) + ' Nm';
        $('barTwR').style.width = Math.max(0, Math.min(100, pTwR)) + '%';
        $('txtTwR').textContent = twR.toFixed(2) + ' Nm';
      });

    // /battery — 5Hz
    new ROSLIB.Topic({ ros: ros, name: '/battery', messageType: 'sensor_msgs/BatteryState', throttle_rate: 200, queue_length: 1 })
      .subscribe((msg) => { valBattery.textContent = msg.voltage.toFixed(1) + ' V'; });

    // /chassis_state — 20Hz
    new ROSLIB.Topic({ ros: ros, name: '/chassis_state', messageType: 'std_msgs/Int8MultiArray', throttle_rate: 50, queue_length: 1 })
      .subscribe((msg) => {
        if (msg.data.length < 4) return;
        const [startFlag, jumpFlag, cL, cR] = msg.data;
        badgeStart.textContent = startFlag ? '运行中' : '停止';
        badgeStart.className = 'status-badge' + (startFlag ? ' active' : '');
        if (cL) { $('contactL').classList.add('on'); $('contactL').querySelector('span:last-child').textContent = '左腿: 着地'; }
        else { $('contactL').classList.remove('on'); $('contactL').querySelector('span:last-child').textContent = '左腿: 离地'; }
        if (cR) { $('contactR').classList.add('on'); $('contactR').querySelector('span:last-child').textContent = '右腿: 着地'; }
        else { $('contactR').classList.remove('on'); $('contactR').querySelector('span:last-child').textContent = '右腿: 离地'; }
        const jsEl = $('jumpState');
        if (jumpFlag > 0) { jsEl.classList.add('on'); jsEl.querySelector('span:last-child').textContent = '跳跃: 阶段 ' + jumpFlag; }
        else { jsEl.classList.remove('on'); jsEl.querySelector('span:last-child').textContent = '跳跃: 就绪'; }
      });

    // /map (OccupancyGrid) — 无节流
    new ROSLIB.Topic({ ros: ros, name: '/map', messageType: 'nav_msgs/OccupancyGrid' })
      .subscribe((msg) => {
        mapInfo = {
          width: msg.info.width,
          height: msg.info.height,
          resolution: msg.info.resolution,
          origin: { x: msg.info.origin.position.x, y: msg.info.origin.position.y }
        };
        mapData = new Int8Array(msg.data);
        mapImageData = null;
        mapDirty = true;
        overlayDirty = true;

        $('mapOfflineOverlay').classList.add('hidden');

        // Detect map change via sampling (avoid call stack overflow on large maps)
        var sampleLen = Math.min(mapData.length, 1000);
        var sampleStr = '';
        for (var si = 0; si < sampleLen; si++) sampleStr += String.fromCharCode(mapData[si]);
        if (mapDataPrev !== null && sampleStr !== mapDataPrev) {
          // Map is being updated → SLAM mode
          mapMode.textContent = 'SLAM 建图中';
          mapMode.className = 'map-mode-badge active';
        } else if (mapDataPrev === null) {
          // First map received → show loading, start stability timer
          mapMode.textContent = '加载地图中...';
          mapMode.className = 'map-mode-badge active';
        }
        mapDataPrev = sampleStr;

        // Reset stability timer — if map is stable for 3s, switch to nav mode
        if (mapStableTimer) clearTimeout(mapStableTimer);
        mapStableTimer = setTimeout(function () {
          mapMode.textContent = '导航中';
          mapMode.className = 'map-mode-badge active';
        }, MAP_STABLE_MS);
      });

    // /scan (LaserScan) — 10Hz
    new ROSLIB.Topic({ ros: ros, name: '/scan', messageType: 'sensor_msgs/LaserScan', throttle_rate: 100, queue_length: 1 })
      .subscribe((msg) => {
        const points = [];
        let angle = msg.angle_min;
        for (let i = 0; i < msg.ranges.length; i++) {
          const r = msg.ranges[i];
          if (r > msg.range_min && r < msg.range_max) {
            const wx = robotX + r * Math.cos(robotYaw + angle);
            const wy = robotY + r * Math.sin(robotYaw + angle);
            points.push({ x: wx, y: wy });
          }
          angle += msg.angle_increment;
        }
        scanPoints = points;
        overlayDirty = true;
      });

    // /plan or /received_global_plan (Path) — 无节流
    new ROSLIB.Topic({ ros: ros, name: '/plan', messageType: 'nav_msgs/Path' })
      .subscribe((msg) => {
        pathPoints = msg.poses.map(p => ({ x: p.pose.position.x, y: p.pose.position.y }));
        overlayDirty = true;
      });

    // /tf — 30Hz  组合 map→odom 与 odom→base_footprint → 机器人全局位姿
    let tfMapOdom = { x: 0, y: 0, yaw: 0 };
    let tfOdomBase = { x: 0, y: 0, yaw: 0 };
    let hasTfMapOdom = false;
    let hasTfOdomBase = false;

    function qToYaw(q) {
      return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    }

    function updateRobotPose() {
      if (!hasTfMapOdom && !hasTfOdomBase) return;
      // T_map_base = T_map_odom * T_odom_base
      const x = tfMapOdom.x + tfOdomBase.x * Math.cos(tfMapOdom.yaw) - tfOdomBase.y * Math.sin(tfMapOdom.yaw);
      const y = tfMapOdom.y + tfOdomBase.x * Math.sin(tfMapOdom.yaw) + tfOdomBase.y * Math.cos(tfMapOdom.yaw);
      const yaw = tfMapOdom.yaw + tfOdomBase.yaw;
      if (robotX !== x || robotY !== y || robotYaw !== yaw) {
        robotX = x; robotY = y; robotYaw = yaw;
        robotVisible = hasTfOdomBase;  // 只有当 odom→base 有效时才显示机器人
        overlayDirty = true;
        updateMapModeNav();
      }
    }

    new ROSLIB.Topic({ ros: ros, name: '/tf', messageType: 'tf2_msgs/TFMessage', throttle_rate: 33, queue_length: 1 })
      .subscribe((msg) => {
        for (const t of msg.transforms) {
          if (t.header.frame_id === 'map' && t.child_frame_id === 'odom') {
            tfMapOdom.x = t.transform.translation.x;
            tfMapOdom.y = t.transform.translation.y;
            tfMapOdom.yaw = qToYaw(t.transform.rotation);
            hasTfMapOdom = true;
            updateRobotPose();
          }
          if (t.header.frame_id === 'odom' && t.child_frame_id === 'base_footprint') {
            tfOdomBase.x = t.transform.translation.x;
            tfOdomBase.y = t.transform.translation.y;
            tfOdomBase.yaw = qToYaw(t.transform.rotation);
            hasTfOdomBase = true;
            updateRobotPose();
      }
    }
  });

    // /region_manager/regions
    new ROSLIB.Topic({ ros: ros, name: '/region_manager/regions', messageType: 'std_msgs/String' })
      .subscribe(function (msg) {
        try { regions = JSON.parse(msg.data); overlayDirty = true; updateRegionList(); } catch (e) {}
      });

    // /region_manager/response
    new ROSLIB.Topic({ ros: ros, name: '/region_manager/response', messageType: 'std_msgs/String' })
      .subscribe(function (msg) {
        try { const resp = JSON.parse(msg.data); showToast(resp.message); } catch (e) {}
      });
  }

  function hexToRGBA(hex, alpha) {
    var h = hex.replace('#', '');
    var r = parseInt(h.substring(0, 2), 16);
    var g = parseInt(h.substring(2, 4), 16);
    var b = parseInt(h.substring(4, 6), 16);
    return 'rgba(' + r + ',' + g + ',' + b + ',' + alpha + ')';
  }

  function updateRegionList() {
    var list = document.getElementById('regionList');
    if (!list) return;
    var count = document.getElementById('regionCount');
    if (count) count.textContent = regions.length + '\u4e2a';

    if (regions.length === 0) {
      list.innerHTML = '<span style="font-size:12px;color:var(--md-sys-color-on-surface-variant);">\u6682\u65e0\u533a\u57df</span>';
      return;
    }

    var html = '';
    for (var i = 0; i < regions.length; i++) {
      var r = regions[i];
      html += '<div class="region-item" data-name="' + r.name + '">' +
        '<span class="region-item__color" style="background:' + r.color + '"></span>' +
        '<span class="region-item__name">' + r.name + '</span>' +
        '<button class="region-item__btn region-item__nav" title="\u5bfc\u822a">\u25b6</button>' +
        '<button class="region-item__btn region-item__del" title="\u5220\u9664">\u2715</button>' +
        '</div>';
    }
    list.innerHTML = html;

    var items = list.querySelectorAll('.region-item');
    for (var j = 0; j < items.length; j++) {
      (function (item, name) {
        item.addEventListener('click', function (e) {
          if (e.target.classList.contains('region-item__del')) {
            if (confirm('\u786e\u5b9a\u5220\u9664\u533a\u57df \u201c' + name + '\u201d \uff1f')) {
              pubRegionDelete.publish({ data: JSON.stringify({ name: name }) });
            }
            return;
          }
          if (e.target.classList.contains('region-item__nav') || !e.target.closest('button')) {
            pubRegionNavigate.publish({ data: JSON.stringify({ name: name }) });
          }
        });
      })(items[j], regions[j].name);
    }
  }

  // 定时重绘地图 (20Hz), 避免高频 subscriber 触发过多 repaint
  setInterval(() => {
    if (overlayDirty || mapDirty) {
      overlayDirty = false;
      repaint();
    }
  }, 50);

  // 窗口尺寸变化 → 立即自适应
  window.addEventListener('resize', () => { mapDirty = true; overlayDirty = true; });

  function updateMapModeNav() {
    mapMode.textContent = '导航中';
    mapMode.className = 'map-mode-badge active';
  }

  function rosStamp() {
    const ms = Date.now();
    return { secs: Math.floor(ms / 1000), nsecs: (ms % 1000) * 1e6 };
  }

  /* ═══════════════════════════════════════════════════
   *  地图 Canvas 交互: 拖拽平移 / 滚轮缩放 / 工具点击
   *  Pan: 非工具模式下鼠标拖拽平移
   *  Zoom: 滚轮以光标为中心缩放 + 双指捏合（触屏）
   *  Tools: 导航目标 / 初始位姿 / 人体跟随 / 区域划分
   *         点击第一下设位置 → 第二下设方向/确认
   ═══════════════════════════════════════════════════ */
  let panDragging = false, panStartX = 0, panStartY = 0;

  mapCanvas.addEventListener('mousedown', (e) => {
    if (activeTool) return;  // tool mode → handled by click
    panDragging = true;
    panStartX = e.clientX;
    panStartY = e.clientY;
    mapCanvas.style.cursor = 'grabbing';
  });

  window.addEventListener('mousemove', (e) => {
    if (panDragging) {
      panX += (e.clientX - panStartX);
      panY += (e.clientY - panStartY);
      panStartX = e.clientX;
      panStartY = e.clientY;
      overlayDirty = true;
    }
    // Init-pose + goal preview: track cursor world position for 2nd step
    if (mapInfo && (activeTool === 'initpose' && (initPoseX || initPoseY))) {
      const rect = mapCanvas.getBoundingClientRect();
      const world = canvasToWorld(e.clientX - rect.left, e.clientY - rect.top);
      if (world.wx !== initPosePrevX || world.wy !== initPosePrevY) {
        initPosePrevX = world.wx; initPosePrevY = world.wy;
        overlayDirty = true;
      }
    }
    if (mapInfo && (activeTool === 'goal' && (goalPoseX || goalPoseY))) {
      const rect = mapCanvas.getBoundingClientRect();
      const world = canvasToWorld(e.clientX - rect.left, e.clientY - rect.top);
      if (world.wx !== goalPosePrevX || world.wy !== goalPosePrevY) {
        goalPosePrevX = world.wx; goalPosePrevY = world.wy;
        overlayDirty = true;
      }
    }
    // Follow rectangle preview: track corner2 while in step 1
    if (mapInfo && activeTool === 'follow' && followRectStep === 1) {
      var fr = mapCanvas.getBoundingClientRect();
      var fw = canvasToWorld(e.clientX - fr.left, e.clientY - fr.top);
      if (fw.wx !== followRectX2 || fw.wy !== followRectY2) {
        followRectX2 = fw.wx; followRectY2 = fw.wy;
        overlayDirty = true;
      }
    }
    // Region tool preview: track mouse while dragging
    if (mapInfo && activeTool === 'region' && regionToolStep === 1) {
      var rr = mapCanvas.getBoundingClientRect();
      var rw = canvasToWorld(e.clientX - rr.left, e.clientY - rr.top);
      if (rw.wx !== regionMouseX || rw.wy !== regionMouseY) {
        regionMouseX = rw.wx; regionMouseY = rw.wy;
        overlayDirty = true;
      }
    }
    // Region hover detection — rotated rectangle hit test
    if (mapInfo && !activeTool && !panDragging) {
      var rh = mapCanvas.getBoundingClientRect();
      var wh = canvasToWorld(e.clientX - rh.left, e.clientY - rh.top);
      var found = -1;
      for (var k = regions.length - 1; k >= 0; k--) {
        var rg = regions[k];
        var dx = wh.wx - (rg.cx !== undefined ? rg.cx : (rg.x1 + rg.x2) / 2);
        var dy = wh.wy - (rg.cy !== undefined ? rg.cy : (rg.y1 + rg.y2) / 2);
        var rot = rg.rotation || 0;
        var cos = Math.cos(-rot), sin = Math.sin(-rot);
        var lx = dx * cos - dy * sin;
        var ly = dx * sin + dy * cos;
        var hw = (rg.width !== undefined ? rg.width : Math.abs(rg.x2 - rg.x1)) / 2;
        var hh = (rg.height !== undefined ? rg.height : Math.abs(rg.y2 - rg.y1)) / 2;
        if (Math.abs(lx) <= hw && Math.abs(ly) <= hh) {
          found = k; break;
        }
      }
      if (found !== regionHoverIdx) {
        regionHoverIdx = found;
        mapCanvas.style.cursor = found >= 0 ? 'pointer' : 'crosshair';
        overlayDirty = true;
      }
    }
  });

  window.addEventListener('mouseup', () => {
    if (!panDragging) return;
    panDragging = false;
    mapCanvas.style.cursor = activeTool ? 'crosshair' : 'crosshair';
  });

  // 滚轮缩放: 以光标指向的世界点为中心 (平移补偿)
  mapCanvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    const rect = mapCanvas.getBoundingClientRect();
    const cx = e.clientX - rect.left, cy = e.clientY - rect.top;

    // Remember world point under cursor
    const world = canvasToWorld(cx, cy);

    // Change zoom
    zoomLevel = e.deltaY < 0
      ? Math.min(ZOOM_MAX, zoomLevel + ZOOM_STEP)
      : Math.max(ZOOM_MIN, zoomLevel - ZOOM_STEP);

    // Recompute transform with new zoom
    updateMapTransform();

    // Find where the world point lands now
    const newPos = worldToCanvas(world.wx, world.wy);

    // Shift pan to bring it back under cursor
    panX += cx - newPos.x;
    panY += cy - newPos.y;

    updateMapTransform();
    updateZoomDisplay();
    overlayDirty = true;
  }, { passive: false });

  // 触屏双指捏合缩放
  let pinchStartDist = 0, pinchStartZoom = 1.0;
  mapCanvas.addEventListener('touchstart', (e) => {
    if (e.touches.length === 2) {
      pinchStartDist = Math.hypot(
        e.touches[0].clientX - e.touches[1].clientX,
        e.touches[0].clientY - e.touches[1].clientY);
      pinchStartZoom = zoomLevel;
    }
  }, { passive: true });

  mapCanvas.addEventListener('touchmove', (e) => {
    if (e.touches.length === 2) {
      const dist = Math.hypot(
        e.touches[0].clientX - e.touches[1].clientX,
        e.touches[0].clientY - e.touches[1].clientY);
      if (pinchStartDist > 0) {
        zoomLevel = Math.min(ZOOM_MAX, Math.max(ZOOM_MIN, pinchStartZoom * (dist / pinchStartDist)));
        updateZoomDisplay();
        overlayDirty = true;
      }
    }
  }, { passive: true });

  function updateZoomDisplay() {
    const el = $('zoomLevelDisplay');
    if (el) el.textContent = Math.round(zoomLevel * 100) + '%';
  }

  // Expose for HTML buttons
  window.mapZoomIn = () => { zoomLevel = Math.min(ZOOM_MAX, zoomLevel + ZOOM_STEP); updateZoomDisplay(); overlayDirty = true; };
  window.mapZoomOut = () => { zoomLevel = Math.max(ZOOM_MIN, zoomLevel - ZOOM_STEP); updateZoomDisplay(); overlayDirty = true; };
  window.mapZoomReset = () => { zoomLevel = 1.0; panX = 0; panY = 0; updateZoomDisplay(); overlayDirty = true; };

  mapCanvas.addEventListener('click', (e) => {
    if (panDragging) return;
    if (!mapInfo) return;

    const rect = mapCanvas.getBoundingClientRect();
    const px = e.clientX - rect.left;
    const py = e.clientY - rect.top;
    const world = canvasToWorld(px, py);

    // Region click navigation (when no active tool, clicking a region navigates to its center)
    if (!activeTool && regionHoverIdx >= 0) {
      var rgn2 = regions[regionHoverIdx];
      var cvx = rgn2.cx !== undefined ? rgn2.cx : (rgn2.x1 + rgn2.x2) / 2;
      var cvy = rgn2.cy !== undefined ? rgn2.cy : (rgn2.y1 + rgn2.y2) / 2;
      pubRegionNavigate.publish({ data: JSON.stringify({ name: rgn2.name, cx: cvx, cy: cvy }) });
      activeTool = null;
    }

    if (!activeTool) return;

    if (activeTool === 'goal') {
      if (!goalPoseX && !goalPoseY) {
        goalPoseX = world.wx;
        goalPoseY = world.wy;
        updateBtnTitle($('btnNavGoal'), '点击选择方向');
      } else {
        const yaw = Math.atan2(world.wy - goalPoseY, world.wx - goalPoseX);
        const cy = Math.cos(yaw * 0.5), sy = Math.sin(yaw * 0.5);
        pubGoalPose.publish({
          header: { frame_id: 'map', stamp: rosStamp() },
          pose: {
            position: { x: goalPoseX, y: goalPoseY, z: 0 },
            orientation: { x: 0, y: 0, z: sy, w: cy }
          }
        });
        activeTool = null;
        goalPoseX = 0; goalPoseY = 0; goalPosePrevX = 0; goalPosePrevY = 0;
        updateBtnTitle($('btnNavGoal'), '导航目标');
        resetToolBtns();
      }
    } else if (activeTool === 'follow') {
      if (followRectStep === 0) {
        followRectX1 = world.wx;
        followRectY1 = world.wy;
        followRectX2 = world.wx;
        followRectY2 = world.wy;
        followRectStep = 1;
        overlayDirty = true;
      } else {
        const cx = (followRectX1 + world.wx) / 2;
        const cy = (followRectY1 + world.wy) / 2;
        const r = Math.max(0.1, Math.hypot(world.wx - followRectX1, world.wy - followRectY1) / 2);
        followTargetX = cx;
        followTargetY = cy;
        followRadius = r;
        followRectX2 = world.wx;
        followRectY2 = world.wy;
        followRectStep = 0;
        followRectW = Math.abs(world.wx - followRectX1);
        followRectH = Math.abs(world.wy - followRectY1);
        pubFollowTarget.publish({
          header: { frame_id: 'map', stamp: rosStamp() },
          point: { x: cx, y: cy, z: 0 }
        });
        pubFollowRadius.publish({ data: r });
        if (pubFollowActive) pubFollowActive.publish({ data: true });
        overlayDirty = true;
      }
    } else if (activeTool === 'region') {
      if (regionToolStep === 0) {
        regionAnchorX = world.wx;
        regionAnchorY = world.wy;
        regionMouseX = world.wx;
        regionMouseY = world.wy;
        regionToolStep = 1;
        updateBtnTitle($('btnRegion'), '移动鼠标选择对角顶点，点击确认');
        overlayDirty = true;
      } else {
        var dx = world.wx - regionAnchorX;
        var dy = world.wy - regionAnchorY;
        var diag = Math.hypot(dx, dy);
        if (diag < 0.05) { overlayDirty = true; return; }
        var side = diag / Math.SQRT2;
        var rotation = Math.atan2(dy, dx) - Math.PI / 4;
        var cx = (regionAnchorX + world.wx) / 2;
        var cy = (regionAnchorY + world.wy) / 2;
        var rName = prompt('\u8bf7\u8f93\u5165\u533a\u57df\u540d\u79f0\uff1a', '\u533a\u57df' + (regions.length + 1));
        if (!rName || !rName.trim()) { activeTool = null; regionToolStep = 0; overlayDirty = true; resetToolBtns(); return; }
        var defaultColor = REGION_COLORS[regionColorIdx % REGION_COLORS.length];
        var rColor = prompt('\u8bf7\u8f93\u5165\u5341\u516d\u8fdb\u5236\u989c\u8272\u4ee3\u7801\uff1a', defaultColor);
        if (!rColor || !rColor.trim()) rColor = defaultColor;
        regionColorIdx++;
        pubRegionSave.publish({ data: JSON.stringify({
          name: rName.trim(),
          cx: cx, cy: cy,
          width: side, height: side,
          rotation: rotation,
          color: rColor.trim()
        })});
        activeTool = null;
        regionToolStep = 0;
        resetToolBtns();
        updateBtnTitle($('btnRegion'), '区域划分工具');
      }
    } else if (activeTool === 'initpose') {
      // First click sets position, second click sets direction
      if (!initPoseX && !initPoseY) {
        initPoseX = world.wx;
        initPoseY = world.wy;
        updateBtnTitle($('btnInitPose'), '点击选择朝向');
      } else {
        const yaw = Math.atan2(world.wy - initPoseY, world.wx - initPoseX);
        const cy = Math.cos(yaw * 0.5), sy = Math.sin(yaw * 0.5);
        pubInitPose.publish({
          header: { frame_id: 'map', stamp: rosStamp() },
          pose: {
            pose: {
              position: { x: initPoseX, y: initPoseY, z: 0 },
              orientation: { x: 0, y: 0, z: sy, w: cy }
            },
            covariance: Array(36).fill(0)
          }
        });
        activeTool = null;
        initPoseX = 0;
        initPoseY = 0; initPosePrevX = 0; initPosePrevY = 0;
        updateBtnTitle($('btnInitPose'), '设初始位姿');
        resetToolBtns();
      }
    }
  });

  function resetToolBtns() {
    ['btnNavGoal', 'btnInitPose', 'btnFollow', 'btnRegion'].forEach(id => {
      $(id).classList.remove('active');
    });
  }

  /* ═══════════════════════════════════════════════════
   *  地图工具栏按钮事件
   *  导航目标 / 初始位姿 / 取消导航 / 人体跟随 / 区域管理
   *  全局重定位 / 保存地图
   ═══════════════════════════════════════════════════ */
  $('btnNavGoal').addEventListener('click', () => {
    activeTool = activeTool === 'goal' ? null : 'goal';
    initPoseX = 0; initPoseY = 0; initPosePrevX = 0; initPosePrevY = 0;
    goalPoseX = 0; goalPoseY = 0; goalPosePrevX = 0; goalPosePrevY = 0;
    followTargetX = 0; followTargetY = 0; followRectW = 0; followRectH = 0;
    regionToolStep = 0;
    updateBtnTitle($('btnNavGoal'), '导航目标');
    $('btnNavGoal').classList.toggle('active', activeTool === 'goal');
    $('btnInitPose').classList.remove('active');
    $('btnFollow').classList.remove('active');
    $('btnRegion').classList.remove('active');
    mapCanvas.style.cursor = activeTool === 'goal' ? 'crosshair' : 'crosshair';
  });

  $('btnInitPose').addEventListener('click', () => {
    activeTool = activeTool === 'initpose' ? null : 'initpose';
    initPoseX = 0; initPoseY = 0; initPosePrevX = 0; initPosePrevY = 0;
    goalPoseX = 0; goalPoseY = 0; goalPosePrevX = 0; goalPosePrevY = 0;
    followTargetX = 0; followTargetY = 0; followRectW = 0; followRectH = 0;
    regionToolStep = 0;
    updateBtnTitle($('btnInitPose'), '设初始位姿');
    $('btnInitPose').classList.toggle('active', activeTool === 'initpose');
    $('btnNavGoal').classList.remove('active');
    $('btnFollow').classList.remove('active');
    $('btnRegion').classList.remove('active');
    mapCanvas.style.cursor = activeTool === 'initpose' ? 'crosshair' : 'crosshair';
  });

  $('btnCancelNav').addEventListener('click', () => {
    if (pubCmdVel) pubCmdVel.publish({ linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } });
    // 以机器人当前位置作为新目标，Nav2 判定"已到达"即终止当前导航
    if (pubGoalPose && robotVisible) {
      pubGoalPose.publish({
        header: { frame_id: 'map', stamp: rosStamp() },
        pose: {
          position: { x: robotX, y: robotY, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 }
        }
      });
    }
    // 同时关闭人体跟随
    if (pubFollowActive) pubFollowActive.publish({ data: false });
  });

  // 人体跟随 — 双击地图设置跟随目标
  $('btnFollow').addEventListener('click', () => {
    activeTool = activeTool === 'follow' ? null : 'follow';
    followTargetX = 0; followTargetY = 0; followRectW = 0; followRectH = 0;
    followRectStep = 0;
    initPoseX = 0; initPoseY = 0; initPosePrevX = 0; initPosePrevY = 0;
    goalPoseX = 0; goalPoseY = 0; goalPosePrevX = 0; goalPosePrevY = 0;
    regionToolStep = 0;
    $('btnFollow').classList.toggle('active', activeTool === 'follow');
    $('btnNavGoal').classList.remove('active');
    $('btnInitPose').classList.remove('active');
    $('btnRegion').classList.remove('active');
    mapCanvas.style.cursor = activeTool === 'follow' ? 'crosshair' : 'crosshair';
    if (activeTool !== 'follow' && pubFollowActive) pubFollowActive.publish({ data: false });
    overlayDirty = true;
  });

  // 区域管理 — 矩形框选划分区域
  const REGION_COLORS = [
    '#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4', '#FFEAA7', '#DDA0DD',
    '#98D8C8', '#F7DC6F', '#BB8FCE', '#85C1E9', '#F8C471', '#82E0AA'
  ];
  var regionColorIdx = 0;

  $('btnRegion').addEventListener('click', () => {
    activeTool = activeTool === 'region' ? null : 'region';
    regionToolStep = 0;
    followTargetX = 0; followTargetY = 0; followRectW = 0; followRectH = 0;
    followRectStep = 0;
    initPoseX = 0; initPoseY = 0; initPosePrevX = 0; initPosePrevY = 0;
    goalPoseX = 0; goalPoseY = 0; goalPosePrevX = 0; goalPosePrevY = 0;
    $('btnRegion').classList.toggle('active', activeTool === 'region');
    $('btnNavGoal').classList.remove('active');
    $('btnInitPose').classList.remove('active');
    $('btnFollow').classList.remove('active');
    mapCanvas.style.cursor = activeTool === 'region' ? 'crosshair' : 'crosshair';
    if (activeTool !== 'follow' && pubFollowActive) pubFollowActive.publish({ data: false });
    overlayDirty = true;
  });

  // 全局重定位 — 调用 AMCL reinitialize_global_localization
  $('btnRelocalize').addEventListener('click', () => {
    const srv = new ROSLIB.Service({
      ros: ros,
      name: '/reinitialize_global_localization',
      serviceType: 'std_srvs/srv/Empty'
    });
    const req = new ROSLIB.ServiceRequest({});
    srv.callService(req, () => {
      showToast('全局重定位已触发，AMCL 正在扫描匹配...');
    }, () => {
      showToast('重定位服务不可用，请确认 AMCL 已启动');
    });
  });

  $('btnSaveMap').addEventListener('click', () => {
    const saveSrv = new ROSLIB.Service({
      ros: ros,
      name: '/map_saver/save_map',
      serviceType: 'nav2_msgs/srv/SaveMap'
    });
    const req = new ROSLIB.ServiceRequest({
      map_topic: '/map',
      map_url: '',
      image_format: 'pgm',
      map_mode: 'trinary',
      free_thresh: 0.25,
      occupied_thresh: 0.65
    });
    saveSrv.callService(req, (res) => {
      if (res && res.result) {
        showToast('地图保存成功');
      } else {
        showToast('地图保存失败');
      }
    }, (err) => {
      showToast('保存服务不可用');
    });
  });

  function showToast(msg) {
    let el = document.querySelector('.snackbar');
    if (!el) {
      el = document.createElement('div');
      el.className = 'snackbar';
      document.body.appendChild(el);
    }
    el.textContent = msg;
    el.classList.add('show');
    clearTimeout(el._timer);
    el._timer = setTimeout(() => el.classList.remove('show'), 2500);
  }

  /* ═══════════════════════════════════════════════════
   *  控制面板按钮事件
   *  使能 / 停止 / 急停 / 跳跃 / 自起
   ═══════════════════════════════════════════════════ */
  $('btnEnable').addEventListener('click', () => {
    isEnabled = true;
    pubEnable.publish({ data: true });
    // 同时发送当前滑块速度, 避免使能瞬间 v=0 导致小车停止
    const v = parseFloat($('sliderVset').value);
    const wz = parseFloat($('sliderYawRate').value);
    pubCmdVel.publish({ linear: { x: v, y: 0, z: 0 }, angular: { x: 0, y: 0, z: wz } });
    updateEnableBadge();
  });
  $('btnDisable').addEventListener('click', () => {
    isEnabled = false;
    pubEnable.publish({ data: false });
    pubCmdVel.publish({ linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } });
    updateEnableBadge();
  });
  $('btnEstop').addEventListener('click', () => {
    isEnabled = false;
    pubEstop.publish({ data: true });
    updateEnableBadge();
  });
  $('btnJump').addEventListener('click', () => { pubJump.publish({ data: true }); });
  $('btnRecover').addEventListener('click', () => { pubRecover.publish({ data: true }); });

  function updateEnableBadge() {
    badgeEnable.textContent = isEnabled ? '已使能' : '未使能';
    badgeEnable.className = 'status-badge' + (isEnabled ? ' active' : '');
  }

  /* ═══════════════════════════════════════════════════
   *  持续心跳开关 (控制指令持续下发模式)
   ═══════════════════════════════════════════════════ */
  $('chkKeepAlive').addEventListener('change', (e) => {
    pubKeepAlive.publish({ data: e.target.checked });
  });

  /* ═══════════════════════════════════════════════════
   *  控制面板滑块: 速度 / 角速度 / 腿长 / Roll / Pitch
   *  每次滑动即发布对应 ROS Topic
   ═══════════════════════════════════════════════════ */
  function bindSlider(sliderId, txtId, fmt, onSlide) {
    const slider = $(sliderId);
    const txt = $(txtId);
    slider.addEventListener('input', () => {
      const val = parseFloat(slider.value);
      txt.textContent = fmt(val);
      onSlide(val);
    });
  }

  bindSlider('sliderVset', 'txtVset', (v) => v.toFixed(2), (v) => {
    if (!pubCmdVel) return;
    pubCmdVel.publish({ linear: { x: v, y: 0, z: 0 }, angular: { x: 0, y: 0, z: parseFloat($('sliderYawRate').value) } });
  });
  bindSlider('sliderYawRate', 'txtYawRate', (v) => v.toFixed(2), (v) => {
    if (!pubCmdVel) return;
    pubCmdVel.publish({ linear: { x: parseFloat($('sliderVset').value), y: 0, z: 0 }, angular: { x: 0, y: 0, z: v } });
  });
  bindSlider('sliderLegSet', 'txtLegSet', (v) => v.toFixed(3), (v) => {
    if (!pubCmdAtt) return;
    pubCmdAtt.publish({ data: [parseFloat($('sliderRollSet').value), parseFloat($('sliderPitchSet').value), v] });
  });
  bindSlider('sliderRollSet', 'txtRollSet', (v) => v.toFixed(2), (v) => {
    if (!pubCmdAtt) return;
    pubCmdAtt.publish({ data: [v, parseFloat($('sliderPitchSet').value), parseFloat($('sliderLegSet').value)] });
  });
  bindSlider('sliderPitchSet', 'txtPitchSet', (v) => v.toFixed(2), (v) => {
    if (!pubCmdAtt) return;
    pubCmdAtt.publish({ data: [parseFloat($('sliderRollSet').value), v, parseFloat($('sliderLegSet').value)] });
  });

  /* ═══════════════════════════════════════════════════
   *  虚拟摇杆 (Canvas) — 鼠标拖拽 / 触屏控制
   *  输出 /cmd_vel, 死区 15%, 最大速度 ±1.5m/s ±3.14rad/s
   ═══════════════════════════════════════════════════ */
  var jCanvas = $('joystickCanvas');
  var jCtx = jCanvas.getContext('2d');
  var joyV = $('joyV');
  var joyW = $('joyW');
  var joyArrowV = $('joyArrowV');
  var joyArrowW = $('joyArrowW');

  var MAX_V = 1.5;
  var MAX_W = 3.14;
  var DEAD_ZONE = 0.15;

  var jRadius = 0, jCx = 0, jCy = 0;
  var jActive = false;

  function resizeJoystick() {
    var comp = document.querySelector('.joystick-compact');
    if (!comp) return;
    var compW = comp.getBoundingClientRect().width;
    var size = Math.min(compW - 32, 140);
    if (size <= 0) return;
    jCanvas.width = size;
    jCanvas.height = size;
    jCx = size / 2;
    jCy = size / 2;
    jRadius = size * 0.42;
    drawJoystick(0, 0);
  }

  function arrowH(ctx, x, y, s, alpha, dir) {
    ctx.save();
    ctx.translate(x, y);
    ctx.globalAlpha = alpha;
    ctx.fillStyle = 'rgba(200,200,215,' + alpha + ')';
    ctx.beginPath();
    if (dir === 'u') { ctx.moveTo(0, -s); ctx.lineTo(-s * 0.55, s * 0.25); ctx.lineTo(s * 0.55, s * 0.25); }
    else if (dir === 'd') { ctx.moveTo(0, s); ctx.lineTo(-s * 0.55, -s * 0.25); ctx.lineTo(s * 0.55, -s * 0.25); }
    else if (dir === 'l') { ctx.moveTo(-s, 0); ctx.lineTo(s * 0.25, -s * 0.55); ctx.lineTo(s * 0.25, s * 0.55); }
    else if (dir === 'r') { ctx.moveTo(s, 0); ctx.lineTo(-s * 0.25, -s * 0.55); ctx.lineTo(-s * 0.25, s * 0.55); }
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  function drawJoystick(kx, ky) {
    var ctx = jCtx, w = jCanvas.width, h = jCanvas.height;
    ctx.clearRect(0, 0, w, h);

    var style = getComputedStyle(document.documentElement);
    var black = style.getPropertyValue('--md-sys-color-surface-container-lowest').trim() || '#0d0e13';
    var surfHi = style.getPropertyValue('--md-sys-color-surface-container-highest').trim() || '#34353a';
    var ring = style.getPropertyValue('--md-sys-color-outline-variant').trim() || '#3a3d48';
    var fill = style.getPropertyValue('--md-sys-color-primary').trim() || '#b0c6ff';

    var r2 = jRadius * 0.92;
    var r3 = jRadius * 0.35;

    // Outer glow ring
    var gradOuter = ctx.createRadialGradient(jCx, jCy, r2 * 0.85, jCx, jCy, r2 * 1.08);
    gradOuter.addColorStop(0, 'rgba(255,255,255,0)');
    gradOuter.addColorStop(0.8, ring + '18');
    gradOuter.addColorStop(1, ring + '30');
    ctx.beginPath(); ctx.arc(jCx, jCy, r2 * 1.08, 0, Math.PI * 2);
    ctx.fillStyle = gradOuter; ctx.fill();

    // Base pad (inset feel)
    var gradBase = ctx.createRadialGradient(jCx - r2 * 0.15, jCy - r2 * 0.15, 0, jCx, jCy, r2);
    gradBase.addColorStop(0, black + '80');
    gradBase.addColorStop(1, surfHi);
    ctx.beginPath(); ctx.arc(jCx, jCy, r2, 0, Math.PI * 2);
    ctx.fillStyle = gradBase; ctx.fill();
    ctx.strokeStyle = ring + '60'; ctx.lineWidth = 1.5; ctx.stroke();

    // Crosshair (subtle)
    ctx.strokeStyle = ring + '25'; ctx.lineWidth = 0.8;
    ctx.setLineDash([]);
    ctx.beginPath(); ctx.moveTo(jCx - r2 * 0.62, jCy); ctx.lineTo(jCx + r2 * 0.62, jCy);
    ctx.moveTo(jCx, jCy - r2 * 0.62); ctx.lineTo(jCx, jCy + r2 * 0.62); ctx.stroke();

    // Direction arrows at ring edge (N/S/E/W)
    var arrowS = jRadius * 0.07, alpha = 0.3;
    var edgeR = r2 - arrowS * 0.3;
    arrowH(ctx, jCx, jCy - edgeR, arrowS, alpha, 'u');
    arrowH(ctx, jCx, jCy + edgeR, arrowS, alpha, 'd');
    arrowH(ctx, jCx - edgeR, jCy, arrowS, alpha, 'l');
    arrowH(ctx, jCx + edgeR, jCy, arrowS, alpha, 'r');

    // Dead zone ring
    var dzR = jRadius * DEAD_ZONE * (r2 / jRadius);
    ctx.beginPath(); ctx.arc(jCx, jCy, dzR, 0, Math.PI * 2);
    ctx.strokeStyle = ring + '30'; ctx.lineWidth = 0.8;
    ctx.setLineDash([2, 3]); ctx.stroke(); ctx.setLineDash([]);

    // Guide line from center to knob (only when active)
    var knX = jCx + kx * jRadius;
    var knY = jCy - ky * jRadius;
    if (Math.abs(kx) > 0.001 || Math.abs(ky) > 0.001) {
      ctx.beginPath(); ctx.moveTo(jCx, jCy); ctx.lineTo(knX, knY);
      ctx.strokeStyle = fill + '22'; ctx.lineWidth = 1.2; ctx.stroke();
    }

    // Knob
    var knobR = jRadius * 0.26;
    var gradK = ctx.createRadialGradient(knX - knobR * 0.3, knY - knobR * 0.3, knobR * 0.1, knX, knY, knobR);
    gradK.addColorStop(0, fill + 'CC');
    gradK.addColorStop(1, fill + '55');

    ctx.beginPath(); ctx.arc(knX, knY, knobR, 0, Math.PI * 2);
    ctx.fillStyle = gradK;
    ctx.shadowColor = fill + (jActive ? '60' : '25');
    ctx.shadowBlur = jActive ? 14 : 6;
    ctx.fill();
    ctx.shadowBlur = 0;

    ctx.strokeStyle = fill + '90'; ctx.lineWidth = 2;
    ctx.stroke();
  }

  function clampJoystick(dx, dy) {
    var dist = Math.sqrt(dx * dx + dy * dy);
    var maxDist = jRadius;
    if (dist > maxDist) {
      dx = dx / dist * maxDist;
      dy = dy / dist * maxDist;
    }
    var normX = dx / jRadius;
    var normY = dy / jRadius;
    var normDist = Math.sqrt(normX * normX + normY * normY);
    if (normDist < DEAD_ZONE) return { kx: 0, ky: 0, v: 0, w: 0 };
    var scale = (normDist - DEAD_ZONE) / (1 - DEAD_ZONE);
    var v = normY * scale * MAX_V;
    var w = -normX * scale * MAX_W;
    return { kx: normX, ky: normY, v: v, w: w };
  }

  function publishJoystick(v, w) {
    if (!pubCmdVel || !pubEnable) return;
    pubCmdVel.publish({
      linear: { x: Number(v.toFixed(2)), y: 0, z: 0 },
      angular: { x: 0, y: 0, z: Number(w.toFixed(2)) }
    });
    pubEnable.publish({ data: true });
  }

  function getEventPos(e) {
    var rect = jCanvas.getBoundingClientRect();
    var clientX = e.touches ? e.touches[0].clientX : e.clientX;
    var clientY = e.touches ? e.touches[0].clientY : e.clientY;
    return { x: clientX - rect.left, y: clientY - rect.top };
  }

  function onJoystickStart(e) {
    e.preventDefault();
    jActive = true;
    var pos = getEventPos(e);
    handleJoystickMove(pos.x, pos.y);
  }

  function onJoystickMove(e) {
    if (!jActive) return;
    e.preventDefault();
    var pos = getEventPos(e);
    handleJoystickMove(pos.x, pos.y);
  }

  function onJoystickEnd(e) {
    if (!jActive) return;
    jActive = false;
    drawJoystick(0, 0);
    joyV.textContent = '0.00 m/s';
    joyW.textContent = '0.00 rad/s';
    joyArrowV.textContent = '\u2014';
    joyArrowW.textContent = '\u2014';
    if (pubCmdVel) pubCmdVel.publish({
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 }
    });
  }

  function handleJoystickMove(px, py) {
    var dx = px - jCx;
    var dy = -(py - jCy);
    var result = clampJoystick(dx, dy);
    drawJoystick(result.kx, result.ky);
    joyV.textContent = result.v.toFixed(2) + ' m/s';
    joyW.textContent = result.w.toFixed(2) + ' rad/s';
    joyArrowV.textContent = result.v > 0.01 ? '\u2191' : result.v < -0.01 ? '\u2193' : '\u2014';
    joyArrowW.textContent = result.w > 0.01 ? '\u21BA' : result.w < -0.01 ? '\u21BB' : '\u2014';
    publishJoystick(result.v, result.w);
  }

  jCanvas.addEventListener('mousedown', onJoystickStart);
  window.addEventListener('mousemove', onJoystickMove);
  window.addEventListener('mouseup', onJoystickEnd);
  jCanvas.addEventListener('touchstart', onJoystickStart, { passive: false });
  window.addEventListener('touchmove', onJoystickMove, { passive: false });
  window.addEventListener('touchend', onJoystickEnd);

  resizeJoystick();
  window.addEventListener('resize', resizeJoystick);

  /* ═══════════════════════════════════════════════════
   *  键盘 WASD 控制 (加速度模拟)
   *  W=前进 S=后退 A=左转 D=右转  P=急停
   *  支持加减速渐变 ramping
   ═══════════════════════════════════════════════════ */
  var keyboardEnabled = false;
  var keysHeld = { w: false, a: false, s: false, d: false };
  var kbdInterval = null;
  var chkKeyboard = $('chkKeyboard');

  var currentV = 0;
  var currentW = 0;
  var kbdSpeedFactor = 1.0;
  var sliderKbdSpeed = $('sliderKbdSpeed');
  var txtKbdSpeed = $('txtKbdSpeed');
  var kbdRampRateV = 1.0;
  var kbdRampRateW = 1.0;
  var sliderKbdRampV = $('sliderKbdRampV');
  var txtKbdRampV = $('txtKbdRampV');
  var sliderKbdRampW = $('sliderKbdRampW');
  var txtKbdRampW = $('txtKbdRampW');

  sliderKbdSpeed.addEventListener('input', function () {
    kbdSpeedFactor = parseInt(this.value) / 100;
    txtKbdSpeed.textContent = Math.round(kbdSpeedFactor * 100) + '%';
  });

  sliderKbdRampV.addEventListener('input', function () {
    kbdRampRateV = parseFloat(this.value);
    txtKbdRampV.textContent = kbdRampRateV >= 5.0 ? 'MAX' : kbdRampRateV.toFixed(1) + ' m/s\u00B2';
  });

  sliderKbdRampW.addEventListener('input', function () {
    kbdRampRateW = parseFloat(this.value);
    txtKbdRampW.textContent = kbdRampRateW >= 5.0 ? 'MAX' : kbdRampRateW.toFixed(1) + ' rad/s\u00B2';
  });

  function updateKeyboardCmd() {
    var targetV = ((keysHeld.w ? MAX_V : 0) - (keysHeld.s ? MAX_V : 0)) * kbdSpeedFactor;
    var targetW = ((keysHeld.a ? MAX_W : 0) - (keysHeld.d ? MAX_W : 0)) * kbdSpeedFactor;

    var dt = 0.05;
    var stepV = kbdRampRateV * dt;
    var stepW = kbdRampRateW * dt;

    if (targetV === 0) currentV = 0;
    else if (kbdRampRateV >= 5.0) currentV = targetV;
    else if (currentV < targetV) currentV = Math.min(currentV + stepV, targetV);
    else if (currentV > targetV) currentV = Math.max(currentV - stepV, targetV);

    if (targetW === 0) currentW = 0;
    else if (kbdRampRateW >= 5.0) currentW = targetW;
    else if (currentW < targetW) currentW = Math.min(currentW + stepW, targetW);
    else if (currentW > targetW) currentW = Math.max(currentW - stepW, targetW);

    var v = currentV, w = currentW;

    var kx = (MAX_W !== 0) ? -w / MAX_W : 0;
    var ky = (MAX_V !== 0) ?  v / MAX_V : 0;
    drawJoystick(kx, ky);

    if (pubCmdVel && pubEnable) {
      pubCmdVel.publish({
        linear: { x: Number(v.toFixed(2)), y: 0, z: 0 },
        angular: { x: 0, y: 0, z: Number(w.toFixed(2)) }
      });
      if (v !== 0 || w !== 0) pubEnable.publish({ data: true });
    }
    joyV.textContent = v.toFixed(2) + ' m/s';
    joyW.textContent = w.toFixed(2) + ' rad/s';
    joyArrowV.textContent = v > 0.01 ? '\u2191' : v < -0.01 ? '\u2193' : '\u2014';
    joyArrowW.textContent = w > 0.01 ? '\u21BA' : w < -0.01 ? '\u21BB' : '\u2014';

    if (!keysHeld.w && !keysHeld.a && !keysHeld.s && !keysHeld.d &&
        Math.abs(currentV) < 0.001 && Math.abs(currentW) < 0.001) {
      currentV = 0;
      currentW = 0;
      clearInterval(kbdInterval);
      kbdInterval = null;
    }
  }

  chkKeyboard.addEventListener('change', function () {
    keyboardEnabled = this.checked;
    if (!keyboardEnabled) {
      keysHeld.w = keysHeld.a = keysHeld.s = keysHeld.d = false;
      currentV = 0;
      currentW = 0;
      if (kbdInterval) { clearInterval(kbdInterval); kbdInterval = null; }
      drawJoystick(0, 0);
      if (pubCmdVel) pubCmdVel.publish({
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 }
      });
      joyV.textContent = '0.00 m/s';
      joyW.textContent = '0.00 rad/s';
    }
  });

  // ESC → cancel active tool
  window.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') {
      if (activeTool === 'initpose') {
        activeTool = null;
        initPoseX = 0; initPoseY = 0; initPosePrevX = 0; initPosePrevY = 0;
        updateBtnTitle($('btnInitPose'), '设初始位姿');
        resetToolBtns();
        overlayDirty = true;
      } else if (activeTool === 'goal') {
        activeTool = null;
        goalPoseX = 0; goalPoseY = 0; goalPosePrevX = 0; goalPosePrevY = 0;
        updateBtnTitle($('btnNavGoal'), '导航目标');
        resetToolBtns();
        overlayDirty = true;
      } else if (activeTool === 'follow') {
        activeTool = null;
        followTargetX = 0; followTargetY = 0; followRectW = 0; followRectH = 0;
        followRectStep = 0;
        resetToolBtns();
        overlayDirty = true;
      } else if (activeTool === 'region') {
        activeTool = null;
        regionToolStep = 0;
        regionAnchorX = 0; regionAnchorY = 0; regionMouseX = 0; regionMouseY = 0;
        resetToolBtns();
        overlayDirty = true;
      }
    }
  });

  window.addEventListener('keydown', function (e) {
    if (!keyboardEnabled) return;
    var k = e.key.toLowerCase();
    if (k === 'w' || k === 'a' || k === 's' || k === 'd') {
      e.preventDefault();
      keysHeld[k] = true;
      updateKeyboardCmd();
      if (!kbdInterval) kbdInterval = setInterval(updateKeyboardCmd, 50);
    } else if (k === 'p') {
      e.preventDefault();
      keysHeld.w = keysHeld.a = keysHeld.s = keysHeld.d = false;
      currentV = 0;
      currentW = 0;
      if (kbdInterval) { clearInterval(kbdInterval); kbdInterval = null; }
      pubEstop.publish({ data: true });
      drawJoystick(0, 0);
      joyV.textContent = '0.00 m/s';
      joyW.textContent = '0.00 rad/s';
      isEnabled = false;
      updateEnableBadge();
    }
  });

  window.addEventListener('keyup', function (e) {
    if (!keyboardEnabled) return;
    var k = e.key.toLowerCase();
    if (k === 'w' || k === 'a' || k === 's' || k === 'd') {
      e.preventDefault();
      keysHeld[k] = false;
      updateKeyboardCmd();
    }
  });

  /* ═══════════════════════════════════════════════════
   *  指令帧实时监控面板
   *  订阅 /cmd_frame_debug, 展示当前指令值和历史记录
   ═══════════════════════════════════════════════════ */
  const CMD_FLAG_BITS = [
    { bit: 0x01, label: 'ENABLE',  cls: '' },
    { bit: 0x02, label: 'JUMP',    cls: '' },
    { bit: 0x04, label: 'ESTOP',   cls: 'flag-estop' },
    { bit: 0x08, label: 'RECOVER', cls: 'flag-recover' },
  ];

  const cmdBuffer = [];
  const MAX_CMD_BUFFER = 100;
  let cmdSeq = 0;
  const cdbTs = $('cdbTs'), cdbV = $('cdbV'), cdbYaw = $('cdbYaw');
  const cdbRoll = $('cdbRoll'), cdbLeg = $('cdbLeg'), cdbPitch = $('cdbPitch');
  const cdbFlags = $('cdbFlags');
  const cmdDebugList = $('cmdDebugList');
  const cmdDebugWrap = document.querySelector('.cmd-debug-list-wrap');

  function parseFlags(val) {
    const i = Math.round(val);
    const names = [];
    for (const fb of CMD_FLAG_BITS) {
      if (i & fb.bit) names.push({ label: fb.label, cls: fb.cls });
    }
    return names;
  }

  function flagsToHTML(flagsArr) {
    if (!flagsArr.length) return '<span class="cmd-debug-row-f">-</span>';
    return flagsArr.map(function (f) {
      return '<span class="' + (f.cls || '') + '">' + f.label + '</span>';
    }).join(' ');
  }

  function updateCmdDebugCurrent(frame) {
    cdbTs.textContent    = frame[0].toFixed(2) + 's';
    cdbV.textContent     = frame[1].toFixed(2) + ' m/s';
    cdbYaw.textContent   = frame[2].toFixed(2) + ' rad/s';
    cdbRoll.textContent  = frame[3].toFixed(2) + ' rad';
    cdbLeg.textContent   = frame[4].toFixed(3) + ' m';
    cdbPitch.textContent = frame[5].toFixed(2) + ' rad';

    const flagsArr = parseFlags(frame[6]);
    if (flagsArr.length) {
      cdbFlags.innerHTML = 'flags: ' + flagsToHTML(flagsArr);
      cdbFlags.className = 'cmd-debug-chip cmd-debug-flags';
    } else {
      cdbFlags.textContent = 'flags: 0x00';
      cdbFlags.className = 'cmd-debug-chip cmd-debug-flags';
    }
  }

  function appendCmdDebugRow(frame) {
    cmdSeq++;
    cmdBuffer.push({ seq: cmdSeq, data: frame });
    if (cmdBuffer.length > MAX_CMD_BUFFER) cmdBuffer.shift();

    var html = '';
    for (var i = 0; i < cmdBuffer.length; i++) {
      var entry = cmdBuffer[i];
      var d = entry.data;
      var f = parseFlags(d[6]);
      var flagsStr = f.map(function (ff) { return ff.label; }).join(' ') || '-';
      html += '<div class="cmd-debug-row">' +
        '<span class="seq">#' + entry.seq + '</span>' +
        '<span class="ts">' + d[0].toFixed(2) + 's</span>' +
        '<span class="val">v=' + d[1].toFixed(2) + '</span>' +
        '<span class="val">w=' + d[2].toFixed(2) + '</span>' +
        '<span class="val">r=' + d[3].toFixed(2) + '</span>' +
        '<span class="val">l=' + d[4].toFixed(3) + '</span>' +
        '<span class="val">p=' + d[5].toFixed(2) + '</span>' +
        '<span class="flags">' + flagsStr + '</span>' +
      '</div>';
    }
    cmdDebugList.innerHTML = html;
    if (cmdDebugWrap) cmdDebugWrap.scrollTop = cmdDebugWrap.scrollHeight;
  }

  var cmdDebugSubInited = false;
  function initCmdDebugSub() {
    if (cmdDebugSubInited) return;
    cmdDebugSubInited = true;
    new ROSLIB.Topic({ ros: ros, name: '/cmd_frame_debug', messageType: 'std_msgs/Float32MultiArray' })
      .subscribe(function (msg) {
        if (!msg.data || msg.data.length < 7) return;
        var frame = msg.data;  // [timestamp, v_set, yaw_rate, roll, leg, pitch, flags]
        updateCmdDebugCurrent(frame);
        appendCmdDebugRow(frame);
      });
  }
  initCmdDebugSub();

  window.ros = ros;  // 暴露给 chat.js

  /* ═══════════════════════════════════════════════════
   *  摄像头预览流 (MJPEG HTTP)
   *  定期轮询 status API 检测摄像头在线状态
   ═══════════════════════════════════════════════════ */
  var CAM_HOST = window.location.hostname;
  var CAM_PORT = 8193;
  var CAM_STREAM_URL = 'http://' + CAM_HOST + ':' + CAM_PORT + '/stream';
  var CAM_STATUS_URL = 'http://' + CAM_HOST + ':' + CAM_PORT + '/status';

  var camStream = $('cameraStream');
  var camOfflineOverlay = $('camOfflineOverlay');
  var camOfflineText = $('camOfflineText');
  var camStatus = $('camStatus');
  var btnCamRetry = $('btnCamRetry');

  var camPollTimer = null;
  var camServerOnline = false;
  var camStreamActive = false;
  var camRetryPending = false;

  function camStreamURL() {
    return CAM_STREAM_URL + '?_t=' + Date.now();
  }

  function camSetStatus(text, cls) {
    camStatus.innerHTML = text;
    camStatus.className = 'camera-status' + (cls ? ' ' + cls : '');
  }

  function camShowOverlay(show) {
    if (show) {
      camOfflineOverlay.style.display = 'flex';
      camOfflineOverlay.style.opacity = '1';
    } else {
      camOfflineOverlay.style.opacity = '0';
      setTimeout(function () {
        if (camOfflineOverlay.style.opacity === '0') {
          camOfflineOverlay.style.display = 'none';
        }
      }, 300);
    }
  }

  function camShowRetry(show) {
    btnCamRetry.style.display = show ? 'inline-flex' : 'none';
  }

  function camTryConnect() {
    if (camRetryPending) return;
    camRetryPending = true;
    camStreamActive = false;
    camShowRetry(false);
    camOfflineText.textContent = '正在连接...';
    camStream.src = camStreamURL();
  }

  function camPoll() {
    fetch(CAM_STATUS_URL, { cache: 'no-store' })
      .then(function (r) { return r.json(); })
      .then(function (status) {
        var wasOffline = !camServerOnline;
        camServerOnline = (status.status === 'ok');

        if (camServerOnline) {
          camSetStatus('&#9679; 在线 (' + status.resolution + ')', 'ok');
          if (wasOffline && !camStreamActive) {
            camTryConnect();
          }
        } else {
          camSetStatus('离线', 'error');
          camShowOverlay(true);
          camShowRetry(true);
          camOfflineText.textContent = '摄像头未连接';
        }
      })
      .catch(function () {
        camServerOnline = false;
        camSetStatus('离线', 'error');
        camShowOverlay(true);
        camShowRetry(true);
        camOfflineText.textContent = '摄像头未连接';
      });

    camRetryPending = false;
  }

  camStream.addEventListener('load', function () {
    camStreamActive = true;
    camRetryPending = false;
    camShowOverlay(false);
    camStream.style.display = 'block';
    camShowRetry(false);
  });

  camStream.addEventListener('error', function () {
    camStreamActive = false;
    camRetryPending = false;
    camStream.style.display = 'none';
    camShowOverlay(true);
    camSetStatus('离线', 'error');
    camShowRetry(true);
    camOfflineText.textContent = '摄像头未连接';
  });

  btnCamRetry.addEventListener('click', function () {
    camStream.style.display = 'none';
    camShowOverlay(true);
    camTryConnect();
  });

  camStream.src = CAM_STREAM_URL;
  camPollTimer = setInterval(camPoll, 5000);

})();
