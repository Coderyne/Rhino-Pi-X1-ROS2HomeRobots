/**
 * 轮足机器人 — 仪表盘核心逻辑
 * roslibjs 实现实时数据可视化、地图渲染、建图导航控制
 */
(function () {
  'use strict';

  const $ = (id) => document.getElementById(id);
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

  /* ── Publishers ────────────────────────────────── */
  let pubCmdVel, pubCmdAtt, pubEnable, pubEstop, pubJump, pubRecover;
  let pubGoalPose, pubInitPose, pubKeepAlive;

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
    // 同步 keep-alive 初始状态到 bridge
    pubKeepAlive.publish({ data: $('chkKeepAlive').checked });
  }

  /* ── DOM refs ──────────────────────────────────── */
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
  let mapInfo = null;          // { width, height, resolution, origin: {x,y} }
  let mapData = null;          // Int8Array of OccupancyGrid data
  let mapImageData = null;     // cached ImageData
  let mapDirty = true;         // re-render needed
  let overlayDirty = false;    // overlay repaint needed
  let scanPoints = [];         // [{x, y}] in map frame
  let pathPoints = [];         // [{x, y}] in map frame
  let robotX = 0, robotY = 0, robotYaw = 0;  // in map frame
  let robotVisible = false;

  // Tool state
  let activeTool = null;       // 'goal' | 'initpose' | null
  let initPoseX = 0, initPoseY = 0;

  // ── Coordinate helpers ────────────────────────
  function worldToPixel(wx, wy) {
    if (!mapInfo) return { px: wx, py: wy };
    const px = (wx - mapInfo.origin.x) / mapInfo.resolution;
    const py = mapCanvas.height - (wy - mapInfo.origin.y) / mapInfo.resolution;
    return { px: px, py: py };
  }

  function pixelToWorld(px, py) {
    if (!mapInfo) return { wx: px, wy: py };
    const wx = px * mapInfo.resolution + mapInfo.origin.x;
    const wy = (mapCanvas.height - py) * mapInfo.resolution + mapInfo.origin.y;
    return { wx: wx, wy: wy };
  }

  // ── Resize canvas to match container ────────────
  function resizeCanvas() {
    const rect = mapCanvas.parentElement.getBoundingClientRect();
    const w = rect.width;
    const h = 420;
    if (mapCanvas.width !== w || mapCanvas.height !== h) {
      mapCanvas.width = w;
      mapCanvas.height = h;
      mapDirty = true;
    }
  }

  // ── Render map background ──────────────────────
  function renderMap() {
    resizeCanvas();
    mapCtx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
    if (!mapData || !mapInfo) return;

    if (mapDirty || !mapImageData) {
      const w = mapInfo.width;
      const h = mapInfo.height;
      mapImageData = mapCtx.createImageData(w, h);

      for (let i = 0; i < mapData.length; i++) {
        const idx = i * 4;
        const v = mapData[i];
        if (v < 0) {
          mapImageData.data[idx] = 68; mapImageData.data[idx + 1] = 71;
          mapImageData.data[idx + 2] = 73; mapImageData.data[idx + 3] = 255;
        } else if (v === 0) {
          mapImageData.data[idx] = 226; mapImageData.data[idx + 1] = 226;
          mapImageData.data[idx + 2] = 233; mapImageData.data[idx + 3] = 255;
        } else {
          const c = Math.floor(26 + (100 - v) * 2.29);
          mapImageData.data[idx] = c; mapImageData.data[idx + 1] = c;
          mapImageData.data[idx + 2] = c; mapImageData.data[idx + 3] = 255;
        }
      }
      mapDirty = false;
    }

    // Build offscreen canvas with raw map image
    const off = document.createElement('canvas');
    off.width = mapInfo.width;
    off.height = mapInfo.height;
    const octx = off.getContext('2d');
    octx.putImageData(mapImageData, 0, 0);

    // Compute uniform scale/offset shared with drawOverlays
    const mapW = mapInfo.width * mapInfo.resolution;
    const mapH = mapInfo.height * mapInfo.resolution;
    const scale = Math.min(mapCanvas.width / mapW, mapCanvas.height / mapH);
    const ox = mapCanvas.width / 2 - mapW * scale / 2;
    const oy = mapCanvas.height / 2 + mapH * scale / 2;

    // Draw map: origin at (0,0) in world → ox/oy in canvas
    const mapOriginX = ox - mapInfo.origin.x * scale;
    const mapOriginY = oy - mapInfo.origin.y * scale;

    mapCtx.save();
    mapCtx.setTransform(scale, 0, 0, -scale, mapOriginX, mapOriginY);
    mapCtx.drawImage(off, 0, 0);
    mapCtx.restore();
  }

  function drawOverlays() {
    if (!mapInfo) return;
    const scale = Math.min(
      mapCanvas.width / (mapInfo.width * mapInfo.resolution),
      mapCanvas.height / (mapInfo.height * mapInfo.resolution));
    const w = mapInfo.width * mapInfo.resolution;
    const h = mapInfo.height * mapInfo.resolution;
    const ox = mapCanvas.width / 2 - w * scale / 2;
    const oy = mapCanvas.height / 2 + h * scale / 2;

    function toCanvas(wx, wy) {
      return { x: ox + wx * scale, y: oy - wy * scale };
    }

    // ── Path (blue line) ─────────────────────
    if (pathPoints.length > 1) {
      mapCtx.beginPath();
      mapCtx.strokeStyle = 'rgba(88, 166, 255, 0.8)';
      mapCtx.lineWidth = 2;
      const p0 = toCanvas(pathPoints[0].x, pathPoints[0].y);
      mapCtx.moveTo(p0.x, p0.y);
      for (let i = 1; i < pathPoints.length; i++) {
        const p = toCanvas(pathPoints[i].x, pathPoints[i].y);
        mapCtx.lineTo(p.x, p.y);
      }
      mapCtx.stroke();
    }

    // ── Laser Scan (green dots) ──────────────
    if (scanPoints.length > 0) {
      mapCtx.fillStyle = 'rgba(63, 185, 80, 0.4)';
      for (const pt of scanPoints) {
        const p = toCanvas(pt.x, pt.y);
        mapCtx.fillRect(p.x - 1, p.y - 1, 2, 2);
      }
    }

    // ── Robot marker (red arrow) ──────────────
    if (robotVisible) {
      const r = toCanvas(robotX, robotY);
      mapCtx.save();
      mapCtx.translate(r.x, r.y);
      mapCtx.rotate(-robotYaw);
      mapCtx.fillStyle = '#f85149';
      mapCtx.beginPath();
      mapCtx.moveTo(10, 0);
      mapCtx.lineTo(-6, -6);
      mapCtx.lineTo(-4, 0);
      mapCtx.lineTo(-6, 6);
      mapCtx.closePath();
      mapCtx.fill();
      mapCtx.strokeStyle = 'rgba(255,255,255,0.6)';
      mapCtx.lineWidth = 1;
      mapCtx.stroke();
      mapCtx.restore();
    }
  }

  function repaint() {
    renderMap();
    drawOverlays();
  }

  // ── Subscribers ──────────────────────────────

  function initSubscribers() {
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
        mapMode.textContent = 'SLAM 建图中';
        mapMode.className = 'map-mode-badge active';
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

    // /tf — 30Hz
    new ROSLIB.Topic({ ros: ros, name: '/tf', messageType: 'tf2_msgs/TFMessage', throttle_rate: 33, queue_length: 1 })
      .subscribe((msg) => {
        for (const t of msg.transforms) {
          if (t.header.frame_id === 'map' && t.child_frame_id === 'base_footprint') {
            robotX = t.transform.translation.x;
            robotY = t.transform.translation.y;
            const q = t.transform.rotation;
            robotYaw = Math.atan2(2 * (q.w * q.z + q.x * q.y),
                                   1 - 2 * (q.y * q.y + q.z * q.z));
            robotVisible = true;
            overlayDirty = true;
            updateMapModeNav();
            break;
          }
        }
      });
  }

  // 定时重绘地图 (20Hz), 避免高频 subscriber 触发过多 repaint
  setInterval(() => {
    if (overlayDirty || mapDirty) {
      overlayDirty = false;
      repaint();
    }
  }, 50);

  function updateMapModeNav() {
    mapMode.textContent = '导航中';
    mapMode.className = 'map-mode-badge active';
  }

  /* ── Canvas click → nav tools ────────────────── */
  mapCanvas.addEventListener('click', (e) => {
    if (!activeTool || !mapInfo) return;
    const rect = mapCanvas.getBoundingClientRect();
    const px = e.clientX - rect.left;
    const py = e.clientY - rect.top;
    const world = pixelToWorld(px, py);

    if (activeTool === 'goal') {
      // Publish goal_pose (default orientation)
      pubGoalPose.publish({
        header: { frame_id: 'map', stamp: Date.now() / 1000 },
        pose: {
          position: { x: world.wx, y: world.wy, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 }
        }
      });
      activeTool = null;
      resetToolBtns();
    } else if (activeTool === 'initpose') {
      // First click sets position, second click sets direction
      if (!initPoseX && !initPoseY) {
        initPoseX = world.wx;
        initPoseY = world.wy;
        $('btnInitPose').textContent = '点击选择朝向';
      } else {
        const yaw = Math.atan2(world.wy - initPoseY, world.wx - initPoseX);
        const cy = Math.cos(yaw * 0.5), sy = Math.sin(yaw * 0.5);
        pubInitPose.publish({
          header: { frame_id: 'map', stamp: Date.now() / 1000 },
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
        initPoseY = 0;
        $('btnInitPose').querySelector('span:last-child').textContent = '设初始位姿';
        resetToolBtns();
      }
    }
  });

  function resetToolBtns() {
    ['btnNavGoal', 'btnInitPose'].forEach(id => {
      $(id).classList.remove('active');
    });
  }

  /* ── Toolbar buttons ──────────────────────────── */
  $('btnNavGoal').addEventListener('click', () => {
    activeTool = activeTool === 'goal' ? null : 'goal';
    initPoseX = 0; initPoseY = 0;
    $('btnNavGoal').classList.toggle('active', activeTool === 'goal');
    $('btnInitPose').classList.remove('active');
    mapCanvas.style.cursor = activeTool === 'goal' ? 'crosshair' : 'crosshair';
  });

  $('btnInitPose').addEventListener('click', () => {
    activeTool = activeTool === 'initpose' ? null : 'initpose';
    initPoseX = 0; initPoseY = 0;
    $('btnInitPose').classList.toggle('active', activeTool === 'initpose');
    $('btnNavGoal').classList.remove('active');
    mapCanvas.style.cursor = activeTool === 'initpose' ? 'crosshair' : 'crosshair';
  });

  $('btnCancelNav').addEventListener('click', () => {
    if (pubCmdVel) pubCmdVel.publish({ linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } });
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

  /* ── Button Events ─────────────────────────────── */
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

  /* ── Keep-alive toggle ────────────────────────── */
  $('chkKeepAlive').addEventListener('change', (e) => {
    pubKeepAlive.publish({ data: e.target.checked });
  });

  /* ── Sliders ────────────────────────────────────── */
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

  /* ── 虚拟摇杆 (Canvas) ─────────────────────── */
  var jCanvas = $('joystickCanvas');
  var jCtx = jCanvas.getContext('2d');
  var joyV = $('joyV');
  var joyW = $('joyW');

  var MAX_V = 1.5;
  var MAX_W = 3.14;
  var DEAD_ZONE = 0.15;

  var jRadius = 0, jCx = 0, jCy = 0;
  var jActive = false;

  function resizeJoystick() {
    var wrap = jCanvas.parentElement;
    if (!wrap) return;
    var rect = wrap.getBoundingClientRect();
    var size = Math.min(rect.width - 16, rect.height - 16, 280);
    if (size <= 0) return;
    jCanvas.width = size;
    jCanvas.height = size;
    jCx = size / 2;
    jCy = size / 2;
    jRadius = size * 0.38;
    drawJoystick(0, 0);
  }

  function drawJoystick(kx, ky) {
    var ctx = jCtx;
    var w = jCanvas.width, h = jCanvas.height;
    ctx.clearRect(0, 0, w, h);

    var style = getComputedStyle(document.documentElement);
    var bg   = style.getPropertyValue('--md-sys-color-surface-container-highest').trim() || '#1e2028';
    var ring = style.getPropertyValue('--md-sys-color-outline-variant').trim() || '#3a3d48';
    var fill = style.getPropertyValue('--md-sys-color-primary').trim() || '#b0c6ff';

    ctx.beginPath();
    ctx.arc(jCx, jCy, jRadius, 0, Math.PI * 2);
    ctx.fillStyle = jActive ? ring + '40' : bg;
    ctx.strokeStyle = ring;
    ctx.lineWidth = 2;
    ctx.fill();
    ctx.stroke();

    ctx.strokeStyle = ring + '60';
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 6]);
    ctx.beginPath();
    ctx.moveTo(jCx - jRadius * 0.7, jCy);
    ctx.lineTo(jCx + jRadius * 0.7, jCy);
    ctx.moveTo(jCx, jCy - jRadius * 0.7);
    ctx.lineTo(jCx, jCy + jRadius * 0.7);
    ctx.stroke();
    ctx.setLineDash([]);

    var kr = jRadius * 0.28;
    ctx.beginPath();
    ctx.arc(jCx + kx * jRadius, jCy - ky * jRadius, kr, 0, Math.PI * 2);
    ctx.fillStyle = fill;
    ctx.shadowColor = fill + '60';
    ctx.shadowBlur = 8;
    ctx.fill();
    ctx.shadowBlur = 0;
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

  /* ── 键盘 WASD 控制 ────────────────────────── */
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

  /* ── 指令帧实时监控 ─────────────────────────── */
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

})();
