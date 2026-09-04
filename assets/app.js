'use strict';

const socket = io(`http://${window.location.host}`);

const ROVER_MODES = {
  0: 'MANUAL', 1: 'ACRO', 3: 'STEERING', 4: 'HOLD',
  5: 'LOITER', 6: 'FOLLOW', 7: 'SIMPLE', 10: 'AUTO',
  11: 'RTL', 12: 'SMART_RTL', 15: 'GUIDED', 16: 'INITIALIZING'
};

const GPS_FIX = ['無定位', '無定位', '2D 定位', '3D 定位', '3D DGPS', 'RTK 浮動', 'RTK 固定'];

// Only two modes are selectable, and only from the controller's D21 switch.
// Without a chart there is no way to place waypoints, so AUTO/GUIDED/RTL have
// no operator interface here — they are decoded for display only.
const MODE_MANUAL = 0;
const MODE_ACRO = 1;

const ctrl = { steering: 0, left_thr: 1500, right_thr: 1500 };
let webCtrlEnabled = false;
let lastCtrlSend = 0;
let _connFieldsDirty = false;   // true while user has unsaved edits in connection fields
let usvConnected = false;
let currentConn = { protocol: 'udp', remote_ip: '', remote_port: '', local_ip: '', local_port: '' };

function el(id) { return document.getElementById(id); }
function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }
function rgbStr(led) { return `rgb(${led.r},${led.g},${led.b})`; }
function ledGlow(led) {
  const max = Math.max(led.r, led.g, led.b);
  if (max < 20) return '';
  return `0 0 10px rgb(${led.r},${led.g},${led.b}), 0 0 20px rgba(${led.r},${led.g},${led.b},0.35)`;
}

function headingDir(deg) {
  if (deg >= 337.5 || deg < 22.5) return '正北';
  if (deg < 67.5) return '東北';
  if (deg < 112.5) return '正東';
  if (deg < 157.5) return '東南';
  if (deg < 202.5) return '正南';
  if (deg < 247.5) return '西南';
  if (deg < 292.5) return '正西';
  return '西北';
}

function setDotState(node, online) {
  node.className = `status-dot ${online ? 'online' : 'offline'}`;
}

function formatPctFromUs(val) {
  return Math.round((val - 1500) / 5);
}

function formatSignedPct(pct) {
  if (pct === 0) return '0%';
  return `${pct > 0 ? '+' : ''}${pct}%`;
}

function formatThrottleValue(val) {
  return `${val} us (${formatPctFromUs(val)}%)`;
}

function formatSteerValue(val) {
  const pct = formatPctFromUs(val);
  return pct === 0 ? `${val} us (居中)` : `${val} us (${formatSignedPct(pct)})`;
}

function steeringUsToCtrl(val) {
  return (val - 1500) * 2;
}

function steeringCtrlToUs(val) {
  return clamp(Math.round(1500 + val / 2), 1000, 2000);
}

function refreshFooterStatus() {
  const remote = currentConn.remote_ip || '—';
  const remotePort = currentConn.remote_port || '—';
  const local = currentConn.local_ip ? `${currentConn.local_ip}:${currentConn.local_port || '—'}` : (currentConn.local_port || '—');
  el('status-text').textContent = usvConnected
    ? `網絡連線已建立。發送目標: ${remote}:${remotePort} | 本機監聽: ${local}`
    : `USV 尚未連線。發送目標: ${remote}:${remotePort} | 本機監聽: ${local}`;
}

function sendControl(force = false) {
  if (!webCtrlEnabled) return;
  const now = Date.now();
  if (!force && now - lastCtrlSend < 100) return;
  socket.emit('control', { ...ctrl });
  lastCtrlSend = now;
}

function setThrottleControl(sliderId, valueId, ctrlKey, value) {
  const slider = el(sliderId);
  slider.value = value;
  ctrl[ctrlKey] = value;
  el(valueId).textContent = formatThrottleValue(value);
}

function resetThrottleControls() {
  setThrottleControl('left-thr-ctrl', 'left-thr-ctrl-val', 'left_thr', 1500);
  setThrottleControl('right-thr-ctrl', 'right-thr-ctrl-val', 'right_thr', 1500);
}

function setSteeringControl(usValue, emit = false) {
  const sliderVal = clamp(usValue, 1000, 2000);
  el('steer-slider').value = sliderVal;
  ctrl.steering = steeringUsToCtrl(sliderVal);
  el('steer-val-text').textContent = formatSteerValue(sliderVal);
  if (emit) sendControl(true);
}

socket.on('connect', () => {
  setDotState(el('ws-dot'), true);
  el('ws-label').textContent = '已連線';
  if (!usvConnected) el('status-text').textContent = 'WebSocket 已連線，等待 USV 連線。';
});

socket.on('disconnect', () => {
  usvConnected = false;
  setDotState(el('ws-dot'), false);
  setDotState(el('usv-dot'), false);
  el('ws-label').textContent = '已斷線';
  el('usv-status-text').textContent = '未連線';
  el('conn-badge').className = 'conn-badge off';
  el('conn-badge').textContent = '未連線';
  el('comm-stats').textContent = '接收: 0 | 發送: 0 | 延遲: -- ms';
  el('status-text').textContent = 'WebSocket 已中斷，等待重連。';
});

socket.on('state', (d) => {
  if (d.connection) updateConnectionUI(d.connection);
  if (d.inputs) updateInputs(d.inputs);
  if (d.comms) updateComms(d.comms);
  if (d.telemetry) updateTelemetry(d.telemetry);
  // Needs both blocks: the vessel's mode lives in telemetry, the switch
  // position in comms.
  if (d.telemetry && d.comms) {
    updateModeReadout(d.telemetry.mode, Boolean(d.comms.mode_sw_acro),
                      Boolean(d.comms.mode_denied), Boolean(d.comms.resume_interlock));
  }
  if (d.leds) updateLeds(d.leds);
});

['conn-proto', 'conn-ip', 'conn-port', 'conn-local-port'].forEach((id) => {
  el(id).addEventListener('input',  () => { _connFieldsDirty = true; });
  el(id).addEventListener('change', () => { _connFieldsDirty = true; });
});

el('conn-btn').addEventListener('click', () => {
  const protocol = el('conn-proto').value;
  const remote_ip = el('conn-ip').value.trim();
  const remote_port = parseInt(el('conn-port').value, 10);
  const local_port = parseInt(el('conn-local-port').value, 10) || 0;

  let valid = true;
  if (!remote_ip || !remote_port || remote_port < 1 || remote_port > 65535) {
    el('conn-ip').style.borderColor = 'var(--red)';
    el('conn-port').style.borderColor = 'var(--red)';
    setTimeout(() => {
      el('conn-ip').style.borderColor = '';
      el('conn-port').style.borderColor = '';
    }, 2000);
    valid = false;
  }

  if (protocol === 'udp' && (!local_port || local_port < 1 || local_port > 65535)) {
    el('conn-local-port').style.borderColor = 'var(--red)';
    setTimeout(() => { el('conn-local-port').style.borderColor = ''; }, 2000);
    valid = false;
  }

  if (!valid) return;

  const btn = el('conn-btn');
  btn.textContent = '🔄 連線中…';
  btn.classList.add('busy');
  setTimeout(() => {
    btn.classList.remove('busy');
    btn.textContent = usvConnected ? '🔴 關閉' : '🔌 連線';
  }, 4000);

  socket.emit('connect_usv', { protocol, remote_ip, remote_port, local_port });
  _connFieldsDirty = false;   // allow server state to sync fields again after submit
});

function updateConnectionUI({ protocol, remote_ip, remote_port, local_ip, local_port, connected }) {
  currentConn = { protocol, remote_ip, remote_port, local_ip, local_port };
  usvConnected = Boolean(connected);

  if (!_connFieldsDirty) {
    el('conn-proto').value = protocol;
    el('conn-ip').value = remote_ip;
    el('conn-port').value = remote_port;
    el('conn-local-port').value = local_port || '';
  }

  el('diag-local-port').textContent = local_port ? `${local_ip || '0.0.0.0'}:${local_port}` : 'port: —';
  el('diag-remote-port').textContent = remote_ip ? `port: ${remote_port}` : 'port: —';
  el('conn-str-label').textContent = local_port
    ? `bind:${local_ip || '0.0.0.0'}:${local_port} → ${remote_ip}:${remote_port}`
    : `${protocol}:${remote_ip}:${remote_port}`;

  setDotState(el('usv-dot'), usvConnected);
  el('usv-status-text').textContent = usvConnected ? '已連線' : '未連線';
  el('conn-btn').textContent = usvConnected ? '🔴 關閉' : '🔌 連線';
  refreshFooterStatus();
}

function updateInputs({ steering, left_thr, right_thr, l_eng_on, r_eng_on }) {
  const angleDeg = 270 + (steering / 1000) * 90;
  const angle = angleDeg * Math.PI / 180;
  const radius = 63;
  const x2 = (110 + radius * Math.cos(angle)).toFixed(1);
  const y2 = (125 + radius * Math.sin(angle)).toFixed(1);
  el('steer-needle').setAttribute('x2', x2);
  el('steer-needle').setAttribute('y2', y2);

  if (Math.abs(steering) > 10) {
    const startAngle = 270 * Math.PI / 180;
    const endAngle = angleDeg * Math.PI / 180;
    const sx = (110 + 70 * Math.cos(startAngle)).toFixed(1);
    const sy = (125 + 70 * Math.sin(startAngle)).toFixed(1);
    const ex = (110 + 70 * Math.cos(endAngle)).toFixed(1);
    const ey = (125 + 70 * Math.sin(endAngle)).toFixed(1);
    const sweep = steering > 0 ? 1 : 0;
    el('steer-arc').setAttribute('d', `M ${sx} ${sy} A 70 70 0 0 ${sweep} ${ex} ${ey}`);
  } else {
    el('steer-arc').setAttribute('d', 'M 110 55 A 0 0 0 0 0 110 55');
  }

  // Update SVG rudder line deflection
  const steeringAngleRad = (steering / 1000) * 0.7853; // max 45 degrees
  const rx = (110 + 20 * Math.sin(steeringAngleRad)).toFixed(1);
  const ry = (125 + 20 * Math.cos(steeringAngleRad)).toFixed(1);
  el('web-rudder').setAttribute('x2', rx);
  el('web-rudder').setAttribute('y2', ry);

  const steeringUs = steeringCtrlToUs(steering);
  el('steer-val').textContent = formatSteerValue(steeringUs);

  const leftPct = clamp(((left_thr - 1000) / 1000) * 100, 0, 100);
  const rightPct = clamp(((right_thr - 1000) / 1000) * 100, 0, 100);
  el('left-thr-bar').style.height = `${leftPct}%`;
  el('right-thr-bar').style.height = `${rightPct}%`;
  el('left-thr-val').textContent = `${left_thr} us`;
  el('right-thr-val').textContent = `${right_thr} us`;

  [['engine-l-badge', l_eng_on], ['engine-r-badge', r_eng_on]].forEach(([id, on]) => {
    const badge = el(id);
    badge.textContent = on ? 'ON' : 'OFF';
    badge.className = `engine-badge ${on ? 'on' : 'off'}`;
  });
}

function updateComms({ link_quality, hb_age_s, in_failsafe, tx_count, rx_count,
                       control_authority, source_system,
                       estop_latched, estop_active }) {
  updateEstop(Boolean(estop_latched), Boolean(estop_active));

  const lq = clamp(link_quality || 0, 0, 100);
  el('lq-bar').style.width = `${lq}%`;
  el('lq-val').textContent = lq;

  el('hb-age').textContent = hb_age_s < 0 ? '—' : `${hb_age_s.toFixed(1)} 秒前`;

  const failsafeEl = el('failsafe-val');
  const failsafeBadge = el('failsafe-badge');
  if (in_failsafe) {
    failsafeEl.textContent = '⚠ 啟動';
    failsafeEl.style.color = 'var(--red)';
    failsafeBadge.classList.remove('hidden');
  } else {
    failsafeEl.textContent = '正常';
    failsafeEl.style.color = 'var(--green)';
    failsafeBadge.classList.add('hidden');
  }

  const connected = usvConnected || lq > 0;
  const connBadge = el('conn-badge');
  connBadge.textContent = connected ? '已連線' : '未連線';
  connBadge.className = `conn-badge ${connected ? 'on' : 'off'}`;

  const tx = tx_count ?? 0;
  const rx = rx_count ?? 0;
  el('tx-count').textContent = tx.toLocaleString();
  el('rx-count').textContent = rx.toLocaleString();
  el('comm-stats').textContent = `接收: ${rx} | 發送: ${tx} | 延遲: ${connected ? '1' : '--'} ms`;

  // Control authority status — the E-STOP latch outranks any authority state
  const AUTH = {
    granted: { text: '✓ 已取得控制權',       cls: 'auth-granted' },
    pending: { text: '⏳ 請求中… (等待 ACK)', cls: 'auth-pending' },
    denied:  { text: '✗ 請求被拒絕',          cls: 'auth-denied'  },
    none:    { text: '— 未請求控制權 —',       cls: 'auth-none'    },
  };
  const a = estop_latched
    ? { text: '⛔ 緊急停止中 — 通訊已切斷', cls: 'auth-denied' }
    : (AUTH[control_authority] || AUTH.none);
  const authEl = el('auth-status');
  authEl.textContent = a.text;
  authEl.className = `auth-status ${a.cls}`;

  // Sync GCS SysID field (only when not focused by user)
  if (source_system != null && document.activeElement !== el('gcs-sysid')) {
    el('gcs-sysid').value = source_system;
  }
}

// Show the E-STOP overlay while latched, and report whether the physical
// button has been reset — step 2 of the clear procedure the overlay lists.
function updateEstop(latched, active) {
  el('estop-overlay').classList.toggle('hidden', !latched);
  if (!latched) return;
  const state = el('estop-btn-state');
  state.textContent = active ? '實體按鈕：尚未復位' : '實體按鈕：已復位 — 可長按控制權按鈕解除';
  state.className = `estop-btn-state ${active ? '' : 'ready'}`;
}

function updateArmBtn(armed) {
  const badge = el('arm-badge');
  badge.textContent = armed ? '⚡ ARMED' : '⚡ DISARMED';
  badge.className = `arm-badge ${armed ? 'armed' : 'disarmed'}`;
}

// Read-only mode indicator. `mode` is what the vessel reports; swAcro is where
// the D21 switch sits. When they disagree the requested mode has not taken
// effect — the operator needs to see that before steering on the assumption
// that it did.
function updateModeReadout(mode, swAcro, denied, interlock) {
  const isAcro = mode === MODE_ACRO;
  const isManual = mode === MODE_MANUAL;
  el('mode-ro-manual').className = `mode-slot ${isManual ? 'active' : ''}`;
  el('mode-ro-acro').className = `mode-slot ${isAcro ? 'active' : ''}`;

  // Compare against the requested mode rather than just "is it ACRO", so a
  // vessel that moved to HOLD/RTL by itself is flagged too.
  const wantMode = swAcro ? MODE_ACRO : MODE_MANUAL;
  const note = el('mode-note');
  if (interlock) {
    note.textContent = '⚠ 急停解除中 — 請將油門推桿歸中立、引擎開關關閉後才會恢復控制';
    note.className = 'mode-note warn';
  } else if (denied) {
    note.textContent = '⚠ 定向已拒絕／退回手動 — 定位訊號不足，無可靠航向源';
    note.className = 'mode-note warn';
  } else if (mode !== wantMode) {
    note.textContent = `⚠ 開關要求「${swAcro ? '定向' : '手動'}」，船端目前為 ${ROVER_MODES[mode] || mode}`;
    note.className = 'mode-note warn';
  } else {
    note.className = 'mode-note hidden';
  }

  const headerBadge = el('mode-badge');
  if (mode === undefined || mode === null || mode < 0) {
    headerBadge.textContent = 'LOCKED';
    headerBadge.className = 'mode-badge locked';
  } else if (isManual || isAcro) {
    headerBadge.textContent = isAcro ? 'ACRO 定向' : 'MANUAL 手動';
    headerBadge.className = `mode-badge ${isAcro ? 'acro' : 'manual'}`;
  } else {
    // A mode we never command — the vessel changed it on its own (failsafe,
    // GCS elsewhere). Show it plainly rather than pretending it is one of ours.
    headerBadge.textContent = ROVER_MODES[mode] || `MODE ${mode}`;
    headerBadge.className = 'mode-badge locked';
  }
}

function updateTelemetry({ speed_kn, speed_ms, battery_pct, voltage_v,
  gps_fix, gps_sats, armed, mode,
  roll, pitch, yaw,
  lat_deg7, lon_deg7,
  throttle_pct, servo1_raw, servo3_raw, servo4_raw }) {
  const bPct = clamp(battery_pct || 0, 0, 100);
  el('batt-pct-val').textContent = `${bPct}%`;
  el('batt-volt-val').textContent = `${(voltage_v || 0).toFixed(2)} V`;
  el('batt-bar').style.width = `${bPct}%`;
  el('batt-bar').style.background = bPct > 60 ? 'var(--green)' : bPct > 30 ? 'var(--yellow)' : 'var(--red)';

  el('speed-val').textContent = `${(speed_ms || 0).toFixed(2)} m/s (${(speed_kn || 0).toFixed(2)} 節)`;
  const yawVal = yaw ?? 0;
  const yawHeading = ((yawVal % 360) + 360) % 360;
  el('yaw-val').textContent = `${yawVal.toFixed(1)} ° (${headingDir(yawHeading)})`;
  el('roll-val').textContent = `${(roll || 0).toFixed(1)} °`;
  el('pitch-val').textContent = `${(pitch || 0).toFixed(1)} °`;
  el('hud-throttle').textContent = `${throttle_pct ?? 0} %`;

  const fixEl = el('gps-fix-val');
  fixEl.textContent = GPS_FIX[clamp(gps_fix || 0, 0, GPS_FIX.length - 1)];
  fixEl.className = gps_fix >= 3 ? 'gps-badge fix3d' : gps_fix === 2 ? 'gps-badge fix2d' : 'gps-badge nofix';
  el('gps-sats-val').textContent = `${gps_sats ?? 0} sats`;
  el('gps-lat').textContent = lat_deg7 ? (lat_deg7 / 1e7).toFixed(7) : '—';
  el('gps-lon').textContent = lon_deg7 ? (lon_deg7 / 1e7).toFixed(7) : '—';

  el('servo1-raw').textContent = `${servo1_raw ?? 1500} us`;
  el('servo3-raw').textContent = `${servo3_raw ?? 1500} us`;
  el('servo4-raw').textContent = `${servo4_raw ?? 1500} us`;

  updateArmBtn(Boolean(armed));

  // Toggle vessel hull armed state
  const hull = el('web-vessel-hull');
  if (armed) {
    hull.classList.add('armed');
  } else {
    hull.classList.remove('armed');
  }
}

function updateLeds({ led1, led2, led3, led4 }) {
  [['led1-dot', led1], ['led2-dot', led2], ['led3-dot', led3], ['led4-dot', led4]].forEach(([id, led]) => {
    if (!led) return;
    const node = el(id);
    const color = rgbStr(led);
    node.style.backgroundColor = color;
    node.style.boxShadow = ledGlow(led);
    node.style.borderColor = color;
  });
}

el('web-ctrl-en').addEventListener('change', (e) => {
  webCtrlEnabled = e.target.checked;
  const grid = el('cockpit-grid');
  const status = el('web-ctrl-status');

  if (webCtrlEnabled) {
    grid.classList.remove('disabled');
    status.textContent = '✅ Web 操控中（硬體輸入暫停）';
    status.className = 'toggle-status ctrl-on';
  } else {
    grid.classList.add('disabled');
    status.textContent = '⛔ 停用中（硬體輸入有效）';
    status.className = 'toggle-status ctrl-off';
    ctrl.steering = 0;
    ctrl.left_thr = 1500;
    ctrl.right_thr = 1500;
    setSteeringControl(1500, false);
    resetThrottleControls();
    socket.emit('control', { ...ctrl });
  }

  socket.emit('web_ctrl_set', { enabled: webCtrlEnabled });
});

function wireThrottle(sliderId, valueId, ctrlKey, centerBtnId) {
  const slider = el(sliderId);
  slider.addEventListener('input', () => {
    const value = parseInt(slider.value, 10);
    ctrl[ctrlKey] = value;
    el(valueId).textContent = formatThrottleValue(value);
    sendControl();
  });

  el(centerBtnId).addEventListener('click', () => {
    setThrottleControl(sliderId, valueId, ctrlKey, 1500);
    sendControl(true);
  });
}

wireThrottle('left-thr-ctrl', 'left-thr-ctrl-val', 'left_thr', 'left-thr-center');
wireThrottle('right-thr-ctrl', 'right-thr-ctrl-val', 'right_thr', 'right-thr-center');

el('steer-slider').addEventListener('input', (e) => {
  const value = parseInt(e.target.value, 10);
  ctrl.steering = steeringUsToCtrl(value);
  el('steer-val-text').textContent = formatSteerValue(value);
  sendControl();
});

function maybeAutoCenterSteering() {
  if (!el('chk-auto-center').checked) return;
  setSteeringControl(1500, true);
}

el('steer-slider').addEventListener('mouseup', maybeAutoCenterSteering);
el('steer-slider').addEventListener('touchend', maybeAutoCenterSteering);
el('steer-center-btn').addEventListener('click', () => setSteeringControl(1500, true));

el('web-arm-btn').addEventListener('click', () => {
  socket.emit('arm', { armed: true });
});

el('web-disarm-btn').addEventListener('click', () => {
  resetThrottleControls();
  setSteeringControl(1500, false);
  socket.emit('arm', { armed: false });
  if (webCtrlEnabled) socket.emit('control', { ...ctrl, steering: 0, left_thr: 1500, right_thr: 1500 });
});

// No mode-change listeners: mode is owned by the controller's D21 switch.

resetThrottleControls();
setSteeringControl(1500, false);
updateArmBtn(false);
updateModeReadout(-1, false, false, false);
refreshFooterStatus();

// ── Operator Control ────────────────────────────────────────────────────────
el('acquire-ctrl-btn').addEventListener('click', () => {
  socket.emit('acquire_control', {});
  el('auth-status').textContent = '⏳ 請求中… (等待 ACK)';
  el('auth-status').className = 'auth-status auth-pending';
});

el('release-ctrl-btn').addEventListener('click', () => {
  socket.emit('release_control', {});
  el('auth-status').textContent = '— 未請求控制權 —';
  el('auth-status').className = 'auth-status auth-none';
});

el('sysid-btn').addEventListener('click', () => {
  const sysid = parseInt(el('gcs-sysid').value, 10);
  if (sysid >= 1 && sysid <= 254) {
    socket.emit('set_gcs_sysid', { sysid });
    el('gcs-sysid').style.borderColor = 'var(--green)';
    setTimeout(() => { el('gcs-sysid').style.borderColor = ''; }, 1500);
  } else {
    el('gcs-sysid').style.borderColor = 'var(--red)';
    setTimeout(() => { el('gcs-sysid').style.borderColor = ''; }, 2000);
  }
});

// ── LED 手動選色 ────────────────────────────────────────────────────────────
(function () {
  const picker = el('led-picker');
  const pickerNum = el('led-picker-num');
  let activeLed = null;

  // 點擊 LED Card 開啟選色面板
  document.querySelectorAll('.led-card').forEach(card => {
    card.addEventListener('click', (e) => {
      const led = card.dataset.led;
      activeLed = parseInt(led);
      pickerNum.textContent = led;

      // 定位在卡片下方
      const rect = card.getBoundingClientRect();
      const parentRect = card.closest('section').getBoundingClientRect();
      picker.style.top  = (rect.bottom - parentRect.top + 6) + 'px';
      picker.style.left = (rect.left - parentRect.left) + 'px';
      picker.style.display = 'block';
      e.stopPropagation();
    });
  });

  // 點選顏色
  picker.querySelectorAll('.led-color-btn').forEach(btn => {
    btn.addEventListener('click', (e) => {
      if (activeLed === null) return;
      socket.emit('set_led', { led: activeLed, color: btn.dataset.color });
      picker.style.display = 'none';
      activeLed = null;
      e.stopPropagation();
    });
  });

  // 點擊其他地方關閉
  document.addEventListener('click', () => {
    picker.style.display = 'none';
    activeLed = null;
  });
})();
