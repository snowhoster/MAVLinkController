"""
main.py — USV Remote Controller (Linux / MPU side)

MCU→MPU: Bridge.provide("on_inputs", ...)  — receives ADC values + switch edge
MPU→MCU: Bridge.call("update_display", ...)— pushes vessel status for LCD
WebUI:   ui.send_message("state", {...})   — broadcasts all data to browser at 5 Hz
"""

import json
import logging
import os
import socket
import threading
import time

from arduino.app_utils import App, Bridge, Leds
from arduino.app_bricks.web_ui import WebUI

from mavlink_handler import MAVLinkHandler, MODE_MANUAL, MODE_ACRO

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
)
logger = logging.getLogger(__name__)

# ── Configuration ─────────────────────────────────────────────────────────────
_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "usv_config.json")
_DEFAULT_CFG = {
    "protocol":    "udp",
    "remote_ip":   "192.168.1.100",
    "remote_port": 14550,
    "local_port":  14551,      # fixed port so the USV can reply-to-sender (symmetric UDP socket)
}


def _load_config() -> dict:
    try:
        with open(_CONFIG_PATH) as f:
            raw = json.load(f)
        # Migrate old format (protocol/ip/port) to new format
        if "ip" in raw and "remote_ip" not in raw:
            raw["remote_ip"]   = raw.pop("ip")
            raw["remote_port"] = raw.pop("port", 14550)
            raw["local_port"]  = raw.pop("local_port", 0)
            proto = raw.pop("protocol", "udpout")
            raw["protocol"]    = "udp" if proto.startswith("udp") else "tcp"
        return {k: raw.get(k, _DEFAULT_CFG[k]) for k in _DEFAULT_CFG}
    except Exception:
        return dict(_DEFAULT_CFG)


def _save_config(cfg: dict):
    try:
        with open(_CONFIG_PATH, "w") as f:
            json.dump(cfg, f, indent=2)
    except Exception as exc:
        logger.error("Failed to save config: %s", exc)


_usv_config = _load_config()


def _get_local_ip(remote_ip: str) -> str:
    """Best-effort local outbound IP for the route toward remote_ip (no packets sent)."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect((remote_ip, 1))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "0.0.0.0"


_local_ip = _get_local_ip(_usv_config["remote_ip"])

# ── MAVLink handler ───────────────────────────────────────────────────────────
mavlink = MAVLinkHandler(
    protocol    = _usv_config["protocol"],
    remote_ip   = _usv_config["remote_ip"],
    remote_port = _usv_config["remote_port"],
    local_port  = _usv_config["local_port"],
)

# ── Web UI ────────────────────────────────────────────────────────────────────
ui = WebUI()

# Switch edge constants (must match sketch EDGE_* defines)
EDGE_NONE    = 0
EDGE_RISING  = 1   # switch OFF → ON
EDGE_FALLING = 2   # switch ON  → OFF

# Control-button gesture codes (must match sketch CTRL_* defines)
CTRL_NONE  = 0
CTRL_SHORT = 1   # 短按 → 請求控制權
CTRL_LONG  = 2   # 長按 2s → 釋放控制權，或解除急停閂鎖

# Control-authority state → int code sent to the MCU (must match sketch AUTH_* defines)
_AUTH_CODE  = {"none": 0, "pending": 1, "granted": 2, "denied": 3}
_AUTH_ESTOP = 4   # 急停閂鎖中 — 覆蓋一切控制權狀態

_last_display_push = 0.0

# Latest MCU inputs (updated by on_inputs callback)
_last_inputs: dict = {
    "steering": 0,
    "left_thr": 1500,
    "right_thr": 1500,
    "l_eng_on": False,
    "r_eng_on": False,
}

# Web control mode: when True, MCU RC override is suppressed
_web_ctrl = False

# ── Emergency stop ────────────────────────────────────────────────────────────
# _estop_active : physical button state right now
# _estop_latched: engaged and staying engaged until deliberately cleared.
#   The latch is what makes the E-STOP safe — without it, releasing the mushroom
#   head would silently reconnect and restore throttle output with no warning.
_estop_active  = False
_estop_latched = False

# Last ARM/DISARM intent we sent, derived from the two engine switches. Tracked
# locally rather than read back from mavlink.armed, which lags by a heartbeat.
_arm_cmd_state = False

# ── Mode switch (D6) ──────────────────────────────────────────────────────────
# Only two modes are offered: 手動 (MANUAL) and 定向 (ACRO). Without a chart
# there is no way to place waypoints, so the waypoint-driven modes (AUTO,
# GUIDED, RTL) have no operator interface and are not selectable.
#
# In ACRO the vessel closes the heading loop itself: CH1 becomes a turn rate and
# centring the wheel holds the current heading. The controller sends exactly the
# same RC channels in both modes — only the vessel's interpretation changes.
#
# _mode_sw_acro is the physical switch position; None until the first report so
# that the initial state is always pushed to the vessel.
_mode_sw_acro = None

# ACRO needs a trustworthy heading. gps_fix is the best proxy available over
# telemetry — a 2D fix or better implies the EKF has a usable yaw estimate.
# Below this, entering 定向 would have the vessel hold a heading it has guessed.
MIN_FIX_FOR_ACRO = 2

# True when 定向 was requested but refused / dropped; drives the LCD mismatch
# indicator and the web badge.
_mode_denied = False

# Earliest time the 定向 retry may run again. Throttled so a fix hovering at the
# threshold cannot flip the vessel between modes several times a second.
_mode_retry_after = 0.0
MODE_RETRY_S = 3.0

# LED state cache — sentinel (-1,-1,-1) forces first-iteration update
_led1_state: tuple = (-1, -1, -1)
_led2_state: tuple = (-1, -1, -1)
_led3_state: tuple = (-1, -1, -1)
_led4_state: tuple = (-1, -1, -1)

# Manual override: None = auto, tuple = forced color
_led_manual: dict = {1: None, 2: None, 3: None, 4: None}

# TX/RX blink tracking (LED3=TX, LED4=RX)
_BLINK_TICKS  = 5           # 5 × 20 ms = 100 ms blink duration
_prev_tx_count: int = 0
_prev_rx_count: int = 0
_led3_blink_count: int = 0
_led4_blink_count: int = 0

# E-STOP indicator blink counter (50 Hz loop ticks)
_estop_blink_tick: int = 0


def _led_to_rgb(state) -> dict:
    """Convert a LED state tuple (bool or int values) to an RGB dict 0–255."""
    if state[0] == -1:
        return {"r": 0, "g": 0, "b": 0}
    if isinstance(state[0], bool):
        return {"r": 255 if state[0] else 0,
                "g": 255 if state[1] else 0,
                "b": 255 if state[2] else 0}
    return {"r": int(state[0]), "g": int(state[1]), "b": int(state[2])}


def _update_leds():
    """
    Update all four LEDs.

    LED1 (Python, R/G/B bool):  MAVLink link quality
      • Green  = LQ ≥ 80  (good)
      • Yellow = LQ 20–79 (degrading)
      • Red    = LQ < 20  (lost)
      • Off    = never connected

    LED2 (Python, R/G/B bool):  Operator control authority
      • Green  = granted
      • Yellow = pending (waiting for ACK)
      • Red    = denied
      • Off    = not requested / none

    LED3 (MCU via Bridge, PWM 0–255):  TX activity — blinks green on each sent packet
    LED4 (MCU via Bridge, ON/OFF bool): RX activity — blinks blue on each received packet

    While the E-STOP is latched, LED1 and LED2 both flash red at 1 Hz and
    manual LED overrides are ignored — the emergency indication must not be
    something a stale web command can paint over.
    """
    global _led1_state, _led2_state, _led3_state, _led4_state
    global _prev_tx_count, _prev_rx_count, _led3_blink_count, _led4_blink_count
    global _estop_blink_tick

    lq = mavlink.link_quality()

    # ── E-STOP latched: both status LEDs flash red, overriding everything ──────
    if _estop_latched:
        _estop_blink_tick = (_estop_blink_tick + 1) % 50   # 50 Hz loop → 1 Hz blink
        on = _estop_blink_tick < 25
        new = (True, False, False) if on else (False, False, False)
        if new != _led1_state:
            Leds.set_led1_color(*new)
            _led1_state = new
        if new != _led2_state:
            Leds.set_led2_color(*new)
            _led2_state = new
        # Kill the TX/RX activity LEDs — there is no traffic once the link is cut,
        # and leaving them lit would suggest otherwise.
        if _led3_state != (0, 0, 0):
            Bridge.call("set_led3_color", 0, 0, 0)
            _led3_state = (0, 0, 0)
        if _led4_state != (False, False, False):
            Bridge.call("set_led4_color", False, False, False)
            _led4_state = (False, False, False)
        return

    # ── LED1: link quality ────────────────────────────────────────────────────
    if _led_manual[1] is None:
        if lq >= 80:
            new1 = (False, True,  False)   # green
        elif lq >= 20:
            new1 = (True,  True,  False)   # yellow
        elif lq > 0:
            new1 = (True,  False, False)   # red
        else:
            new1 = (False, False, False)   # off
        if new1 != _led1_state:
            Leds.set_led1_color(*new1)
            _led1_state = new1

    # ── LED2: operator control authority ──────────────────────────────────────
    if _led_manual[2] is None:
        auth = mavlink.control_authority
        if auth == "granted":
            new2 = (False, True,  False)   # green
        elif auth == "pending":
            new2 = (True,  True,  False)   # yellow
        elif auth == "denied":
            new2 = (True,  False, False)   # red
        else:                              # "none"
            new2 = (False, False, False)   # off
        if new2 != _led2_state:
            Leds.set_led2_color(*new2)
            _led2_state = new2

    # ── LED3: TX blink (MCU side, PWM) ────────────────────────────────────────
    if _led_manual[3] is None:
        if mavlink.tx_count != _prev_tx_count:
            _prev_tx_count    = mavlink.tx_count
            _led3_blink_count = _BLINK_TICKS
        if _led3_blink_count > 0:
            _led3_blink_count -= 1
            new3 = (0, 200, 0)    # green — TX active
        else:
            new3 = (0, 0, 0)      # off
        if new3 != _led3_state:
            Bridge.call("set_led3_color", *new3)
            _led3_state = new3

    # ── LED4: RX blink (MCU side, ON/OFF bool) ────────────────────────────────
    if _led_manual[4] is None:
        if mavlink.rx_count != _prev_rx_count:
            _prev_rx_count    = mavlink.rx_count
            _led4_blink_count = _BLINK_TICKS
        if _led4_blink_count > 0:
            _led4_blink_count -= 1
            new4 = (False, False, True)   # blue — RX active
        else:
            new4 = (False, False, False)  # off
        if new4 != _led4_state:
            Bridge.call("set_led4_color", *new4)
            _led4_state = new4


def _engage_estop():
    """Run the emergency-stop sequence off the Bridge callback thread.

    The sequence blocks for ~200 ms (repeated sends plus socket teardown);
    doing that inline would stall the MCU notification handler.
    """
    global _estop_latched
    _estop_latched = True
    threading.Thread(target=mavlink.emergency_stop, daemon=True, name="mav-estop").start()


def _clear_estop():
    """Clear the latch and bring the link back up."""
    global _estop_latched, _arm_cmd_state, _mode_sw_acro
    _estop_latched = False
    _arm_cmd_state = False
    # Forget the last known switch position so the next on_inputs re-pushes the
    # mode. After a reconnect the vessel may be in any mode, and the switch not
    # having moved must not be mistaken for the mode already being correct.
    _mode_sw_acro  = None
    _last_inputs["left_thr"]  = 1500
    _last_inputs["right_thr"] = 1500
    threading.Thread(target=mavlink.resume_from_estop, daemon=True, name="mav-resume").start()


def _apply_mode(want_acro: bool) -> bool:
    """Command the mode the switch is asking for. Returns True if accepted.

    定向 is refused without a usable heading source: ACRO holds whatever heading
    the EKF believes it is on, so a bad estimate means the vessel drives off on
    a heading nobody chose. Refusing leaves it in 手動, where the operator's
    wheel maps straight to the rudder and the vessel cannot run away.
    """
    global _mode_denied
    if want_acro and mavlink.gps_fix < MIN_FIX_FOR_ACRO:
        logger.warning("定向 refused — gps_fix=%d < %d (no usable heading source)",
                       mavlink.gps_fix, MIN_FIX_FOR_ACRO)
        _mode_denied = True
        mavlink.set_mode(MODE_MANUAL)
        return False
    _mode_denied = False
    return mavlink.set_mode(MODE_ACRO if want_acro else MODE_MANUAL)


def on_inputs(steering: int, left_thr: int, right_thr: int,
              l_eng_state: int, l_eng_edge: int,
              r_eng_state: int, r_eng_edge: int,
              ctrl_edge: int, estop_state: int, mode_sw_acro: int):
    """Called by MCU (via Bridge.notify) with current control inputs."""
    global _estop_active, _arm_cmd_state, _mode_sw_acro

    # ── Emergency stop — evaluated before anything else ───────────────────────
    estop = bool(estop_state)
    if estop and not _estop_active:
        logger.critical("E-STOP pressed → cutting link")
        _estop_active = True
        _engage_estop()
        return
    _estop_active = estop

    if _estop_latched:
        # Latched: ignore every control input. The only gesture that gets
        # through is the clear, and only once the physical button is released.
        if ctrl_edge == CTRL_LONG and not _estop_active:
            logger.warning("E-STOP clear gesture accepted")
            _clear_estop()
        elif ctrl_edge == CTRL_LONG:
            logger.warning("E-STOP clear refused — reset the physical button first")
        return

    _last_inputs["steering"]  = steering
    _last_inputs["left_thr"]  = left_thr
    _last_inputs["right_thr"] = right_thr
    _last_inputs["l_eng_on"]  = bool(l_eng_state)
    _last_inputs["r_eng_on"]  = bool(r_eng_state)

    # ── Mode switch → MANUAL / ACRO ───────────────────────────────────────────
    want_acro = bool(mode_sw_acro)
    if want_acro != _mode_sw_acro:          # also fires on the first report
        logger.info("Mode switch → %s", "定向 (ACRO)" if want_acro else "手動 (MANUAL)")
        _mode_sw_acro = want_acro
        _apply_mode(want_acro)

    # Per-engine gates — a switched-off engine pins its channel to 1500 even
    # when the web UI is driving, so the physical switch is always authoritative.
    mavlink.set_engine_gates(bool(l_eng_state), bool(r_eng_state))

    # Skip RC override when web control is active (web sends its own)
    if not _web_ctrl:
        mavlink.send_rc_override(steering, left_thr, right_thr)

    # ── Engine switches → vessel ARM/DISARM ───────────────────────────────────
    # MAVLink arms the whole vessel; there is no per-engine ARM. So either
    # switch being ON arms the ship, and only both OFF disarms it. Which engine
    # actually turns is decided by the throttle gates above.
    if l_eng_edge != EDGE_NONE or r_eng_edge != EDGE_NONE:
        any_on = bool(l_eng_state) or bool(r_eng_state)
        if any_on != _arm_cmd_state:
            _arm_cmd_state = any_on
            if any_on:
                logger.info("Engine switch (L=%s R=%s) → ARM", bool(l_eng_state), bool(r_eng_state))
                mavlink.send_arm()
            else:
                logger.info("Both engine switches OFF → DISARM")
                mavlink.send_disarm()
                # Reset throttle to neutral on disarm (matches C# reference slider reset)
                _last_inputs["left_thr"]  = 1500
                _last_inputs["right_thr"] = 1500

    # ── Control-authority button ──────────────────────────────────────────────
    if ctrl_edge == CTRL_SHORT:
        logger.info("Hardware button → ACQUIRE_CONTROL (sysid=%s)", mavlink.source_system)
        mavlink.send_request_operator_control(request=True)
    elif ctrl_edge == CTRL_LONG:
        logger.info("Hardware button (hold) → RELEASE_CONTROL (sysid=%s)", mavlink.source_system)
        mavlink.send_request_operator_control(request=False)


# ── Web UI message handlers ───────────────────────────────────────────────────

def _blocked_by_estop(action: str) -> bool:
    """Refuse any command that would drive the vessel while the E-STOP is latched.

    Clearing the latch is deliberately not exposed to the web UI — it requires
    resetting the physical button plus a 2 s hold on the control key, so the
    person clearing it is the person standing at the controller.
    """
    if _estop_latched:
        logger.warning("E-STOP latched — refusing web %s", action)
        return True
    return False


def _handle_web_ctrl_set(_, data: dict):
    global _web_ctrl
    if _blocked_by_estop("web_ctrl_set"):
        return
    _web_ctrl = bool(data.get("enabled", False))
    logger.info("Web control mode: %s", "ON" if _web_ctrl else "OFF")
    if not _web_ctrl:
        mavlink.send_safe_state()


def _handle_control(_, data: dict):
    """Web browser sending steering + throttle values."""
    if not _web_ctrl or _estop_latched:
        return
    try:
        steering  = int(data.get("steering",  0))
        left_thr  = int(data.get("left_thr",  1500))
        right_thr = int(data.get("right_thr", 1500))
        _last_inputs.update({"steering": steering, "left_thr": left_thr, "right_thr": right_thr})
        mavlink.send_rc_override(steering, left_thr, right_thr)
    except Exception as exc:
        logger.error("Web control error: %s", exc)


def _handle_arm(_, data: dict):
    """Web browser ARM / DISARM request."""
    if _blocked_by_estop("arm"):
        return
    try:
        if bool(data.get("armed", False)):
            logger.info("Web → ARM")
            mavlink.send_arm()
        else:
            logger.info("Web → DISARM")
            mavlink.send_disarm()
            # Reset throttle to neutral on disarm (matches C# reference slider reset)
            _last_inputs["left_thr"]  = 1500
            _last_inputs["right_thr"] = 1500
    except Exception as exc:
        logger.error("Web arm error: %s", exc)


def _handle_set_led(_, data: dict):
    """Web browser sets LED color manually, or restores auto mode."""
    global _led_manual, _led1_state, _led2_state, _led3_state, _led4_state
    led_num = int(data.get("led", 0))
    color   = data.get("color", "auto")
    if led_num not in (1, 2, 3, 4):
        return
    COLOR_BOOL = {
        "red":    (True,  False, False),
        "green":  (False, True,  False),
        "blue":   (False, False, True),
        "yellow": (True,  True,  False),
        "off":    (False, False, False),
    }
    COLOR_PWM = {
        "red":    (200, 0,   0),
        "green":  (0,   200, 0),
        "blue":   (0,   0,   200),
        "yellow": (200, 200, 0),
        "off":    (0,   0,   0),
    }
    if color == "auto":
        _led_manual[led_num] = None
        # Reset sentinel so auto picks up on next cycle
        if   led_num == 1: _led1_state = (-1, -1, -1)
        elif led_num == 2: _led2_state = (-1, -1, -1)
        elif led_num == 3: _led3_state = (-1, -1, -1)
        elif led_num == 4: _led4_state = (-1, -1, -1)
        return
    if led_num in (1, 2):
        val = COLOR_BOOL.get(color, (False, False, False))
        _led_manual[led_num] = val
        if led_num == 1:
            Leds.set_led1_color(*val); _led1_state = val
        else:
            Leds.set_led2_color(*val); _led2_state = val
    elif led_num == 3:
        val = COLOR_PWM.get(color, (0, 0, 0))
        _led_manual[led_num] = val
        Bridge.call("set_led3_color", *val); _led3_state = val
    elif led_num == 4:
        val = COLOR_BOOL.get(color, (False, False, False))
        _led_manual[led_num] = val
        Bridge.call("set_led4_color", *val); _led4_state = val


def _handle_set_mode(_, data: dict):
    """Mode changes come from the D6 switch only — the web request is refused.

    Two authorities over one setting would fight: the switch re-asserts its
    position on every change and after every reconnect, so a web-selected mode
    would be silently reverted. Better to refuse it outright and keep the
    switch's physical position an honest indicator of the vessel's mode.
    """
    logger.warning("Web → SET_MODE refused; mode is controlled by the D6 switch")


def _handle_acquire_control(_, data: dict):
    """Web browser requesting operator control (MAV_CMD_REQUEST_OPERATOR_CONTROL, cmd=410)."""
    if _blocked_by_estop("acquire_control"):
        return
    logger.info("Web → ACQUIRE_CONTROL (sysid=%s)", mavlink.source_system)
    mavlink.send_request_operator_control(request=True)


def _handle_release_control(_, data: dict):
    """Web browser releasing operator control (MAV_CMD_REQUEST_OPERATOR_CONTROL, cmd=410, param2=0)."""
    if _blocked_by_estop("release_control"):
        return
    logger.info("Web → RELEASE_CONTROL (sysid=%s)", mavlink.source_system)
    mavlink.send_request_operator_control(request=False)


def _handle_set_gcs_sysid(_, data: dict):
    """Web browser setting the GCS source system ID (1–254)."""
    try:
        sysid = int(data.get("sysid", 255))
        if 1 <= sysid <= 254:
            logger.info("Web → SET_GCS_SYSID(%d)", sysid)
            mavlink.set_source_system(sysid)
        else:
            logger.warning("Invalid GCS sysid: %d", sysid)
    except Exception as exc:
        logger.error("Set sysid error: %s", exc)


def _handle_connect_usv(_, data: dict):
    """Web browser requesting USV connection change."""
    global _usv_config, _local_ip
    if _blocked_by_estop("connect_usv"):
        return
    protocol    = str(data.get("protocol",    "udp")).lower()
    remote_ip   = str(data.get("remote_ip",   "192.168.1.100")).strip()
    remote_port = int(data.get("remote_port", 14550))
    local_port  = int(data.get("local_port",  0))

    if protocol not in ("udp", "tcp"):
        logger.warning("Unknown protocol: %s", protocol)
        return
    if not remote_ip or remote_port < 1 or remote_port > 65535:
        logger.warning("Invalid remote params: %s:%s", remote_ip, remote_port)
        return
    # local_port is required for UDP (ship always sends telemetry to a fixed port)
    if protocol == "udp" and (local_port < 1 or local_port > 65535):
        logger.warning("local_port required for UDP, got: %s", local_port)
        return

    _usv_config = {
        "protocol":    protocol,
        "remote_ip":   remote_ip,
        "remote_port": remote_port,
        "local_port":  local_port,
    }
    _save_config(_usv_config)
    logger.info("Reconnecting  local:%s → %s:%s (%s)",
                local_port or "auto", remote_ip, remote_port, protocol)
    mavlink.reconnect(protocol, remote_ip, remote_port, local_port)
    _local_ip = _get_local_ip(remote_ip)


def _mode_watchdog(now: float):
    """Keep 定向 valid, or drop out of it.

    Losing the fix while in ACRO is the dangerous case: the vessel keeps holding
    a heading it can no longer verify. Falling back to 手動 hands steering
    straight back to the operator's wheel.

    The switch stays where it is, so once the fix recovers the requested mode is
    re-applied — the switch position is a standing instruction, not a one-shot.
    """
    global _mode_denied, _mode_retry_after

    if _estop_latched or not _mode_sw_acro:
        return

    fix_ok = mavlink.gps_fix >= MIN_FIX_FOR_ACRO

    if not fix_ok and mavlink.mode == MODE_ACRO:
        logger.warning("定向 dropped — gps_fix=%d lost, falling back to 手動", mavlink.gps_fix)
        _mode_denied = True
        _mode_retry_after = now + MODE_RETRY_S
        mavlink.set_mode(MODE_MANUAL)
    elif fix_ok and _mode_denied and now >= _mode_retry_after:
        _mode_retry_after = now + MODE_RETRY_S
        logger.info("定向 retry — gps_fix=%d recovered", mavlink.gps_fix)
        _apply_mode(True)


def loop():
    """Main loop — push vessel status to MCU for LCD display at 5 Hz, update LEDs."""
    global _last_display_push
    now = time.time()

    _mode_watchdog(now)

    if now - _last_display_push >= 0.2:
        lq = mavlink.link_quality()

        Bridge.call("update_display",
            int(mavlink.speed_knots * 10),  # 1. speed_x10
            int(mavlink.heading),           # 2. heading
            int(mavlink.battery_pct),       # 3. bat
            int(mavlink.gps_fix),           # 4. fix (colors the GPS position row)
            int(mavlink.armed),             # 5. armed
            int(mavlink.mode),              # 6. mode
            int(mavlink.lat_degE7),         # 7. GPS latitude  (deg * 1e7)
            int(mavlink.lon_degE7),         # 8. GPS longitude (deg * 1e7)
            _usv_config["remote_ip"],       # 9. 通訊：船舶 IP
            int(_usv_config["remote_port"]),# 10. 通訊：船舶 Port
            _local_ip,                      # 11. 本機 IP
            int(mavlink.local_bound_port),  # 12. 本機 Port
            # 13. 控制權狀態 (header badge) — 急停閂鎖覆蓋一切，並讓 LCD 保持
            #     急停橫幅（實體按鈕已復位但尚未解除閂鎖時仍需顯示）
            _AUTH_ESTOP if _estop_latched else _AUTH_CODE.get(mavlink.control_authority, 0),
            int(lq > 0),                    # 14. MAVLink 連線狀態 (0=斷線 1=已連線)
        )

        # Push full state to web browser
        hb_age = round(now - mavlink._last_hb_time, 1) if mavlink._last_hb_time > 0 else -1
        ui.send_message("state", {
            "connection": {
                "protocol":    _usv_config["protocol"],
                "remote_ip":   _usv_config["remote_ip"],
                "remote_port": _usv_config["remote_port"],
                "local_ip":    _local_ip,
                "local_port":  mavlink.local_bound_port,
                "connected":   lq > 0,
            },
            "inputs": {
                "steering":  _last_inputs["steering"],
                "left_thr":  _last_inputs["left_thr"],
                "right_thr": _last_inputs["right_thr"],
                "l_eng_on":  _last_inputs["l_eng_on"],
                "r_eng_on":  _last_inputs["r_eng_on"],
            },
            "comms": {
                "link_quality": lq,
                "hb_age_s":     hb_age,
                "in_failsafe":  mavlink.in_failsafe,
                "web_ctrl":     _web_ctrl,
                "tx_count":     mavlink.tx_count,
                "rx_count":     mavlink.rx_count,
                "control_authority": mavlink.control_authority,
                "source_system":     mavlink.source_system,
                "estop_latched":     _estop_latched,
                "estop_active":      _estop_active,
                "mode_sw_acro":      bool(_mode_sw_acro),
                "mode_denied":       _mode_denied,
            },
            "telemetry": {
                "speed_kn":    round(mavlink.speed_knots, 1),
                "speed_ms":    round(mavlink.speed_knots / 1.94384, 2),
                "heading":     mavlink.heading,
                "battery_pct": mavlink.battery_pct,
                "voltage_v":   round(mavlink.voltage_mv / 1000.0, 2),
                "gps_fix":     mavlink.gps_fix,
                "gps_sats":    mavlink.gps_sats,
                "lat_deg7":    mavlink.lat_degE7,
                "lon_deg7":    mavlink.lon_degE7,
                "armed":       mavlink.armed,
                "mode":        mavlink.mode,
                "roll":        round(mavlink.roll_deg, 1),
                "pitch":       round(mavlink.pitch_deg, 1),
                "yaw":         round(mavlink.yaw_deg, 1),
                "throttle_pct": mavlink.throttle_pct,
                "servo1_raw":  mavlink.servo1_raw,
                "servo3_raw":  mavlink.servo3_raw,
                "servo4_raw":  mavlink.servo4_raw,
            },
            "leds": {
                "led1": _led_to_rgb(_led1_state),
                "led2": _led_to_rgb(_led2_state),
                "led3": _led_to_rgb(_led3_state),
                "led4": _led_to_rgb(_led4_state),
            },
        })

        _last_display_push = now

    _update_leds()

    time.sleep(0.02)   # 50 Hz loop rate


# ── Startup ───────────────────────────────────────────────────────────────────
Bridge.provide("on_inputs", on_inputs)
ui.on_message("web_ctrl_set",     _handle_web_ctrl_set)
ui.on_message("control",          _handle_control)
ui.on_message("arm",              _handle_arm)
ui.on_message("set_led",          _handle_set_led)
ui.on_message("set_mode",         _handle_set_mode)
ui.on_message("connect_usv",      _handle_connect_usv)
ui.on_message("acquire_control",  _handle_acquire_control)
ui.on_message("release_control",  _handle_release_control)
ui.on_message("set_gcs_sysid",    _handle_set_gcs_sysid)
mavlink.connect()
logger.info("USV Controller ready — waiting for MCU and USV heartbeat")

App.run(user_loop=loop)


