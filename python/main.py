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
import time

from arduino.app_utils import App, Bridge, Leds
from arduino.app_bricks.web_ui import WebUI

from mavlink_handler import MAVLinkHandler

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
EDGE_RISING  = 1   # switch ON  → ARM
EDGE_FALLING = 2   # switch OFF → DISARM

# Control-authority state → int code sent to the MCU (must match sketch AUTH_* defines)
_AUTH_CODE = {"none": 0, "pending": 1, "granted": 2, "denied": 3}

_last_display_push = 0.0

# Latest MCU inputs (updated by on_inputs callback)
_last_inputs: dict = {
    "steering": 0,
    "left_thr": 1500,
    "right_thr": 1500,
    "sw_state": False,
}

# Web control mode: when True, MCU RC override is suppressed
_web_ctrl = False

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
    """
    global _led1_state, _led2_state, _led3_state, _led4_state
    global _prev_tx_count, _prev_rx_count, _led3_blink_count, _led4_blink_count

    lq = mavlink.link_quality()

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


def on_inputs(steering: int, left_thr: int, right_thr: int,
              sw_state: int, sw_edge: int):
    """Called by MCU (via Bridge.notify) with current control inputs."""
    _last_inputs["steering"]  = steering
    _last_inputs["left_thr"]  = left_thr
    _last_inputs["right_thr"] = right_thr
    _last_inputs["sw_state"]  = bool(sw_state)

    # Skip RC override when web control is active (web sends its own)
    if not _web_ctrl:
        mavlink.send_rc_override(steering, left_thr, right_thr)

    # Hardware engine switch always works regardless of web control mode
    if sw_edge == EDGE_RISING:
        logger.info("Engine switch ON → ARM")
        mavlink.send_arm()
    elif sw_edge == EDGE_FALLING:
        logger.info("Engine switch OFF → DISARM")
        mavlink.send_disarm()
        # Reset throttle to neutral on disarm (matches C# reference slider reset)
        _last_inputs["left_thr"]  = 1500
        _last_inputs["right_thr"] = 1500


# ── Web UI message handlers ───────────────────────────────────────────────────

def _handle_web_ctrl_set(_, data: dict):
    global _web_ctrl
    _web_ctrl = bool(data.get("enabled", False))
    logger.info("Web control mode: %s", "ON" if _web_ctrl else "OFF")
    if not _web_ctrl:
        mavlink.send_safe_state()


def _handle_control(_, data: dict):
    """Web browser sending steering + throttle values."""
    if not _web_ctrl:
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
    """Web browser MANUAL / GUIDED / RTL mode change."""
    try:
        mode = int(data.get("mode", 0))
        logger.info("Web → SET_MODE(%d)", mode)
        mavlink._send_set_mode(mode)
    except Exception as exc:
        logger.error("Set mode error: %s", exc)


def _handle_acquire_control(_, data: dict):
    """Web browser requesting operator control (MAV_CMD_REQUEST_OPERATOR_CONTROL, cmd=410)."""
    logger.info("Web → ACQUIRE_CONTROL (sysid=%s)", mavlink.source_system)
    mavlink.send_request_operator_control(request=True)


def _handle_release_control(_, data: dict):
    """Web browser releasing operator control (MAV_CMD_REQUEST_OPERATOR_CONTROL, cmd=410, param2=0)."""
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


def loop():
    """Main loop — push vessel status to MCU for LCD display at 5 Hz, update LEDs."""
    global _last_display_push
    now = time.time()

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
            _AUTH_CODE.get(mavlink.control_authority, 0),  # 13. 控制權狀態 (header badge)
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
                "sw_state":  _last_inputs["sw_state"],
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


