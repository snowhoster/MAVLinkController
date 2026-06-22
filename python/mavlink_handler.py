"""
mavlink_handler.py — MAVLink v2 communication with the USV.

Based on the USV receiver (C# MavLinkDriver) analysis:

Sends to USV:
  • HEARTBEAT (1 Hz)
  • RC_CHANNELS_OVERRIDE #70  (10 Hz — CH1=Steering, CH3=LeftThr, CH4=RightThr)
  • COMMAND_LONG #76 cmd=400  (ARM/DISARM)
  • COMMAND_LONG #76 cmd=176  (MAV_CMD_DO_SET_MODE)

Receives from USV:
  • HEARTBEAT #0       → armed, custom_mode
  • VFR_HUD #74        → airspeed, groundspeed, heading, throttle
  • GLOBAL_POSITION_INT #33 → lat, lon, alt, heading (cdeg)
  • ATTITUDE #30       → roll, pitch, yaw (radians)
  • SYS_STATUS #1      → voltage_battery(mV), current_battery(cA), battery_remaining(%)
  • SERVO_OUTPUT_RAW #36 → servo1–4 raw PWM

Failsafe:
  If no control packet is sent for CONTROL_FAILSAFE_S seconds,
  send CH1/3/4 = 1500 to stop the USV.
"""

import logging
import math
import os
import threading
import time
from typing import Optional

# Force MAVLink v2 (STX=0xFD) — C# MavLinkDriver parser only accepts v2 frames.
# Must be set before mavutil.mavlink_connection() is called.
os.environ['MAVLINK20'] = '1'

from pymavlink import mavutil

logger = logging.getLogger(__name__)

# ArduPilot Rover/Boat custom modes (matches USV Set_Mode ControlType)
MODE_MANUAL = 0
MODE_RTL    = 11
MODE_GUIDED = 15

CONTROL_FAILSAFE_S = 1.0   # seconds without control → send neutral


class MAVLinkHandler:
    """Thread-safe MAVLink handler matching the USV's C# MavLinkDriver protocol."""

    def __init__(self,
                 protocol: str  = "udp",
                 remote_ip: str = "192.168.1.100",
                 remote_port: int = 14550,
                 local_port: int  = 0,
                 source_system: int = 255,
                 source_component: int = 190,
                 target_system: int = 1,
                 target_component: int = 1):
        self.protocol         = protocol        # "udp" | "tcp"
        self.remote_ip        = remote_ip       # USV IP
        self.remote_port      = remote_port     # USV listening port
        self.local_port       = local_port      # local bind port (0 = auto)
        self.source_system    = source_system
        self.source_component = source_component
        self.target_system    = target_system
        self.target_component = target_component

        self._conn: Optional[mavutil.mavudp] = None
        self._send_lock  = threading.Lock()
        self._running    = False

        # ── Telemetry (updated by RX thread) ───────────────────────────────
        self.speed_knots:    float = 0.0   # groundspeed converted to kn
        self.heading:        int   = 0     # degrees 0–359
        self.altitude_m:     float = 0.0   # metres
        self.climb_ms:       float = 0.0   # m/s
        self.throttle_pct:   int   = 0     # 0–100
        self.battery_pct:    int   = 0     # 0–100 (%)
        self.voltage_mv:     int   = 0     # millivolts
        self.current_ca:     int   = 0     # centi-amps
        self.gps_fix:        int   = 0     # 0=none 2=2D 3=3D
        self.gps_sats:       int   = 0
        self.lat_degE7:      int   = 0
        self.lon_degE7:      int   = 0
        self.roll_deg:       float = 0.0
        self.pitch_deg:      float = 0.0
        self.yaw_deg:        float = 0.0
        self.servo1_raw:     int   = 1500  # current servo outputs (μs)
        self.servo3_raw:     int   = 1500
        self.servo4_raw:     int   = 1500
        self.armed:          bool  = False
        self.mode:           int   = 0
        self.in_failsafe:    bool  = False

        # Packet counters (displayed in web UI comms panel)
        self.tx_count:       int   = 0
        self.rx_count:       int   = 0

        self._last_hb_time:      float = 0.0
        self._last_control_time: float = 0.0
        self._last_send_time:    float = 0.0   # updated on every successful TX

        # Cached RC values — continuously re-sent by _control_loop at 10 Hz
        # (mirrors C# SendTimer_Tick which always sends the current slider values)
        self._rc_ch1: int = 1500   # steering  μs
        self._rc_ch3: int = 1500   # left thr  μs
        self._rc_ch4: int = 1500   # right thr μs

        # Operator control authority state
        # "none" | "pending" | "granted" | "denied"
        self.control_authority:       str  = "none"
        self._pending_control_request: bool = False

    def _build_conn_str(self) -> str:
        """Build pymavlink connection string from current config."""
        if self.protocol == "tcp":
            return f"tcp:{self.remote_ip}:{self.remote_port}"
        elif self.local_port:
            return f"udpin:0.0.0.0:{self.local_port}"
        else:
            return f"udpout:{self.remote_ip}:{self.remote_port}"

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def connect(self) -> bool:
        """Connect to USV. Returns True on success, False on error."""
        conn_str = self._build_conn_str()
        logger.info("Connecting  local=%s  →  %s:%s  (%s)",
                    self.local_port or "auto", self.remote_ip, self.remote_port, self.protocol)
        try:
            self._conn = mavutil.mavlink_connection(
                conn_str,
                source_system=self.source_system,
                source_component=self.source_component,
            )
        except Exception as exc:
            logger.error("mavlink_connection failed: %s", exc)
            self._conn = None
            return False

        # For udpin: pre-add remote address to clients so we can send before
        # receiving the first packet (pymavlink write() only sends to self.clients)
        if self.protocol != "tcp" and self.local_port:
            self._conn.clients.add((self.remote_ip, self.remote_port))
            logger.info("Pre-added remote client: %s:%s", self.remote_ip, self.remote_port)

        self._running = True
        threading.Thread(target=self._rx_loop,        daemon=True, name="mav-rx").start()
        threading.Thread(target=self._heartbeat_loop, daemon=True, name="mav-hb").start()
        threading.Thread(target=self._control_loop,   daemon=True, name="mav-ctrl").start()
        logger.info("MAVLink handler running  conn_str=%s", conn_str)
        return True

    def disconnect(self):
        self._running = False
        if self._conn:
            self._conn.close()

    def reconnect(self, protocol: str, remote_ip: str, remote_port: int, local_port: int):
        """Stop current connection and restart with new settings."""
        logger.info("Reconnecting  %s:%s (local:%s) → %s:%s (local:%s)",
                    self.remote_ip, self.remote_port, self.local_port,
                    remote_ip, remote_port, local_port)
        self._running = False
        if self._conn:
            try:
                self._conn.close()
            except Exception:
                pass
            self._conn = None
        time.sleep(0.4)   # wait for background threads to exit

        self.protocol    = protocol
        self.remote_ip   = remote_ip
        self.remote_port = remote_port
        self.local_port  = local_port

        # Reset stale telemetry
        self._last_hb_time      = 0.0
        self._last_control_time = 0.0
        self._last_send_time    = 0.0
        self.in_failsafe        = False
        self.tx_count           = 0
        self.rx_count           = 0
        self.connect()

    # ── Control commands ──────────────────────────────────────────────────────

    def send_rc_override(self, steering: int, left_thr: int, right_thr: int):
        """
        Send RC_CHANNELS_OVERRIDE (#70) to USV.

        steering  : -1000…+1000  (from MCU ADC, mapped to CH1 1000–2000 μs)
        left_thr  :  1000…2000 μs  → CH3
        right_thr :  1000…2000 μs  → CH4
        CH2/5-18  : 65535 (no override)

        Channel mapping matches USV driver:
          CH1 = Steering,  CH3 = Left Throttle,  CH4 = Right Throttle

        Values are cached so _control_loop can re-send at 10 Hz without
        requiring a new MCU input — mirrors C# SendTimer_Tick behaviour.
        """
        if not self._conn:
            return

        # Map steering -1000…+1000  →  1000…2000 μs
        ch1 = int(1500 + steering / 2)
        ch1 = max(1000, min(2000, ch1))
        ch3 = max(1000, min(2000, left_thr))
        ch4 = max(1000, min(2000, right_thr))

        # Cache for continuous re-send by _control_loop
        self._rc_ch1 = ch1
        self._rc_ch3 = ch3
        self._rc_ch4 = ch4

        self._last_control_time = time.time()
        self.in_failsafe = False

    def send_arm(self):
        """Set MANUAL mode then ARM — runs in background thread to avoid blocking Bridge callback."""
        if not self._conn:
            return
        threading.Thread(target=self._send_arm_async, daemon=True, name="mav-arm").start()

    def _send_arm_async(self):
        logger.info("Sending SET_MODE(MANUAL) + ARM")
        self._send_set_mode(MODE_MANUAL)
        time.sleep(0.15)
        with self._send_lock:
            self._conn.mav.command_long_send(
                self.target_system, self.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,      # confirmation
                1.0,    # param1: 1 = ARM
                0, 0, 0, 0, 0, 0,
            )

    def send_disarm(self):
        """DISARM (MAV_CMD_COMPONENT_ARM_DISARM = 400, param1=0)."""
        if not self._conn:
            return
        logger.info("Sending DISARM")
        with self._send_lock:
            self._conn.mav.command_long_send(
                self.target_system, self.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0.0, 0, 0, 0, 0, 0, 0,
            )

    def set_source_system(self, sysid: int):
        """Update the GCS source system ID used in all outgoing MAVLink messages."""
        self.source_system = sysid
        if self._conn:
            self._conn.source_system = sysid
            self._conn.mav.srcSystem  = sysid
        logger.info("GCS source_system updated to %d", sysid)

    def send_request_operator_control(self, request: bool):
        """Send MAV_CMD_REQUEST_OPERATOR_CONTROL (cmd=410).

        request=True  → request control  (param1=GCS sysid, param2=1)
        request=False → release control  (param1=GCS sysid, param2=0)

        ACK result is handled in _handle(); control_authority is updated there.
        """
        if not self._conn:
            return
        self._pending_control_request = request
        self.control_authority = "pending"
        logger.info("Sending REQUEST_OPERATOR_CONTROL request=%s (sysid=%s)",
                    request, self.source_system)
        with self._send_lock:
            self._conn.mav.command_long_send(
                self.target_system, self.target_component,
                410,    # MAV_CMD_REQUEST_OPERATOR_CONTROL
                0,      # confirmation
                float(self.source_system),    # param1: requesting GCS system ID
                1.0 if request else 0.0,      # param2: 1=request, 0=release
                0, 0, 0, 0, 0,
            )
        self.tx_count += 1

    def send_safe_state(self):
        """Send neutral RC override (CH1/3/4 = 1500) to stop USV and reset cache."""
        if not self._conn:
            return
        logger.warning("Failsafe: sending neutral RC override (1500/1500/1500)")
        self._rc_ch1 = 1500
        self._rc_ch3 = 1500
        self._rc_ch4 = 1500
        with self._send_lock:
            self._conn.mav.rc_channels_override_send(
                self.target_system, self.target_component,
                1500, 65535, 1500, 1500,
                65535, 65535, 65535, 65535,
                65535, 65535, 65535, 65535,
                65535, 65535, 65535, 65535,
                65535, 65535,
            )

    def link_quality(self) -> int:
        """Returns 0–100 based on received heartbeat recency.

        Only considers received packets from the ship. If no heartbeat has
        ever been received, returns 0 (not connected).
        """
        if not self._conn or not self._running:
            return 0
        if self._last_hb_time <= 0:
            return 0
        age = time.time() - self._last_hb_time
        if age < 2.0:
            return 100
        if age < 7.0:
            return int(100 - (age - 2.0) * 16.7)
        return 0

    # ── Private: mode change ──────────────────────────────────────────────────

    def _send_set_mode(self, custom_mode: int):
        """MAV_CMD_DO_SET_MODE (cmd=176) as used by USV driver Set_Mode."""
        with self._send_lock:
            self._conn.mav.command_long_send(
                self.target_system, self.target_component,
                176,    # MAV_CMD_DO_SET_MODE
                0,
                1.0,    # param1: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                float(custom_mode),
                0, 0, 0, 0, 0,
            )

    # ── Background threads ────────────────────────────────────────────────────

    def _heartbeat_loop(self):
        """Send GCS heartbeat at 1 Hz.

        Includes armed state (base_mode bit 7) and current custom_mode so the
        USV driver can track the GCS status — matches the C# MavLinkDriver
        reference behavior where heartbeat carries _isArmed and _currentCustomMode.
        """
        while self._running:
            if self._conn:
                base_mode      = 0x80 if self.armed else 0
                system_status  = 4 if self.armed else 3  # MAV_STATE_ACTIVE=4, MAV_STATE_STANDBY=3
                with self._send_lock:
                    self._conn.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        base_mode,
                        self.mode,
                        system_status,
                    )
                self._last_send_time = time.time()
                self.tx_count += 1
            time.sleep(1.0)

    def _control_loop(self):
        """
        10 Hz RC sender + failsafe monitor — mirrors C# SendTimer_Tick pattern.

        Every tick (100 ms):
          • If last user input is fresh  → re-send cached RC values to keep
            the ship moving (prevents ship-side failsafe from triggering).
          • If no input for CONTROL_FAILSAFE_S seconds → send neutral 1500.
        """
        while self._running:
            time.sleep(0.1)   # 10 Hz
            if not self._conn or self._last_control_time == 0:
                continue

            now = time.time()
            if now - self._last_control_time > CONTROL_FAILSAFE_S:
                if not self.in_failsafe:
                    logger.warning("Failsafe: sending neutral RC (1500/1500/1500)")
                    self.in_failsafe = True
                with self._send_lock:
                    self._conn.mav.rc_channels_override_send(
                        self.target_system, self.target_component,
                        1500, 65535, 1500, 1500,
                        65535, 65535, 65535, 65535,
                        65535, 65535, 65535, 65535,
                        65535, 65535, 65535, 65535,
                        65535, 65535,
                    )
                self._last_control_time = now  # avoid repeat spam
                self.tx_count += 1
            else:
                # Continuously re-send the last known RC values
                with self._send_lock:
                    self._conn.mav.rc_channels_override_send(
                        self.target_system, self.target_component,
                        self._rc_ch1, 65535, self._rc_ch3, self._rc_ch4,
                        65535, 65535, 65535, 65535,
                        65535, 65535, 65535, 65535,
                        65535, 65535, 65535, 65535,
                        65535, 65535,
                    )
                self.tx_count += 1

    def _rx_loop(self):
        """Receive and parse MAVLink telemetry from USV."""
        while self._running:
            if not self._conn:
                time.sleep(0.1)
                continue
            try:
                msg = self._conn.recv_match(blocking=True, timeout=0.5)
                if msg:
                    self._handle(msg)
            except Exception as exc:
                logger.error("MAVLink RX error: %s", exc)
                time.sleep(0.1)

    def _handle(self, msg):
        t = msg.get_type()
        self.rx_count += 1

        if t == "HEARTBEAT":
            self._last_hb_time = time.time()
            # Armed bit: base_mode & 0x80 (MAV_MODE_FLAG_SAFETY_ARMED)
            self.armed = bool(msg.base_mode & 0x80)
            self.mode  = msg.custom_mode

        elif t == "VFR_HUD":
            self.speed_knots  = msg.groundspeed * 1.94384  # m/s → kn
            self.heading      = int(msg.heading) % 360
            self.altitude_m   = msg.alt
            self.climb_ms     = msg.climb
            self.throttle_pct = int(msg.throttle)

        elif t == "GLOBAL_POSITION_INT":
            self.lat_degE7 = msg.lat
            self.lon_degE7 = msg.lon
            # hdg: 0–36000 cdeg (0.01 deg), 65535=unknown
            if msg.hdg != 65535:
                self.heading = int(msg.hdg / 100) % 360

        elif t == "ATTITUDE":
            self.roll_deg  = math.degrees(msg.roll)
            self.pitch_deg = math.degrees(msg.pitch)
            self.yaw_deg   = math.degrees(msg.yaw) % 360

        elif t == "SYS_STATUS":
            self.voltage_mv  = msg.voltage_battery       # mV
            self.current_ca  = msg.current_battery       # cA
            if msg.battery_remaining >= 0:
                self.battery_pct = msg.battery_remaining

        elif t == "GPS_RAW_INT":
            self.gps_fix  = msg.fix_type
            self.gps_sats = msg.satellites_visible

        elif t == "SERVO_OUTPUT_RAW":
            self.servo1_raw = msg.servo1_raw   # CH1 steering
            self.servo3_raw = msg.servo3_raw   # CH3 left throttle
            self.servo4_raw = msg.servo4_raw   # CH4 right throttle

        elif t == "COMMAND_ACK":
            if msg.command == 410:
                if msg.result == 0 and self._pending_control_request:
                    self.control_authority = "granted"
                    logger.info("Operator control GRANTED (sysid=%s)", self.source_system)
                elif msg.result == 0 and not self._pending_control_request:
                    self.control_authority = "none"
                    logger.info("Operator control released (ACK OK)")
                else:
                    self.control_authority = "denied"
                    logger.warning("Operator control DENIED (result=%s)", msg.result)

