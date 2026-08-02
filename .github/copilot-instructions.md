# Copilot Instructions — USV Remote Controller

## App Lifecycle

```bash
arduino-app-cli app start   ~/ArduinoApps/usv_controller   # deploys + runs (stops any running app first)
arduino-app-cli app stop    ~/ArduinoApps/usv_controller
arduino-app-cli app restart ~/ArduinoApps/usv_controller
arduino-app-cli app logs    ~/ArduinoApps/usv_controller --follow   # Python stdout
arduino-app-cli monitor                                               # MCU Serial.print output
arduino-app-cli app clean-cache user:usv_controller --force          # clear stale sketch build
```

Python dependencies:
```bash
cd python && pip install pymavlink
```

## Architecture

This is an **Arduino UNO Q dual-core app**:

- **MCU (M4/Zephyr, `sketch/sketch.ino`)** — reads ADC inputs (steering A0, left throttle A1, right throttle A2), detects engine switch edges on D2 (INPUT_PULLUP, active-low), drives the ILI9341 LCD via SPI at 5 Hz, controls LED3 (PWM) and LED4 (digital).
- **MPU (Linux, `python/main.py`)** — receives MCU inputs via RouterBridge, forwards them as MAVLink RC_CHANNELS_OVERRIDE to the USV over WiFi UDP, pushes telemetry back to the MCU LCD display at 5 Hz.
- **`python/mavlink_handler.py`** — all MAVLink v2 logic: three background threads (`mav-rx`, `mav-hb`, `mav-ctrl`), failsafe, operator-control-authority state machine.
- **`sketch/lcd_display.h`** — all ILI9341 drawing code; MCU imports this header.
- **`python/usv_config.json`** — persists connection settings (protocol, remote_ip, remote_port, local_port); written by `_save_config()` on every Web UI reconnect.
- **`assets/`** — Web UI (HTML/JS/CSS) served by the `arduino:web_ui` brick on port 7000.

### Communication timing

| Link | Direction | Rate |
|------|-----------|------|
| `Bridge.notify("on_inputs", ...)` | MCU → MPU | 10 Hz + edge-triggered |
| `Bridge.call("update_display", ...)` | MPU → MCU | 5 Hz |
| `Bridge.call("set_led3_color", ...)` | MPU → MCU | on change |
| `Bridge.call("set_led4_color", ...)` | MPU → MCU | on change |
| MAVLink RC_CHANNELS_OVERRIDE | MPU → USV | 10 Hz (by `_control_loop`) |
| MAVLink HEARTBEAT | MPU → USV | 1 Hz |
| MAVLink telemetry (RX) | USV → MPU | async |

## Key Conventions

### RouterBridge name contract
Bridge method names **must match exactly** between MCU `Bridge.provide("name", fn)` and MPU `Bridge.call/notify("name", ...)`. A mismatch on `notify` fails silently; on `call` it raises. The names in use are: `on_inputs`, `update_display`, `set_led3_color`, `set_led4_color`.

### String parameters in RPClite
When an MCU function receives `String` arguments from the MPU, the parameter type **must be `String`** (not `const char*`). RPClite's MsgPack unpacker can only deserialize into a `String&`. See `update_display()` in `sketch.ino` for the pattern.

### Shared constants — keep both sides in sync
The following integer codes are defined in both the sketch and Python and **must match**:

| Constant | Sketch (`sketch.ino`) | Python (`main.py`) |
|----------|----------------------|-------------------|
| Switch edges | `EDGE_NONE=0`, `EDGE_RISING=1`, `EDGE_FALLING=2` | `EDGE_RISING=1`, `EDGE_FALLING=2` |
| Control authority | `AUTH_NONE=0`, `AUTH_PENDING=1`, `AUTH_GRANTED=2`, `AUTH_DENIED=3` | `_AUTH_CODE = {"none":0,"pending":1,"granted":2,"denied":3}` |

### MAVLink v2 — force before import
`os.environ['MAVLINK20'] = '1'` must be set **before** `from pymavlink import mavutil`. It's set at module level in `mavlink_handler.py`.

### RC override: cache then re-send
`send_rc_override()` only **caches** CH1/CH3/CH4 values and updates `_last_control_time`. The actual UDP sending is done by `_control_loop` at 10 Hz — mirroring the C# reference's `SendTimer_Tick`. Never call `rc_channels_override_send` directly outside `_control_loop` or `send_safe_state`.

### Unused RC channels = 65535
In all `rc_channels_override_send` calls, unused channels must be set to `65535` (meaning "no override"), not `0`.

### ARM is always async
`send_arm()` spawns a thread (`mav-arm`) to avoid blocking the Bridge callback. It also sends `SET_MODE(MANUAL)` 150 ms before ARM — do not collapse these into a single synchronous call.

### UDP socket mode
When `local_port` is non-zero, `MAVLinkHandler` opens `udpin:0.0.0.0:{local_port}` and pre-adds the remote address to `self._conn.clients` so outbound packets work before the first incoming packet. This is required because the USV always sends telemetry to a fixed local port.

### LED split: Python vs MCU
- **LED1, LED2** — controlled by Python `Leds.set_led1_color(r,g,b)` / `Leds.set_led2_color(r,g,b)` (bool R/G/B).
- **LED3** — MCU-controlled, Python calls `Bridge.call("set_led3_color", r, g, b)` with PWM ints 0–255.
- **LED4** — MCU-controlled, Python calls `Bridge.call("set_led4_color", r, g, b)` with bool values.
All four LEDs are **active-low** on the hardware.

### ADC mapping
- Steering (A0): `map(0–1023 → -1000…+1000)` → sent as-is over Bridge; MAVLink CH1 = `1500 + steering/2`
- Throttles (A1, A2): `map(0–1023 → 1000…2000 μs)` → sent directly as MAVLink CH3/CH4

### Web UI messages
The Web UI communicates via WebSocket messages registered with `ui.on_message(type, handler)`. Outbound state is pushed as `ui.send_message("state", {...})` at 5 Hz. Inbound message types: `web_ctrl_set`, `control`, `arm`, `set_led`, `set_mode`, `connect_usv`, `acquire_control`, `release_control`, `set_gcs_sysid`.

### MAVLink IDs (fixed)
- GCS: `source_system=255`, `source_component=190`
- USV: `target_system=1`, `target_component=1` (defaults; overridable at init)
- `MAV_CMD_REQUEST_OPERATOR_CONTROL` = cmd 410 (not in standard pymavlink enum — used as a raw integer)
