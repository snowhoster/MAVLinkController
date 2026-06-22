# ⚓ USV Remote Controller

手持式無人水面載具（USV）遠端遙控器，基於 **Arduino UNO Q** 雙核心平台開發。
操作者透過可變電阻方向盤、左右油門推桿與引擎開關控制無人船，並透過 2.8 吋彩色 LCD 即時監看船舶狀態。

---

## 目錄

- [系統概覽](#系統概覽)
- [硬體需求](#硬體需求)
- [接線圖](#接線圖)
- [系統架構](#系統架構)
- [資料流說明](#資料流說明)
- [MAVLink 協議細節](#mavlink-協議細節)
- [LCD 顯示畫面](#lcd-顯示畫面)
- [Failsafe 安全機制](#failsafe-安全機制)
- [專案結構](#專案結構)
- [相依套件](#相依套件)
- [安裝與部署](#安裝與部署)
- [設定說明](#設定說明)
- [開發備注](#開發備注)

---

## 系統概覽

```
┌─────────────────────────────────────┐
│         USV Remote Controller       │
│            Arduino UNO Q            │
│                                     │
│  ┌──────────────┐  ┌──────────────┐ │
│  │  M4 (Zephyr) │  │ Linux (MPU)  │ │
│  │              │  │              │ │
│  │  ADC 讀取    │  │  Python App  │ │
│  │  開關偵測    │◄►│  mavlink_    │ │
│  │  LCD 顯示    │  │  handler.py  │ │
│  │              │  │              │ │
│  └──────────────┘  └──────┬───────┘ │
│                            │ WiFi    │
└────────────────────────────┼────────┘
                             │ MAVLink v2 UDP
                             ▼
                    ┌────────────────┐
                    │   USV 無人船   │
                    │  ArduPilot /   │
                    │  自製 MAVLink  │
                    └────────────────┘
```

- **M4 核心（MCU）**：執行 Zephyr RTOS，負責所有硬體 I/O（ADC、GPIO、SPI LCD）
- **Linux 核心（MPU）**：執行 Python App，負責 MAVLink v2 WiFi 通訊
- **MCU ↔ MPU 通訊**：透過 `Arduino_RouterBridge`（RPClite/MsgPack 協議）
- **MPU ↔ USV 通訊**：透過 WiFi UDP，MAVLink v2 標準協議

---

## 硬體需求

| 元件 | 規格 / 型號 | 數量 |
|------|------------|:---:|
| 主控板 | Arduino UNO Q | 1 |
| 方向盤可變電阻 | 10kΩ 旋轉電位計 | 1 |
| 左油門可變電阻 | 10kΩ 線性滑動電位計 | 1 |
| 右油門可變電阻 | 10kΩ 線性滑動電位計 | 1 |
| 引擎開關 | SPST 撥動開關（常開） | 1 |
| 彩色 LCD | 2.8 吋 ILI9341 SPI（320×240） | 1 |

---

## 接線圖

### 類比輸入（可變電阻）

可變電阻三腳接法（以方向盤為例，左右油門相同）：

```
3.3V ──┬── [電位計 Pin 1]
       │
       └── [電位計 Pin 2] ──→ Arduino A0 (方向盤)
                              Arduino A1 (左油門)
                              Arduino A2 (右油門)
       ┌── [電位計 Pin 3]
       │
GND  ──┘
```

### 引擎開關

```
Arduino D2 ──[開關]── GND
（使用 INPUT_PULLUP，按下時 LOW，鬆開時 HIGH）
```

### ILI9341 LCD（SPI）

| LCD 腳位 | Arduino 腳位 | 說明 |
|---------|-------------|------|
| VCC     | 3.3V        | 電源 |
| GND     | GND         | 接地 |
| CS      | D10         | Chip Select |
| DC/RS   | D9          | Data/Command |
| RST     | D8          | Reset |
| MOSI    | D11 (MOSI)  | SPI 資料 |
| SCK     | D13 (SCK)   | SPI 時脈 |
| LED     | 3.3V（或 PWM 調光） | 背光 |

> **注意**：ILI9341 為 3.3V 邏輯，與 Arduino UNO Q 相容，無需電位轉換器。

---

## 系統架構

### 雙核心分工

```
┌─────────────────────────────────────────────────────┐
│                   Arduino UNO Q                      │
│                                                       │
│  ┌─────────────────────┐   ┌───────────────────────┐ │
│  │    M4 (sketch.ino)  │   │   Linux (main.py)     │ │
│  │                     │   │                       │ │
│  │  • analogRead(A0)   │   │  • Arduino_RouterBridge│ │
│  │    方向盤 → steering │   │    on_inputs() 接收   │ │
│  │                     │   │                       │ │
│  │  • analogRead(A1)   │   │  • mavlink_handler.py │ │
│  │    左油門 → left_thr│   │    RC_Override 發送   │ │
│  │                     │   │    HEARTBEAT 發送     │ │
│  │  • analogRead(A2)   │   │    ARM/DISARM 指令    │ │
│  │    右油門→right_thr │   │                       │ │
│  │                     │   │  • loop() 5Hz         │ │
│  │  • digitalRead(D2)  │   │    Bridge.call        │ │
│  │    引擎開關偵測邊緣  │   │    ("update_display") │ │
│  │                     │   │                       │ │
│  │  • ILI9341 LCD 顯示 │   │  • Failsafe 執行緒    │ │
│  │    5Hz 刷新畫面     │   │    1秒無指令→歸零     │ │
│  │                     │   │                       │ │
│  └──────────┬──────────┘   └──────────┬────────────┘ │
│             │  Arduino_RouterBridge    │              │
│             └──────────────────────────┘              │
└─────────────────────────────────────────────────────┘
```

### RouterBridge 通訊協議

MCU 與 MPU 之間透過 `Arduino_RouterBridge`（基於 RPClite + MsgPack）進行雙向 RPC 通訊：

| 方向 | 方法名稱 | 類型 | 頻率 | 內容 |
|------|---------|------|:----:|------|
| MCU → MPU | `on_inputs` | `notify`（單向） | 10 Hz + 邊緣觸發 | steering, left_thr, right_thr, sw_state, sw_edge |
| MPU → MCU | `update_display` | `call`（雙向 RPC） | 5 Hz | speed_x10, heading, bat, gps_fix, gps_sats, armed, mode, lq |

**sw_edge 代碼：**

| 值 | 意義 | 觸發動作 |
|----|------|---------|
| 0 | 無邊緣 | — |
| 1 | 上升沿（OFF→ON） | ARM + 切換 MANUAL 模式 |
| 2 | 下降沿（ON→OFF） | DISARM |

---

## 資料流說明

### 控制端 → USV（下行）

```
可變電阻 ADC
  │
  ▼ 50Hz (sketch loop)
M4 analogRead()
  │ map(0–1023 → 範圍)
  ▼
steering  : -1000 … +1000
left_thr  :  1000 … 2000 μs
right_thr :  1000 … 2000 μs
  │
  ▼ 10Hz Bridge.notify("on_inputs")
Python on_inputs()
  │
  ├─ send_rc_override()
  │    CH1 = 1500 + steering/2  (1000–2000 μs)
  │    CH3 = left_thr           (1000–2000 μs)
  │    CH4 = right_thr          (1000–2000 μs)
  │
  ▼ RC_CHANNELS_OVERRIDE #70 via WiFi UDP
USV 接收 → 左右推進器 PWM 輸出
```

### USV → 控制端（上行遙測）

```
USV MAVLink 發送
  │
  ▼ WiFi UDP
Python _rx_loop()
  │
  ├─ HEARTBEAT  #0  → armed, custom_mode
  ├─ VFR_HUD    #74 → speed_knots, heading, altitude, throttle
  ├─ GLOBAL_POSITION_INT #33 → lat, lon, alt, heading(cdeg)
  ├─ ATTITUDE   #30 → roll_deg, pitch_deg, yaw_deg
  ├─ SYS_STATUS #1  → voltage_mv, current_ca, battery_pct
  ├─ GPS_RAW_INT #24 → gps_fix, gps_sats
  └─ SERVO_OUTPUT_RAW #36 → servo1/3/4 raw PWM
  │
  ▼ 5Hz Bridge.call("update_display", ...)
M4 update_display() → 更新 g_vessel
  │
  ▼ 5Hz lcd_update_dynamic()
ILI9341 LCD 顯示刷新
```

---

## MAVLink 協議細節

### 版本與連線

- **協議版本**：MAVLink v2（STX = `0xFD`）
- **傳輸層**：WiFi UDP
- **GCS SystemId**：255，ComponentId：190
- **USV SystemId**：1，ComponentId：1（預設，可設定）

### 發送訊息

#### `RC_CHANNELS_OVERRIDE` (Message ID: 70, CRC Extra: 124)

控制指令，10 Hz 發送：

| 通道 | 功能 | 數值範圍 | 說明 |
|------|------|---------|------|
| CH1 | 方向舵 / 轉向 | 1000–2000 μs | 中立 = 1500，右轉 > 1500，左轉 < 1500 |
| CH2 | 未使用 | 65535 | 65535 = 無覆蓋 |
| CH3 | 左推進器油門 | 1000–2000 μs | 中立 = 1500，前進 > 1500 |
| CH4 | 右推進器油門 | 1000–2000 μs | 中立 = 1500，前進 > 1500 |
| CH5–18 | 未使用 | 65535 | 65535 = 無覆蓋 |

**方向盤映射公式：**
```
CH1_μs = 1500 + (steering_adc_val / 2)
steering_adc_val ∈ [-1000, +1000]  →  CH1 ∈ [1000, 2000]
```

#### `COMMAND_LONG` (Message ID: 76, CRC Extra: 152)

**ARM（引擎啟動）：**
```
cmd = 400 (MAV_CMD_COMPONENT_ARM_DISARM)
param1 = 1.0  (ARM)
```

**DISARM（引擎停止）：**
```
cmd = 400 (MAV_CMD_COMPONENT_ARM_DISARM)
param1 = 0.0  (DISARM)
```

**切換模式：**
```
cmd = 176 (MAV_CMD_DO_SET_MODE)
param1 = 1.0  (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
param2 = custom_mode
  0  = MANUAL（手動）
  11 = RTL（自動返航）
  15 = GUIDED（引導模式）
```

#### `HEARTBEAT` (Message ID: 0, CRC Extra: 50)

GCS 心跳，1 Hz 發送，宣告遙控端存在：
```
type      = MAV_TYPE_GCS (6)
autopilot = MAV_AUTOPILOT_INVALID (8)
```

### 接收訊息

| 訊息名稱 | ID | 解析欄位 | 用途 |
|---------|:--:|---------|------|
| HEARTBEAT | 0 | `base_mode & 0x80` = armed, `custom_mode` | ARM 狀態、飛控模式 |
| VFR_HUD | 74 | `groundspeed`→kn, `heading`, `alt`, `climb`, `throttle` | 速度、航向、高度 |
| GLOBAL_POSITION_INT | 33 | `lat`, `lon`, `alt`, `hdg`(cdeg) | GPS 座標、精確航向 |
| ATTITUDE | 30 | `roll`, `pitch`, `yaw`(rad) | 姿態（角度） |
| SYS_STATUS | 1 | `voltage_battery`(mV), `current_battery`(cA), `battery_remaining`(%) | 電源狀態 |
| GPS_RAW_INT | 24 | `fix_type`, `satellites_visible` | GPS 鎖定狀態 |
| SERVO_OUTPUT_RAW | 36 | `servo1_raw`, `servo3_raw`, `servo4_raw` | 當前舵機輸出 PWM |

---

## LCD 顯示畫面

顯示器為 **ILI9341 橫向 320×240 像素**，畫面分為六個區域：

```
┌─────────────────────────────────────┐ ← y=0
│      USV REMOTE CONTROLLER          │  標題列（深藍背景）
├──────────────┬──────────────────────┤ ← y=20
│ SPD  5.2 kn  │ GPS  3D FIX  8SAT   │
│ HDG  045 deg │ MODE MANUAL         │
│ BAT  ████░░72%│ LINK ████████ 100%  │
├──────────────┴──────────────────────┤ ← y=82
│ L-THR ███████░░░░ 60%               │
│                 R-THR ████████░░ 65%│
├─────────────────────────────────────┤ ← y=122
│ HELM  ──────────●────────────  +45  │ ← y=126
├─────────────────────────────────────┤ ← y=142
│                                     │
│         ENGINE                      │
│    ┌──────────────────────┐         │
│    │     [ ARMED ]        │ ← 綠色  │
│    │   [ DISARMED ]       │ ← 紅色  │
│    └──────────────────────┘         │
└─────────────────────────────────────┘ ← y=240
```

### 色彩規範

| 元素 | 正常 | 警告 | 危險 |
|------|:----:|:----:|:----:|
| 電量 Bar | 🟢 綠（>50%） | 🟡 黃（20–50%） | 🔴 紅（<20%） |
| 連線品質 Bar | 🟢 綠（>70%） | 🟡 黃（30–70%） | 🔴 紅（<30%） |
| GPS 狀態 | 🟢 綠（3D FIX） | 🟡 黃（2D FIX） | 🔴 紅（NO FIX） |
| ARM 徽章 | 🟢 綠底黑字 ARMED | — | 🔴 紅底白字 DISARMED |
| HELM Bar | 🔵 青色（左右偏移） | — | — |
| 油門 Bar | 🟠 橘色 | — | — |

---

## Failsafe 安全機制

本系統實作雙層 Failsafe：

### 層 1：GCS 端控制逾時（Python 端）

- **觸發條件**：`send_rc_override()` 超過 **1 秒** 未被呼叫（例如 RouterBridge 斷線）
- **動作**：自動發送 CH1=1500、CH3=1500、CH4=1500（全部中立歸零）
- **實作**：`mavlink_handler.py` 中的 `_failsafe_loop` 執行緒，每 200ms 檢查一次

### 層 2：USV 端控制逾時（船端）

- **觸發條件**：USV 的 C# MavLinkDriver 超過設定的 `ControlFailsafeTimeoutMs` 未收到控制封包
- **動作**：船端自動發送 CH1=1500、CH3=1500、CH4=1500 以停止推進器
- **效果**：即使控制端整個斷電，船也會自動停止

```
正常操作                  GCS 端 Failsafe              USV 端 Failsafe
─────────                ────────────────             ─────────────────
MCU 傳送輸入              MCU/Bridge 故障              WiFi 斷線
  │ 10Hz                   │ > 1s 無控制               │ > ControlFailsafe
  ▼                         ▼                            ▼
Python 接收              Python 自動發送              USV 自動歸零
send_rc_override()       1500/1500/1500              推進器停止
```

---

## 專案結構

```
usv_controller/
│
├── app.yaml                    # Arduino App 主設定（名稱、圖示）
│
├── sketch/
│   ├── sketch.yaml             # MCU 編譯設定（fqbn、函式庫清單）
│   ├── sketch.ino              # M4 主程式：ADC 讀取、開關偵測、LCD 驅動、RouterBridge
│   └── lcd_display.h           # ILI9341 LCD 畫面繪製函式庫
│
├── python/
│   ├── main.py                 # MPU 主程式：RouterBridge 整合、主迴圈
│   └── mavlink_handler.py      # MAVLink v2 收發封裝（pymavlink）
│
└── venv/                       # Python 虛擬環境
```

### 各檔案職責

#### `sketch/sketch.ino`
- 使用 `analogRead()` 讀取三路可變電阻 ADC（50Hz）
- 偵測引擎開關邊緣（上升/下降）
- 以 `Bridge.notify("on_inputs", ...)` 推送輸入到 MPU（10Hz，邊緣時立即觸發）
- 提供 `update_display()` RPC 方法供 MPU 呼叫以更新 LCD
- 以 5Hz 刷新 ILI9341 LCD 畫面

#### `sketch/lcd_display.h`
- 定義 `VesselStatus` 結構體
- `lcd_init()` / `lcd_draw_static()`：初始化及繪製靜態框架
- `lcd_update_dynamic()`：更新動態資料區（速度、航向、電量、GPS、油門 Bar、HELM Bar、ARM 徽章）
- RGB565 色彩定義與 Bar 繪製工具函式

#### `python/main.py`
- 以 `Bridge.provide("on_inputs", on_inputs)` 接收 MCU 輸入
- `on_inputs()` 呼叫 `mavlink.send_rc_override()` 轉發控制指令
- 開關邊緣觸發 `mavlink.send_arm()` / `mavlink.send_disarm()`
- `loop()` 每 200ms 呼叫 `Bridge.call("update_display", ...)` 推送船舶狀態到 LCD

#### `python/mavlink_handler.py`
- 連線管理：`connect()` / `disconnect()`
- 控制發送：`send_rc_override()` / `send_arm()` / `send_disarm()` / `send_safe_state()`
- 遙測接收：背景執行緒解析 HEARTBEAT / VFR_HUD / ATTITUDE / SYS_STATUS / GPS_RAW_INT 等
- Failsafe：`_failsafe_loop()` 自動送出中立指令

---

## 相依套件

### MCU 端（sketch.yaml）

| 函式庫 | 版本 | 用途 |
|--------|------|------|
| Arduino_RPClite | 0.2.1 | RouterBridge 底層 RPC |
| MsgPack | 0.4.2 | RouterBridge 訊息序列化 |
| DebugLog | 0.8.4 | RouterBridge 除錯日誌 |
| ArxContainer | 0.7.0 | RouterBridge 依賴容器 |
| ArxTypeTraits | 0.3.1 | RouterBridge 依賴型別特性 |
| Adafruit ILI9341 | 1.6.3 | LCD 驅動 |
| Adafruit GFX Library | 1.12.6 | LCD 圖形基礎 |
| Adafruit BusIO | 1.17.4 | SPI/I2C 抽象層 |

> `Arduino_RouterBridge` 由 `arduino:zephyr` 平台內建提供，無需在 sketch.yaml 中宣告。

### MPU 端（Python）

```bash
pip install pymavlink pyserial
```

| 套件 | 用途 |
|------|------|
| pymavlink | MAVLink v2 協議封裝（訊息編碼/解碼） |
| arduino-app-utils | Arduino App 框架（RouterBridge Python 端） |

---

## 安裝與部署

### 1. 安裝 Python 依賴

```bash
cd usv_controller
source venv/bin/activate
pip install pymavlink pyserial
```

### 2. 修改 USV 連線設定

編輯 `python/main.py` 第 22 行，填入 USV 的 WiFi IP 與 MAVLink 監聽埠：

```python
USV_CONNECTION_STR = "udpout:192.168.1.100:14550"
#                           ↑ USV 的 IP    ↑ USV MAVLink UDP Port
```

### 3. 部署 App

使用 Arduino App Lab CLI 或 IDE 部署：

```bash
arduino-app deploy
```

此指令會：
1. 編譯並燒錄 `sketch.ino` 到 M4 核心
2. 將 Python 程式部署到 Linux 核心並執行

### 4. 驗證運作

App 啟動後，LCD 應顯示靜態框架。當 WiFi 連線到 USV 後：
- 標題列保持顯示
- LINK 欄位應由紅變綠
- USV 發送 HEARTBEAT 後，ARM 徽章會更新狀態

---

## 設定說明

### USV 端 MAVLink 設定對應

本控制器與 USV 的 `MavLinkConfig` 對應關係如下：

| 控制器設定 | USV 端設定 | 說明 |
|-----------|-----------|------|
| `source_system=255` | GCS SystemId | 地面站固定為 255 |
| `source_component=190` | GCS ComponentId | 地面站固定為 190 |
| `target_system=1` | `TargetSystemId` | USV 預設 SystemId |
| `target_component=1` | `TargetComponentId` | USV 預設 ComponentId |
| UDP 發送至 USV IP:Port | `UdpConfig.LocalPort` | USV 監聽的 UDP Port |

### MAVLink Source/Target 自訂

如需修改 SystemId 等設定，可在 `python/main.py` 初始化時傳入：

```python
mavlink = MAVLinkHandler(
    connection_str   = "udpout:192.168.1.100:14550",
    source_system    = 255,   # GCS SystemId
    source_component = 190,   # GCS ComponentId
    target_system    = 1,     # USV SystemId
    target_component = 1,     # USV ComponentId
)
```

### Failsafe 逾時調整

於 `python/mavlink_handler.py` 頂部修改：

```python
CONTROL_FAILSAFE_S = 1.0   # 秒，控制信號中斷後自動歸零的等待時間
```

---

## 開發備注

### 核心通訊時序

| 任務 | 頻率 | 執行端 |
|------|:----:|--------|
| ADC 讀取 | 50 Hz | M4 |
| RouterBridge notify (inputs) | 10 Hz | M4 → MPU |
| MAVLink RC_Override 發送 | 10 Hz | MPU → USV |
| MAVLink HEARTBEAT 發送 | 1 Hz | MPU → USV |
| RouterBridge call (display) | 5 Hz | MPU → M4 |
| LCD 刷新 | 5 Hz | M4 |
| Failsafe 檢查 | 5 Hz | MPU |

### ADC 值映射對照

| 硬體輸入 | ADC 原始值 | 映射後 | 用途 |
|---------|:---------:|:------:|------|
| 方向盤（A0） | 0–1023 | -1000 … +1000 | RouterBridge `steering` 參數 |
| 方向盤 | -1000 … +1000 | 1000–2000 μs | MAVLink CH1（`1500 + val/2`） |
| 左油門（A1） | 0–1023 | 1000–2000 μs | RouterBridge `left_thr` / MAVLink CH3 |
| 右油門（A2） | 0–1023 | 1000–2000 μs | RouterBridge `right_thr` / MAVLink CH4 |

### ArduPilot Rover 模式代碼

| 代碼 | 模式 | 說明 |
|:----:|------|------|
| 0 | MANUAL | 完全手動，直接傳遞 RC 輸入 |
| 4 | HOLD | 原地保持 |
| 10 | AUTO | 自動任務 |
| 11 | RTL | 自動返回起點 |
| 15 | GUIDED | 引導模式（接受外部航點） |

### 電源狀態欄位單位

| 欄位 | 來源訊息 | 單位 | 換算 |
|------|---------|:----:|------|
| `voltage_mv` | SYS_STATUS | mV | ÷1000 = V |
| `current_ca` | SYS_STATUS | cA（釐安） | ÷100 = A |
| `battery_pct` | SYS_STATUS | % | 直接使用 |
