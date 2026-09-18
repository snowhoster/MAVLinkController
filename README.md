# ⚓ USV Remote Controller

手持式無人水面載具（USV）遠端遙控器，基於 **Arduino UNO Q** 雙核心平台開發。
操作者透過可變電阻方向盤、左右油門推桿與引擎開關控制無人船，並透過 3.2 吋彩色 LCD 即時監看船舶狀態。

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
| 主控板 | Arduino UNO Q（雙核 STM32U585 M4 ＋ Linux MPU） | 1 |
| **雙操舵旋鈕** | **WH148 10kΩ 線性電位計 ＋ Φ15mm×H13mm 調音台防滑旋鈕帽**（左旋鈕 ＆ 右旋鈕各 1 組，3.3V 供電，6mm花軸緊配合） | **2** |
| **操舵旋鈕切換開關** | **MTS-102 SPDT 兩段撥動開關**（撥上啟用左操舵旋鈕，撥下啟用右操舵旋鈕，零延遲硬體選通至 A0） | **1** |
| **油門推桿控制器** | **APEM SN 系列工業級 T-Bar**（非接觸霍爾效應、PTFE+鋼珠滾道、鋁合金子彈握把、12-bit、0~5V 類比輸出） | **2**（左油門、右油門各 1） |
| **精密分壓與濾波** | **10kΩ ＋ 20kΩ 1% 金屬膜精密電阻** ＋ 100nF 陶瓷電容（5V 轉 3.33V 保護 STM32 ADC） | **2 組**（保護 A1 與 A2） |
| 引擎搖頭開關 | MTS-123 SPDT (ON)-OFF-(ON) 雙向瞬時自復位（撥上啟動、撥下關閉、自動回中） | 1 |
| 控制權按鈕 | GQ16H-10 16mm 金屬平頭瞬時按鈕（1NO 常開） | 1 |
| **緊急停止按鈕** | **LA16-11ZS 16mm 蘑菇頭自鎖式，具 NC 常閉接點（Fail-Safe）** | **1** |
| 模式開關 | MTS-102 SPDT 兩段撥動開關（自保持，撥上定向 / 撥下手動） | 1 |
| 彩色 LCD | 米羅科技 Mirotek #400300（3.2 吋 ILI9341 SPI 320×240 彩色螢幕帶觸控，PCB 55×89.3mm） | 1 |
| **外置不斷電電源** | **雙節 18650 UPS 獨立電池倉**（外殼底槽專屬獨立固定，不佔洞洞板面積，Type-C 直插 UNO Q，5V 3A 邊充邊用） | 1 |
| 主機載板 | 40孔 × 24孔 雙面噴錫玻纖萬用洞洞板（標準 2.54mm 孔距，提供每個洞正面插裝 ＆ 背面仰視鏡像焊接規劃） | 1 |

---

## 硬體配線、萬用洞洞板施工圖與全套分解圖解 SOP

本專案提供高度專業化的向量硬體工程手冊與施工作業指南，包含互動式 HTML 圖面與已排版完成之 A4 彩色工程 PDF：
- **HTML 互動圖面與分解圖解**：[`docs/hardware_diagram.html`](docs/hardware_diagram.html)
- **A4 高解析工程手冊 PDF**：[`docs/hardware_diagram.pdf`](docs/hardware_diagram.pdf)

### 施工手冊頁面導覽

| 頁碼 | 頁面名稱 | 內容亮點與工程規範 |
|:---:|---------|------------------|
| **第一頁 (A/B)** | **USV 遙控器外殼與下方電路板空間裝配關係圖 ＆ 硬體配線總覽圖** | **第一頁 (A)**：外殼機構與內部載板空間關係工程剖面圖（正視橫向剖面圖、側視垂直堆疊剖面圖）；**第一頁 (B)**：雙 APEM 霍爾推桿、獨立雙 18650 UPS、Type-C 直插與尼龍蛇皮網管走線總覽 |
| **第二頁** | **萬用洞洞板正面元件插裝與座標佈局圖** | 40列 × 24行標準 2.54mm 俯視圖，各接頭直針公座、精密分壓被動元件、UNO Q 倒扣投影區 |
| **第三頁** | **萬用洞洞板背面實際焊接走線圖** | 仰視鏡像（40Col ← 1Col），消除左右顛倒失誤；Row H 裸銅接地排、Row F 拖焊錫橋、OK線跳線 |
| **第四頁** | **40×24 孔位座標電路明細表** | 逐孔座標定義、正反面鏡像對照表、針腳訊號走向與焊接工法規範 |
| **第五頁** | **實焊四大工法、防短路三級測試與 BOM 清單** | 烙鐵溫度 330~350°C、拖焊錫橋標準、三級三用電表冷態與熱態測試、材料清單 |
| **第六頁** | **實物零件照片圖鑑與台灣採購指南（上篇）** | APEM SN 推桿（含代理/進口商）、Arduino UNO Q、雙節 18650 UPS 載板、松下 NCR18650B 電芯 |
| **第七頁** | **實物零件照片圖鑑與台灣採購指南（下篇）** | 雙面玻纖洞洞板、米羅科技 #400300 3.2" TFT LCD 模組、LA16-11ZS 急停開關、MTS 開關、精密電阻電容與端子線材 |
| **第八頁** | **萬用洞洞板內部元件插裝與焊接工藝分解圖解 SOP** | **SOP-01 ~ SOP-08 向量分解圖**：板材裁切倒角、Row H 裸銅貫穿地排、排母垂直限位、阻容直角彎折、Row F 拖焊錫橋 4 步分解、OK 線防燙剝線與三級測試 |
| **第九頁** | **外部零件面板安裝、機械固定與整機裝配圖解 SOP** | **SOP-09 ~ SOP-15 向量分解圖**：APEM 推桿 70×20mm 銑槽與沉頭固定、WH148 防轉銷定位、急停 NC 迴路、MTS 雙螺母對鎖、3.2" TFT LCD 銅柱避震、底艙 UPS 15cm 短線與蛇皮網束線總裝 |

---

## 外殼頂部操作面板人因工學佈局

```
┌────────────────────────────────────────────────────────────────────────────┐
│ [🚨安全急停]                 [ 3.2 吋彩色 LCD 螢幕 ]                       │
│ (LA16-11ZS)                    (320×240 ILI9341 SPI)                       │
│                                                                            │
│ [🔄舵輪切換]   [ 🎛️ 左操舵旋鈕 ]                        [ 🎛️ 右操舵旋鈕 ]   [防滑握把] │
│ (左舵 / 右舵)   (WH148 B10K)                           (WH148 B10K)   [5V/3.3V]  │
│                                                                       [LED指示]  │
│ [🧭模式切換]                 [ 雙 APEM SN 油門推桿 ]                       │
│ (定向 / 手動)                ┌─────────┐ ┌─────────┐                       │
│                              │ 左油門  │ │ 右油門  │                       │
│ [⚡引擎啟閉]                 │ A1      │ │ A2      │                       │
│ (啟動 / 熄火)                └─────────┘ └─────────┘                       │
│                                                                            │
│ [🎮控制權鍵]                                                               │
│ (短按 / 長按)                                                              │
└────────────────────────────────────────────────────────────────────────────┘
```

---

## 接線圖

### 類比輸入（雙方向盤轉輪 ＆ APEM SN 雙 T-Bar 霍爾推桿）

系統包含三路主要類比輸入，因感測器電氣特性不同採用分立供電架構：

#### 1. 左右雙操舵調音台旋鈕 ＆ 選擇開關（雙 WH148 B10K ＋ MTS-102 — 直供 3.3V，零延遲硬體選通）
系統配置左右兩側對稱之獨立操舵旋鈕（左旋鈕專供左手操舵、右旋鈕專供右手操舵，搭配 Φ15mm×H13mm 調音台防滑直槽旋鈕帽）。兩者共用 3.3V 與 GND 供電，各自的電位計 Wiper 訊號線引至面板左側之 **MTS-102 舵輪選擇開關**，選通後以純硬體方式零延遲直入 Arduino A0：

```
[ 左操舵旋鈕 (Φ15mm調音台旋鈕) ]
3.3V ──┬── [左電位計 Pin 1]
GND  ───── [左電位計 Pin 3]
       │
       └── [左電位計 Pin 2 (Wiper L)] ──┐
                                       ├── [MTS-102 選擇開關] ──┬──→ Arduino A0 (舵角, 0~3.3V)
       ┌── [右電位計 Pin 2 (Wiper R)] ──┘   (撥上:左 / 撥下:右) └──┤├── GND (100nF 濾波電容)
       │
[ 右操舵旋鈕 (Φ15mm調音台旋鈕) ]
3.3V ──┼── [右電位計 Pin 1]
GND  ───── [右電位計 Pin 3]
```

#### 2. 左右油門推桿（雙 APEM SN 系列 T-Bar 霍爾推桿 — 5V 供電 ＋ 獨立 10k:20k 精密分壓）
APEM SN 系列推桿內部霍爾感測晶片需 **5V 獨立供電**（由 UPS 5V 匯流軌統一供電），輸出為 **0~5V 類比線性電壓**。由於 Arduino UNO Q（STM32U585）ADC 類比腳位**絕對不耐 5V（上限 3.6V）**，載板為左油門與右油門各配置一組 $10\text{k}\Omega : 20\text{k}\Omega$ 1% 金屬膜精密分壓與 100nF 濾波電路，將 0~5V 安全等比降至 0~3.33V：

```
[ 左油門 APEM SN T-Bar ]
UPS 5V ────── [左推桿 Pin 1 (紅線)]
GND    ────── [左推桿 Pin 3 (黑線)]

左推桿 Pin 2 (黃線, 0~5V) 
   │
   └───[ 10kΩ 1% (R1) ]───┬───→ Arduino A1 (左油門, 0~3.33V)
                          │
                   [ 20kΩ 1% (R2) ] ＋ [ 100nF 電容 (C1) ]
                          │
                         GND

[ 右油門 APEM SN T-Bar ]
UPS 5V ────── [右推桿 Pin 1 (紅線)]
GND    ────── [右推桿 Pin 3 (黑線)]

右推桿 Pin 2 (黃線, 0~5V) 
   │
   └───[ 10kΩ 1% (R3) ]───┬───→ Arduino A2 (右油門, 0~3.33V)
                          │
                   [ 20kΩ 1% (R4) ] ＋ [ 100nF 電容 (C2) ]
                          │
                         GND
```

$$V_{ADC} = V_{OUT} \times \frac{20\text{k}\Omega}{10\text{k}\Omega + 20\text{k}\Omega} = V_{OUT} \times \frac{2}{3} = 0 \sim 3.333\text{V}$$

- **負載阻抗相容性**：分壓總阻抗 $R1 + R2 = 30\text{k}\Omega$，遠高於 APEM 原廠規範的最低負載要求（$\ge 10\text{k}\Omega$），單組耗電僅 0.16mA，確保推桿全行程極致線性度。
- **RC 低通濾波**：並聯的 100nF 陶瓷電容構成截止頻率約 $238\text{ Hz}$ 之低通濾波器，徹底濾除馬達電磁干擾（EMI）與長線雜訊。
- **差動雙推進獨立控制**：韌體獨立讀取 A1（左油門）與 A2（右油門），分別映射為 1000~2000 μs PWM 訊號，經 MAVLink CH3 與 CH4 獨立發送，實現無人船雙槳差動與原地迴旋操控。

### 數位輸入（開關與按鈕）

| 功能 | 腳位 | MCU | 元件 | 接法 | 判定 |
|------|:----:|:---:|------|------|------|
| 引擎啟動 (Start) | A3 | PA7 | 搖頭開關往上接點 | `INPUT_PULLUP` → GND | 撥上 = LOW (啟動 / ARM)，放開回中 |
| 引擎關閉 (Stop) | A4 | PC1 | 搖頭開關往下接點 | `INPUT_PULLUP` → GND | 撥下 = LOW (關閉 / DISARM)，放開回中 |
| 控制權 | A5 | PC0 | 瞬時按鈕 | `INPUT_PULLUP` → GND | 按下 = LOW |
| 緊急停止 | D20 | PB11 | 蘑菇頭自鎖式 | `INPUT_PULLUP` → **NC 常閉接點** → GND | 拍下 = HIGH |
| 模式 | D21 | PB10 | SPDT 兩段撥動開關 | `INPUT_PULLUP` → GND | 撥上(LOW) = 定向 |

```
Arduino A3  ──[搖頭開關-往上接點]── GND  (撥上啟動，放開彈回中間)
Arduino A4  ──[搖頭開關-往下接點]── GND  (撥下關閉，放開彈回中間)
Arduino A5  ──[控制權按鈕]────────── GND
Arduino D20 ──[E-STOP NC接點]─────── GND
Arduino D21 ──[模式開關]──────────── GND
```

> **為什麼不用 D2–D6？**
>
> UNO 外形的 `D7`↔`D8` 間距是 0.16"（4.06 mm），落不到萬用板的 0.1" 孔上。若把
> 開關接在 D2–D6，做擴充板時整排 D0–D7 排針要彎 1.02 mm 才插得進去。改用
> `A3`–`A5` 與 `D20`/`D21` 之後，擴充板只需焊 **10P（D8–D21）／8P（電源）／
> 6P（類比）** 三排，全部正落在孔位，完全不必彎腳。
>
> `D20`/`D21` 是 I2C2 的 SDA/SCL，當一般 GPIO 使用，因此 **sketch 不可呼叫
> `Wire.begin()`**。Qwiic 走的是 I2C4（PD12/PD13），不受影響。
>
> 另一個附帶好處：`~D3`（PB0）是 TT 型 I/O，**只耐 3.6 V**，任何模式都不可接
> 5 V。把右引擎開關移離這隻腳，少一個踩雷點。

> **⚠ 緊急停止必須使用 NC 常閉接點**
>
> 正常狀態下 NC 接點導通，D20 被拉至 LOW；拍下按鈕時接點斷開，`INPUT_PULLUP` 將
> D20 拉至 HIGH，觸發急停。這代表**線材脫落、接點氧化、接頭鬆脫也會觸發急停**，
> 符合 fail-safe 原則。
>
> 若誤用 NO 常開接點，斷線將導致急停功能靜默失效——按下去沒有任何反應，且無從
> 察覺。這是安全設計上不可妥協的一點。

所有數位輸入皆經過 **25 ms 軟體消抖**（`sketch.ino` 的 `DebouncedInput`），
避免機械接點彈跳造成重複的 ARM/DISARM 指令。

### ILI9341 LCD（SPI）

| LCD 腳位 | Arduino 腳位 | 說明 |
|---------|-------------|------|
| VCC     | 3.3V        | 電源 |
| GND     | GND         | 接地 |
| CS      | D10         | Chip Select |
| DC/RS   | D8          | Data/Command |
| RST     | D9          | Reset |
| MOSI    | D11 (MOSI)  | SPI 資料 |
| SCK     | D13 (SCK)   | SPI 時脈 |
| LED     | 3.3V（或 PWM 調光） | 背光 |

> **注意**：ILI9341 為 3.3V 邏輯，與 Arduino UNO Q 相容，無需電位轉換器。
> **推薦指定型號**：[米羅科技 Mirotek #400300 3.2吋 TFT LCD 液晶螢幕模組](https://shop.mirotek.com.tw/shop/400300/)（ILI9341 320×240 帶 XPT2046 觸控，模組 PCB 尺寸 55.04×89.30 mm，標準 14-Pin J2 排針前 8 腳直插載板 CON 1）。

### 硬體機構與接線架構（方案 A：單主板集中插座 ＋ 帶線母頭線束）

詳細互動式架構圖請參見：[docs/hardware_diagram.html](file:///E:/Source/MIS/2026/MAVLinkController/docs/hardware_diagram.html) ｜ PDF 列印圖紙：[docs/hardware_diagram.pdf](file:///E:/Source/MIS/2026/MAVLinkController/docs/hardware_diagram.pdf)

1. **外殼操作面板（零小板極簡化設計）**：
   - 頂部面板背面完全不需固定分線小板，節省垂直厚度與加工螺孔。
   - 所有元件腳位直焊市售現成的 **「JST-XH 2.54mm 單頭帶線（20cm）」**，引腳套 2mm 熱縮套管絕緣。
   - 7 組控制線穿入一條 **直徑 8mm 黑色尼龍編織網管（蛇皮網）** 束整，耐磨抗折。
   - 母頭具備**彈性防脫卡扣（卡彈）**，下垂直接對插主板。
2. **主機載板（Carrier Motherboard，40×24 孔萬用洞洞板，尺寸約 10×6cm）**：
   - **外置 18650 UPS 獨立電池倉**：電池座獨立安裝於機殼底槽（不佔用洞洞板面積），利用 15cm 軟質 Type-C 短線直插 Arduino UNO Q Type-C 供電口；全板 5V 電源由 UNO Q Power 8P 的 `Pin 5 (5V)` 引出至洞洞板 `Row C` 供電軌，專門轉供雙 APEM 霍爾推桿。
   - **Arduino UNO Q 倒扣安裝**：主板面朝下倒扣在洞洞板焊裝的母排座上（Row J 電源與類比排母、Row U 數位排母），Type-C 向左微幅外露；主板投影區（Cols 6~24, Rows K~T）洞洞板正面貼附耐高溫 Kapton 金手指絕緣膠帶防短路。
   - **集中配置 JST-XH2.54 直針公座排（全數集中於 Row B，Cols 3~35）**：
     - **CON 1（8P）**：LCD 模組 `(Cols 3~10, Row B)`
     - **CON 2A（3P）**：方向盤電位計 `(Cols 12~14, Row B)`
     - **CON 2B（3P）**：**左油門 APEM SN 工業 T-Bar** `(Cols 16~18, Row B)`
     - **CON 2C（3P）**：**右油門 APEM SN 工業 T-Bar** `(Cols 20~22, Row B)`
     - **CON 2D（3P）**：引擎三段搖頭開關 `(Cols 24~26, Row B)`
     - **CON 2E（2P）**：模式切換開關 `(Cols 28~29, Row B)`
     - **CON 2F（2P）**：控制權按鈕 `(Cols 31~32, Row B)`
     - **CON 4（2P）**：安全急停開關 `(Cols 34~35, Row B)`（D20 NC 常閉迴路）
   - **40×24 每個洞孔位座標規劃與正背面焊接工法**：
     - **Row H 貫穿接地排**：0.8mm 鍍錫裸銅線貫穿橫跨整板（Col 4 至 Col 35），所有元件 GND 腳直接折腳拖焊其上。
     - **Row F 拖焊錫橋**：左油門分壓節點 `(16-F, 17-F, 18-F)` 與右油門分壓節點 `(20-F, 21-F, 22-F)` 採用背面折腳拖焊成一體飽滿錫球，引出跳線分別直插 Arduino `A1` 與 `A2`。
     - **背面仰視鏡像圖紙**：左右已水平鏡像對調（Col 40 到 Col 1），拿著烙鐵翻轉洞洞板仰視施工時看哪裡就焊哪裡，免左右反向心算！
      - 完整 12 頁工程圖紙、3D 外觀視角、孔位表、實物照片圖鑑與全套分解圖解 SOP 請參見：[docs/hardware_diagram.html](file:///E:/Source/MIS/2026/MAVLinkController/docs/hardware_diagram.html) ｜ 12 頁完整高解析 PDF 列印圖紙：[docs/hardware_diagram.pdf](file:///E:/Source/MIS/2026/MAVLinkController/docs/hardware_diagram.pdf)
     - **實物照片圖鑑與台灣在地採購指南**：包含 13 大關鍵零件（雙 APEM SN 霍爾推桿、Arduino UNO Q、雙節 18650 UPS 模組、松下 BSMI 電芯、FR-4 玻纖板、米羅科技 3.2" TFT LCD 模組、急停按鈕、搖頭開關、精密電阻電容與端子線材）之真實產品照片、關鍵參數、台灣推薦通路（米羅科技官方商城、Mouser、DigiKey、ICShop、良興電子、蝦皮現貨、廣華電子）與採購防坑手冊。

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
│  │    APEM 左推桿      │   │    RC_Override 發送   │ │
│  │    → left_thr       │   │    HEARTBEAT 發送     │ │
│  │                     │   │    ARM/DISARM 指令    │ │
│  │  • analogRead(A2)   │   │                       │ │
│  │    APEM 右推桿      │   │  • loop() 5Hz         │ │
│  │    → right_thr      │   │    Bridge.call        │ │
│  │                     │   │    ("update_display") │ │
│  │  • digitalRead      │   │                       │ │
│  │    A3 啟動 A4 關閉  │   │  • Failsafe 執行緒    │ │
│  │    A5 控制權 D20急停│   │    1秒無指令→歸零     │ │
│  │    D21 模式切換     │   │                       │ │
│  │                     │   │                       │ │
│  │  • ILI9341 LCD 顯示 │   │                       │ │
│  │    5Hz 刷新畫面     │   │                       │ │
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
| MCU → MPU | `on_inputs` | `notify`（單向） | 10 Hz + 邊緣觸發 | steering, left_thr, right_thr, l_eng_state, l_eng_edge, r_eng_state, r_eng_edge, ctrl_edge, estop_state |
| MPU → MCU | `update_display` | `call`（雙向 RPC） | 5 Hz | speed_x10, heading, bat, gps_fix, armed, mode, lat_e7, lon_e7, remote_ip, remote_port, local_ip, local_port, control_authority |

**`on_inputs` 參數：**

| 參數 | 型別 | 說明 |
|------|:----:|------|
| `steering` | int | −1000 … +1000 |
| `left_thr` / `right_thr` | int | 1000–2000 μs |
| `l_eng_state` / `r_eng_state` | int | 左／右引擎開關現況 0=OFF 1=ON |
| `l_eng_edge` / `r_eng_edge` | int | 邊緣代碼（見下表） |
| `ctrl_edge` | int | 控制權按鈕手勢（見下表） |
| `estop_state` | int | 0=正常 1=急停按鈕拍下中 |
| `mode_sw_acro` | int | 模式開關位置 0=手動 1=定向（只送現況，MPU 端比對前值判斷變化） |

**引擎開關動作（`EDGE_*`）：**

| 值 | 意義 | 觸發動作 |
|----|------|---------|
| 0 | 無邊緣（開關回中） | 維持當前引擎狀態 |
| 1 | 上升沿（撥往上 A3） | 啟動引擎（ARM + 進入設定模式） |
| 2 | 下降沿（撥往下 A4） | 關閉引擎（DISARM + 油門歸中立檔） |

**控制權按鈕手勢代碼（`CTRL_*`）：**

| 值 | 手勢 | 觸發動作 |
|----|------|---------|
| 0 | 無 | — |
| 1 | 短按 | 請求控制權（cmd 410, param2=1） |
| 2 | 長按 2 秒 | 釋放控制權（cmd 410, param2=0）／急停閂鎖中則為**解除急停** |

**控制權狀態代碼（`AUTH_*`，經 `update_display` 回傳給 LCD）：**

| 值 | 狀態 | LCD 標題列圓點 |
|----|------|---------------|
| 0 | 未請求 | 灰 |
| 1 | 請求中（等待 ACK） | 黃 |
| 2 | 已取得 | 綠 |
| 3 | 被拒絕 | 紅 |
| 4 | **急停閂鎖中** | 顯示全畫面急停橫幅 |

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
  ├─ set_engine_gates(左開關, 右開關)   ← 引擎閘門
  │
  ├─ send_rc_override()
  │    CH1 = 1500 + steering/2         (1000–2000 μs)
  │    CH3 = 左開關 ON ? left_thr  : 1500
  │    CH4 = 右開關 ON ? right_thr : 1500
  │
  ▼ RC_CHANNELS_OVERRIDE #70 via WiFi UDP
USV 接收 → 左右推進器 PWM 輸出
```

### 操作模式：手動與定向

本控制器**只提供兩種模式**。手持遙控器沒有海圖，無法設定航點，因此
AUTO / GUIDED / RTL 這些航點驅動的模式沒有任何操作介面，也不開放選擇
（`mavlink_handler.SELECTABLE_MODES` 會拒絕其他模式值）。

| | 手動 MANUAL (0) | 定向 ACRO (1) |
|---|---|---|
| 方向盤 | 舵角——直接對應舵往哪偏 | **轉向速率**——轉多快 |
| 方向盤回中 | 舵回正，船受風流影響會偏航 | **鎖定當前航向**，船自動修正 |
| 左右油門 | 直接控制推進器 | 同左（不變） |
| 閉迴路位置 | 無 | **船端** |

**關鍵設計**：兩種模式送出的 RC 通道完全相同，只有船端解讀不同。切換模式時
GCS 端不改變任何控制邏輯，只送出一次 `DO_SET_MODE`。

```
手動：CH1 = 1500 + steering/2  → 船端當作舵角
定向：CH1 = 1500 + steering/2  → 船端當作轉向速率，回中(1500)即 0°/s = 保持航向
```

航向的閉迴路跑在**船上**，不跨越無線鏈路。若改由遙控器端跑羅盤回授 PID，控制迴路
就架在會丟包、有延遲的 UDP 上，而船舶轉向慣性大，這種架構極易震盪——這是刻意避開的
設計選項。

#### 定向模式的航向源防護

ACRO 依賴船端 EKF 的航向估計。若航向估計不可靠，船會鎖定一個沒有人選擇的航向直直開走，
比手動更危險。因此：

| 情境 | 行為 |
|------|------|
| `gps_fix < 2` 時撥到定向 | **拒絕**，改送 MANUAL，LCD 顯示 `手動 →定向`（紅字） |
| 定向中 `gps_fix` 掉到 < 2 | **自動退回手動**，LCD 顯示不一致警告 |
| 定位恢復（開關仍在定向） | 自動重試進入定向，**3 秒節流**避免臨界抖動反覆切換 |
| SET_MODE 遺失或被拒（船端仍在手動） | 持續重新宣告，直到船端模式與開關一致 |
| 船端自行改模式（如船端 failsafe → HOLD/RTL） | **不強制覆蓋**，僅在 LCD 與網頁顯示不一致——船端有它自己的理由 |
| ARM 時定位已失效 | ARM 前先把 `desired_mode` 降回手動，避免武裝進入無效的定向 |
| 連線中斷 | 看門狗停止動作，不對著斷掉的鏈路重送 |

`MIN_FIX_FOR_ACRO = 2` 定義於 `main.py`。GPS fix 是遙測能取得的最佳航向可靠度代理
指標（真正的航向源是羅盤，但 2D fix 以上意味著 EKF 有可用的 yaw 估計）。

#### 模式切換的權威

模式**只能由 D21 實體開關控制**，網頁的 `set_mode` 一律拒絕。兩個權威控制同一個設定會
互相打架：實體開關會在每次變化與每次重連後重新宣告自己的位置，網頁選的模式會被無聲
還原。拒絕網頁切換，才能讓開關的物理位置始終是船舶模式的誠實指標。

急停解除後 `_mode_sw_acro` 會重置為 `None`，強迫下一次 `on_inputs` 重新推送模式——
重連後船端可能處於任意模式，「開關沒動過」不等於「模式已經正確」。

ARM 時送出的是 `desired_mode` 而非硬寫的 MANUAL。否則在定向模式下撥動引擎開關，
會把船默默打回手動。

### 左右引擎的閘控機制

MAVLink 的 `MAV_CMD_COMPONENT_ARM_DISARM` 是**整船 ARM**，協議層沒有「單獨啟動左馬達」
這種指令，船端 C# `MavLinkDriver` 也只解讀 CH1/CH3/CH4。因此左右引擎以 **GCS 端閘控**
實現，船端程式完全不需修改：

```
任一開關 ON   → 整船 ARM   (cmd 400, param1=1)
兩顆皆 OFF    → 整船 DISARM (cmd 400, param1=0)

CH3 = 左開關 ON ? 左推桿值 : 1500   ← OFF 時強制中立，推桿失效
CH4 = 右開關 ON ? 右推桿值 : 1500
```

閘控實作於 `mavlink_handler.send_rc_override()` 這個單一節點，因此**網頁滑桿同樣受
實體開關管轄**——關掉左引擎，網頁也推不動左推進器。

> **注意**：LCD「引擎」列顯示的左右狀態是**遙控器端的閘門狀態**，不是船端各引擎的
> 實際 armed 狀態（MAVLink 無法取得後者）。該列右側的小圓點才是船端整體 ARM 狀態。

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

顯示器為 **ILI9341 橫向 320×240 像素**，以 x=160 分為左右兩欄。中文以
`chinese_fonts.h` 的 16×16 點陣字繪製。

```
┌────────────────────┬────────────────────────┐ ← y=0
│    無人船遠端遙控器          　　　　　　  ● │ 標題列 + 控制權圓點(308,10)
├────────────────────┼────────────────────────┤ ← y=20
│ 速度   12.3節      │                        │ ← y=24
│ 航向   271度       │   左油門      右油門   │ ← y=48
│ 電量   ████░░ 77%  │     █           █      │ ← y=72
│ GPS位置 25.12,121.56│    ██          ██      │ ← y=96
│ 模式   定向        │    ███         ███     │ ← y=120
│ 通訊   192.168.1.100:14550                  │ ← y=144
│ 引擎   左開 右關 ●  │    60%         65%     │ ← y=168
│ 本機   192.168.1.50:14551   ╱▔╲             │ ← y=192
│ 舵角   +12度       │  方向盤 │船│           │ ← y=216
│                    │    +270  ╲│╱ ← 舵角線  │
└────────────────────┴────────────────────────┘ ← y=240
```

左欄第 5 列「模式」：顯示**船端回報的實際模式**。當它與 D21 開關位置不一致時，
會追加紅色 `->定向` 表示「開關要求定向但尚未生效」（被拒絕或指令遺失）——操作者
即將依照模式來操舵，模式沒生效必須看得見。

左欄第 7 列「引擎」：`左開 右關` 為遙控器端的閘門狀態（綠=開、紅=關），
其右的小圓點（146,176）才是船端回報的整體 ARM 狀態。

### 緊急停止橫幅

急停觸發時，整個畫面被橫幅取代——急停期間唯一該被看見的資訊就是急停本身：

```
┌─────────────────────────────────────────────┐  紅色背景
│   ┌─────────────────────────────────────┐   │
│   │            緊 急 停 止              │   │  紅字
│   │        通訊已切斷                   │   │  白字
│   │  RESET E-STOP, THEN HOLD CTRL 2s    │   │  黃字
│   └─────────────────────────────────────┘   │  黃色外框
└─────────────────────────────────────────────┘
```

橫幅顯示條件為 `estop_local || control_authority == 4`：前者讓 MCU 在按下當下
立即反應（不等 MPU 往返），後者讓實體按鈕已復位但閂鎖未解除時橫幅仍然保持。

### 色彩規範

| 元素 | 正常 | 警告 | 危險 |
|------|:----:|:----:|:----:|
| 電量 Bar | 🟢 綠（>50%） | 🟡 黃（20–50%） | 🔴 紅（<20%） |
| GPS 位置 | 🟢 綠（3D FIX） | 🟡 黃（2D FIX） | 🔴 紅（無定位） |
| 模式 | 🟢 綠（定向生效）／🔵 青（手動） | — | 🔴 紅（開關要求未生效） |
| 引擎左／右 | 🟢 綠（開） | — | 🔴 紅（關） |
| 船端 ARM 圓點 | 🟢 綠（ARMED） | — | 🔴 紅（DISARMED） |
| 控制權圓點 | 🟢 綠（已取得） | 🟡 黃（請求中） | 🔴 紅（被拒絕）／⚪ 灰（未請求） |
| 船體輪廓 | 🟢 綠（ARMED） | — | 🔴 紅（DISARMED） |
| 舵角線 | 🔵 青色 | — | — |
| 油門 Bar | 🟠 橘色 | — | — |
| **急停橫幅** | — | — | **🔴 紅底 + 黃框（覆蓋全畫面）** |

### LED 指示

| LED | 位置 | 意義 |
|:---:|------|------|
| LED1 | MPU | 連線品質：綠 ≥80、黃 20–79、紅 <20、滅=未連線 |
| LED2 | MPU | 控制權：綠=已取得、黃=請求中、紅=被拒絕、滅=未請求 |
| LED3 | MCU | TX 活動：每送出封包閃綠 100 ms |
| LED4 | MCU | RX 活動：每收到封包閃藍 100 ms |

急停閂鎖期間，**LED1 與 LED2 同步紅色 1 Hz 閃爍**，且無視網頁的手動 LED 覆蓋
——緊急指示不能被過期的網頁指令蓋掉。LED3/LED4 則熄滅（通訊已斷，不該再顯示活動）。

---

## Failsafe 安全機制

本系統實作三層 Failsafe：

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

### 層 3：手動緊急停止（實體按鈕）

前兩層是被動的——等逾時發生才作用。第三層讓操作者能**主動**在任何時刻切斷一切。

**觸發**：拍下 D20 蘑菇頭按鈕（或該線路斷線）

**動作序列**（`main._engage_estop` → `mavlink_handler.emergency_stop`）：

```
E-STOP 拍下
  │
  ├─[MCU 本地，不等 MPU]  steering=0, l_thr=1500, r_thr=1500
  │                       立即 Bridge.notify（不等 10 Hz 週期）
  │                       LCD 立刻切換為全畫面急停橫幅
  │
  └─[MPU]  1. send_safe_state() ×3，間隔 50 ms   ← UDP 無送達保證，故重送
           2. send_disarm()
           3. send_request_operator_control(False)
           4. disconnect()                        ← 關閉 socket
           5. _estop_latched = True               ← 閂鎖
```

**順序至為關鍵**：socket 一旦關閉就再也送不出任何指令，所以所有停止指令必須先送出。
而切斷通訊本身就是最強的 failsafe——沒有後續控制封包，船端 `ControlFailsafeTimeoutMs`
逾時後會自行歸零推進器，完全不依賴遙控器還能送出什麼。

**閂鎖狀態**：
- 忽略所有 MCU 輸入（推桿、引擎開關、控制權按鈕）
- 拒絕所有網頁控制指令（ARM／油門／模式／控制權／重新連線）
- LED1 + LED2 紅色 1 Hz 閃爍，且**無視網頁的手動 LED 覆蓋**
- LCD 全畫面紅底橫幅；網頁顯示全螢幕紅色遮罩

**解除**（兩個條件都要滿足）：
1. 旋轉復位實體 E-STOP 按鈕（D20 回到 LOW）
2. 長按控制權按鈕 2 秒

解除後回到**冷啟動狀態**：重新連線、無控制權、通道中立。操作者必須重新請求控制權——
解除急停不等於重新獲得駕駛授權。

#### 解除後的重啟互鎖

解除閂鎖**不會**立刻恢復控制。系統進入互鎖狀態，強制輸出中立，直到操作者：

1. 將左右油門推桿歸回中立（±100 μs 內），且
2. 將兩顆引擎開關都撥到 OFF

在此之前，引擎閘門強制關閉、方向與油門固定送 1500、ARM 指令被忽略。

沒有這道互鎖，解除後的第一個 `on_inputs`（100 ms 內）就會把推桿當下的位置重新套用——
操作者若沒收油門就解除急停並撥引擎開關，`send_arm()` 會在滿油門 RC 已經串流的狀態下觸發。

同時，**網頁控制會被強制關閉**。急停閂鎖期間無法關閉網頁控制（該端點被 `_blocked_by_estop`
擋住），若解除後仍保持啟用，瀏覽器將獨佔控制權而遙控器的推桿完全失效——與「解除急停的人
就是站在遙控器前的人」這個前提正好相反。

> 閂鎖是必要的：若只看按鈕當下狀態，急停按鈕一復位就會自動重連並恢復油門輸出，
> 等同無預警重新啟動。
>
> 解除功能**刻意不開放給網頁**——必須有人站在遙控器前面才能解除。

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
- 使用 `analogRead()` 讀取三路可變電阻 ADC
- `DebouncedInput` 對四路數位輸入做 25 ms 消抖與邊緣偵測
- 控制權按鈕的長／短按判別（`CTRL_HOLD_MS` = 2000 ms）
- 急停觸發時**本地立即歸零**推桿值，不等 MPU 回應
- 以 `Bridge.notify("on_inputs", ...)` 推送 9 個輸入參數到 MPU（10 Hz，邊緣時立即觸發）
- 提供 `update_display()` RPC 方法供 MPU 呼叫以更新 LCD
- 以 5 Hz 刷新 ILI9341 LCD 畫面

#### `sketch/lcd_display.h`
- 定義 `VesselStatus` 結構體與 `AUTHV_*` 控制權代碼
- `lcd_init()` / `lcd_draw_static()`：初始化及繪製靜態框架
- `lcd_update_dynamic()`：更新動態資料區（速度、航向、電量、GPS、模式、通訊、
  左右引擎閘門、本機位址、舵角、油門 Bar、船體輪廓與舵角線）
- `lcd_draw_estop_banner()`：全畫面急停橫幅
- RGB565 色彩定義與 Bar 繪製工具函式

#### `sketch/chinese_fonts.h`
- 71 個 16×16 中文點陣字模與 `get_char_index()` Unicode 查表
- 查不到的字會被 `draw_utf8_string()` 靜默跳過，新增介面文字時務必確認字模存在

#### `python/main.py`
- 以 `Bridge.provide("on_inputs", on_inputs)` 接收 MCU 輸入
- 急停狀態機：`_estop_active`（按鈕現況）／`_estop_latched`（閂鎖）、
  `_engage_estop()` / `_clear_estop()`
- 模式管理：`_apply_mode()`（含航向源檢查）、`_mode_watchdog()`（定位掉失自動退回
  手動、持續重新宣告開關要求的模式、不干預船端自主模式）
- 重啟互鎖：`_resume_interlock`（急停解除後要求推桿歸中立、引擎開關關閉）
- 引擎開關 → `mavlink.set_engine_gates()` 與整船 ARM/DISARM
  （`_arm_cmd_state` 追蹤本地意圖，避免重複下令）
- 控制權按鈕手勢 → `mavlink.send_request_operator_control()`
- `_blocked_by_estop()` 守衛所有網頁控制端點
- `loop()` 每 200 ms 呼叫 `Bridge.call("update_display", ...)` 推送船舶狀態到 LCD

#### `python/mavlink_handler.py`
- 連線管理：`connect()` / `disconnect()` / `reconnect()`
- 控制發送：`send_rc_override()`（含引擎閘控）/ `send_arm()` / `send_disarm()` /
  `send_safe_state()` / `set_engine_gates()`
- 模式：`set_mode()`（僅接受 `SELECTABLE_MODES`）、`desired_mode`（ARM 時沿用）
- `_send()`：所有送出的單一出口，容忍 `disconnect()` 併發清空 socket。
  背景執行緒若在檢查後、送出前被斷線，未保護的解參考會拋 `AttributeError`
  並靜默終止該執行緒
- `disconnect()` 會 **join** 背景執行緒後才返回，避免重連時產生兩組 RC 串流
- 急停：`emergency_stop()`（歸零 ×3 → DISARM → 釋放控制權 → 斷線）、
  `resume_from_estop()`
- 遙測接收：背景執行緒解析 HEARTBEAT / VFR_HUD / ATTITUDE / SYS_STATUS / GPS_RAW_INT 等
- Failsafe：`_control_loop()` 逾時自動送出中立指令

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
| 左油門（A1） | 0–1023 | 1000–2000 μs | RouterBridge `left_thr` / MAVLink CH3（受左引擎閘門） |
| 右油門（A2） | 0–1023 | 1000–2000 μs | RouterBridge `right_thr` / MAVLink CH4（受右引擎閘門） |

### 操作流程

```
1. 開機          → LCD 顯示遙測框架，LED1 依連線品質亮燈
2. 短按控制權鍵  → 請求控制權，LED2 黃 → 綠（船端 ACK 通過）
3. 選擇模式(D21) → 撥下=手動、撥上=定向（定向需 GPS fix ≥ 2）
4. 撥上引擎開關  → 整船 ARM（沿用目前模式），該側推桿生效
5. 操作推桿方向盤 → RC_CHANNELS_OVERRIDE 10 Hz 送出
                   手動：方向盤=舵角／定向：方向盤=轉向速率，回中鎖航向
6. 撥下引擎開關  → 該側油門鎖 1500；兩側皆下 → DISARM
7. 長按控制權鍵  → 釋放控制權

  任何時刻拍下急停 → 歸零、DISARM、切斷通訊、閂鎖
  解除：復位急停鈕 + 長按控制權鍵 2 秒
        → 重新連線，但進入互鎖：需推桿歸中立 + 引擎開關 OFF 才恢復控制
        → 網頁控制被強制關閉，需重新請求控制權
```

### ArduPilot Rover 模式代碼

| 代碼 | 模式 | 本控制器 | 說明 |
|:----:|------|:-------:|------|
| **0** | **MANUAL** | ✅ 可選 | 手動：CH1 直接對應舵角 |
| **1** | **ACRO** | ✅ 可選 | 定向：CH1 為轉向速率，回中鎖定當前航向 |
| 4 | HOLD | ❌ | 原地保持 |
| 10 | AUTO | ❌ | 自動任務（需航點） |
| 11 | RTL | ❌ | 自動返回起點 |
| 15 | GUIDED | ❌ | 引導模式（需外部航點） |

不可選的模式**仍會正確解碼顯示**——若船端因自身 failsafe 或其他 GCS 而切換模式，
LCD 與網頁都會如實顯示，不會偽裝成手動。這些常數定義於 `lcd_display.h` 與
`mavlink_handler.py`，兩處必須一致（`MODE_*` 是 MAVLink `custom_mode` 原始值，
不是顯示索引）。

### 電源狀態欄位單位

| 欄位 | 來源訊息 | 單位 | 換算 |
|------|---------|:----:|------|
| `voltage_mv` | SYS_STATUS | mV | ÷1000 = V |
| `current_ca` | SYS_STATUS | cA（釐安） | ÷100 = A |
| `battery_pct` | SYS_STATUS | % | 直接使用 |
