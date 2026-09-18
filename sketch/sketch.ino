// USV Remote Controller — M4 Sketch
// MCU↔MPU communication via Arduino_RouterBridge (RPClite/MsgPack)

// 必須留在 Adafruit 標頭之前，且不能刪 —— arduino-cli 只會把「被 include 到的」
// 函式庫加進 include path。詳見 libs/ZephyrAvrCompat/src/ZephyrAvrCompat.h
#include <ZephyrAvrCompat.h>
#include <Arduino_RouterBridge.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include "lcd_display.h"

// ── Pin definitions ──────────────────────────────────────────────────────────
#define PIN_STEERING   A0   // 方向盤電位計 (WH148 B10K, 3.3V 供電)
#define PIN_LEFT_THR   A1   // 左油門：工業級 APEM SN 系列 T-Bar 霍爾推桿 (5V供電，經載板 10k:20k 分壓為 0~3.33V)
#define PIN_RIGHT_THR  A2   // 右油門：工業級 APEM SN 系列 T-Bar 霍爾推桿 (5V供電，經載板 10k:20k 分壓為 0~3.33V)

// ── 類比輸入校正 ─────────────────────────────────────────────────────────────
// 方向盤：電位計實際接線方向與舵向相反（往左轉舵卻往右跑、最左顯示 +1000），
// 故將輸出極性反轉，讓「往左 = 負、往右 = 正」。若日後重新接線可改回 0。
#define STEER_REVERSED  1

// 油門推桿：霍爾推桿實測行程不會涵蓋整個 0~1023，原先只能顯示 26%~100% / 22%~100%。
// 以下為實測 ADC 端點（拉到底 / 推到底），用 map 把實際行程展成 1000~2000 μs 全範圍，
// 對應 LCD 上的 -100% ~ +100%。若更換推桿，量測 Serial 或 LCD 端點值後修改這裡即可。
#define LEFT_THR_ADC_MIN    266   // 26% × 1023 → 左推桿拉到底（-100%）
#define LEFT_THR_ADC_MAX   1023   //            左推桿推到底（+100%）
#define RIGHT_THR_ADC_MIN   225   // 22% × 1023 → 右推桿拉到底（-100%）
#define RIGHT_THR_ADC_MAX  1023   //            右推桿推到底（+100%）

// 這 5 個數位輸入刻意避開 D0–D7 那排排針。UNO 外形的 D7↔D8 間距是 0.16"（4.06mm），
// 落不到萬用板的 0.1" 孔上，整排要彎 1.02mm 才插得進去。改用 A3–A5 與 D20/D21 之後，
// 擴充板只需焊 10P（D8–D21）／8P（電源）／6P（類比）三排，全部正落在孔位，不必彎腳。
// D20/D21 是 I2C2 的 SDA/SCL —— 當一般 GPIO 用，所以本 sketch 不可呼叫 Wire.begin()。
#define PIN_ENG_START  A3   // 搖頭開關往上（啟動引擎，PA7）— INPUT_PULLUP, 撥上 = LOW, 放開彈回中間
#define PIN_ENG_STOP   A4   // 搖頭開關往下（關閉引擎，PC1）— INPUT_PULLUP, 撥下 = LOW, 放開彈回中間
#define PIN_CTRL_BTN   A5   // 控制權按鈕（瞬時，PC0）— INPUT_PULLUP, 按下 = LOW
#define PIN_ESTOP     D20   // 緊急停止（PB11）— 蘑菇頭自鎖式，接 NC 常閉接點
                            // 正常時電路導通拉 LOW；拍下（或線材脫落）= HIGH → 觸發
                            // NC 接法讓斷線也視為急停，符合 fail-safe 原則
#define PIN_MODE_SW   D21   // 模式開關（兩段撥動，PB10）— INPUT_PULLUP
                            // 撥上(LOW) = 定向 ACRO；撥下(HIGH) = 手動 MANUAL

#define TFT_CS    10
#define TFT_DC     8
#define TFT_RST    9

// Switch edge codes (must match Python constants)
#define EDGE_NONE    0
#define EDGE_RISING  1   // inactive → active
#define EDGE_FALLING 2   // active   → inactive

// Control-button gesture codes (must match Python CTRL_* constants)
#define CTRL_NONE    0
#define CTRL_SHORT   1   // 短按 → 請求控制權
#define CTRL_LONG    2   // 長按 2s → 釋放控制權，或解除急停閂鎖

// Control-authority codes (must match Python _AUTH_CODE)
#define AUTH_NONE    0
#define AUTH_PENDING 1
#define AUTH_GRANTED 2
#define AUTH_DENIED  3
#define AUTH_ESTOP   4   // MPU 端急停閂鎖中（通訊已切斷）

#define DEBOUNCE_MS    25
#define CTRL_HOLD_MS 2000   // 控制權按鈕長按門檻

// ── Globals ───────────────────────────────────────────────────────────────────
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

VesselStatus g_vessel = {0}; // 這行現在可以被編譯器正常解析了

int16_t  g_steering   = 0;
uint16_t g_left_thr   = 1500;
uint16_t g_right_thr  = 1500;
bool     g_engine_on  = false;

unsigned long t_last_send    = 0;
unsigned long t_last_display = 0;

// ── Debounced digital input ───────────────────────────────────────────────────
// active_high=false → LOW 視為 active（INPUT_PULLUP 接 GND 的開關/按鈕）
// active_high=true  → HIGH 視為 active（E-STOP 的 NC 常閉接法）
struct DebouncedInput {
    uint8_t       pin;
    bool          active_high;
    bool          stable;     // 已消抖的邏輯狀態
    bool          last_raw;
    unsigned long t_change;
};

DebouncedInput db_eng_start;
DebouncedInput db_eng_stop;
DebouncedInput db_ctrl;
DebouncedInput db_estop;
DebouncedInput db_mode;

static bool db_raw(const DebouncedInput* d) {
    return (digitalRead(d->pin) == HIGH) == d->active_high;
}

static void db_init(DebouncedInput* d, uint8_t pin, bool active_high) {
    d->pin         = pin;
    d->active_high = active_high;
    pinMode(pin, INPUT_PULLUP);
    bool raw       = db_raw(d);
    d->stable      = raw;
    d->last_raw    = raw;
    d->t_change    = 0;
}

// 回傳 EDGE_NONE / EDGE_RISING / EDGE_FALLING（已消抖）
static uint8_t db_update(DebouncedInput* d, unsigned long now) {
    bool raw = db_raw(d);
    if (raw != d->last_raw) {
        d->last_raw = raw;
        d->t_change = now;
        return EDGE_NONE;
    }
    if (raw != d->stable && (now - d->t_change) >= DEBOUNCE_MS) {
        d->stable = raw;
        return raw ? EDGE_RISING : EDGE_FALLING;
    }
    return EDGE_NONE;
}

// 控制權按鈕手勢狀態
unsigned long t_ctrl_press   = 0;
bool          ctrl_long_sent = false;
// The hold timer only arms once an actual press edge has been seen. Without
// this, a button held (or A5 shorted) at power-up would satisfy
// `now - t_ctrl_press >= CTRL_HOLD_MS` as soon as millis() passed 2000 and emit
// a RELEASE_CONTROL nobody asked for.
bool          ctrl_press_seen = false;

// ── LED helpers ───────────────────────────────────────────────────────────────
void set_led3_color(int r, int g, int b) {
    analogWrite(LED3_R, r);
    analogWrite(LED3_G, g);
    analogWrite(LED3_B, b);
}

void set_led4_color(bool r, bool g, bool b) {
    digitalWrite(LED4_R, r ? LOW : HIGH);
    digitalWrite(LED4_G, g ? LOW : HIGH);
    digitalWrite(LED4_B, b ? LOW : HIGH);
}

// ── RouterBridge handler ──────────────────────────────────────────────────────
// NOTE: string params must be `String` (arduino::msgpack::str_t) — RPClite's
// MsgPack unpacker can only deserialize into a String&, not a const char*.
void update_display(int speed_x10, int heading, int bat, int fix, int armed, int mode,
                    int32_t lat_e7, int32_t lon_e7,
                    String remote_ip, int remote_port,
                    String local_ip, int local_port,
                    int control_authority, int link_ok) {
    g_vessel.speed_x10   = (uint16_t)speed_x10;
    g_vessel.heading     = (uint16_t)heading;
    g_vessel.battery_pct = (uint8_t)bat;
    g_vessel.gps_fix     = (uint8_t)fix;
    g_vessel.armed       = (bool)armed;
    g_vessel.mode        = (uint8_t)mode;
    g_vessel.lat_degE7   = lat_e7;
    g_vessel.lon_degE7   = lon_e7;
    strncpy(g_vessel.remote_ip, remote_ip.c_str(), sizeof(g_vessel.remote_ip) - 1);
    g_vessel.remote_ip[sizeof(g_vessel.remote_ip) - 1] = '\0';
    g_vessel.remote_port = (uint16_t)remote_port;
    strncpy(g_vessel.local_ip, local_ip.c_str(), sizeof(g_vessel.local_ip) - 1);
    g_vessel.local_ip[sizeof(g_vessel.local_ip) - 1] = '\0';
    g_vessel.local_port  = (uint16_t)local_port;
    g_vessel.control_authority = (uint8_t)control_authority;
    g_vessel.link_ok     = (bool)link_ok;   // MAVLink RX link status — drives "斷線" indicator
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    db_init(&db_eng_start, PIN_ENG_START, false);  // 撥上 = LOW
    db_init(&db_eng_stop,  PIN_ENG_STOP,  false);  // 撥下 = LOW
    db_init(&db_ctrl,      PIN_CTRL_BTN,  false);  // 按下 = LOW
    db_init(&db_estop,     PIN_ESTOP,     true);   // NC 常閉：拍下/斷線 = HIGH
    db_init(&db_mode,      PIN_MODE_SW,   false);  // 撥上(LOW) = 定向

    pinMode(LED4_R, OUTPUT);
    pinMode(LED4_G, OUTPUT);
    pinMode(LED4_B, OUTPUT);
    set_led3_color(0, 0, 0);
    set_led4_color(false, false, false);

    // 強制硬體重置螢幕
    pinMode(TFT_RST, OUTPUT);
    digitalWrite(TFT_RST, HIGH); delay(10);
    digitalWrite(TFT_RST, LOW);  delay(20);
    digitalWrite(TFT_RST, HIGH); delay(150);

    // 初始化螢幕與靜態線條
    lcd_init(&tft, 8000000);
    lcd_draw_static(&tft);

    // 註冊通訊協定
    Bridge.begin();
    Bridge.provide("update_display", update_display);
    Bridge.provide("set_led3_color", set_led3_color);
    Bridge.provide("set_led4_color", set_led4_color);
}

// ── Main loop ─────────────────────────────────────────────────────────────────
void loop() {
    Bridge.update();

    unsigned long now = millis();

    // ── 數位輸入消抖與邊緣偵測 ────────────────────────────────────────────────
    uint8_t start_edge = db_update(&db_eng_start, now);
    uint8_t stop_edge  = db_update(&db_eng_stop,  now);
    uint8_t ctrl_raw   = db_update(&db_ctrl,  now);
    uint8_t estop_edge = db_update(&db_estop, now);
    uint8_t mode_edge  = db_update(&db_mode,  now);

    bool estop_on  = db_estop.stable;
    bool mode_acro = db_mode.stable;   // 撥上 = 定向

    // 搖頭開關：往上撥(A3) = 啟動引擎，往下撥(A4) = 關閉引擎，放開自動彈回中間保持狀態
    // 船端引擎已並聯接在一起不分左右。安全原則：若有衝突以關閉（STOP）優先
    uint8_t eng_edge = EDGE_NONE;
    if (stop_edge == EDGE_RISING) {
        g_engine_on = false;
        eng_edge    = EDGE_FALLING;
    } else if (start_edge == EDGE_RISING) {
        g_engine_on = true;
        eng_edge    = EDGE_RISING;
    }

    // 控制權按鈕：短按 = 請求，長按 CTRL_HOLD_MS = 釋放 / 解除急停
    uint8_t ctrl_edge = CTRL_NONE;
    if (ctrl_raw == EDGE_RISING) {          // 按下
        t_ctrl_press    = now;
        ctrl_long_sent  = false;
        ctrl_press_seen = true;
    } else if (ctrl_raw == EDGE_FALLING) {  // 放開 — 長按已觸發過就不再算短按
        if (ctrl_press_seen && !ctrl_long_sent) ctrl_edge = CTRL_SHORT;
        ctrl_press_seen = false;
    }
    // 按住超過門檻時立即觸發長按，不等放開（操作者能從 LCD 得到即時回饋）
    if (ctrl_press_seen && db_ctrl.stable && !ctrl_long_sent &&
        (now - t_ctrl_press) >= CTRL_HOLD_MS) {
        ctrl_long_sent = true;
        ctrl_edge      = CTRL_LONG;
    }

    // ── 類比輸入 ──────────────────────────────────────────────────────────────
    // A0: 方向盤舵角 (WH148 B10K, 0~3.3V) → -1000(左) ~ +1000(右)
#if STEER_REVERSED
    g_steering  = (int16_t)map(analogRead(PIN_STEERING), 0, 1023, 1000, -1000);
#else
    g_steering  = (int16_t)map(analogRead(PIN_STEERING), 0, 1023, -1000, 1000);
#endif
    // A1: 左油門 APEM SN Series T-Bar 霍爾推桿 (5V 供電，經載板 10k:20k 精密分壓為 0~3.33V)
    //     實測行程 LEFT_THR_ADC_MIN~MAX → 展成 1000~2000 μs（LCD 顯示 -100%~+100%）
    g_left_thr  = (uint16_t)map(constrain(analogRead(PIN_LEFT_THR), LEFT_THR_ADC_MIN, LEFT_THR_ADC_MAX),
                                LEFT_THR_ADC_MIN, LEFT_THR_ADC_MAX, 1000, 2000);
    // A2: 右油門 APEM SN Series T-Bar 霍爾推桿 (5V 供電，經載板 10k:20k 精密分壓為 0~3.33V)
    //     實測行程 RIGHT_THR_ADC_MIN~MAX → 展成 1000~2000 μs（LCD 顯示 -100%~+100%）
    g_right_thr = (uint16_t)map(constrain(analogRead(PIN_RIGHT_THR), RIGHT_THR_ADC_MIN, RIGHT_THR_ADC_MAX),
                                RIGHT_THR_ADC_MIN, RIGHT_THR_ADC_MAX, 1000, 2000);

    // 急停期間 MCU 端就先歸零，不依賴 MPU 有沒有收到（推桿未歸位也強制中立，引擎強制關閉）
    if (estop_on) {
        g_engine_on = false;
        g_steering  = 0;
        g_left_thr  = 1500;
        g_right_thr = 1500;
    }

    // ── 推送輸入到 MPU ────────────────────────────────────────────────────────
    // 任一邊緣事件立即送出，其餘時間維持 10 Hz 快照
    // 模式開關只送現況不送邊緣 — MPU 端比對前值即可判斷變化，省一個參數；
    // 切換當下仍會走 has_event 立即送出，延遲與邊緣寫法相同
    bool has_event = (eng_edge   != EDGE_NONE) ||
                     (ctrl_edge  != CTRL_NONE) ||
                     (estop_edge != EDGE_NONE) ||
                     (mode_edge  != EDGE_NONE);

    if (has_event || (now - t_last_send >= 100)) {
        Bridge.notify("on_inputs",
            (int)g_steering, (int)g_left_thr, (int)g_right_thr,
            (int)g_engine_on, (int)eng_edge,
            (int)g_engine_on, (int)eng_edge,
            (int)ctrl_edge, (int)estop_on, (int)mode_acro);
        t_last_send = now;
    }

    // ── 5 Hz 刷新螢幕 ─────────────────────────────────────────────────────────
    if (now - t_last_display >= 200) {
        lcd_update_dynamic(&tft, &g_vessel,
                           g_steering, g_left_thr, g_right_thr,
                           g_engine_on, estop_on, mode_acro);
        t_last_display = now;
    }
}
