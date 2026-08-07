// USV Remote Controller — M4 Sketch
// MCU↔MPU communication via Arduino_RouterBridge (RPClite/MsgPack)

#include <Arduino_RouterBridge.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include "lcd_display.h"

// ── Pin definitions ──────────────────────────────────────────────────────────
#define PIN_STEERING   A0
#define PIN_LEFT_THR   A1
#define PIN_RIGHT_THR  A2

#define PIN_ENG_L       2   // 左引擎撥動開關 — INPUT_PULLUP, ON = LOW
#define PIN_ENG_R       3   // 右引擎撥動開關 — INPUT_PULLUP, ON = LOW
#define PIN_CTRL_BTN    4   // 控制權按鈕（瞬時）— INPUT_PULLUP, 按下 = LOW
#define PIN_ESTOP       5   // 緊急停止 — 蘑菇頭自鎖式，接 NC 常閉接點
                            // 正常時電路導通拉 LOW；拍下（或線材脫落）= HIGH → 觸發
                            // NC 接法讓斷線也視為急停，符合 fail-safe 原則
#define PIN_MODE_SW     6   // 模式開關（兩段撥動）— INPUT_PULLUP
                            // 撥上(LOW) = 定向 ACRO；撥下(HIGH) = 手動 MANUAL

#define TFT_CS    10
#define TFT_DC     9
#define TFT_RST    8

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

DebouncedInput db_eng_l;
DebouncedInput db_eng_r;
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
    db_init(&db_eng_l, PIN_ENG_L,    false);  // ON = LOW
    db_init(&db_eng_r, PIN_ENG_R,    false);  // ON = LOW
    db_init(&db_ctrl,  PIN_CTRL_BTN, false);  // 按下 = LOW
    db_init(&db_estop, PIN_ESTOP,    true);   // NC 常閉：拍下/斷線 = HIGH
    db_init(&db_mode,  PIN_MODE_SW,  false);  // 撥上(LOW) = 定向

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
    uint8_t l_eng_edge  = db_update(&db_eng_l, now);
    uint8_t r_eng_edge  = db_update(&db_eng_r, now);
    uint8_t ctrl_raw    = db_update(&db_ctrl,  now);
    uint8_t estop_edge  = db_update(&db_estop, now);
    uint8_t mode_edge   = db_update(&db_mode,  now);

    bool l_eng_on = db_eng_l.stable;
    bool r_eng_on = db_eng_r.stable;
    bool estop_on = db_estop.stable;
    bool mode_acro = db_mode.stable;   // 撥上 = 定向

    // 控制權按鈕：短按 = 請求，長按 CTRL_HOLD_MS = 釋放 / 解除急停
    uint8_t ctrl_edge = CTRL_NONE;
    if (ctrl_raw == EDGE_RISING) {          // 按下
        t_ctrl_press   = now;
        ctrl_long_sent = false;
    } else if (ctrl_raw == EDGE_FALLING) {  // 放開 — 長按已觸發過就不再算短按
        if (!ctrl_long_sent) ctrl_edge = CTRL_SHORT;
    }
    // 按住超過門檻時立即觸發長按，不等放開（操作者能從 LCD 得到即時回饋）
    if (db_ctrl.stable && !ctrl_long_sent && (now - t_ctrl_press) >= CTRL_HOLD_MS) {
        ctrl_long_sent = true;
        ctrl_edge      = CTRL_LONG;
    }

    // ── 類比輸入 ──────────────────────────────────────────────────────────────
    g_steering  = (int16_t)map(analogRead(PIN_STEERING),  0, 1023, -1000, 1000);
    g_left_thr  = (uint16_t)map(analogRead(PIN_LEFT_THR),  0, 1023, 1000, 2000);
    g_right_thr = (uint16_t)map(analogRead(PIN_RIGHT_THR), 0, 1023, 1000, 2000);

    // 急停期間 MCU 端就先歸零，不依賴 MPU 有沒有收到（推桿未歸位也強制中立）
    if (estop_on) {
        g_steering  = 0;
        g_left_thr  = 1500;
        g_right_thr = 1500;
    }

    // ── 推送輸入到 MPU ────────────────────────────────────────────────────────
    // 任一邊緣事件立即送出，其餘時間維持 10 Hz 快照
    // 模式開關只送現況不送邊緣 — MPU 端比對前值即可判斷變化，省一個參數；
    // 切換當下仍會走 has_event 立即送出，延遲與邊緣寫法相同
    bool has_event = (l_eng_edge != EDGE_NONE) || (r_eng_edge != EDGE_NONE) ||
                     (ctrl_edge  != CTRL_NONE) || (estop_edge != EDGE_NONE) ||
                     (mode_edge  != EDGE_NONE);

    if (has_event || (now - t_last_send >= 100)) {
        Bridge.notify("on_inputs",
            (int)g_steering, (int)g_left_thr, (int)g_right_thr,
            (int)l_eng_on, (int)l_eng_edge,
            (int)r_eng_on, (int)r_eng_edge,
            (int)ctrl_edge, (int)estop_on, (int)mode_acro);
        t_last_send = now;
    }

    // ── 5 Hz 刷新螢幕 ─────────────────────────────────────────────────────────
    if (now - t_last_display >= 200) {
        lcd_update_dynamic(&tft, &g_vessel,
                           g_steering, g_left_thr, g_right_thr,
                           l_eng_on, r_eng_on, estop_on, mode_acro);
        t_last_display = now;
    }
}
