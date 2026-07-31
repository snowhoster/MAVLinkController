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
#define PIN_ENGINE_SW  2

#define TFT_CS    10
#define TFT_DC     9
#define TFT_RST    8

// Switch edge codes (must match Python constants)
#define EDGE_NONE    0
#define EDGE_RISING  1   // switch OFF → ON  (ARM)
#define EDGE_FALLING 2   // switch ON  → OFF (DISARM)

// Control-authority codes (must match Python _AUTH_CODE)
#define AUTH_NONE    0
#define AUTH_PENDING 1
#define AUTH_GRANTED 2
#define AUTH_DENIED  3

// ── Globals ───────────────────────────────────────────────────────────────────
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

VesselStatus g_vessel = {0}; // 這行現在可以被編譯器正常解析了

int16_t  g_steering   = 0;
uint16_t g_left_thr   = 1000;
uint16_t g_right_thr  = 1000;
bool     g_sw_state   = false;

unsigned long t_last_send    = 0;
unsigned long t_last_display = 0;

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
                    int control_authority) {
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
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    pinMode(PIN_ENGINE_SW, INPUT_PULLUP);

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

    // 讀取類比搖桿資料
    g_steering  = (int16_t)map(analogRead(PIN_STEERING),  0, 1023, -1000, 1000);
    g_left_thr  = (uint16_t)map(analogRead(PIN_LEFT_THR),  0, 1023, 1000, 2000);
    g_right_thr = (uint16_t)map(analogRead(PIN_RIGHT_THR), 0, 1023, 1000, 2000);

    // 開關邊緣檢測
    bool sw = !digitalRead(PIN_ENGINE_SW);
    uint8_t sw_edge = EDGE_NONE;
    if (sw && !g_sw_state)       sw_edge = EDGE_RISING;
    else if (!sw && g_sw_state)  sw_edge = EDGE_FALLING;
    g_sw_state = sw;

    // 開關狀態改變時立即通知 Linux 系統 (ARM/DISARM)
    if (sw_edge != EDGE_NONE) {
        Bridge.notify("on_inputs",
            (int)g_steering, (int)g_left_thr, (int)g_right_thr,
            (int)g_sw_state, (int)sw_edge);
        t_last_send = now;
    }

    // 定時 10 Hz 傳送搖桿數據快照
    if (now - t_last_send >= 100) {
        Bridge.notify("on_inputs",
            (int)g_steering, (int)g_left_thr, (int)g_right_thr,
            (int)g_sw_state, (int)EDGE_NONE);
        t_last_send = now;
    }

    // 定時 5 Hz 刷新螢幕
    if (now - t_last_display >= 200) {
        lcd_update_dynamic(&tft, &g_vessel,
                           g_steering, g_left_thr, g_right_thr,
                           g_sw_state);
        t_last_display = now;
    }
}