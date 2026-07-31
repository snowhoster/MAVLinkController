#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <math.h>
#include "chinese_fonts.h"

// RGB565 colors
#define C_BLACK     0x0000
#define C_WHITE     0xFFFF
#define C_GREEN     0x07E0
#define C_RED       0xF800
#define C_YELLOW    0xFFE0
#define C_CYAN      0x07FF
#define C_ORANGE    0xFD20
#define C_GRAY      0x7BEF
#define C_DARKGRAY  0x39E7
#define C_BG        0x0841  // Dark navy background
#define C_HEADER    0x0210  // Dark blue header

// USV display modes
#define MODE_MANUAL  0
#define MODE_GUIDED  1
#define MODE_AUTO    2
#define MODE_HOLD    3

// 定義船體狀態結構體
struct VesselStatus {
    uint16_t speed_x10;     // knots * 10
    uint16_t heading;       // degrees 0–359
    uint8_t  battery_pct;   // 0–100
    uint8_t  gps_fix;       // 0=none 2=2D 3=3D (colors the GPS position row)
    bool     armed;
    uint8_t  mode;          // see MODE_* constants
    int32_t  lat_degE7;     // GPS latitude  * 1e7
    int32_t  lon_degE7;     // GPS longitude * 1e7
    char     remote_ip[16]; // 船舶 (USV) IP
    uint16_t remote_port;
    char     local_ip[16];  // 本機 (MPU) IP
    uint16_t local_port;
    uint8_t  control_authority; // 0=none 1=pending 2=granted 3=denied (header badge)
};

// ── UTF-8 Drawing Helper ─────────────────────────────────────────────────────
inline void draw_utf8_string(Adafruit_ILI9341* tft, int16_t x, int16_t y, const char* str, uint16_t color, uint16_t bg) {
    int16_t cur_x = x;
    int16_t cur_y = y;
    while (*str) {
        uint8_t c = (uint8_t)*str;
        uint32_t unicode = 0;
        int len = 0;
        
        if (c < 0x80) {
            unicode = c;
            len = 1;
        } else if ((c & 0xE0) == 0xC0) {
            unicode = ((c & 0x1F) << 6) | ((uint8_t)str[1] & 0x3F);
            len = 2;
        } else if ((c & 0xF0) == 0xE0) {
            unicode = ((c & 0x0F) << 12) | (((uint8_t)str[1] & 0x3F) << 6) | ((uint8_t)str[2] & 0x3F);
            len = 3;
        } else if ((c & 0xF8) == 0xF0) {
            unicode = ((c & 0x07) << 18) | (((uint8_t)str[1] & 0x3F) << 12) | (((uint8_t)str[2] & 0x3F) << 6) | ((uint8_t)str[3] & 0x3F);
            len = 4;
        } else {
            str++;
            continue;
        }
        
        str += len;
        
        int idx = get_char_index(unicode);
        if (idx >= 0) {
            tft->fillRect(cur_x, cur_y, 16, 16, bg);
            tft->drawBitmap(cur_x, cur_y, ch_font_bitmaps[idx], 16, 16, color);
            cur_x += 16;
        } else {
            if (unicode >= 32 && unicode <= 126) {
                tft->drawChar(cur_x, cur_y, (char)unicode, color, bg, 2);
                cur_x += 12;
            }
        }
    }
}

// ── Helpers ──────────────────────────────────────────────────────────────────
static void draw_hbar(Adafruit_ILI9341* tft,
                      int x, int y, int w, int h,
                      int value, int vmin, int vmax,
                      uint16_t fill_color) {
    int filled = constrain(map(value, vmin, vmax, 0, w), 0, w);
    if (filled > 0) {
        tft->fillRect(x, y, filled, h, fill_color);
    }
    if (w - filled > 0) {
        tft->fillRect(x + filled, y, w - filled, h, C_DARKGRAY);
    }
}

static void draw_vbar(Adafruit_ILI9341* tft,
                      int x, int y, int w, int h,
                      int value, int vmin, int vmax,
                      uint16_t fill_color) {
    int filled = constrain(map(value, vmin, vmax, 0, h), 0, h);
    if (filled > 0) {
        tft->fillRect(x, y + h - filled, w, filled, fill_color);
    }
    if (h - filled > 0) {
        tft->fillRect(x, y, w, h - filled, C_DARKGRAY);
    }
}

static uint16_t battery_color(uint8_t pct) {
    if (pct > 50) return C_GREEN;
    if (pct > 20) return C_YELLOW;
    return C_RED;
}

// Compact ASCII-only text (IP:Port strings) at half height, so long strings still fit their row.
static void draw_small_ascii(Adafruit_ILI9341* tft, int16_t x, int16_t y, const char* str, uint16_t color, uint16_t bg) {
    tft->setTextSize(1);
    tft->setTextColor(color, bg);
    tft->setCursor(x, y);
    tft->print(str);
}

static const char* ch_mode_str(uint8_t m) {
    switch (m) {
        case MODE_MANUAL: return "手動";
        case MODE_GUIDED: return "引導";
        case MODE_AUTO:   return "自動";
        case MODE_HOLD:   return "保持";
        default:          return "手動";
    }
}

// ── Initialise & static chrome ────────────────────────────────────────────────
inline void lcd_init(Adafruit_ILI9341* tft, uint32_t spi_hz = 8000000) {
    tft->begin(spi_hz);
    tft->setRotation(1);  // landscape 320×240
    tft->fillScreen(C_BG);
}

inline void lcd_draw_static(Adafruit_ILI9341* tft) {
    // Header bar
    tft->fillRect(0, 0, 320, 20, C_HEADER);
    draw_utf8_string(tft, 80, 2, "無人船遠端遙控器", C_CYAN, C_HEADER);

    // Dividers
    tft->drawFastHLine(0, 20, 320, C_DARKGRAY);    // below header
    tft->drawFastVLine(160, 20, 220, C_DARKGRAY);  // splits left and right columns

    // Left telemetry column horizontal grid lines
    for (int y = 44; y < 240; y += 24) {
        tft->drawFastHLine(0, y, 160, C_DARKGRAY);
    }

    // Static row labels in Chinese (Left column)
    draw_utf8_string(tft, 4, 24, "速度", C_GRAY, C_BG);
    draw_utf8_string(tft, 4, 48, "航向", C_GRAY, C_BG);
    draw_utf8_string(tft, 4, 72, "電量", C_GRAY, C_BG);
    draw_utf8_string(tft, 4, 96, "GPS位置", C_GRAY, C_BG);
    draw_utf8_string(tft, 4, 120, "模式", C_GRAY, C_BG);
    draw_utf8_string(tft, 4, 144, "通訊", C_GRAY, C_BG);
    draw_utf8_string(tft, 4, 168, "引擎", C_GRAY, C_BG);
    draw_utf8_string(tft, 4, 192, "本機", C_GRAY, C_BG);
    draw_utf8_string(tft, 4, 216, "舵角", C_GRAY, C_BG);

    // Static boat labels (Right column)
    draw_utf8_string(tft, 170, 45, "左油門", C_GRAY, C_BG);
    draw_utf8_string(tft, 270, 45, "右油門", C_GRAY, C_BG);
    draw_utf8_string(tft, 208, 170, "方向盤", C_GRAY, C_BG);

    // Dynamic vertical throttle bar slots
    tft->drawRect(185, 70, 10, 70, C_DARKGRAY);
    tft->drawRect(285, 70, 10, 70, C_DARKGRAY);
}

// ── Dynamic update ────────────────────────────────────────────────────────────
inline void lcd_update_dynamic(Adafruit_ILI9341* tft,
                               const VesselStatus* vs,
                               int16_t  steering,
                               uint16_t l_thr,
                               uint16_t r_thr,
                               bool     sw_state) {
    char buf[24];

    // ── Header: control-authority badge (dot) ─────────────────────────────────
    // 0=none(gray) 1=pending(yellow) 2=granted(green) 3=denied(red)
    uint16_t auth_color;
    switch (vs->control_authority) {
        case 2:  auth_color = C_GREEN;  break;
        case 1:  auth_color = C_YELLOW; break;
        case 3:  auth_color = C_RED;    break;
        default: auth_color = C_GRAY;   break;
    }
    tft->fillCircle(308, 10, 6, auth_color);

    // ── Left Column Telemetry Update ──────────────────────────────────────────
    // Row 1: Speed
    snprintf(buf, sizeof(buf), "%d.%d節", vs->speed_x10 / 10, vs->speed_x10 % 10);
    tft->fillRect(45, 24, 110, 16, C_BG);
    draw_utf8_string(tft, 45, 24, buf, C_WHITE, C_BG);

    // Row 2: Heading
    snprintf(buf, sizeof(buf), "%03d度", vs->heading);
    tft->fillRect(45, 48, 110, 16, C_BG);
    draw_utf8_string(tft, 45, 48, buf, C_WHITE, C_BG);

    // Row 3: Battery
    draw_hbar(tft, 45, 75, 75, 10, vs->battery_pct, 0, 100, battery_color(vs->battery_pct));
    tft->fillRect(125, 72, 32, 16, C_BG);
    snprintf(buf, sizeof(buf), "%d%%", vs->battery_pct);
    draw_utf8_string(tft, 125, 72, buf, C_WHITE, C_BG);

    // Row 4: GPS Position (label is wider — content starts further right)
    tft->fillRect(80, 96, 76, 16, C_BG);
    if (vs->gps_fix < 2) {
        draw_utf8_string(tft, 80, 96, "無定位", C_RED, C_BG);
    } else {
        uint16_t fix_color = (vs->gps_fix >= 3) ? C_GREEN : C_YELLOW;
        int lat_deg  = vs->lat_degE7 / 10000000;
        int lat_frac = abs((int)(vs->lat_degE7 % 10000000)) / 100000;
        int lon_deg  = vs->lon_degE7 / 10000000;
        int lon_frac = abs((int)(vs->lon_degE7 % 10000000)) / 100000;
        snprintf(buf, sizeof(buf), "%d.%02d,%d.%02d", lat_deg, lat_frac, lon_deg, lon_frac);
        draw_small_ascii(tft, 80, 100, buf, fix_color, C_BG);
    }

    // Row 5: Mode
    tft->fillRect(45, 120, 110, 16, C_BG);
    draw_utf8_string(tft, 45, 120, ch_mode_str(vs->mode), C_CYAN, C_BG);

    // Row 6: Comms — vessel (remote) IP:Port
    tft->fillRect(45, 144, 113, 16, C_BG);
    snprintf(buf, sizeof(buf), "%s:%u", vs->remote_ip, vs->remote_port);
    draw_small_ascii(tft, 45, 148, buf, C_WHITE, C_BG);

    // Row 7: Engine Switch
    tft->fillRect(45, 168, 110, 16, C_BG);
    draw_utf8_string(tft, 45, 168, vs->armed ? "啟動" : "關閉", vs->armed ? C_GREEN : C_RED, C_BG);

    // Row 8: Local (MPU) IP:Port
    tft->fillRect(45, 192, 113, 16, C_BG);
    snprintf(buf, sizeof(buf), "%s:%u", vs->local_ip, vs->local_port);
    draw_small_ascii(tft, 45, 196, buf, C_WHITE, C_BG);

    // Row 9: Rudder angle (degrees, max deflection ±45°)
    int steer_deg = (int)(steering * 45L / 1000L);
    tft->fillRect(45, 216, 110, 16, C_BG);
    snprintf(buf, sizeof(buf), "%+d度", steer_deg);
    draw_utf8_string(tft, 45, 216, buf, C_WHITE, C_BG);

    // ── Right Column: Throttles, Steering & Vessel Graphics ──────────────────
    // 1. Vertical Throttle Bars
    int l_pct = map(l_thr, 1000, 2000, 0, 100);
    int r_pct = map(r_thr, 1000, 2000, 0, 100);

    draw_vbar(tft, 185, 70, 10, 70, l_thr, 1000, 2000, C_ORANGE);
    tft->fillRect(170, 145, 40, 16, C_BG);
    snprintf(buf, sizeof(buf), "%d%%", l_pct);
    draw_utf8_string(tft, 175, 145, buf, C_WHITE, C_BG);

    draw_vbar(tft, 285, 70, 10, 70, r_thr, 1000, 2000, C_ORANGE);
    tft->fillRect(270, 145, 40, 16, C_BG);
    snprintf(buf, sizeof(buf), "%d%%", r_pct);
    draw_utf8_string(tft, 275, 145, buf, C_WHITE, C_BG);

    // 2. Dynamic Steering Numeric Display
    tft->fillRect(205, 188, 70, 16, C_BG);
    snprintf(buf, sizeof(buf), "%+d", steering);
    draw_utf8_string(tft, 210, 188, buf, C_WHITE, C_BG);

    // 3. Vessel Outline Graphic
    uint16_t boat_color = vs->armed ? C_GREEN : C_RED;
    
    tft->drawLine(240, 70, 222, 95, boat_color);  // Bow Left
    tft->drawLine(240, 70, 258, 95, boat_color);  // Bow Right
    tft->drawLine(222, 95, 222, 140, boat_color); // Left Side
    tft->drawLine(258, 95, 258, 140, boat_color); // Right Side

    // 4. Rudder Line drawing & erasing
    tft->fillRect(220, 140, 40, 22, C_BG); // Erase old rudder area
    tft->drawLine(222, 140, 258, 140, boat_color); // Restore Stern line
    
    // Calculate rudder line end point based on steering angle (steer_deg, ±45° max)
    float angle_rad = steer_deg * DEG_TO_RAD;
    int rx = 240 + (int)(sin(angle_rad) * 15.0);
    int ry = 140 + (int)(cos(angle_rad) * 15.0);
    
    tft->drawLine(240, 140, rx, ry, C_CYAN); // Draw rudder
}

#endif  // LCD_DISPLAY_H