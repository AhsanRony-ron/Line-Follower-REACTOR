#pragma once
#include <Arduino.h>
#include <stdarg.h>
#include <U8g2lib.h>
#include "config.h"
#include "types.h"

// ─────────────────────────────────────────
//  EXTERN
// ─────────────────────────────────────────
extern uint8_t  g_sensor_raw[SENSOR_COUNT];
extern uint8_t  g_sensor_thresh[SENSOR_COUNT];
extern int      g_LOUT;
extern int      g_ROUT;
extern uint8_t  g_counter_idx;
extern volatile int g_timer;
extern GlobalConfig g_config;
extern CheckpointParam g_checkpoint[CP_MAX];
extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;

float   voltage = read_voltage();
uint8_t pct     = batt_percent(voltage);

// forward declaration — definisi ada di sensor.h

// ─────────────────────────────────────────
//  KONSTANTA LAYOUT
//  128x64, text size 2 = 12x16px per char
//  → 10 char per baris, 4 baris maksimal
//  Kita pakai 4 baris aktif + 1 baris header
// ─────────────────────────────────────────
#define FONT_W    6
#define FONT_H    10         
#define FONT_BASE 10         

#define ROW(n)  ((n) * FONT_H + FONT_BASE)

// ─────────────────────────────────────────
//  INIT DISPLAY
// ─────────────────────────────────────────
void display_init() {
    u8g2.begin();    
    u8g2.clearBuffer();
    u8g2.setDrawColor(1);
    u8g2.setFont(u8g2_font_spleen6x12_mr);
    u8g2.sendBuffer();
}

// ─────────────────────────────────────────
//  HELPER DASAR
// ─────────────────────────────────────────

// tulis teks di posisi kolom (char) dan baris (0-based)
void disp_text(uint8_t col, uint8_t row, const char* str) {
    u8g2.setCursor(col * FONT_W, ROW(row));
    u8g2.print(str);
}

void disp_textf(uint8_t col, uint8_t row, const char* fmt, ...) {
    char buf[32];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    u8g2.setCursor(col * FONT_W, ROW(row));
    u8g2.print(buf);
}

// highlight block putih (invert) pada area tertentu
void disp_highlight(uint8_t row, bool active) {
    if (active) {
        u8g2.setDrawColor(1);
        u8g2.drawBox(0, ROW(row) - FONT_BASE + 2, OLED_WIDTH, FONT_H);
        u8g2.setDrawColor(0);  // teks berikutnya jadi hitam
    } else {
        u8g2.setDrawColor(1);  // teks normal putih
    }
}

// reset warna teks ke normal
void disp_color_reset() {
    u8g2.setDrawColor(1);
}

// ─────────────────────────────────────────
//  SENSOR BAR
//  Tampilkan bar 14 sensor secara horizontal
//  row: baris awal bar di OLED
// ─────────────────────────────────────────
// disp_sensor_bar: gambar bar 14 sensor
// y_top    = pixel paling atas area bar (misal 16)
// y_bottom = pixel paling bawah area bar (misal 47)
// bar tumbuh dari bawah ke atas, pakai fillRect (tidak bergaris)
void disp_sensor_bar(uint8_t y_top, uint8_t y_bottom) {

    for (uint8_t ii = 0; ii < SENSOR_COUNT; ii++) {

        uint8_t x =
            9 * (SENSOR_COUNT - 1 - ii);

        // SCALE LEBIH TINGGI
        uint8_t y_bar =
            map(
                g_sensor_raw[ii],
                0,
                150,
                y_bottom,
                y_top
            );

        for (int y = y_bottom; y >= y_bar; y--) {

            u8g2.drawHLine(
                x,
                y,
                8
            );
        }
    }
}

// sensor bar kalibrasi: bar + garis threshold atas & bawah
// area y=10..63
void disp_sensor_bar_calib(uint8_t high[SENSOR_COUNT], uint8_t low[SENSOR_COUNT]) {
    const uint8_t y_top    = 10;
    const uint8_t y_bottom = 63;
    const uint8_t area_h   = y_bottom - y_top;

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        // Display sensors in reverse order: sensor 0 at right, sensor 13 at left
        uint8_t x = (SENSOR_COUNT - 1 - i) * 9;

        // bar nilai saat ini — fillRect, tumbuh dari bawah
        uint8_t h     = (uint8_t)(((uint16_t)g_sensor_raw[i] * area_h) / 255);
        if (h > 0) u8g2.drawBox(x + 1, y_bottom - h, 6, h);

        // garis threshold atas (high = nilai ADC saat di hitam)
        uint8_t y_hi  = y_bottom - (uint8_t)(((uint16_t)high[i] * area_h) / 255);
        u8g2.drawHLine(x, y_hi, 8);

        // garis threshold bawah (low = nilai ADC saat di putih)
        uint8_t y_lo  = y_bottom - (uint8_t)(((uint16_t)low[i]  * area_h) / 255);
        u8g2.drawHLine(x, y_lo, 8);
    }
}

// ─────────────────────────────────────────
//  LAYAR STANDBY
//  Tampilkan CP, Battery, dan MEMORY SLOT AKTIF
// ─────────────────────────────────────────
// ┌─────────────────────┐
// │ Cp:02 Slot:1 B:12.3V│
// │                     │
// │      REACTOR        │
// │ |||||||||||||||     │
// └─────────────────────┘

void display_standby(uint8_t cp_idx, float voltage, uint8_t mem_slot = 0, GlobalConfig& cfg = g_config) {
    u8g2.clearBuffer();
    // Baris 0: CP aktif + counter berikutnya dari checkpoint
    u8g2.setCursor(0, ROW(0));
    u8g2.print("CP:");
    uint8_t cp_sel = cp_idx;
    if (cp_sel == 0) {
        u8g2.print("00");
    } else {
        if (cp_sel < 10) u8g2.print('0');
        u8g2.print(cp_sel);
    }

    u8g2.print(" N:");
    if (cp_sel == 0) {
        u8g2.print("C00");
    } else {
        const CheckpointParam& cp = g_checkpoint[cp_sel  - 1];
        if (cp.counter_pos == 0xFF) {
            u8g2.print("C00");
        } else {
            uint8_t next_counter = cp.counter_pos + 1;
            u8g2.print('C');
            if (next_counter < 10) u8g2.print('0');
            u8g2.print(next_counter);
        }
    }
    disp_textf(13, 0, cfg.mode == MODE_NORMAL ? "NORMAL" : "COUNTER");
    disp_textf(0, 1, "Slot:%d", mem_slot);
    disp_textf(13, 1, cfg.mirrored ? "MIRROR" : "");
    uint8_t pct     = batt_percent(voltage);

    int v_int  = (int)voltage;
    int v_frac = (int)((voltage - v_int) * 10);
    if (v_frac < 0) v_frac = -v_frac;

    disp_textf(0, 2, "B:%d.%dV %d%%", v_int, v_frac, pct);
    disp_text(7, 3, "REACTOR");
    disp_sensor_bar(16, 63);
    u8g2.sendBuffer();
}

// ─────────────────────────────────────────
//  LAYAR KALIBRASI
// ─────────────────────────────────────────
// ┌─────────────────────┐
// │ Calibrate           │
// │ [bar+garis batas]   │
// │ [bar+garis batas]   │
// │ [bar+garis batas]   │
// └─────────────────────┘

void display_kalibrasi(uint8_t high[SENSOR_COUNT], uint8_t low[SENSOR_COUNT]) {

    u8g2.clearBuffer();
    disp_text(0, 0, "Calibrate");
    disp_sensor_bar_calib(high, low);
    u8g2.sendBuffer();
}

// ─────────────────────────────────────────
//  LAYAR RUNNING
// ─────────────────────────────────────────
// ┌─────────────────────┐
// │ C:03  T:245  S:120  │
// │ Kp:10    00:12.3s   │
// │ |||||||||||||||     │
// │ L:120      R:085    │
// └─────────────────────┘

void display_running(uint8_t counter, int timer_val, uint8_t speed, uint8_t kp) {
    static unsigned long last_refresh = 0;
    if (millis() - last_refresh < 100) return;  // max 10fps
    last_refresh = millis();

    u8g2.clearBuffer();

    // y=0..7  : baris 0 — counter, timer, speed
    disp_textf(0, 0, "C:%02d T:%03d S:%03d", counter, timer_val, speed);
    disp_textf(0, 1, "Kp:%-3d  ", kp);

    // // y=16..47 : bar sensor (32px tinggi)
    // disp_sensor_bar(16, 47);

    // y=48..55 : baris 3 — PWM motor (ROW(6) = 48)
    u8g2.setCursor(0, 48);
    char buf[22];
    snprintf(buf, sizeof(buf), "L:%-4d     R:%-4d", g_LOUT, g_ROUT);
    u8g2.print(buf);

    u8g2.sendBuffer();
}

// ─────────────────────────────────────────
//  LAYAR STOP / FINISH
// ─────────────────────────────────────────
void display_finish(unsigned long elapsed_ms) {
    uint8_t  min_val = (uint8_t)(elapsed_ms / 60000);
    uint8_t  sec_val = (uint8_t)((elapsed_ms % 60000) / 1000);
    uint16_t ms_val  = (uint16_t)(elapsed_ms % 1000) / 10;

    u8g2.clearBuffer();

    u8g2.setFont(u8g2_font_spleen8x16_mr);  // set font DULU
    disp_text(2, 0, "FINISH!");

    u8g2.setFont(u8g2_font_spleen6x12_mr);   // kembali ke font normal
    disp_text(0, 2, "Time:");
    disp_textf(0, 3, "%02d:%02d.%02ds", min_val, sec_val, ms_val);

    u8g2.sendBuffer();
}

// ─────────────────────────────────────────
//  MENU 1 — KONFIGURASI GLOBAL
//  Scroll list, 4 baris, highlight aktif
// ─────────────────────────────────────────

// item menu 1
static const char* MENU1_LABELS[] = {
    "Calibrate", "Mode", "Speed", "PID", "Line",
    "Periode", "T-Blank", "PWM_F", "Mirror",
    "H-Check", "M_Slot"
};

#define MENU1_COUNT 11

void display_menu1(uint8_t scroll, uint8_t highlight, GlobalConfig& cfg, uint8_t edit_sub = 0) {
    u8g2.clearBuffer();

    for (uint8_t i = 0; i < 6; i++) {
        uint8_t idx = scroll + i;
        if (idx >= MENU1_COUNT) break;

        bool active = (idx == highlight);
        disp_highlight(i, active);

        // label
        u8g2.setCursor(0, ROW(i));
        u8g2.print(MENU1_LABELS[idx]);
        u8g2.print(":");

        // nilai
        u8g2.setCursor(9 * FONT_W, ROW(i));
        switch (idx) {
            case 0: u8g2.print(">>"); break;
            case 1: u8g2.print(cfg.mode == MODE_NORMAL ? "NORMAL" : "COUNTER"); break;
            case 2: u8g2.print(cfg.speed_mode); break;
            case 3: {
                // PID Kp & Kd dengan [] highlight untuk sub yang aktif
                if (active && edit_sub == 0) u8g2.print("[");
                u8g2.print("Kp:");
                u8g2.print(cfg.kp);
                if (active && edit_sub == 0) u8g2.print("]");
                
                u8g2.print(" ");
                
                if (active && edit_sub == 1) u8g2.print("[");
                u8g2.print("Kd:");
                u8g2.print(cfg.kd);
                if (active && edit_sub == 1) u8g2.print("]");
                break;
            }
            case 4:
                if (cfg.line == LINE_AUTO)       u8g2.print("AUTO");
                else if (cfg.line == LINE_HITAM) u8g2.print("HITAM");
                else                             u8g2.print("PUTIH");
                break;
            case 5: u8g2.print(cfg.periode); u8g2.print("ms"); break;
            case 6: u8g2.print(cfg.t_blank * 10); u8g2.print("ms"); break;
            case 7: u8g2.print(cfg.pwm_freq_khz, 1); u8g2.print("kHz"); break;
            case 8: u8g2.print(cfg.mirrored ? "YES" : "NO"); break;
            case 9: u8g2.print(">>"); break;
            case 10: 
            // ✅ Tampilkan mem_slot dengan label jelas
                u8g2.print("Slot:");
                u8g2.print(cfg.mem_slot);
                u8g2.print("/");
                u8g2.print(EEPROM_SLOT_COUNT - 1);
                break;
            
        }

        disp_color_reset();
    }

    u8g2.sendBuffer();
}

// ─────────────────────────────────────────
//  MENU 2 — PILIHAN UTAMA
// ─────────────────────────────────────────

static const char* MENU2_LABELS[] = {
    "Counter", "Check Point", "Copy Mem", "Delete", "Reset"
};
#define MENU2_COUNT 5

void display_menu2(uint8_t scroll, uint8_t highlight) {
    u8g2.clearBuffer();

    for (uint8_t i = 0; i < 6; i++) {
        uint8_t idx = scroll + i;
        if (idx >= MENU2_COUNT) break;

        bool active = (idx == highlight);
        disp_highlight(i, active);
        u8g2.setCursor(0, ROW(i));
        u8g2.print(MENU2_LABELS[idx]);
        disp_color_reset();
    }

    u8g2.sendBuffer();
}

// ─────────────────────────────────────────
//  TRIGGER NAME HELPER (lokal, tanpa depend menu.h)
// ─────────────────────────────────────────
static const uint16_t _TRIG_VAL[] = {
    (1<<9),(1<<10),(1<<11),(1<<12),(1<<13),
    (1<<4),(1<<3),(1<<2),(1<<1),(1<<0),
    (1<<9)|(1<<4),(1<<10)|(1<<3),(1<<11)|(1<<2),(1<<12)|(1<<1),(1<<13)|(1<<0),
    TRIGGER_BLANK,
    TRIGGER_TIMER,
    TRIGGER_TICK   // ← sekarang nilai beda, tidak tabrakan
};
static const char* _TRIG_NAME[] = {
    "L1","L2","L3","L4","L5",
    "R1","R2","R3","R4","R5",
    "LR1","LR2","LR3","LR4","LR5",
    "BLK","TMR","ENC"
};
static const uint8_t _TRIG_COUNT = 18;

const char* disp_trigger_name(uint16_t trigger) {
    for (uint8_t i = 0; i < _TRIG_COUNT; i++) {
        if (_TRIG_VAL[i] == trigger) return _TRIG_NAME[i];
    }
    return "???";
}

// ─────────────────────────────────────────
//  MENU 2 — DETAIL COUNTER
//  Header C:XXX fixed di baris 0
//  3 baris scroll di bawahnya
// ─────────────────────────────────────────

// item counter (urutan scroll)
// 0: Decision + Trigger
// 1: Timer
// 2: Speed1 + Speed2
// 3: Delay type + durasi
// 4: Kp
// 5: Belok PWM L + R
// 6: Encoder Values
// 7: Line Counter Mode (C1+)
#define COUNTER_ITEM_COUNT 8

void display_counter(uint8_t counter_idx, uint8_t scroll, uint8_t highlight,
                     CounterParam& p, bool edit_mode = false, uint8_t edit_sub = 0, GlobalConfig& cfg = g_config) {
    u8g2.clearBuffer();

    // baris 0: header fixed, highlight jika highlight==0
    if (highlight == 0) disp_highlight(0, true);
    disp_textf(0, 0, "C:%03d", counter_idx);
    if (highlight == 0) disp_color_reset();

    // baris 1-5: parameter scroll
    // C0 khusus: hanya Timer(1), Speed(2), Kp(4) → remapping ke index 0,1,2
    // C1+: semua item normal tanpa free PWM terpisah
    bool is_c0 = (counter_idx == 0);
    uint8_t max_idx = is_c0 ? 4 : COUNTER_ITEM_COUNT;

    // lookup index item C0: 0→Timer(1), 1→Speed(2), 2→Kp(4), 3→Encoder(6)
    const uint8_t c0_map[] = {1, 2, 4, 6}; 

    for (uint8_t i = 0; i < 6; i++) {
        uint8_t idx_raw = scroll + i;
        if (idx_raw >= max_idx) break;

        // remapping index untuk C0
        uint8_t idx = is_c0 ? c0_map[idx_raw] : idx_raw;

        if (!is_c0 && idx >= COUNTER_ITEM_COUNT) break;

        uint8_t row = i + 1;
        bool active = (highlight > 0) && (idx_raw == highlight - 1);  // pakai idx_raw bukan idx
        disp_highlight(row, active);
        u8g2.setCursor(0, ROW(row));

        char buf[22];
        switch (idx) {
            case 0:
                // decision + trigger di pojok kanan
                u8g2.print("Dec:");
                if (edit_mode && active && edit_sub == 0) u8g2.print("[");
                switch (p.decision) {
                    case DEC_LOST:        u8g2.print("LOST"); break;
                    case DEC_FREE:        u8g2.print("FREE"); break;
                    case DEC_BELOK_KIRI:  u8g2.print("LEFT"); break;
                    case DEC_BELOK_KANAN: u8g2.print("RIGHT"); break;
                    case DEC_STOP:        u8g2.print("STOP"); break;
                }
                if (edit_mode && active && edit_sub == 0) u8g2.print("]");
                // trigger di pojok kanan
                u8g2.setCursor(80, ROW(row));
                if (edit_mode && active && edit_sub == 1) u8g2.print("[");
                u8g2.print(disp_trigger_name(p.trigger));
                if (edit_mode && active && edit_sub == 1) u8g2.print("]");
                break;
            case 1: {
                char tbuf[22];
                snprintf(tbuf, sizeof(tbuf), "Timer:%3dx %3dms", p.timer, cfg.periode);
                u8g2.setCursor(0, ROW(row));
                u8g2.print(tbuf);
                break;
            }
            case 2:
                u8g2.print("Spd:");
                if (edit_mode && active && edit_sub == 0) u8g2.print("[");
                u8g2.print(p.speed1);
                if (edit_mode && active && edit_sub == 0) u8g2.print("]");
                u8g2.print(" ");
                if (edit_mode && active && edit_sub == 1) u8g2.print("[");
                u8g2.print(p.speed2);
                if (edit_mode && active && edit_sub == 1) u8g2.print("]");
                break;
            case 3:
                u8g2.print("Dly:");
                if (edit_mode && active && edit_sub == 0) u8g2.print("[");
                u8g2.print(p.delay_type == DELAY_A ? 'A' : 'B');
                if (edit_mode && active && edit_sub == 0) u8g2.print("]");
                u8g2.print(" ");
                if (edit_mode && active && edit_sub == 1) u8g2.print("[");
                if (p.Encd_b > 0) {
                    snprintf(buf, sizeof(buf), "E:%4d", p.Encd_b);
                } else {
                    snprintf(buf, sizeof(buf), "T:%4dms", p.delay_ms);
                }
                u8g2.print(buf);
                if (edit_mode && active && edit_sub == 1) u8g2.print("]");
                break;
            case 4: {
                char kbuf[22];
                snprintf(kbuf, sizeof(kbuf), "Kp: %-3d", p.kp);
                u8g2.setCursor(0, ROW(row));
                u8g2.print(kbuf);
                break;
            }
            case 5:
                u8g2.print("Blk:");
                if (edit_mode && active && edit_sub == 0) u8g2.print("[");
                snprintf(buf, sizeof(buf), "L:%-4d", p.belok_l);
                u8g2.print(buf);
                if (edit_mode && active && edit_sub == 0) u8g2.print("]");
                u8g2.print(" ");
                if (edit_mode && active && edit_sub == 1) u8g2.print("[");
                snprintf(buf, sizeof(buf), "R:%-4d", p.belok_r);
                u8g2.print(buf);
                if (edit_mode && active && edit_sub == 1) u8g2.print("]");
                break;
            case 6: {
                int16_t tick_val  = (p.Encd_r > 0) ? p.Encd_r : p.Encd_l;
                char    tick_side = (p.Encd_r > 0) ? 'R' : 'L';

                u8g2.print("Tick:");
                snprintf(buf, sizeof(buf), "%c:%-4d", tick_side, tick_val);
                u8g2.print(buf);
                break;
            }
            case 7:
                u8g2.print("Line:");
                switch (p.Line_C) {
                    case AUTO_NORMAL : u8g2.print("AUTO_NORMAL"); break;
                    case AUTO_CENTER : u8g2.print("AUTO_CENTER"); break;
                    case AUTO_LEFT   : u8g2.print("AUTO_LEFT"); break;
                    case AUTO_RIGHT  : u8g2.print("AUTO_RIGHT"); break;
                    case BLACK_NORMAL: u8g2.print("BLACK_NORMAL"); break;
                    case BLACK_CENTER: u8g2.print("BLACK_CENTER"); break;
                    case BLACK_LEFT: u8g2.print("BLACK_LEFT"); break;
                    case BLACK_RIGHT: u8g2.print("BLACK_RIGHT"); break;
                    case WHITE_NORMAL: u8g2.print("WHITE_NORMAL"); break;
                    case WHITE_CENTER: u8g2.print("WHITE_CENTER"); break;
                    case WHITE_LEFT: u8g2.print("WHITE_LEFT"); break;
                    case WHITE_RIGHT: u8g2.print("WHITE_RIGHT"); break;
                }
                break;
        }

        disp_color_reset();
    }

    u8g2.sendBuffer();
}

// ─────────────────────────────────────────
//  CHECKPOINT MENU
//  Tampil 5 checkpoint per layar, highlight aktif + subfield edit
// ─────────────────────────────────────────
void display_checkpoint(uint8_t scroll, uint8_t highlight, uint8_t edit_sub = 0) {
    u8g2.clearBuffer();
    disp_text(0, 0, "CheckPoint");

    for (uint8_t i = 0; i < 5; i++) {
        uint8_t idx = scroll + i;
        if (idx >= CP_MAX) break;

        bool active = (idx == highlight);
        disp_highlight(i + 1, active);

        u8g2.setCursor(0, ROW(i + 1));
        u8g2.print("CP");
        if (idx < 10) u8g2.print('0');
        u8g2.print(idx + 1);
        u8g2.print(": ");

        CheckpointParam& cp = g_checkpoint[idx];

        if (active && edit_sub == 0) u8g2.print('[');
        u8g2.print("C:");
        if (cp.counter_pos == 0xFF) {
            u8g2.print("--");
        } else {
            if (cp.counter_pos < 10) u8g2.print('0');
            u8g2.print(cp.counter_pos + 1);
        }
        if (active && edit_sub == 0) u8g2.print(']');

        u8g2.print(" ");

        if (active && edit_sub == 1) u8g2.print('[');
        u8g2.print("T:");
        u8g2.print(cp.timer_cp);
        u8g2.print("0ms");
        if (active && edit_sub == 1) u8g2.print(']');

        disp_color_reset();
    }

    u8g2.sendBuffer();
}

// ─────────────────────────────────────────
//  KONFIRMASI (delete, reset, copy)
// ─────────────────────────────────────────
void display_confirm(const char* msg) {
    u8g2.clearBuffer();
    disp_text(0, 0, msg);
    disp_text(0, 2, "SAVE: Ya");
    disp_text(0, 3, "X   : Batal");
    u8g2.sendBuffer();
}

// ─────────────────────────────────────────
//  PESAN SINGKAT (saving, done, error)
// ─────────────────────────────────────────
void display_msg(const char* line1, const char* line2 = nullptr) {
    u8g2.clearBuffer();
    disp_text(0, 1, line1);
    if (line2) disp_text(0, 2, line2);
    u8g2.sendBuffer();
}