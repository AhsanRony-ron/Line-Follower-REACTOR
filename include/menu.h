#pragma once
#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "hardware.h"
#include "display.h"
#include "eeprom.h"
#include "sensor.h"

// ─────────────────────────────────────────
//  EXTERN
// ─────────────────────────────────────────
extern GlobalConfig     g_config;
extern CounterParam     g_counter[COUNTER_MAX];
extern CheckpointParam  g_checkpoint[CP_MAX];
extern uint8_t          g_sensor_thresh[SENSOR_COUNT];
extern volatile uint16_t g_periodo_isr;  // volatile copy dari g_config.periode untuk ISR
extern volatile int32_t countKiri;
extern volatile int32_t countKanan;

static int32_t lastEnc_r = 0;
static int32_t lastEnc_l = 0;
static int32_t lastEnc = 0;

// forward declarations
void menu1();
void menu2();
void run_kalibrasi();
void run_hcheck();

// ─────────────────────────────────────────
//  NAVIGASI HELPER
//  wrap=true: mentok bawah lompat ke atas (dan sebaliknya)
//  visible: jumlah baris yang tampil sekaligus
// ─────────────────────────────────────────
void nav_up(uint8_t& highlight, uint8_t& scroll, uint8_t count, uint8_t visible, bool wrap = true) {
    if (highlight > 0) {
        highlight--;
        if (highlight < scroll) scroll--;
    } else if (wrap) {
        highlight = count - 1;
        scroll = (count > visible) ? count - visible : 0;
    }
}

void nav_down(uint8_t& highlight, uint8_t& scroll, uint8_t count, uint8_t visible, bool wrap = true) {
    if (highlight < count - 1) {
        highlight++;
        if (highlight > scroll + visible - 1) scroll++;
    } else if (wrap) {
        highlight = 0;
        scroll = 0;
    }
}


// ─────────────────────────────────────────
//  DEBOUNCE HELPER
// ─────────────────────────────────────────
void wait_release() {
    delay(20);
    while (sw_next() || sw_back() || sw_up() || sw_down() || sw_save() || sw_x());
    delay(20);
}

// ─────────────────────────────────────────
// BUTTON AUTO REPEAT
// ─────────────────────────────────────────

bool button_repeat(bool state,
                   uint32_t& last_time,
                   bool& hold_state,
                   uint16_t first_delay = 300,
                   uint16_t repeat_delay = 30)
{
    uint32_t now = millis();

    if (state && !hold_state) {
        // tombol baru ditekan → trigger langsung
        hold_state = true;
        last_time  = now;
        return true;
    }

    if (state && hold_state) {
        // tombol ditahan → tunggu first_delay, lalu repeat setiap repeat_delay
        if (now - last_time >= first_delay) {
            last_time = now - (first_delay - repeat_delay);
            return true;
        }
    }

    if (!state) {
        hold_state = false;
    }

    return false;
}

bool btn_up() {
    static uint32_t t = 0;
    static bool hold = false;
    return button_repeat(sw_up(), t, hold);
}

bool btn_down() {
    static uint32_t t = 0;
    static bool hold = false;
    return button_repeat(sw_down(), t, hold);
}

bool btn_next() {
    static uint32_t t = 0;
    static bool hold = false;
    return button_repeat(sw_next(), t, hold);
}

bool btn_back() {
    static uint32_t t = 0;
    static bool hold = false;
    return button_repeat(sw_back(), t, hold);
}

bool btn_save() {
    static bool last = false;

    bool now = sw_save();

    if (now && !last) {
        last = true;
        return true;
    }

    if (!now)
        last = false;

    return false;
}

bool btn_x() {
    static bool last = false;

    bool now = sw_x();

    if (now && !last) {
        last = true;
        return true;
    }

    if (!now)
        last = false;

    return false;
}

// ─────────────────────────────────────────
//  LAYAR STANDBY
//  return: index CP yang dipilih (0 = mulai dari awal)
// ─────────────────────────────────────────
uint8_t screen_standby() {
    uint8_t cp_sel = 0;
    unsigned long last_disp = 0;

    // flush semua tombol saat masuk standby
    wait_release();

    while (true) {
        unsigned long now = millis();
        bool just_displayed = false;

        if (now - last_disp >= 50) {
            scan_sensor_bar();
            float volt = read_voltage();
            display_standby(cp_sel, volt, g_config.mem_slot);
            last_disp = now;
            just_displayed = true;
        }

        if (!just_displayed) {
            if (btn_up())   { if (cp_sel < CP_MAX) cp_sel++; }
            if (btn_down()) { if (cp_sel > 0) cp_sel--; }
        }

        // SAVE = START
        if (btn_save()) {
            wait_release();   // flush agar tidak langsung stop di mode running
            u8g2.clearBuffer();
            u8g2.sendBuffer();
            return cp_sel;
        }

        // X = kalibrasi sensor
        if (btn_x()) {
            run_kalibrasi();
            wait_release();
        }

        // NEXT = Menu 1 (Config)
        if (btn_next()) {
            menu1();
            wait_release();
        }

        // BACK = Menu 2 (Counter)
        if (btn_back()) {
            menu2();
            wait_release();
        }
    }
}

// ─────────────────────────────────────────
//  KALIBRASI SENSOR
// ─────────────────────────────────────────
#define SENSOR_OFFSET_PCT 50  // naikkan untuk redam noise, turunkan untuk sensitifitas

void run_kalibrasi() {
    uint8_t high[SENSOR_COUNT] = {0};
    uint8_t low[SENSOR_COUNT];
    uint8_t low_offset[SENSOR_COUNT] = {0};  // ← tambah array ini
    memset(low, 255, sizeof(low));

    while (true) {
        for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
            uint8_t val = read_adc(i);
            g_sensor_raw[i] = val;
            if (val > high[i]) high[i] = val;
            if (val < low[i])  low[i]  = val;

            uint8_t range = high[i] - low[i];
            uint8_t offset = (uint8_t)((uint16_t)range * SENSOR_OFFSET_PCT / 100);
            low_offset[i] = low[i] + offset;  // ← simpan per sensor

            g_sensor_thresh[i] = (high[i] - low_offset[i]) / 2 + low_offset[i];
        }

        display_kalibrasi(high, low_offset);  // ← pass low_offset bukan low

        if (btn_save()) {
            eeprom_save_sensor(g_config.mem_slot);
            display_msg("Saved!", "Calibration OK");
            delay(1000);
            return;
        }
    }
}

// ─────────────────────────────────────────
//  HCHECK — TEST MOTOR
// ─────────────────────────────────────────
void run_hcheck() {
    display_msg("Hcheck", "SAVE:maju X:stop");
    delay(500);

    while (true) {
        if (btn_x()) {

            display_msg("Hcheck", "Encoder");
            char b1[32], b2[32];
            sprintf(b1, "Kiri:  %ld", encoderKiriRead());
            sprintf(b2, "Kanan: %ld", encoderKananRead());
            display_msg(b1, b2);
        }
        if (btn_save()) {
            motor_stop();
            display_msg("Hcheck", "STOP");
            delay(500);
            return;
        }
        if (btn_up())   { set_motors(150,  150, DEFAULT_MAX_PWM); display_msg("Hcheck", "MAJU...");}
        if (btn_down()) { set_motors(-100, -100, DEFAULT_MAX_PWM); display_msg("Hcheck", "MUNDUR..."); }
        if (btn_next()) { set_motors(100, -100, DEFAULT_MAX_PWM); display_msg("Hcheck","Putar KA"); }
        if (btn_back()) { set_motors(-100, 100, DEFAULT_MAX_PWM); display_msg("Hcheck","Putar KI"); }
    }
}

// ─────────────────────────────────────────
//  KONFIRMASI GENERIC
//  return true jika SAVE, false jika X
// ─────────────────────────────────────────
bool confirm(const char* msg) {
    display_confirm(msg);
    while (true) {
        if (btn_save()) return true;
        if (btn_x())    return false;
    }
}

// ─────────────────────────────────────────
//  EDIT NILAI GENERIC
//  Naik/turun dengan up/down, keluar dengan X
//  min_val / max_val: batas nilai
// ─────────────────────────────────────────
template<typename T>
void edit_value(T& val, T step, T min_val, T max_val) {
    if (btn_next()) {
        val += step;
        if (val > max_val) val = min_val;  // wrap around to min
    }
    if (btn_back()) {
        // Cek batas SEBELUM operasi untuk mencegah wrap around hardware
        if (val > min_val) {
            if (val - step < min_val) {
                val = min_val;
            } else {
                val -= step;
            }
        } else if (val == min_val) {
            val = max_val;  // wrap around to max
        }
    }
}

// ─────────────────────────────────────────
//  ENCODE / DECODE TRIGGER
//  Trigger bisa: L1-L5, R1-R5, LR1-LR5, Blank, Timer
//  Dengan bit order reversed:
//    bit 0-4   = R5-R1 (sensor 0-4, rightmost to right-inner)
//    bit 9-13  = L1-L5 (sensor 9-13, left-inner to leftmost)
// ─────────────────────────────────────────

// list trigger yang bisa dipilih user (urut)
// index 0-4   = L1-L5 (left side)
// index 5-9   = R1-R5 (right side)
// index 10-14 = LR1-LR5 (both sides)
// index 15    = Blank
// index 16    = Timer
#define TRIGGER_LIST_COUNT 18

static const uint16_t TRIGGER_LIST[TRIGGER_LIST_COUNT] = {
    // L1-L5: bit 9-13 (sensor 9-13, left side)
    (1 << 9), (1 << 10), (1 << 11), (1 << 12), (1 << 13),
    // R1-R5: bit 4-0 (sensor 4-0, right side)
    (1 << 4), (1 << 3), (1 << 2), (1 << 1), (1 << 0),
    // LR1-LR5: bit kiri + kanan bersamaan
    (1 << 9) | (1 << 4),
    (1 << 10) | (1 << 3),
    (1 << 11) | (1 << 2),
    (1 << 12) | (1 << 1),
    (1 << 13) | (1 << 0),
    // Blank & Timer
    TRIGGER_BLANK,
    TRIGGER_TIMER,
    TRIGGER_TICK
};

const char* TRIGGER_NAMES[TRIGGER_LIST_COUNT] = {
    "L1","L2","L3","L4","L5",
    "R1","R2","R3","R4","R5",
    "LR1","LR2","LR3","LR4","LR5",
    "Blank","Timer","Encd"
};

uint8_t trigger_to_idx(uint16_t trigger) {
    for (uint8_t i = 0; i < TRIGGER_LIST_COUNT; i++) {
        if (TRIGGER_LIST[i] == trigger) return i;
    }
    return TRIGGER_LIST_COUNT - 1;
}

// konversi trigger value ke nama string
inline const char* trigger_name(uint16_t trigger) {
    return TRIGGER_NAMES[trigger_to_idx(trigger)];
}

// ─────────────────────────────────────────
//  MENU COUNTER — EDIT SATU COUNTER
// ─────────────────────────────────────────
void menu_counter_edit(uint8_t cidx) {
    uint8_t scroll    = 0;
    uint8_t highlight = 0;
    uint8_t edit_sub  = 0;

    encoderKiriReset();
    encoderKananReset();
    lastEnc_r = encoderKananRead();  // ← sync lastEnc ke nilai sekarang (0)
    lastEnc_l = encoderKiriRead();

    // lookup index item C0: 0→Timer(1), 1→Speed(2), 2→Kp(4)
    const uint8_t c0_map[] = {1, 2, 4, 6};

    while (true) {
        CounterParam& p = g_counter[cidx];
        bool is_c0 = (cidx == 0);

        // C0: hanya 3 item (Timer, Speed, Kp)
        // C1+: normal 6 item
        uint8_t max_item = is_c0 ? 4 : 8;

        if (highlight > max_item) highlight = max_item;

        display_counter(cidx, scroll, highlight, p, true, edit_sub);

        // ── UP ──
        if (btn_up()) {
            if (highlight == 0) {
                if (cidx > 0) {scroll = 0; highlight = 0; edit_sub = 0; }
            } else if (highlight == 1) {
                highlight = 0;
                edit_sub  = 0;
            } else {
                highlight--;
                if (highlight - 1 < scroll) scroll = highlight - 1;
                edit_sub = 0;
            }
        }

        // ── DOWN ──
        if (btn_down()) {
            if (highlight == 0) {
                highlight = 1;
                scroll    = 0;
                edit_sub  = 0;
            } else if (highlight < max_item) {
                highlight++;
                if (highlight - 1 >= scroll + 5) scroll = highlight - 5;
                edit_sub = 0;
            }
        }

        // ── X: toggle edit_sub ──
        if (btn_x()) {
            if (is_c0) {
                // Speed (highlight==2) dan Encoder (highlight==4) punya sub
                if (highlight == 2 || highlight == 4) {
                    edit_sub = !edit_sub;
                } else {
                    edit_sub = 0;
                }
            } else {
                switch (highlight) {
                    case 0: case 2: case 5:   // C, Timer, Kp: tidak ada sub
                        edit_sub = 0;
                        break;
                    default:
                        edit_sub = !edit_sub;
                        break;
                }
            }
        }

        // ── NEXT/BACK: ubah nilai ──
        if (highlight == 0) {
            if (btn_next()) {
                if (cidx < COUNTER_MAX - 1) { cidx++; scroll = 0; highlight = 0; edit_sub = 0; }
            }
            if (btn_back()) {
                if (cidx > 0) { cidx--; scroll = 0; highlight = 0; edit_sub = 0; }
            }
        } else {
            // konversi highlight ke item index
            // C0: pakai c0_map, C1+: langsung highlight-1
            uint8_t item = is_c0 ? c0_map[highlight - 1] : (highlight - 1);

            switch (item) {
                case 0: // Decision & Trigger (C1+ only)
                    if (edit_sub == 0) {
                        uint8_t d = (uint8_t)p.decision;
                        if (btn_next()) { if (d < 4) d++; else d = 0; }    
                        if (btn_back()) { if (d > 0) d--; else d = 4; }
                        p.decision = (Decision)d;
                    } else {
                        uint8_t tidx = trigger_to_idx(p.trigger);
                        if (btn_next()) { if (tidx < TRIGGER_LIST_COUNT - 1) tidx++; else tidx = 0; }
                        if (btn_back()) { if (tidx > 0) tidx--; else tidx = TRIGGER_LIST_COUNT - 1; }
                        p.trigger = TRIGGER_LIST[tidx];
                    }
                    break;

                    case 1: // Timer
                    if (btn_next()) { if (p.timer < 1000) p.timer++; }
                    if (btn_back()) { if (p.timer > 0)    p.timer--; }
                    break;

                case 2: // Speed1 & Speed2
                    if (edit_sub == 0) {
                        if (btn_next()) { if (p.speed1 < 255) p.speed1 += 5; }
                        if (btn_back()) { if (p.speed1 > 0)   p.speed1 -= 5; }
                    } else {
                        if (btn_next()) { if (p.speed2 < 255) p.speed2 += 5; }
                        if (btn_back()) { if (p.speed2 > 0)   p.speed2 -= 5; }
                    }
                    break;

                case 3: {
                    static bool enc_mode  = false;
                    static int8_t init_cidx = -1;
                    if (init_cidx != (int8_t)cidx) {
                        enc_mode  = (p.Encd_b > 0);  // init dari nilai yang ada
                        init_cidx = (int8_t)cidx;
                        lastEnc_l = encoderKiriRead();
                        lastEnc_r = encoderKananRead();
                    }

                    if (edit_sub == 0) {
                        if (btn_next() || btn_back()) {
                            p.delay_type = (p.delay_type == DELAY_A) ? DELAY_B : DELAY_A;
                        }
                    } else {
                        // toggle mode
                        if (btn_next()) {
                            enc_mode = !enc_mode;
                        }

                        // reset nilai aktif
                        if (btn_back()) {
                            if (enc_mode) p.Encd_b  = 0;
                            else          p.delay_ms = 0;
                        }

                        // encoder fisik
                        int32_t cur_l   = encoderKiriRead();
                        int32_t cur_r   = encoderKananRead();
                        int32_t delta_l = cur_l - lastEnc_l;
                        int32_t delta_r = cur_r - lastEnc_r;
                        int32_t delta   = (delta_l != 0) ? delta_l : delta_r;

                        if (delta != 0) {
                            if (enc_mode) {
                                p.Encd_b   = (int16_t)constrain((int32_t)p.Encd_b   + delta,      0, 10000);
                            } else {
                                p.delay_ms = (uint16_t)constrain((int32_t)p.delay_ms + delta, 0, 10000);
                            }
                            lastEnc_l = cur_l;
                            lastEnc_r = cur_r;
                        }
                    }
                    break;
                }

                case 4: // Kp
                    if (btn_next()) { if (p.kp < 255) p.kp++; }
                    if (btn_back()) { if (p.kp > 0)   p.kp--; }
                    break;

                case 5: // Belok PWM L & R
                    if (edit_sub == 0) {
                        if (btn_next()) { if (p.belok_l < 255)  p.belok_l += 5; }
                        if (btn_back()) { if (p.belok_l > -255) p.belok_l -= 5; }
                    } else {
                        if (btn_next()) { if (p.belok_r < 255)  p.belok_r += 5; }
                        if (btn_back()) { if (p.belok_r > -255) p.belok_r -= 5; }
                    }
                    break;
                case 6: {
                    int32_t cur_l = encoderKiriRead();
                    int32_t cur_r = encoderKananRead();

                    // tentukan sisi aktif sekali di awal — jangan berubah kecuali btn_next
                    static bool use_right = false;
                    static int8_t last_cidx = -1;
                    if (last_cidx != (int8_t)cidx) {
                        use_right  = (p.Encd_r > 0 && p.Encd_l == 0);
                        last_cidx  = (int8_t)cidx;
                        lastEnc_l  = cur_l;
                        lastEnc_r  = cur_r;
                    }

                    // btn_next: pindah sisi, bawa nilai
                    if (btn_next()) {
                        if (!use_right) {
                            p.Encd_r  = p.Encd_l;
                            p.Encd_l  = 0;
                            use_right = true;
                        } else {
                            p.Encd_l  = p.Encd_r;
                            p.Encd_r  = 0;
                            use_right = false;
                        }
                        lastEnc_l = cur_l;
                        lastEnc_r = cur_r;
                    }

                    // ubah nilai 1:1 sesuai sisi aktif
                    if (!use_right) {
                        int32_t delta_l = cur_l - lastEnc_l;
                        if (delta_l != 0) {
                            p.Encd_l  = (int16_t)constrain((int32_t)p.Encd_l + delta_l, 0, 10000);
                            lastEnc_l = cur_l;
                        }
                    } else {
                        int32_t delta_r = cur_r - lastEnc_r;
                        if (delta_r != 0) {
                            p.Encd_r  = (int16_t)constrain((int32_t)p.Encd_r + delta_r, 0, 10000);
                            lastEnc_r = cur_r;
                        }
                    }

                    // btn_back: reset sisi aktif
                    if (btn_back()) {
                        if (!use_right) p.Encd_l = 0;
                        else            p.Encd_r = 0;
                    }
                    break;
                }
                case 7: // Line Counter Mode                   
                    uint8_t d = (uint8_t)p.Line_C;
                    if (btn_next()) { if (d < 11) d++; else d = 0; }    
                    if (btn_back()) { if (d > 0) d--; else d = 11; }
                    p.Line_C = (LineCounterMode)d;
                    break;
            }
        }

        // ── SAVE ──
        if (btn_save()) {
            eeprom_save_counter(g_config.mem_slot);
            display_msg("Saved!");
            delay(800);
            return;
        }
    }
}

// ─────────────────────────────────────────
//  MENU COUNTER — PILIH COUNTER
// ─────────────────────────────────────────
void menu_counter() {
    menu_counter_edit(0);
}

// ─────────────────────────────────────────
//  MENU CHECKPOINT
// ─────────────────────────────────────────
void menu_checkpoint() {
    uint8_t scroll    = 0;
    uint8_t highlight = 0;
    uint8_t edit_sub  = 0;

    while (true) {
        display_checkpoint(scroll, highlight, edit_sub);

        CheckpointParam& cp = g_checkpoint[highlight];

        // ── UP ──
        if (btn_up()) {
            nav_up(highlight, scroll, CP_MAX, 5);
            edit_sub = 0;
        }

        // ── DOWN ──
        if (btn_down()) {
            nav_down(highlight, scroll, CP_MAX, 5);
            edit_sub = 0;
        }

        // ── X: toggle sub-field ──
        if (btn_x()) {
            edit_sub = !edit_sub;
        }

        // ── NEXT / BACK: ubah nilai sesuai sub-field ──
        if (edit_sub == 0) {
            if (btn_next()) {
                if (cp.counter_pos == 0xFF) {
                    cp.counter_pos = 0;
                } else if (cp.counter_pos < COUNTER_MAX - 1) {
                    cp.counter_pos++;
                }
            }
            if (btn_back()) {
                if (cp.counter_pos == 0) {
                    cp.counter_pos = 0xFF;
                } else if (cp.counter_pos != 0xFF) {
                    cp.counter_pos--;
                }
            }
        } else {
            if (btn_next()) { if (cp.timer_cp < 1000) cp.timer_cp++; }
            if (btn_back()) { if (cp.timer_cp > 0)    cp.timer_cp--; }
        }

        // ── SAVE ──
        if (btn_save()) {
            eeprom_save_cp(g_config.mem_slot);
            display_msg("Saved!");
            delay(800);
            return;
        }
    }
}

// ─────────────────────────────────────────
//  MENU COPY MEM
// ─────────────────────────────────────────
void menu_copy_mem() {
    uint8_t dst = (g_config.mem_slot + 1) % EEPROM_SLOT_COUNT;

    while (true) {
        u8g2.clearBuffer();
        disp_text(0, 0, "Copy Memory Slot");
        u8g2.setCursor(0, ROW(1));
        u8g2.print("Src:"); u8g2.print(g_config.mem_slot);
        u8g2.print(" -> Dst:"); u8g2.print(dst);
        disp_text(0, 2, "up/dn:ganti tujuan");
        disp_text(0, 3, "SAVE:ok  X:batal");
        u8g2.sendBuffer();

        if (btn_up())   { if (dst < EEPROM_SLOT_COUNT - 1) dst++; }
        if (btn_down()) { if (dst > 0) dst--; }
        if (btn_save()) {
            if (dst != g_config.mem_slot) {
                display_msg("Copying...", "Please wait");
                eeprom_copy_slot(g_config.mem_slot, dst);
                display_msg("Copy Done!");
                delay(1000);
            }
            return;
        }
        if (btn_x()) return;
    }
}

// ─────────────────────────────────────────
//  MENU 1 — KONFIGURASI GLOBAL
// ─────────────────────────────────────────
void menu1() {
    uint8_t scroll    = 0;
    uint8_t highlight = 0;
    uint8_t edit_sub  = 0;  // untuk parameter 2-sub: 0=kiri, 1=kanan
    uint8_t old_mem_slot = g_config.mem_slot;  // ✅ SIMPAN slot lama

    while (true) {
        display_menu1(scroll, highlight, g_config, edit_sub);

        // Navigation: Up/Down only untuk pindah item
        if (btn_up()) {
            nav_up(highlight, scroll, MENU1_COUNT, 6);
            edit_sub = 0;  // reset edit_sub saat navigasi
        }
        if (btn_down()) {
            nav_down(highlight, scroll, MENU1_COUNT, 6);
            edit_sub = 0;
        }

        // X: Toggle sub-item atau action
        if (btn_x()) {
            if (highlight == 9) { run_hcheck(); continue; }
            if (highlight == 0) { run_kalibrasi(); continue; }
            if (highlight == 3) { edit_sub = !edit_sub; }  // PID: toggle Kp/Kd
        }

        // Save config dan exit
        if (btn_save()) {
            pwm_init(g_config.pwm_freq_khz);
            uint8_t new_slot = g_config.mem_slot;

            if (new_slot != old_mem_slot) {
                // Slot berubah:
                // 1. Simpan semua perubahan ke slot LAMA dulu
                g_config.mem_slot = old_mem_slot;
                eeprom_save_all(old_mem_slot);

                // 2. Catat slot aktif baru
                eeprom_save_active_slot(new_slot);

                // 3. Load semua dari slot baru
                display_msg("Loading...", "Mem Slot");
                eeprom_load_all(new_slot);
                g_config.mem_slot = new_slot;  // paksa karena load bisa overwrite
                g_periodo_isr = g_config.periode;  // Sync volatile copy setelah load

                display_msg("Slot changed!");
                delay(800);
            } else {
                // Slot sama — simpan semua ke slot aktif
                eeprom_save_all(g_config.mem_slot);
                display_msg("Saved!", "Config OK");
                delay(800);
            }
            return;
        }

        // Edit nilai dengan Next/Back
        switch (highlight) {
            case 1: 
                if (btn_next() || btn_back()) {
                    g_config.mode = (g_config.mode == MODE_NORMAL) ? MODE_COUNTER : MODE_NORMAL;
                }
                break;
            case 2:
                if (btn_next()) {
                    g_config.speed_mode += 1;
                    if (g_config.speed_mode > 255) g_config.speed_mode = 255;
                }
                if (btn_back()) {
                    if (g_config.speed_mode > 0) g_config.speed_mode -= 1;
                }
                
            case 3: // PID Kp & Kd
                if (edit_sub == 0) {
                    // Edit Kp
                    if (btn_next()) {
                        g_config.kp += 1;
                        if (g_config.kp > 255) g_config.kp = 255;
                    }
                    if (btn_back()) {
                        if (g_config.kp > 0) g_config.kp -= 1;
                    }
                } else {
                    // Edit Kd
                    if (btn_next()) {
                        g_config.kd += 1;
                        if (g_config.kd > 255) g_config.kd = 255;
                    }
                    if (btn_back()) {
                        if (g_config.kd > 0) g_config.kd -= 1;
                    }
                }
                break;

            case 4: // Line
                if (btn_next()) {
                    uint8_t l = (uint8_t)g_config.line;
                    if (l < 2) g_config.line = (LineColor)(l + 1);
                }
                if (btn_back()) {
                    uint8_t l = (uint8_t)g_config.line;
                    if (l > 0) g_config.line = (LineColor)(l - 1);
                }
                break;

            case 5: // Periode
                if (btn_next()) {
                    g_config.periode += 1;
                    if (g_config.periode > 9999) g_config.periode = 9999;
                    g_periodo_isr = g_config.periode;  // Sync volatile copy untuk ISR
                }
                if (btn_back()) {
                    if (g_config.periode > 1) g_config.periode -= 1;
                    g_periodo_isr = g_config.periode;  // Sync volatile copy untuk ISR
                }
                break;

            case 6: // T-Blank
                if (btn_next()) {
                    g_config.t_blank += 1;
                    if (g_config.t_blank > 9999) g_config.t_blank = 9999;
                }
                if (btn_back()) {
                    if (g_config.t_blank > 0) g_config.t_blank -= 1;
                }
                break;

            case 7: // PWM Freq
                if (btn_next()) {
                    g_config.pwm_freq_khz += 0.1f;
                    if (g_config.pwm_freq_khz > PWM_FREQ_MAX) g_config.pwm_freq_khz = PWM_FREQ_MAX;
                }
                if (btn_back()) {
                    g_config.pwm_freq_khz -= 0.1f;
                    if (g_config.pwm_freq_khz < PWM_FREQ_MIN) g_config.pwm_freq_khz = PWM_FREQ_MIN;
                }
                break;
            case 8: // Mirror Line
                if (btn_next() || btn_back()) {
                    g_config.mirrored = !g_config.mirrored;
                }
                break;

            case 10: // Mem Slot
                if (btn_next()) {
                    if (g_config.mem_slot < EEPROM_SLOT_COUNT - 1) g_config.mem_slot++;
                }
                if (btn_back()) {
                    if (g_config.mem_slot > 0) g_config.mem_slot--;
                }
                break;


                break;

        }
    }
}

// ─────────────────────────────────────────
//  MENU 2 — PILIHAN UTAMA
// ─────────────────────────────────────────
void menu2() {
    uint8_t scroll    = 0;
    uint8_t highlight = 0;

    while (true) {
        display_menu2(scroll, highlight);

        if (btn_up()) {
            
            nav_up(highlight, scroll, MENU2_COUNT, 6);
            
        }
        if (btn_down()) nav_down(highlight, scroll, MENU2_COUNT, 6);
        if (btn_x() || btn_next() || btn_back()) {
            switch (highlight) {
                case 0: menu_counter();   break;
                case 1: menu_checkpoint(); break;
                case 2: menu_copy_mem();  break;
                case 3:
                    if (confirm("Delete slot?")) {
                        display_msg("Deleting...", "Please wait");
                        eeprom_delete_slot(g_config.mem_slot);
                        load_defaults();
                        eeprom_save_all(g_config.mem_slot);
                        display_msg("Deleted!", "Done");
                        delay(1000);
                    }
                    break;
                case 4:
                    if (confirm("Reset ALL?")) {
                        display_msg("Resetting...", "Please wait");
                        eeprom_write_defaults_all_slots();
                        eeprom_save_active_slot(0);
                        display_msg("Reset!", "All cleared");
                        delay(1000);
                        system_restart();
                    }
                    break;
            }
        }
        if (btn_save()) return;
    }
}
