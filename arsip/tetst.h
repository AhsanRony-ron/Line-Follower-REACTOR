#pragma once
#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "sensor.h"
#include "motor.h"
#include "pid.h"
#include "display.h"

// ─────────────────────────────────────────
//  EXTERN
// ─────────────────────────────────────────
extern GlobalConfig     g_config;
extern CounterParam     g_counter[COUNTER_MAX];
extern CheckpointParam  g_checkpoint[CP_MAX];
extern volatile int     g_timer;
extern volatile int     g_cacah;
extern uint8_t          g_counter_idx;
extern int              g_LOUT;
extern int              g_ROUT;
extern int              g_error;
extern int              g_last_error;

// ─────────────────────────────────────────
//  FLAG COND — adopt dari panzer
//  0 = idle
//  1 = seek kiri  (setelah belok kanan, tunggu sensor kiri aktif)
//  2 = seek kanan (setelah belok kiri,  tunggu sensor kanan aktif)
// ─────────────────────────────────────────
static uint8_t g_flag_cond = 0;

// ─────────────────────────────────────────
//  RESET TIMER
// ─────────────────────────────────────────
inline void reset_timer() {
    noInterrupts();
    g_timer = 0;
    g_cacah = 0;
    interrupts();
}

inline int read_timer() {
    noInterrupts();
    int t = g_timer;
    interrupts();
    return t;
}

// ─────────────────────────────────────────
//  FOLLOWING — PID ngikuti garis
// ─────────────────────────────────────────
void following(uint8_t kecepatan, uint8_t kp) {
    scan_sensor(g_config.line);

    if (g_sensor_out != 0) {
        g_pv_out = input_error(g_sensor_out);
    }
    g_error = -g_pv_out;

    calc_pid(kp, g_config.kd);

    g_LOUT = constrain((int)kecepatan - g_out_p - g_out_d, -255, 255);
    g_ROUT = constrain((int)kecepatan + g_out_p + g_out_d, -255, 255);

    set_motors(g_LOUT, g_ROUT, DEFAULT_MAX_PWM);
}

// ─────────────────────────────────────────
//  CEK FLAG COND — adopt dari panzer
//  is_mirrored: kalau true, sensor kiri/kanan dibalik
// ─────────────────────────────────────────
bool cek_flag_cond(bool is_mirrored) {
    if (g_flag_cond == 0) return false;

    // Normal:
    //   flag_cond=1 → pivot kanan → cari sensor KIRI  (bit 0-2 = 0x0007)
    //   flag_cond=2 → pivot kiri  → cari sensor KANAN (bit 11-13 = 0x3800)
    // Mirrored: semua dibalik
    uint16_t mask_fc1 = is_mirrored ? 0x3800 : 0x0007;
    uint16_t mask_fc2 = is_mirrored ? 0x0007 : 0x3800;
    int      pv_fc1   = is_mirrored ?    -24 :    +24;
    int      pv_fc2   = is_mirrored ?    +24 :    -24;

    if (g_flag_cond == 1 && (g_sensor_out & mask_fc1) != 0) {
        g_pv_out     = pv_fc1;
        g_error      = -g_pv_out;
        g_last_error =  g_error;
        g_flag_cond  = 0;
        return true;
    }

    if (g_flag_cond == 2 && (g_sensor_out & mask_fc2) != 0) {
        g_pv_out     = pv_fc2;
        g_error      = -g_pv_out;
        g_last_error =  g_error;
        g_flag_cond  = 0;
        return true;
    }

    return false;
}

// ─────────────────────────────────────────
//  EKSEKUSI BELOK — non-blocking
//  1. Set motor belok selama delay_ms
//  2. Set flag_cond → seek dilanjutkan di loop utama
// ─────────────────────────────────────────
void eksekusi_belok(int16_t belok_l, int16_t belok_r, uint16_t delay_ms) {
    set_motors(belok_l, belok_r, DEFAULT_MAX_PWM);
    delay(delay_ms);

    // flag_cond ditentukan dari tanda belok_l/belok_r
    // yang sudah di-mirror sebelum dipassing ke sini
    if      (belok_l > 0 && belok_r < 0) g_flag_cond = 1;  // pivot kanan → seek kiri
    else if (belok_l < 0 && belok_r > 0) g_flag_cond = 2;  // pivot kiri  → seek kanan
    else                                  g_flag_cond = 0;
}

// ─────────────────────────────────────────
//  DELAY B — Free tanpa memory
// ─────────────────────────────────────────
void eksekusi_delay_b(int16_t belok_l, int16_t belok_r, uint16_t delay_ms) {
    set_motors(belok_l, belok_r, DEFAULT_MAX_PWM);
    delay(delay_ms);
    clear_pid();
    g_flag_cond = 0;
}

// ─────────────────────────────────────────
//  TRIGGER MATCHED
//  is_mirrored: mirror bit trigger kiri↔kanan
// ─────────────────────────────────────────
inline bool trigger_matched(uint16_t trigger, uint16_t sensor_mask, bool is_mirrored) {
    if (trigger == TRIGGER_TIMER) return true;
    if (trigger == TRIGGER_BLANK) return (sensor_mask == 0);

    if (is_mirrored) {
        uint16_t mirrored_trigger = 0;
        for (uint8_t i = 0; i < 14; i++) {
            if (trigger & (1 << i)) mirrored_trigger |= (1 << (13 - i));
        }
        trigger = mirrored_trigger;
    }

    return (sensor_mask & trigger) != 0;  // ANY bit match
}

// ─────────────────────────────────────────
//  HITUNG SPEED RAMP
// ─────────────────────────────────────────
inline uint8_t ramp_speed(uint8_t speed1, uint8_t speed2, int t, int timer) {
    if (timer <= 0) return speed2;
    if (t <= 0)     return speed1;
    if (t >= timer) return speed2;
    int spd = (int)speed1 + ((int)speed2 - (int)speed1) * t / timer;
    return (uint8_t)constrain(spd, 0, 255);
}

// ─────────────────────────────────────────
//  EKSEKUSI DECISION
//  is_mirrored: belok kiri↔kanan dibalik
//  return true jika DEC_STOP (mode_counter harus return)
// ─────────────────────────────────────────
bool eksekusi_decision(CounterParam& p, bool is_mirrored, unsigned long elapsed_start) {
    switch (p.decision) {

        case DEC_LOST:
            if (p.delay_type == DELAY_A) {
                set_motors(p.belok_l, p.belok_r, DEFAULT_MAX_PWM);
                delay(p.delay_ms);
                g_flag_cond = 0;
                clear_pid();
            } else {
                eksekusi_delay_b(p.belok_l, p.belok_r, p.delay_ms);
            }
            break;

        case DEC_BELOK_KIRI:
            // normal : L mundur (neg), R maju (pos)
            // mirrored: L maju (pos), R mundur (neg) → jadi belok kanan
            if (p.delay_type == DELAY_A) {
                if (is_mirrored) eksekusi_belok( (int16_t)p.belok_l, -(int16_t)p.belok_r, p.delay_ms);
                else             eksekusi_belok(-(int16_t)p.belok_l,  (int16_t)p.belok_r, p.delay_ms);
            } else {
                if (is_mirrored) eksekusi_delay_b( (int16_t)p.belok_l, -(int16_t)p.belok_r, p.delay_ms);
                else             eksekusi_delay_b(-(int16_t)p.belok_l,  (int16_t)p.belok_r, p.delay_ms);
            }
            break;

        case DEC_BELOK_KANAN:
            // normal : L maju (pos), R mundur (neg)
            // mirrored: L mundur (neg), R maju (pos) → jadi belok kiri
            if (p.delay_type == DELAY_A) {
                if (is_mirrored) eksekusi_belok(-(int16_t)p.belok_l,  (int16_t)p.belok_r, p.delay_ms);
                else             eksekusi_belok( (int16_t)p.belok_l, -(int16_t)p.belok_r, p.delay_ms);
            } else {
                if (is_mirrored) eksekusi_delay_b(-(int16_t)p.belok_l,  (int16_t)p.belok_r, p.delay_ms);
                else             eksekusi_delay_b( (int16_t)p.belok_l, -(int16_t)p.belok_r, p.delay_ms);
            }
            break;

        case DEC_FREE: {
            // mirrored: tukar L dan R (bukan negasikan)
            int16_t fl = is_mirrored ? (int16_t)p.free_r : (int16_t)p.free_l;
            int16_t fr = is_mirrored ? (int16_t)p.free_l : (int16_t)p.free_r;

            if (p.trigger == TRIGGER_TIMER) {
                int free_start = read_timer();
                while ((read_timer() - free_start) < (int)p.timer) {
                    led_lcd(false);
                    led_timer((read_timer() / 5) % 2 == 1);
                    set_motors(fl, fr, DEFAULT_MAX_PWM);
                }
            } else {
                // gerak sampai trigger terpenuhi (realtime, bukan akumulasi)
                uint16_t free_akumulasi = 0;
                while (true) {
                    set_motors(fl, fr, DEFAULT_MAX_PWM);
                    scan_sensor(g_config.line);
                    free_akumulasi |= g_sensor_out;
                    if (trigger_matched(p.trigger, free_akumulasi, is_mirrored)) break;
                }
            }
            clear_pid();
            g_flag_cond = 0;
            break;
        }

        case DEC_STOP:
            motor_stop();
            clear_pid();
            g_flag_cond = 0;
            led_lcd(false);
            led_timer(false);
            display_finish(millis() - elapsed_start);
            while (!sw_save() && !sw_x() && !sw_next() && !sw_back());
            return true;

        default:
            break;
    }

    return false;
}

// ─────────────────────────────────────────
//  SEEK MOTOR — set motor pivot sesuai flag_cond
//  Tidak perlu bedakan mirrored karena belok_l/belok_r
//  yang dipassing ke eksekusi_belok sudah di-mirror
// ─────────────────────────────────────────
inline void set_motor_seek(int16_t belok_l, int16_t belok_r) {
    if (g_flag_cond == 1) set_motors( (int16_t)belok_l, -(int16_t)belok_r, DEFAULT_MAX_PWM);
    else                  set_motors(-(int16_t)belok_l,  (int16_t)belok_r, DEFAULT_MAX_PWM);
}

// ─────────────────────────────────────────
//  MODE COUNTER — MAIN LOOP
// ─────────────────────────────────────────
bool mode_counter(uint8_t cp_start) {
    lcd.clearDisplay();
    lcd.display();

    unsigned long elapsed_start = millis();
    uint8_t  start_counter = 0;
    uint16_t start_timer   = g_config.t_blank;

    // resume dari CP
    if (cp_start < CP_MAX) {
        CheckpointParam& cp = g_checkpoint[cp_start];
        if (cp.counter_pos != 0xFF) {
            start_counter = cp.counter_pos + 1;
            start_timer   = cp.timer_cp;
        }
    }

    // ── T-BLANK / CP TIMER ──
    g_flag_cond = 0;
    reset_timer();
    while (read_timer() < start_timer) {
        led_lcd(false);
        led_timer((read_timer() / 5) % 2 == 1);
        following(g_config.speed_mode, g_config.kp);
    }

    // ── LOOP COUNTER SEKUENSIAL ──
    for (uint8_t cidx = start_counter; cidx < COUNTER_MAX - 1; cidx++) {
        CounterParam& cur = g_counter[cidx];
        CounterParam& nxt = g_counter[cidx + 1];

        // mirrored diambil dari config global
        bool is_mirrored = g_config.mirrored;

        uint16_t sensor_akumulasi  = 0;
        bool beraksi           = false;
        bool timer_sudah_habis = false;

        // ── LOOP UTAMA ──
        while (!beraksi) {
            int t = read_timer();

            led_lcd(t >= cur.timer);
            led_timer((t / 5) % 2 == 1);

            uint8_t spd = (t < cur.timer)
                ? ramp_speed(cur.speed1, cur.speed2, t, cur.timer)
                : cur.speed2;

            // scan sensor
            scan_sensor(g_config.line);
            if (g_sensor_out != 0) g_pv_out = input_error(g_sensor_out);

            // cek seeking
            bool seeking = (g_flag_cond != 0);
            bool found   = cek_flag_cond(is_mirrored);

            // PID
            g_error = -g_pv_out;
            calc_pid(cur.kp, g_config.kd);

            if (seeking && !found) {
                // pertahankan pivot — arah dari flag_cond
                // belok_l/belok_r sudah di-mirror saat eksekusi_belok dipanggil
                set_motor_seek(cur.belok_l, cur.belok_r);
            } else {
                g_LOUT = constrain((int)spd - g_out_p - g_out_d, -255, 255);
                g_ROUT = constrain((int)spd + g_out_p + g_out_d, -255, 255);
                set_motors(g_LOUT, g_ROUT, DEFAULT_MAX_PWM);
            }

            // cek trigger — hanya setelah timer habis dan tidak seeking
            if (t >= cur.timer && !seeking) {
                if (!timer_sudah_habis) {
                    sensor_akumulasi  = 0;
                    timer_sudah_habis = true;
                }
                sensor_akumulasi |= g_sensor_out;

                if (trigger_matched(nxt.trigger, sensor_akumulasi, is_mirrored)) {
                    beraksi = true;
                }
            }
        }

        // ── EKSEKUSI DECISION ──
        if (eksekusi_decision(nxt, is_mirrored, elapsed_start)) return true;

        // ── TUNGGU SEEKING SELESAI — baru reset timer ──
        while (g_flag_cond != 0) {
            scan_sensor(g_config.line);
            if (g_sensor_out != 0) g_pv_out = input_error(g_sensor_out);

            cek_flag_cond(is_mirrored);

            g_error = -g_pv_out;
            calc_pid(nxt.kp, g_config.kd);

            if (g_flag_cond != 0) {
                set_motor_seek(nxt.belok_l, nxt.belok_r);
            } else {
                g_LOUT = constrain((int)nxt.speed2 - g_out_p - g_out_d, -255, 255);
                g_ROUT = constrain((int)nxt.speed2 + g_out_p + g_out_d, -255, 255);
                set_motors(g_LOUT, g_ROUT, DEFAULT_MAX_PWM);
            }
        }

        // seeking selesai → reset timer untuk counter berikutnya
        clear_pid();
        reset_timer();
        led_lcd(false);
        led_timer(false);
    }

    // semua counter habis tanpa STOP
    motor_stop();
    g_flag_cond = 0;
    led_lcd(false);
    led_timer(false);
    display_finish(millis() - elapsed_start);
    while (!sw_save() && !sw_x());
    return true;
}