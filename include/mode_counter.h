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
extern uint8_t          sensor_inverted;

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
//  ENCODER HELPERS
// ─────────────────────────────────────────

// rata-rata absolut kedua encoder
inline int32_t enc_avg() {
    return (abs(encoderKiriRead()) + abs(encoderKananRead())) / 2;
}

// ambil tick roda luar berdasarkan arah actual motor
inline int32_t enc_outer(int16_t actual_l, int16_t actual_r) {
    if (actual_l > 0) return abs(encoderKiriRead());
    if (actual_r > 0) return abs(encoderKananRead());
    return enc_avg();
}

// resolve Encd dengan mirror swap + pilih outer wheel
// dipakai untuk eksekusi belok/free (arah motor sudah diketahui)
inline int16_t resolve_encd(int16_t actual_l, int16_t actual_r,
                             int16_t encd_l,   int16_t encd_r,
                             bool is_mirrored) {
    int16_t el = is_mirrored ? encd_r : encd_l;
    int16_t er = is_mirrored ? encd_l : encd_r;
    return (actual_l > 0) ? el : er;
}

// resolve Encd untuk counter cur (maju lurus — tidak perlu arah motor)
// ambil nilai yang non-zero, prioritas kiri
inline int16_t resolve_encd_cur(int16_t encd_l, int16_t encd_r) {
    if (encd_l > 0) return encd_l;
    if (encd_r > 0) return encd_r;
    return 0;
}

// ─────────────────────────────────────────
//  FOLLOWING — PID ngikuti garis
// ─────────────────────────────────────────
void following(uint8_t kecepatan, uint8_t kp) {
    scan_sensor();
    if (g_sensor_out != 0) {
        g_pv_out = input_error(g_sensor_out);
    }
    g_error = -g_pv_out;
    calc_pid(kp, g_config.kd);
    g_LOUT = constrain((int)kecepatan + g_out_p + g_out_d, -255, 255);
    g_ROUT = constrain((int)kecepatan - g_out_p - g_out_d, -255, 255);
    set_motors(g_LOUT, g_ROUT, DEFAULT_MAX_PWM);
}

// ─────────────────────────────────────────
//  CEK FLAG COND — adopt dari panzer
//  Dipanggil setiap iterasi loop utama setelah scan sensor
//  Cek apakah robot sudah nemu garis setelah belok
//  Kalau sudah → reset flag_cond, paksa error ke arah koreksi
// ─────────────────────────────────────────
bool cek_flag_cond(bool is_mirrored) {
    if (g_flag_cond == 0) return false;

    // flag_cond=1: belok kanan → tunggu sensor KIRI aktif
    if (g_flag_cond == 1 && (g_sensor_out & 0x0007) != 0) {
        g_pv_out     = is_mirrored ? -24 : 24;
        g_error      = -g_pv_out;
        g_last_error =  g_error;
        g_flag_cond  = 0;
        return true;
    }

    // flag_cond=2: belok kiri → tunggu sensor KANAN aktif
    if (g_flag_cond == 2 && (g_sensor_out & 0x3800) != 0) {
        g_pv_out     = is_mirrored ? 24 : -24;  
        g_error      = -g_pv_out;
        g_last_error =  g_error;
        g_flag_cond  = 0;
        return true;
    }

    return false;
}

// ─────────────────────────────────────────
//  SEEK MOTOR — set motor pivot sesuai flag_cond
// ─────────────────────────────────────────
inline void set_motor_seek(int16_t belok_l, int16_t belok_r) {
    if (g_flag_cond == 1)
        set_motors( (int16_t)belok_l, -(int16_t)belok_r, DEFAULT_MAX_PWM);
    else
        set_motors(-(int16_t)belok_l,  (int16_t)belok_r, DEFAULT_MAX_PWM);
}

// ─────────────────────────────────────────
//  EKSEKUSI BELOK — non-blocking (DELAY_A)
//  1. Set motor belok selama delay_ms
//  2. Set flag_cond → seek dilanjutkan di loop utama
// ─────────────────────────────────────────
void eksekusi_belok(int16_t belok_l, int16_t belok_r,
                    uint16_t delay_ms, bool is_mirrored) {
    if (is_mirrored) {
        set_motors(-belok_l, -belok_r, DEFAULT_MAX_PWM);
    } else {
        set_motors(belok_l, belok_r, DEFAULT_MAX_PWM);
    }
    delay(delay_ms);

    // set flag_cond sesuai arah belok
    if      (belok_l > 0 && belok_r < 0) g_flag_cond = 1;  // seek kiri
    else if (belok_l < 0 && belok_r > 0) g_flag_cond = 2;  // seek kanan
    else                                  g_flag_cond = 0;  // lurus
}

// ─────────────────────────────────────────
//  EKSEKUSI DELAY B — blocking (DELAY_B)
//  Durasi pakai encoder tick kalau Encd ada value,
//  fallback ke delay_ms kalau Encd = 0
//  flag_cond = 0 setelah selesai (tidak ada seek)
// ─────────────────────────────────────────
void eksekusi_delay_b(int16_t belok_l, int16_t belok_r,
                      uint16_t delay_ms, bool is_mirrored,
                      int16_t encd_l, int16_t encd_r) {
    int16_t actual_l = is_mirrored ? -belok_l : belok_l;
    int16_t actual_r = is_mirrored ? -belok_r : belok_r;

    set_motors(actual_l, actual_r, DEFAULT_MAX_PWM);

    // int16_t target = resolve_encd(actual_l, actual_r, encd_l, encd_r, is_mirrored);

    // if (target > 0) {
    //     // tick mode — outer wheel
    //     int32_t start = enc_outer(actual_l, actual_r);
    //     while ((enc_outer(actual_l, actual_r) - start) < target);
    // } else {
    //     // fallback time
    //     delay(delay_ms);
    // }

    delay(delay_ms);
    clear_pid();
    g_flag_cond = 0;
}

// ─────────────────────────────────────────
//  TRIGGER MATCHED
//  sensor_mask    : akumulasi OR sensor sejak timer habis
//  sensor_realtime: g_sensor_out saat ini (untuk TRIGGER_BLANK)
// ─────────────────────────────────────────
inline bool trigger_matched(uint16_t trigger, uint16_t sensor_mask, bool is_mirrored) {
    if (trigger == TRIGGER_TIMER) return true;
    if (trigger == TRIGGER_BLANK) return sensor_mask == 0;

    if (is_mirrored) {
        uint16_t mirrored_trigger = 0;
        for (uint8_t i = 0; i < 14; i++) {
            if (trigger & (1 << i)) mirrored_trigger |= (1 << (13 - i));
        }
        trigger = mirrored_trigger;
    }

    return (sensor_mask & trigger) == trigger;
}

// ─────────────────────────────────────────
//  HITUNG SPEED RAMP
//  Linear interpolasi speed1 → speed2
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
// ─────────────────────────────────────────
bool eksekusi_decision(CounterParam& p, bool is_mirrored, unsigned long elapsed_start) {
    switch (p.decision) {

        case DEC_LOST:
            // Do Noting
            break;

        case DEC_BELOK_KIRI:
            if (p.delay_type == DELAY_A) {
                eksekusi_belok(-(int16_t)p.belok_l, (int16_t)p.belok_r,
                               p.delay_ms, is_mirrored);
            } else {
                eksekusi_delay_b(-(int16_t)p.belok_l, (int16_t)p.belok_r,
                                 p.delay_ms, is_mirrored, p.Encd_l, p.Encd_r);
            }
            break;

        case DEC_BELOK_KANAN:
            if (p.delay_type == DELAY_A) {
                eksekusi_belok((int16_t)p.belok_l, -(int16_t)p.belok_r,
                               p.delay_ms, is_mirrored);
            } else {
                eksekusi_delay_b((int16_t)p.belok_l, -(int16_t)p.belok_r,
                                 p.delay_ms, is_mirrored, p.Encd_l, p.Encd_r);
            }
            break;

        case DEC_FREE: {
            int16_t fl = is_mirrored ? (int16_t)p.belok_r : (int16_t)p.belok_l;
            int16_t fr = is_mirrored ? (int16_t)p.belok_l : (int16_t)p.belok_r;

            if (p.trigger == TRIGGER_TICK) {
                // durasi pakai encoder — outer wheel sesuai arah fl/fr
                int16_t target = resolve_encd(fl, fr, p.Encd_l, p.Encd_r, is_mirrored);
                if (target > 0) {
                    int32_t start = enc_outer(fl, fr);
                    while ((enc_outer(fl, fr) - start) < target) {
                        led_lcd(false);
                        led_timer((read_timer() / 5) % 2 == 1);
                        set_motors(fl, fr, DEFAULT_MAX_PWM);
                    }
                }
            } else {
                // durasi pakai timer (TRIGGER_TIMER atau TRIGGER_SENSOR)
                int free_start = read_timer();
                while ((read_timer() - free_start) < (int)p.timer) {
                    led_lcd(false);
                    led_timer((read_timer() / 5) % 2 == 1);
                    set_motors(fl, fr, DEFAULT_MAX_PWM);
                }
            }

            // clear di luar kedua branch — selalu dieksekusi
            clear_pid();
            g_flag_cond = 0;
            break;
        }
        case DEC_STOP:
            // --- TRIGGER_TIMER: following selama p.timer ---
            if (p.trigger == TRIGGER_TIMER) {
                reset_timer();
                while (read_timer() < p.timer) {
                    led_lcd(true);
                    led_timer((read_timer() / 5) % 2 == 1);
                    uint8_t spd = ramp_speed(p.speed1, p.speed2,
                                            read_timer(), p.timer);
                    scan_sensor();
                    if (g_sensor_out != 0) g_pv_out = input_error(g_sensor_out);
                    g_error = -g_pv_out;
                    calc_pid(p.kp, g_config.kd);
                    g_LOUT = constrain((int)spd + g_out_p + g_out_d, -255, 255);
                    g_ROUT = constrain((int)spd - g_out_p - g_out_d, -255, 255);
                    set_motors(g_LOUT, g_ROUT, DEFAULT_MAX_PWM);
                }                          // ← tutup while
            }                              // ← tutup if TRIGGER_TIMER

            // --- TRIGGER_SENSOR: following sampai sensor match ---
            else if (p.trigger != TRIGGER_TICK) {  // TRIGGER_SENSOR / TRIGGER_BLANK
                uint16_t akum = 0;
                while (true) {
                    led_lcd(true);
                    led_timer((read_timer() / 5) % 2 == 1);
                    scan_sensor();
                    if (g_sensor_out != 0) g_pv_out = input_error(g_sensor_out);
                    g_error = -g_pv_out;
                    calc_pid(p.kp, g_config.kd);
                    g_LOUT = constrain((int)p.speed2 + g_out_p + g_out_d, -255, 255);
                    g_ROUT = constrain((int)p.speed2 - g_out_p - g_out_d, -255, 255);
                    set_motors(g_LOUT, g_ROUT, DEFAULT_MAX_PWM);
                    akum |= g_sensor_out;
                    if (trigger_matched(p.trigger, akum, is_mirrored)) break;
                }
            }

            // --- STOP & finish ---
            motor_brake();
            delay(20);
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
//  MODE COUNTER — MAIN LOOP
// ─────────────────────────────────────────
bool mode_counter(uint8_t cp_start) {
    u8g2.clearBuffer();
    u8g2.sendBuffer();

    unsigned long elapsed_start = millis();
    uint8_t  start_counter = 0;
    uint16_t start_timer   = g_config.t_blank;

    // resume dari CP terpilih
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
        decode_zone(0);
    }

    // ── LOOP COUNTER SEKUENSIAL ──
    for (uint8_t cidx = start_counter; cidx < COUNTER_MAX - 1; cidx++) {
        g_counter_idx = cidx;
        CounterParam& cur = g_counter[cidx];
        CounterParam& nxt = g_counter[cidx + 1];

        bool is_mirrored = g_config.mirrored;

        // ── Mode timing cur: encoder atau timer ──
        // resolve_encd_cur: ambil Encd yang non-zero (maju lurus, tidak perlu arah)
        // int16_t cur_tick_target  = resolve_encd_cur(cur.Encd_l, cur.Encd_r);
        // int32_t enc_counter_start = enc_avg();  // referensi encoder awal counter ini

        uint16_t sensor_akumulasi  = 0;
        bool beraksi           = false;
        bool timer_sudah_habis = false;
        // bool tick_init         = false;
        // bool sensor_cleared    = false;
        // int32_t tick_start     = 0;

        // ── LOOP UTAMA ──
        while (!beraksi) {
            int t         = read_timer();
            // int32_t enc_now = enc_avg() - enc_counter_start;

            // // ── LED ──
            // bool waktu_habis_led = (cur_tick_target > 0)
            //     ? (enc_now >= cur_tick_target)
            //     : (t >= cur.timer);
            // led_lcd(waktu_habis_led);
            // led_timer((t / 5) % 2 == 1);

            led_lcd(t >= cur.timer);
            led_timer((t / 5) % 2 == 1);

            uint8_t spd = (t < cur.timer)
                ? ramp_speed(cur.speed1, cur.speed2, t, cur.timer)
                : cur.speed2;
            // ── RAMP SPEED ──
            // uint8_t spd;
            // if (cur_tick_target > 0) {
            //     spd = ramp_speed(cur.speed1, cur.speed2,
            //                      (int)enc_now, (int)cur_tick_target);
            // } else {
            //     spd = (t < cur.timer)
            //         ? ramp_speed(cur.speed1, cur.speed2, t, cur.timer)
            //         : cur.speed2;
            // }

            // ── SENSOR & PID ──
            scan_sensor();
            if (g_sensor_out != 0) g_pv_out = input_error(g_sensor_out);

            bool seeking = (g_flag_cond != 0);
            bool found   = cek_flag_cond(is_mirrored);

            g_error = -g_pv_out;
            calc_pid(cur.kp, g_config.kd);

            if (seeking && !found) {
                set_motor_seek(cur.belok_l, cur.belok_r);
            } else {
                g_LOUT = constrain((int)spd + g_out_p + g_out_d, -255, 255);
                g_ROUT = constrain((int)spd - g_out_p - g_out_d, -255, 255);
                set_motors(g_LOUT, g_ROUT, DEFAULT_MAX_PWM);
            }

            // ── CEK TRIGGER ──

            // if (nxt.trigger == TRIGGER_TICK) {
            //     // TRIGGER_TICK: bypass timer, cek encoder dari awal counter
            //     if (!tick_init) {
            //         tick_start = enc_avg();
            //         tick_init  = true;
            //     }
            //     int16_t target = resolve_encd(
            //         (int16_t)nxt.belok_l, (int16_t)nxt.belok_r,
            //         nxt.Encd_l, nxt.Encd_r, is_mirrored
            //     );
            //     int32_t tick_delta = enc_avg() - tick_start;
            //     if (target == 0 || tick_delta >= target) {
            //         beraksi = true;
            //     }

            // } else if (!seeking) {
            //     // TRIGGER_TIMER / TRIGGER_SENSOR: tunggu waktu habis dulu
            //     bool waktu_habis = (cur_tick_target > 0)
            //         ? (enc_now >= cur_tick_target)
            //         : (t >= cur.timer);

                // if (waktu_habis) {
                //     if (!timer_sudah_habis) {
                //         sensor_akumulasi  = 0;
                //         timer_sudah_habis = true;
                //     }

                //     if (!sensor_cleared) {
                //         if (g_sensor_out == 0) sensor_cleared = true;
                //     }

                    // if (sensor_cleared) {                              // ← guard ini
                    //     sensor_akumulasi |= g_sensor_out; }
            if (t >= cur.timer && !seeking) {
                if (!timer_sudah_habis) {
                    sensor_akumulasi  = 0;
                    timer_sudah_habis = true;
                }
                sensor_akumulasi |= g_sensor_out;

                if (nxt.decision == DEC_FREE ||
                    trigger_matched(nxt.trigger, sensor_akumulasi, is_mirrored)) {
                    beraksi = true;
                }
            }
        }
        

        // ── EKSEKUSI DECISION ──
        if (eksekusi_decision(nxt, is_mirrored, elapsed_start)) return true;

        // ── TUNGGU SEEKING SELESAI ──
        while (g_flag_cond != 0) {
            scan_sensor();
            if (g_sensor_out != 0) g_pv_out = input_error(g_sensor_out);

            bool found = cek_flag_cond(is_mirrored);

            g_error = -g_pv_out;
            calc_pid(nxt.kp, g_config.kd);

            if (g_flag_cond != 0) {
                set_motor_seek(nxt.belok_l, nxt.belok_r);
            } else {
                // sudah nemu garis → PID normal
                g_LOUT = constrain((int)nxt.speed2 + g_out_p + g_out_d, -255, 255);
                g_ROUT = constrain((int)nxt.speed2 - g_out_p - g_out_d, -255, 255);
                set_motors(g_LOUT, g_ROUT, DEFAULT_MAX_PWM);
            }
        }

        // ── RESET UNTUK COUNTER BERIKUTNYA ──
        clear_pid();
        reset_timer();
        decode_zone(g_counter[cidx].Line_C);
        led_lcd(false);
        led_timer(false);
    }

    // semua counter habis tanpa STOP
    // motor_brake();
    // delay(20);
    motor_stop();
    g_flag_cond = 0;
    led_lcd(false);
    led_timer(false);
    display_finish(millis() - elapsed_start);
    while (!sw_save() && !sw_x());
    return true;
}