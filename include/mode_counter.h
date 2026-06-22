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
extern uint8_t g_resume_cp;  // deklarasi extern, definisi di main.cpp

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

// // ambil tick roda luar berdasarkan arah actual motor
// inline int32_t enc_outer(int16_t actual_l, int16_t actual_r) {
//     if (actual_l > 0) return abs(encoderKiriRead());
//     if (actual_r > 0) return abs(encoderKananRead());
//     return enc_avg();
// }

// resolve Encd dengan mirror swap + pilih outer wheel
// dipakai untuk eksekusi belok/free (arah motor sudah diketahui)
inline int16_t resolve_encd(int16_t actual_l, int16_t actual_r,
                             int16_t encd_l,   int16_t encd_r,
                             bool is_mirrored) {
    int16_t el = is_mirrored ? encd_r : encd_l;
    int16_t er = is_mirrored ? encd_l : encd_r;

    if (actual_l > 0 && actual_r > 0) {
        return (el > 0) ? el : er;  // ambil yang ada
    }

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
        g_pv_out     = -24;
        g_error      = -g_pv_out;
        g_last_error =  g_error;
        g_flag_cond  = 0;
        return true;
    }

    // flag_cond=2: belok kiri → tunggu sensor KANAN aktif
    if (g_flag_cond == 2 && (g_sensor_out & 0x3800) != 0) {
        g_pv_out     = 24;  
        g_error      = -g_pv_out;
        g_last_error =  g_error;
        g_flag_cond  = 0;
        return true;
    }

    return 0;
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
//  Durasi pakai Encd_b kalau ada value,
//  fallback ke delay_ms kalau Encd_b = 0
//  Set flag_cond → seek dilanjutkan di loop utama
// ─────────────────────────────────────────
void eksekusi_belok(int16_t actual_l, int16_t actual_r, uint16_t delay_ms, int16_t encd_b) {
    if (encd_b > 0 && (actual_l != 0 || actual_r != 0)) {
        bool use_left = (abs(actual_l) >= abs(actual_r));

        // reset encoder sebelum mulai — hindari nilai lama
        int32_t start;
        if (use_left) { encoderKiriReset();  start = encoderKiriRead();  }
        else          { encoderKananReset(); start = encoderKananRead(); }
        while (true) {
            int32_t now = use_left ? abs(encoderKiriRead()) : abs(encoderKananRead());
            if ((now - start) >= encd_b) break;
            set_motors(actual_l, actual_r, DEFAULT_MAX_PWM);
        }
    } else {
        set_motors(actual_l, actual_r, DEFAULT_MAX_PWM);
        delay(delay_ms);
    }

    if      (actual_l > 0 && actual_r < 0) g_flag_cond = 1;
    else if (actual_l < 0 && actual_r > 0) g_flag_cond = 2;
    else                                    g_flag_cond = 0;
}

// ─────────────────────────────────────────
//  EKSEKUSI DELAY B — blocking (DELAY_B)
//  Durasi pakai Encd_b kalau ada value,
//  fallback ke delay_ms kalau Encd_b = 0
//  flag_cond = 0 setelah selesai (tidak ada seek)
// ─────────────────────────────────────────
void eksekusi_delay_b(int16_t belok_l, int16_t belok_r,
                      uint16_t delay_ms, bool is_mirrored,
                      int16_t encd_b) {
    int16_t actual_l = belok_l;
    int16_t actual_r = belok_r;
    if (is_mirrored) { int16_t tmp = actual_l; actual_l = actual_r; actual_r = tmp; }

    set_motors(actual_l, actual_r, DEFAULT_MAX_PWM);

    if (encd_b > 0) {
        // encoder mode — outer wheel (abs)
        bool use_left = (abs(actual_l) >= abs(actual_r));
        int32_t start;
        if (use_left) { encoderKiriReset();  start = encoderKiriRead();  }
        else          { encoderKananReset(); start = encoderKananRead(); }
        while (true) {
            int32_t now = use_left ? abs(encoderKiriRead()) : abs(encoderKananRead());
            if ((now - start) >= encd_b) break;
            set_motors(actual_l, actual_r, DEFAULT_MAX_PWM);
        }
    } else {
        delay(delay_ms);
    }

    clear_pid();
    g_flag_cond = 0;
}

// ─────────────────────────────────────────
//  TRIGGER MATCHED
//  sensor_mask    : akumulasi OR sensor sejak timer habis
//  sensor_realtime: g_sensor_out saat ini (untuk TRIGGER_BLANK)
// ─────────────────────────────────────────
inline bool trigger_matched(uint16_t trigger, uint16_t sensor_mask, uint16_t sensor_realtime, bool is_mirrored) {
    if (trigger == TRIGGER_TIMER) return true;
    if (trigger == TRIGGER_BLANK) return sensor_realtime == 0;

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
bool eksekusi_decision(CounterParam& p, bool is_mirrored, unsigned long elapsed_start,  uint16_t sensor_akum_in = 0) {
    switch (p.decision) {

        case DEC_LOST:
            // Do Noting
            break;

        case DEC_BELOK_KANAN: {
            int16_t l =  (int16_t)p.motor_l;
            int16_t r = -(int16_t)p.motor_r;
            if (is_mirrored) { int16_t tmp = l; l = r; r = tmp; }
            if (p.delay_type == DELAY_A) {
                eksekusi_belok(l, r, p.delay_ms, p.Encd_b);
            } else {
                eksekusi_delay_b(l, r, p.delay_ms, false, p.Encd_b);  // mirror sudah dihandle
            }
            break;
        }
        case DEC_BELOK_KIRI: {
            int16_t l = -(int16_t)p.motor_l;
            int16_t r =  (int16_t)p.motor_r;
            if (is_mirrored) { int16_t tmp = l; l = r; r = tmp; }
            if (p.delay_type == DELAY_A) {
                eksekusi_belok(l, r, p.delay_ms, p.Encd_b);
            } else {
                eksekusi_delay_b(l, r, p.delay_ms, false, p.Encd_b);
            }
            break;
        }

        case DEC_FREE: {
            int16_t fl = is_mirrored ? (int16_t)p.motor_r : (int16_t)p.motor_l;
            int16_t fr = is_mirrored ? (int16_t)p.motor_l : (int16_t)p.motor_r;
            int16_t target = resolve_encd(fl, fr, p.Encd_l, p.Encd_r, is_mirrored);

            if (p.trigger == TRIGGER_TICK || target > 0) {
                bool use_left = is_mirrored ? (p.Encd_r > 0 ? false : true)
                            : (p.Encd_l > 0);

                int32_t start = use_left ? encoderKiriRead() : encoderKananRead();

                while (true) {
                    int32_t now = use_left ? encoderKiriRead() : encoderKananRead();
                    if (abs(now - start) >= target) break;

                    led_lcd(false);
                    led_timer((read_timer() / 5) % 2 == 1);
                    set_motors(fl, fr, DEFAULT_MAX_PWM);
                }
            } else {
                int free_start = read_timer();
                while ((read_timer() - free_start) < (int)p.timer) {
                    led_lcd(false);
                    led_timer((read_timer() / 5) % 2 == 1);
                    set_motors(fl, fr, DEFAULT_MAX_PWM);
                }
            }
            clear_pid();
            g_flag_cond = 0;
            break;
        }
        case DEC_STOP:
            reset_timer();
            // --- TRIGGER_TICK: following selama encoder tick ---
            if (p.trigger == TRIGGER_TICK) {
                int16_t target = resolve_encd_cur(p.Encd_l, p.Encd_r);
                if (target > 0) {
                    // tentukan sisi encoder yang dipakai
                    bool use_left = (is_mirrored ? p.Encd_r : p.Encd_l) > 0 ||
                                    (p.Encd_l == 0 && p.Encd_r == 0);
                    int32_t start = use_left ? encoderKiriRead() : encoderKananRead();

                    while (true) {
                        int32_t now = use_left ? encoderKiriRead() : encoderKananRead();
                        if (abs(now - start) >= target) break;
                        led_lcd(true);
                        led_timer((read_timer() / 5) % 2 == 1);
                        scan_sensor();
                        if (g_sensor_out != 0) g_pv_out = input_error(g_sensor_out, is_mirrored);
                        g_error = -g_pv_out;
                        calc_pid(p.kp, g_config.kd);
                        g_LOUT = constrain((int)p.speed2 + g_out_p + g_out_d, -255, 255);
                        g_ROUT = constrain((int)p.speed2 - g_out_p - g_out_d, -255, 255);
                        set_motors(g_LOUT, g_ROUT, DEFAULT_MAX_PWM, true);
                    }
                }
            }
            // --- TRIGGER_TIMER: following selama p.timer ---
            else if (p.trigger == TRIGGER_TIMER) {
                reset_timer();
                while (read_timer() < p.timer) {
                    led_lcd(true);
                    led_timer((read_timer() / 5) % 2 == 1);
                    uint8_t spd = ramp_speed(p.speed1, p.speed2, read_timer(), p.timer);
                    scan_sensor();
                    if (g_sensor_out != 0) g_pv_out = input_error(g_sensor_out, is_mirrored);
                    g_error = -g_pv_out;
                    calc_pid(p.kp, g_config.kd);
                    g_LOUT = constrain((int)spd + g_out_p + g_out_d, -255, 255);
                    g_ROUT = constrain((int)spd - g_out_p - g_out_d, -255, 255);
                    set_motors(g_LOUT, g_ROUT, DEFAULT_MAX_PWM);
                }
            }

            // --- TRIGGER_SENSOR/BLANK: following sampai sensor match ---
            else {
                uint16_t akum = 0;
                while (true) {
                    led_lcd(true);
                    led_timer((read_timer() / 5) % 2 == 1);
                    scan_sensor();
                    if (g_sensor_out != 0) g_pv_out = input_error(g_sensor_out, is_mirrored);
                    g_error = -g_pv_out;
                    calc_pid(p.kp, g_config.kd);
                    g_LOUT = constrain((int)p.speed2 + g_out_p + g_out_d, -255, 255);
                    g_ROUT = constrain((int)p.speed2 - g_out_p - g_out_d, -255, 255);
                    set_motors(g_LOUT, g_ROUT, DEFAULT_MAX_PWM);
                    akum |= g_sensor_out;
                    if (trigger_matched(p.trigger, akum, g_sensor_out, is_mirrored)) break;
                }
            }

            // --- STOP & finish ---
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

    return 0;
}

// ─────────────────────────────────────────
//  MODE COUNTER — MAIN LOOP
// ─────────────────────────────────────────
uint8_t mode_counter(uint8_t cp_start) {
    u8g2.clearBuffer();
    u8g2.sendBuffer();

    unsigned long elapsed_start = millis();
    uint8_t  start_counter = 0;
    uint16_t start_timer   = g_config.t_blank;
    bool is_cp_resume = (cp_start > 0); 

    // ─────────────────────────────────────────
    //  RESUME DARI CHECKPOINT
    //  cp_start == 0 → mulai dari awal (no CP)
    //  cp_start 1–10 → resume dari CP index cp_start-1
    //  start_counter : index counter pertama yang dieksekusi
    //  start_timer   : durasi T-BLANK/CP sebelum masuk loop counter
    // ─────────────────────────────────────────
    if (cp_start > 0) {
        uint8_t cp_idx = cp_start - 1;
        CheckpointParam& cp = g_checkpoint[cp_idx];
        if (cp.counter_pos != 0xFF) {
            start_counter = cp.counter_pos;
            start_timer   = cp.timer_cp;
        }
    }

    // ─────────────────────────────────────────
    //  T-BLANK / CP TIMER
    //  Robot following garis selama start_timer tick
    //  Normal start → durasi = g_config.t_blank
    //  Resume CP    → durasi = cp.timer_cp
    //  decode_zone(0) dipanggil untuk deteksi zona selama fase ini
    // ─────────────────────────────────────────
    g_flag_cond = 0;
    reset_timer();
    decode_zone(0);
    while (read_timer() < start_timer) {
        led_lcd(false);
        led_timer((read_timer() / 5) % 2 == 1);
        following(g_config.speed_mode, g_config.kp);
    }

    // Reset timer agar counter pertama mulai dari 0
    // (bukan sisa timer dari T-BLANK/CP)
    reset_timer();

    // ─────────────────────────────────────────
    //  LOOP COUNTER SEKUENSIAL
    //  Iterasi dari start_counter sampai COUNTER_MAX-2
    //  cur = counter saat ini (yang sedang dijalankan)
    //  nxt = counter berikutnya (yang akan di-trigger dan dieksekusi)
    //  Setiap counter: robot following, lalu cek trigger nxt,
    //  lalu eksekusi decision nxt, lalu reset untuk counter berikutnya
    // ─────────────────────────────────────────
    for (uint8_t cidx = start_counter; cidx < COUNTER_MAX - 1; cidx++) {
        g_counter_idx = cidx;

        // ── CEK CP ──
        for (uint8_t cp_i = 0; cp_i < CP_MAX; cp_i++) {
            if (g_checkpoint[cp_i].counter_pos != 0xFF &&
                g_checkpoint[cp_i].counter_pos == cidx) {
                g_last_cp = cp_i + 1;  // 1-based, sama dengan cp_sel di standby
            }
        }

        CounterParam& cur = g_counter[cidx];
        CounterParam& nxt = g_counter[cidx + 1];

        bool is_mirrored = g_config.mirrored;
        bool skip_cur = is_cp_resume && (cidx == start_counter); 

        // ── DURASI COUNTER CUR ──
        // cur_tick_target > 0 → durasi pakai encoder (Encd_l atau Encd_r)
        // cur_tick_target = 0 → durasi pakai cur.timer
        // enc_counter_start   → referensi encoder di awal counter ini
        int16_t cur_tick_target   = resolve_encd_cur(cur.Encd_l, cur.Encd_r);
        bool cur_use_left         = (is_mirrored ? cur.Encd_r : cur.Encd_l) > 0 || (cur.Encd_l == 0 && cur.Encd_r == 0);
        int32_t enc_counter_start = cur_use_left ? encoderKiriRead() : encoderKananRead();

        // ── STATE VARIABEL PER COUNTER ──
        uint16_t sensor_akumulasi  = 0;     // OR akumulasi sensor sejak waktu habis
        bool beraksi           = false;     // flag: trigger matched, siap eksekusi
        bool timer_sudah_habis = false;     // flag: waktu cur sudah habis (sekali saja)
        bool tick_init         = false;     // flag: tick_start sudah diinit untuk TRIGGER_TICK
        bool sensor_cleared    = false;     // flag: sensor sudah pernah 0 setelah waktu habis
        int32_t tick_start     = 0;         // referensi encoder untuk TRIGGER_TICK nxt

        // ─────────────────────────────────────────
        //  LOOP UTAMA — jalan terus sampai trigger nxt matched
        // ─────────────────────────────────────────
        while (!beraksi) {
            if (sw_save() || sw_x()) {
                motor_stop();
                return g_last_cp;
            }

            int t = read_timer();

            // tentukan encoder sisi mana yang dipakai untuk cur
            // prioritas: Encd_l dulu, kalau 0 pakai Encd_r
            // mirror: swap sisi


            int32_t enc_now = abs((cur_use_left ? encoderKiriRead() : encoderKananRead()) - enc_counter_start);

            // ── LED STATUS ──
            bool waktu_habis_led = (cur_tick_target > 0)
                ? (enc_now >= cur_tick_target)
                : (t >= cur.timer);
            led_lcd(waktu_habis_led);
            led_timer((t / 5) % 2 == 1);

            // ── RAMP SPEED ──
            uint8_t spd;
            if (cur_tick_target > 0) {
                spd = ramp_speed(cur.speed1, cur.speed2,
                                (int)enc_now, (int)cur_tick_target);
            } else {
                spd = (t < cur.timer)
                    ? ramp_speed(cur.speed1, cur.speed2, t, cur.timer)
                    : cur.speed2;
            }

            // ── SENSOR & PID ──
            // scan_sensor   : baca semua sensor, hasil di g_sensor_out
            // input_error   : konversi g_sensor_out → g_pv_out (posisi garis)
            // cek_flag_cond : cek apakah seek selesai (robot nemu garis setelah belok)
            // calc_pid      : hitung output PID dari g_error
            scan_sensor();
            if (g_sensor_out != 0) g_pv_out = input_error(g_sensor_out, is_mirrored);

            bool seeking = (g_flag_cond != 0);
            bool found   = cek_flag_cond(is_mirrored);

            g_error = -g_pv_out;
            calc_pid(cur.kp, g_config.kd);

            // ── SET MOTOR ──
            // Kalau masih seeking (robot putar cari garis setelah belok) →
            //   set motor pivot sesuai arah seek
            // Kalau normal/sudah nemu garis →
            //   set motor PID following
            if (seeking && !found) {
                set_motor_seek(cur.motor_l, cur.motor_r);
            } else {
                g_LOUT = constrain((int)spd + g_out_p + g_out_d, -255, 255);
                g_ROUT = constrain((int)spd - g_out_p - g_out_d, -255, 255);
                set_motors(g_LOUT, g_ROUT, DEFAULT_MAX_PWM);
            }

            // ─────────────────────────────────────────
            //  CEK TRIGGER NXT
            //  Menentukan kapan robot beralih ke decision nxt
            //
            //  Prioritas:
            //  1. DEC_FREE + TRIGGER_TIMER/TICK → langsung beraksi (skip timer cur)
            //  2. TRIGGER_TICK                  → cek encoder, fallback timer
            //  3. Lainnya (TIMER/SENSOR/BLANK)  → tunggu waktu cur habis,
            //                                     lalu tunggu sensor match
            // ─────────────────────────────────────────


            if (nxt.decision == DEC_FREE) {
                if (nxt.trigger == TRIGGER_TIMER || nxt.trigger == TRIGGER_TICK) {
                    beraksi = true;
                } else if (!seeking) {
                    // TRIGGER_SENSOR / TRIGGER_BLANK — tunggu sensor match seperti Kasus 3,
                    // tanpa menunggu timer cur habis
                    sensor_akumulasi |= g_sensor_out;
                    if (trigger_matched(nxt.trigger, sensor_akumulasi, g_sensor_out, is_mirrored)) {
                        beraksi = true;
                    }
                }

            // Kasus 2: TRIGGER_TICK — durasi trigger pakai encoder
            // tick_start diinit sekali saat pertama masuk kondisi ini
            // target > 0 → cek encoder; target == 0 → fallback waktu cur
            } else if (!seeking && nxt.trigger == TRIGGER_TICK) {
                bool nxt_use_left = (is_mirrored ? nxt.Encd_r : nxt.Encd_l) > 0;
                if (!tick_init) {
                    tick_start = nxt_use_left ? encoderKiriRead() : encoderKananRead();
                    tick_init  = true;
                }
                int32_t tick_now   = nxt_use_left ? encoderKiriRead() : encoderKananRead();
                int32_t tick_delta = abs(tick_now - tick_start); // pastikan abs() agar bisa mundur

                int16_t target = resolve_encd_cur(nxt.Encd_l, nxt.Encd_r);  // ambil nilai yang ada

                if (target > 0) {
                    if (tick_delta >= target) beraksi = true;
                } else {
                    // fallback ke waktu cur
                    bool waktu_habis = (cur_tick_target > 0)
                        ? (enc_now >= cur_tick_target)
                        : (t >= cur.timer);
                    if (waktu_habis) beraksi = true;
                }

            // Kasus 3: TRIGGER_TIMER / TRIGGER_SENSOR / TRIGGER_BLANK
            // Tidak cek saat seeking — tunggu robot lurus dulu
            // Alur: tunggu waktu habis → tunggu sensor bersih → akumulasi sensor → match
            } else if (!seeking) {
                bool waktu_habis = skip_cur ? true :
                    (cur_tick_target > 0)
                    ? (enc_now >= cur_tick_target)
                    : (t >= cur.timer);

                if (waktu_habis) {
                    // Reset akumulasi sekali saat pertama kali waktu habis
                    if (!timer_sudah_habis) {
                        sensor_akumulasi  = 0;
                        timer_sudah_habis = true;
                    }
                    // // Tunggu sensor bersih dulu sebelum mulai akumulasi
                    // // Mencegah trigger dari sisa sensor sebelum posisi target
                    // if (!sensor_cleared) {
                    //     if (g_sensor_out == 0) sensor_cleared = true;
                    // }
                    // // Akumulasi OR sensor — catat semua sensor yang pernah aktif
                    // if (sensor_cleared) {
                    //     sensor_akumulasi |= g_sensor_out;
                    // }
                    sensor_akumulasi |= g_sensor_out;
                    // Cek apakah pola sensor sudah match dengan trigger nxt
                    if (trigger_matched(nxt.trigger, sensor_akumulasi, g_sensor_out, is_mirrored)) {
                        beraksi = true;
                    }
                }
            }

        } // end while(!beraksi)

        // ─────────────────────────────────────────
        //  EKSEKUSI DECISION NXT
        //  Jalankan aksi sesuai nxt.decision:
        //  BELOK_KANAN/KIRI → pivot lalu seek
        //  FREE             → jalan bebas (encoder/timer)
        //  STOP             → following lalu berhenti & finish
        //  Return true kalau DEC_STOP (mode selesai)
        // ─────────────────────────────────────────
        if (eksekusi_decision(nxt, is_mirrored, elapsed_start)) return 0;

        // ─────────────────────────────────────────
        //  TUNGGU SEEKING SELESAI
        //  Setelah belok, robot pivot mencari garis
        //  Loop ini jalan selama g_flag_cond != 0
        //  Begitu sensor nemu garis → cek_flag_cond reset flag,
        //  lalu motor langsung PID normal dengan speed nxt.speed2
        // ─────────────────────────────────────────
        while (g_flag_cond != 0) {
            if (sw_save() || sw_x()) {
                motor_stop();
                g_flag_cond = 0;
                return g_last_cp;
            }
            scan_sensor();
            if (g_sensor_out != 0) g_pv_out = input_error(g_sensor_out, is_mirrored);

            bool found = cek_flag_cond(is_mirrored);

            g_error = -g_pv_out;
            calc_pid(nxt.kp, g_config.kd);

            if (g_flag_cond != 0) {
                set_motor_seek(nxt.motor_l, nxt.motor_r);
            } else {
                g_LOUT = constrain((int)nxt.speed2 + g_out_p + g_out_d, -255, 255);
                g_ROUT = constrain((int)nxt.speed2 - g_out_p - g_out_d, -255, 255);
                set_motors(g_LOUT, g_ROUT, DEFAULT_MAX_PWM);
            }
        }

        // ─────────────────────────────────────────
        //  RESET UNTUK COUNTER BERIKUTNYA
        //  clear_pid   : reset integral/derivatif PID
        //  reset_timer : timer mulai dari 0 untuk counter berikutnya
        //  decode_zone : update zona/line counter sesuai Line_C cur
        // ─────────────────────────────────────────
        clear_pid();
        reset_timer();
        decode_zone(g_counter[cidx + 1].Line_C);
        led_lcd(false);
        led_timer(false);

    } // end for cidx

    // ─────────────────────────────────────────
    //  SEMUA COUNTER HABIS TANPA DEC_STOP
    //  Terjadi kalau semua counter sudah dieksekusi
    //  tapi tidak ada yang ber-decision STOP
    //  Robot berhenti, tampil finish screen
    // ─────────────────────────────────────────
    motor_stop();
    g_flag_cond = 0;
    led_lcd(false);
    led_timer(false);
    display_finish(millis() - elapsed_start);
    while (!sw_save() && !sw_x());
    return true;
}