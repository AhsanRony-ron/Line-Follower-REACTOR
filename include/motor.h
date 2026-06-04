#pragma once
#include <Arduino.h>
#include "config.h"

// ─────────────────────────────────────────
//  SET MOTOR
//  LPWM, RPWM : -255 sampai 255
//  maxpwm     : batas atas absolut
//
//  stm32duino: gunakan analogWrite() bukan pwmWrite()
//  analogWriteResolution(8) dipanggil di setup() → range 0-255
// ─────────────────────────────────────────

void set_motors(int lpwm, int rpwm, uint8_t maxpwm) {
    // clamp maxpwm
    if (maxpwm > PWM_MAX) maxpwm = PWM_MAX;
    if (maxpwm < 0)       maxpwm = 0;

    // clamp output ke maxpwm
    if (lpwm >  maxpwm) lpwm =  maxpwm;
    if (lpwm < -maxpwm) lpwm = -maxpwm;
    if (rpwm >  maxpwm) rpwm =  maxpwm;
    if (rpwm < -maxpwm) rpwm = -maxpwm;

    // motor kiri
    if (lpwm == 0) {
        analogWrite(PIN_LPWM1, 0);
        analogWrite(PIN_LPWM2, 0);
    } else if (lpwm > 0) {
        analogWrite(PIN_LPWM2, 0);
        analogWrite(PIN_LPWM1, lpwm);
    } else {
        analogWrite(PIN_LPWM2, -lpwm);
        analogWrite(PIN_LPWM1, 0);
    }

    // motor kanan
    if (rpwm == 0) {
        analogWrite(PIN_RPWM1, 0);
        analogWrite(PIN_RPWM2, 0);
    } else if (rpwm > 0) {
        analogWrite(PIN_RPWM2, rpwm);
        analogWrite(PIN_RPWM1, 0);
    } else {
        analogWrite(PIN_RPWM2, 0);
        analogWrite(PIN_RPWM1, -rpwm);
    }
}

// ─────────────────────────────────────────
//  HELPER GERAKAN DASAR
// ─────────────────────────────────────────

inline void motor_stop()               { set_motors(0,    0,    PWM_MAX); }
inline void motor_maju(int spd)        { set_motors(spd,  spd,  PWM_MAX); }
inline void motor_mundur(int spd)      { set_motors(-spd, -spd, PWM_MAX); }
inline void motor_putar_kanan(int spd) { set_motors(spd,  -spd, PWM_MAX); }
inline void motor_putar_kiri(int spd)  { set_motors(-spd,  spd, PWM_MAX); }
inline void motor_belok_kanan(int spd) { set_motors(spd,   0,   PWM_MAX); }
inline void motor_belok_kiri(int spd)  { set_motors(0,    spd,  PWM_MAX); }