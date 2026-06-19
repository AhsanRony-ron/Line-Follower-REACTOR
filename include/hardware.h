#pragma once
#include <Arduino.h>
#include <HardwareTimer.h>
#include "config.h"
#include "types.h"

#define VOLT_CALIB  (11.34f / 10.5f)
// ─────────────────────────────────────────
//  FORWARD DECLARATIONS
// ─────────────────────────────────────────
extern GlobalConfig g_config;
extern volatile int g_timer;
extern volatile int g_cacah;
extern volatile int32_t countKiri;
extern volatile int32_t countKanan;
void timer_isr();  // definisi di main.cpp

float g_volt_filtered = 0.0f;  // simpan di global

// ─────────────────────────────────────────
//  HARDWARE TIMER INSTANCES
//  Didefinisikan sekali di sini (header-only guard via #pragma once)
// ─────────────────────────────────────────
static HardwareTimer* _htim1 = nullptr;
static HardwareTimer* _htim2 = nullptr;
static HardwareTimer* _htim4 = nullptr;

// ─────────────────────────────────────────
//  TOMBOL
// ─────────────────────────────────────────

inline bool sw_next() { return digitalRead(PIN_SW_NEXT) == LOW; }
inline bool sw_back() { return digitalRead(PIN_SW_BACK) == LOW; }
inline bool sw_up()   { return digitalRead(PIN_SW_UP)   == LOW; }
inline bool sw_down() { return digitalRead(PIN_SW_DOWN) == LOW; }
inline bool sw_save() { return digitalRead(PIN_SW_SAVE) == LOW; }
inline bool sw_x()    { return digitalRead(PIN_SW_X)    == LOW; }

// ─────────────────────────────────────────
//  LED
// ─────────────────────────────────────────

inline void led_lcd(bool on)   { digitalWrite(PIN_LED_LCD,   on ? LOW : HIGH); }
inline void led_timer(bool on) { digitalWrite(PIN_LED_TIMER, on ? LOW  : HIGH); } // active LOW

// ─────────────────────────────────────────
//  VOLTASE BATERAI
// ─────────────────────────────────────────

float read_voltage() {
    long sum = 0;
    for (int i = 0; i < 10; i++) sum += analogRead(PIN_VSENSOR);
    float adc_avg = (float)sum / 10.0f;
    float volt    = adc_avg * VOLT_ADC_REF / 1023.0f;
    float voltout = volt * ((VOLT_R1 + VOLT_R2) / VOLT_R2);
    float raw     = voltout * VOLT_CALIB;

    // seed pertama kali supaya tidak mulai dari 0
    if (g_volt_filtered == 0.0f) g_volt_filtered = raw;

    // low-pass filter: filtered = alpha*raw + (1-alpha)*filtered
    g_volt_filtered = VOLT_FILTER_ALPHA * raw + (1.0f - VOLT_FILTER_ALPHA) * g_volt_filtered;

    return g_volt_filtered;
}

uint8_t batt_percent(float voltage) {
    if (voltage >= 12.6f) { g_batt_low = false; return 100; }
    if (voltage <= 10.5f) { g_batt_low = true;  return 0;   }
    g_batt_low = false;
    return (uint8_t)((voltage - 10.5f) / 2.1f * 100.0f);
}

void isrKiriA() {
    bool a = digitalRead(ENC_KIRI_A);
    bool b = digitalRead(ENC_KIRI_B);
    countKiri += (a == b) ? +1 : -1;
}

void isrKananA() {
    bool a = digitalRead(ENC_KANAN_A);
    bool b = digitalRead(ENC_KANAN_B);
    countKanan += (a == b) ? -1 : +1;
}
int32_t encoderKiriRead() {
    noInterrupts();
    int32_t val = countKiri;
    interrupts();
    return val;
}
int32_t encoderKananRead() {
    noInterrupts();
    int32_t val = countKanan;
    interrupts();
    return val;
}

void encoderKiriReset() {
    noInterrupts();
    countKiri = 0;
    interrupts();
}
void encoderKananReset() { countKanan = 0; }

// ─────────────────────────────────────────
//  PWM INIT
//  stm32duino HardwareTimer API:
//    setOverflow(freq, HERTZ_FORMAT)  — set frekuensi PWM
//    resume()                         — mulai timer
//
//  Pin PA9  = TIM1_CH2, PA10 = TIM1_CH3
//  Pin PB8  = TIM4_CH3, PB9  = TIM4_CH4
// ─────────────────────────────────────────

void pwm_init(float freq_khz) {
    uint32_t freq_hz = (uint32_t)(freq_khz * 1000.0f);

    // Setup pin sebagai PWM output
    pinMode(PIN_LPWM1, OUTPUT);
    pinMode(PIN_LPWM2, OUTPUT);
    pinMode(PIN_RPWM1, OUTPUT);
    pinMode(PIN_RPWM2, OUTPUT);

    // Timer1 untuk motor kiri (PA9=TIM1_CH2, PA10=TIM1_CH3)
    _htim1 = new HardwareTimer(TIM1);
    _htim1->setMode(2, TIMER_OUTPUT_COMPARE_PWM1, PIN_LPWM1);
    _htim1->setMode(3, TIMER_OUTPUT_COMPARE_PWM1, PIN_LPWM2);
    _htim1->setOverflow(freq_hz, HERTZ_FORMAT);
    _htim1->setCaptureCompare(2, 0, PERCENT_COMPARE_FORMAT);
    _htim1->setCaptureCompare(3, 0, PERCENT_COMPARE_FORMAT);
    _htim1->resume();

    // Timer4 untuk motor kanan (PB8=TIM4_CH3, PB9=TIM4_CH4)
    _htim4 = new HardwareTimer(TIM4);
    _htim4->setMode(3, TIMER_OUTPUT_COMPARE_PWM1, PIN_RPWM1);
    _htim4->setMode(4, TIMER_OUTPUT_COMPARE_PWM1, PIN_RPWM2);
    _htim4->setOverflow(freq_hz, HERTZ_FORMAT);
    _htim4->setCaptureCompare(3, 0, PERCENT_COMPARE_FORMAT);
    _htim4->setCaptureCompare(4, 0, PERCENT_COMPARE_FORMAT);
    _htim4->resume();
}

// ─────────────────────────────────────────
//  TIMER INTERRUPT (sistem cacah 10ms)
//  Timer2 dipakai sebagai pure interrupt timer (bukan PWM)
//  1000 Hz = 1ms per interrupt
//  g_timer naik setiap g_config.periode × 1ms (ikuti referensi panzer.h)
// ─────────────────────────────────────────

void timer_init() {
    _htim2 = new HardwareTimer(TIM2);
    _htim2->setOverflow(1000, HERTZ_FORMAT);  // 1ms per interrupt
    _htim2->attachInterrupt(timer_isr);
    _htim2->resume();
}

// ─────────────────────────────────────────
//  INIT SEMUA PIN
// ─────────────────────────────────────────

void hardware_init() {
    Wire.begin();

    // Tombol
    pinMode(PIN_SW_NEXT, INPUT_PULLUP);
    pinMode(PIN_SW_BACK, INPUT_PULLUP);
    pinMode(PIN_SW_UP,   INPUT_PULLUP);
    pinMode(PIN_SW_DOWN, INPUT_PULLUP);
    pinMode(PIN_SW_SAVE, INPUT_PULLUP);
    pinMode(PIN_SW_X,    INPUT_PULLUP);

    // Multiplexer control
    pinMode(PIN_MUX_A, OUTPUT);
    pinMode(PIN_MUX_B, OUTPUT);
    pinMode(PIN_MUX_C, OUTPUT);

    // ADC sensor
    pinMode(PIN_ADC0, INPUT_ANALOG);
    pinMode(PIN_ADC1, INPUT_ANALOG);

    // Voltase
    pinMode(PIN_VSENSOR, INPUT_ANALOG);

    // LED
    pinMode(PIN_LED_LCD,   OUTPUT);
    pinMode(PIN_LED_TIMER, OUTPUT);
    digitalWrite(PIN_LED_LCD,       HIGH); // active LOW
    digitalWrite(PIN_LED_TIMER, HIGH);

    pinMode(ENC_KIRI_A,  INPUT_PULLUP);
    pinMode(ENC_KIRI_B,  INPUT_PULLUP);
    pinMode(ENC_KANAN_A, INPUT_PULLUP);
    pinMode(ENC_KANAN_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC_KIRI_A),  isrKiriA,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_KANAN_A), isrKananA, CHANGE);

    // PWM motor & Timer interrupt
    // Catatan: pinMode motor dilakukan di dalam pwm_init()
    pwm_init(DEFAULT_PWM_FREQ);
    timer_init();
}

// ─────────────────────────────────────────
//  SOFT RESET — Restart STM32
//  Dipakai setelah ganti mem_slot untuk reload data
// ─────────────────────────────────────────
inline void system_restart() {
    delay(100);  // tunggu buffer flush
    NVIC_SystemReset();  // Trigger soft reset
}

