#pragma once

// ─────────────────────────────────────────
//  PIN DEFINITIONS
// ─────────────────────────────────────────

// Multiplexer (sensor)
#define PIN_ADC0    PA0   // input ADC ch 0-7
#define PIN_ADC1    PA1   // input ADC ch 8-13
#define PIN_MUX_A   PA2   // mux control bit A
#define PIN_MUX_B   PA3   // mux control bit B
#define PIN_MUX_C   PA4   // mux control bit C

// Motor kiri
#define PIN_LPWM1   PA9   // TIMER1_CH2
#define PIN_LPWM2   PA10  // TIMER1_CH3

// Motor kanan
#define PIN_RPWM1   PB8   // TIMER4_CH3
#define PIN_RPWM2   PB9   // TIMER4_CH4

// Tombol (active LOW, INPUT_PULLUP)
#define PIN_SW_NEXT     PB5   // sw1 - next
#define PIN_SW_BACK     PB4   // sw2 - back
#define PIN_SW_UP       PA12  // sw3 - up
#define PIN_SW_DOWN     PA11  // sw4 - down
#define PIN_SW_SAVE     PA15  // sw5 - save
#define PIN_SW_X        PB3   // sw6 - x

// Encoder
#define ENC_KIRI_A  PB12
#define ENC_KIRI_B  PB13
#define ENC_KANAN_A PB14
#define ENC_KANAN_B PB15

// Display
#define PIN_OLED_RESET  PA8
#define PIN_LED_LCD     PC14
#define PIN_LED_TIMER   PC13

// Sensor tegangan baterai
#define PIN_VSENSOR     PA5

// ─────────────────────────────────────────
//  HARDWARE CONSTANTS
// ─────────────────────────────────────────

// OLED
#define OLED_ADDRESS    0x3C
#define OLED_WIDTH      128
#define OLED_HEIGHT     64

// Sensor
#define SENSOR_COUNT    14

// Motor
#define PWM_MAX         255
#define PWM_FREQ_DEFAULT 5.0f   // kHz
#define PWM_FREQ_MIN     0.0f
#define PWM_FREQ_MAX    20.0f

// Voltase baterai (pembagi tegangan R1=10k, R2=1k)
#define VOLT_R1         10000.0f
#define VOLT_R2         1000.0f
#define VOLT_ADC_REF    3.3f
#define VOLT_ADC_MAX    4095.0f
#define VOLT_FILTER_ALPHA 0.5f

// Timer sistem (interrupt interval)
#define TIMER_INTERVAL_US   10000   // 10ms per cacah

// EEPROM
#define EEPROM_DEVICE_ADDR  0x50
#define EEPROM_SLOT_COUNT   14

// ─────────────────────────────────────────
//  MULTIPLEXER LOOKUP TABLE
// ─────────────────────────────────────────
static const uint8_t MUX_A[] = {0, 1, 0, 1, 0, 1, 0, 1};
static const uint8_t MUX_B[] = {0, 0, 1, 1, 0, 0, 1, 1};
static const uint8_t MUX_C[] = {0, 0, 0, 0, 1, 1, 1, 1};

volatile uint8_t flag_dir = 0; 
static int16_t g_seek_l = 0;
static int16_t g_seek_r = 0;

volatile int32_t countKiri  = 0;
volatile int32_t countKanan = 0;

uint8_t tick_period;  // PID dihitung setiap N tick encoder

bool g_batt_low = false; 
