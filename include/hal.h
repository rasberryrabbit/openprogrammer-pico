/*
 * hal.h — Hardware Abstraction Layer for the Open Programmer Pico port.
 *
 * This file is the bridge between the original PIC18 OPcontrol.c (which
 * talks directly to SFRs like LATBbits.LATB5) and the RP2040 hardware.
 * The original firmware expresses every ICSP/I2C/SPI/JTAG/OW bit-bang
 * operation through a small set of macros and helper functions
 * (D0/D1/CK0/CK1/CKpulse/Nop/INTCONbits.GIE/...) defined in OPcontrol.c.
 *
 * For the Pico port we KEEP those macro names (so OPcontrol.c needs minimal
 * edits) but redefine them here to operate on RP2040 GPIOs via the Pico SDK.
 *
 * ---- Pin mapping (RP2040) ----------------------------------------------
 *
 * The Open Programmer hardware routes ICSP signals to fixed PIC port pins:
 *   RB0 = SDA / uW data in  / SPI MISO
 *   RB1 = SCL / SPI clock   / uW clock
 *   RB3 = A0 / SPI CS       / CCP2 mux
 *   RB4 = A1 / SPI HOLD
 *   RB5 = D   (ICSP data)
 *   RB6 = CK  (ICSP clock)
 *   RB7 = PGM
 *   RA0 = AN0 (Vreg feedback ADC)
 *   RA1 = LED1
 *   RA2 = LED2
 *   RC0 = VPP control
 *   RC1 = VCC control
 *   RC2 = PWM1 (DCDC converter gate)
 *   RC6 = WP (write protect for some EEPROMs)
 *   RC7 = SDO (SPI MOSI out)
 *   RE3 = SW1 (programmer mode select button)
 *
 * RP2040 pin assignment (chosen to keep hardware PWM slice 0 channel A on
 * the DCDC gate, ADC input 0 on the Vreg feedback, and all signal pins on
 * contiguous GPIO numbers for an easy PCB layout). Adjust PIN_* constants
 * below to match your board.
 *
 *   PIN_D      GP5   (was RB5)
 *   PIN_CK     GP6   (was RB6)
 *   PIN_PGM    GP7   (was RB7)
 *   PIN_SDA    GP10  (was RB0)
 *   PIN_SCL    GP11  (was RB1)
 *   PIN_A0     GP14  (was RB3)
 *   PIN_A1     GP15  (was RB4)
 *   PIN_LED1   GP18  (was RA1)
 *   PIN_LED2   GP19  (was RA2)
 *   PIN_VPP    GP20  (was RC0)
 *   PIN_VCC    GP21  (was RC1)
 *   PIN_PWM    GP22  (was RC2 / CCP1)  -- PWM slice 3 channel A
 *   PIN_WP     GP26  (was RC6)
 *   PIN_SDO    GP27  (was RC7)
 *   PIN_SW1    GP17  (was RE3)         -- mode select button (active low)
 *   ADC_VREG   ADC0  (was AN0 / RA0)   -> GP26 is ADC0, but we re-route Vreg
 *                                        feedback to GP27 = ADC1; see hal.c.
 *
 * The DCDC feedback ADC input is wired to GP27 (ADC1) on this board because
 * GP26 is taken by PIN_WP. If you wire Vreg feedback to a different ADC pin,
 * update ADC_VREG_CHANNEL in hal.c.
 */
#ifndef HAL_H
#define HAL_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/sync.h"   // save_and_disable_interrupts / restore_interrupts

#ifdef __cplusplus
extern "C" {
#endif

// ---- Pin assignments (adjust to your board) ------------------------------
#define PIN_D        5
#define PIN_CK       6
#define PIN_PGM      7
#define PIN_SDA      10
#define PIN_SCL      11
#define PIN_A0       14
#define PIN_A1       15
#define PIN_SW1      17
#define PIN_LED1     18
#define PIN_LED2     19
#define PIN_VPP      20
#define PIN_VCC      21
#define PIN_PWM      22
#define PIN_WP       26
#define PIN_SDO      27
#define PIN_VREG_FB  28   // ADC input 2 (Vreg * 12k / 34k feedback)

#define ADC_VREG_CHANNEL 2  // ADC2 = GP28

// ---- AT_HV_RTX pin aliases (AVR high-voltage serial programming) ---------
// The original firmware maps AVR HVSP signals onto PORTB/PORTC pins:
//   PB1 = RB0 (PIN_SDA)   — HVSP SDI (data to AVR)
//   PB2 = RB1 (PIN_SCL)   — HVSP SDO (data from AVR)
//   PB0 = RC7 (PIN_SDO)   — HVSP SII (instruction)
//   PB3 = RC6 (PIN_WP)    — HVSP SCK (clock)
// The macros PB0..PB3 below carry the RP2040 pin numbers so the AT_HV_RTX
// handler can drive them with the same names as the PIC firmware.
#define PB1          PIN_SDA
#define PB2          PIN_SCL
#define PB0          PIN_SDO
#define PB3          PIN_WP

// ---- Signal macros (drop-in for the PIC SFR macros in OPcontrol.c) --------
//
// The original OPcontrol.c uses LATBbits.LATB5 etc. We provide a parallel
// set of macros with identical names so the ported OPcontrol.c reads almost
// the same as the original.
//
// D (data) line — ICSP data
// Helper macros with the same names as the PIC firmware.
//   D_bit = x   -> set D line level
//   Ddir_bit = x -> set D line direction (1=input, 0=output)
// These are used as `D_bit = 0;` style assignments in the original code.
// On RP2040 we route them through helper functions; we use macros that
// evaluate the RHS exactly once.
#define D_bit         HAL_GPIO_ASSIGN(PIN_D)
#define Ddir_bit      HAL_DIR_ASSIGN(PIN_D)
#define CK_bit        HAL_GPIO_ASSIGN(PIN_CK)
#define CKdir_bit     HAL_DIR_ASSIGN(PIN_CK)
#define PGM_bit       HAL_GPIO_ASSIGN(PIN_PGM)
#define PGMdir_bit    HAL_DIR_ASSIGN(PIN_PGM)

#define D0()          gpio_put(PIN_D, 0)
#define D1()          gpio_put(PIN_D, 1)
#define Din()         gpio_set_dir(PIN_D, GPIO_IN)
#define Dout()        gpio_set_dir(PIN_D, GPIO_OUT)

#define CK0()         gpio_put(PIN_CK, 0)
#define CK1()         gpio_put(PIN_CK, 1)
#define CKin()        gpio_set_dir(PIN_CK, GPIO_IN)
#define CKout()       gpio_set_dir(PIN_CK, GPIO_OUT)

// CK pulse variants. The PIC firmware distinguishes three pulse widths:
//   CKpulse()  -> fastest (~250 ns)
//   CKpulseN() -> normal (~500 ns, used for ICSP bit-bang)
//   CKpulseL() -> long (~750 ns, used for PIC24 SIX)
// On RP2040 at 133 MHz we use the RAM-resident exact cycle-counted delay
// (hal_delay_cycles) so CK half-periods are deterministic, not "at least".
// PIN_CK stays on SIO for the whole firmware: JTAG is host-driven bit-bang
// (TCKpulse -> CKpulse here) and never goes through a PIO block, so its
// FUNCSEL is never taken by pio_gpio_init().
#define CKpulse()     do { gpio_put(PIN_CK,1); __compiler_memory_barrier(); \
                           gpio_put(PIN_CK,0); } while(0)
#define CKpulseN()    do { gpio_put(PIN_CK,1); hal_delay_cycles(8); \
                           gpio_put(PIN_CK,0); hal_delay_cycles(8); } while(0)
#define CKpulseL()    do { gpio_put(PIN_CK,1); hal_delay_cycles(16); \
                           gpio_put(PIN_CK,0); } while(0)

#define PGM0()        gpio_put(PIN_PGM, 0)
#define PGM1()        gpio_put(PIN_PGM, 1)
#define PGMin()       gpio_set_dir(PIN_PGM, GPIO_IN)
#define PGMout()      gpio_set_dir(PIN_PGM, GPIO_OUT)

// Software I2C pins (SDA = GP10, SCL = GP11) — mirror of the PIC RB0/RB1
#define SCL           gpio_put(PIN_SCL, 0)   // used as `SCL = x` in some places
#define SDA           gpio_put(PIN_SDA, 0)
#define SDA_p         (gpio_get(PIN_SDA))
#define SCL_dir       HAL_DIR_ASSIGN(PIN_SCL)
#define SDA_dir       HAL_DIR_ASSIGN(PIN_SDA)
#define DATA_LOW      (gpio_set_dir(PIN_SDA, GPIO_OUT))
#define DATA_HI       (gpio_set_dir(PIN_SDA, GPIO_IN))
#define DATA_LAT      (gpio_put(PIN_SDA, 0), 1)
#define DATA_PIN      (gpio_get(PIN_SDA))
#define CLOCK_LOW     (gpio_set_dir(PIN_SCL, GPIO_OUT))
#define CLOCK_HI      (gpio_set_dir(PIN_SCL, GPIO_IN))
#define SCLK_LAT      (gpio_put(PIN_SCL, 0), 1)

// Software SPI pins (SCK = GP11, MOSI/SDO = GP27, MISO = GP10)
// Note: the original firmware multiplexes SPI clock onto RB1 with I2C SCL.
// On the Pico we keep that multiplexing (same physical pin), so SCK = PIN_SCL.
#define SPI_CS        gpio_put(PIN_A0, 0)    // LATBbits.LATB3
#define SPI_HLD       gpio_put(PIN_A1, 0)    // LATBbits.LATB4
#define SPICK_dir     HAL_DIR_ASSIGN(PIN_SCL)
#define SPICK         gpio_put(PIN_SCL, 0)
#define SDO           gpio_put(PIN_SDO, 0)
#define SDO_dir       HAL_DIR_ASSIGN(PIN_SDO)

// Microwire pins (clock on RB1, data in on RB0, data out on RC7) — same as
// the original which multiplexes uWCK with SCL and uWDI with SDA.
#define uWCK_dir      HAL_DIR_ASSIGN(PIN_SCL)
#define uWCK          gpio_put(PIN_SCL, 0)
#define uWCKnum       11
#define uWDI_dir      HAL_DIR_ASSIGN(PIN_SDA)
#define uWDI          gpio_put(PIN_SDA, 0)
#define uWDInum       10
#define uWDO_dir      HAL_DIR_ASSIGN(PIN_SDO)
#define uWDO          gpio_put(PIN_SDO, 0)
#define uWDOnum       27

// JTAG pins (all on PORTB in the original: TCK=RB6, TDI=RB7, TDO=RB5, TMS=RB4).
// On the Pico we keep TCK on PIN_CK, TDI on PIN_PGM, TDO on PIN_D, TMS on PIN_A1.
#define TCKnum        PIN_CK
#define TDInum        PIN_PGM
#define TDOnum        PIN_D
#define TMSnum        PIN_A1
#define TCKpulse()    CKpulse()

// PORTA LED pins
#define LED1          (gpio_put(PIN_LED1, 0), 1)  // wrapped; assign via helper
#define LED2          (gpio_put(PIN_LED2, 0), 1)
// Re-define as assignable lvalues for `LED1 = !LED2;` style original code:
#undef  LED1
#undef  LED2
#define LED1          HAL_GPIO_ASSIGN(PIN_LED1)
#define LED2          HAL_GPIO_ASSIGN(PIN_LED2)

// PORTC VPP/VCC control bits
#define vcc_bit       HAL_GPIO_ASSIGN(PIN_VCC)
#define vcc_dir       HAL_DIR_ASSIGN(PIN_VCC)
#define vpp_bit       HAL_GPIO_ASSIGN(PIN_VPP)
#define vpp_dir       HAL_DIR_ASSIGN(PIN_VPP)
#define vcc_ON()      gpio_put(PIN_VCC, 0)   // active low in original (vcc_bit=0)
#define vcc_OFF()     gpio_put(PIN_VCC, 1)
#define vpp_ON()      gpio_put(PIN_VPP, 1)   // active high in original
#define vpp_OFF()     gpio_put(PIN_VPP, 0)

#define WP            HAL_GPIO_ASSIGN(PIN_WP)
#define WP_dir        HAL_DIR_ASSIGN(PIN_WP)

// PORTE switch (mode-select button) — RE3 in original
#define SW1           (gpio_get(PIN_SW1))

// ---- Internal helpers for assignable lvalue macros -----------------------
// We need `D_bit = 0;` to set the GPIO. We cannot overload assignment to a
// function call directly in C, so we expose a tiny struct+assignment helper.
// Each `*_bit`/`*_dir` macro expands to a temporary of type hal_gpio_t whose
// operator= writes the pin. In C this is done via a compound-literal + helper
// function; the macro HAL_GPIO_ASSIGN wraps it.
typedef struct { uint8_t pin; } hal_gpio_t;
static inline void hal_gpio_set(hal_gpio_t g, int v) { gpio_put(g.pin, v ? 1 : 0); }
static inline void hal_gpio_dir(hal_gpio_t g, int d) {
    gpio_set_dir(g.pin, d ? GPIO_IN : GPIO_OUT);
}

// Usage:  D_bit = 0;    expands to   hal_gpio_assign(PIN_D) = (0);
#define HAL_GPIO_ASSIGN(pin)  (hal_gpio_lhs(pin))
#define HAL_DIR_ASSIGN(pin)   (hal_gpio_dir_lhs(pin))

// The following construction lets `x = value` actually call gpio_put.
// We rely on a tiny struct with an inline assignment helper. The temporary
// lives until the end of the full expression, so the side effect occurs.
typedef struct {
    uint8_t pin;
    void (*setfn)(uint8_t, int);
} hal_pin_lhs_t;
static inline hal_pin_lhs_t hal_gpio_lhs(uint8_t pin) {
    hal_pin_lhs_t h = { pin, (void(*)(uint8_t,int))gpio_put };
    // gpio_put signature is (uint gpio, bool val); cast to our generic shape.
    return h;
}
static inline hal_pin_lhs_t hal_gpio_dir_lhs(uint8_t pin) {
    hal_pin_lhs_t h = { pin, NULL };
    (void)h;
    return h;  // dir handled by separate macro below
}

// We can't easily override `=` on a struct in C. Use a different approach:
// expose each *_bit as a real function-call macro that takes the RHS.
#define D_bit_SET(v)     gpio_put(PIN_D, (v)?1:0)
#define CK_bit_SET(v)    gpio_put(PIN_CK, (v)?1:0)
#define PGM_bit_SET(v)   gpio_put(PIN_PGM, (v)?1:0)
#define LED1_SET(v)      gpio_put(PIN_LED1, (v)?1:0)
#define LED2_SET(v)      gpio_put(PIN_LED2, (v)?1:0)
#define vcc_bit_SET(v)   gpio_put(PIN_VCC, (v)?1:0)
#define vpp_bit_SET(v)   gpio_put(PIN_VPP, (v)?1:0)
#define WP_SET(v)        gpio_put(PIN_WP, (v)?1:0)
#define Ddir_bit_SET(v)  gpio_set_dir(PIN_D, (v)?GPIO_IN:GPIO_OUT)
#define CKdir_bit_SET(v) gpio_set_dir(PIN_CK, (v)?GPIO_IN:GPIO_OUT)
#define PGMdir_bit_SET(v) gpio_set_dir(PIN_PGM, (v)?GPIO_IN:GPIO_OUT)
#define vcc_dir_SET(v)   gpio_set_dir(PIN_VCC, (v)?GPIO_IN:GPIO_OUT)
#define vpp_dir_SET(v)   gpio_set_dir(PIN_VPP, (v)?GPIO_IN:GPIO_OUT)
#define WP_dir_SET(v)    gpio_set_dir(PIN_WP, (v)?GPIO_IN:GPIO_OUT)
#define SDO_dir_SET(v)   gpio_set_dir(PIN_SDO, (v)?GPIO_IN:GPIO_OUT)
#define SPICK_dir_SET(v) gpio_set_dir(PIN_SCL, (v)?GPIO_IN:GPIO_OUT)
#define SCL_dir_SET(v)   gpio_set_dir(PIN_SCL, (v)?GPIO_IN:GPIO_OUT)
#define SDA_dir_SET(v)   gpio_set_dir(PIN_SDA, (v)?GPIO_IN:GPIO_OUT)

// Suppress the unused *_bit macros that the original code uses as `x = value`
// syntax; the ported OPcontrol.c uses the _SET variants instead. Keeping the
// bare macros defined (as no-ops) preserves the symbol names for grep parity
// but the ported code never writes to them directly.
#undef D_bit
#undef CK_bit
#undef PGM_bit
#undef LED1
#undef LED2
#undef vcc_bit
#undef vpp_bit
#undef WP
#undef Ddir_bit
#undef CKdir_bit
#undef PGMdir_bit
#undef vcc_dir
#undef vpp_dir
#undef WP_dir
#undef SDO_dir
#undef SPICK_dir
#undef SCL_dir
#undef SDA_dir
// Read-back shims (used as `if (PORTBbits.RB5) i|=1;` in the original).
#define D_bit        (gpio_get(PIN_D))
#define CK_bit       (gpio_get(PIN_CK))
#define PGM_bit      (gpio_get(PIN_PGM))
#define LED1         hal_led1_getset()
#define LED2         hal_led2_getset()
#define vcc_bit      (gpio_get(PIN_VCC))
#define vpp_bit      (gpio_get(PIN_VPP))
#define WP           (gpio_get(PIN_WP))
#define Ddir_bit     (gpio_get_dir(PIN_D) == GPIO_IN ? 1 : 0)
#define CKdir_bit    (gpio_get_dir(PIN_CK) == GPIO_IN ? 1 : 0)
#define PGMdir_bit   (gpio_get_dir(PIN_PGM) == GPIO_IN ? 1 : 0)
#define vcc_dir      (gpio_get_dir(PIN_VCC) == GPIO_IN ? 1 : 0)
#define vpp_dir      (gpio_get_dir(PIN_VPP) == GPIO_IN ? 1 : 0)
#define WP_dir       (gpio_get_dir(PIN_WP) == GPIO_IN ? 1 : 0)
#define SDO_dir      (gpio_get_dir(PIN_SDO) == GPIO_IN ? 1 : 0)
#define SPICK_dir    (gpio_get_dir(PIN_SCL) == GPIO_IN ? 1 : 0)
#define SCL_dir      (gpio_get_dir(PIN_SCL) == GPIO_IN ? 1 : 0)
#define SDA_dir      (gpio_get_dir(PIN_SDA) == GPIO_IN ? 1 : 0)

// LED read/write: the original does `LED1=1; LED2=!LED2;` etc.
// We expose them as int-typed lvalues via a small helper. Because true
// overloaded assignment in C is impossible, the ported OPcontrol.c uses
// LED1_SET/LED2_SET for writes. The bare `LED1` symbol is provided for any
// read uses. (There are none in OPcontrol.c — LEDs are write-only.)
int hal_led1_getset(void);
int hal_led2_getset(void);
void hal_led1_write(int v);   // write LED1 (0/1)
void hal_led2_write(int v);   // write LED2 (0/1)

// ---- Timing primitives ---------------------------------------------------
// The PIC firmware uses Nop() loops calibrated for 48 MHz to produce ~1us
// delays. On RP2040 at 133 MHz we use the SDK timing helpers.
#define Nop()           __nop()
void Delay1us(uint8_t delay);     // from original
void Delay2us(uint8_t delay);     // from original
void DelayT(void);                // software I2C half-bit delay
void Wait_us(uint16_t us);        // arbitrary microsecond wait

// ---- Precision timing (RAM-resident, cycle-counter based) ----------------
// These are the jitter-free alternatives to busy_wait_us_32 / busy_wait_at_
// least_cycles for tight bit-bang protocols (One-Wire, JTAG, UNI/O,
// Microwire). Two properties make them precise:
//
//   1. RAM-resident: the functions are placed in SRAM via
//      __not_in_flash_func, so they never stall on the XIP flash cache
//      (the dominant source of ~1 us jitter on RP2040 bit-bangs).
//   2. Cycle-counter driven: instead of a NOP loop whose length the
//      compiler may fold or the CPU may stretch on a cache miss, the
//      delay spins on the free-running 24-bit DWT/SysTick cycle counter,
//      so the wait length is exact in clock cycles regardless of the
//      code path taken to reach it.
//
// Callers MUST already have disabled interrupts (save_and_disable_inter-
// rupts) so no ISR steals a slot mid-delay. The functions are safe to
// nest inside a critical section.
//
//   hal_delay_us_exact(n)   - wait exactly n microseconds (±1 cycle)
//   hal_delay_ns_exact(n)   - wait approximately n nanoseconds
//   hal_delay_cycles(n)     - wait exactly n CPU cycles
void hal_delay_us_exact(uint32_t us);
void hal_delay_ns_exact(uint32_t ns);
void hal_delay_cycles(uint32_t cycles);

// Initialise the precision-timing cycle counter. Call once at boot.
void hal_timing_init(void);

// ---- Open-drain / push-pull pin helpers ----------------------------------
// One-Wire and UNI/O need open-drain emulation on RP2040 (no native OD):
// the pin level is held at 0 and the bus is controlled by toggling the
// direction. "release" = input (external pull-up wins); "drive low" =
// output. These are RAM-resident for jitter-free slot edges.
void hal_od_release(uint pin);   // set input  -> bus goes high (pull-up)
void hal_od_drive_low(uint pin); // set output -> bus goes low (level 0)
void hal_od_init(uint pin);      // level 0, input (released)

// ---- Interrupt / critical-section emulation ------------------------------
// Original: INTCONbits.GIE = 0/1. On RP2040 we use save_and_disable_interrupts
// + restore_interrupts.
typedef uint32_t hal_irq_state_t;
#define INTCONBITS_GIE_SAVE(var)  (var) = save_and_disable_interrupts()
#define INTCONBITS_GIE_RESTORE(var) restore_interrupts((var))

// ---- ADC (Vreg feedback) -------------------------------------------------
void     hal_adc_init(void);
uint16_t hal_adc_read_raw(void);     // 12-bit raw value

// ---- PWM (DCDC converter gate) -------------------------------------------
void hal_pwm_init(uint32_t freq_hz);
void hal_pwm_set_duty_8(uint8_t duty);      // 0..255 mapped to slice CC
void hal_pwm_disable(void);

// ---- Whole-port direction / read helpers --------------------------------
// The original OPcontrol.c occasionally writes a whole TRIS register (e.g.
// `TRISB=0xC5;` for uW_INIT) or reads PORTB as a byte (e.g. READ_B). On
// RP2040 there is no port register; we provide byte-shaped helpers that
// operate on the PORTB-equivalent pins (RB0..RB7 -> PIN_SDA, PIN_SCL,
// PIN_A0, PIN_A1, PIN_PGM, PIN_D, PIN_CK, PIN_WP). The bit order matches
// the PIC18 PORTB layout so existing host-side code keeps working.
void    hal_set_trisb(uint8_t mask);   // 1=input, 0=output
uint8_t hal_read_portb(void);          // read PIN_SDA..PIN_WP as a byte

// ---- DCDC control loop ISR ----------------------------------------------
// The PIC firmware runs the PID-like control loop in a low-priority ISR at
// ~4 kHz (every 250 us, triggered by CCP2). On RP2040 we use a timer alarm
// callback at the same rate.
void hal_dcdc_loop_start(uint32_t period_us);
void hal_dcdc_loop_stop(void);
bool hal_dcdc_tick(struct repeating_timer *rt);

// ---- HLVD (high/low-voltage detect) emulation ----------------------------
// The PIC firmware uses the HLVD comparator at 4.5V to detect power-supply
// droop. On RP2040 we sample the Vreg feedback ADC at the start of each
// control-loop tick; if the sample is below a threshold we set the HLVDIF
// flag. The original semantics are preserved as flag bits.
extern volatile bool hlvdif_flag;
#define HLVDCON_HLVDIF  hlvdif_flag

// ---- Misc original SFR bits used in OPcontrol.c --------------------------
// These are referenced as `PIR2bits.HLVDIF`, `PIE1bits.ADIE`, etc. in the
// original. We provide a small shim namespace so the ported OPcontrol.c can
// keep the same field names.
typedef struct {
    volatile uint8_t ADIF  : 1;
    volatile uint8_t TMR1IF: 1;
} hal_pir1_t;
extern hal_pir1_t PIR1bits;

typedef struct {
    volatile uint8_t HLVDIF : 1;
} hal_pir2_t;
extern hal_pir2_t PIR2bits;

typedef struct {
    volatile uint8_t ADIE   : 1;
    volatile uint8_t TMR1IE : 1;
} hal_pie1_t;
extern hal_pie1_t PIE1bits;

typedef struct {
    volatile uint8_t GIE    : 1;
    volatile uint8_t TMR0IF : 1;
} hal_intcon_t;
extern hal_intcon_t INTCONbits;

// HLVDCON bit aliases (the original uses HLVDCONbits.IRVST/IVRST as well as
// the HLVDCON byte register).
typedef struct {
    volatile uint8_t IRVST : 1;
} hal_hlvdccon_t;
extern hal_hlvdccon_t HLVDCONbits;
extern volatile uint8_t HLVDCON;

// ---- HAL init ------------------------------------------------------------
void hal_init(void);

#ifdef __cplusplus
}
#endif

#endif // HAL_H
