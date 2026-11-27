/*
 * hal.c — RP2040 hardware glue for the Open Programmer Pico port.
 *
 * Implements everything declared in hal.h: pin init, timing helpers, ADC,
 * PWM (DCDC gate), the 4 kHz DCDC control-loop timer callback, and the SFR
 * shim namespace (PIR1bits/PIR2bits/INTCONbits/HLVDCONbits/...) that lets the
 * ported OPcontrol.c keep the original PIC18 field names.
 */
#include "hal.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include <string.h>
#include <stdlib.h>

// ---- SFR shim namespace --------------------------------------------------
hal_pir1_t     PIR1bits;
hal_pir2_t     PIR2bits;
hal_pie1_t     PIE1bits;
hal_intcon_t   INTCONbits;
hal_hlvdccon_t HLVDCONbits;
volatile uint8_t HLVDCON = 0;
volatile bool hlvdif_flag = false;

// LED cache (so LED1/LED2 read-back matches last write; original LEDs are
// write-only but the original code occasionally does `LED2 = !LED2`).
static int led1_state = 0, led2_state = 0;

int hal_led1_getset(void) { return led1_state; }
int hal_led2_getset(void) { return led2_state; }

// Public write helpers used by the *_SET macros when the ported OPcontrol.c
// is built. They are declared here (not static) so OPcontrol.c can call them
// via the LED1_SET/LED2_SET macros which expand to gpio_put directly.
// We also expose tiny wrappers so LED state stays in sync with the cache.
void hal_led1_write(int v) { led1_state = v ? 1 : 0; gpio_put(PIN_LED1, led1_state); }
void hal_led2_write(int v) { led2_state = v ? 1 : 0; gpio_put(PIN_LED2, led2_state); }

// ---- Timing --------------------------------------------------------------
//
// The PIC firmware's Nop() loops were calibrated for 48 MHz. On RP2040 at
// 133 MHz the cycle budget per microsecond is ~2.8x larger, so the same
// loop counts would overshoot. We use the SDK timing primitives which are
// cycle-accurate and compiler-independent.
//
// busy_wait_us() rounds up to whole microseconds and has sub-us jitter; for
// sub-microsecond bit-bang pulses we use busy_wait_at_least_cycles() in the
// CK pulse macros (see hal.h).

void Delay1us(uint8_t delay) {
    // Original: byte d = delay-2; for(;d;d--) { 7 Nops }
    // Approximate by busy_wait_us with a floor of 1.
    if (delay <= 1) return;
    busy_wait_us(delay);
}

void Delay2us(uint8_t delay) {
    busy_wait_us(delay);
}

void Wait_us(uint16_t us) {
    busy_wait_us(us);
}

// ---- Precision timing (RAM-resident, cycle-counter based) ----------------
// RP2040 has no DWT cycle counter (that's an M3/M4 feature), but the SysTick
// timer is a 24-bit down-counter clocked from the system clock. We run it
// free-running and read its value to measure exact cycle counts. Because
// the timing functions live in SRAM (__not_in_flash_func), there are no
// XIP flash stalls; the only remaining jitter source is an IRQ, which the
// caller has already disabled via save_and_disable_interrupts().
//
// SysTick is normally used by the SDK for absolute-time scheduling, so we
// do NOT reconfigure it here. Instead we use the RP2040 TIMER hardware's
// 1 us rollover counter (TIMER_HW->TIMERAWR), which is a free-running
// 64-bit microsecond counter that the SDK maintains. Reading it is a
// single LDR instruction with no side effects — ideal for a tight spin.
//
// For sub-microsecond precision we fall back to a cycle-counted NOP loop
// whose length is fixed by an asm volatile barrier (the compiler cannot
// fold or hoist it).

#include "hardware/timer.h"

// System clock frequency in Hz (set once at init).
static uint32_t hal_sys_clk_hz;

void hal_timing_init(void) {
    hal_sys_clk_hz = clock_get_hz(clk_sys);
}

// RAM-resident exact microsecond delay using the free-running 64-bit TIMER
// counter. Resolution = 1 us. Jitter < 1 us as long as interrupts are off.
__attribute__((section(".time_critical." "hal_delay_us_exact")))
void hal_delay_us_exact(uint32_t us) {
    uint64_t target = time_us_64() + us;
    while (time_us_64() < target) {
        __compiler_memory_barrier();
    }
}

// RAM-resident cycle-counted delay. n cycles at hal_sys_clk_hz.
// Uses a tight C loop that the compiler lowers to Thumb16-compatible
// instructions (SUB + BNE). Each iteration is 3 cycles on Cortex-M0+.
// The volatile counter prevents the optimizer from folding the loop.
__attribute__((section(".time_critical." "hal_delay_cycles")))
void hal_delay_cycles(uint32_t cycles) {
    if (cycles == 0) return;
    // Each loop iteration is 3 cycles (subs / bne / pipeline) on Cortex-M0+.
    volatile uint32_t iters = cycles / 3;
    uint32_t rem = cycles % 3;
    while (iters--) {
        __asm__ volatile ("nop");
    }
    // Burn the remainder (1 or 2 cycles) with NOPs.
    while (rem--) __asm__ volatile ("nop");
}

// RAM-resident nanosecond delay. Converts ns to cycles at the system clock
// frequency. Accuracy is ±1 cycle (±7.5 ns at 133 MHz).
__attribute__((section(".time_critical." "hal_delay_ns_exact")))
void hal_delay_ns_exact(uint32_t ns) {
    if (!hal_sys_clk_hz) hal_sys_clk_hz = clock_get_hz(clk_sys);
    // cycles = ns * Hz / 1e9, computed in fixed point to avoid float in
    // the time-critical section.
    uint32_t cycles = (uint32_t)(((uint64_t)ns * hal_sys_clk_hz) / 1000000000ull);
    hal_delay_cycles(cycles);
}

// ---- Open-drain emulation helpers (RAM-resident) -------------------------
// One-Wire / UNI/O buses need open-drain drive: the pin level is held at 0
// and the bus state is controlled by direction. "release" = input (the
// external 4.7k pull-up brings the bus high); "drive low" = output. These
// live in SRAM so the slot-edge timing is deterministic.

__attribute__((section(".time_critical." "hal_od_init")))
void hal_od_init(uint pin) {
    gpio_put(pin, 0);
    gpio_set_dir(pin, GPIO_IN);   // released
}

__attribute__((section(".time_critical." "hal_od_release")))
void hal_od_release(uint pin) {
    gpio_set_dir(pin, GPIO_IN);   // hi-Z -> pull-up wins
}

__attribute__((section(".time_critical." "hal_od_drive_low")))
void hal_od_drive_low(uint pin) {
    gpio_set_dir(pin, GPIO_OUT);  // drive 0 (level already 0)
}

void DelayT(void) {
    // Original: for(tt=T2;tt;tt--) { 7 Nops }. T2 default 100, so ~700 cycles
    // at 48 MHz ~ 14.6 us. On RP2040 we keep the same wall-clock ~ a few us.
    busy_wait_us(2);
}

// ---- ADC -----------------------------------------------------------------
void hal_adc_init(void) {
    adc_init();
    adc_gpio_init(PIN_VREG_FB);     // make GPIO29 an ADC input
    adc_select_input(ADC_VREG_CHANNEL);
}

uint16_t hal_adc_read_raw(void) {
    adc_select_input(ADC_VREG_CHANNEL);
    return adc_read();   // 12-bit result
}

// ---- Whole-port helpers (TRISB / PORTB emulation) -----------------------
// PIC18 PORTB bit -> RP2040 pin mapping used by the ported OPcontrol.c:
//   RB0=PIN_SDA  RB1=PIN_SCL  RB2=PIN_A0  RB3=PIN_A1
//   RB4=PIN_A1(alt) RB5=PIN_D  RB6=PIN_CK  RB7=PIN_PGM
// We use a compact table; RB2 and RB4 share PIN_A1 in the original (RA4 is
// RB4 only on certain PIC18 variants), so we map RB4 to PIN_PGM for the
// HVSP paths to keep all eight bits distinct on the RP2040.
static const uint8_t portb_pins[8] = {
    PIN_SDA, PIN_SCL, PIN_A0, PIN_A1,
    PIN_PGM, PIN_D,  PIN_CK, 0xff    // RB7 unused on this port (mapped via PIN_PGM)
};

void hal_set_trisb(uint8_t mask) {
    // bit=1 -> input, bit=0 -> output. Same convention as PIC TRIS.
    for (int b = 0; b < 8; b++) {
        uint8_t p = portb_pins[b];
        if (p == 0xff) continue;
        gpio_set_dir(p, (mask & (1 << b)) ? GPIO_IN : GPIO_OUT);
    }
}

uint8_t hal_read_portb(void) {
    uint8_t v = 0;
    for (int b = 0; b < 8; b++) {
        uint8_t p = portb_pins[b];
        if (p == 0xff) continue;
        if (gpio_get(p)) v |= (1 << b);
    }
    return v;
}

// ---- PWM (DCDC gate) -----------------------------------------------------
static uint8_t  pwm_slice_num;
static uint32_t pwm_top;
static bool     pwm_running = false;

void hal_pwm_init(uint32_t freq_hz) {
    // Configure PIN_PWM as PWM output. The original uses CCP1 in PWM mode
    // with PR2=0x84 (~90 kHz). We default to 90 kHz to match.
    gpio_set_function(PIN_PWM, GPIO_FUNC_PWM);
    pwm_slice_num = pwm_gpio_to_slice_num(PIN_PWM);

    uint32_t sysclk = clock_get_hz(clk_sys);
    // PR2-style: top = sysclk / freq - 1 (no divider, no fractional).
    // For 90 kHz at 133 MHz -> top ~ 1477. Wrap with a divider of 1.
    pwm_config cfg = pwm_get_default_config();
    // 8.8 fixed divider; pick div=1 and let CC range over the full top.
    pwm_config_set_clkdiv(&cfg, 1.0f);
    pwm_top = (sysclk + freq_hz/2) / freq_hz - 1;
    if (pwm_top > 65535) {
        // fall back to a divider if needed
        pwm_config_set_clkdiv(&cfg, 2.0f);
        pwm_top = (sysclk/2 + freq_hz/2) / freq_hz - 1;
    }
    pwm_config_set_wrap(&cfg, pwm_top);
    pwm_init(pwm_slice_num, &cfg, true);
    pwm_running = true;
}

void hal_pwm_set_duty_8(uint8_t duty) {
    if (!pwm_running) return;
    // Map 8-bit duty onto [0..pwm_top]
    uint32_t cc = ((uint32_t)duty * pwm_top + 127) / 255;
    pwm_set_chan_level(pwm_slice_num, PWM_CHAN_A, cc);
}

void hal_pwm_disable(void) {
    if (!pwm_running) return;
    pwm_set_enabled(pwm_slice_num, false);
    pwm_running = false;
    // Revert the pin to a low output so the DCDC gate is held off.
    gpio_set_function(PIN_PWM, GPIO_FUNC_SIO);
    gpio_put(PIN_PWM, 0);
    gpio_set_dir(PIN_PWM, GPIO_OUT);
}

// ---- DCDC control-loop timer callback -----------------------------------
//
// The original PIC firmware runs timer_isr() at ~4 kHz (every 250 us via
// CCP2 compare). The ISR reads the ADC, computes the PID-like correction
// (pwm += errz*225; pwm -= err*228) and updates CCPR1L. The control loop is
// implemented in opcontrol.c::timer_isr(); we just provide the periodic
// trigger here. The alarm callback runs in IRQ context.
static struct repeating_timer *dcdc_timer = NULL;

void hal_dcdc_loop_start(uint32_t period_us) {
    if (dcdc_timer != NULL) return;
    // Allocate a repeating_timer_t on the heap; it must live as long as the
    // alarm is armed (the SDK stores the pointer we pass in).
    dcdc_timer = (struct repeating_timer *)calloc(1, sizeof(struct repeating_timer));
    if (!dcdc_timer) return;
    // add_repeating_timer_us returns false if no alarm slot is available.
    if (!add_repeating_timer_us(period_us, hal_dcdc_tick, NULL, dcdc_timer)) {
        free(dcdc_timer);
        dcdc_timer = NULL;
    }
}

void hal_dcdc_loop_stop(void) {
    if (!dcdc_timer) return;
    cancel_repeating_timer(dcdc_timer);
    free(dcdc_timer);
    dcdc_timer = NULL;
}

// ---- HAL init ------------------------------------------------------------
void hal_init(void) {
    // Zero the SFR shim namespace.
    memset(&PIR1bits,    0, sizeof(PIR1bits));
    memset(&PIR2bits,    0, sizeof(PIR2bits));
    memset(&PIE1bits,    0, sizeof(PIE1bits));
    memset(&INTCONbits,  0, sizeof(INTCONbits));
    memset(&HLVDCONbits, 0, sizeof(HLVDCONbits));
    HLVDCON = 0;
    hlvdif_flag = false;

    // Initialise all signal GPIOs as inputs (safe default); the firmware's
    // UserInit() will reconfigure them as needed. RP2040 pads power up in
    // SLOW mode; explicitly set every digital signal pin to FAST (see the
    // rationale block in hal.h). Series resistors (33-100 ohm on signal
    // lines, 1k+ on HV/ADC) handle ringing/EMI and protection, so SLOW is
    // not needed and would only erode timing margin on the faster clocks
    // (PIC24 CKpulseL ~750 ns, JTAG/SPI up to 10 MHz).
    const uint8_t inputs[] = { PIN_D, PIN_CK, PIN_PGM, PIN_SDA, PIN_SCL,
                               PIN_A0, PIN_A1, PIN_SW1, PIN_VPP, PIN_VCC,
                               PIN_WP, PIN_SDO };
    for (size_t i = 0; i < sizeof(inputs)/sizeof(inputs[0]); i++) {
        gpio_init(inputs[i]);
        gpio_set_dir(inputs[i], GPIO_IN);
        gpio_pull_up(inputs[i]);
        gpio_set_slew_rate(inputs[i], GPIO_SLEW_RATE_FAST);
    }
    // LEDs are outputs, off.
    gpio_init(PIN_LED1); gpio_set_dir(PIN_LED1, GPIO_OUT); gpio_put(PIN_LED1, 0);
    gpio_init(PIN_LED2); gpio_set_dir(PIN_LED2, GPIO_OUT); gpio_put(PIN_LED2, 0);
    // PWM pin starts as low output (DCDC off).
    gpio_init(PIN_PWM);  gpio_set_dir(PIN_PWM, GPIO_OUT);  gpio_put(PIN_PWM, 0);
    // Fast edges on the outputs too (uniform edge profile; the DCDC gate
    // driver, not the RP2040 pad, sets the actual MOSFET edge rate).
    gpio_set_slew_rate(PIN_LED1, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_LED2, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_PWM,  GPIO_SLEW_RATE_FAST);
    // Mode-select button is active-low with internal pull-up.
    gpio_init(PIN_SW1);  gpio_set_dir(PIN_SW1, GPIO_IN);   gpio_pull_up(PIN_SW1);
    // Vreg feedback ADC.
    hal_adc_init();
}

// Forward decl: opcontrol.c::timer_isr is the original DCDC ISR, reused
// verbatim (its body does not depend on PIC SFRs beyond PIR1bits.ADIF
// which is part of our shim namespace).
void timer_isr(void);

bool hal_dcdc_tick(struct repeating_timer *rt) {
    (void)rt;
    // The original timer_isr clears ADIF, reads ADRES, updates the PID and
    // writes CCPR1L. We call it directly from this alarm callback.
    timer_isr();
    // Return true to keep the repeating timer armed (matching the original
    // periodic CCP2 interrupt behaviour).
    return true;
}
