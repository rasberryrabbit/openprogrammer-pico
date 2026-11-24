/*
 * main.c — Open Programmer firmware, RP2040 (Raspberry Pi Pico) port.
 *
 * Ported from the original PIC18 main.c (Alberto Maccioni, GPL2). The boot
 * sequence mirrors the original:
 *
 *   1. UserInit()              — configure I/O, ADC, timers, default params
 *   2. Initialise USB (TinyUSB)
 *   3. super-loop:
 *        tud_task()            — drive USB enumeration and transactions
 *        ProcessIO()           — application layer (in OPcontrol.c)
 *
 * There is no separate USB thread; TinyUSB's device task is polled from the
 * main loop (OPT_OS_NONE). The DCDC control loop runs from a repeating
 * timer alarm in IRQ context (see hal.c::hal_dcdc_tick).
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "tusb.h"

#include "hal.h"
#include "OPcontrol.h"
#include "usb_app.h"
#include "pio_drivers.h"

// On the pico-sdk + TinyUSB device port, board_init() is provided by the
// tinyusb_board library (which wraps pico_stdlib's stdio_init_all and clock
// setup). The BSP header is exposed under "bsp/board_api.h" in newer
// TinyUSB checkouts; declare the prototype here so we don't depend on the
// BSP header path (which differs across SDK versions).
extern void board_init(void);

int main(void) {
    // Board init: 133 MHz default system clock, USB PLL pulls 48 MHz from
    // the crystal. board_init() configures clocks and the USB peripheral.
    board_init();
    hal_init();
    // Cache the system clock frequency for the RAM-resident precision
    // delay primitives (hal_delay_us_exact / hal_delay_ns_exact). Must
    // run AFTER board_init() so the clock is at its final 133 MHz.
    hal_timing_init();
    // Load the PIO programs (One-Wire, JTAG, UNI/O, Microwire) into the
    // RP2040 PIO blocks and configure the state machines. Done at boot so
    // the SMs are ready when the first protocol command arrives.
    pio_drivers_init();
    usb_app_init();
    // BOARD_TUD_RHPORT is 0 on RP2040 (single USB port). Use the literal
    // since the BSP header is not included on the pico-sdk build.
    tud_init(0);

    // Call user init (mirrors main.c::UserInit on PIC18).
    UserInit();

    while (1) {
        // Drive the TinyUSB device stack.
        tud_task();
        // Application work (USB HID RX/TX + command parser).
        ProcessIO();
    }

    return 0;
}

// ---- USB device event hooks (TinyUSB) ------------------------------------
// We don't need to take action on these; the application polls tud_task().
void tud_mount_cb(void)           {}
void tud_umount_cb(void)          {}
void tud_suspend_cb(bool remote_wakeup_en) { (void)remote_wakeup_en; }
void tud_resume_cb(void)          {}
