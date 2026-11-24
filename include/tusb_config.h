/*
 * tusb_config.h — TinyUSB configuration for the Open Programmer Pico port.
 *
 * Wire-protocol compatibility with the original PIC18 firmware is mandatory:
 *   - VID 0x1209, PID 0x5432 (allocated to Open Programmer v0.10.0, June 2016)
 *   - HID interrupt endpoint, max packet 64 bytes (input + output report)
 *   - Vendor-defined usage page 0xFFA0
 *
 * The original device descriptor and report descriptor (usb.c) are mirrored
 * verbatim in usb_descriptors.c so the host-side openprog driver sees the
 * same device, same report sizes, same usages.
 */
#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

// ---- Device stack --------------------------------------------------------
#define CFG_TUSB_RHPORT0_MODE   OPT_MODE_DEVICE
// The pico-sdk build sets CFG_TUSB_OS=OPT_OS_PICO on the command line; only
// define it here for non-pico builds (OPT_OS_NONE) so we don't trigger a
// redefinition warning.
#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS             OPT_OS_NONE
#endif

// Single HID interface, single configuration.
#define CFG_TUD_HID             1
#define CFG_TUD_CDC             0
#define CFG_TUD_MSC             0
#define CFG_TUD_MIDI            0
#define CFG_TUD_VENDOR          0

// Endpoint / buffer sizes. The original firmware uses 64-byte HID IN/OUT
// interrupt endpoints with a 1 ms interval.
#define CFG_TUD_ENDPOINT0_SIZE  8
#define CFG_TUD_HID_EP_BUFSIZE  64
#define CFG_TUD_HID_EP_INTERVAL 1

// Task mode: the main loop calls tud_task() explicitly (the original firmware
// is a single-threaded super-loop). No RTOS, no interrupt-driven task.
#define CFG_TUSB_DEBUG          0

#ifdef __cplusplus
}
#endif

#endif // TUSB_CONFIG_H
