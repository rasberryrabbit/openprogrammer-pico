/*
 * usb_app.h — TinyUSB HID glue for the Open Programmer Pico port.
 *
 * The original usb.c/usb.h on PIC18 manages HID OUT (host->device) and
 * HID IN (device->host) buffers with a hand-rolled SIE driver. On RP2040
 * we use TinyUSB but keep the same buffer interface so OPcontrol.c can be
 * ported with minimal changes:
 *
 *   uint8_t HIDRxReport(uint8_t *buf, uint8_t len)  -> read OUT report
 *   uint8_t HIDTxReport(uint8_t *buf, uint8_t len)  -> send IN report
 *
 * Plus an "ep1Bi.Stat & UOWN" check that the original uses to know whether
 * the previous IN transfer is still in flight. We expose a flag
 * (hid_tx_busy()) that returns true while the previous IN is pending.
 */
#ifndef USB_APP_H
#define USB_APP_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HID_INPUT_REPORT_BYTES   64
#define HID_OUTPUT_REPORT_BYTES  64

void    usb_app_init(void);
uint8_t HIDRxReport(uint8_t *buf, uint8_t len);   // returns #bytes read (0 if none)
uint8_t HIDTxReport(uint8_t *buf, uint8_t len);   // returns len on success
bool    hid_tx_busy(void);                         // true if previous IN still pending
bool    usb_configured(void);                      // true if host has configured the device

#ifdef __cplusplus
}
#endif

#endif // USB_APP_H
