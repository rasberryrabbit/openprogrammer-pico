/*
 * usb_app.c — TinyUSB HID glue for the Open Programmer Pico port.
 *
 * Replaces the original PIC18 usb.c (hand-rolled SIE driver). Exposes the
 * same HIDRxReport / HIDTxReport interface that OPcontrol.c uses, so the
 * application layer is unaware that the USB transport has changed.
 *
 * The original firmware posts a 64-byte IN endpoint report when TXptr fills
 * the transmit_buffer, and reads a 64-byte OUT endpoint report whenever
 * the host sends one. TinyUSB handles the SIE, the BDT, the data toggles
 * and the standard chapter-9 requests; we only implement:
 *
 *   - tud_hid_set_report_cb() : store the OUT report into a ring buffer
 *                                (so OPcontrol.c can fetch it via HIDRxReport)
 *   - tud_hid_get_report_cb() : not used (no Feature reports)
 *   - HIDTxReport()           : try to send an IN report, return 0 if busy
 *   - HIDRxReport()           : copy one OUT report out of the ring buffer
 */
#include "tusb.h"
#include "usb_app.h"
#include <string.h>

#define OUT_RING_SIZE 4   // queue up to 4 OUT reports (4 * 64 = 256 bytes)

typedef struct {
    uint8_t  data[HID_OUTPUT_REPORT_BYTES];
    uint8_t  len;
} out_entry_t;

static out_entry_t out_ring[OUT_RING_SIZE];
static volatile uint8_t out_head = 0;   // producer (set_report callback)
static volatile uint8_t out_tail = 0;   // consumer (HIDRxReport)

// Single-shot IN buffer. The original OPcontrol.c reuses transmit_buffer[]
// in place and calls HIDTxReport(transmit_buffer, 64). We copy into this
// staging buffer so the caller can immediately reuse transmit_buffer.
static uint8_t  in_stage[HID_INPUT_REPORT_BYTES];
static volatile bool in_flight = false;

void usb_app_init(void) {
    out_head = out_tail = 0;
    in_flight = false;
    memset(out_ring, 0, sizeof(out_ring));
}

bool usb_configured(void) {
    return tud_ready();
}

bool hid_tx_busy(void) {
    return in_flight || !tud_hid_ready();
}

uint8_t HIDTxReport(uint8_t *buf, uint8_t len) {
    if (len > HID_INPUT_REPORT_BYTES) len = HID_INPUT_REPORT_BYTES;
    if (in_flight || !tud_hid_ready()) return 0;
    memcpy(in_stage, buf, len);
    if (len < HID_INPUT_REPORT_BYTES) {
        // The original firmware always sends full 64-byte reports.
        memset(in_stage + len, 0, HID_INPUT_REPORT_BYTES - len);
    }
    // tud_hid_report returns true if the report was accepted.
    if (tud_hid_report(0, in_stage, HID_INPUT_REPORT_BYTES)) {
        in_flight = false; // TinyUSB will complete the transfer asynchronously
        return len;
    }
    return 0;
}

uint8_t HIDRxReport(uint8_t *buf, uint8_t len) {
    if (len > HID_OUTPUT_REPORT_BYTES) len = HID_OUTPUT_REPORT_BYTES;
    if (out_head == out_tail) return 0;   // ring empty
    out_entry_t *e = &out_ring[out_tail];
    uint8_t n = (e->len < len) ? e->len : len;
    memcpy(buf, e->data, n);
    if (n < len) memset(buf + n, 0, len - n);
    out_tail = (out_tail + 1) % OUT_RING_SIZE;
    return n;
}

// ---- TinyUSB HID callbacks ----------------------------------------------

// Host sent a SET_REPORT (control) or an OUT interrupt report. We only
// expect the interrupt OUT path (report_type == HID_REPORT_TYPE_OUTPUT).
// Push the payload into the ring buffer; drop if full (matching the
// original firmware's behaviour where the host resends if the device is
// slow to consume the report).
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type,
                           uint8_t const *buffer, uint16_t bufsize) {
    (void)instance; (void)report_id;
    if (report_type != HID_REPORT_TYPE_OUTPUT) return;
    if (bufsize == 0) return;

    uint8_t next = (out_head + 1) % OUT_RING_SIZE;
    if (next == out_tail) return;  // ring full, drop the report
    out_entry_t *e = &out_ring[out_head];
    uint16_t n = (bufsize < HID_OUTPUT_REPORT_BYTES) ? bufsize
                                                     : HID_OUTPUT_REPORT_BYTES;
    memcpy(e->data, buffer, n);
    if (n < HID_OUTPUT_REPORT_BYTES) memset(e->data + n, 0, HID_OUTPUT_REPORT_BYTES - n);
    e->len = HID_OUTPUT_REPORT_BYTES;
    out_head = next;
}

// Host issued GET_REPORT (control). Return the current IN staging buffer;
// the original firmware does not use this path but TinyUSB requires the cb.
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type,
                               uint8_t *buffer, uint16_t reqlen) {
    (void)instance; (void)report_id; (void)report_type;
    uint16_t n = (reqlen < HID_INPUT_REPORT_BYTES) ? reqlen
                                                   : HID_INPUT_REPORT_BYTES;
    memcpy(buffer, in_stage, n);
    return n;
}

// The original firmware responds to standard HID class requests
// (GET_REPORT/SET_REPORT/SET_IDLE/SET_PROTOCOL) implicitly through its
// hand-rolled chapter-9 code. TinyUSB handles all of these in its HID
// class driver; no action needed here.
