/*
 * usb_descriptors.c — USB descriptors for the Open Programmer Pico port.
 *
 * Verbatim copy of the descriptor block in the original usb.c (PIC18 firmware
 * v0.12.0) so the host-side openprog driver enumerates the device without
 * any host-side change. Same VID/PID (0x1209 / 0x5432), same HID report
 * layout (64-byte IN + OUT, vendor usage page 0xFFA0).
 *
 * The descriptor byte values match the PIC firmware byte-for-byte; only the
 * TinyUSB descriptor callback wiring is new.
 */
#include "tusb.h"

// ---- HID report sizes (mirror of usb.h in the original firmware) ---------
#define HID_INPUT_REPORT_BYTES   64
#define HID_OUTPUT_REPORT_BYTES  64

// Vendor USB identifiers — same as the PIC firmware since v0.10.0.
#define USBD_VID  0x1209
#define USBD_PID  0x5432

// TinyUSB's string_desc_arr is char const*[]; the descriptors are encoded
// as UTF-16 on the wire by the stack, so we provide plain C strings here.
#define USBD_MANUFACTURER_STRING "Open Programmer"
#define USBD_PRODUCT_STRING      "Open Programmer USB HID class"
#define USBD_SERIAL_STRING       "0"

enum {
    ITF_NUM_HID,
    ITF_NUM_TOTAL
};

enum {
    STRIDX_LANGID    = 0,
    STRIDX_MANUFACTURER,
    STRIDX_PRODUCT,
    STRIDX_SERIAL,
};

// Report descriptor — same 34-byte block as the original HIDReport[] in usb.c.
// Vendor-defined usage page 0xFFA0, application collection with one 64-byte
// input report and one 64-byte output report.
static uint8_t const desc_hid_report[] = {
    0x06, 0xa0, 0xff,  // Usage Page (Vendor Defined Page 1)
    0x09, 0x01,        // Usage (Vendor Usage 1)
    0xa1, 0x01,        // Collection (Application)
    // Input report
    0x09, 0x03,        //   Usage ID - vendor defined
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xff, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8 bits)
    0x95, 0x40,        //   Report Count (64 fields)
    0x81, 0x02,        //   Input (Data, Variable, Absolute)
    // Output report
    0x09, 0x04,        //   Usage ID - vendor defined
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xff, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8 bits)
    0x95, 0x40,        //   Report Count (64 fields)
    0x91, 0x02,        //   Output (Data, Variable, Absolute)
    0xc0               // End Collection
};

// Configuration descriptor — same layout as configDescriptor in usb.c:
//   9-byte config + 9-byte interface + 9-byte HID + 7-byte EP1-IN + 7-byte EP1-OUT
//   = 0x29 = 41 bytes total.
#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN)
#define EPNUM_HID_OUT     0x01
#define EPNUM_HID_IN      0x81

static uint8_t const desc_configuration[] = {
    // Config descriptor, interface descriptor, HID descriptor, two endpoints.
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP,
                          50),  // 100 mA (original used 0x32 = 50 -> 100mA unit)
    TUD_HID_INOUT_DESCRIPTOR(ITF_NUM_HID, 0, HID_ITF_PROTOCOL_NONE,
                             sizeof(desc_hid_report),
                             EPNUM_HID_OUT, EPNUM_HID_IN,
                             HID_OUTPUT_REPORT_BYTES,
                             1)  // 1 ms interval (original)
};

// String descriptor array. TinyUSB indexes by STRIDX_*; the 0th entry is
// the LANGID. Product/manufacturer strings mirror the PIC firmware
// stringDescriptor1 / stringDescriptor2.
static char const *string_desc_arr[] = {
    (const char[]) {0x09, 0x04}, // 0: English
    USBD_MANUFACTURER_STRING,    // 1: "Open Programmer"
    USBD_PRODUCT_STRING,         // 2: "Open Programmer USB HID class"
    USBD_SERIAL_STRING,          // 3: "0"
};

// ---- TinyUSB callbacks ----------------------------------------------------

uint8_t const *tud_descriptor_device_cb(void) {
    // Same 18-byte device descriptor as deviceDescriptor[] in usb.c:
    //   bcdUSB 2.00, class/subclass/proto 0/0/0 (defined at interface level),
    //   max packet size 8, VID 0x1209, PID 0x5432, bcdDevice 0.0001,
    //   iManufacturer 1, iProduct 2, iSerialNumber 0 (none), 1 configuration.
    static tusb_desc_device_t const desc_device = {
        .bLength            = sizeof(tusb_desc_device_t),
        .bDescriptorType    = TUSB_DESC_DEVICE,
        .bcdUSB             = 0x0200,
        .bDeviceClass       = 0x00,
        .bDeviceSubClass    = 0x00,
        .bDeviceProtocol    = 0x00,
        .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
        .idVendor           = USBD_VID,
        .idProduct          = USBD_PID,
        .bcdDevice          = 0x0001,
        .iManufacturer      = STRIDX_MANUFACTURER,
        .iProduct           = STRIDX_PRODUCT,
        .iSerialNumber      = STRIDX_SERIAL,
        .bNumConfigurations = 0x01
    };
    return (uint8_t const *) &desc_device;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void) index;
    return desc_configuration;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void) langid;
    static uint16_t _desc_str[32 + 1]; // up to 32 UTF-16 code units + length
    uint8_t chr_count;

    switch (index) {
        case STRIDX_LANGID:
            _desc_str[1] = 0x0409; // English
            chr_count = 1;
            break;
        default:
            if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) {
                return NULL;
            }
            chr_count = strlen(string_desc_arr[index]);
            if (chr_count > 31) chr_count = 31;
            for (uint8_t i = 0; i < chr_count; i++) {
                _desc_str[1 + i] = string_desc_arr[index][i];
            }
            break;
    }

    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);
    return _desc_str;
}

// HID report descriptor callback — TinyUSB calls this on GET_DESCRIPTOR
// for the report type.
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
    (void) instance;
    return desc_hid_report;
}
