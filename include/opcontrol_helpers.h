/*
 * opcontrol_helpers.h — Common bit-shift helpers for the Open Programmer
 * Pico port.
 *
 * The original PIC18 OPcontrol.c expresses every ICSP / PIC24 / JTAG bit
 * shift as a hand-tuned inline-assembly block that rotates a 16/24/32-bit
 * value through the PIC's STATUS.CARRY flag, toggling D and CK on each
 * iteration. On RP2040 (a 32-bit ARM Cortex-M0+) the same logic compiles
 * to a handful of single-cycle shift/branch instructions in plain C, so we
 * replace every _asm block with one of the helpers below.
 *
 * Naming convention mirrors the original:
 *   shift_out_msb_first : emit n bits of `v`, MSB first, D line toggles CK
 *   shift_in_msb_first  : read n bits from D, MSB first, CK toggles D
 *   shift_out_lsb_first : emit n bits LSB first (used by JTAG / microwire)
 *   shift_in_lsb_first  : read n bits LSB first (JTAG TDO read-back)
 */
#ifndef OPCONTROL_HELPERS_H
#define OPCONTROL_HELPERS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ICSP data direction helpers (mirror of the original Ddir_bit assignments).
void icsp_d_out(void);
void icsp_d_in(void);

// Shift `nbits` of `v` out on the D line, MSB first, with a CK pulse per bit.
// `v` may be 8/16/24-bit; only the low `nbits` bits are used.
void shift_out_msb_first(uint32_t v, uint8_t nbits);

// Read `nbits` from the D line, MSB first, returning the assembled value.
uint32_t shift_in_msb_first(uint8_t nbits);

// Shift `nbits` of `v` out on the D line, LSB first.
void shift_out_lsb_first(uint32_t v, uint8_t nbits);

// Read `nbits` from the D line, LSB first.
uint32_t shift_in_lsb_first(uint8_t nbits);

// JTAG TDO read helper: shift `nbits` of `v` out on TDI while reading TDO
// simultaneously (LSB first). Returns the 32-bit TDO value.
uint32_t jtag_xfer(uint32_t v, uint8_t nbits);

// PIC24 SIX: shift 24 bits out MSB first using the long CK pulse width.
void     six_shift24(uint32_t v);

// PIC24 REGOUT: read 16 bits MSB first using the long CK pulse width.
uint16_t regout_read16(void);

// ICSP8: shift 8 bits out MSB first using the normal CK pulse width.
void     icsp8_shift8(uint8_t v);

// AT_HV_RTX (AVR HVSP): simultaneous 8-bit shift out on PB1/PB0 (SDI/SII)
// while reading PB2 (SDO), with clock on PB3. Returns the 8-bit SDO byte.
uint8_t  hvsp_xfer(uint8_t sdi, uint8_t sii);

// Microwire: shift `nbits` of `v` out on uWDO (RC7) MSB first, clock on uWCK (RB1).
void     uw_shift_out(uint32_t v, uint8_t nbits);
// Microwire: read `nbits` from uWDI (RB0) MSB first, clock on uWCK (RB1).
uint32_t uw_shift_in(uint8_t nbits);

// TX16 / RX16: 16-bit shifts with a T1-calibrated half-bit delay between
// the rising and falling CK edges (long clock period variant).
void     tx16_shift(uint16_t v, uint8_t t1);
uint16_t rx16_shift(uint8_t t1);

#ifdef __cplusplus
}
#endif

#endif // OPCONTROL_HELPERS_H
