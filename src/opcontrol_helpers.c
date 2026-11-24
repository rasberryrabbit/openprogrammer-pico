/*
 * opcontrol_helpers.c — Bit-shift helpers for the Open Programmer Pico port.
 *
 * Each function replaces a hand-tuned PIC18 inline-assembly block. The C
 * below compiles to a small loop of single-cycle ARM instructions at 133 MHz,
 * giving similar or faster timing than the original PIC18 at 48 MHz.
 */
#include "opcontrol_helpers.h"
#include "hal.h"

void icsp_d_out(void) { Dout(); }
void icsp_d_in(void)  { Din();  }

void shift_out_msb_first(uint32_t v, uint8_t nbits) {
    // Equivalent to the original PIC18 _asm block:
    //   for (i = nbits; i; i--) {
    //     RRCF d+1; RRCF d;          // rotate right through carry (MSB out)
    //     BCF LATB, Dnum;             // D0
    //     BTFSC STATUS, C; BSF LATB, Dnum;  // D1 if carry
    //     BSF LATB, CKnum; nop; BCF LATB, CKnum;  // CK pulse
    //   }
    // On ARM we just shift `v` left and test the top bit each iteration.
    // (The PIC's RRCF chain shifts LSB-first into CARRY, which sends MSB
    // first; we replicate the MSB-first behaviour with a left shift.)
    while (nbits--) {
        gpio_put(PIN_D, (v >> nbits) & 1);
        // CK pulse (normal width)
        gpio_put(PIN_CK, 1);
        __compiler_memory_barrier();
        gpio_put(PIN_CK, 0);
        __compiler_memory_barrier();
    }
}

uint32_t shift_in_msb_first(uint8_t nbits) {
    // Original PIC18 _asm:
    //   for (i = nbits; i; i--) {
    //     BSF LATB, CKnum; nop; BCF LATB, CKnum;  // CK pulse (rising then falling)
    //     BCF STATUS, C;
    //     BTFSC PORTB, Dnum; BSF STATUS, C;
    //     RRCF d+1; RRCF d;       // shift C into MSB of d
    //   }
    // i.e. on each CK pulse, sample D into the top of the accumulator.
    uint32_t v = 0;
    while (nbits--) {
        gpio_put(PIN_CK, 1);
        __compiler_memory_barrier();
        gpio_put(PIN_CK, 0);
        __compiler_memory_barrier();
        v = (v << 1) | (gpio_get(PIN_D) ? 1 : 0);
    }
    return v;
}

void shift_out_lsb_first(uint32_t v, uint8_t nbits) {
    // Used by JTAG TDI shift and microwire shift. Original uses RLCF which
    // shifts MSB out; the JTAG block compensates by feeding the data byte
    // reversed. We keep the LSB-first contract here and let callers reverse
    // if needed.
    for (uint8_t i = 0; i < nbits; i++) {
        gpio_put(PIN_PGM, v & 1);   // TDI = PIN_PGM in this port
        // TCK pulse
        gpio_put(PIN_CK, 1);
        __compiler_memory_barrier();
        gpio_put(PIN_CK, 0);
        __compiler_memory_barrier();
        v >>= 1;
    }
}

uint32_t shift_in_lsb_first(uint8_t nbits) {
    uint32_t v = 0;
    for (uint8_t i = 0; i < nbits; i++) {
        gpio_put(PIN_CK, 1);
        __compiler_memory_barrier();
        gpio_put(PIN_CK, 0);
        __compiler_memory_barrier();
        v |= (gpio_get(PIN_D) ? 1u : 0u) << i;   // TDO = PIN_D
    }
    return v;
}

// JTAG TDO read helper: shift `nbits` of `v` out on TDI while reading TDO
// simultaneously (LSB first). Returns the 32-bit TDO value.
// The ported opcontrol.c inlines the JTAG transfer (to honour the exact
// TMS walk sequence per command); this helper is kept as part of the API
// for future use.
__attribute__((unused))
uint32_t jtag_xfer(uint32_t v, uint8_t nbits) {
    // Shift `nbits` out on TDI while reading TDO simultaneously (LSB first),
    // matching the original JTAG_XFER_DATA block. The TMS line is held low
    // during the shift by the caller.
    uint32_t tdo = 0;
    for (uint8_t i = 0; i < nbits; i++) {
        gpio_put(PIN_PGM, v & 1);             // TDI = bit
        gpio_put(PIN_CK, 1);
        __compiler_memory_barrier();
        // Sample TDO on the rising edge.
        if (gpio_get(PIN_D)) tdo |= (1u << i);
        gpio_put(PIN_CK, 0);
        __compiler_memory_barrier();
        v >>= 1;
    }
    return tdo;
}

// ---- PIC24 SIX / REGOUT (long CK pulse width) ---------------------------
// The PIC24 ICSP protocol uses a longer clock period than PIC18. The
// original CKpulseL() macro produces ~750 ns pulses; we approximate with
// busy_wait_at_least_cycles(16). SIX shifts 24 bits MSB first; REGOUT reads
// 16 bits MSB first. These run inside a critical section and use the
// RAM-resident exact cycle delay so the PIC24 long-clock half-period is
// deterministic (the original PIC18 _asm block was cycle-calibrated too).
__attribute__((section(".time_critical." "six_shift24")))
void six_shift24(uint32_t v) {
    for (int8_t b = 23; b >= 0; b--) {
        gpio_put(PIN_D, (v >> b) & 1);
        gpio_put(PIN_CK, 1);
        hal_delay_cycles(16);
        gpio_put(PIN_CK, 0);
        hal_delay_cycles(16);
    }
}

__attribute__((section(".time_critical." "regout_read16")))
uint16_t regout_read16(void) {
    uint16_t v = 0;
    for (uint8_t b = 0; b < 16; b++) {
        gpio_put(PIN_CK, 1);
        hal_delay_cycles(16);
        v = (v << 1) | (gpio_get(PIN_D) ? 1 : 0);
        gpio_put(PIN_CK, 0);
        hal_delay_cycles(16);
    }
    return v;
}

// ---- ICSP8 (8-bit PIC, e.g. PIC16F1xxx) ----------------------------------
__attribute__((section(".time_critical." "icsp8_shift8")))
void icsp8_shift8(uint8_t v) {
    // MSB first, normal CK pulse width. Matches the original ciclo_300.
    for (int8_t b = 7; b >= 0; b--) {
        gpio_put(PIN_D, (v >> b) & 1);
        gpio_put(PIN_CK, 1);
        __compiler_memory_barrier();
        gpio_put(PIN_CK, 0);
        __compiler_memory_barrier();
    }
}

// ---- AT_HV_RTX (AVR high-voltage serial programming) --------------------
// Simultaneous 8-bit shift: drive SDI on PB1 and SII on PB0, read SDO on
// PB2, clock on PB3. Matches the original ciclo_r200 block. The caller
// pre-configures the four pins as outputs (PB1/PB0/PB3) and input (PB2).
// RAM-resident + exact cycle delays so the SCK half-period matches the
// AVR HVSP timing (~2 us min); the original PIC18 _asm was cycle-calibrated.
__attribute__((section(".time_critical." "hvsp_xfer")))
uint8_t hvsp_xfer(uint8_t sdi, uint8_t sii) {
    uint8_t sdo = 0;
    // Leading clock pulse (matches the original BSF/BCF PB3 before the loop).
    gpio_put(PB3, 1);
    hal_delay_cycles(4);
    gpio_put(PB3, 0);
    for (int8_t b = 7; b >= 0; b--) {
        // Read SDO bit (PB2) on the rising clock edge, before driving new data.
        // (Original samples PORTB,PB2 into carry then RLCF i2; we read first.)
        uint8_t bit = gpio_get(PB2) ? 1 : 0;
        sdo = (sdo << 1) | bit;
        // Drive SDI (PB1) and SII (PB0) from the MSB of each input byte.
        gpio_put(PB1, (sdi >> b) & 1);
        gpio_put(PB0, (sii >> b) & 1);
        // Clock pulse.
        gpio_put(PB3, 1);
        hal_delay_cycles(4);
        gpio_put(PB3, 0);
    }
    // Two trailing clock pulses (matches the original post-loop sequence).
    gpio_put(PB3, 1); hal_delay_cycles(4); gpio_put(PB3, 0);
    hal_delay_cycles(4);
    gpio_put(PB3, 1); hal_delay_cycles(4); gpio_put(PB3, 0);
    // Return D lines low.
    gpio_put(PB1, 0);
    gpio_put(PB0, 0);
    return sdo;
}

// ---- Microwire (uWCK = RB1, uWDI = RB0, uWDO = RC7) ----------------------
// These two helpers are part of the public helper API but the ported
// opcontrol.c inlines the microwire bit-bang directly (to match the
// original byte-packing logic more closely). Mark them unused-aware so the
// compiler does not warn.
__attribute__((unused, section(".time_critical." "uw_shift_out")))
void uw_shift_out(uint32_t v, uint8_t nbits) {
    // MSB first. Matches the original uWTX _asm block: RLCF d, carry -> uWDO,
    // then CK pulse on uWCK.
    for (int8_t b = nbits - 1; b >= 0; b--) {
        gpio_put(PIN_SDO, (v >> b) & 1);   // uWDO = RC7
        gpio_put(PIN_SCL, 1);              // uWCK = RB1
        hal_delay_cycles(8);
        gpio_put(PIN_SCL, 0);
        hal_delay_cycles(8);
    }
}

__attribute__((unused, section(".time_critical." "uw_shift_in")))
uint32_t uw_shift_in(uint8_t nbits) {
    uint32_t v = 0;
    for (int8_t b = nbits - 1; b >= 0; b--) {
        gpio_put(PIN_SCL, 1);              // uWCK = RB1
        hal_delay_cycles(8);
        gpio_put(PIN_SCL, 0);
        hal_delay_cycles(8);
        v = (v << 1) | (gpio_get(PIN_SDA) ? 1 : 0);  // uWDI = RB0
    }
    return v;
}

// ---- TX16 / RX16 (PIC24 long-clock 16-bit shifts) -----------------------
// The original uses a T1-calibrated half-bit delay between CK rising and
// falling edges. We use the RAM-resident exact microsecond delay so the
// long-clock half-period is jitter-free (the original PIC18 T1-calibrated
// NOP loop was exact too).
__attribute__((section(".time_critical." "tx16_shift")))
void tx16_shift(uint16_t v, uint8_t t1) {
    for (int8_t b = 15; b >= 0; b--) {
        gpio_put(PIN_D, (v >> b) & 1);
        gpio_put(PIN_CK, 1);
        hal_delay_us_exact(t1);
        gpio_put(PIN_CK, 0);
        hal_delay_us_exact(t1);
    }
}

__attribute__((section(".time_critical." "rx16_shift")))
uint16_t rx16_shift(uint8_t t1) {
    uint16_t v = 0;
    for (uint8_t b = 0; b < 16; b++) {
        gpio_put(PIN_CK, 1);
        hal_delay_us_exact(t1);
        v = (v << 1) | (gpio_get(PIN_D) ? 1 : 0);
        gpio_put(PIN_CK, 0);
        hal_delay_us_exact(t1);
    }
    return v;
}
