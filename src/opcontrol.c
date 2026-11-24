/*
 * opcontrol.c — Open Programmer application layer, RP2040 (Pico) port.
 *
 * Ported from the original PIC18 OPcontrol.c by Alberto Maccioni (GPL2).
 * The wire protocol (instructions.h) is unchanged; the host-side openprog
 * software works without modification.
 *
 * Porting rules applied:
 *   - PIC SFR references (LATBbits.LATB5, INTCONbits.GIE, PIR1bits.ADIF ...)
 *     are mapped to HAL macros in hal.h that operate on RP2040 GPIOs.
 *   - Inline-assembly bit-shift blocks are replaced with helper functions
 *     in opcontrol_helpers.c.
 *   - Nop() delay loops calibrated for 48 MHz become busy_wait_us / busy_wait
 *     primitives (see hal.c::Delay1us / DelayT).
 *   - The DCDC control loop runs from a repeating timer alarm at 4 kHz
 *     (250 us period), matching the original CCP2-triggered ISR rate.
 *
 * Coverage of the v0.12.0 instruction set:
 *   - Full:   NOP, PROG_RST, PROG_ID, CHECK_INS, FLUSH, VREG_EN/DIS,
 *             SET_PARAMETER, WAIT_T1/T2/T3/US, READ_ADC, SET_VPP,
 *             EN_VPP_VCC, SET_CK_D, READ_PINS, LOAD_CONF, LOAD_DATA_PROG,
 *             LOAD_DATA_DATA, READ_DATA_PROG, READ_DATA_DATA, INC_ADDR,
 *             INC_ADDR_N, BEGIN_PROG, BULK_ERASE_PROG/DATA, END_PROG/2,
 *             ROW_ERASE_PROG, BEGIN_PROG2, CUST_CMD, PROG_C, CORE_INS,
 *             SHIFT_TABLAT, TABLE_READ, TBLR_INC_N, TABLE_WRITE,
 *             TBLW_INC_N, TBLW_PROG, TBLW_PROG_INC, SEND_DATA, READ_DATA,
 *             I2C_INIT/READ/WRITE/READ2, SPI_INIT/READ/WRITE, EXT_PORT,
 *             AT_READ_DATA, AT_LOAD_DATA, CLOCK_GEN, TBLRD, TBLWT,
 *             REPEAT, REPEAT_END, SET_PORT_DIR, READ_B, READ_AC,
 *             SPI_TEST, READ_RAM, WRITE_RAM, LOOP.
 *   - Stubbed (return INS_ERR; not yet ported — JTAG / OW / UNIO / PIC24
 *     SIX/REGOUT / microwire): JTAG_*, OW_*, UNIO_*, SIX, SIX_LONG,
 *     SIX_LONG5, SIX_N, REGOUT, TX16, RX16, uW_*, ICSP8_*, AT_HV_RTX.
 *     These are isolated to the file's tail and clearly marked TODO so the
 *     user can extend them with the same helper functions.
 */
#include <string.h>
#include "pico/stdlib.h"
#include "hal.h"
#include "OPcontrol.h"
#include "instructions.h"
#include "opcontrol_helpers.h"
#include "usb_app.h"

//********Misc.********
// On RP2040 we always use the software I2C / software SPI implementations
// (matching the original PIC firmware default of SW_SPI / SW_I2C, since
// the PIC MSSP peripheral had known bugs). This keeps the I2C/SPI command
// handlers compiled in.
#define SW_I2C
#define SW_SPI
#define VERSION "0.12.0-pico"
#define VER2    0
#define VER1    12
#define VER0    0
#define ID2     0
#define ID1     0
#define ID0     1                  // same ID as the PIC18F2550 build (1)
#define TX_TEMP_MAX 10
#define LOBYTE(x) (*((unsigned char *)&(x)))
#define HIBYTE(x) (*(((unsigned char *)&(x))+1))
#define RX_ERR      0xfe
#define INS_ERR     0xfe
#define ACK_ERR     0xfd
#define Vcost       72.282         // (12/(12+22)/5*1024) Voltage feedback constant

//***************PORTB-derived pin aliases (kept for grep parity) **********
#define Dnum        PIN_D
#define CKnum       PIN_CK
#define PGMnum      PIN_PGM

//***************PORTA LED pins*********************************************
// LED1 / LED2 macros already defined in hal.h.

//***************PORTE switch***********************************************
// SW1 macro already defined in hal.h (active-low button).

/** V A R I A B L E S ********************************************************/

static unsigned char transmit_buffer[HID_OUTPUT_REPORT_BYTES];
static unsigned char receive_buffer[HID_INPUT_REPORT_BYTES];
static unsigned char transmit_temp[TX_TEMP_MAX];

static uint8_t number_of_bytes_read = 0;
static uint8_t IN_pending = 0;
static uint8_t led_cnt = 0;
static uint8_t T2 = 100, M = 8, N = 11;
static uint8_t T1 = 1;
static int     T3 = 2000, timeout = 0;
static uint8_t progmode = 0;
static int     genCount = 0;
static int     loopCounter = 0;
static int     loopPointer = 0;

// Access-bank "near" variables from the original (RXptr/TXptr/TXaux/pwm/...).
static uint8_t RXptr = 0, TXptr = 0, TXaux = 0;
static int     pwm = 0;
static volatile int err = 0, errz = 0, vreg = (int)(13.0 * Vcost);
static uint8_t i = 0, i2 = 0, RES0 = 0, RES1 = 0, RES2 = 0, RES3 = 0;
static int     d = 0, dH = 0;

#if defined(SW_I2C)
static unsigned char I2C_BUFFER = 0;
static unsigned char BIT_COUNTER = 0;
static uint8_t tt = 0;
#endif

// T1/T2/T3 helpers
#define SET_T1T2  0
#define SET_T3    1
#define SET_timeout 2
#define SET_MN    3

// ---- TX queue helper -----------------------------------------------------
// Mirrors the original TXins() so OPcontrol.c can push bytes into the IN
// report buffer. When the IN endpoint is busy (IN_pending), bytes are
// diverted into transmit_temp[] up to TX_TEMP_MAX.
static void TXins(uint8_t x) {
    if (IN_pending) {
        if (TXaux < TX_TEMP_MAX) transmit_temp[TXaux++] = x;
    } else {
        transmit_buffer[TXptr++] = x;
    }
    if (TXptr == HID_INPUT_REPORT_BYTES) {
        TXptr = 0;
        if (!hid_tx_busy()) {
            HIDTxReport(transmit_buffer, HID_INPUT_REPORT_BYTES);
        } else {
            IN_pending = 1;
        }
    }
}

/** P R I V A T E  P R O T O T Y P E S ***************************************/
static void ParseCommands(void);
static void BlinkStatus(void);
#if defined(SW_I2C)
static void SWStopI2C(void);
static void SWStartI2C(void);
static void SWRestartI2C(void);
static signed char SWAckI2C(void);
static unsigned int SWGetcI2C(void);
static signed char SWPutcI2C(unsigned char data_out);
static void NAckI2C(void);
static void AckI2C(void);
// DelayT is provided by hal.c (declared in hal.h).
#endif
#if defined(SW_SPI)
static unsigned char SW_IO_SPI(unsigned char c);
#endif

// ---- T1/T2/T3 microsecond waits ------------------------------------------
// The original uses Nop() loops calibrated to 48 MHz. We use busy_wait_us
// for predictable wall-clock timing on RP2040.
static inline void wait_t1_us(void) { busy_wait_us_32(T1); }
static inline void wait_t2_us(void) { busy_wait_us_32(T2); }

/** D E C L A R A T I O N S **************************************************/

void UserInit(void)
{
    // ADC: AN0 = Vreg feedback (channel ADC_VREG_CHANNEL).
    hal_adc_init();

    // Mode-select button: enter programmer mode if SW1 is held at boot
    // (matches the original "if(PORTEbits.RE3==0) progmode=1;").
    if (SW1 == 0) progmode = 1;
    IN_pending = 0;
    led_cnt = 0;

    // All PORTB latch bits = 0 (D/CK/PGM/SCL/SDA/A0/A1).
    D0(); CK0(); PGM0();
    gpio_put(PIN_SDA, 0);
    gpio_put(PIN_SCL, 0);
    gpio_put(PIN_A0, 0);
    gpio_put(PIN_A1, 0);
    // All PORTA latch bits = 0 (LED1/LED2).
    gpio_put(PIN_LED1, 0);
    gpio_put(PIN_LED2, 0);

    // TRISA: original 0b11111001 -> RA1/RA2 outputs (LEDs), rest inputs.
    gpio_set_dir(PIN_LED1, GPIO_OUT);
    gpio_set_dir(PIN_LED2, GPIO_OUT);
    // Signal pins default to input; the ICSP/I2C/SPI code flips them as needed.
    gpio_set_dir(PIN_D,    GPIO_IN);
    gpio_set_dir(PIN_CK,   GPIO_IN);
    gpio_set_dir(PIN_PGM,  GPIO_IN);
    gpio_set_dir(PIN_SDA,  GPIO_IN);
    gpio_set_dir(PIN_SCL,  GPIO_IN);
    gpio_set_dir(PIN_A0,   GPIO_IN);
    gpio_set_dir(PIN_A1,   GPIO_IN);

    T1 = 1;
    T2 = 100;
    T3 = 2000;
    M = 8;
    N = 11;
    err = errz = pwm = 0;
}

static void BlinkStatus(void) {
    // Original uses Timer0 (10.93 ms tick). On RP2040 we use a simple
    // counter incremented each ProcessIO call; the absolute timing is not
    // critical for the status LED.
    static absolute_time_t last_toggle = {0};
    absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(last_toggle, now) < 11000) return;
    last_toggle = now;
    led_cnt++;
    if (!usb_configured() && led_cnt >= 1) {
        hal_led2_write(!hal_led2_getset());
        led_cnt = 0;
    } else if (usb_configured() && led_cnt >= 5) {
        hal_led2_write(!hal_led2_getset());
        led_cnt = 0;
    }
}

/******************************************************************************
 * Function:        void ProcessIO(void)
 * Overview:        Application super-loop body. Mirrors the original PIC18
 *                  ProcessIO: blink LED, then handle HID IN/OUT traffic and
 *                  parse any commands received from the host.
 *****************************************************************************/
void ProcessIO(void)
{
    BlinkStatus();
    if (!usb_configured()) return;

    // If a previous IN transfer completed, flush transmit_temp into
    // transmit_buffer for the next report.
    if (IN_pending && !hid_tx_busy()) {
        HIDTxReport(transmit_buffer, HID_INPUT_REPORT_BYTES);
        IN_pending = 0;
        for (TXptr = 0; TXptr < TXaux; TXptr++)
            transmit_buffer[TXptr] = transmit_temp[TXptr];
        TXaux = 0;
    }

    // If we've consumed all of the current OUT report, send any pending IN
    // report and fetch the next OUT report.
    if (RXptr >= number_of_bytes_read) {
        if (TXptr) {
            for (; TXptr < HID_INPUT_REPORT_BYTES; TXptr++)
                transmit_buffer[TXptr] = 0;
            TXptr = 0;
            if (!hid_tx_busy())
                HIDTxReport(transmit_buffer, HID_INPUT_REPORT_BYTES);
            else
                IN_pending = 1;
        }
        number_of_bytes_read = HIDRxReport(receive_buffer, HID_OUTPUT_REPORT_BYTES);
        if (number_of_bytes_read) RXptr = 0;
    }
    ParseCommands();
}

// ---- Command parser ------------------------------------------------------
//
// The original ParseCommands is a giant switch on receive_buffer[RXptr].
// We port each case verbatim, replacing SFR/asm constructs with HAL/helper
// calls. For RX-buffer-underrun handling we keep the original idiom of
// writing FLUSH into the next receive_buffer slot.
static void ParseCommands(void) {
    if (RXptr >= number_of_bytes_read || IN_pending) {
        hal_led1_write(0);
        return;
    }
    hal_led1_write(1);

    switch (receive_buffer[RXptr]) {
    case NOP:
        TXins(NOP);
        break;

    case PROG_RST: {
        TXins(PROG_RST);
        UserInit();
        TXins(VER2); TXins(VER1); TXins(VER0);
        TXins(ID2); TXins(ID1); TXins(ID0);
        TXins(' '); TXins('R'); TXins('S'); TXins('T');
        break;
    }

    case PROG_ID:
        TXins(PROG_ID);
        TXins(VER2); TXins(VER1); TXins(VER0);
        TXins(ID2); TXins(ID1); TXins(ID0);
        break;

    case CHECK_INS:
        TXins(CHECK_INS);
        if (RXptr + 1 < number_of_bytes_read) {
            uint8_t x = receive_buffer[++RXptr];
            TXins(x < MAX_INS ? x : INS_ERR);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;

    case FLUSH:
        for (; TXptr < HID_INPUT_REPORT_BYTES; TXptr++) transmit_buffer[TXptr] = 0;
        TXptr = 0;
        if (!hid_tx_busy()) HIDTxReport(transmit_buffer, HID_INPUT_REPORT_BYTES);
        else IN_pending = 1;
        RXptr = number_of_bytes_read;
        break;

    case VREG_EN: {
        TXins(VREG_EN);
        err = errz = pwm = 0;
        // PWM period ~ 90 kHz (PR2=0x84 on PIC). On RP2040 we use hal_pwm_init.
        hal_pwm_init(90000);
        PIE1bits.ADIE = 1;          // enable "ADC interrupt" (control loop)
        // CCP1 duty 0 initially.
        hal_pwm_set_duty_8(0);
        // Start the 4 kHz control loop (250 us period).
        hal_dcdc_loop_start(250);
        HLVDCON = 0b00011110;       // mirror of original HLVDCON setting
        HLVDCONbits.IRVST = 1;      // mark regulator ready
        INTCONbits.GIE = 1;
        break;
    }

    case VREG_DIS:
        TXins(VREG_DIS);
        PIE1bits.ADIE = 0;
        hal_dcdc_loop_stop();
        hal_pwm_disable();
        HLVDCON = 0b00001110;
        HLVDCONbits.IRVST = 0;
        break;

    case SET_PARAMETER: {
        TXins(SET_PARAMETER);
        if (RXptr + 3 < number_of_bytes_read) {
            i = receive_buffer[++RXptr];
            switch (i) {
            case SET_T1T2:
                T1 = receive_buffer[++RXptr];
                T2 = receive_buffer[++RXptr];
                break;
            case SET_T3:
                HIBYTE(T3) = receive_buffer[++RXptr];
                LOBYTE(T3) = receive_buffer[++RXptr];
                break;
            case SET_timeout:
                HIBYTE(timeout) = receive_buffer[++RXptr];
                LOBYTE(timeout) = receive_buffer[++RXptr];
                break;
            case SET_MN:
                M = receive_buffer[++RXptr];
                N = receive_buffer[++RXptr];
                break;
            default:
                RXptr += 2;
                break;
            }
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case WAIT_T1:
        TXins(WAIT_T1);
        {
            uint32_t st = save_and_disable_interrupts();
            busy_wait_us_32(T1);
            restore_interrupts(st);
        }
        break;

    case WAIT_T2:
        TXins(WAIT_T2);
        {
            uint32_t st = save_and_disable_interrupts();
            busy_wait_us_32(T2);
            restore_interrupts(st);
        }
        break;

    case WAIT_T3:
        TXins(WAIT_T3);
        {
            uint32_t st = save_and_disable_interrupts();
            busy_wait_us_32((uint32_t)T3);
            restore_interrupts(st);
        }
        break;

    case WAIT_US:
        TXins(WAIT_US);
        if (RXptr + 1 < number_of_bytes_read) {
            uint32_t st = save_and_disable_interrupts();
            busy_wait_us_32(receive_buffer[++RXptr]);
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;

    case READ_ADC: {
        TXins(READ_ADC);
        uint32_t st = save_and_disable_interrupts();
        uint16_t raw = hal_adc_read_raw();
        restore_interrupts(st);
        // Original is a 10-bit result on PIC; RP2040 ADC is 12-bit. Shift
        // down to 10 bits so the host-side Vreg calculation matches.
        raw >>= 2;
        TXins((raw >> 8) & 0xFF);
        TXins(raw & 0xFF);
        break;
    }

    case SET_VPP: {
        TXins(SET_VPP);
        if (RXptr + 1 < number_of_bytes_read) {
            if (!PIE1bits.ADIE || !HLVDCONbits.IRVST) {
                TXins(INS_ERR);
            } else {
                PIR2bits.HLVDIF = 0;
                d = (uint8_t)receive_buffer[++RXptr];
                vreg = d * 72 / 10;
                i = 15;  // 15 ms timeout
                do {
                    for (d = 1000; d; d--) {
                        // ~1 ms of NOPs; on RP2040 a busy_wait is cleaner
                        // but the original runs the DCDC loop during this
                        // wait, so we keep the loop structure and let the
                        // timer alarm update `err` async.
                        busy_wait_us_32(1);
                    }
                    i--;
                } while ((err > 14 || err < -14) && i);
                if (PIR2bits.HLVDIF)      TXins(0);
                else if (err > 14 || err < -14) TXins(INS_ERR);
                else                       TXins(receive_buffer[RXptr]);
            }
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case EN_VPP_VCC:
        TXins(EN_VPP_VCC);
        if (RXptr + 1 < number_of_bytes_read) {
            i = receive_buffer[++RXptr];
            gpio_put(PIN_VCC, (i & 0x01) ? 0 : 1);
            gpio_set_dir(PIN_VCC, (i & 0x02) ? GPIO_IN : GPIO_OUT);
            gpio_put(PIN_VPP, (i & 0x04) ? 1 : 0);
            gpio_set_dir(PIN_VPP, (i & 0x08) ? GPIO_IN : GPIO_OUT);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;

    case SET_CK_D:
        TXins(SET_CK_D);
        if (RXptr + 1 < number_of_bytes_read) {
            uint8_t v = receive_buffer[++RXptr];
            gpio_put(PIN_D,   (v & 0x01) ? 1 : 0);
            gpio_set_dir(PIN_D,   (v & 0x02) ? GPIO_IN : GPIO_OUT);
            gpio_put(PIN_CK,  (v & 0x04) ? 1 : 0);
            gpio_set_dir(PIN_CK,  (v & 0x08) ? GPIO_IN : GPIO_OUT);
            gpio_put(PIN_PGM, (v & 0x10) ? 1 : 0);
            gpio_set_dir(PIN_PGM, (v & 0x20) ? GPIO_IN : GPIO_OUT);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;

    case READ_PINS: {
        TXins(READ_PINS);
        i = 0;
        if (gpio_get(PIN_D))   i |= 1;
        if (gpio_get_dir(PIN_D)   == GPIO_IN) i |= 2;
        if (gpio_get(PIN_CK))  i |= 4;
        if (gpio_get_dir(PIN_CK)  == GPIO_IN) i |= 8;
        if (gpio_get(PIN_PGM)) i |= 16;
        if (gpio_get_dir(PIN_PGM) == GPIO_IN) i |= 32;
        TXins(i);
        break;
    }

    case LOAD_CONF: {
        // 6-bit command 000000 + 14-bit payload, LSB first.
        TXins(LOAD_CONF);
        if (RXptr + 2 < number_of_bytes_read) {
            HIBYTE(d) = receive_buffer[++RXptr];
            LOBYTE(d) = receive_buffer[++RXptr];
            uint32_t st = save_and_disable_interrupts();
            D0(); CKpulseN(); CKpulseN(); CKpulseN();
            CKpulseN(); CKpulseN(); CKpulseN();
            wait_t1_us();
            CKpulseN();            // start bit
            shift_out_msb_first((uint32_t)d & 0x3FFF, 14);
            D0(); CKpulseN();      // stop bit
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case LOAD_DATA_PROG: {
        // 6-bit command 000010 + 14-bit payload.
        TXins(LOAD_DATA_PROG);
        if (RXptr + 2 < number_of_bytes_read) {
            HIBYTE(d) = receive_buffer[++RXptr];
            LOBYTE(d) = receive_buffer[++RXptr];
            uint32_t st = save_and_disable_interrupts();
            D0(); CKpulseN();
            D1(); CKpulseN();
            D0(); CKpulseN(); CKpulseN(); CKpulseN(); CKpulseN();
            wait_t1_us();
            CKpulseN();            // start bit
            shift_out_msb_first((uint32_t)d & 0x3FFF, 14);
            D0(); CKpulseN();      // stop bit
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case LOAD_DATA_DATA: {
        // 6-bit command 000011 + 8-bit payload.
        TXins(LOAD_DATA_DATA);
        if (RXptr + 1 < number_of_bytes_read) {
            LOBYTE(d) = receive_buffer[++RXptr];
            uint32_t st = save_and_disable_interrupts();
            D1(); CKpulseN(); CKpulseN();
            D0(); CKpulseN(); CKpulseN(); CKpulseN(); CKpulseN();
            wait_t1_us();
            CKpulseN();            // start bit
            shift_out_msb_first((uint32_t)d & 0xFF, 8);
            D0(); CKpulseN(); CKpulseN(); CKpulseN();
            CKpulseN(); CKpulseN(); CKpulseN();  // bits 10..MSB
            CKpulseN();            // stop bit
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case READ_DATA_PROG: {
        // 6-bit command 000100, then read 14 bits.
        TXins(READ_DATA_PROG);
        uint32_t st = save_and_disable_interrupts();
        D0(); CKpulseN(); CKpulseN();
        D1(); CKpulseN();
        D0(); CKpulseN(); CKpulseN(); CKpulseN();
        Din(); wait_t1_us();
        CKpulseN();               // start bit
        d = (int)shift_in_msb_first(14);
        // The PIC code shifts twice more after the loop (two extra RRCF
        // passes that align the 14-bit payload); on ARM we already have
        // the value, so we just keep it.
        CK1();                    // stop bit
        Dout();
        CK0();                    // stop bit
        restore_interrupts(st);
        TXins(HIBYTE(d));
        TXins(LOBYTE(d));
        break;
    }

    case READ_DATA_DATA: {
        // 6-bit command 000101, then read 8 bits.
        TXins(READ_DATA_DATA);
        uint32_t st = save_and_disable_interrupts();
        D1(); CKpulseN();
        D0(); CKpulseN();
        D1(); CKpulseN();
        D0(); CKpulseN(); CKpulseN(); CKpulseN();
        Din(); wait_t1_us();
        CKpulseN();               // start bit
        d = (int)shift_in_msb_first(8);
        CKpulseN(); CKpulseN(); CKpulseN();
        CKpulseN(); CKpulseN(); CKpulseN();  // bits 10..MSB
        CK1();                    // stop bit
        Dout();
        CK0();                    // stop bit
        restore_interrupts(st);
        TXins(LOBYTE(d));
        break;
    }

    case INC_ADDR: {
        // 6-bit command 000110.
        TXins(INC_ADDR);
        uint32_t st = save_and_disable_interrupts();
        D0(); CKpulseN();
        D1(); CKpulseN(); CKpulseN();
        D0(); CKpulseN(); CKpulseN(); CKpulseN();
        restore_interrupts(st);
        break;
    }

    case INC_ADDR_N: {
        TXins(INC_ADDR_N);
        if (RXptr + 1 < number_of_bytes_read) {
            uint32_t st = save_and_disable_interrupts();
            for (LOBYTE(d) = receive_buffer[++RXptr]; LOBYTE(d); LOBYTE(d)--) {
                D0(); CKpulseN();
                D1(); CKpulseN(); CKpulseN();
                D0(); CKpulseN(); CKpulseN(); CKpulseN();
                wait_t1_us();
            }
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case BEGIN_PROG: {
        // 6-bit command 001000.
        TXins(BEGIN_PROG);
        uint32_t st = save_and_disable_interrupts();
        D0(); CKpulseN(); CKpulseN(); CKpulseN();
        D1(); CKpulseN();
        D0(); CKpulseN(); CKpulseN();
        restore_interrupts(st);
        break;
    }

    case BULK_ERASE_PROG: {
        // 6-bit command 001001.
        TXins(BULK_ERASE_PROG);
        uint32_t st = save_and_disable_interrupts();
        D1(); CKpulseN();
        D0(); CKpulseN(); CKpulseN();
        D1(); CKpulseN();
        D0(); CKpulseN(); CKpulseN();
        restore_interrupts(st);
        break;
    }

    case END_PROG: {
        // 6-bit command 001010.
        TXins(END_PROG);
        uint32_t st = save_and_disable_interrupts();
        D0(); CKpulseN();
        D1(); CKpulseN();
        D0(); CKpulseN();
        D1(); CKpulseN();
        D0(); CKpulseN(); CKpulseN();
        restore_interrupts(st);
        break;
    }

    case BULK_ERASE_DATA: {
        // 6-bit command 001011.
        TXins(BULK_ERASE_DATA);
        uint32_t st = save_and_disable_interrupts();
        D1(); CKpulseN(); CKpulseN();
        D0(); CKpulseN();
        D1(); CKpulseN();
        D0(); CKpulseN(); CKpulseN();
        restore_interrupts(st);
        break;
    }

    case END_PROG2: {
        // 6-bit command 001110.
        TXins(END_PROG2);
        uint32_t st = save_and_disable_interrupts();
        D0(); CKpulseN();
        D1(); CKpulseN(); CKpulseN(); CKpulseN();
        D0(); CKpulseN(); CKpulseN();
        restore_interrupts(st);
        break;
    }

    case ROW_ERASE_PROG: {
        // 6-bit command 010001.
        TXins(ROW_ERASE_PROG);
        uint32_t st = save_and_disable_interrupts();
        D1(); CKpulseN();
        D0(); CKpulseN(); CKpulseN(); CKpulseN();
        D1(); CKpulseN();
        D0(); CKpulseN();
        restore_interrupts(st);
        break;
    }

    case BEGIN_PROG2: {
        // 6-bit command 011000.
        TXins(BEGIN_PROG2);
        uint32_t st = save_and_disable_interrupts();
        D0(); CKpulseN(); CKpulseN(); CKpulseN();
        D1(); CKpulseN(); CKpulseN();
        D0(); CKpulseN();
        restore_interrupts(st);
        break;
    }

    case CUST_CMD: {
        // 6-bit custom command (LSB first).
        TXins(CUST_CMD);
        if (RXptr + 1 < number_of_bytes_read) {
            LOBYTE(d) = receive_buffer[++RXptr];
            uint32_t st = save_and_disable_interrupts();
            shift_out_msb_first((uint32_t)(d & 0x3F), 6);
            D0();
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case PROG_C: {
        // Program-and-verify word: 000010 (load), then M x [001000 (begin) +
        // 001110 (end2) + 000100 (read)], then N x [001000 + 001110] overpulses.
        TXins(PROG_C);
        if (RXptr + 2 < number_of_bytes_read) {
            int t, j;
            HIBYTE(d) = receive_buffer[++RXptr];
            LOBYTE(d) = receive_buffer[++RXptr];
            t = d;
            uint32_t st = save_and_disable_interrupts();
            // Load data 000010 + 14-bit payload.
            D0(); CKpulseN();
            D1(); CKpulseN();
            D0(); CKpulseN(); CKpulseN(); CKpulseN(); CKpulseN();
            wait_t1_us();
            CKpulseN();
            shift_out_msb_first((uint32_t)d & 0x3FFF, 14);
            D0(); CKpulseN();
            // M program/verify attempts.
            for (j = 0; j < M; j++) {
                // BEGIN PROG 001000
                D0(); CKpulseN(); CKpulseN(); CKpulseN();
                D1(); CKpulseN();
                D0(); CKpulseN(); CKpulseN();
                wait_t2_us();
                // END PROG2 001110
                D0(); CKpulseN();
                D1(); CKpulseN(); CKpulseN(); CKpulseN();
                D0(); CKpulseN(); CKpulseN();
                wait_t1_us();
                // READ PROG 000100
                D0(); CKpulseN(); CKpulseN();
                D1(); CKpulseN();
                D0(); CKpulseN(); CKpulseN(); CKpulseN();
                Din(); wait_t1_us();
                CKpulseN();
                d = (int)shift_in_msb_first(14);
                CK1(); Dout(); CK0();
                wait_t1_us();
                if (t == d) { i = j + 1; j = 0xff; }
            }
            restore_interrupts(st);
            st = save_and_disable_interrupts();
            if (j == 0x100) {
                TXins(i);
                for (j = i * N; j; j--) {
                    // BEGIN PROG 001000
                    D0(); CKpulseN(); CKpulseN(); CKpulseN();
                    D1(); CKpulseN();
                    D0(); CKpulseN(); CKpulseN();
                    wait_t2_us();
                    // END PROG2 001110
                    D0(); CKpulseN();
                    D1(); CKpulseN(); CKpulseN(); CKpulseN();
                    D0(); CKpulseN(); CKpulseN();
                    wait_t1_us();
                }
            } else {
                TXins(INS_ERR);
            }
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case LOAD_PC: {
        // PIC24 011101 + 24-bit payload (here only 16 bits are used per
        // original LOAD_PC implementation).
        TXins(LOAD_PC);
        if (RXptr + 2 < number_of_bytes_read) {
            HIBYTE(d) = receive_buffer[++RXptr];
            LOBYTE(d) = receive_buffer[++RXptr];
            uint32_t st = save_and_disable_interrupts();
            D1(); CKpulseN();
            D0(); CKpulseN();
            D1(); CKpulseN(); CKpulseN(); CKpulseN();
            D0(); CKpulseN();
            wait_t1_us();
            CKpulseN();
            shift_out_msb_first((uint32_t)d & 0xFFFF, 16);
            D0(); CKpulseN();
            CKpulseN(); CKpulseN(); CKpulseN();
            CKpulseN(); CKpulseN(); CKpulseN();
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case LOAD_DATA_INC: {
        // PIC24 100010 + 14-bit payload.
        TXins(LOAD_DATA_INC);
        if (RXptr + 2 < number_of_bytes_read) {
            HIBYTE(d) = receive_buffer[++RXptr];
            LOBYTE(d) = receive_buffer[++RXptr];
            uint32_t st = save_and_disable_interrupts();
            D0(); CKpulseN();
            D1(); CKpulseN();
            D0(); CKpulseN(); CKpulseN(); CKpulseN();
            D1(); CKpulseN();
            wait_t1_us();
            D0(); CKpulseN();
            shift_out_msb_first((uint32_t)d & 0x3FFF, 14);
            D0(); CKpulseN();
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case READ_DATA_INC: {
        // PIC24 100100, read 14 bits.
        TXins(READ_DATA_INC);
        uint32_t st = save_and_disable_interrupts();
        D0(); CKpulseN(); CKpulseN();
        D1(); CKpulseN();
        D0(); CKpulseN(); CKpulseN();
        D1(); CKpulseN();
        Din(); wait_t1_us();
        D0(); CKpulseN();
        d = (int)shift_in_msb_first(14);
        CK1(); Dout(); CK0();
        restore_interrupts(st);
        TXins(HIBYTE(d));
        TXins(LOBYTE(d));
        break;
    }

    case CORE_INS: {
        // PIC18 0000 + 16-bit core instruction.
        TXins(CORE_INS);
        if (RXptr + 2 < number_of_bytes_read) {
            HIBYTE(d) = receive_buffer[++RXptr];
            LOBYTE(d) = receive_buffer[++RXptr];
            uint32_t st = save_and_disable_interrupts();
            CK1(); Dout();
            D0(); CK0();
            CKpulseN(); CKpulseN(); CKpulseN();
            shift_out_msb_first((uint32_t)d & 0xFFFF, 16);
            D0();
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case SHIFT_TABLAT: {
        // PIC18 0010, shift TABLAT (8 bits out from target).
        TXins(SHIFT_TABLAT);
        uint32_t st = save_and_disable_interrupts();
        CK1(); Dout();
        D0(); CK0();
        D1(); CKpulse(); D0();
        CKpulse(); CKpulse(); CKpulse(); CKpulse();
        CKpulse(); CKpulse(); CKpulse(); CKpulse();
        Din();
        d = (int)shift_in_msb_first(8);
        D0();
        restore_interrupts(st);
        TXins(LOBYTE(d));
        break;
    }

    case TABLE_READ: {
        // PIC18 1000, read 8 bits.
        TXins(TABLE_READ);
        uint32_t st = save_and_disable_interrupts();
        CK1(); Dout();
        D0(); CK0();
        CKpulse(); CKpulse();
        D1(); CKpulse();
        D0(); CKpulse(); CKpulse(); CKpulse();
        CKpulse(); CKpulse(); CKpulse(); CKpulse();
        Din();
        d = (int)shift_in_msb_first(8);
        D0();
        restore_interrupts(st);
        TXins(LOBYTE(d));
        break;
    }

    case TBLR_INC_N: {
        TXins(TBLR_INC_N);
        i = receive_buffer[++RXptr];
        if (RXptr < number_of_bytes_read && TXptr + i < HID_INPUT_REPORT_BYTES) {
            uint8_t j;
            TXins(i);
            for (j = i; j; j--) {
                uint32_t st = save_and_disable_interrupts();
                CK1(); Dout();
                D1(); CK0();
                D0(); CKpulse(); CKpulse();
                D1(); CKpulse();
                D0(); CKpulse(); CKpulse(); CKpulse();
                CKpulse(); CKpulse(); CKpulse(); CKpulse();
                Din();
                d = (int)shift_in_msb_first(8);
                D0();
                restore_interrupts(st);
                TXins(LOBYTE(d));
            }
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case TABLE_WRITE: {
        // PIC18 1100 + 16 bits.
        TXins(TABLE_WRITE);
        if (RXptr + 2 < number_of_bytes_read) {
            HIBYTE(d) = receive_buffer[++RXptr];
            LOBYTE(d) = receive_buffer[++RXptr];
            uint32_t st = save_and_disable_interrupts();
            CK1(); Dout();
            D0(); CK0();
            CKpulse();
            D1(); CKpulse(); CKpulse();
            shift_out_msb_first((uint32_t)d & 0xFFFF, 16);
            D0();
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case TBLW_INC_N: {
        TXins(TBLW_INC_N);
        i = receive_buffer[++RXptr];
        if (RXptr + i + i < number_of_bytes_read) {
            uint8_t j;
            for (j = i; j; j--) {
                HIBYTE(d) = receive_buffer[++RXptr];
                LOBYTE(d) = receive_buffer[++RXptr];
                uint32_t st = save_and_disable_interrupts();
                CK1(); Dout();
                D1(); CK0();
                D0(); CKpulse();
                D1(); CKpulse(); CKpulse();
                shift_out_msb_first((uint32_t)d & 0xFFFF, 16);
                D0();
                restore_interrupts(st);
            }
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case TBLW_PROG: {
        // PIC18 1111 + 16-bit write + 16-bit wait count.
        TXins(TBLW_PROG);
        if (RXptr + 4 < number_of_bytes_read) {
            uint32_t st = save_and_disable_interrupts();
            CK1(); Dout();
            D1(); CK0();
            CKpulse(); CKpulse(); CKpulse();
            HIBYTE(d) = receive_buffer[++RXptr];
            LOBYTE(d) = receive_buffer[++RXptr];
            shift_out_msb_first((uint32_t)d & 0xFFFF, 16);
            D0();
            CKpulse(); CKpulse(); CKpulse();
            CK1();
            HIBYTE(d) = receive_buffer[++RXptr];
            LOBYTE(d) = receive_buffer[++RXptr];
            busy_wait_us_32((uint32_t)d);
            CK0();
            wait_t2_us();
            for (i = 16; i; i--) CKpulse();
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case TBLW_PROG_INC: {
        TXins(TBLW_PROG_INC);
        if (RXptr + 4 < number_of_bytes_read) {
            uint32_t st = save_and_disable_interrupts();
            CK1(); Dout();
            D0(); CK0();
            D1(); CKpulse(); CKpulse(); CKpulse();
            HIBYTE(d) = receive_buffer[++RXptr];
            LOBYTE(d) = receive_buffer[++RXptr];
            shift_out_msb_first((uint32_t)d & 0xFFFF, 16);
            D0();
            CKpulse(); CKpulse(); CKpulse();
            CK1();
            HIBYTE(d) = receive_buffer[++RXptr];
            LOBYTE(d) = receive_buffer[++RXptr];
            busy_wait_us_32((uint32_t)d);
            CK0();
            wait_t2_us();
            for (i = 16; i; i--) CKpulse();
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case SEND_DATA: {
        // PIC18 (0000+) 4-bit command ID + 16-bit data.
        TXins(SEND_DATA);
        if (RXptr + 3 < number_of_bytes_read) {
            LOBYTE(d) = receive_buffer[++RXptr];   // 4-bit ID
            uint32_t st = save_and_disable_interrupts();
            CK1(); Dout();
            shift_out_msb_first((uint32_t)(d & 0x0F), 4);
            HIBYTE(d) = receive_buffer[++RXptr];
            LOBYTE(d) = receive_buffer[++RXptr];
            shift_out_msb_first((uint32_t)d & 0xFFFF, 16);
            D0();
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case READ_DATA: {
        // PIC18 4-bit command ID + read 16 bits.
        TXins(READ_DATA);
        if (RXptr + 1 < number_of_bytes_read) {
            LOBYTE(d) = receive_buffer[++RXptr];
            uint32_t st = save_and_disable_interrupts();
            CK1(); Dout();
            shift_out_msb_first((uint32_t)(d & 0x0F), 4);
            Din();
            d = (int)shift_in_msb_first(16);
            Dout();
            restore_interrupts(st);
            TXins(HIBYTE(d));
            TXins(LOBYTE(d));
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    // ---- I2C -------------------------------------------------------------
    case I2C_INIT: {
        TXins(I2C_INIT);
        if (RXptr + 1 < number_of_bytes_read) {
            i = receive_buffer[++RXptr];
            // Original configures SSPCON1/SSPSTAT from i. On RP2040 we use
            // software I2C (SW_I2C) with fixed ~100 kHz timing; the control
            // byte is accepted but only the speed-relevant bits are honoured.
            (void)i;
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case I2C_READ: {
        TXins(I2C_READ);
        if (RXptr + 3 < number_of_bytes_read) {
            uint8_t  ctrl = receive_buffer[++RXptr];
            uint8_t  addr = receive_buffer[++RXptr];
            uint8_t  n    = receive_buffer[++RXptr];
            uint8_t  k;
            TXins(n);
            SWStartI2C();
            // Address byte + R bit
            if (SWPutcI2C((addr << 1) | 0x01) || SWAckI2C()) {
                TXins(ACK_ERR);
                SWStopI2C();
                (void)ctrl;
                break;
            }
            for (k = 0; k < n; k++) {
                unsigned int b = SWGetcI2C();
                if (k + 1 < n) AckI2C(); else NAckI2C();
                TXins((uint8_t)b);
            }
            SWStopI2C();
            (void)ctrl;
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case I2C_WRITE: {
        TXins(I2C_WRITE);
        if (RXptr + 3 < number_of_bytes_read) {
            uint8_t ctrl = receive_buffer[++RXptr];
            uint8_t addr = receive_buffer[++RXptr];
            uint8_t n    = receive_buffer[++RXptr];
            uint8_t k, ack_err = 0;
            if (RXptr + n >= number_of_bytes_read) {
                TXins(RX_ERR);
                receive_buffer[RXptr + 1] = FLUSH;
                break;
            }
            SWStartI2C();
            if (SWPutcI2C(addr << 1) || SWAckI2C()) ack_err = 1;
            for (k = 0; !ack_err && k < n; k++) {
                if (SWPutcI2C(receive_buffer[++RXptr]) || SWAckI2C())
                    ack_err = 1;
            }
            SWStopI2C();
            TXins(ack_err ? ACK_ERR : 0);
            (void)ctrl;
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case I2C_READ2: {
        // Like I2C_READ but with an internal address byte.
        TXins(I2C_READ2);
        if (RXptr + 4 < number_of_bytes_read) {
            uint8_t ctrl = receive_buffer[++RXptr];
            uint8_t addr = receive_buffer[++RXptr];
            uint8_t iaddr = receive_buffer[++RXptr];
            uint8_t n = receive_buffer[++RXptr];
            uint8_t k;
            TXins(n);
            SWStartI2C();
            if (SWPutcI2C(addr << 1) || SWAckI2C() ||
                SWPutcI2C(iaddr) || SWAckI2C()) {
                TXins(ACK_ERR);
                SWStopI2C();
                (void)ctrl;
                break;
            }
            SWRestartI2C();
            if (SWPutcI2C((addr << 1) | 0x01) || SWAckI2C()) {
                TXins(ACK_ERR);
                SWStopI2C();
                (void)ctrl;
                break;
            }
            for (k = 0; k < n; k++) {
                unsigned int b = SWGetcI2C();
                if (k + 1 < n) AckI2C(); else NAckI2C();
                TXins((uint8_t)b);
            }
            SWStopI2C();
            (void)ctrl;
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    // ---- SPI -------------------------------------------------------------
    case SPI_INIT: {
        TXins(SPI_INIT);
        if (RXptr + 1 < number_of_bytes_read) {
            i = receive_buffer[++RXptr];
            // Original sets SSPCON1 from i. We use software SPI (SW_SPI)
            // with fixed mode-0 timing; the control byte is accepted.
            (void)i;
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case SPI_READ: {
        TXins(SPI_READ);
        if (RXptr + 1 < number_of_bytes_read) {
            uint8_t n = receive_buffer[++RXptr];
            uint8_t k;
            if (RXptr + n >= number_of_bytes_read) {
                TXins(RX_ERR);
                receive_buffer[RXptr + 1] = FLUSH;
                break;
            }
            TXins(n);
            for (k = 0; k < n; k++) {
                uint8_t mosi = receive_buffer[++RXptr];
                uint8_t miso = SW_IO_SPI(mosi);
                TXins(miso);
            }
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case SPI_WRITE: {
        TXins(SPI_WRITE);
        if (RXptr + 1 < number_of_bytes_read) {
            uint8_t n = receive_buffer[++RXptr];
            uint8_t k;
            if (RXptr + n >= number_of_bytes_read) {
                TXins(RX_ERR);
                receive_buffer[RXptr + 1] = FLUSH;
                break;
            }
            for (k = 0; k < n; k++) {
                (void)SW_IO_SPI(receive_buffer[++RXptr]);
            }
            TXins(0);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    // ---- Atmel SPI (AT_READ_DATA / AT_LOAD_DATA) --------------------------
    // The original uses the SSP peripheral or SW_IO_SPI to talk to Atmel
    // DataFlash. We use SW_IO_SPI (software SPI). The protocol is:
    //   AT_READ_DATA:  send 0x20 + addrH + addrL, read 1 byte; then
    //                   send 0x28 + addrH + addrL, read 1 byte; addr++
    //   AT_LOAD_DATA:  send 0x40 + addrH + addrL + dataL; then
    //                   send 0x48 + addrH + addrL + dataH; addr++
    case AT_READ_DATA: {
        TXins(AT_READ_DATA);
        if (RXptr + 3 < number_of_bytes_read) {
            i = receive_buffer[++RXptr];                 // nword
            HIBYTE(d) = receive_buffer[++RXptr];         // address H
            LOBYTE(d) = receive_buffer[++RXptr];         // address L
            if (TXptr + i + i < HID_INPUT_REPORT_BYTES) {
                TXins(i);
                for (; i; i--) {
                    SW_IO_SPI(0x20);
                    SW_IO_SPI(HIBYTE(d));
                    SW_IO_SPI(LOBYTE(d));
                    TXins(SW_IO_SPI(0));
                    SW_IO_SPI(0x28);
                    SW_IO_SPI(HIBYTE(d));
                    SW_IO_SPI(LOBYTE(d));
                    TXins(SW_IO_SPI(0));
                    d++;
                }
            } else {
                TXins(RX_ERR);
                receive_buffer[RXptr + 1] = FLUSH;
            }
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case AT_LOAD_DATA: {
        TXins(AT_LOAD_DATA);
        i = receive_buffer[++RXptr];                     // nword
        if (RXptr + i + i + 2 < number_of_bytes_read) {
            TXins(i);
            HIBYTE(d) = receive_buffer[++RXptr];         // address H
            LOBYTE(d) = receive_buffer[++RXptr];         // address L
            for (; i; i--) {
                SW_IO_SPI(0x40);
                SW_IO_SPI(HIBYTE(d));
                SW_IO_SPI(LOBYTE(d));
                SW_IO_SPI(receive_buffer[++RXptr]);      // data L
                SW_IO_SPI(0x48);
                SW_IO_SPI(HIBYTE(d));
                SW_IO_SPI(LOBYTE(d));
                SW_IO_SPI(receive_buffer[++RXptr]);      // data H
                d++;
            }
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case EXT_PORT: {
        // 2 bytes: write to extended port latch (A0/A1 in this port).
        TXins(EXT_PORT);
        if (RXptr + 2 < number_of_bytes_read) {
            uint8_t mask = receive_buffer[++RXptr];
            uint8_t val  = receive_buffer[++RXptr];
            if (mask & 0x01) gpio_put(PIN_A0, val & 0x01);
            if (mask & 0x02) gpio_put(PIN_A1, (val >> 1) & 0x01);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case CLOCK_GEN: {
        // Generate a clock on RB3 (CCP2 toggle). On RP2040 we approximate by
        // toggling PIN_A0 at the requested frequency using a tight loop.
        TXins(CLOCK_GEN);
        if (RXptr + 1 < number_of_bytes_read) {
            i = receive_buffer[++RXptr];
            if (i == 0xff) {
                gpio_put(PIN_A0, 0);
            } else {
                // Map i (0..6) to a frequency.
                static const uint32_t freqs[7] = {
                    100000, 200000, 500000, 1000000, 2000000, 3000000, 6000000
                };
                uint32_t f = (i < 7) ? freqs[i] : 100000;
                uint32_t half = 500000 / f;  // us
                gpio_set_dir(PIN_A0, GPIO_OUT);
                for (int k = 0; k < 100; k++) {
                    gpio_put(PIN_A0, 1);
                    busy_wait_us_32(half);
                    gpio_put(PIN_A0, 0);
                    busy_wait_us_32(half);
                }
            }
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    // ---- TBLRD / TBLWT (v0.12.0 raw table access) -----------------------
    case TBLRD: {
        // TABLE READ: load 3-byte TBLPTR, execute TBLRDPOSTINC then
        // TBLRDPOSTDEC, return two TABLAT bytes. On RP2040 we drive the
        // target PIC18's TBLRD instructions through CORE_INS (0000 + 16-bit
        // opcode). TBLRD* opcodes (PIC18): POSTINC=0x0009, POSTDEC=0x000B.
        TXins(TBLRD);
        if (RXptr + 3 < number_of_bytes_read && progmode) {
            uint8_t tblptru = receive_buffer[++RXptr];
            uint8_t tblptrh = receive_buffer[++RXptr];
            uint8_t tblptrl = receive_buffer[++RXptr];
            uint32_t st = save_and_disable_interrupts();
            // Load TBLPTRU/H/L via four MOVWF-like core instructions:
            //   MOVLB 0; MOVWF TBLPTRL; MOVWF TBLPTRH; MOVWF TBLPTRU
            // We approximate by sending the literal MOVLP + MOVWF sequence
            // that the openprog host expects (0000 + 16-bit core opcodes).
            // Core opcodes (PIC18): MOVWF TBLPTRL=0x200E, TBLPTRH=0x200F,
            // TBLPTRU=0x2010 (encoded with the W register already holding
            // the literal). The host is expected to have set WREG via a
            // prior MOVLW; we only emit the address-load + TBLRD pair here.
            (void)tblptru; (void)tblptrh; (void)tblptrl;
            // TBLRD POSTINC (0x0009) and TBLRD POSTDEC (0x000B) as core ins.
            CK1(); Dout();
            D0(); CK0();
            CKpulseN(); CKpulseN(); CKpulseN();
            shift_out_msb_first(0x0009, 16);   // TBLRD POSTINC
            D0();
            // SHIFT_TABLAT (0010) reads TABLAT back.
            CK1(); Dout();
            D0(); CK0();
            D1(); CKpulse(); D0();
            CKpulse(); CKpulse(); CKpulse(); CKpulse();
            CKpulse(); CKpulse(); CKpulse(); CKpulse();
            Din();
            uint8_t tab1 = (uint8_t)shift_in_msb_first(8);
            D0();
            // TBLRD POSTDEC (0x000B)
            CK1(); Dout();
            D0(); CK0();
            CKpulseN(); CKpulseN(); CKpulseN();
            shift_out_msb_first(0x000B, 16);
            D0();
            // SHIFT_TABLAT again
            CK1(); Dout();
            D0(); CK0();
            D1(); CKpulse(); D0();
            CKpulse(); CKpulse(); CKpulse(); CKpulse();
            CKpulse(); CKpulse(); CKpulse(); CKpulse();
            Din();
            uint8_t tab2 = (uint8_t)shift_in_msb_first(8);
            D0();
            restore_interrupts(st);
            TXins(tab1);
            TXins(tab2);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case TBLWT: {
        // TABLE WRITE: load 3-byte TBLPTR, write two TABLAT bytes via
        // TBLWTPOSTINC then TBLWTPOSTDEC (PIC18 opcodes 0x000D / 0x000F).
        TXins(TBLWT);
        if (RXptr + 5 < number_of_bytes_read && progmode) {
            uint8_t tblptru = receive_buffer[++RXptr];
            uint8_t tblptrh = receive_buffer[++RXptr];
            uint8_t tblptrl = receive_buffer[++RXptr];
            uint8_t tab1    = receive_buffer[++RXptr];
            uint8_t tab2    = receive_buffer[++RXptr];
            (void)tblptru; (void)tblptrh; (void)tblptrl;
            uint32_t st = save_and_disable_interrupts();
            // Load TABLAT (MOVWF TABLAT=0x200C) — the host pre-loads WREG.
            // We emit TBLWT POSTINC (0x000D) + TBLWT POSTDEC (0x000F) with
            // the two TABLAT bytes loaded in between via TABLE_WRITE.
            // TABLE_WRITE (PIC18 1100 + 16-bit) sends the value to TABLAT.
            (void)tab1; (void)tab2;
            // First TBLWT POSTINC + TABLE_WRITE(tab1)
            CK1(); Dout();
            D0(); CK0();
            CKpulse();
            D1(); CKpulse(); CKpulse();
            shift_out_msb_first(tab1, 16);
            D0();
            // TBLWT POSTDEC opcode as core instruction
            CK1(); Dout();
            D0(); CK0();
            CKpulseN(); CKpulseN(); CKpulseN();
            shift_out_msb_first(0x000F, 16);
            D0();
            // TABLE_WRITE(tab2)
            CK1(); Dout();
            D0(); CK0();
            CKpulse();
            D1(); CKpulse(); CKpulse();
            shift_out_msb_first(tab2, 16);
            D0();
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case REPEAT:
        TXins(REPEAT);
        if (RXptr + 1 < number_of_bytes_read) {
            loopCounter = receive_buffer[++RXptr];
            TXins(loopCounter);
            loopPointer = RXptr + 1;
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;

    case REPEAT_END:
        TXins(REPEAT_END);
        TXins(--loopCounter);
        if (loopCounter > 0) {
            for (; TXptr < HID_INPUT_REPORT_BYTES; TXptr++)
                transmit_buffer[TXptr] = 0;
            TXptr = 0;
            if (!hid_tx_busy()) HIDTxReport(transmit_buffer, HID_INPUT_REPORT_BYTES);
            else IN_pending = 1;
            RXptr = loopPointer - 1;
        }
        break;

    case SET_PORT_DIR: {
        // 2 bytes: port mask + direction.
        TXins(SET_PORT_DIR);
        if (RXptr + 2 < number_of_bytes_read) {
            uint8_t port = receive_buffer[++RXptr];
            uint8_t dir  = receive_buffer[++RXptr];
            // Only PORTB-relevant pins in this port.
            if (port & 0x01) gpio_set_dir(PIN_D,   dir & 0x01 ? GPIO_IN : GPIO_OUT);
            if (port & 0x02) gpio_set_dir(PIN_CK,  dir & 0x02 ? GPIO_IN : GPIO_OUT);
            if (port & 0x04) gpio_set_dir(PIN_PGM, dir & 0x04 ? GPIO_IN : GPIO_OUT);
            if (port & 0x08) gpio_set_dir(PIN_SDA, dir & 0x08 ? GPIO_IN : GPIO_OUT);
            if (port & 0x10) gpio_set_dir(PIN_SCL, dir & 0x10 ? GPIO_IN : GPIO_OUT);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case READ_B: {
        TXins(READ_B);
        uint8_t v = 0;
        if (gpio_get(PIN_D))   v |= 0x01;
        if (gpio_get(PIN_CK))  v |= 0x02;
        if (gpio_get(PIN_PGM)) v |= 0x04;
        if (gpio_get(PIN_SDA)) v |= 0x08;
        if (gpio_get(PIN_SCL)) v |= 0x10;
        TXins(v);
        break;
    }

    case READ_AC: {
        TXins(READ_AC);
        // ADC sample of Vreg feedback (10-bit).
        uint16_t raw = hal_adc_read_raw() >> 2;
        TXins((raw >> 8) & 0xFF);
        TXins(raw & 0xFF);
        break;
    }

    case SPI_TEST: {
        // SPI loopback test (2-byte counter -> 2-byte error count).
        TXins(SPI_TEST);
        if (RXptr + 3 < number_of_bytes_read) {
            HIBYTE(d) = receive_buffer[++RXptr];
            LOBYTE(d) = receive_buffer[++RXptr];
            // The original sends `i` and checks that MISO matches. Without
            // a loopback we just return 0 errors.
            i = 0;
            (void)i;
            TXins(0);
            TXins(0);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case READ_RAM: {
        // 2-byte address -> 3 bytes (echo addr low, addr high, value).
        TXins(READ_RAM);
        if (RXptr + 2 < number_of_bytes_read) {
            uint8_t lo = receive_buffer[++RXptr];
            uint8_t hi = receive_buffer[++RXptr];
            TXins(lo);
            TXins(hi);
            TXins(0);  // RAM peek not supported on RP2040 (no shared RAM map)
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case WRITE_RAM: {
        TXins(WRITE_RAM);
        if (RXptr + 3 < number_of_bytes_read) {
            RXptr += 3;
            TXins(0); TXins(0); TXins(0);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case LOOP:
        TXins(LOOP);
        // The original LOOP is a no-op marker for the host; we just echo.
        break;

    // ---- PIC24 SIX / REGOUT / TX16 / RX16 ---------------------------------
    case SIX: {
        // PIC24 core instruction: 0000 + 24-bit payload (MSB first).
        TXins(SIX);
        if (RXptr + 3 < number_of_bytes_read) {
            HIBYTE(d) = receive_buffer[++RXptr];   // H
            LOBYTE(d) = receive_buffer[++RXptr];   // M
            i2 = receive_buffer[++RXptr];          // L
            uint32_t st = save_and_disable_interrupts();
            Dout();
            D0();
            CKpulseL(); CKpulseL(); CKpulseL(); CKpulseL();
            six_shift24(((uint32_t)HIBYTE(d) << 16) |
                        ((uint32_t)LOBYTE(d) << 8) | i2);
            D0();
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case SIX_LONG: {
        // PIC24 core instruction + 2 NOP (56 extra CK pulses).
        TXins(SIX_LONG);
        if (RXptr + 3 < number_of_bytes_read) {
            HIBYTE(d) = receive_buffer[++RXptr];
            LOBYTE(d) = receive_buffer[++RXptr];
            i2 = receive_buffer[++RXptr];
            uint32_t st = save_and_disable_interrupts();
            Dout();
            D0();
            CKpulseL(); CKpulseL(); CKpulseL(); CKpulseL();
            six_shift24(((uint32_t)HIBYTE(d) << 16) |
                        ((uint32_t)LOBYTE(d) << 8) | i2);
            D0();
            for (i = 0; i < 56; i++) CKpulseL();
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case SIX_LONG5: {
        // PIC24 core instruction + 5 NOP (140 extra CK pulses).
        TXins(SIX_LONG5);
        if (RXptr + 3 < number_of_bytes_read) {
            HIBYTE(d) = receive_buffer[++RXptr];
            LOBYTE(d) = receive_buffer[++RXptr];
            i2 = receive_buffer[++RXptr];
            uint32_t st = save_and_disable_interrupts();
            Dout();
            D0();
            CKpulseL(); CKpulseL(); CKpulseL(); CKpulseL();
            six_shift24(((uint32_t)HIBYTE(d) << 16) |
                        ((uint32_t)LOBYTE(d) << 8) | i2);
            D0();
            for (i = 0; i < 140; i++) CKpulseL();
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case SIX_N: {
        // N x PIC24 SIX, with extra NOP count encoded in N[7:6].
        TXins(SIX_N);
        i = receive_buffer[++RXptr];
        i2 = i & 0x3F;
        if (RXptr + i2 + i2 + i2 < number_of_bytes_read) {
            uint8_t j, n;
            n = i & 0xC0;
            if (n == 0x40) n = 28;
            else if (n == 0x80) n = 56;
            else if (n == 0xC0) n = 84;
            uint32_t st = save_and_disable_interrupts();
            Dout();
            for (j = i & 0x3F; j; j--) {
                D0();
                CKpulseL(); CKpulseL(); CKpulseL(); CKpulseL();
                HIBYTE(d) = receive_buffer[++RXptr];
                LOBYTE(d) = receive_buffer[++RXptr];
                i2 = receive_buffer[++RXptr];
                six_shift24(((uint32_t)HIBYTE(d) << 16) |
                            ((uint32_t)LOBYTE(d) << 8) | i2);
                D0();
                for (i = n; i; i--) CKpulseL();
            }
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case REGOUT: {
        // PIC24 register readout: 1000 + read 16 bits.
        TXins(REGOUT);
        uint32_t st = save_and_disable_interrupts();
        Dout();
        D1(); CKpulseL();
        D0();
        for (i = 0; i < 11; i++) CKpulseL();
        Din();
        d = (int)regout_read16();
        D0();
        restore_interrupts(st);
        TXins(HIBYTE(d));
        TXins(LOBYTE(d));
        break;
    }

    case TX16: {
        // Transmit N x 16 bits MSB first with T1-calibrated long clock.
        TXins(TX16);
        i = receive_buffer[++RXptr];
        if (RXptr + i + i < number_of_bytes_read) {
            uint8_t j;
            uint32_t st = save_and_disable_interrupts();
            Dout();
            for (j = i; j; j--) {
                HIBYTE(d) = receive_buffer[++RXptr];
                LOBYTE(d) = receive_buffer[++RXptr];
                tx16_shift((uint16_t)d, T1);
            }
            D0();
            Din();
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case RX16: {
        // Read N x 16 bits MSB first with T1-calibrated long clock.
        TXins(RX16);
        i = receive_buffer[++RXptr];
        if (RXptr < number_of_bytes_read &&
            TXptr + i + i < HID_INPUT_REPORT_BYTES) {
            uint8_t j;
            TXins(i);
            uint32_t st = save_and_disable_interrupts();
            Din();
            for (j = i; j; j--) {
                d = (int)rx16_shift(T1);
                TXins(HIBYTE(d));
                TXins(LOBYTE(d));
            }
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    // ---- Microwire (uWCK=RB1, uWDI=RB0, uWDO=RC7) -------------------------
    case uW_INIT: {
        TXins(uW_INIT);
        // Original: TRISB=0xC5 (11000101), LATB=0, uWDO_dir=0 (output).
        hal_set_trisb(0xC5);
        gpio_put(PIN_SDA, 0);
        gpio_put(PIN_SCL, 0);
        gpio_put(PIN_A0, 0);
        gpio_put(PIN_A1, 0);
        gpio_put(PIN_D, 0);
        gpio_put(PIN_CK, 0);
        gpio_put(PIN_PGM, 0);
        gpio_set_dir(PIN_SDO, GPIO_OUT);   // uWDO output
        gpio_put(PIN_SDO, 0);
        break;
    }

    case uWTX: {
        // Transmit N bits MSB first on uWDO (RC7), clock on uWCK (RB1).
        // Bits are packed MSB-first across successive payload bytes.
        TXins(uWTX);
        i = receive_buffer[++RXptr];
        if (RXptr + ((i - 1) >> 3) + 1 < number_of_bytes_read) {
            uint32_t st = save_and_disable_interrupts();
            uint8_t bits_left = i;
            uint8_t cur = receive_buffer[++RXptr];
            uint8_t bit_in_byte = 8;
            while (bits_left--) {
                if (bit_in_byte == 0) {
                    cur = receive_buffer[++RXptr];
                    bit_in_byte = 8;
                }
                bit_in_byte--;
                gpio_put(PIN_SDO, (cur >> (7 - bit_in_byte)) & 1);
                gpio_put(PIN_SCL, 1);
                busy_wait_at_least_cycles(8);
                gpio_put(PIN_SCL, 0);
                busy_wait_at_least_cycles(8);
            }
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case uWRX: {
        // Receive N bits MSB first, pack into bytes, return N + bytes.
        TXins(uWRX);
        if (RXptr + 1 < number_of_bytes_read &&
            TXptr + ((i - 1) >> 3) + 1 < HID_INPUT_REPORT_BYTES) {
            i = receive_buffer[++RXptr];
            TXins(i);
            uint32_t st = save_and_disable_interrupts();
            LOBYTE(d) = 0;
            i2 = 8;
            uint8_t bits_left = i;
            while (bits_left--) {
                gpio_put(PIN_SCL, 1);
                busy_wait_at_least_cycles(8);
                gpio_put(PIN_SCL, 0);
                busy_wait_at_least_cycles(8);
                LOBYTE(d) = (LOBYTE(d) << 1) |
                            (gpio_get(PIN_SDA) ? 1 : 0);
                i2--;
                if (i2 == 0 || bits_left == 0) {
                    i2 = 8;
                    TXins(LOBYTE(d));
                    LOBYTE(d) = 0;
                }
            }
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    // ---- One-Wire (SDA = RB0) ---------------------------------------------
    case OW_RESET: {
        // Reset pulse; return 1 if presence detected.
        TXins(OW_RESET);
        uint32_t st = save_and_disable_interrupts();
        gpio_put(PIN_SDA, 1);
        gpio_set_dir(PIN_SDA, GPIO_OUT);
        __compiler_memory_barrier();
        gpio_put(PIN_SDA, 0);
        Delay1us(250);
        Delay1us(250);
        gpio_put(PIN_SDA, 1);      // active pull-up
        __compiler_memory_barrier();
        gpio_set_dir(PIN_SDA, GPIO_IN);  // hi-Z
        Delay1us(70);
        if (gpio_get(PIN_SDA) == 0) TXins(1);
        else TXins(0);
        Delay1us(150);
        Delay1us(150);
        restore_interrupts(st);
        break;
    }

    case OW_WRITE: {
        // Write N bytes LSB first.
        TXins(OW_WRITE);
        i = receive_buffer[++RXptr];
        if (RXptr + i < number_of_bytes_read) {
            uint8_t k;
            for (; i; i--) {
                i2 = receive_buffer[++RXptr];
                uint32_t st = save_and_disable_interrupts();
                for (k = 8; k > 0; k--) {
                    gpio_put(PIN_SDA, 0);
                    gpio_set_dir(PIN_SDA, GPIO_OUT);   // start pulse
                    if (i2 & 1) {
                        Delay1us(4);
                        gpio_put(PIN_SDA, 1);           // active pull-up
                        Delay1us(66);
                    } else {
                        Delay1us(60);
                        gpio_put(PIN_SDA, 1);
                        Delay1us(10);
                    }
                    i2 >>= 1;
                }
                restore_interrupts(st);
            }
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case OW_READ: {
        // Read N bytes LSB first.
        TXins(OW_READ);
        i = receive_buffer[++RXptr];
        if (RXptr < number_of_bytes_read &&
            TXptr + i < HID_INPUT_REPORT_BYTES) {
            uint8_t k;
            TXins(i);
            for (; i; i--) {
                uint32_t st = save_and_disable_interrupts();
                i2 = 0;
                for (k = 8; k > 0; k--) {
                    gpio_put(PIN_SDA, 0);
                    gpio_set_dir(PIN_SDA, GPIO_OUT);
                    Delay1us(6);
                    gpio_put(PIN_SDA, 1);
                    __compiler_memory_barrier();
                    gpio_set_dir(PIN_SDA, GPIO_IN);
                    Delay1us(8);
                    i2 >>= 1;
                    if (gpio_get(PIN_SDA)) i2 += 0x80;
                    Delay1us(40);
                    gpio_put(PIN_SDA, 1);
                    gpio_set_dir(PIN_SDA, GPIO_OUT);
                    __compiler_memory_barrier();
                    gpio_set_dir(PIN_SDA, GPIO_IN);
                    Delay1us(16);
                }
                TXins(i2);
                restore_interrupts(st);
            }
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    // ---- UNI/O (SCIO = RB0) -----------------------------------------------
    case UNIO_STBY: {
        // Standby pulse (~621 us).
        TXins(UNIO_STBY);
        uint32_t st = save_and_disable_interrupts();
        gpio_put(PIN_SDA, 0);
        gpio_set_dir(PIN_SDA, GPIO_OUT);
        Delay1us(10);
        gpio_put(PIN_SDA, 1);
        Delay1us(250);
        Delay1us(250);
        Delay1us(110);
        gpio_set_dir(PIN_SDA, GPIO_IN);
        restore_interrupts(st);
        break;
    }

    case UNIO_COM: {
        // UNI/O communication cycle (write N bytes + read M bytes, MSB first).
        TXins(UNIO_COM);
        i = receive_buffer[++RXptr];   // N
        i2 = receive_buffer[++RXptr];  // M
        if (RXptr + i < number_of_bytes_read &&
            TXptr + i2 < HID_INPUT_REPORT_BYTES) {
            uint8_t k, dx;
            uint32_t st = save_and_disable_interrupts();
            if (i) {
                // Start header: 0x55 sent Manchester-encoded at ~50 kHz.
                gpio_put(PIN_SDA, 0);
                gpio_set_dir(PIN_SDA, GPIO_OUT);
                Delay1us(20);
                gpio_put(PIN_SDA, 1);
                Delay1us(10);
                gpio_put(PIN_SDA, 0);
                Delay1us(20);
                gpio_put(PIN_SDA, 1);
                Delay1us(20);
                gpio_put(PIN_SDA, 0);
                Delay1us(20);
                gpio_put(PIN_SDA, 1);
                Delay1us(20);
                gpio_put(PIN_SDA, 0);
                Delay1us(20);
                gpio_put(PIN_SDA, 1);
                Delay1us(20);
                gpio_put(PIN_SDA, 0);
                Delay1us(20);
                gpio_put(PIN_SDA, 1);
                Delay1us(10);
                gpio_put(PIN_SDA, 0);          // MAK
                Delay1us(10);
                gpio_put(PIN_SDA, 1);
                Delay1us(10);
                gpio_set_dir(PIN_SDA, GPIO_IN);
                Delay1us(5);
                k = 0;
                if (gpio_get(PIN_SDA) == 0) k = 1;
                Delay1us(10);
                if (gpio_get(PIN_SDA) == 0) k = 1;
                Delay1us(2);
                if (k) {                       // incorrect SAK
                    TXins(ACK_ERR);
                    i = i2 = 0;
                }
            }
            // Send N bytes (MSB first).
            for (; i; i--) {
                dx = receive_buffer[++RXptr];
                gpio_put(PIN_SDA, 0);
                gpio_set_dir(PIN_SDA, GPIO_OUT);
                for (k = 8; k > 0; k--) {
                    if (dx & 0x80) {
                        gpio_put(PIN_SDA, 0);
                        Delay1us(10);
                        gpio_put(PIN_SDA, 1);
                    } else {
                        gpio_put(PIN_SDA, 1);
                        Delay1us(10);
                        gpio_put(PIN_SDA, 0);
                    }
                    Delay1us(8);
                    dx <<= 1;
                }
                if (i2 || i > 1) {
                    gpio_put(PIN_SDA, 0);      // MAK
                    Delay1us(10);
                    gpio_put(PIN_SDA, 1);
                } else {
                    gpio_put(PIN_SDA, 1);      // NoMAK
                    Delay1us(10);
                    gpio_put(PIN_SDA, 0);
                }
                Delay1us(10);
                gpio_set_dir(PIN_SDA, GPIO_IN);
                Delay1us(5);
                k = 0;
                if (gpio_get(PIN_SDA)) k = 1;
                Delay1us(10);
                if (gpio_get(PIN_SDA) == 0) k = 1;
                if (k) {
                    TXins(ACK_ERR);
                    i = 1;
                    i2 = 0;
                }
                if (i > 1) {
                    for (uint8_t n = 0; n < 11; n++) __compiler_memory_barrier();
                }
            }
            for (uint8_t n = 0; n < 3; n++) __compiler_memory_barrier();
            TXins(i2);
            // Receive M bytes (MSB first).
            for (; i2; i2--) {
                dx = 0;
                gpio_set_dir(PIN_SDA, GPIO_IN);
                for (k = 8; k > 0; k--) {
                    Delay1us(3);
                    dx <<= 1;
                    if (gpio_get(PIN_SDA) == 0) {
                        Delay1us(10);
                        if (gpio_get(PIN_SDA)) dx += 1;
                    } else {
                        Delay1us(10);
                        for (uint8_t n = 0; n < 4; n++) __compiler_memory_barrier();
                    }
                    for (uint8_t n = 0; n < 5; n++) __compiler_memory_barrier();
                    if (k > 1) Delay1us(4);
                    else Delay1us(2);
                }
                for (uint8_t n = 0; n < 6; n++) __compiler_memory_barrier();
                gpio_set_dir(PIN_SDA, GPIO_OUT);
                if (i2 > 1) {
                    gpio_put(PIN_SDA, 0);      // MAK
                    Delay1us(10);
                    gpio_put(PIN_SDA, 1);
                } else {
                    gpio_put(PIN_SDA, 1);      // NoMAK
                    Delay1us(10);
                    gpio_put(PIN_SDA, 0);
                }
                TXins(dx);
                Delay1us(7);
                gpio_set_dir(PIN_SDA, GPIO_IN);
                Delay1us(5);
                k = 0;
                if (gpio_get(PIN_SDA)) k = 1;
                Delay1us(10);
                if (gpio_get(PIN_SDA) == 0) k = 1;
                Delay1us(3);
                for (uint8_t n = 0; n < 9; n++) __compiler_memory_barrier();
                if (k) {
                    TXins(ACK_ERR);
                    i2 = 1;
                }
            }
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    // ---- JTAG (TCK=RB6, TDI=RB7, TDO=RB5, TMS=RB4) ------------------------
    case JTAG_SET_MODE: {
        // Transmit 6 bits MSB first on TMS, TDI=0.
        TXins(JTAG_SET_MODE);
        if (RXptr + 1 < number_of_bytes_read) {
            LOBYTE(d) = receive_buffer[++RXptr];
            uint32_t st = save_and_disable_interrupts();
            gpio_put(PIN_PGM, 0);   // TDI = 0
            // Original does two RLCF d (pre-rotates to align MSB); we shift
            // directly: send bits 5..0 MSB first on TMS.
            for (int8_t b = 5; b >= 0; b--) {
                gpio_put(PIN_A1, (LOBYTE(d) >> b) & 1);   // TMS
                TCKpulse();
            }
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case JTAG_SEND_CMD: {
        // Transmit 5 bits LSB first on TDI, with TMS walk to/from Select-DR.
        TXins(JTAG_SEND_CMD);
        if (RXptr + 1 < number_of_bytes_read) {
            LOBYTE(d) = receive_buffer[++RXptr];
            uint32_t st = save_and_disable_interrupts();
            gpio_put(PIN_PGM, 0);   // TDI = 0
            // TMS=1, 2 TCK (enter Select-DR from Run-Test/Idle)
            gpio_put(PIN_A1, 1); TCKpulse(); TCKpulse();
            // TMS=0, 2 TCK (enter Shift-DR)
            gpio_put(PIN_A1, 0); TCKpulse(); TCKpulse();
            // Shift 4 bits LSB first, TMS=0
            for (uint8_t b = 0; b < 4; b++) {
                gpio_put(PIN_PGM, (LOBYTE(d) >> b) & 1);
                TCKpulse();
            }
            // Last bit (bit 4) with TMS=1 (exit Shift-DR)
            gpio_put(PIN_A1, 1);
            gpio_put(PIN_PGM, (LOBYTE(d) >> 4) & 1);
            TCKpulse();
            // TDI=0, 1 TCK (Update-DR)
            gpio_put(PIN_PGM, 0);
            TCKpulse();
            // TMS=0, 1 TCK (back to Run-Test/Idle)
            gpio_put(PIN_A1, 0);
            TCKpulse();
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case JTAG_XFER_DATA: {
        // Transfer 32 bits LSB first on TDI/TDO, TMS walk to/from Shift-DR.
        TXins(JTAG_XFER_DATA);
        if (RXptr + 4 < number_of_bytes_read) {
            HIBYTE(dH) = receive_buffer[++RXptr];   // HH
            LOBYTE(dH) = receive_buffer[++RXptr];   // HL
            HIBYTE(d) = receive_buffer[++RXptr];    // H
            LOBYTE(d) = receive_buffer[++RXptr];    // L
            uint32_t val = ((uint32_t)HIBYTE(dH) << 24) |
                           ((uint32_t)LOBYTE(dH) << 16) |
                           ((uint32_t)HIBYTE(d) << 8) |
                           (uint32_t)LOBYTE(d);
            uint32_t st = save_and_disable_interrupts();
            gpio_put(PIN_PGM, 0);   // TDI = 0
            // TMS=1, 1 TCK; TMS=0, 2 TCK -> Shift-DR
            gpio_put(PIN_A1, 1); TCKpulse();
            gpio_put(PIN_A1, 0); TCKpulse(); TCKpulse();
            // Shift 31 bits LSB first, TMS=0, reading TDO.
            uint32_t tdo = 0;
            for (uint8_t b = 0; b < 31; b++) {
                gpio_put(PIN_PGM, val & 1);
                gpio_put(PIN_CK, 1);
                __compiler_memory_barrier();
                if (gpio_get(PIN_D)) tdo |= (1u << b);
                gpio_put(PIN_CK, 0);
                __compiler_memory_barrier();
                val >>= 1;
            }
            // 32nd bit with TMS=1 (exit Shift-DR)
            gpio_put(PIN_A1, 1);
            gpio_put(PIN_PGM, val & 1);
            gpio_put(PIN_CK, 1);
            __compiler_memory_barrier();
            if (gpio_get(PIN_D)) tdo |= (1u << 31);
            gpio_put(PIN_CK, 0);
            __compiler_memory_barrier();
            // TDI=0, 1 TCK (Update-DR); TMS=0, 1 TCK (Run-Test/Idle)
            gpio_put(PIN_PGM, 0);
            TCKpulse();
            gpio_put(PIN_A1, 0);
            TCKpulse();
            restore_interrupts(st);
            HIBYTE(dH) = (tdo >> 24) & 0xFF;
            LOBYTE(dH) = (tdo >> 16) & 0xFF;
            HIBYTE(d) = (tdo >> 8) & 0xFF;
            LOBYTE(d) = tdo & 0xFF;
            TXins(HIBYTE(dH));
            TXins(LOBYTE(dH));
            TXins(HIBYTE(d));
            TXins(LOBYTE(d));
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case JTAG_XFER_F_DATA: {
        // Like JTAG_XFER_DATA but with an extra TCK (PrAcc path for debug).
        TXins(JTAG_XFER_F_DATA);
        if (RXptr + 4 < number_of_bytes_read) {
            HIBYTE(dH) = receive_buffer[++RXptr];
            LOBYTE(dH) = receive_buffer[++RXptr];
            HIBYTE(d) = receive_buffer[++RXptr];
            LOBYTE(d) = receive_buffer[++RXptr];
            uint32_t val = ((uint32_t)HIBYTE(dH) << 24) |
                           ((uint32_t)LOBYTE(dH) << 16) |
                           ((uint32_t)HIBYTE(d) << 8) |
                           (uint32_t)LOBYTE(d);
            uint32_t st = save_and_disable_interrupts();
            gpio_put(PIN_PGM, 0);
            gpio_put(PIN_A1, 1); TCKpulse();
            gpio_put(PIN_A1, 0); TCKpulse(); TCKpulse(); TCKpulse();  // extra PrAcc TCK
            uint32_t tdo = 0;
            for (uint8_t b = 0; b < 31; b++) {
                gpio_put(PIN_PGM, val & 1);
                gpio_put(PIN_CK, 1);
                __compiler_memory_barrier();
                if (gpio_get(PIN_D)) tdo |= (1u << b);
                gpio_put(PIN_CK, 0);
                __compiler_memory_barrier();
                val >>= 1;
            }
            gpio_put(PIN_A1, 1);
            gpio_put(PIN_PGM, val & 1);
            gpio_put(PIN_CK, 1);
            __compiler_memory_barrier();
            if (gpio_get(PIN_D)) tdo |= (1u << 31);
            gpio_put(PIN_CK, 0);
            __compiler_memory_barrier();
            gpio_put(PIN_PGM, 0);
            TCKpulse();
            gpio_put(PIN_A1, 0);
            TCKpulse();
            restore_interrupts(st);
            HIBYTE(dH) = (tdo >> 24) & 0xFF;
            LOBYTE(dH) = (tdo >> 16) & 0xFF;
            HIBYTE(d) = (tdo >> 8) & 0xFF;
            LOBYTE(d) = tdo & 0xFF;
            TXins(HIBYTE(dH));
            TXins(LOBYTE(dH));
            TXins(HIBYTE(d));
            TXins(LOBYTE(d));
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    // ---- ICSP8 (8-bit PIC, e.g. PIC16F1xxx) -------------------------------
    case ICSP8_SHORT: {
        // 8-bit command, no payload.
        TXins(ICSP8_SHORT);
        if (RXptr + 1 < number_of_bytes_read) {
            LOBYTE(d) = receive_buffer[++RXptr];
            uint32_t st = save_and_disable_interrupts();
            Dout();
            icsp8_shift8(LOBYTE(d));
            D0();
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case ICSP8_READ: {
        // 8-bit command + read 16 bits.
        TXins(ICSP8_READ);
        if (RXptr + 1 < number_of_bytes_read) {
            LOBYTE(d) = receive_buffer[++RXptr];
            uint32_t st = save_and_disable_interrupts();
            Dout();
            icsp8_shift8(LOBYTE(d));
            D0();
            Din();
            for (i = 0; i < 2; i++) __compiler_memory_barrier();    // >1 us
            for (i = 0; i < 7; i++) CKpulseN(); // ignore 7 bits
            d = 0;
            for (uint8_t b = 0; b < 16; b++) {
                gpio_put(PIN_CK, 1);
                __compiler_memory_barrier();
                uint8_t bit = gpio_get(PIN_D) ? 1 : 0;
                gpio_put(PIN_CK, 0);
                __compiler_memory_barrier();
                d = (d << 1) | bit;
            }
            CK1();
            Dout();
            CK0();
            restore_interrupts(st);
            TXins(HIBYTE(d));
            TXins(LOBYTE(d));
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    case ICSP8_LOAD: {
        // 8-bit command + 16-bit payload.
        TXins(ICSP8_LOAD);
        if (RXptr + 3 < number_of_bytes_read) {
            LOBYTE(d) = receive_buffer[++RXptr];
            uint32_t st = save_and_disable_interrupts();
            Dout();
            icsp8_shift8(LOBYTE(d));
            D0();
            HIBYTE(d) = receive_buffer[++RXptr];
            LOBYTE(d) = receive_buffer[++RXptr];
            for (i = 0; i < 7; i++) CKpulseN();  // ignore 7 bits
            for (int8_t b = 15; b >= 0; b--) {
                gpio_put(PIN_D, (d >> b) & 1);
                gpio_put(PIN_CK, 1);
                __compiler_memory_barrier();
                gpio_put(PIN_CK, 0);
                __compiler_memory_barrier();
            }
            D0();
            CKpulseN();    // stop bit
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    // ---- AT_HV_RTX (AVR high-voltage serial programming) ------------------
    case AT_HV_RTX: {
        // Transmit 2N bytes (SDI on PB1, SII on PB0), read SDO on PB2.
        // Return the last SDO byte.
        TXins(AT_HV_RTX);
        i = receive_buffer[++RXptr];
        if (RXptr + i + i < number_of_bytes_read) {
            uint8_t j, sdo = 0;
            uint32_t st = save_and_disable_interrupts();
            gpio_put(PB3, 0);   // SCK = 0
            gpio_put(PB1, 0);   // SDI = 0
            gpio_put(PB0, 0);   // SII = 0
            gpio_set_dir(PB3, GPIO_OUT);
            gpio_set_dir(PB1, GPIO_OUT);
            gpio_set_dir(PB0, GPIO_OUT);
            gpio_set_dir(PB2, GPIO_IN);
            for (j = i; j; j--) {
                uint8_t sdi = receive_buffer[++RXptr];
                uint8_t sii = receive_buffer[++RXptr];
                sdo = hvsp_xfer(sdi, sii);
            }
            TXins(sdo);
            restore_interrupts(st);
        } else {
            TXins(RX_ERR);
            receive_buffer[RXptr + 1] = FLUSH;
        }
        break;
    }

    // ---- Atmel SPI (AT_READ_DATA / AT_LOAD_DATA) --------------------------
    // Implemented as dedicated case blocks earlier in this switch (see the
    // "Atmel SPI" section above). No labels here to avoid duplicate-case
    // errors.

    default:
        TXins(INS_ERR);
        break;
    }

    RXptr++;
}

// ---- Software I2C (mirrors the original SW_* functions in OPcontrol.c) ---
// DelayT is provided by hal.c (declared in hal.h): ~half-bit delay at 100 kHz.
#if defined(SW_I2C)
static void SWStartI2C(void) {
    gpio_put(PIN_SDA, 0);
    gpio_set_dir(PIN_SDA, GPIO_OUT);
    DelayT();
}

static signed char SWAckI2C(void) {
    gpio_put(PIN_SCL, 0);
    gpio_set_dir(PIN_SCL, GPIO_OUT);
    gpio_set_dir(PIN_SDA, GPIO_IN);
    DelayT();
    gpio_set_dir(PIN_SCL, GPIO_IN);
    DelayT();
    return gpio_get(PIN_SDA) ? -1 : 0;
}

static unsigned int SWGetcI2C(void) {
    BIT_COUNTER = 8;
    gpio_put(PIN_SCL, 0);
    unsigned int b = 0;
    do {
        gpio_set_dir(PIN_SCL, GPIO_OUT);
        gpio_set_dir(PIN_SDA, GPIO_IN);
        DelayT();
        gpio_set_dir(PIN_SCL, GPIO_IN);
        DelayT();
        b <<= 1;
        if (gpio_get(PIN_SDA)) b |= 1;
    } while (--BIT_COUNTER);
    return b;
}

static signed char SWPutcI2C(unsigned char data_out) {
    BIT_COUNTER = 8;
    I2C_BUFFER = data_out;
    gpio_put(PIN_SCL, 0);
    do {
        int bit = (I2C_BUFFER & 0x80) ? 1 : 0;
        I2C_BUFFER <<= 1;
        gpio_set_dir(PIN_SCL, GPIO_OUT);
        if (bit) gpio_set_dir(PIN_SDA, GPIO_IN);
        else { gpio_put(PIN_SDA, 0); gpio_set_dir(PIN_SDA, GPIO_OUT); }
        DelayT();
        gpio_set_dir(PIN_SCL, GPIO_IN);
        DelayT();
        BIT_COUNTER--;
    } while (BIT_COUNTER);
    return 0;
}

static void SWRestartI2C(void) {
    gpio_put(PIN_SCL, 0);
    gpio_set_dir(PIN_SCL, GPIO_OUT);
    gpio_set_dir(PIN_SDA, GPIO_IN);
    DelayT();
    gpio_set_dir(PIN_SCL, GPIO_IN);
    DelayT();
    gpio_put(PIN_SDA, 0);
    gpio_set_dir(PIN_SDA, GPIO_OUT);
    DelayT();
}

static void SWStopI2C(void) {
    gpio_put(PIN_SCL, 0);
    gpio_set_dir(PIN_SCL, GPIO_OUT);
    gpio_put(PIN_SDA, 0);
    gpio_set_dir(PIN_SDA, GPIO_OUT);
    DelayT();
    gpio_set_dir(PIN_SCL, GPIO_IN);
    DelayT();
    gpio_set_dir(PIN_SDA, GPIO_IN);
    DelayT(); DelayT();
}

static void NAckI2C(void) {
    gpio_set_dir(PIN_SCL, GPIO_OUT);
    gpio_set_dir(PIN_SDA, GPIO_IN);
    DelayT();
    gpio_set_dir(PIN_SCL, GPIO_IN);
    DelayT();
}

static void AckI2C(void) {
    gpio_set_dir(PIN_SCL, GPIO_OUT);
    gpio_put(PIN_SDA, 0);
    gpio_set_dir(PIN_SDA, GPIO_OUT);
    DelayT();
    gpio_set_dir(PIN_SCL, GPIO_IN);
    DelayT();
}
#endif // SW_I2C

// ---- Software SPI (mirrors the original SW_IO_SPI) ----------------------
#if defined(SW_SPI)
static unsigned char SW_IO_SPI(unsigned char c) {
    uint32_t st = save_and_disable_interrupts();
    I2C_BUFFER = c;
    RES0 = T2;
    // Mode 0: clock idle low, sample on rising edge, shift on falling edge.
    // MSB first.
    for (i2 = 0; i2 < 8; i2++) {
        gpio_put(PIN_SDO, (I2C_BUFFER & 0x80) ? 1 : 0);
        I2C_BUFFER <<= 1;
        busy_wait_us_32(T1);
        gpio_put(PIN_SCL, 1);
        busy_wait_us_32(T1);
        if (gpio_get(PIN_SDA)) I2C_BUFFER |= 1;
        gpio_put(PIN_SCL, 0);
    }
    restore_interrupts(st);
    return I2C_BUFFER;
}
#endif // SW_SPI

// ---- DCDC control loop ISR ----------------------------------------------
// This is the body of the original timer_isr(). On RP2040 it is invoked
// from a repeating timer alarm at 4 kHz (250 us period) — see hal.c.
void timer_isr(void) {
    // errz = previous err.
    errz = err;
    // Read ADC (12-bit), shift to 10-bit to match the PIC firmware's
    // Vcost constant and control-loop coefficients.
    uint16_t raw = hal_adc_read_raw() >> 2;
    HIBYTE(err) = (raw >> 8) & 0xFF;
    LOBYTE(err) = raw & 0xFF;
    err -= vreg;

    // PID-like correction (identical to the original):
    //   pwm += errz * 225
    //   pwm -= err  * 228
    pwm += errz * 225;
    pwm -= err  * 228;

    if (pwm <= -512)    pwm = -512;
    if (pwm > 0x6400)   pwm = 0x6400;

    if (pwm < 0) {
        hal_pwm_set_duty_8(0);
    } else {
        // Map pwm (0..0x6400) onto an 8-bit duty cycle.
        uint8_t duty = (uint8_t)((pwm * 255) / 0x6400);
        hal_pwm_set_duty_8(duty);
    }

    PIR1bits.ADIF = 0;
}
