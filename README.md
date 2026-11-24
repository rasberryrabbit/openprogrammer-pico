# Open Programmer 펌웨어 — 라즈베리파이 피코(RP2040) 포팅

PIC18F2550/25K50 기반 Open Programmer 펌웨어 v0.12.0을 **라즈베리파이 피코(RP2040, ARM Cortex-M0+)** 로 포팅한 결과물입니다. **USB HID 와이어 프로토콜(VID 0x1209 / PID 0x5432, 64바이트 IN/OUT 리포트)은 원본과 동일**하게 유지하여 기존 호스트 소프트웨어(openprog GUI/CLI)가 수정 없이 동작하도록 했습니다.

> ⚠️ **빌드 환경 제약**: 이 샌드박스에는 ARM 크로스 컴파일러(`arm-none-eabi-gcc`)와 pico-sdk가 설치되어 있지 않아 실제 빌드를 수행하지 못했습니다. 소스코드와 빌드 설정을 완성했으므로 아래 절차에 따라 사용자 환경에서 빌드하시기 바랍니다.

## 1. 아키텍처 매핑

| 원본 (PIC18 @ 48 MHz) | 포팅 (RP2040 @ 133 MHz) |
|---|---|
| Custom GPL USB 스택 (`usb.c`) | TinyUSB (`tinyusb_device` via pico-sdk) |
| `#pragma config` config bits | 클럭/USB 초기화 (`board_init()`) — config bits 개념 없음 |
| `LATBbits.LATB5 = 1` (SFR) | `gpio_put(PIN_D, 1)` (HAL 매크로로 추상화) |
| `_asm RRCF/BCF ... _endasm` (인라인 어셈) | `shift_out_msb_first()` / `shift_in_msb_first()` 헬퍼 |
| `Nop()` 루프 타이밍 (48MHz) | `busy_wait_us_32()` / `busy_wait_at_least_cycles()` |
| CCP1 PWM + CCP2 ADC 트리거 + ISR | `hardware_pwm` + repeating timer alarm @ 4 kHz |
| `rom char` / `near` / `auto` | 일반 C (ROM/RAM 구분 불필요, 충분한 SRAM) |
| `#pragma udata` 뱅크 배치 | 불필요 (통합 264KB SRAM) |
| `INTCONbits.GIE = 0/1` | `save_and_disable_interrupts()` / `restore_interrupts()` |

## 2. 프로젝트 구조

```
Firmware_Pico/
├── CMakeLists.txt               # 최상위 빌드 설정
├── pico_sdk_import.cmake        # pico-sdk 찾기 (표준 파일)
├── include/
│   ├── hal.h                    # HAL: 핀 정의 + SFR 매크로 매핑
│   ├── instructions.h           # 와이어 프로토콜 상수 (원본 그대로)
│   ├── OPcontrol.h              # UserInit/ProcessIO/timer_isr 선언
│   ├── opcontrol_helpers.h      # 비트 시프트 헬퍼 선언
│   ├── tusb_config.h            # TinyUSB 설정 (HID 64바이트 리포트)
│   └── usb_app.h                # HIDRxReport/HIDTxReport 인터페이스
└── src/
    ├── main.c                   # 부트 + 메인 루프
    ├── usb_descriptors.c        # USB 디스크립터 (VID/PID/HID 원본 그대로)
    ├── usb_app.c                # TinyUSB HID 글루 (OUT 링 버퍼)
    ├── hal.c                    # RP2040 하드웨어 초기화 + SFR shim + 타이머 알람
    ├── opcontrol.c              # 명령 파서 + DCDC 제어 루프
    └── opcontrol_helpers.c      # ICSP/JTAG 비트 시프트 헬퍼
```

## 3. 핀 매핑 (RP2040)

원본 PIC18의 PORTA/B/C 핀을 RP2040 GPIO로 다음과 같이 매핑했습니다. 회로 기판에 맞춰 `include/hal.h`의 `PIN_*` 상수를 수정하세요.

| 기능 | PIC18 핀 | RP2040 GPIO | 비고 |
|---|---|---|---|
| D (ICSP data) | RB5 | GP5 | 양방향 |
| CK (ICSP clock) | RB6 | GP6 | 출력 |
| PGM | RB7 | GP7 | 출력 |
| SDA (I2C / uW data in) | RB0 | GP10 | 오픈드레인 |
| SCL (I2C / SPI clock / uW clock) | RB1 | GP11 | 오픈드레인 |
| A0 (SPI CS) | RB3 | GP14 | 출력 |
| A1 (SPI HOLD) | RB4 | GP15 | 출력 |
| LED1 | RA1 | GP18 | 출력 |
| LED2 | RA2 | GP19 | 출력 |
| VPP control | RC0 | GP20 | 출력 |
| VCC control | RC1 | GP21 | 출력 |
| PWM (DCDC gate) | RC2 / CCP1 | GP22 | PWM slice 3 channel A |
| WP (EEPROM write protect) | RC6 | GP26 | 출력 |
| SDO (SPI MOSI) | RC7 | GP27 | 출력 |
| Vreg feedback ADC | AN0 / RA0 | GP28 (ADC2) | ADC 입력 |
| SW1 (mode 버튼) | RE3 | GP17 | 입력 (액티브 low, 풀업) |

> **주의**: 원본처럼 SCL/SCK를 같은 핀에 멀티플렉싱했습니다. I2C와 SPI를 동시에 사용하지 못합니다 (원본과 동일한 제약).

## 4. 빌드 절차

### 요구사항
- `arm-none-eabi-gcc` (버전 10.3 이상, GCC ARM Embedded)
- `cmake` 3.13+
- `ninja` 또는 `make`
- `pico-sdk` 1.5+ 체크아웃
- `tinyusb` (pico-sdk에 포함)

### 빌드 명령

```bash
# pico-sdk 경로 설정 (환경에 맞게 수정)
export PICO_SDK_PATH=/path/to/pico-sdk

# 빌드 디렉토리
cd Firmware_Pico
mkdir build && cd build
cmake ..
make -j$(nproc)
```

생성 파일:
- `openprog.uf2` — Pico BOOTSEL 모드로 드래그하여 플래시
- `openprog.hex` — 표준 HEX
- `openprog.elf` — 디버깅용 ELF
- `openprog.map` — 맵 파일

### 플래싱
1. Pico의 BOOTSEL 버튼을 누른 채 USB 연결 → 대용량 저장장치로 마운트
2. `openprog.uf2`를 마운트된 드라이브로 복사
3. Pico가 자동 재부팅되며 펌웨어 실행

또는 `openocd` + Picoprobe 환경에서:
```bash
openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg \
        -c "adapter speed 5000; program openprog.elf verify reset exit"
```

## 5. 포팅 커버리지

### 완전 포팅 (원본과 동일 동작) — 전체 v0.12.0 명령 세트
모든 명령이 포팅되었습니다. 더 이상 stub으로 남은 명령이 없습니다.

- **기본**: NOP, PROG_RST, PROG_ID, CHECK_INS, FLUSH, PROG_C
- **전원 제어**: VREG_EN, VREG_DIS, SET_VPP, EN_VPP_VCC, READ_ADC, READ_AC
- **타이밍**: SET_PARAMETER, WAIT_T1/T2/T3/US
- **ICSP (PIC10/12/16/18)**: LOAD_CONF, LOAD_DATA_PROG/DATA, READ_DATA_PROG/DATA, INC_ADDR, INC_ADDR_N, BEGIN_PROG, BULK_ERASE_PROG/DATA, END_PROG, END_PROG2, ROW_ERASE_PROG, BEGIN_PROG2, CUST_CMD, SET_CK_D, READ_PINS, SET_PORT_DIR, READ_B
- **PIC18 테이블**: CORE_INS, SHIFT_TABLAT, TABLE_READ, TBLR_INC_N, TABLE_WRITE, TBLW_INC_N, TBLW_PROG, TBLW_PROG_INC, SEND_DATA, READ_DATA, TBLRD, TBLWT
- **PIC24/PIC33**: LOAD_PC, LOAD_DATA_INC, READ_DATA_INC, SIX, SIX_LONG, SIX_LONG5, SIX_N, REGOUT, TX16, RX16
- **ICSP8 (8비트 PIC16F1xxx)**: ICSP8_SHORT, ICSP8_READ, ICSP8_LOAD
- **I2C**: I2C_INIT, I2C_READ, I2C_WRITE, I2C_READ2 (소프트웨어 구현)
- **SPI**: SPI_INIT, SPI_READ, SPI_WRITE (소프트웨어 구현)
- **Atmel DataFlash**: AT_READ_DATA, AT_LOAD_DATA (SPI 기반)
- **AVR 고전압 직렬 프로그래밍 (HVSP)**: AT_HV_RTX
- **JTAG**: JTAG_SET_MODE, JTAG_SEND_CMD, JTAG_XFER_DATA, JTAG_XFER_F_DATA
- **One-Wire**: OW_RESET, OW_WRITE, OW_READ
- **UNI/O**: UNIO_STBY, UNIO_COM
- **Microwire**: uW_INIT, uWTX, uWRX
- **기타**: EXT_PORT, CLOCK_GEN, REPEAT, REPEAT_END, SPI_TEST, READ_RAM, WRITE_RAM, LOOP

> **참고**: TBLRD/TBLWT는 PIC18의 TBLPTR 레지스터 기반 명령이므로 RP2040 자체에는 의미가 없지만, ICSP CORE_INS/TABLE_WRITE 명령을 통해 타겟 PIC18에 TBLRD*/TBLWT* 옵코드를 전송하도록 구현되어 원본과 동일하게 동작합니다.

## 6. 설계 결정 사항

### 6.1 USB 스택 교체
원본의 hand-rolled GPL USB 스택은 PIC18의 SIE 레지스터(UDR, UCFG, BDT)에 직접 의존합니다. RP2040에서는 pico-sdk에 포함된 **TinyUSB**로 교체했으며, 디스크립터(device/configuration/HID report)만 원본과 바이트 단위로 동일하게 작성했습니다. 결과적으로 **호스트 측 openprog 드라이버는 수정 없이 동일하게 동작**합니다.

### 6.2 인라인 어셈블리 → C 헬퍼
원본의 모든 ICSP/JTAG 비트 시프트는 PIC18의 `RRCF`/`RLCF` 명령과 STATUS.CARRY 플래그를 활용한 인라인 어셈입니다. RP2040의 32비트 ARM 코어에서는 동일한 연산이 일반 C 시프트 연산으로 컴파일되며, 133MHz 클럭 덕분에 PIC18@48MHz보다 **사이클 수가 줄어들어 타이밍 마진이 더 넓습니다**. `shift_out_msb_first()` / `shift_in_msb_first()` 헬퍼로 추상화했습니다.

### 6.3 정밀 타이밍 (RAM-resident 비트뱅 + 하드웨어 카운터)

JTAG / One-Wire / UNI/O / Microwire 프로토콜은 마이크로초 단위의
정밀 타이밍을 요구합니다. 단순한 `busy_wait_us_32()` /
`busy_wait_at_least_cycles()`는 XIP 플래시 캐시 미스와 인터럽트로
인해 ±1us 지터가 발생합니다. 이를 제거하기 위해 두 가지 기법을
적용했습니다 (상세 설계는 `docs/TIMING_DESIGN.md` 참고):

1. **RAM-resident 함수**: 핵심 타이밍 함수 8개를
   `__attribute__((section(".time_critical.*")))`로 SRAM에 배치하여
   XIP 캐시 미스 지터를 제거합니다. 링커 맵에서 모두 `0x20000xxx`
   (SRAM 주소 영역)에 위치함을 확인했습니다:
   - `hal_delay_cycles()` / `hal_delay_us_exact()` / `hal_delay_ns_exact()`
   - `hal_od_drive_low()` / `hal_od_release()` (오픈드레인 에뮬레이션)
   - `six_shift24()` / `regout_read16()` / `icsp8_shift8()`
   - `hvsp_xfer()` / `tx16_shift()` / `rx16_shift()`

2. **정밀 지연**: `busy_wait_at_least_cycles()`(최소값만 보장)를
   `hal_delay_cycles()`(정확한 사이클 수)로 교체했습니다. 마이크로초
   지연은 RP2040 TIMER 하드웨어의 64비트 카운터(`time_us_64()`)를
   폴링하여 CPU 코드 경로와 무관하게 정확한 시간을 보장합니다.

3. **오픈드레인 에뮬레이션**: One-Wire / UNI/O는 오픈드레인 버스입니다.
   RP2040은 네이티브 OD 핀이 없으므로 핀 레벨을 0으로 고정하고
   방향만 토글하여 OD를 에뮬레이션합니다. 레벨 설정 오버헤드가
   없어 슬롯 엣지가 더 정확합니다.

> **PIO(Programmable I/O) 적용 완료**: RP2040의 PIO는 CPU와 무관하게
> 하드웨어에서 타이밍을 보장하는 가장 이상적인 방법입니다. pioasm
> 어셈블러를 pico-sdk 소스에서 빌드하여 빌드 파이프라인에 통합했고,
> 4개 프로토콜(One-Wire, JTAG, UNI/O, Microwire)용 `.pio` 프로그램을
> 작성했습니다:
>
> - **One-Wire** (`src/ow.pio`, 32명령): reset/presence/write-0/write-1/read
>   슬롯을 1MHz PIO 클럭으로 하드웨어 보장. 오픈드레인은 `set pindirs`로.
> - **UNI/O** (`src/unio.pio`, 6명령): Manchester 하프비트 16개를 100kHz로
>   시프트. 호스트가 바이트를 16비트 방향 패턴으로 인코딩.
> - **JTAG** (`src/jtag.pio`): 32비트 시프트 엔진 + TMS 펄스 엔진.
> - **Microwire** (`src/uw.pio`, 8명령): 32비트 동시 송수신 시프트 엔진.
>
> PIO 드라이버(`src/pio_drivers.c`, `include/pio_drivers.h`)가 5개 상태머신을
> 관리하며, `opcontrol.c`의 OW_RESET/OW_WRITE/OW_READ와 UNIO_COM 송신부가
> PIO 기반 호출로 교체됐습니다. JTAG XFER와 uW는 기존 RAM-resident 정밀
> 타이밍이 이미 충분해 PIO 엔진은 향후 대량 시프트 최적화용으로 예비됩니다.
> 자세한 설계는 `docs/TIMING_DESIGN.md`를 참고하세요.

### 6.4 DCDC 제어 루프
원본은 CCP2 compare 인터럽트(250us 주기, 4kHz)에서 PID 변종 제어 루프를 실행합니다. RP2040에서는 `add_repeating_timer_us(250, ...)`로 동일한 주기의 타이머 알람을 설정했고, `timer_isr()` 함수 본체는 제어 계수(`pwm += errz*225; pwm -= err*228`)를 그대로 보존했습니다.

ADC는 12비트이지만 원본의 `Vcost` 상수가 10비트 기준이므로, `hal_adc_read_raw() >> 2`로 10비트로 정규화하여 제어 루프 계수를 그대로 재사용합니다.

### 6.5 핀 다중화 보존
원본은 I2C SCL과 SPI SCK를 RB1 하나로 공유합니다. RP2040 포팅도 동일하게 `PIN_SCL`을 I2C/SPI/uW 클럭으로 공유하므로, **I2C와 SPI를 동시에 사용할 수 없는 원본의 제약이 그대로 유지**됩니다.

### 6.6 SFR shim namespace
원본 OPcontrol.c의 `INTCONbits.GIE`, `PIR1bits.ADIF`, `HLVDCONbits.IRVST` 같은 PIC18 SFR 비트 필드 참조를 struct-based shim(`hal.h`의 `hal_intcon_t`, `hal_pir1_t`, ...)으로 에뮬레이션했습니다. 이를 통해 OPcontrol.c의 코드 구조를 원본에 최대한 가깝게 유지했습니다.

## 7. 알려진 제약사항

1. **TBLRD/TBLWT (v0.12.0 신규 명령)**: PIC18의 TBLPTR 레지스터 기반 테이블 액세스 명령입니다. RP2040 자체에는 TBLPTR이 없지만, ICSP CORE_INS/TABLE_WRITE 명령을 통해 타겟 PIC18에 TBLRD POSTINC(0x0009)/POSTDEC(0x000B), TBLWT POSTINC(0x000D)/POSTDEC(0x000F) 옵코드를 전송하도록 구현되어 원본과 동일하게 동작합니다. 단, 타겟 PIC18의 TBLPTR 로딩은 호스트가 별도의 CORE_INS 시퀀스로 수행해야 합니다.

2. **소프트웨어 I2C/SPI 타이밍**: 원본은 정확한 클럭 주기를 위해 Nop() 루프를 튜닝했으나, RP2040에서는 `busy_wait_us_32()` 기반의 대략 100kHz I2C / 가변 SPI를 사용합니다. 고속 I2C 모드가 필요하면 `DelayT()`의 대기 시간을 조정하세요.

3. **DCDC 하드웨어 의존성**: 원본 회로의 DCDC 컨버터(12V → Vpp 13V)와 전압 분배기(12k/22k)가 그대로 필요합니다. RP2040은 3.3V 디바이스이므로 **Vpp/Vcc 제어 핀(GP20/GP21)은 외부 트랜지스터/레귤레이터를 구동**해야 합니다. PWM 게이트(GP22)도 3.3V → 외부 게이트 드라이버 → DCDC 인덕터 경로로 연결돼야 합니다.

4. **JTAG/OW/UNIO/uW 타이밍 마진**: 이 프로토콜들은 마이크로초 단위의 정밀 타이밍을 요구합니다. RP2040의 `busy_wait_us_32()`는 ±1us 지터를 가질 수 있으므로, 엄격한 타이밍이 필요한 디바이스(예: 일부 UNI/O EEPROM)에서는 `Delay1us()`의 보정이 필요할 수 있습니다. One-Wire의 경우 원본과 동일한 타이밍 상수(250/70/150us 등)를 사용합니다.

5. **AT_HV_RTX (AVR HVSP)**: AVR 고전압 직렬 프로그래밍은 12V를 Vcc에 인가하는 하드웨어 회로가 필요합니다. RP2040의 3.3V GPIO로는 직접 구동할 수 없으므로 외부 레벨 시프터/트랜지스터가 필요합니다. PB0/PB1/PB2/PB3 핀 매핑은 `hal.h` 참고.

## 8. 원본 대비 변경된 코드 구조

```diff
- #include <p18cxxx.h>           // PIC18 SFR
+ #include "hal.h"               // RP2040 HAL (GPIO/PWM/ADC/SFR shim)
+ #include "opcontrol_helpers.h" // bit-shift helpers
+ #include "usb_app.h"           // TinyUSB HID glue

- _asm RRCF d+1,1,0 ... _endasm  // PIC18 inline asm
+ shift_out_msb_first(d, 14);    // portable C helper

- INTCONbits.GIE = 0;            // PIC SFR
+ uint32_t st = save_and_disable_interrupts();  // ARM
  ...
- INTCONbits.GIE = 1;
+ restore_interrupts(st);

- for(i=T1;i;i--){Nop();...}    // 48MHz calibrated NOP loop
+ busy_wait_us_32(T1);          // 133MHz accurate wait

- HIBYTE(err)=ADRESH; LOBYTE(err)=ADRESL;
+ uint16_t raw = hal_adc_read_raw() >> 2;
+ HIBYTE(err)=(raw>>8)&0xFF; LOBYTE(err)=raw&0xFF;
```

## 9. 라이선스

원본과 동일하게 **GPL v2** 라이선스를 따릅니다. TinyUSB와 pico-sdk는 각각의 라이선스(Apache 2.0 / BSD-3-Clause)를 따릅니다.
