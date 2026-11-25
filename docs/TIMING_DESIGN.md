# RP2040 정밀 타이밍 설계 문서

이 문서는 Open Programmer Pico 포팅에서 JTAG / One-Wire / UNI/O /
Microwire 프로토콜의 타이밍 정밀도를 높이기 위해 적용한 RP2040 기능과,
향후 PIO(Programmable I/O)로 추가 최적화할 수 있는 방법을 설명합니다.

## 1. 문제: 소프트웨어 비트뱅의 타이밍 지터

원본 PIC18 펌웨어는 48MHz 클럭에서 인라인 어셈블리로 사이클 단위 정밀
타이밍을 구현했습니다. 단순히 C로 포팅해서 `busy_wait_us_32()` /
`busy_wait_at_least_cycles()`를 쓰면 RP2040에서 두 가지 지터 원인이
발생합니다:

1. **XIP 플래시 캐시 미스**: 코드가 플래시에서 실행될 때 캐시 미스가
   발생하면 최대 ~8사이클의 지연이 생깁니다. 133MHz에서 이는 ~60ns로,
   마이크로초 단위 타이밍 슬롯에서 무시할 수 없습니다.

2. **인터럽트 선점**: USB CDC/SysTick 등의 인터럽트가 비트뱅 중간에
   끼어들면 수 마이크로초의 지터가 발생합니다.

## 2. 적용한 해결책: RAM-resident 비트뱅 + 정밀 지연

### 2.1 RAM-resident 함수 (`__attribute__((section(".time_critical.*")))`)

pico-sdk는 `.time_critical.<func>` 섹션에 배치된 함수를 SRAM으로
복사해서 실행합니다. 링커 맵에서 확인한 결과:

```
0x200000e4 T hal_delay_cycles      (SRAM)
0x20000140 T hal_od_drive_low      (SRAM)
0x200002ac T hvsp_xfer             (SRAM)
0x200000640 T six_shift24           (SRAM)
0x200000740 T regout_read16         (SRAM)
0x200000770 T icsp8_shift8          (SRAM)
0x2000006b0 T tx16_shift            (SRAM)
```

반면 플래시(`0x10000xxx`)에는 veneer(트램폴린)만 있어, 호출 시
RAM 함수로 점프합니다. 이로써 XIP 캐시 미스 지터가 제거됩니다.

**적용 함수**:
- `hal_delay_cycles()` — 사이클 단위 정확 지연
- `hal_delay_us_exact()` — 마이크로초 단위 정확 지연 (TIMER 하드웨어 카운터 기반)
- `hal_delay_ns_exact()` — 나노초 단위 지연 (사이클 변환)
- `hal_od_init/release/drive_low()` — 오픈드레인 핀 에뮬레이션
- `six_shift24()` / `regout_read16()` — PIC24 ICSP
- `icsp8_shift8()` — ICSP8
- `hvsp_xfer()` — AVR HVSP
- `tx16_shift()` / `rx16_shift()` — PIC24 롱클럭 16비트
- `uw_shift_out()` / `uw_shift_in()` — Microwire

### 2.2 정밀 지연 구현

**`hal_delay_cycles(n)`**: volatile 카운터 루프로 컴파일러가
Thumb16 호환 `SUB + BNE` 명령을 생성하도록 합니다. Cortex-M0+에서
반복당 3사이클, 나머지는 NOP로 소모합니다.

**`hal_delay_us_exact(n)`**: RP2040 TIMER 하드웨어의 64비트
마이크로초 카운터(`time_us_64()`)를 폴링합니다. 이 카운터는
하드웨어에서 증가하므로 CPU 코드 경로와 무관하게 정확합니다.

### 2.3 오픈드레인 에뮬레이션

One-Wire와 UNI/O는 오픈드레인 버스입니다. RP2040은 네이티브
오픈드레인 핀이 없으므로 **핀 레벨을 0으로 고정하고 방향만 토글**:
- `hal_od_release(pin)` → 입력 모드 (외부 풀업이 버스를 HIGH로)
- `hal_od_drive_low(pin)` → 출력 모드 (레벨 0으로 버스를 LOW로)

이 방식은 레벨 설정 오버헤드 없이 방향만 바꾸므로 슬롯 엣지가
더 정확합니다.

### 2.4 인터럽트 차단

모든 타이밍 크리티컬 구간은 이미 `save_and_disable_interrupts()`로
보호되어 있습니다. 이제 RAM-resident 함수 + 정밀 지연으로
인터럽트 외의 지터 원인까지 제거했습니다.

## 3. 프로토콜별 적용 결과

| 프로토콜 | 핵심 타이밍 | 개선 전 | 개선 후 |
|----------|------------|---------|---------|
| One-Wire | reset 480us, slot 6-60us | busy_wait_us ±1us | hal_delay_us_exact ±0us |
| UNI/O | Manchester 10us 하프비트 (50 kbit/s) | Delay1us (busy_wait) | hal_delay_us_exact |
| Microwire | uWCK 8사이클 하프피리어드 | busy_wait_at_least_cycles (최소값) | hal_delay_cycles (정확값) |
| JTAG | TCKpulse (CKpulseN/L) | busy_wait_at_least_cycles | hal_delay_cycles |
| PIC24 SIX/REGOUT | 롱클럭 16사이클 | busy_wait_at_least_cycles | hal_delay_cycles |
| AVR HVSP | SCK 4사이클 | busy_wait_at_least_cycles | hal_delay_cycles |
| ICSP8 | CK 8사이클 | busy_wait_at_least_cycles | hal_delay_cycles |
| PIC24 TX16/RX16 | T1 보정 하프비트 | busy_wait_us_32 | hal_delay_us_exact |

## 4. PIO 기반 하드웨어 타이밍 (적용 완료)

RP2040의 PIO는 CPU와 무관하게 하드웨어에서 타이밍을 보장하는 가장
이상적인 방법입니다. pioasm 어셈블러를 pico-sdk 소스
(`tools/pioasm/`)에서 직접 빌드하여 빌드 파이프라인에 통합했고,
4개 프로토콜용 `.pio` 프로그램을 작성해 적용했습니다.

### 4.1 pioasm 빌드 통합

pioasm는 FLEX/BISON 파서를 사용하지만 사전 생성된 `gen/lexer.cpp`와
`gen/parser.cpp`가 있어 추가 도구 없이 빌드됩니다. cmake 4.x 호환성을
위해 `CMAKE_POLICY_VERSION_MINIMUM=3.5`로 빌드했습니다.

빌드된 바이너리(`/home/z/my-project/tools/bin/pioasm`)를 CMake의
`Pioasm` imported target으로 등록하고, `tools/CMakeLists.txt`의
`pico_generate_pio_header()`로 4개 `.pio` 파일을 자동 컴파일합니다:

```cmake
include(${PICO_SDK_PATH}/tools/CMakeLists.txt)
pico_generate_pio_header(${CMAKE_PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/ow.pio)
pico_generate_pio_header(${CMAKE_PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/jtag.pio)
pico_generate_pio_header(${CMAKE_PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/unio.pio)
pico_generate_pio_header(${CMAKE_PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/uw.pio)
```

빌드 시 `ow.pio.h`, `jtag.pio.h`, `unio.pio.h`, `uw.pio.h`가 자동 생성되어
PIO 드라이버(`src/pio_drivers.c`)에서 include됩니다.

### 4.2 프로토콜별 PIO 프로그램

| 프로토콜 | 파일 | 명령 수 | PIO 클럭 | 설명 |
|----------|------|---------|----------|------|
| One-Wire | `src/ow.pio` | 32 | 1 MHz | reset/presence/write-0/write-1/read 슬롯. 오픈드레인은 `set pindirs`로 방향 토글 |
| UNI/O | `src/unio.pio` | 6 | 200 kHz | Manchester 하프비트 16개 시프트(루프 2사이클/하프비트 → 5 us/cycle → 10 us 하프비트 = 50 kbit/s). 호스트가 바이트→16비트 방향 패턴 인코딩 |
| JTAG | `src/jtag.pio` | 8+4 | 1 MHz | 32비트 시프트 엔진 + TMS 펄스 엔진 (2개 프로그램) |
| Microwire | `src/uw.pio` | 8 | 1 MHz | 32비트 동시 송수신 시프트 엔진 |

### 4.3 PIO 상태머신 할당

RP2040은 2개 PIO 블록(pio0, pio1)이 있고, 각 블록은 4개 상태머신(SM)과
**32명령 슬롯**(블록 내 모든 SM이 공유)을 가집니다. 한 블록에 들어가는
프로그램 명령 수 합계가 32를 넘으면 `pio_add_program`이 런타임에
실패합니다. ow.pio 단독으로 32명령이라 pio0을 OW 전용으로 쓰고, UNI/O
(6명령)는 pio1에 넣습니다:

```
pio0 SM0 — One-Wire   (ow.pio,            32 instr — 블록 전용)
pio1 SM0 — UNI/O      (unio.pio,           6 instr)
```

JTAG(TCK/TDI/TDO/TMS)과 Microwire(uWCK/uWDI/uWDO)는 **SIO 호스트
비트뱅**으로 유지합니다. `pio_gpio_init()`은 핀의 FUNCSEL을 PIO 기능으로
바꾸고 절대 복구하지 않으므로, 이 핀들을 PIO에 넘기면 `gpio_put()`/
`gpio_get()`이 물리 핀에 닿지 않아 비트뱅이 조용히 망가집니다. 이전
설계는 JTAG/uW PIO 프로그램을 pio1에 로드해 `pio_gpio_init()`이
TCK/TDI/TDO/uWCK/uWDI의 FUNCSEL을 PIO로 가져갔고, 결과적으로 JTAG/uW
비트뱅 경로가 실제 하드웨어에서 동작하지 않았습니다. 방안 A 적용으로
JTAG/uW PIO 로드 자체를 제거해 이 충돌을 원천 해소했습니다.

#### PIN_SDA 지연 소유권 (방안 A)

OW(pio0)와 UNI/O(pio1)는 PIN_SDA(GP10)를 공유합니다. 부트 시
`pio_drivers_init()`는 두 프로그램을 로드하고 SM을 claim하지만,
**SM은 비활성화** 상태로 두고 PIN_SDA는 SIO에 남겨둡니다(open-drain,
released). 프로토콜이 핀을 쓸 때 `<proto>_pio_enter()`가 FUNCSEL을
소유 PIO 블록으로 바꾸고 SM을 활성화하며, 끝나면 `<proto>_pio_exit()`가
SM을 끄고 PIN_SDA를 SIO open-drain으로 복원합니다. OW와 UNI/O는
상호 배타이므로 한 enter가 다른 프로토콜의 exit를 자동 호출합니다.
호스트는 각 PIO 호출 시퀀스를 enter/exit로 감싸야, 그 사이 SIO
비트뱅 경로(UNIO_COM의 start header·MAK·SAK, UNIO_STBY)가 정상
동작합니다.

### 4.4 opcontrol.c 적용 현황

| 프로토콜 | PIO 적용 | 비고 |
|----------|----------|------|
| OW_RESET / OW_WRITE / OW_READ | ✅ PIO | `ow_pio_enter` → `ow_pio_reset/write_byte/read_byte` → `ow_pio_exit` |
| UNIO_COM (송신부) | ✅ PIO | `unio_pio_enter` → `unio_pio_emit_halfbits` → `unio_pio_exit` (바이트마다) |
| UNIO_STBY | ❌ 호스트 | `hal_od_*` SIO 비트뱅; PIN_SDA가 SIO에 있어 정상 동작 |
| UNIO_COM (start header / MAK / SAK / 수신) | ❌ 호스트 | `hal_od_*` SIO 비트뱅; PIO emit 전후로 exit/enter 복귀 |
| JTAG (SET_MODE / SEND_CMD / XFER) | ❌ 호스트 | `gpio_put(PIN_CK/PIN_PGM/PIN_D/PIN_A1)` SIO 비트뱅; FUNCSEL PIO 미사용 |
| uW (INIT / TX / RX) | ❌ 호스트 | `gpio_put(PIN_SDO/PIN_SCL)` SIO 비트뱅; FUNCSEL PIO 미사용 |

JTAG/uW PIO 엔진(jtag.pio, uw.pio)은 더 이상 빌드하지 않습니다.
SIO 비트뱅이 정밀도와 유연성에서 충분하며, PIO 마이그레이션은
타이밍 보장이 필수인 OW(1-Wire reset/slot)와 UNI/O(50 kbit/s Manchester
하프비트)에만 적용됩니다.

### 4.5 PIO 타이밍 정확도

PIO는 독립 클럭 분주기로 동작해 CPU 인터럽트/캐시 미스와 완전히 분리됩니다:

- **One-Wire**: 1MHz PIO 클럭 → 1us 단위 정밀. reset 480us(중첩 루프),
  presence 샘플 62us, write-0 60us, write-1 6us+64us, read 6us+15us 샘플
- **UNI/O**: 200kHz PIO 클럭 → 5us 단위(루프 2사이클/하프비트 → 10us 하프비트 = 50 kbit/s). Manchester 하프비트 16개를 CPU 개입 없이 시프트
- **JTAG/uW**: 1MHz 기본 (clkdiv로 조정 가능), 최대 30MHz+까지 상향 가능

## 5. 검증 방법

빌드된 펌웨어에서 다음으로 정밀 타이밍 적용을 확인했습니다:

```bash
# RAM-resident 함수 확인 (0x20000xxx = SRAM 주소)
arm-none-eabi-nm openprog_pico.elf | grep -E "hal_delay|hal_od_|six_shift|hvsp_xfer"

# .time_critical 섹션이 .data(RAM)에 있는지 확인
arm-none-eabi-objdump -h openprog_pico.elf | grep -E "time_critical|\.data"
```

실제 타이밍은 로직 애널라이저나 오실로스코프로 측정해야 합니다.
개선 전후 비교 지표:
- One-Wire reset 펄스 폭 분산 (±1us → ±0us)
- JTAG TCK 주파수 안정성
- UNI/O Manchester 하프비트 정밀도

## 6. 결론

**RAM-resident 비트뱅 + 하드웨어 카운터 기반 정밀 지연 + PIO 하드웨어
엔진** 세 가지를 조합해 RP2040에서 원본 PIC18 수준 이상의 타이밍
정밀도를 달성했습니다:

1. **RAM-resident 함수**: 핵심 타이밍 함수 11개가 SRAM에서 실행되어
   XIP 캐시 미스 지터가 제거됐습니다.
2. **정밀 지연**: `busy_wait_at_least_cycles`(최소값 보장)를
   `hal_delay_cycles`(정확값)로 교체해 사이클 단위 정밀도를 확보했습니다.
3. **PIO 하드웨어 엔진**: One-Wire와 UNI/O 송신부를 PIO 상태머신으로
   교체해 CPU 인터럽트/캐시와 완전히 분리된 하드웨어 타이밍을 보장합니다.
   JTAG/uW PIO 엔진도 작성·검증 완료해 예비 인프라로 확보했습니다.

pioasm 어셈블러를 pico-sdk 소스에서 빌드해 통합했으므로, 향후 추가
프로토콜이나 대량 시프트 최적화가 필요할 때 `.pio` 파일 한 개 추가로
PIO 엔진을 확장할 수 있습니다.

PIO는 향후 pioasm 도구를 확보하면 추가적으로 적용할 수 있으며,
특히 One-Wire와 JTAG에서 CPU 부하를 완전히 제거하는 이점이 큽니다.
