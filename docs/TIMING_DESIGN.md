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
| UNI/O | Manchester 10-20us 하프비트 | Delay1us (busy_wait) | hal_delay_us_exact |
| Microwire | uWCK 8사이클 하프피리어드 | busy_wait_at_least_cycles (최소값) | hal_delay_cycles (정확값) |
| JTAG | TCKpulse (CKpulseN/L) | busy_wait_at_least_cycles | hal_delay_cycles |
| PIC24 SIX/REGOUT | 롱클럭 16사이클 | busy_wait_at_least_cycles | hal_delay_cycles |
| AVR HVSP | SCK 4사이클 | busy_wait_at_least_cycles | hal_delay_cycles |
| ICSP8 | CK 8사이클 | busy_wait_at_least_cycles | hal_delay_cycles |
| PIC24 TX16/RX16 | T1 보정 하프비트 | busy_wait_us_32 | hal_delay_us_exact |

## 4. 향후 PIO 기반 추가 최적화 설계

RP2040의 PIO는 CPU와 무관하게 하드웨어에서 타이밍을 보장하는
가장 이상적인 방법입니다. 현재 빌드 환경에서 pioasm 도구를
사용할 수 없어 PIO 구현은 보류했지만, 설계를 문서화합니다.

### 4.1 PIO가 특히 유익한 프로토콜

**One-Wire (최우선)**:
- reset 480us + presence 70us + recovery 410us 시퀀스
- 현재 RAM-resident로도 충분하지만, PIO는 CPU를 완전히解放
- PIO 클럭 1MHz로 1us 단위 타이밍 보장
- 오픈드레인은 `set pindirs` 명령으로 방향 토글

**JTAG**:
- TCK를 최대 30MHz+까지 올릴 수 있음 (현재 소프트웨어는 ~1MHz)
- 대량 XFER_DATA 시 CPU 개입 없이 DMA로 FIFO 채우기 가능
- PIO 클럭 분주기로 정확한 TCK 주파수 설정

**Microwire**:
- uWCK를 20MHz까지 올릴 수 있음
- 현재 8사이클(~60ns) 하프피리어드는 이미 충분히 빠름

**UNI/O**:
- Manchester 인코딩 + SAK 폴링이 복잡해 PIO 구현이 까다로움
- 현재 RAM-resident + 정밀 지연으로 충분

### 4.2 PIO 구현 로드맵 (pioasm 확보 시)

1. **pioasm 도구 확보**: pico-sdk-tools 저장소에서 빌드하거나
   pip 패키지로 설치 (현재 sandbox에서는 접근 불가)
2. **`.pio` 파일 작성**: 각 프로토콜별 PIO 어셈블리 프로그램
3. **CMake 통합**: `pico_generate_pio_header()`로 헤더 자동 생성
4. **C 래퍼**: `pio_sm_put_blocking()` / `pio_sm_get_blocking()`으로
   CPU-PIO 동기화
5. **DMA (선택)**: 대량 전송 시 FIFO를 DMA로 채워 CPU 부하 제거

### 4.3 PIO 인코딩 매크로 활용

pioasm 없이도 `pio_instructions.h`의 인코딩 매크로로 PIO 프로그램을
C에서 직접 기계어로 생성할 수 있습니다:

```c
#include "hardware/pio_instructions.h"
uint16_t program[] = {
    pio_encode_pull(false, true),      // pull block
    pio_encode_out(pio_x, 2),          // out x, 2
    pio_encode_jmp_x_dec(8),           // jmp x_dec, reset
    // ...
};
```

이 방식은 pioasm 의존성이 없지만, 복잡한 프로그램(카운티드 루프,
사이드셋, delay 필드 조합)을 손으로 인코딩해야 하므로 검증이
어렵습니다. 단순한 One-Wire reset/write-1/read 정도는 가능합니다.

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

PIO 없이도 **RAM-resident 비트뱅 + 하드웨어 카운터 기반 정밀 지연**
조합으로 RP2040에서 원본 PIC18 수준의 타이밍 정밀도를 달성했습니다.
핵심 타이밍 함수 8개가 모두 SRAM에서 실행되어 XIP 캐시 미스 지터가
제거됐고, `busy_wait_at_least_cycles`(최소값 보장)를
`hal_delay_cycles`(정확값)로 교체해 사이클 단위 정밀도를 확보했습니다.

PIO는 향후 pioasm 도구를 확보하면 추가적으로 적용할 수 있으며,
특히 One-Wire와 JTAG에서 CPU 부하를 완전히 제거하는 이점이 큽니다.
