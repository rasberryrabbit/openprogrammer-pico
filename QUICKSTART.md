# Open Programmer for Raspberry Pi Pico

Open Programmer 펌웨어의 라즈베리파이 피코(RP2040) 포팅입니다.
원본 PIC18 펌웨어 v0.12.0(Alberto Maccioni, GPL2)의 전체 명령 세트를 포팅했으며,
동일한 USB HID 와이어 프로토콜(VID 0x1209 / PID 0x5432)을 사용하므로
기존 openprog 호스트 소프트웨어(GUI/CLI)가 수정 없이 동작합니다.

## 빠른 시작 (펌웨어 설치)

가장 간단한 방법 — UF2 드래그 앤 드롭:

1. 피코의 **BOOTSEL** 버튼을 누른 채 USB로 PC에 연결
2. 피코가 `RPI-RP2` 대용량 저장장치로 마운트됨
3. `firmware/openprog_pico.uf2` 파일을 `RPI-RP2` 드라이브로 드래그하여 복사
4. 피코가 자동 재부팅되어 Open Programmer로 열거됨
5. 기존 openprog 호스트 소프트웨어 실행 → 장치 자동 인식

## 패키지 구조

```
OpenProgrammer_Pico/
├── README.md                  # 포팅 상세 문서
├── QUICKSTART.md              # 이 파일
├── CMakeLists.txt             # 빌드 설정
├── pico_sdk_import.cmake      # pico-sdk 임포트
├── include/                   # 헤더 파일
│   ├── hal.h                  # RP2040 HAL (GPIO/PWM/ADC/SFR shim)
│   ├── instructions.h         # 와이어 프로토콜 명령 정의
│   ├── opcontrol_helpers.h    # 비트 시프트 헬퍼 선언
│   ├── OPcontrol.h            # 명령 처리 함수 선언
│   ├── tusb_config.h          # TinyUSB 설정
│   └── usb_app.h              # USB 애플리케이션 인터페이스
├── src/                       # 소스 코드
│   ├── main.c                 # 진입점 + 메인 루프
│   ├── opcontrol.c            # 명령 파서 (91개 case, v0.12.0 전체 명령)
│   ├── opcontrol_helpers.c    # 비트 시프트 헬퍼 구현
│   ├── hal.c                  # HAL 구현 (DCDC 제어 루프 포함)
│   ├── usb_app.c              # HID 애플리케이션
│   └── usb_descriptors.c      # USB 디스크립터
└── firmware/                  # 빌드된 펌웨어 바이너리
    ├── openprog_pico.uf2      # 플래시용 (드래그 앤 드롭)
    ├── openprog_pico.elf      # 디버거용 (GDB/OpenOCD)
    ├── openprog_pico.hex      # 범용 플래시 포맷
    └── openprog_pico.bin      # Raw 바이너리
```

## 펌웨어 바이너리 정보

| 파일 | 크기 | 용도 |
|------|------|------|
| openprog_pico.uf2 | 145 KB | 피코 USB 드래그 앤 드롭 설치 |
| openprog_pico.elf | 111 KB | GDB/OpenOCD 디버깅 |
| openprog_pico.hex | 203 KB | 범용 플래시 포맷 |
| openprog_pico.bin | 73 KB | Raw 바이너리 |

메모리 사용량: text 73,784 B (72 KB) / data 0 / bss 3,672 B
→ RP2040의 2 MB 플래시 / 264 KB SRAM 대비 매우 여유

## 지원 디바이스 (v0.12.0 전체 명령 세트)

모든 명령이 포팅되었습니다. stub으로 남은 명령이 없습니다.

- **PIC10/12/16/18** (ICSP) — 코어/테이블/설정 명령 일체
- **PIC24/PIC33** (PIC24 ICSP) — SIX/REGOUT/TX16/RX16
- **8비트 PIC16F1xxx** (ICSP8) — SHORT/READ/LOAD
- **AVR** (HVSP 고전압 직렬) — AT_HV_RTX
- **JTAG** — SET_MODE/SEND_CMD/XFER_DATA/XFER_F_DATA
- **I2C/SPI EEPROM** — 소프트웨어 구현
- **Atmel DataFlash** — AT_READ_DATA/AT_LOAD_DATA
- **One-Wire** — OW_RESET/OW_WRITE/OW_READ
- **UNI/O** — UNIO_STBY/UNIO_COM
- **Microwire** — uW_INIT/uWTX/uWRX

## 소스에서 빌드하기

필요 도구:
- pico-sdk 1.5.1+
- ARM GCC (arm-none-eabi-gcc 10.3+)
- CMake 3.13+, Ninja

```bash
export PICO_SDK_PATH=/path/to/pico-sdk
export PATH=/path/to/arm-gnu-toolchain/bin:$PATH

cd OpenProgrammer_Pico
mkdir build && cd build
cmake -G Ninja -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

빌드 결과: `build/openprog_pico.uf2` (및 .elf/.hex/.bin)

## 하드웨어 주의사항

RP2040은 3.3V 디바이스이므로, 원본 Open Programmer 회로의
DCDC 컨버터(12V → Vpp 13V)와 전압 분배기, Vpp/Vcc 제어용
외부 트랜지스터/레귤레이터가 그대로 필요합니다.
핀 매핑은 `include/hal.h` 상단의 `PIN_*` 정의를 참고하세요.

## 라이선스

GPL2 — 원본 Open Programmer 펌웨어(Alberto Maccioni)를 따릅니다.
