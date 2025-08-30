# System Protocol and Wiring

This document captures the exact data framing, pin mappings, and communication settings between the STM32F103C8T6 scanner MCU and the ESP32‑C3 transport, plus notes for the PC visualizer.

## Overview

- Sensor matrix: 32 rows × 64 columns (2048 samples per frame)
- Scanner: STM32F103C8T6 reads the matrix via ADG732 analog multiplexers into ADC1 (PA0)
- Transport: ESP32‑C3 (master) reads frames over SPI from the STM32 (slave) and broadcasts over UDP
- Visualizer: Python script (`software/viz_full_4.py`) assembles 10 UDP chunks into a 32×64 frame

## Data flow and timing

1) STM32 continuously scans the 32×64 matrix into a double buffer in RAM
- Buffer shape: uint16[32][64] (little‑endian in memory)
- Scan timing: address select → delay_us(DELAY_MicroSec) → ADC read (ADC1_IN0)
- Default delay: `#define DELAY_MicroSec 10`

2) ESP32 performs one SPI burst read per frame
- Reads 2048 × 16‑bit words using 2048 calls to `SPI.transfer16(0x0000)`
- SPI transaction speed for frame: 400 kHz, MSB first, MODE 0
- Chip Select is asserted low for the whole burst

3) ESP32 sends the frame in 10 UDP packets
- Each packet carries 205 values (205×2 = 410 bytes) with start/end markers
- 10 chunks → 2050 values; visualizer uses the first 2048 to form a frame

## SPI link details (ESP32 master → STM32 slave)

- Mode: SPI Mode 0 (CPOL=0, CPHA=0)
- Bit order: MSB first on ESP32
- Word size on STM32: 8‑bit (HAL config), bytes stream out in memory order
- Clock: 400 kHz for frame reads (IMU test uses 4 MHz on same bus with a different CS)
- Transaction length: 4096 bytes per frame (2048 × uint16)
- Burst framing: ESP32 asserts CS low before the burst and high after

Endianness note:
- STM32 stores uint16 in little‑endian; HAL SPI streams low byte then high byte from memory
- ESP32 `transfer16` with MSB first assembles a 16‑bit word with the first received byte as MSB
- The PC visualizer auto‑detects endianness; recommended setting is little‑endian (LE)

## UDP frame format (ESP32 → PC)

Per‑packet payload is 415 bytes:
- [0]  0xAA
- [1]  0x55
- [2]  chunk_index in 0..9
- [3..412] 410 bytes = 205 × uint16 values (LSB first per value)
- [413] 0x55
- [414] 0xAA

Per‑frame:
- 10 packets (indices 0..9). Concatenate in index order; take first 2048 values → reshape 32×64
- Default UDP dest: IP set in `firmware/esp32/ESP32_C3_Zero_SPI_2048_UDP.ino` via `udpAddress`, port 12345
- Visualizer bind defaults: `--ip 0.0.0.0 --port 12345` (overridable)

## ESP32‑C3 pins (Waveshare ESP32‑C3 Zero style)

Defined in `firmware/esp32/ESP32_C3_Zero_SPI_2048_UDP.ino`:
- SPI SCK: GPIO 2 (`PIN_CLK`)
- SPI MISO: GPIO 3 (`PIN_MISO`)
- SPI MOSI: GPIO 4 (`PIN_MOSI`)
- STM32 CS/NSS: GPIO 1 (`PIN_CS`)
- IMU CS (ICM‑20948 module): GPIO 5 (`PIN_IMU_CS`)

Wi‑Fi/UDP:
- SSID: `uofrGuest` (password empty) [edit as needed]
- UDP destination: `udpAddress` (your PC’s IP), `udpPort = 12345`

SPI transactions on ESP32:
- Frame read: `SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0))`
- IMU WHO_AM_I read (same bus): `SPISettings(4000000, MSBFIRST, SPI_MODE0)` with CS on GPIO 5

## STM32F103C8T6 pins and peripherals

SPI1 (slave) – from `Core/Src/stm32f1xx_hal_msp.c`:
- NSS:  PA4 (input)
- SCK:  PA5 (input)
- MISO: PA6 (AF push‑pull output)
- MOSI: PA7 (input)

ADC1:
- Channel: ADC1_IN0 on PA0 (analog)
- Sampling time: 1.5 cycles (CubeMX init); conversions triggered in software

UART1 (debug/log):
- TX: PA9, RX: PA10
- CubeMX default baud: 115200; custom `USART1_Init()` sets 921600 (8N1)

GPIOs to ADG732 multiplexers (control and enables) – defined in `Core/Src/main.c`:
- Address lines (A0..A4) on GPIOB:
  - A0 = PB4, A1 = PB3, A2 = PB15, A3 = PB14, A4 = PB13
- Write and enable strobes on GPIOB:
  - WR = PB10, EN = PB1 (active‑low enable logic in code; default EN reset)
- Row side selects:
  - ROW_SEL_L = PB8, ROW_SEL_R = PB9
- Column enable selects (Left half):
  - CON_SEL_L_1 = PB11, CON_SEL_L_2 = PB12, CON_SEL_L_3 = PB6, CON_SEL_L_4 = PA1
- Column enable selects (Right half):
  - CON_SEL_R_1 = PB0, CON_SEL_R_2 = PB5, CON_SEL_R_3 = PB7, CON_SEL_R_4 = PA2

These pins are initialized in `ADG732_Init()` with GPIOB and GPIOA clocks enabled.

Row/column mapping helpers:
- Columns are remapped with `MapColumnIndex(col)` to hardware order (handles mixed wiring)
- Rows use `MapRowAddrForSide(row, col)` inside `ADG732_SelectChannel2()` to compute the ADG732 row address per half (left/right)

Scanning sequence (per sample):
1) Assert appropriate ROW side (left/right), set mapped row address, strobe WR
2) Set column address, strobe WR
3) Assert the single column‑enable for the current side and row‑group (8‑row groups)
4) Delay `DELAY_MicroSec` and read ADC1

## Double‑buffered SPI streaming (STM32)

- Two frame buffers: `bufA`, `bufB` (uint16[32][64])
- ISR (`HAL_SPI_TxRxCpltCallback`): re‑arms SPI immediately; swaps TX/fill buffers only when `frame_ready` is set by the scanner loop
- Main loop: continuously scans into the fill buffer and then sets `frame_ready = 1`
- SPI is configured as 8‑bit 2‑line slave with NSS hardware input; TX length per burst is 4096 bytes

## Visualizer expectations

- Binds UDP on configurable IP/port (defaults `0.0.0.0:12345`)
- Accepts 415‑byte packets described above; assembles 10 chunks into a frame
- Endianness auto‑detected from plausible ranges (0..4095). You can force LE/BE inside `viz_full_4.py`
- Optional geometry fixes: the script can mirror each 32‑column half independently to match physical layout

## Wiring summary (inter‑board)

ESP32‑C3 ↔ STM32F103C8T6 SPI:
- ESP32 GPIO2 (SCK) → STM32 PA5 (SPI1_SCK)
- ESP32 GPIO3 (MISO) ← STM32 PA6 (SPI1_MISO)
- ESP32 GPIO4 (MOSI) → STM32 PA7 (SPI1_MOSI)
- ESP32 GPIO1 (CS)   → STM32 PA4 (SPI1_NSS)
  - Shared ground and 3V3 compatible logic levels required

ESP32‑C3 ↔ IMU (ICM‑20948 module):
- Shared SCK/MISO/MOSI; dedicated CS on ESP32 GPIO5

STM32F103C8T6 → Sensor matrix:
- ADC input: PA0
- ADG732 address/control/enable pins as listed above to drive row/column selection

## Known defaults

- System clock: 72 MHz (HSE ×9)
- ADC align: right; single conversion, software trigger
- UART: 921600 8N1 in user init; 115200 in CubeMX init (user code overrides)
- Visualizer port: 12345; ESP32 sends to `udpAddress` configured in the sketch

## Quick checks

- If the picture looks byte‑swapped, set `ENDIAN_MODE = "LE"` in `viz_full_4.py`
- If halves are mirrored, toggle `APPLY_HALF_MIRROR` in `viz_full_4.py`
- Ensure ESP32 `udpAddress` is your PC’s IP and firewall allows UDP on the chosen port
