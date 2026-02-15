# RetroShield Z80 CP/M 2.2 — Arduino Giga R1 WiFi

CP/M 2.2 running on a real Zilog Z80 processor via the [RetroShield Z80](https://www.tindie.com/products/8bitforce/retroshield-for-arduino-mega-z80/), controlled by an [Arduino Giga R1 WiFi](https://store.arduino.cc/products/giga-r1-wifi). Disk I/O is served over WiFi by a [Rust sector server](https://github.com/ajokela/retroshield-sector-server).

The Z80's 64KB address space lives in the Giga's internal SRAM. A custom [level converter PCB](https://github.com/ajokela/retroshield-level-shifter-pcb) bridges the Giga's 3.3V logic to the RetroShield's 5V. The system boots CP/M, runs DIR, and plays Zork I interactively.

## Hardware

| Component | Description |
|-----------|-------------|
| Arduino Giga R1 WiFi | STM32H747, 480MHz Cortex-M7, 1MB SRAM, WiFi |
| RetroShield Z80 | Real Zilog Z80 CPU on shield board, 5V logic |
| Level converter PCB | Custom shield with TXB0108 bidirectional level shifters ([design files](https://github.com/ajokela/retroshield-level-shifter-pcb)) |
| Sector server | Rust TCP server on local network ([source](https://github.com/ajokela/retroshield-sector-server)) |

## The TXB0108 Problem

The TXB0108 level converters use auto-direction sensing, which fails for several Z80 bus signals:

- **IORQ_N** — stuck HIGH (never toggles)
- **RD_N** — stuck HIGH (never toggles)
- **WR_N** — works for memory cycles only, not I/O
- **Data bus** — Z80-to-Arduino direction invisible during I/O

These failures required a fundamentally different approach: instead of passively reading bus signals, the Arduino actively decodes the Z80's instruction stream and maintains software copies of the CPU's internal registers.

## Software Architecture

### Guard-Only M1 Detection

Each Z80 instruction takes a known number of T-states. A lookup table (`tStates[256]`) provides the count for every opcode. After detecting one M1 (opcode fetch), a guard timer prevents false M1 detection until the instruction completes. The next MREQ-active memory read after the guard expires is the next M1.

### Software PC (softPC)

A software copy of the Z80's program counter, updated at each M1 by analyzing the opcode and computing the next address. Immune to the TXB0108's address bus propagation delay.

### Shadow Registers

Software copies of A, B, C, D, E, H, L, F, and SP, updated at M1 time by decoding each opcode from `z80RAM[softPC]`. Used for:

- **I/O data** — `OUT` instructions need the A register value, which can't be read from the bus
- **Branch prediction** — conditional branches need flag values to determine the taken/not-taken path
- **Pre-writes** — memory store instructions use shadow register values to write correct data to `z80RAM`

### Pre-Writes and Deferred Writes

Since bus writes go to wrong addresses (TXB0108 propagation delay), all memory stores are handled in software at M1 detection time. Read-modify-write instructions (INC/DEC (HL), CB-prefix on (HL)) use deferred writes to avoid double-applying operations.

### Direct GPIO Register Access

All hot-path I/O uses direct STM32H747 GPIO register access (BSRR, IDR, MODER) instead of Arduino HAL `digitalRead()`/`digitalWrite()`. This, combined with removing unnecessary `pinMode()` calls and I-cache optimization, achieves ~690,000 Z80 cycles/sec (~0.69 MHz effective clock).

## Files

| File | Description |
|------|-------------|
| `kz80_cpm_giga.ino` | Main Arduino sketch (~2,600 lines) |
| `boot.bin` | Z80 boot loader binary (331 bytes) — loads CPM.SYS via sector server |
| `CPM.SYS` | CP/M 2.2 system image: CCP + BDOS + BIOS (6,607 bytes) |
| `A.DSK` | CP/M disk image for drive A (256 KB) — contains ZORK1, ZORK2 |

## Pin Mapping

The RetroShield uses the same physical pin positions on the Giga as on the Mega 2560. The direct GPIO register access requires knowing the STM32H747 port/pin for each Arduino pin:

| Function | Arduino Pin | STM32 Port/Pin |
|----------|-------------|----------------|
| CLK | D52 | PK2 |
| MREQ_N | D41 | PK7 |
| WR_N | D40 | PE6 |
| IORQ_N | D39 | PI14 |
| INT_N | D50 | PI11 |
| RESET_N | D38 | PJ7 |
| Data D0–D7 | D49–D42 | PE4, PK0, PB2, PH15, PI13, PG10, PI10, PI15 |
| Addr A0–A7 | D22–D29 | PJ12, PG13, PG12, PJ0, PJ14, PJ1, PJ15, PJ2 |
| Addr A8–A15 | D37–D30 | PJ6, PK6, PJ5, PK5, PJ4, PK4, PJ3, PK3 |

## CP/M Memory Map

```
0000-00FF   Page Zero (jump vectors, FCBs, command buffer)
0100-DFFF   TPA (Transient Program Area) — 56KB
E000-E7FF   CCP (Console Command Processor)
E800-F5FF   BDOS (Basic Disk Operating System)
F600-FFFF   BIOS (Basic I/O System)
```

## I/O Port Map

| Port | Function |
|------|----------|
| 0x80 | MC6850 ACIA control/status (console serial) |
| 0x81 | MC6850 ACIA data (console TX/RX) |
| 0x10 | Disk command register |
| 0x11 | Disk status register |
| 0x12 | Disk data byte |
| 0x13 | Disk filename (write char by char, null-terminated) |
| 0x14–0x15 | Disk seek position (low, mid bytes) |
| 0x16–0x17 | DMA address (low, high bytes) |
| 0x18 | Block command (0=read, 1=write) |
| 0x19 | Seek position high byte |

## Prerequisites

- [Arduino IDE](https://www.arduino.cc/en/software) or `arduino-cli` with the `arduino:mbed_giga` board package installed
- A [RetroShield Z80](https://www.tindie.com/products/8bitforce/retroshield-for-arduino-mega-z80/) with a level converter board
- A machine on the same WiFi network running the [sector server](https://github.com/ajokela/retroshield-sector-server)

## Configuration

Edit the WiFi credentials and server IP at the top of `kz80_cpm_giga.ino`:

```cpp
const char* WIFI_SSID = "YourNetwork";
const char* WIFI_PASS = "YourPassword";
IPAddress SERVER_IP(192, 168, 0, 248);
const int SERVER_PORT = 9000;
```

## Build & Upload

```bash
# Using arduino-cli
arduino-cli compile --fqbn arduino:mbed_giga:giga kz80_cpm_giga.ino
arduino-cli upload -p /dev/cu.usbmodem2101 --fqbn arduino:mbed_giga:giga kz80_cpm_giga.ino

# Serial monitor
screen /dev/cu.usbmodem2101 115200
```

Or open `kz80_cpm_giga.ino` in the Arduino IDE, select **Arduino Giga R1 WiFi** as the board, and click Upload.

## Usage

1. Start the sector server on your host machine, pointing it at a directory containing `boot.bin`, `CPM.SYS`, and disk images:
   ```bash
   sector_server ./cpm_files 9000
   ```

2. Upload the sketch to the Giga and open a serial terminal at 115200 baud.

3. The Giga connects to WiFi, connects to the sector server, loads the boot loader, and starts the Z80. You should see:
   ```
   RetroShield Z80 Boot Loader
   Copyright (c) 2025 Alex Jokela, tinycomputers.io

   Loading CPM.SYS.....................................................
   Boot complete.

   RetroShield CP/M 2.2
   56K TPA

   a>
   ```

4. Type `dir` to see disk contents, or `zork1` to play Zork I.

## Blog Posts

This project is documented in a three-part series on [tinycomputers.io](https://tinycomputers.io):

1. [My Experience Using Fiverr for Custom PCB Design: A $468 Arduino Giga Shield](https://tinycomputers.io/posts/fiverr-pcb-design-arduino-giga-shield/)
2. [Porting CP/M to the Arduino Giga R1: When Level Converters Fight Back](https://tinycomputers.io/posts/cpm-on-arduino-giga-r1-wifi/)
3. [Playing Zork on a Real Z80: From CP/M Boot to the Great Underground Empire](https://tinycomputers.io/posts/zork-on-retroshield-z80-arduino-giga/)

## License

BSD 3-Clause License. See [LICENSE](LICENSE).

## Author

Alex Jokela — [tinycomputers.io](https://tinycomputers.io)
