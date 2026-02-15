////////////////////////////////////////////////////////////////////
// Z80 RetroShield CP/M 2.2 - Arduino Giga R1 WiFi
// 2026/02/14
//
// Copyright (c) 2019, 2023 Erturk Kocalar, 8Bitforce.com
// Copyright (c) 2025, 2026 Alex Jokela, tinycomputers.io
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// Hardware:
//   - Arduino Giga R1 WiFi (STM32H747, 480MHz Cortex-M7)
//   - Z80 RetroShield
//   - 3.3V-to-5V level converter (passthru between Giga and RetroShield)
//
// Disk I/O is handled by a sector server running on the network.
// The Giga connects to the server over WiFi and forwards Z80
// disk I/O port operations as TCP messages.
//
// Ported from kz80_cpm (Arduino Mega 2560 + KDRAM2560 DRAM shield).
// Changes:
//   - 64KB Z80 RAM in byte array (Giga has 1MB internal SRAM)
//   - AVR direct port I/O replaced with digitalRead/digitalWrite
//   - SD card replaced with WiFi sector server
//
////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <WiFi.h>

////////////////////////////////////////////////////////////////////
// Configuration
////////////////////////////////////////////////////////////////////
#define outputDEBUG     0       // Set to 1 for debug output

// WiFi credentials
const char* WIFI_SSID = "TP-Link_A8A8";
const char* WIFI_PASS = "CHANGEMEPASSWORD";

// Sector server address
const char* SERVER_IP   = "192.168.0.248";
const uint16_t SERVER_PORT = 9000;

// Network timeout (ms)
#define NET_TIMEOUT_MS  5000

////////////////////////////////////////////////////////////////////
// Z80 RAM - 64KB in SRAM (replaces KDRAM2560 DRAM shield)
////////////////////////////////////////////////////////////////////
byte z80RAM[65536];

////////////////////////////////////////////////////////////////////
// Z80 Pin Definitions
//
// The RetroShield uses the same physical pin positions as on
// the Mega. The pin numbers are identical. On the Mega these
// mapped to AVR port registers for fast parallel I/O; on the
// Giga we use digitalRead/digitalWrite instead.
//
// Data bus:     Mega PORTL = pins 49(bit0) .. 42(bit7)
// Address low:  Mega PORTA = pins 22(bit0) .. 29(bit7)
// Address high: Mega PORTC = pins 37(bit0) .. 30(bit7)
////////////////////////////////////////////////////////////////////

// Pin-to-bit mapping must match the AVR port register layout
// so the RetroShield wiring stays correct.
const int DATA_PINS[8]   = {49, 48, 47, 46, 45, 44, 43, 42};  // PL0-PL7
const int ADDR_L_PINS[8] = {22, 23, 24, 25, 26, 27, 28, 29};  // PA0-PA7
const int ADDR_H_PINS[8] = {37, 36, 35, 34, 33, 32, 31, 30};  // PC0-PC7

#define uP_RESET_N  38
#define uP_MREQ_N   41
#define uP_IORQ_N   39
#define uP_RD_N     53
#define uP_WR_N     40
#define uP_NMI_N    51
#define uP_INT_N    50
#define uP_CLK      52

////////////////////////////////////////////////////////////////////
// Direct GPIO Register Access (STM32H747)
//
// Arduino HAL digitalRead/digitalWrite go through pin lookup tables
// and function calls — far too slow for a hot loop.  Direct GPIO
// register access is ~10x faster.
//
// Pin-to-port mapping (from variants/GIGA/variant.cpp):
//   D52 (CLK)    → PK2     D41 (MREQ_N)  → PK7
//   D40 (WR_N)   → PE6     D50 (INT_N)   → PI11
//   D38 (RESET)  → PJ7     D39 (IORQ_N)  → PI14
//
//   Data bus D49..D42 (bits 0..7):
//     D49→PE4  D48→PK0  D47→PB2  D46→PH15
//     D45→PI13 D44→PG10 D43→PI10 D42→PI15
//
//   Address low D22..D29 (A0..A7):
//     D22→PJ12 D23→PG13 D24→PG12 D25→PJ0
//     D26→PJ14 D27→PJ1  D28→PJ15 D29→PJ2
//
//   Address high D37..D30 (A8..A15):
//     D37→PJ6  D36→PK6  D35→PJ5  D34→PK5
//     D33→PJ4  D32→PK4  D31→PJ3  D30→PK3
////////////////////////////////////////////////////////////////////

// Use STM32 HAL GPIO peripheral pointers (GPIOB, GPIOE, etc.) which
// are defined in stm32h747xx.h and available in the Arduino mbed core.
// These resolve to GPIO_TypeDef* pointers at known addresses.

// Fast pin set/clear using BSRR (bit set/reset register)
#define FAST_SET(port, pin)    ((port)->BSRR = (1U << (pin)))
#define FAST_CLR(port, pin)    ((port)->BSRR = (1U << ((pin) + 16)))
#define FAST_READ(port, pin)   (((port)->IDR >> (pin)) & 1U)

// Clock: PK2
#define CLK_HIGH      FAST_SET(GPIOK, 2)
#define CLK_LOW       FAST_CLR(GPIOK, 2)

// Control signal reads
#define STATE_MREQ_N  FAST_READ(GPIOK, 7)   // D41 → PK7
#define STATE_WR_N    FAST_READ(GPIOE, 6)    // D40 → PE6
#define STATE_IORQ_N  FAST_READ(GPIOI, 14)   // D39 → PI14

// RD_N is inferred from WR_N: during any bus cycle, the Z80 is either
// reading or writing. If WR_N is not asserted, it must be a read.
// This avoids reading pin 53 which is unreliable through the TXB0108
// level converter.
#define STATE_RD_N    (!STATE_WR_N)

// INT_N: PI11 — direct set/clear
#define INT_N_HIGH    FAST_SET(GPIOI, 11)
#define INT_N_LOW     FAST_CLR(GPIOI, 11)

// Short settle delay — a few NOPs (~10ns each at 480MHz).
// TXB0108 propagation is ~4-12ns; 50ns provides comfortable margin.
// Replaces delayMicroseconds(2) which cost 2000ns per tick.
inline void __attribute__((always_inline)) busSettle() {
    __asm volatile("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
                   "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
                   "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
}

////////////////////////////////////////////////////////////////////
// Bus I/O Helper Functions — Direct GPIO Register Access
////////////////////////////////////////////////////////////////////

bool dataBusIsOutput = false;

// Helper macro to set a single pin's MODER to output (01) or input (00)
#define GPIO_SET_OUTPUT(port, pin)  ((port)->MODER = ((port)->MODER & ~(3U << ((pin)*2))) | (1U << ((pin)*2)))
#define GPIO_SET_INPUT(port, pin)   ((port)->MODER = ((port)->MODER & ~(3U << ((pin)*2))))

inline void setDataBusOutput() {
    if (!dataBusIsOutput) {
        GPIO_SET_OUTPUT(GPIOE, 4);   // D49
        GPIO_SET_OUTPUT(GPIOK, 0);   // D48
        GPIO_SET_OUTPUT(GPIOB, 2);   // D47
        GPIO_SET_OUTPUT(GPIOH, 15);  // D46
        GPIO_SET_OUTPUT(GPIOI, 13);  // D45
        GPIO_SET_OUTPUT(GPIOG, 10);  // D44
        GPIO_SET_OUTPUT(GPIOI, 10);  // D43
        GPIO_SET_OUTPUT(GPIOI, 15);  // D42
        dataBusIsOutput = true;
    }
}

inline void setDataBusInput() {
    if (dataBusIsOutput) {
        GPIO_SET_INPUT(GPIOE, 4);
        GPIO_SET_INPUT(GPIOK, 0);
        GPIO_SET_INPUT(GPIOB, 2);
        GPIO_SET_INPUT(GPIOH, 15);
        GPIO_SET_INPUT(GPIOI, 13);
        GPIO_SET_INPUT(GPIOG, 10);
        GPIO_SET_INPUT(GPIOI, 10);
        GPIO_SET_INPUT(GPIOI, 15);
        dataBusIsOutput = false;
    }
}

// Write data bus — fully unrolled, one BSRR write per port
// D49(bit0)→PE4  D48(bit1)→PK0  D47(bit2)→PB2  D46(bit3)→PH15
// D45(bit4)→PI13 D44(bit5)→PG10 D43(bit6)→PI10 D42(bit7)→PI15
inline void __attribute__((always_inline)) writeDataBus(byte val) {
    // Port E: bit 0 → PE4
    GPIOE->BSRR = (val & 0x01) ? (1U << 4) : (1U << (4 + 16));
    // Port K: bit 1 → PK0
    GPIOK->BSRR = (val & 0x02) ? (1U << 0) : (1U << (0 + 16));
    // Port B: bit 2 → PB2
    GPIOB->BSRR = (val & 0x04) ? (1U << 2) : (1U << (2 + 16));
    // Port H: bit 3 → PH15
    GPIOH->BSRR = (val & 0x08) ? (1U << 15) : (1U << (15 + 16));
    // Port I: bit 4 → PI13, bit 6 → PI10, bit 7 → PI15 (combine into one write)
    uint32_t iBSRR = 0;
    iBSRR |= (val & 0x10) ? (1U << 13) : (1U << (13 + 16));
    iBSRR |= (val & 0x40) ? (1U << 10) : (1U << (10 + 16));
    iBSRR |= (val & 0x80) ? (1U << 15) : (1U << (15 + 16));
    GPIOI->BSRR = iBSRR;
    // Port G: bit 5 → PG10
    GPIOG->BSRR = (val & 0x20) ? (1U << 10) : (1U << (10 + 16));
}

inline byte __attribute__((always_inline)) readDataBus() {
    uint32_t eIDR = GPIOE->IDR;
    uint32_t kIDR = GPIOK->IDR;
    uint32_t bIDR = GPIOB->IDR;
    uint32_t hIDR = GPIOH->IDR;
    uint32_t iIDR = GPIOI->IDR;
    uint32_t gIDR = GPIOG->IDR;

    byte val = 0;
    if (eIDR & (1U << 4))  val |= 0x01;  // D49 PE4  = bit 0
    if (kIDR & (1U << 0))  val |= 0x02;  // D48 PK0  = bit 1
    if (bIDR & (1U << 2))  val |= 0x04;  // D47 PB2  = bit 2
    if (hIDR & (1U << 15)) val |= 0x08;  // D46 PH15 = bit 3
    if (iIDR & (1U << 13)) val |= 0x10;  // D45 PI13 = bit 4
    if (gIDR & (1U << 10)) val |= 0x20;  // D44 PG10 = bit 5
    if (iIDR & (1U << 10)) val |= 0x40;  // D43 PI10 = bit 6
    if (iIDR & (1U << 15)) val |= 0x80;  // D42 PI15 = bit 7
    return val;
}

inline uint16_t __attribute__((always_inline)) readAddress() {
    // Read Port J, K, and G IDR registers once each
    uint32_t jIDR = GPIOJ->IDR;
    uint32_t kIDR = GPIOK->IDR;
    uint32_t gIDR = GPIOG->IDR;

    uint16_t addr = 0;
    // Low byte (A0..A7)
    if (jIDR & (1U << 12)) addr |= (1 << 0);  // A0 = PJ12
    if (gIDR & (1U << 13)) addr |= (1 << 1);  // A1 = PG13
    if (gIDR & (1U << 12)) addr |= (1 << 2);  // A2 = PG12
    if (jIDR & (1U << 0))  addr |= (1 << 3);  // A3 = PJ0
    if (jIDR & (1U << 14)) addr |= (1 << 4);  // A4 = PJ14
    if (jIDR & (1U << 1))  addr |= (1 << 5);  // A5 = PJ1
    if (jIDR & (1U << 15)) addr |= (1 << 6);  // A6 = PJ15
    if (jIDR & (1U << 2))  addr |= (1 << 7);  // A7 = PJ2

    // High byte (A8..A15)
    if (jIDR & (1U << 6))  addr |= (1 << 8);  // A8  = PJ6
    if (kIDR & (1U << 6))  addr |= (1 << 9);  // A9  = PK6
    if (jIDR & (1U << 5))  addr |= (1 << 10); // A10 = PJ5
    if (kIDR & (1U << 5))  addr |= (1 << 11); // A11 = PK5
    if (jIDR & (1U << 4))  addr |= (1 << 12); // A12 = PJ4
    if (kIDR & (1U << 4))  addr |= (1 << 13); // A13 = PK4
    if (jIDR & (1U << 3))  addr |= (1 << 14); // A14 = PJ3
    if (kIDR & (1U << 3))  addr |= (1 << 15); // A15 = PK3

    return addr;
}

////////////////////////////////////////////////////////////////////
// MC6850 ACIA (Console)
////////////////////////////////////////////////////////////////////
#define ADDR_6850_DATA        0x81
#define ADDR_6850_CONTROL     0x80
#define CONTROL_RTS_STATE     (reg6850_CONTROL & 0b01000000)
#define CONTROL_TX_INT_ENABLE (reg6850_CONTROL & 0b00100000)
#define CONTROL_RX_INT_ENABLE (reg6850_CONTROL & 0b10000000)

byte reg6850_DATA_RX    = 0x00;
byte reg6850_DATA_TX    = 0x00;
byte reg6850_CONTROL    = 0x00;
byte reg6850_STATUS     = 0x00;

void mc6850_init() {
    reg6850_DATA_RX    = 0x00;
    reg6850_DATA_TX    = 0x00;
    reg6850_CONTROL    = 0b01010100;  // RTS HIGH, TX INT Disabled, RX INT Disabled
    reg6850_STATUS     = 0b00000010;  // CTS LOW, DCD LOW, TX EMPTY 1, RX FULL 0
}

////////////////////////////////////////////////////////////////////
// Disk I/O Ports (same port numbers as SD card version)
// The Z80 BIOS uses these ports identically regardless of
// whether the backing store is SD or network.
////////////////////////////////////////////////////////////////////
#define DISK_CMD_PORT      0x10   // Command register
#define DISK_STATUS_PORT   0x11   // Status register
#define DISK_DATA_PORT     0x12   // Data byte
#define DISK_FNAME_PORT    0x13   // Filename input
#define DISK_SEEK_LO       0x14   // Seek position low byte
#define DISK_SEEK_HI       0x15   // Seek position middle byte
#define DISK_DMA_LO        0x16   // DMA address low byte
#define DISK_DMA_HI        0x17   // DMA address high byte
#define DISK_BLOCK_CMD     0x18   // Block command
#define DISK_SEEK_EX       0x19   // Seek position high byte (bits 16-23)

// Commands (match sector server protocol)
#define DISK_CMD_OPEN_READ   0x01
#define DISK_CMD_CREATE      0x02
#define DISK_CMD_OPEN_APPEND 0x03
#define DISK_CMD_SEEK_START  0x04
#define DISK_CMD_CLOSE       0x05
#define DISK_CMD_DIR         0x06
#define DISK_CMD_OPEN_RW     0x07
#define DISK_CMD_SEEK        0x08
#define NET_CMD_READ_BLOCK   0x10
#define NET_CMD_WRITE_BLOCK  0x11

// Status bits
#define DISK_STATUS_READY    0x01
#define DISK_STATUS_ERROR    0x02
#define DISK_STATUS_DATA     0x80

// Block size for DMA
#define DISK_BLOCK_SIZE      128

////////////////////////////////////////////////////////////////////
// Network Connection
////////////////////////////////////////////////////////////////////
WiFiClient server;

////////////////////////////////////////////////////////////////////
// Disk I/O State
////////////////////////////////////////////////////////////////////
String diskFilename;
String diskActiveFile;    // Last successfully opened file (for reconnection)
bool diskFileOpen = false;
bool diskDirActive = false;
String diskDirBuffer;
uint16_t diskDirBufferPos = 0;
uint8_t diskStatus = DISK_STATUS_READY;
uint32_t diskSeekPos = 0;          // 24-bit seek position
uint16_t diskDmaAddr = 0x0080;     // DMA address
uint8_t diskBlockStatus = 0;
uint32_t diskOpCount = 0;          // Count disk operations for debug

////////////////////////////////////////////////////////////////////
// Network Helpers
////////////////////////////////////////////////////////////////////

// Ensure the server connection is alive. Reconnects if needed.
// Returns true if connected, false if reconnection failed.
bool ensureServerConnection() {
    if (server.connected()) return true;

    Serial.println("[NET] Connection lost, reconnecting...");
    server.stop();

    for (int attempt = 0; attempt < 3; attempt++) {
        if (server.connect(SERVER_IP, SERVER_PORT)) {
            Serial.println("[NET] Reconnected to server");

            // Re-open the disk file if one was open
            if (diskFileOpen && diskActiveFile.length() > 0) {
                netSendFileCommand(DISK_CMD_OPEN_RW, diskActiveFile);
                uint8_t status = netReadStatus();
                if (status != 0) {
                    Serial.println("[NET] Re-open file failed!");
                    diskFileOpen = false;
                    return false;
                }
                // Re-seek to previous position
                uint8_t seekCmd[4];
                seekCmd[0] = DISK_CMD_SEEK;
                seekCmd[1] = diskSeekPos & 0xFF;
                seekCmd[2] = (diskSeekPos >> 8) & 0xFF;
                seekCmd[3] = (diskSeekPos >> 16) & 0xFF;
                server.write(seekCmd, 4);
                uint8_t seekStatus = netReadStatus();
                if (seekStatus != 0) {
                    Serial.println("[NET] Re-seek failed!");
                }
                Serial.print("[NET] Restored file: ");
                Serial.print(diskActiveFile);
                Serial.print(" at seek=");
                Serial.println(diskSeekPos);
            }
            return true;
        }
        Serial.print("[NET] Reconnect attempt ");
        Serial.print(attempt + 1);
        Serial.println(" failed");
        delay(500);
    }
    Serial.println("[NET] Reconnection failed!");
    return false;
}

// Read exactly `len` bytes from the server connection.
bool netReadExact(uint8_t* buf, size_t len) {
    size_t pos = 0;
    unsigned long deadline = millis() + NET_TIMEOUT_MS;
    while (pos < len) {
        if (millis() > deadline) {
            Serial.println("[NET] Read timeout!");
            return false;
        }
        int avail = server.available();
        if (avail > 0) {
            int n = server.read(buf + pos, len - pos);
            if (n > 0) pos += n;
        } else {
            yield();  // Let RTOS service USB while waiting for network data
        }
    }
    return true;
}

// Read a single status byte from the server.
uint8_t netReadStatus() {
    uint8_t status;
    if (netReadExact(&status, 1)) {
        return status;
    }
    return 0xFF;  // Timeout = error
}

// Send a command + null-terminated filename to the server.
void netSendFileCommand(uint8_t cmd, const String& filename) {
    server.write(cmd);
    server.print(filename);
    server.write((uint8_t)0);
}

////////////////////////////////////////////////////////////////////
// Disk I/O Functions
////////////////////////////////////////////////////////////////////

bool disk_handles_port(uint8_t port) {
    return (port >= DISK_CMD_PORT && port <= DISK_SEEK_EX);
}

uint8_t disk_read_port(uint8_t port) {
    switch (port) {
        case DISK_STATUS_PORT: {
            uint8_t s = diskStatus;
            if (diskFileOpen || diskDirActive) {
                s |= DISK_STATUS_DATA;
            }
            return s;
        }

        case DISK_DATA_PORT: {
            if (diskDirActive) {
                return disk_read_dir_byte();
            }
            return 0;
        }

        case DISK_BLOCK_CMD:
            return diskBlockStatus;

        default:
            return 0xFF;
    }
}

void disk_write_port(uint8_t port, uint8_t val) {
#if outputDEBUG
    Serial.print("[DISK_W] port=0x");
    Serial.print(port, HEX);
    Serial.print(" val=0x");
    Serial.println(val, HEX);
#endif
    switch (port) {
        case DISK_CMD_PORT:
            disk_handle_command(val);
            break;

        case DISK_DATA_PORT:
            // Single-byte file writes not used by CP/M BIOS
            break;

        case DISK_FNAME_PORT:
            if (val == 0) {
                // Filename complete (debug below if needed)
            } else {
                diskFilename += (char)val;
            }
            break;

        case DISK_SEEK_LO:
            diskSeekPos = (diskSeekPos & 0xFFFF00) | val;
            break;

        case DISK_SEEK_HI:
            diskSeekPos = (diskSeekPos & 0xFF00FF) | ((uint32_t)val << 8);
            break;

        case DISK_SEEK_EX:
            diskSeekPos = (diskSeekPos & 0x00FFFF) | ((uint32_t)val << 16);
            break;

        case DISK_DMA_LO:
            diskDmaAddr = (diskDmaAddr & 0xFF00) | val;
            break;

        case DISK_DMA_HI:
            diskDmaAddr = (diskDmaAddr & 0x00FF) | ((uint16_t)val << 8);
            break;

        case DISK_BLOCK_CMD:
            if (val == 0) {
                disk_do_block_read();
            } else {
                disk_do_block_write();
            }
            break;
    }
}

void disk_handle_command(uint8_t cmd) {
    if (!ensureServerConnection()) {
        diskStatus = DISK_STATUS_ERROR | DISK_STATUS_READY;
        return;
    }
    switch (cmd) {
        case DISK_CMD_OPEN_READ:
            netSendFileCommand(DISK_CMD_OPEN_READ, diskFilename);
            {
                uint8_t st = netReadStatus();
                if (st == 0) {
                    diskFileOpen = true;
                    diskActiveFile = diskFilename;
                    diskStatus = DISK_STATUS_READY;
                } else {
                    diskFileOpen = false;
                    diskStatus = DISK_STATUS_ERROR | DISK_STATUS_READY;
                }
            }
            diskFilename = "";
            break;

        case DISK_CMD_CREATE:
#if outputDEBUG
            Serial.print("[DISK] CREATE: ");
            Serial.println(diskFilename);
#endif
            netSendFileCommand(DISK_CMD_CREATE, diskFilename);
            if (netReadStatus() == 0) {
                diskFileOpen = true;
                diskActiveFile = diskFilename;
                diskStatus = DISK_STATUS_READY;
            } else {
                diskFileOpen = false;
                diskStatus = DISK_STATUS_ERROR | DISK_STATUS_READY;
            }
            diskFilename = "";
            break;

        case DISK_CMD_OPEN_APPEND:
#if outputDEBUG
            Serial.print("[DISK] OPEN_APPEND: ");
            Serial.println(diskFilename);
#endif
            netSendFileCommand(DISK_CMD_OPEN_APPEND, diskFilename);
            if (netReadStatus() == 0) {
                diskFileOpen = true;
                diskActiveFile = diskFilename;
                diskStatus = DISK_STATUS_READY;
            } else {
                diskFileOpen = false;
                diskStatus = DISK_STATUS_ERROR | DISK_STATUS_READY;
            }
            diskFilename = "";
            break;

        case DISK_CMD_SEEK_START:
#if outputDEBUG
            Serial.println("[DISK] SEEK_START");
#endif
            server.write(DISK_CMD_SEEK_START);
            if (netReadStatus() == 0) {
                diskStatus = DISK_STATUS_READY;
            } else {
                diskStatus = DISK_STATUS_ERROR | DISK_STATUS_READY;
            }
            break;

        case DISK_CMD_CLOSE:
#if outputDEBUG
            Serial.println("[DISK] CLOSE");
#endif
            server.write(DISK_CMD_CLOSE);
            netReadStatus();
            diskFileOpen = false;
            diskActiveFile = "";
            diskDirActive = false;
            diskStatus = DISK_STATUS_READY;
            break;

        case DISK_CMD_DIR: {
#if outputDEBUG
            Serial.println("[DISK] DIR");
#endif
            server.write(DISK_CMD_DIR);
            uint8_t status = netReadStatus();
            if (status == 0) {
                // Read the full directory listing until null terminator
                diskDirBuffer = "";
                uint8_t ch;
                while (netReadExact(&ch, 1) && ch != 0) {
                    diskDirBuffer += (char)ch;
                }
                diskDirBufferPos = 0;
                diskDirActive = true;
                diskStatus = DISK_STATUS_READY;
            } else {
                diskDirActive = false;
                diskStatus = DISK_STATUS_ERROR | DISK_STATUS_READY;
            }
            break;
        }

        case DISK_CMD_OPEN_RW:
#if outputDEBUG
            Serial.print("[DISK] OPEN_RW: ");
            Serial.println(diskFilename);
#endif
            netSendFileCommand(DISK_CMD_OPEN_RW, diskFilename);
            {
                uint8_t st = netReadStatus();
                if (st == 0) {
                    diskFileOpen = true;
                    diskActiveFile = diskFilename;
                    diskStatus = DISK_STATUS_READY;
                } else {
                    diskFileOpen = false;
                    diskStatus = DISK_STATUS_ERROR | DISK_STATUS_READY;
                    Serial.println("[DISK] OPEN FAIL");
                }
            }
            diskFilename = "";
            break;

        case DISK_CMD_SEEK: {
            if (false) {
                Serial.print("[DISK] SEEK: ");
                Serial.println(diskSeekPos);
            }
            uint8_t seekBuf[4];
            seekBuf[0] = DISK_CMD_SEEK;
            seekBuf[1] = (uint8_t)(diskSeekPos & 0xFF);
            seekBuf[2] = (uint8_t)((diskSeekPos >> 8) & 0xFF);
            seekBuf[3] = (uint8_t)((diskSeekPos >> 16) & 0xFF);
            server.write(seekBuf, 4);
            if (netReadStatus() == 0) {
                diskStatus = DISK_STATUS_READY;
            } else {
                diskStatus = DISK_STATUS_ERROR | DISK_STATUS_READY;
            }
            break;
        }
    }
}

uint8_t disk_read_dir_byte() {
    if (!diskDirActive) return 0;

    if (diskDirBufferPos < diskDirBuffer.length()) {
        return diskDirBuffer[diskDirBufferPos++];
    }

    // End of listing
    diskDirActive = false;
    diskStatus = DISK_STATUS_READY;
    return 0;
}

void disk_do_block_read() {
    diskOpCount++;
    if (!diskFileOpen) {
        if (false) Serial.println("[BLK] no file open!");
        diskBlockStatus = 1;
        return;
    }

    if (false) {
        char dbg[60];
        snprintf(dbg, sizeof(dbg), "[BLK_RD] #%lu DMA=%04X seek=%lu",
                 diskOpCount, diskDmaAddr, diskSeekPos);
        Serial.println(dbg);
    }

    if (!ensureServerConnection()) {
        diskBlockStatus = 1;
        return;
    }

    server.write(NET_CMD_READ_BLOCK);
    uint8_t status = netReadStatus();

    if (status != 0) {
        diskBlockStatus = 1;
        // Still need to consume the 128 data bytes from server
        uint8_t discard[DISK_BLOCK_SIZE];
        netReadExact(discard, DISK_BLOCK_SIZE);
        if (false) Serial.println("[BLK_RD] server error!");
        return;
    }

    uint8_t buffer[DISK_BLOCK_SIZE];
    if (!netReadExact(buffer, DISK_BLOCK_SIZE)) {
        diskBlockStatus = 1;
        if (false) Serial.println("[BLK_RD] read timeout!");
        return;
    }

    // Copy to Z80 RAM
    memcpy(&z80RAM[diskDmaAddr], buffer, DISK_BLOCK_SIZE);

    diskBlockStatus = 0;
#if outputDEBUG
    Serial.print("[DISK] Block read: ");
    Serial.print(DISK_BLOCK_SIZE);
    Serial.print(" bytes to ");
    Serial.println(diskDmaAddr, HEX);
#endif
}

void disk_do_block_write() {
    if (!diskFileOpen) {
#if outputDEBUG
        Serial.println("[DISK] Block write: no file open");
#endif
        diskBlockStatus = 1;
        return;
    }

    if (!ensureServerConnection()) {
        diskBlockStatus = 1;
        return;
    }

    // Send write command + 128 bytes from Z80 RAM
    server.write(NET_CMD_WRITE_BLOCK);
    server.write(&z80RAM[diskDmaAddr], DISK_BLOCK_SIZE);

    uint8_t status = netReadStatus();

    if (status != 0) {
        diskBlockStatus = 1;
        return;
    }

    diskBlockStatus = 0;
#if outputDEBUG
    Serial.print("[DISK] Block write: ");
    Serial.print(DISK_BLOCK_SIZE);
    Serial.print(" bytes from ");
    Serial.println(diskDmaAddr, HEX);
#endif
}

////////////////////////////////////////////////////////////////////
// Processor Variables
////////////////////////////////////////////////////////////////////
unsigned long clock_cycle_count;
unsigned int uP_ADDR;
byte uP_DATA;
byte prevDATA = 0;

////////////////////////////////////////////////////////////////////
// IO Detection Strategy
//
// IORQ_N (pin 39), RD_N (pin 53), and WR_N (during IO) are
// invisible through the TXB0108 level converter. The data bus
// and address bus also cannot be read Z80→Arduino during IO.
//
// Guard-only approach:
//   M1 detection uses only a time-based guard (m1Guard). After
//   detecting an opcode, m1Guard is set based on the instruction's
//   T-state count. The next MREQ-active read after the guard
//   expires is the next M1. No transition tracking (MREQ edges
//   are unreliable through TXB0108).
//
//   IO is handled immediately at M1 detection:
//   - OUT (D3): port = z80RAM[PC+1], data = shadowA
//   - IN  (DB): port = z80RAM[PC+1], drive response during IO cycle
//
//   Conditional RET (cc) uses max T-states (11) for the guard,
//   which may cause the following instruction(s) to be missed.
//   A retroactive scan handles any missed OUT instructions.
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
// Shadow Z80 Registers (kept for debug tracing, not used for IO)
////////////////////////////////////////////////////////////////////
uint8_t shadowA = 0, shadowB = 0, shadowC = 0, shadowD = 0, shadowE = 0;
uint8_t shadowH = 0, shadowL = 0;

// Shadow flags register: S Z - H - P/V N C  (bits 7..0)
// Used to resolve conditional branches in software instead of
// relying on the unreliable TXB0108-delayed address bus.
#define FLAG_C  0x01
#define FLAG_N  0x02
#define FLAG_PV 0x04
#define FLAG_H  0x10
#define FLAG_Z  0x40
#define FLAG_S  0x80
uint8_t shadowF = 0;

// Parity lookup table (true = even parity)
static const uint8_t parityTable[256] = {
    1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
    0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
    0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
    1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
    0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
    1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
    1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
    0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
};

// Helper: compute Z80 flags for 8-bit logical ops (AND/OR/XOR)
inline uint8_t flagsLogic(uint8_t result, bool isAND) {
    return (result & FLAG_S)
         | (result == 0 ? FLAG_Z : 0)
         | (isAND ? FLAG_H : 0)
         | (parityTable[result] ? FLAG_PV : 0);
    // N=0, C=0
}

// Helper: compute Z80 flags for INC r
inline uint8_t flagsInc(uint8_t result, uint8_t oldF) {
    return (result & FLAG_S)
         | (result == 0 ? FLAG_Z : 0)
         | ((result & 0x0F) == 0 ? FLAG_H : 0)
         | (result == 0x80 ? FLAG_PV : 0)   // overflow: 0x7F+1=0x80
         | (oldF & FLAG_C);                  // C unchanged
    // N=0
}

// Helper: compute Z80 flags for DEC r
inline uint8_t flagsDec(uint8_t result, uint8_t oldF) {
    return (result & FLAG_S)
         | (result == 0 ? FLAG_Z : 0)
         | ((result & 0x0F) == 0x0F ? FLAG_H : 0)
         | (result == 0x7F ? FLAG_PV : 0)   // overflow: 0x80-1=0x7F
         | FLAG_N
         | (oldF & FLAG_C);
}

// Helper: compute Z80 flags for 8-bit ADD/ADC
inline uint8_t flagsAdd(uint8_t a, uint8_t b, uint8_t carry) {
    uint16_t sum = a + b + carry;
    uint8_t result = sum & 0xFF;
    return (result & FLAG_S)
         | (result == 0 ? FLAG_Z : 0)
         | (((a ^ b ^ result) & 0x10) ? FLAG_H : 0)
         | ((((a ^ ~b) & (a ^ result)) & 0x80) ? FLAG_PV : 0)
         | (sum > 0xFF ? FLAG_C : 0);
    // N=0
}

// Helper: compute Z80 flags for 8-bit SUB/SBC/CP
inline uint8_t flagsSub(uint8_t a, uint8_t b, uint8_t carry) {
    uint16_t diff = a - b - carry;
    uint8_t result = diff & 0xFF;
    return (result & FLAG_S)
         | (result == 0 ? FLAG_Z : 0)
         | (((a ^ b ^ result) & 0x10) ? FLAG_H : 0)
         | ((((a ^ b) & (a ^ result)) & 0x80) ? FLAG_PV : 0)
         | FLAG_N
         | (diff > 0xFF ? FLAG_C : 0);
}
bool shadowZF = false, shadowCF = false, shadowSF = false;

////////////////////////////////////////////////////////////////////
// IO State Machine
////////////////////////////////////////////////////////////////////
enum IoState : uint8_t {
    IO_IDLE = 0,
    IO_IN_PENDING,     // Waiting for IO cycle to start driving
    IO_IN_DRIVING,     // Driving IO read response on data bus
};

IoState ioState = IO_IDLE;
uint8_t ioResponse = 0;
unsigned long ioInDriveAfter = 0;  // tick when IN response driving starts

// Time-based guard: no M1 detection before this tick.
// Set to clock_cycle_count + tStates[opcode] after each M1.
// Initialized to max to suppress false M1 during reset ticks.
unsigned long m1Guard = 0xFFFFFFFF;

// Bus address captured at previous M1 detection (for validation at next M1)
uint16_t prevBusAddr = 0xFFFF;

// Retroactive IO scan: when a conditional RET is detected,
// record the fallthrough address. If the next M1 is nearby
// (RET was not taken), scan for missed OUT instructions.
uint16_t prevM1Addr = 0xFFFF;
bool checkMissedIO = false;
uint16_t missedIOScanStart = 0;

// Alternate softPC for conditional branches: the taken path.
// At next M1, use shadow flags to decide which path was taken.
uint16_t softPC_alt = 0xFFFF;  // 0xFFFF = no alternate

// Deferred SP delta to apply if the taken (alt) path is selected.
// Positive = pop (RET cc taken), negative = push (CALL cc taken).
int8_t softPC_alt_spDelta = 0;

// For CALL cc taken: the return address to push onto the shadow stack.
uint16_t softPC_alt_retAddr = 0;

// Opcode of the conditional branch instruction, used at resolution
// time to determine which flag/condition to check.
uint8_t softPC_alt_opcode = 0;

// Deferred memory write for read-modify-write instructions (INC/DEC (HL),
// CB-prefix shifts/rotates/SET/RES on (HL)).
// These instructions read a memory location, modify it, and write it back.
// If we pre-write at M1 time, the Z80 reads the already-modified value
// during its execute phase, causing a double-modify.
// Instead, we defer the write and apply it at the next M1 detection,
// after the Z80 has finished executing the instruction.
uint16_t deferredWriteAddr = 0xFFFF;  // 0xFFFF = no pending write
uint8_t  deferredWriteVal  = 0;

////////////////////////////////////////////////////////////////////
// T-states table for guard-only M1 detection
//
// tStates[opcode] = total T-state count for the instruction.
// For conditional instructions: use max (taken) count for RET cc
// (because taken path has memory reads that would cause false M1),
// and min (not-taken) count for JR cc/CALL cc/DJNZ (because
// remaining taken-path cycles are writes or internal).
//
// Prefixes: CB=15, DD=23, ED=21, FD=23 (worst-case for any
// instruction in that prefix group).
////////////////////////////////////////////////////////////////////
static const uint8_t tStates[256] = {
    // 0x00-0x0F: NOP, LD BC/nn, LD(BC)/A, INC BC, INC B, DEC B, LD B/n, RLCA,
    //            EX AF, ADD HL/BC, LD A/(BC), DEC BC, INC C, DEC C, LD C/n, RRCA
     4, 10,  7,  6,  4,  4,  7,  4,  4, 11,  7,  6,  4,  4,  7,  4,
    // 0x10-0x1F: DJNZ(8min), LD DE/nn, LD(DE)/A, INC DE, INC D, DEC D, LD D/n, RLA,
    //            JR(12), ADD HL/DE, LD A/(DE), DEC DE, INC E, DEC E, LD E/n, RRA
     8, 10,  7,  6,  4,  4,  7,  4, 12, 11,  7,  6,  4,  4,  7,  4,
    // 0x20-0x2F: JR NZ(7min), LD HL/nn, LD(nn)/HL, INC HL, INC H, DEC H, LD H/n, DAA,
    //            JR Z(7min), ADD HL/HL, LD HL/(nn), DEC HL, INC L, DEC L, LD L/n, CPL
     7, 10, 16,  6,  4,  4,  7,  4,  7, 11, 16,  6,  4,  4,  7,  4,
    // 0x30-0x3F: JR NC(7min), LD SP/nn, LD(nn)/A, INC SP, INC(HL), DEC(HL), LD(HL)/n, SCF,
    //            JR C(7min), ADD HL/SP, LD A/(nn), DEC SP, INC A, DEC A, LD A/n, CCF
     7, 10, 13,  6, 11, 11, 10,  4,  7, 11, 13,  6,  4,  4,  7,  4,
    // 0x40-0x4F: LD r,r' (4) except LD r,(HL) (7)
     4,  4,  4,  4,  4,  4,  7,  4,  4,  4,  4,  4,  4,  4,  7,  4,
    // 0x50-0x5F
     4,  4,  4,  4,  4,  4,  7,  4,  4,  4,  4,  4,  4,  4,  7,  4,
    // 0x60-0x6F
     4,  4,  4,  4,  4,  4,  7,  4,  4,  4,  4,  4,  4,  4,  7,  4,
    // 0x70-0x7F: LD (HL),r (7) except HALT (4)
     7,  7,  7,  7,  7,  7,  4,  7,  4,  4,  4,  4,  4,  4,  7,  4,
    // 0x80-0x8F: ALU A,r (4) except ALU A,(HL) (7)
     4,  4,  4,  4,  4,  4,  7,  4,  4,  4,  4,  4,  4,  4,  7,  4,
    // 0x90-0x9F
     4,  4,  4,  4,  4,  4,  7,  4,  4,  4,  4,  4,  4,  4,  7,  4,
    // 0xA0-0xAF
     4,  4,  4,  4,  4,  4,  7,  4,  4,  4,  4,  4,  4,  4,  7,  4,
    // 0xB0-0xBF
     4,  4,  4,  4,  4,  4,  7,  4,  4,  4,  4,  4,  4,  4,  7,  4,
    // 0xC0-0xCF: RET NZ(11max), POP BC(10), JP NZ(10), JP(10), CALL NZ(10min),
    //            PUSH BC(11), ADD A/n(7), RST 00(11), RET Z(11max), RET(10),
    //            JP Z(10), CB prefix(15), CALL Z(10min), CALL(17), ADC A/n(7), RST 08(11)
    11, 10, 10, 10, 10, 11,  7, 11, 11, 10, 10, 15, 10, 17,  7, 11,
    // 0xD0-0xDF: RET NC(11max), POP DE(10), JP NC(10), OUT(11), CALL NC(10min),
    //            PUSH DE(11), SUB n(7), RST 10(11), RET C(11max), EXX(4),
    //            JP C(10), IN(11), CALL C(10min), DD prefix(23), SBC A/n(7), RST 18(11)
    11, 10, 10, 11, 10, 11,  7, 11, 11,  4, 10, 11, 10, 23,  7, 11,
    // 0xE0-0xEF: RET PO(11max), POP HL(10), JP PO(10), EX(SP)HL(19), CALL PO(10min),
    //            PUSH HL(11), AND n(7), RST 20(11), RET PE(11max), JP(HL)(4),
    //            JP PE(10), EX DE/HL(4), CALL PE(10min), ED prefix(21), XOR n(7), RST 28(11)
    11, 10, 10, 19, 10, 11,  7, 11, 11,  4, 10,  4, 10, 21,  7, 11,
    // 0xF0-0xFF: RET P(11max), POP AF(10), JP P(10), DI(4), CALL P(10min),
    //            PUSH AF(11), OR n(7), RST 30(11), RET M(11max), LD SP/HL(6),
    //            JP M(10), EI(4), CALL M(10min), FD prefix(23), CP n(7), RST 38(11)
    11, 10, 10,  4, 10, 11,  7, 11, 11,  6, 10,  4, 10, 23,  7, 11,
};

////////////////////////////////////////////////////////////////////
// Software PC tracking
//
// The address bus through TXB0108 lags by 1-3 ticks. Instead of
// reading the address bus for M1 opcode identification, we maintain
// a software PC (softPC) and use z80RAM[softPC] for the opcode.
//
// After each M1, softPC is advanced based on instruction length
// and branch targets. For conditional branches, softPC is set to
// the fallthrough (not-taken) path; if the bus address matches a
// branch target instead, softPC is corrected.
////////////////////////////////////////////////////////////////////
uint16_t softPC = 0x0000;  // reset vector
uint16_t softSP = 0x0000;  // shadow SP for RET tracking

// Instruction byte length (1, 2, or 3 bytes).
// Prefixes (CB/DD/ED/FD) use length 0 as marker (handled specially).
static const uint8_t instrLen[256] = {
    // 0x00-0x0F
    1, 3, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1,
    // 0x10-0x1F
    2, 3, 1, 1, 1, 1, 2, 1, 2, 1, 1, 1, 1, 1, 2, 1,
    // 0x20-0x2F
    2, 3, 3, 1, 1, 1, 2, 1, 2, 1, 3, 1, 1, 1, 2, 1,
    // 0x30-0x3F
    2, 3, 3, 1, 1, 1, 2, 1, 2, 1, 3, 1, 1, 1, 2, 1,
    // 0x40-0x7F: all 1
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    // 0x80-0xBF: all 1
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    // 0xC0-0xCF: RET cc(1), POP(1), JP cc(3), JP(3), CALL cc(3),
    //            PUSH(1), ALU n(2), RST(1), RET cc(1), RET(1),
    //            JP cc(3), CB prefix(0), CALL cc(3), CALL(3), ALU n(2), RST(1)
    1, 1, 3, 3, 3, 1, 2, 1, 1, 1, 3, 0, 3, 3, 2, 1,
    // 0xD0-0xDF
    1, 1, 3, 2, 3, 1, 2, 1, 1, 1, 3, 2, 3, 0, 2, 1,
    // 0xE0-0xEF
    1, 1, 3, 1, 3, 1, 2, 1, 1, 1, 3, 1, 3, 0, 2, 1,
    // 0xF0-0xFF
    1, 1, 3, 1, 3, 1, 2, 1, 1, 1, 3, 1, 3, 0, 2, 1,
};

// Compute next PC after executing instruction at addr.
// Returns fallthrough PC for conditional branches.
// Also updates softSP for PUSH/POP/CALL/RET.
uint16_t computeNextPC(uint16_t addr, uint8_t opcode) {
    uint16_t fallthrough = addr + instrLen[opcode];

    // Clear alternate path — only conditional branches set it below
    softPC_alt = 0xFFFF;

    switch (opcode) {
        // Unconditional JP nn
        case 0xC3:
            return z80RAM[addr + 1] | (z80RAM[addr + 2] << 8);

        // Unconditional CALL nn
        case 0xCD:
            softSP -= 2;
            z80RAM[softSP] = (addr + 3) & 0xFF;        // push return addr
            z80RAM[softSP + 1] = (addr + 3) >> 8;
            return z80RAM[addr + 1] | (z80RAM[addr + 2] << 8);

        // Unconditional RET
        case 0xC9:
            { uint16_t ret = z80RAM[softSP] | (z80RAM[softSP + 1] << 8);
              softSP += 2;
              return ret; }

        // Unconditional JR d
        case 0x18:
            return addr + 2 + (int8_t)z80RAM[addr + 1];

        // JP (HL)
        case 0xE9:
            return (shadowH << 8) | shadowL;

        // RST vectors — push return address (addr+1) then jump
        case 0xC7: case 0xCF: case 0xD7: case 0xDF:
        case 0xE7: case 0xEF: case 0xF7: case 0xFF: {
            softSP -= 2;
            z80RAM[softSP] = (addr + 1) & 0xFF;
            z80RAM[softSP + 1] = (addr + 1) >> 8;
            return opcode & 0x38;  // RST vector: 00,08,10,18,20,28,30,38
        }

        // LD SP, nn
        case 0x31:
            softSP = z80RAM[addr + 1] | (z80RAM[addr + 2] << 8);
            return fallthrough;

        // LD SP, HL
        case 0xF9:
            softSP = (shadowH << 8) | shadowL;
            return fallthrough;

        // PUSH rr (SP -= 2)
        case 0xC5: case 0xD5: case 0xE5: case 0xF5:
            softSP -= 2;
            return fallthrough;

        // POP rr (SP += 2)
        case 0xC1: case 0xD1: case 0xE1: case 0xF1:
            softSP += 2;
            return fallthrough;

        // CB prefix: all CB xx instructions are 2 bytes
        case 0xCB: return addr + 2;

        // ED prefix: mostly 2 bytes, some are 4; RETN/RETI pop like RET
        // Block repeat instructions re-execute at same addr when not done.
        case 0xED: {
            uint8_t ed_op = z80RAM[addr + 1];
            // 4-byte: ED 43/4B/53/5B/63/6B/73/7B nn nn = LD (nn),rr / LD rr,(nn)
            if ((ed_op & 0xC7) == 0x43) return addr + 4;
            // RETN (ED 45) / RETI (ED 4D) — pop return address like RET
            if (ed_op == 0x45 || ed_op == 0x4D) {
                uint16_t retAddr = z80RAM[softSP] | (z80RAM[softSP + 1] << 8);
                softSP += 2;
                return retAddr;
            }
            // Block repeat: LDIR(B0), LDDR(B8), CPIR(B1), CPDR(B9)
            // Z80 re-executes at same PC when repeat condition still met.
            // Shadow tracking already decremented BC by 1 for this iteration.
            // For LDIR/LDDR: repeat while BC != 0
            if (ed_op == 0xB0 || ed_op == 0xB8) {
                uint16_t bc = (shadowB << 8) | shadowC;
                if (bc != 0) return addr;  // repeat: stay at same instruction
                return addr + 2;           // done: advance
            }
            // For CPIR/CPDR: repeat while BC != 0 AND last compare != match (Z not set)
            if (ed_op == 0xB1 || ed_op == 0xB9) {
                uint16_t bc = (shadowB << 8) | shadowC;
                if (bc != 0 && !(shadowF & FLAG_Z)) return addr;  // repeat
                return addr + 2;  // done (BC=0 or match found)
            }
            // INIR/INDR/OTIR/OTDR — repeat while B != 0
            if (ed_op == 0xB2 || ed_op == 0xBA || ed_op == 0xB3 || ed_op == 0xBB) {
                if (shadowB != 0) return addr;
                return addr + 2;
            }
            return addr + 2;
        }

        // DD/FD prefix: 2, 3, or 4 bytes depending on sub-opcode
        case 0xDD: case 0xFD: {
            uint8_t ix_op = z80RAM[addr + 1];
            // DDCB/FDCB d xx = 4 bytes
            if (ix_op == 0xCB) return addr + 4;
            // Instructions that take nn (2 extra bytes) = 4 bytes total
            // LD IX,nn (21), LD (nn),IX (22), LD IX,(nn) (2A)
            if (ix_op == 0x21 || ix_op == 0x22 || ix_op == 0x2A) return addr + 4;
            // LD (IX+d),n = 36 d n = 4 bytes
            if (ix_op == 0x36) return addr + 4;
            // PUSH IX (E5): SP -= 2
            if (ix_op == 0xE5) { softSP -= 2; return addr + 2; }
            // POP IX (E1): SP += 2
            if (ix_op == 0xE1) { softSP += 2; return addr + 2; }
            // 2-byte instructions (no displacement):
            // ADD IX,rr: 09/19/29/39
            // INC IX/DEC IX: 23/2B
            // JP (IX): E9 (can't resolve target without IX tracking, just advance)
            // LD SP,IX: F9
            // EX (SP),IX: E3
            if (ix_op == 0x09 || ix_op == 0x19 || ix_op == 0x29 || ix_op == 0x39 ||
                ix_op == 0x23 || ix_op == 0x2B ||
                ix_op == 0xE3 || ix_op == 0xE9 || ix_op == 0xF9)
                return addr + 2;
            // Everything else with displacement d = 3 bytes:
            // LD r,(IX+d), LD (IX+d),r, ALU A,(IX+d), INC/DEC (IX+d), etc.
            return addr + 3;
        }

        // Conditional RET cc — fallthrough = not taken, alt = return addr
        case 0xC0: case 0xC8: case 0xD0: case 0xD8:
        case 0xE0: case 0xE8: case 0xF0: case 0xF8:
            softPC_alt = z80RAM[softSP] | (z80RAM[softSP + 1] << 8);
            softPC_alt_spDelta = 2;  // pop if taken
            softPC_alt_opcode = opcode;
            return fallthrough;

        // Conditional JP cc nn — fallthrough = not taken, alt = jump target
        case 0xC2: case 0xCA: case 0xD2: case 0xDA:
        case 0xE2: case 0xEA: case 0xF2: case 0xFA:
            softPC_alt = z80RAM[addr + 1] | (z80RAM[addr + 2] << 8);
            softPC_alt_spDelta = 0;  // no SP change
            softPC_alt_opcode = opcode;
            return fallthrough;

        // Conditional CALL cc nn — fallthrough = not taken, alt = call target
        case 0xC4: case 0xCC: case 0xD4: case 0xDC:
        case 0xE4: case 0xEC: case 0xF4: case 0xFC:
            softPC_alt = z80RAM[addr + 1] | (z80RAM[addr + 2] << 8);
            softPC_alt_spDelta = -2;  // push if taken
            softPC_alt_retAddr = addr + 3;  // return address to push
            softPC_alt_opcode = opcode;
            return fallthrough;

        // Conditional JR cc d — fallthrough = not taken, alt = branch target
        case 0x20: case 0x28: case 0x30: case 0x38:
            softPC_alt = addr + 2 + (int8_t)z80RAM[addr + 1];
            softPC_alt_spDelta = 0;
            softPC_alt_opcode = opcode;
            return fallthrough;

        // DJNZ d — same as conditional JR
        case 0x10:
            softPC_alt = addr + 2 + (int8_t)z80RAM[addr + 1];
            softPC_alt_spDelta = 0;
            softPC_alt_opcode = opcode;
            return fallthrough;

        // All other instructions: simple fallthrough
        default:
            return fallthrough;
    }
}

// Debug: log first N branch resolutions
#define BR_DBG_MAX 50
struct BrDbgEntry {
    unsigned long clk;
    uint16_t busAddr;
    uint16_t sPC;
    uint16_t sPCalt;
    uint16_t diffMain;
    uint16_t diffAlt;
    uint8_t picked; // 0=main, 1=alt
};
BrDbgEntry brDbg[BR_DBG_MAX];
int brDbgCount = 0;

// Debug: log first N IO events
#define IO_DBG_MAX 200
struct IoDbgEntry {
    unsigned long clk;
    uint16_t addr;
    uint8_t port;
    uint8_t data;
    uint8_t type;  // 'W' for write, 'R' for read
};
IoDbgEntry ioDbg[IO_DBG_MAX];
int ioDbgCount = 0;
bool ioDbgPrinted = false;

// Debug: log M1 detections during loading loop (start at LD HL,nn before loop)
#define M1_DBG_MAX 120
struct M1DbgEntry {
    unsigned long clk;
    uint16_t addr;
    uint8_t opcode;
    uint8_t sA;
    uint8_t sH;
    uint8_t sL;
    uint8_t sB;
    uint8_t gTicks;
    uint16_t sSP;
};
M1DbgEntry m1Dbg[M1_DBG_MAX];
int m1DbgCount = 0;
bool m1DbgActive = false;

// Milestone tracking: print once when softPC reaches key addresses
uint8_t milestoneHit = 0;  // bitmask: bit0=F600, bit1=F633, bit2=F690, bit3=E000(CCP)

////////////////////////////////////////////////////////////////////
// Processor Initialization
////////////////////////////////////////////////////////////////////
void uP_init() {
    // Data bus as input
    for (int i = 0; i < 8; i++) pinMode(DATA_PINS[i], INPUT);
    dataBusIsOutput = false;

    // Address bus as input
    for (int i = 0; i < 8; i++) {
        pinMode(ADDR_L_PINS[i], INPUT);
        pinMode(ADDR_H_PINS[i], INPUT);
    }

    pinMode(uP_RESET_N, OUTPUT);
    pinMode(uP_WR_N, INPUT);
    pinMode(uP_RD_N, INPUT);
    pinMode(uP_MREQ_N, INPUT);
    pinMode(uP_IORQ_N, INPUT);
    pinMode(uP_INT_N, OUTPUT);
    pinMode(uP_NMI_N, OUTPUT);
    pinMode(uP_CLK, OUTPUT);

    uP_assert_reset();
    digitalWrite(uP_CLK, LOW);

    clock_cycle_count = 0;
}

void uP_assert_reset() {
    digitalWrite(uP_RESET_N, LOW);
    digitalWrite(uP_INT_N, HIGH);
    digitalWrite(uP_NMI_N, HIGH);
}

void uP_release_reset() {
    digitalWrite(uP_RESET_N, HIGH);
}

////////////////////////////////////////////////////////////////////
// IO Handlers (called from instruction-tracking state machine)
////////////////////////////////////////////////////////////////////
uint8_t handle_io_read(uint8_t port) {
    uint8_t data = 0xFF;

    if (disk_handles_port(port)) {
        data = disk_read_port(port);
    }
    else if (port == ADDR_6850_DATA) {
        data = reg6850_DATA_RX = Serial.read();
    }
    else if (port == ADDR_6850_CONTROL) {
        data = reg6850_STATUS;
    }

    // Data bus driving is handled by the IO_IN_DRIVING state
    // in cpu_tick's MREQ-inactive handler.
    prevDATA = data;
    return data;
}

void handle_io_write(uint8_t port, uint8_t data) {
    if (disk_handles_port(port)) {
        disk_write_port(port, data);
    }
    else if (port == ADDR_6850_DATA) {
        reg6850_DATA_TX = data;
        reg6850_STATUS = reg6850_STATUS & 0b11111101;
        Serial.write(reg6850_DATA_TX);
        reg6850_STATUS = reg6850_STATUS | 0b00000010;
    }
    else if (port == ADDR_6850_CONTROL) {
        reg6850_CONTROL = data;
    }
    prevDATA = data;
}

////////////////////////////////////////////////////////////////////
// CPU Tick - Main Processing Loop
//
// Guard-only M1 detection:
//   No MREQ transition tracking. After each M1, m1Guard is set
//   based on the instruction's T-state count. The next MREQ-active
//   read after the guard expires is the next M1.
//
//   IO handling:
//   - OUT: handled immediately at M1 (port+data from z80RAM/shadowA)
//   - IN:  response driven during IO cycle via IO_IN_PENDING state
//   - Missed OUT after conditional RET: retroactive scan
////////////////////////////////////////////////////////////////////
void __attribute__((noinline)) cpu_tick() {
    // Check for serial input (ACIA RX) — only every 256 ticks
    // to reduce overhead. Still responsive for interactive typing.
    if ((clock_cycle_count & 0xFF) == 0) {
        if (!CONTROL_RTS_STATE && Serial.available()) {
            reg6850_STATUS |= 0b00000001;
            if (CONTROL_RX_INT_ENABLE) {
                INT_N_LOW;
            }
        } else {
            reg6850_STATUS &= 0b11111110;
            INT_N_HIGH;
        }
    }

    CLK_HIGH;
    busSettle();  // ~50ns for TXB0108 signal propagation

    bool mreq_active = !STATE_MREQ_N;
    bool wr_active   = !STATE_WR_N;

    //////////////////////////////////////////////////////////////////////
    // IO_IN_DRIVING: keep driving response until next M1 arrives.
    //////////////////////////////////////////////////////////////////////
    if (ioState == IO_IN_DRIVING) {
        if (mreq_active && clock_cycle_count >= m1Guard) {
            // Next M1 cycle arrived — stop driving, fall through
            setDataBusInput();
            ioState = IO_IDLE;
        } else {
            // Keep driving IO response
            CLK_LOW;
            clock_cycle_count++;
            return;
        }
    }

    //////////////////////////////////////////////////////////////////////
    // IO_IN_PENDING: waiting for IO cycle. Serve memory normally
    // during M2 (operand read). Start driving when MREQ goes
    // inactive and enough ticks have elapsed.
    //////////////////////////////////////////////////////////////////////
    if (ioState == IO_IN_PENDING) {
        if (!mreq_active && clock_cycle_count >= ioInDriveAfter) {
            // IO cycle reached — drive response onto data bus
            setDataBusOutput();
            writeDataBus(ioResponse);
            ioState = IO_IN_DRIVING;
            CLK_LOW;
            clock_cycle_count++;
            return;
        }
        // Fall through to normal memory handling (serves M2 operand)
    }

    //////////////////////////////////////////////////////////////////////
    // MREQ inactive (not IO_IN_PENDING) — nothing to do
    //////////////////////////////////////////////////////////////////////
    if (!mreq_active) {
        CLK_LOW;
        clock_cycle_count++;
        return;
    }

    //////////////////////////////////////////////////////////////////////
    // Memory Access (MREQ active)
    //////////////////////////////////////////////////////////////////////
    uP_ADDR = readAddress();

    if (!wr_active) {
        // Memory Read — serve data from z80RAM
        setDataBusOutput();
        byte data = z80RAM[uP_ADDR];
        writeDataBus(data);

        /////////////////////////////////////////////////////////////
        // M1 detection: guard-only + softPC
        //
        // When the guard expires, the next MREQ-active read is M1.
        // Use softPC (not the address bus) for opcode identification
        // because the address bus lags through TXB0108.
        /////////////////////////////////////////////////////////////
        if (clock_cycle_count >= m1Guard) {
            // Apply deferred write from previous read-modify-write instruction
            // (INC/DEC (HL), CB-prefix on (HL)). This must happen before anything
            // else at M1 time — the Z80 has finished executing the instruction now.
            if (deferredWriteAddr != 0xFFFF) {
                z80RAM[deferredWriteAddr] = deferredWriteVal;
                deferredWriteAddr = 0xFFFF;
            }

            // Resolve conditional branch using shadow flags
            if (softPC_alt != 0xFFFF) {
                // Evaluate the condition from the saved opcode
                bool taken = false;
                uint8_t cc = softPC_alt_opcode;

                if (cc == 0x10) {
                    // DJNZ: taken if B != 0 (B was already decremented by Z80)
                    // We track B in shadow — decrement it here and test
                    shadowB--;
                    taken = (shadowB != 0);
                } else {
                    // Extract condition code from opcode:
                    // RET cc: 0xC0|cc*8  JP cc: 0xC2|cc*8  CALL cc: 0xC4|cc*8
                    // JR cc:  0x20|cc*8 (cc=0..3 only: NZ/Z/NC/C)
                    uint8_t cond;
                    if (cc >= 0x20 && cc <= 0x38) {
                        cond = (cc - 0x20) >> 3;  // 0=NZ, 1=Z, 2=NC, 3=C
                    } else {
                        cond = (cc >> 3) & 0x07;  // bits 5..3
                    }
                    // cond: 0=NZ, 1=Z, 2=NC, 3=C, 4=PO, 5=PE, 6=P, 7=M
                    switch (cond) {
                        case 0: taken = !(shadowF & FLAG_Z); break;  // NZ
                        case 1: taken =  (shadowF & FLAG_Z); break;  // Z
                        case 2: taken = !(shadowF & FLAG_C); break;  // NC
                        case 3: taken =  (shadowF & FLAG_C); break;  // C
                        case 4: taken = !(shadowF & FLAG_PV); break; // PO
                        case 5: taken =  (shadowF & FLAG_PV); break; // PE
                        case 6: taken = !(shadowF & FLAG_S); break;  // P
                        case 7: taken =  (shadowF & FLAG_S); break;  // M
                    }
                }

                uint8_t picked = taken ? 1 : 0;
                if (brDbgCount < BR_DBG_MAX)
                    brDbg[brDbgCount++] = {clock_cycle_count, (uint16_t)cc, softPC, softPC_alt, (uint16_t)(shadowF), 0, picked};

                if (taken) {
                    // Taken path — apply deferred SP adjustment
                    if (softPC_alt_spDelta > 0) {
                        softSP += softPC_alt_spDelta;  // RET cc taken: pop
                    } else if (softPC_alt_spDelta < 0) {
                        softSP += softPC_alt_spDelta;  // CALL cc taken: push (negative delta)
                        z80RAM[softSP] = softPC_alt_retAddr & 0xFF;
                        z80RAM[softSP + 1] = softPC_alt_retAddr >> 8;
                    }
                    softPC = softPC_alt;
                }
                softPC_alt = 0xFFFF;  // consumed
            }

            // Use softPC for opcode — immune to TXB0108 address lag
            uint8_t opcode = z80RAM[softPC];
            uP_ADDR = softPC;  // for debug logging
            uint8_t m1_shadowA_before = shadowA;


            // Also drive the correct opcode on the data bus
            // (in case TXB0108 address lag caused wrong data earlier)
            writeDataBus(opcode);

            // Retroactive IO scan: if previous M1 was a conditional
            // RET and we fell through (not taken), scan the gap for
            // any OUT instructions we missed due to the long guard.
            if (checkMissedIO) {
                checkMissedIO = false;
                if (softPC > missedIOScanStart &&
                    (softPC - missedIOScanStart) <= 6) {
                    for (uint16_t s = missedIOScanStart; s + 1 < softPC; s++) {
                        if (z80RAM[s] == 0xD3) {
                            uint8_t mport = z80RAM[s + 1];
                            handle_io_write(mport, shadowA);
                            s++; // skip port byte
                        }
                    }
                }
            }

            // Handle IO instructions immediately
            if (opcode == 0xD3) {
                // OUT (n), A — port from operand byte, data from shadowA
                uint8_t port = z80RAM[softPC + 1];
                handle_io_write(port, shadowA);
            }
            else if (opcode == 0xDB) {
                // IN A, (n) — port from operand byte
                uint8_t port = z80RAM[softPC + 1];
                ioResponse = handle_io_read(port);
                shadowA = ioResponse;  // IN A,(n) loads A
                ioInDriveAfter = clock_cycle_count + 6;
                ioState = IO_IN_PENDING;
            }

            // Shadow register + flag tracking (uses softPC for operand refs)
            switch (opcode) {
                // ---- LD register instructions (no flag changes) ----
                case 0x3E: shadowA = z80RAM[softPC + 1]; break;  // LD A, n
                case 0x79: shadowA = shadowC; break;               // LD A, C
                case 0x78: shadowA = shadowB; break;               // LD A, B
                case 0x7A: shadowA = shadowD; break;               // LD A, D
                case 0x7B: shadowA = shadowE; break;               // LD A, E
                case 0x7C: shadowA = shadowH; break;               // LD A, H
                case 0x7D: shadowA = shadowL; break;               // LD A, L
                case 0x7E: shadowA = z80RAM[(shadowH << 8) | shadowL]; break; // LD A, (HL)
                case 0x7F: break;                                    // LD A, A (nop)
                case 0x47: shadowB = shadowA; break;               // LD B, A
                case 0x40: break;                                    // LD B, B (nop)
                case 0x41: shadowB = shadowC; break;               // LD B, C
                case 0x42: shadowB = shadowD; break;               // LD B, D
                case 0x43: shadowB = shadowE; break;               // LD B, E
                case 0x44: shadowB = shadowH; break;               // LD B, H
                case 0x45: shadowB = shadowL; break;               // LD B, L
                case 0x46: shadowB = z80RAM[(shadowH << 8) | shadowL]; break; // LD B, (HL)
                case 0x06: shadowB = z80RAM[softPC + 1]; break;  // LD B, n
                case 0x4F: shadowC = shadowA; break;               // LD C, A
                case 0x48: shadowC = shadowB; break;               // LD C, B
                case 0x49: break;                                    // LD C, C (nop)
                case 0x4A: shadowC = shadowD; break;               // LD C, D
                case 0x4B: shadowC = shadowE; break;               // LD C, E
                case 0x4C: shadowC = shadowH; break;               // LD C, H
                case 0x4D: shadowC = shadowL; break;               // LD C, L
                case 0x4E: shadowC = z80RAM[(shadowH << 8) | shadowL]; break; // LD C, (HL)
                case 0x0E: shadowC = z80RAM[softPC + 1]; break;  // LD C, n
                case 0x57: shadowD = shadowA; break;               // LD D, A
                case 0x50: shadowD = shadowB; break;               // LD D, B
                case 0x51: shadowD = shadowC; break;               // LD D, C
                case 0x53: shadowD = shadowE; break;               // LD D, E
                case 0x54: shadowD = shadowH; break;               // LD D, H
                case 0x55: shadowD = shadowL; break;               // LD D, L
                case 0x56: shadowD = z80RAM[(shadowH << 8) | shadowL]; break; // LD D, (HL)
                case 0x16: shadowD = z80RAM[softPC + 1]; break;  // LD D, n
                case 0x5F: shadowE = shadowA; break;               // LD E, A
                case 0x58: shadowE = shadowB; break;               // LD E, B
                case 0x59: shadowE = shadowC; break;               // LD E, C
                case 0x5A: shadowE = shadowD; break;               // LD E, D
                case 0x5C: shadowE = shadowH; break;               // LD E, H
                case 0x5D: shadowE = shadowL; break;               // LD E, L
                case 0x5E: shadowE = z80RAM[(shadowH << 8) | shadowL]; break; // LD E, (HL)
                case 0x1E: shadowE = z80RAM[softPC + 1]; break;  // LD E, n
                case 0x67: shadowH = shadowA; break;               // LD H, A
                case 0x60: shadowH = shadowB; break;               // LD H, B
                case 0x61: shadowH = shadowC; break;               // LD H, C
                case 0x62: shadowH = shadowD; break;               // LD H, D
                case 0x63: shadowH = shadowE; break;               // LD H, E
                case 0x65: shadowH = shadowL; break;               // LD H, L
                case 0x66: shadowH = z80RAM[(shadowH << 8) | shadowL]; break; // LD H, (HL)
                case 0x26: shadowH = z80RAM[softPC + 1]; break;  // LD H, n
                case 0x6F: shadowL = shadowA; break;               // LD L, A
                case 0x68: shadowL = shadowB; break;               // LD L, B
                case 0x69: shadowL = shadowC; break;               // LD L, C
                case 0x6A: shadowL = shadowD; break;               // LD L, D
                case 0x6B: shadowL = shadowE; break;               // LD L, E
                case 0x6C: shadowL = shadowH; break;               // LD L, H
                case 0x6E: shadowL = z80RAM[(shadowH << 8) | shadowL]; break; // LD L, (HL)
                case 0x2E: shadowL = z80RAM[softPC + 1]; break;  // LD L, n
                case 0x21: {  // LD HL, nn
                    shadowL = z80RAM[softPC + 1];
                    shadowH = z80RAM[softPC + 2];
                    break;
                }
                case 0x11: {  // LD DE, nn
                    shadowE = z80RAM[softPC + 1];
                    shadowD = z80RAM[softPC + 2];
                    break;
                }
                case 0x01: {  // LD BC, nn
                    shadowC = z80RAM[softPC + 1];
                    shadowB = z80RAM[softPC + 2];
                    break;
                }
                case 0x0A: shadowA = z80RAM[(shadowB << 8) | shadowC]; break; // LD A, (BC)
                case 0x1A: shadowA = z80RAM[(shadowD << 8) | shadowE]; break; // LD A, (DE)
                case 0x3A: {  // LD A, (nn)
                    uint16_t addr = z80RAM[softPC + 1] | (z80RAM[softPC + 2] << 8);
                    shadowA = z80RAM[addr];
                    break;
                }
                case 0x2A: {  // LD HL, (nn)
                    uint16_t addr = z80RAM[softPC + 1] | (z80RAM[softPC + 2] << 8);
                    shadowL = z80RAM[addr];
                    shadowH = z80RAM[addr + 1];
                    break;
                }

                // ---- Memory store pre-writes ----
                // TXB0108 address bus lag means Z80 bus writes go to wrong
                // z80RAM addresses. Pre-write correct values at M1 time.
                case 0x32: {  // LD (nn), A
                    uint16_t addr = z80RAM[softPC + 1] | (z80RAM[softPC + 2] << 8);
                    z80RAM[addr] = shadowA;
                    break;
                }
                case 0x22: {  // LD (nn), HL
                    uint16_t addr = z80RAM[softPC + 1] | (z80RAM[softPC + 2] << 8);
                    z80RAM[addr] = shadowL;
                    z80RAM[addr + 1] = shadowH;
                    break;
                }
                case 0x02:  // LD (BC), A
                    z80RAM[(shadowB << 8) | shadowC] = shadowA;
                    break;
                case 0x12:  // LD (DE), A
                    z80RAM[(shadowD << 8) | shadowE] = shadowA;
                    break;
                case 0x70:  // LD (HL), B
                    z80RAM[(shadowH << 8) | shadowL] = shadowB;
                    break;
                case 0x71:  // LD (HL), C
                    z80RAM[(shadowH << 8) | shadowL] = shadowC;
                    break;
                case 0x72:  // LD (HL), D
                    z80RAM[(shadowH << 8) | shadowL] = shadowD;
                    break;
                case 0x73:  // LD (HL), E
                    z80RAM[(shadowH << 8) | shadowL] = shadowE;
                    break;
                case 0x74:  // LD (HL), H
                    z80RAM[(shadowH << 8) | shadowL] = shadowH;
                    break;
                case 0x75:  // LD (HL), L
                    z80RAM[(shadowH << 8) | shadowL] = shadowL;
                    break;
                case 0x77:  // LD (HL), A
                    z80RAM[(shadowH << 8) | shadowL] = shadowA;
                    break;
                case 0x36: {  // LD (HL), n
                    z80RAM[(shadowH << 8) | shadowL] = z80RAM[softPC + 1];
                    break;
                }
                case 0xEB: {  // EX DE, HL
                    uint8_t t;
                    t = shadowD; shadowD = shadowH; shadowH = t;
                    t = shadowE; shadowE = shadowL; shadowL = t;
                    break;
                }
                case 0xE3: {  // EX (SP), HL — exchange HL with top of stack
                    // Pre-write: write shadowL to [SP], shadowH to [SP+1]
                    // Also update shadow HL from old stack values
                    uint8_t oldL = z80RAM[softSP];
                    uint8_t oldH = z80RAM[softSP + 1];
                    z80RAM[softSP] = shadowL;
                    z80RAM[softSP + 1] = shadowH;
                    shadowL = oldL;
                    shadowH = oldH;
                    break;
                }

                // ---- 16-bit INC/DEC (no flag changes) ----
                case 0x23: {  // INC HL
                    uint16_t hl = ((shadowH << 8) | shadowL) + 1;
                    shadowH = hl >> 8; shadowL = hl & 0xFF;
                    break;
                }
                case 0x2B: {  // DEC HL
                    uint16_t hl = ((shadowH << 8) | shadowL) - 1;
                    shadowH = hl >> 8; shadowL = hl & 0xFF;
                    break;
                }
                case 0x13: {  // INC DE
                    uint16_t de = ((shadowD << 8) | shadowE) + 1;
                    shadowD = de >> 8; shadowE = de & 0xFF;
                    break;
                }
                case 0x1B: {  // DEC DE
                    uint16_t de = ((shadowD << 8) | shadowE) - 1;
                    shadowD = de >> 8; shadowE = de & 0xFF;
                    break;
                }
                case 0x03: {  // INC BC
                    uint16_t bc = ((shadowB << 8) | shadowC) + 1;
                    shadowB = bc >> 8; shadowC = bc & 0xFF;
                    break;
                }
                case 0x0B: {  // DEC BC
                    uint16_t bc = ((shadowB << 8) | shadowC) - 1;
                    shadowB = bc >> 8; shadowC = bc & 0xFF;
                    break;
                }

                // ---- ADD HL, rr (16-bit add — affects H, N, C; preserves S, Z, P/V) ----
                case 0x09: {  // ADD HL, BC
                    uint16_t hl = (shadowH << 8) | shadowL;
                    uint16_t bc = (shadowB << 8) | shadowC;
                    uint32_t r = (uint32_t)hl + bc;
                    shadowH = (r >> 8) & 0xFF; shadowL = r & 0xFF;
                    shadowF = (shadowF & (FLAG_S | FLAG_Z | FLAG_PV)) |
                              ((r >> 16) ? FLAG_C : 0);
                    break;
                }
                case 0x19: {  // ADD HL, DE
                    uint16_t hl = (shadowH << 8) | shadowL;
                    uint16_t de = (shadowD << 8) | shadowE;
                    uint32_t r = (uint32_t)hl + de;
                    shadowH = (r >> 8) & 0xFF; shadowL = r & 0xFF;
                    shadowF = (shadowF & (FLAG_S | FLAG_Z | FLAG_PV)) |
                              ((r >> 16) ? FLAG_C : 0);
                    break;
                }
                case 0x29: {  // ADD HL, HL
                    uint16_t hl = (shadowH << 8) | shadowL;
                    uint32_t r = (uint32_t)hl + hl;
                    shadowH = (r >> 8) & 0xFF; shadowL = r & 0xFF;
                    shadowF = (shadowF & (FLAG_S | FLAG_Z | FLAG_PV)) |
                              ((r >> 16) ? FLAG_C : 0);
                    break;
                }
                case 0x39: {  // ADD HL, SP
                    uint16_t hl = (shadowH << 8) | shadowL;
                    uint32_t r = (uint32_t)hl + softSP;
                    shadowH = (r >> 8) & 0xFF; shadowL = r & 0xFF;
                    shadowF = (shadowF & (FLAG_S | FLAG_Z | FLAG_PV)) |
                              ((r >> 16) ? FLAG_C : 0);
                    break;
                }

                // ---- ALU operations that affect flags ----
                // XOR A (and XOR r)
                case 0xAF: shadowA = 0; shadowF = FLAG_Z | FLAG_PV; break;  // XOR A: always 0, Z=1, P=1
                case 0xA8: shadowA ^= shadowB; shadowF = flagsLogic(shadowA, false); break;  // XOR B
                case 0xA9: shadowA ^= shadowC; shadowF = flagsLogic(shadowA, false); break;  // XOR C
                case 0xAA: shadowA ^= shadowD; shadowF = flagsLogic(shadowA, false); break;  // XOR D
                case 0xAB: shadowA ^= shadowE; shadowF = flagsLogic(shadowA, false); break;  // XOR E
                case 0xAC: shadowA ^= shadowH; shadowF = flagsLogic(shadowA, false); break;  // XOR H
                case 0xAD: shadowA ^= shadowL; shadowF = flagsLogic(shadowA, false); break;  // XOR L
                case 0xAE: shadowA ^= z80RAM[(shadowH << 8) | shadowL]; shadowF = flagsLogic(shadowA, false); break; // XOR (HL)
                case 0xEE: shadowA ^= z80RAM[softPC + 1]; shadowF = flagsLogic(shadowA, false); break; // XOR n

                // OR A (and OR r)
                case 0xB7: shadowA |= 0; shadowF = flagsLogic(shadowA, false); break;  // OR A
                case 0xB0: shadowA |= shadowB; shadowF = flagsLogic(shadowA, false); break;  // OR B
                case 0xB1: shadowA |= shadowC; shadowF = flagsLogic(shadowA, false); break;  // OR C
                case 0xB2: shadowA |= shadowD; shadowF = flagsLogic(shadowA, false); break;  // OR D
                case 0xB3: shadowA |= shadowE; shadowF = flagsLogic(shadowA, false); break;  // OR E
                case 0xB4: shadowA |= shadowH; shadowF = flagsLogic(shadowA, false); break;  // OR H
                case 0xB5: shadowA |= shadowL; shadowF = flagsLogic(shadowA, false); break;  // OR L
                case 0xB6: shadowA |= z80RAM[(shadowH << 8) | shadowL]; shadowF = flagsLogic(shadowA, false); break; // OR (HL)
                case 0xF6: shadowA |= z80RAM[softPC + 1]; shadowF = flagsLogic(shadowA, false); break; // OR n

                // AND A (and AND r)
                case 0xA7: shadowF = flagsLogic(shadowA, true); break;  // AND A (A unchanged)
                case 0xA0: shadowA &= shadowB; shadowF = flagsLogic(shadowA, true); break;  // AND B
                case 0xA1: shadowA &= shadowC; shadowF = flagsLogic(shadowA, true); break;  // AND C
                case 0xA2: shadowA &= shadowD; shadowF = flagsLogic(shadowA, true); break;  // AND D
                case 0xA3: shadowA &= shadowE; shadowF = flagsLogic(shadowA, true); break;  // AND E
                case 0xA4: shadowA &= shadowH; shadowF = flagsLogic(shadowA, true); break;  // AND H
                case 0xA5: shadowA &= shadowL; shadowF = flagsLogic(shadowA, true); break;  // AND L
                case 0xA6: shadowA &= z80RAM[(shadowH << 8) | shadowL]; shadowF = flagsLogic(shadowA, true); break; // AND (HL)
                case 0xE6: shadowA &= z80RAM[softPC + 1]; shadowF = flagsLogic(shadowA, true); break; // AND n

                // CP (compare — like SUB but don't store result)
                case 0xBF: shadowF = flagsSub(shadowA, shadowA, 0); break;  // CP A
                case 0xB8: shadowF = flagsSub(shadowA, shadowB, 0); break;  // CP B
                case 0xB9: shadowF = flagsSub(shadowA, shadowC, 0); break;  // CP C
                case 0xBA: shadowF = flagsSub(shadowA, shadowD, 0); break;  // CP D
                case 0xBB: shadowF = flagsSub(shadowA, shadowE, 0); break;  // CP E
                case 0xBC: shadowF = flagsSub(shadowA, shadowH, 0); break;  // CP H
                case 0xBD: shadowF = flagsSub(shadowA, shadowL, 0); break;  // CP L
                case 0xBE: shadowF = flagsSub(shadowA, z80RAM[(shadowH << 8) | shadowL], 0); break; // CP (HL)
                case 0xFE: shadowF = flagsSub(shadowA, z80RAM[softPC + 1], 0); break; // CP n

                // ADD A, r
                case 0x87: shadowF = flagsAdd(shadowA, shadowA, 0); shadowA += shadowA; break;  // ADD A, A
                case 0x80: shadowF = flagsAdd(shadowA, shadowB, 0); shadowA += shadowB; break;  // ADD A, B
                case 0x81: shadowF = flagsAdd(shadowA, shadowC, 0); shadowA += shadowC; break;  // ADD A, C
                case 0x82: shadowF = flagsAdd(shadowA, shadowD, 0); shadowA += shadowD; break;  // ADD A, D
                case 0x83: shadowF = flagsAdd(shadowA, shadowE, 0); shadowA += shadowE; break;  // ADD A, E
                case 0x84: shadowF = flagsAdd(shadowA, shadowH, 0); shadowA += shadowH; break;  // ADD A, H
                case 0x85: shadowF = flagsAdd(shadowA, shadowL, 0); shadowA += shadowL; break;  // ADD A, L
                case 0x86: { uint8_t v = z80RAM[(shadowH << 8) | shadowL]; shadowF = flagsAdd(shadowA, v, 0); shadowA += v; break; } // ADD A, (HL)
                case 0xC6: { uint8_t v = z80RAM[softPC + 1]; shadowF = flagsAdd(shadowA, v, 0); shadowA += v; break; } // ADD A, n

                // SUB r
                case 0x97: shadowF = flagsSub(shadowA, shadowA, 0); shadowA = 0; break;  // SUB A
                case 0x90: shadowF = flagsSub(shadowA, shadowB, 0); shadowA -= shadowB; break;  // SUB B
                case 0x91: shadowF = flagsSub(shadowA, shadowC, 0); shadowA -= shadowC; break;  // SUB C
                case 0x92: shadowF = flagsSub(shadowA, shadowD, 0); shadowA -= shadowD; break;  // SUB D
                case 0x93: shadowF = flagsSub(shadowA, shadowE, 0); shadowA -= shadowE; break;  // SUB E
                case 0x94: shadowF = flagsSub(shadowA, shadowH, 0); shadowA -= shadowH; break;  // SUB H
                case 0x95: shadowF = flagsSub(shadowA, shadowL, 0); shadowA -= shadowL; break;  // SUB L
                case 0x96: { uint8_t v = z80RAM[(shadowH << 8) | shadowL]; shadowF = flagsSub(shadowA, v, 0); shadowA -= v; break; } // SUB (HL)
                case 0xD6: { uint8_t v = z80RAM[softPC + 1]; shadowF = flagsSub(shadowA, v, 0); shadowA -= v; break; } // SUB n

                // ADC A, r (add with carry)
                case 0x8F: { uint8_t c = shadowF & FLAG_C ? 1 : 0; shadowF = flagsAdd(shadowA, shadowA, c); shadowA = shadowA + shadowA + c; break; }
                case 0x88: { uint8_t c = shadowF & FLAG_C ? 1 : 0; shadowF = flagsAdd(shadowA, shadowB, c); shadowA += shadowB + c; break; }
                case 0x89: { uint8_t c = shadowF & FLAG_C ? 1 : 0; shadowF = flagsAdd(shadowA, shadowC, c); shadowA += shadowC + c; break; }
                case 0x8A: { uint8_t c = shadowF & FLAG_C ? 1 : 0; shadowF = flagsAdd(shadowA, shadowD, c); shadowA += shadowD + c; break; }
                case 0x8B: { uint8_t c = shadowF & FLAG_C ? 1 : 0; shadowF = flagsAdd(shadowA, shadowE, c); shadowA += shadowE + c; break; }
                case 0x8C: { uint8_t c = shadowF & FLAG_C ? 1 : 0; shadowF = flagsAdd(shadowA, shadowH, c); shadowA += shadowH + c; break; }
                case 0x8D: { uint8_t c = shadowF & FLAG_C ? 1 : 0; shadowF = flagsAdd(shadowA, shadowL, c); shadowA += shadowL + c; break; }
                case 0x8E: { uint8_t v = z80RAM[(shadowH << 8) | shadowL]; uint8_t c = shadowF & FLAG_C ? 1 : 0; shadowF = flagsAdd(shadowA, v, c); shadowA += v + c; break; }
                case 0xCE: { uint8_t v = z80RAM[softPC + 1]; uint8_t c = shadowF & FLAG_C ? 1 : 0; shadowF = flagsAdd(shadowA, v, c); shadowA += v + c; break; } // ADC A, n

                // SBC A, r (subtract with carry)
                case 0x9F: { uint8_t c = shadowF & FLAG_C ? 1 : 0; shadowF = flagsSub(shadowA, shadowA, c); shadowA = shadowA - shadowA - c; break; }
                case 0x98: { uint8_t c = shadowF & FLAG_C ? 1 : 0; shadowF = flagsSub(shadowA, shadowB, c); shadowA -= shadowB + c; break; }
                case 0x99: { uint8_t c = shadowF & FLAG_C ? 1 : 0; shadowF = flagsSub(shadowA, shadowC, c); shadowA -= shadowC + c; break; }
                case 0x9A: { uint8_t c = shadowF & FLAG_C ? 1 : 0; shadowF = flagsSub(shadowA, shadowD, c); shadowA -= shadowD + c; break; }
                case 0x9B: { uint8_t c = shadowF & FLAG_C ? 1 : 0; shadowF = flagsSub(shadowA, shadowE, c); shadowA -= shadowE + c; break; }
                case 0x9C: { uint8_t c = shadowF & FLAG_C ? 1 : 0; shadowF = flagsSub(shadowA, shadowH, c); shadowA -= shadowH + c; break; }
                case 0x9D: { uint8_t c = shadowF & FLAG_C ? 1 : 0; shadowF = flagsSub(shadowA, shadowL, c); shadowA -= shadowL + c; break; }
                case 0x9E: { uint8_t v = z80RAM[(shadowH << 8) | shadowL]; uint8_t c = shadowF & FLAG_C ? 1 : 0; shadowF = flagsSub(shadowA, v, c); shadowA -= v + c; break; }
                case 0xDE: { uint8_t v = z80RAM[softPC + 1]; uint8_t c = shadowF & FLAG_C ? 1 : 0; shadowF = flagsSub(shadowA, v, c); shadowA -= v + c; break; } // SBC A, n

                // INC r (affects S, Z, H, P/V, N; C unchanged)
                case 0x3C: shadowA++; shadowF = flagsInc(shadowA, shadowF); break;  // INC A
                case 0x04: shadowB++; shadowF = flagsInc(shadowB, shadowF); break;  // INC B
                case 0x0C: shadowC++; shadowF = flagsInc(shadowC, shadowF); break;  // INC C
                case 0x14: shadowD++; shadowF = flagsInc(shadowD, shadowF); break;  // INC D
                case 0x1C: shadowE++; shadowF = flagsInc(shadowE, shadowF); break;  // INC E
                case 0x24: shadowH++; shadowF = flagsInc(shadowH, shadowF); break;  // INC H
                case 0x2C: shadowL++; shadowF = flagsInc(shadowL, shadowF); break;  // INC L
                case 0x34: {  // INC (HL) — deferred write
                    uint16_t addr = (shadowH << 8) | shadowL;
                    uint8_t val = z80RAM[addr] + 1;
                    shadowF = flagsInc(val, shadowF);
                    deferredWriteAddr = addr;
                    deferredWriteVal = val;
                    break;
                }

                // DEC r (affects S, Z, H, P/V, N; C unchanged)
                case 0x3D: shadowA--; shadowF = flagsDec(shadowA, shadowF); break;  // DEC A
                case 0x05: shadowB--; shadowF = flagsDec(shadowB, shadowF); break;  // DEC B
                case 0x0D: shadowC--; shadowF = flagsDec(shadowC, shadowF); break;  // DEC C
                case 0x15: shadowD--; shadowF = flagsDec(shadowD, shadowF); break;  // DEC D
                case 0x1D: shadowE--; shadowF = flagsDec(shadowE, shadowF); break;  // DEC E
                case 0x25: shadowH--; shadowF = flagsDec(shadowH, shadowF); break;  // DEC H
                case 0x2D: shadowL--; shadowF = flagsDec(shadowL, shadowF); break;  // DEC L
                case 0x35: {  // DEC (HL) — deferred write
                    uint16_t addr = (shadowH << 8) | shadowL;
                    uint8_t val = z80RAM[addr] - 1;
                    shadowF = flagsDec(val, shadowF);
                    deferredWriteAddr = addr;
                    deferredWriteVal = val;
                    break;
                }

                // DAA — decimal adjust accumulator
                case 0x27: {
                    uint8_t a = shadowA;
                    uint8_t correction = 0;
                    bool newC = false;
                    if ((shadowF & FLAG_H) || (a & 0x0F) > 9) correction |= 0x06;
                    if ((shadowF & FLAG_C) || a > 0x99) { correction |= 0x60; newC = true; }
                    if (shadowF & FLAG_N) {
                        shadowA -= correction;
                    } else {
                        shadowA += correction;
                    }
                    shadowF = (shadowA & FLAG_S) | (shadowA ? 0 : FLAG_Z) |
                              (parityTable[shadowA] ? FLAG_PV : 0) |
                              (shadowF & FLAG_N) |
                              (newC ? FLAG_C : 0) |
                              (((a ^ shadowA) & 0x10) ? FLAG_H : 0);
                    break;
                }

                // CPL (complement A) — sets H and N, preserves S/Z/P/C
                case 0x2F:
                    shadowA = ~shadowA;
                    shadowF = (shadowF & (FLAG_S | FLAG_Z | FLAG_PV | FLAG_C)) | FLAG_H | FLAG_N;
                    break;

                // SCF (set carry flag) — C=1, H=0, N=0, preserves S/Z/P
                case 0x37:
                    shadowF = (shadowF & (FLAG_S | FLAG_Z | FLAG_PV)) | FLAG_C;
                    break;

                // CCF (complement carry flag) — C=~C, H=old_C, N=0, preserves S/Z/P
                case 0x3F: {
                    uint8_t oldC = shadowF & FLAG_C;
                    shadowF = (shadowF & (FLAG_S | FLAG_Z | FLAG_PV)) | (oldC ? FLAG_H : FLAG_C);
                    break;
                }

                // RLCA/RRCA/RLA/RRA — rotate A, affect C/H/N only
                case 0x07: {  // RLCA
                    uint8_t bit7 = shadowA >> 7;
                    shadowA = (shadowA << 1) | bit7;
                    shadowF = (shadowF & (FLAG_S | FLAG_Z | FLAG_PV)) | (bit7 ? FLAG_C : 0);
                    break;
                }
                case 0x0F: {  // RRCA
                    uint8_t bit0 = shadowA & 1;
                    shadowA = (shadowA >> 1) | (bit0 << 7);
                    shadowF = (shadowF & (FLAG_S | FLAG_Z | FLAG_PV)) | (bit0 ? FLAG_C : 0);
                    break;
                }
                case 0x17: {  // RLA
                    uint8_t bit7 = shadowA >> 7;
                    shadowA = (shadowA << 1) | ((shadowF & FLAG_C) ? 1 : 0);
                    shadowF = (shadowF & (FLAG_S | FLAG_Z | FLAG_PV)) | (bit7 ? FLAG_C : 0);
                    break;
                }
                case 0x1F: {  // RRA
                    uint8_t bit0 = shadowA & 1;
                    shadowA = (shadowA >> 1) | ((shadowF & FLAG_C) ? 0x80 : 0);
                    shadowF = (shadowF & (FLAG_S | FLAG_Z | FLAG_PV)) | (bit0 ? FLAG_C : 0);
                    break;
                }

                // PUSH/POP shadow tracking
                //
                // The TXB0108 address bus propagation delay means the
                // Z80's stack writes (during PUSH) may go to wrong
                // addresses in z80RAM. To compensate, we pre-write
                // shadow values to the correct stack location at M1
                // time, and read them back during POP.
                //
                // Shadow tracking runs BEFORE computeNextPC, so softSP
                // still points to the pre-PUSH/POP position.
                // PUSH: writes high byte at [SP-1], low byte at [SP-2]
                // POP: reads low byte from [SP], high byte from [SP+1]

                // PUSH AF
                case 0xF5:
                    z80RAM[softSP - 1] = shadowA;
                    z80RAM[softSP - 2] = shadowF;
                    break;

                // PUSH BC
                case 0xC5:
                    z80RAM[softSP - 1] = shadowB;
                    z80RAM[softSP - 2] = shadowC;
                    break;

                // PUSH DE
                case 0xD5:
                    z80RAM[softSP - 1] = shadowD;
                    z80RAM[softSP - 2] = shadowE;
                    break;

                // PUSH HL
                case 0xE5:
                    z80RAM[softSP - 1] = shadowH;
                    z80RAM[softSP - 2] = shadowL;
                    break;

                // POP AF
                case 0xF1:
                    shadowF = z80RAM[softSP];
                    shadowA = z80RAM[softSP + 1];
                    break;

                // POP BC
                case 0xC1:
                    shadowC = z80RAM[softSP];
                    shadowB = z80RAM[softSP + 1];
                    break;

                // POP DE
                case 0xD1:
                    shadowE = z80RAM[softSP];
                    shadowD = z80RAM[softSP + 1];
                    break;

                // POP HL
                case 0xE1:
                    shadowL = z80RAM[softSP];
                    shadowH = z80RAM[softSP + 1];
                    break;

                // DI/EI — no register/flag effects we care about
                case 0xF3: case 0xFB: break;

                // NOP / HALT
                case 0x00: case 0x76: break;

                // ---- ED-prefixed instructions ----
                case 0xED: {
                    uint8_t ed_op = z80RAM[softPC + 1];
                    switch (ed_op) {
                        // NEG — A = 0 - A
                        case 0x44: {
                            shadowF = flagsSub(0, shadowA, 0);
                            shadowA = -shadowA;
                            break;
                        }
                        // LD I,A / LD R,A — no shadow effect
                        case 0x47: case 0x4F: break;
                        // LD A,I / LD A,R — we can't track I/R, but set flags
                        case 0x57: case 0x5F: {
                            // P/V = IFF2 (unknown), approximate: preserve A, set flags
                            shadowF = (shadowA & FLAG_S) | (shadowA ? 0 : FLAG_Z) |
                                      (shadowF & (FLAG_C | FLAG_PV));  // preserve C, approximate P/V
                            break;
                        }
                        // LD (nn),BC — pre-write
                        case 0x43: {
                            uint16_t addr = z80RAM[softPC + 2] | (z80RAM[softPC + 3] << 8);
                            z80RAM[addr] = shadowC;
                            z80RAM[addr + 1] = shadowB;
                            break;
                        }
                        // LD BC,(nn)
                        case 0x4B: {
                            uint16_t addr = z80RAM[softPC + 2] | (z80RAM[softPC + 3] << 8);
                            shadowC = z80RAM[addr];
                            shadowB = z80RAM[addr + 1];
                            break;
                        }
                        // LD (nn),DE — pre-write
                        case 0x53: {
                            uint16_t addr = z80RAM[softPC + 2] | (z80RAM[softPC + 3] << 8);
                            z80RAM[addr] = shadowE;
                            z80RAM[addr + 1] = shadowD;
                            break;
                        }
                        // LD DE,(nn)
                        case 0x5B: {
                            uint16_t addr = z80RAM[softPC + 2] | (z80RAM[softPC + 3] << 8);
                            shadowE = z80RAM[addr];
                            shadowD = z80RAM[addr + 1];
                            break;
                        }
                        // LD (nn),HL (ED 63) — pre-write
                        case 0x63: {
                            uint16_t addr = z80RAM[softPC + 2] | (z80RAM[softPC + 3] << 8);
                            z80RAM[addr] = shadowL;
                            z80RAM[addr + 1] = shadowH;
                            break;
                        }
                        // LD HL,(nn) (ED 6B)
                        case 0x6B: {
                            uint16_t addr = z80RAM[softPC + 2] | (z80RAM[softPC + 3] << 8);
                            shadowL = z80RAM[addr];
                            shadowH = z80RAM[addr + 1];
                            break;
                        }
                        // LD (nn),SP — pre-write
                        case 0x73: {
                            uint16_t addr = z80RAM[softPC + 2] | (z80RAM[softPC + 3] << 8);
                            z80RAM[addr] = softSP & 0xFF;
                            z80RAM[addr + 1] = softSP >> 8;
                            break;
                        }
                        // LD SP,(nn)
                        case 0x7B: {
                            uint16_t addr = z80RAM[softPC + 2] | (z80RAM[softPC + 3] << 8);
                            softSP = z80RAM[addr] | (z80RAM[addr + 1] << 8);
                            break;
                        }
                        // LDIR / LDI — Z80 does the actual transfer; we track one iteration
                        // For LDIR (B0), Z80 re-executes at same PC until BC=0.
                        // Each M1 detection = one iteration. computeNextPC handles repeat.
                        case 0xB0: case 0xA0: {
                            uint16_t hl = (shadowH << 8) | shadowL;
                            uint16_t de = (shadowD << 8) | shadowE;
                            uint16_t bc = (shadowB << 8) | shadowC;
                            // Pre-write: copy (HL) to (DE) in z80RAM
                            z80RAM[de] = z80RAM[hl];
                            // Single iteration for both LDI and LDIR
                            hl++; de++; bc--;
                            shadowH = hl >> 8; shadowL = hl & 0xFF;
                            shadowD = de >> 8; shadowE = de & 0xFF;
                            shadowB = bc >> 8; shadowC = bc & 0xFF;
                            // P/V = (BC != 0), H=0, N=0
                            shadowF = (shadowF & (FLAG_S | FLAG_Z | FLAG_C)) | (bc ? FLAG_PV : 0);
                            break;
                        }
                        // LDDR / LDD — single iteration, computeNextPC handles repeat
                        case 0xB8: case 0xA8: {
                            uint16_t hl = (shadowH << 8) | shadowL;
                            uint16_t de = (shadowD << 8) | shadowE;
                            uint16_t bc = (shadowB << 8) | shadowC;
                            // Pre-write: copy (HL) to (DE) in z80RAM
                            z80RAM[de] = z80RAM[hl];
                            hl--; de--; bc--;
                            shadowH = hl >> 8; shadowL = hl & 0xFF;
                            shadowD = de >> 8; shadowE = de & 0xFF;
                            shadowB = bc >> 8; shadowC = bc & 0xFF;
                            shadowF = (shadowF & (FLAG_S | FLAG_Z | FLAG_C)) | (bc ? FLAG_PV : 0);
                            break;
                        }
                        // CPIR / CPI — single iteration, computeNextPC handles repeat
                        case 0xB1: case 0xA1: {
                            uint16_t hl = (shadowH << 8) | shadowL;
                            uint16_t bc = (shadowB << 8) | shadowC;
                            uint8_t result = shadowA - z80RAM[hl];
                            hl++; bc--;
                            shadowH = hl >> 8; shadowL = hl & 0xFF;
                            shadowB = bc >> 8; shadowC = bc & 0xFF;
                            // S, Z from result; P/V = (BC != 0); N=1; C unchanged
                            shadowF = (result & FLAG_S) | (result ? 0 : FLAG_Z) |
                                      FLAG_N | (bc ? FLAG_PV : 0) | (shadowF & FLAG_C);
                            break;
                        }
                        // CPDR / CPD — single iteration, computeNextPC handles repeat
                        case 0xB9: case 0xA9: {
                            uint16_t hl = (shadowH << 8) | shadowL;
                            uint16_t bc = (shadowB << 8) | shadowC;
                            uint8_t result = shadowA - z80RAM[hl];
                            hl--; bc--;
                            shadowH = hl >> 8; shadowL = hl & 0xFF;
                            shadowB = bc >> 8; shadowC = bc & 0xFF;
                            shadowF = (result & FLAG_S) | (result ? 0 : FLAG_Z) |
                                      FLAG_N | (bc ? FLAG_PV : 0) | (shadowF & FLAG_C);
                            break;
                        }
                        // SBC HL,rr — ED 42/52/62/72
                        case 0x42: case 0x52: case 0x62: case 0x72: {
                            uint16_t hl = (shadowH << 8) | shadowL;
                            uint16_t operand;
                            switch (ed_op) {
                                case 0x42: operand = (shadowB << 8) | shadowC; break;
                                case 0x52: operand = (shadowD << 8) | shadowE; break;
                                case 0x62: operand = hl; break;
                                default:   operand = softSP; break;
                            }
                            uint8_t c = (shadowF & FLAG_C) ? 1 : 0;
                            uint32_t r = (uint32_t)hl - operand - c;
                            uint16_t result16 = (uint16_t)r;
                            shadowH = result16 >> 8; shadowL = result16 & 0xFF;
                            shadowF = ((result16 >> 8) & FLAG_S) | (result16 ? 0 : FLAG_Z) |
                                      FLAG_N | ((r >> 16) ? FLAG_C : 0);
                            // Overflow
                            int32_t sr = (int16_t)hl - (int16_t)operand - c;
                            if (sr > 32767 || sr < -32768) shadowF |= FLAG_PV;
                            break;
                        }
                        // ADC HL,rr — ED 4A/5A/6A/7A
                        case 0x4A: case 0x5A: case 0x6A: case 0x7A: {
                            uint16_t hl = (shadowH << 8) | shadowL;
                            uint16_t operand;
                            switch (ed_op) {
                                case 0x4A: operand = (shadowB << 8) | shadowC; break;
                                case 0x5A: operand = (shadowD << 8) | shadowE; break;
                                case 0x6A: operand = hl; break;
                                default:   operand = softSP; break;
                            }
                            uint8_t c = (shadowF & FLAG_C) ? 1 : 0;
                            uint32_t r = (uint32_t)hl + operand + c;
                            uint16_t result16 = (uint16_t)r;
                            shadowH = result16 >> 8; shadowL = result16 & 0xFF;
                            shadowF = ((result16 >> 8) & FLAG_S) | (result16 ? 0 : FLAG_Z) |
                                      ((r >> 16) ? FLAG_C : 0);
                            int32_t sr = (int16_t)hl + (int16_t)operand + c;
                            if (sr > 32767 || sr < -32768) shadowF |= FLAG_PV;
                            break;
                        }
                        // IN r,(C) — ED 40/48/50/58/60/68/78
                        case 0x40: case 0x48: case 0x50: case 0x58:
                        case 0x60: case 0x68: case 0x78: {
                            // We handle IN via the IO state machine, just note it reads port C
                            uint8_t port = shadowC;
                            ioResponse = handle_io_read(port);
                            uint8_t* dest;
                            switch (ed_op) {
                                case 0x40: dest = &shadowB; break;
                                case 0x48: dest = &shadowC; break;
                                case 0x50: dest = &shadowD; break;
                                case 0x58: dest = &shadowE; break;
                                case 0x60: dest = &shadowH; break;
                                case 0x68: dest = &shadowL; break;
                                default:   dest = &shadowA; break;
                            }
                            *dest = ioResponse;
                            shadowF = flagsLogic(ioResponse, false) | (shadowF & FLAG_C); // preserve C
                            shadowF &= ~FLAG_H; // H is reset for IN
                            ioInDriveAfter = clock_cycle_count + 6;
                            ioState = IO_IN_PENDING;
                            break;
                        }
                        // OUT (C),r — ED 41/49/51/59/61/69/79
                        case 0x41: case 0x49: case 0x51: case 0x59:
                        case 0x61: case 0x69: case 0x79: {
                            uint8_t port = shadowC;
                            uint8_t data;
                            switch (ed_op) {
                                case 0x41: data = shadowB; break;
                                case 0x49: data = shadowC; break;
                                case 0x51: data = shadowD; break;
                                case 0x59: data = shadowE; break;
                                case 0x61: data = shadowH; break;
                                case 0x69: data = shadowL; break;
                                default:   data = shadowA; break;
                            }
                            handle_io_write(port, data);
                            break;
                        }
                        // RETN/RETI — handled in computeNextPC (SP already adjusted)
                        case 0x45: case 0x4D: break;
                        // EX AF,AF' variants, IM 0/1/2 — no tracked effect
                        case 0x46: case 0x56: case 0x5E: break;
                        // INI/INIR — B decremented per iteration; computeNextPC handles repeat
                        case 0xA2: case 0xB2: {
                            shadowB--;
                            uint16_t hl = (shadowH << 8) | shadowL;
                            hl++;
                            shadowH = hl >> 8; shadowL = hl & 0xFF;
                            break;
                        }
                        // IND/INDR — B decremented, HL decremented
                        case 0xAA: case 0xBA: {
                            shadowB--;
                            uint16_t hl = (shadowH << 8) | shadowL;
                            hl--;
                            shadowH = hl >> 8; shadowL = hl & 0xFF;
                            break;
                        }
                        // OUTI/OTIR — B decremented, HL incremented
                        case 0xA3: case 0xB3: {
                            shadowB--;
                            uint16_t hl = (shadowH << 8) | shadowL;
                            hl++;
                            shadowH = hl >> 8; shadowL = hl & 0xFF;
                            break;
                        }
                        // OUTD/OTDR — B decremented, HL decremented
                        case 0xAB: case 0xBB: {
                            shadowB--;
                            uint16_t hl = (shadowH << 8) | shadowL;
                            hl--;
                            shadowH = hl >> 8; shadowL = hl & 0xFF;
                            break;
                        }
                    }
                    break;
                }

                // ---- CB-prefixed instructions (bit/shift/rotate) ----
                case 0xCB: {
                    uint8_t cb_op = z80RAM[softPC + 1];
                    uint8_t* reg;
                    uint8_t memVal;
                    bool isMem = ((cb_op & 0x07) == 6);
                    uint16_t hlAddr = (shadowH << 8) | shadowL;

                    // Get source/dest register
                    switch (cb_op & 0x07) {
                        case 0: reg = &shadowB; break;
                        case 1: reg = &shadowC; break;
                        case 2: reg = &shadowD; break;
                        case 3: reg = &shadowE; break;
                        case 4: reg = &shadowH; break;
                        case 5: reg = &shadowL; break;
                        case 6: memVal = z80RAM[hlAddr]; reg = &memVal; break;
                        default: reg = &shadowA; break;
                    }

                    uint8_t group = cb_op >> 6;
                    if (group == 0) {
                        // Shifts and rotates: 00-3F
                        uint8_t subOp = (cb_op >> 3) & 0x07;
                        uint8_t val = *reg;
                        uint8_t bit7, bit0;
                        switch (subOp) {
                            case 0: // RLC
                                bit7 = val >> 7;
                                val = (val << 1) | bit7;
                                shadowF = (val & FLAG_S) | (val ? 0 : FLAG_Z) |
                                          (parityTable[val] ? FLAG_PV : 0) | (bit7 ? FLAG_C : 0);
                                break;
                            case 1: // RRC
                                bit0 = val & 1;
                                val = (val >> 1) | (bit0 << 7);
                                shadowF = (val & FLAG_S) | (val ? 0 : FLAG_Z) |
                                          (parityTable[val] ? FLAG_PV : 0) | (bit0 ? FLAG_C : 0);
                                break;
                            case 2: // RL
                                bit7 = val >> 7;
                                val = (val << 1) | ((shadowF & FLAG_C) ? 1 : 0);
                                shadowF = (val & FLAG_S) | (val ? 0 : FLAG_Z) |
                                          (parityTable[val] ? FLAG_PV : 0) | (bit7 ? FLAG_C : 0);
                                break;
                            case 3: // RR
                                bit0 = val & 1;
                                val = (val >> 1) | ((shadowF & FLAG_C) ? 0x80 : 0);
                                shadowF = (val & FLAG_S) | (val ? 0 : FLAG_Z) |
                                          (parityTable[val] ? FLAG_PV : 0) | (bit0 ? FLAG_C : 0);
                                break;
                            case 4: // SLA
                                bit7 = val >> 7;
                                val <<= 1;
                                shadowF = (val & FLAG_S) | (val ? 0 : FLAG_Z) |
                                          (parityTable[val] ? FLAG_PV : 0) | (bit7 ? FLAG_C : 0);
                                break;
                            case 5: // SRA
                                bit0 = val & 1;
                                val = (val >> 1) | (val & 0x80);  // preserve sign bit
                                shadowF = (val & FLAG_S) | (val ? 0 : FLAG_Z) |
                                          (parityTable[val] ? FLAG_PV : 0) | (bit0 ? FLAG_C : 0);
                                break;
                            case 6: // SLL (undocumented) — shift left, bit 0 = 1
                                bit7 = val >> 7;
                                val = (val << 1) | 1;
                                shadowF = (val & FLAG_S) | (val ? 0 : FLAG_Z) |
                                          (parityTable[val] ? FLAG_PV : 0) | (bit7 ? FLAG_C : 0);
                                break;
                            case 7: // SRL
                                bit0 = val & 1;
                                val >>= 1;
                                shadowF = (val & FLAG_S) | (val ? 0 : FLAG_Z) |
                                          (parityTable[val] ? FLAG_PV : 0) | (bit0 ? FLAG_C : 0);
                                break;
                        }
                        *reg = val;
                        if (isMem) {
                            deferredWriteAddr = hlAddr;
                            deferredWriteVal = val;
                        }
                    }
                    else if (group == 1) {
                        // BIT n,r — test bit, set Z flag (no write)
                        uint8_t bit = (cb_op >> 3) & 0x07;
                        uint8_t val = *reg;
                        shadowF = (shadowF & FLAG_C) | FLAG_H |
                                  ((val & (1 << bit)) ? 0 : FLAG_Z);
                        if (bit == 7 && (val & 0x80)) shadowF |= FLAG_S;
                    }
                    else if (group == 2) {
                        // RES n,r — reset bit
                        uint8_t bit = (cb_op >> 3) & 0x07;
                        *reg &= ~(1 << bit);
                        if (isMem) {
                            deferredWriteAddr = hlAddr;
                            deferredWriteVal = *reg;
                        }
                    }
                    else {
                        // SET n,r — set bit
                        uint8_t bit = (cb_op >> 3) & 0x07;
                        *reg |= (1 << bit);
                        if (isMem) {
                            deferredWriteAddr = hlAddr;
                            deferredWriteVal = *reg;
                        }
                    }
                    break;
                }

                // ---- DD/FD-prefixed instructions (IX/IY indexed) ----
                // We don't track IX/IY registers, but we handle ops that
                // affect our tracked registers (A, flags, etc.)
                case 0xDD: case 0xFD: {
                    uint8_t ix_op = z80RAM[softPC + 1];
                    // For indexed instructions with displacement:
                    // DD xx dd [nn] where dd = displacement byte at softPC+2
                    switch (ix_op) {
                        // ALU A,(IX+d) — affect A and flags
                        case 0x86: { // ADD A,(IX+d)
                            // We can't compute IX+d without IX, but the Z80 will
                            // execute it correctly. We lose shadow accuracy here.
                            break;
                        }
                        // PUSH/POP IX — SP handled in computeNextPC
                        case 0xE5: case 0xE1: break;
                        // Everything else: we can't track IX/IY ops meaningfully
                        // without the index register values, so just break
                    }
                    break;
                }
            }

            // Set guard for next M1 based on T-states
            // For conditional instructions, dynamically choose taken/not-taken
            // T-state count using shadowF (already updated by shadow tracking above).
            // This is critical because the difference can be large (e.g., RET cc:
            // taken=11 vs not-taken=5) and using the wrong value causes missed M1s.
            uint8_t guard_ticks = tStates[opcode];
            {

                switch (opcode) {
                    // RET cc: taken=11, not-taken=5
                    case 0xC0: case 0xC8: case 0xD0: case 0xD8:
                    case 0xE0: case 0xE8: case 0xF0: case 0xF8: {
                        uint8_t cond = (opcode >> 3) & 0x07;
                        bool taken = false;
                        switch (cond) {
                            case 0: taken = !(shadowF & FLAG_Z); break;
                            case 1: taken =  (shadowF & FLAG_Z); break;
                            case 2: taken = !(shadowF & FLAG_C); break;
                            case 3: taken =  (shadowF & FLAG_C); break;
                            case 4: taken = !(shadowF & FLAG_PV); break;
                            case 5: taken =  (shadowF & FLAG_PV); break;
                            case 6: taken = !(shadowF & FLAG_S); break;
                            case 7: taken =  (shadowF & FLAG_S); break;
                        }
                        guard_ticks = taken ? 11 : 5;
                        break;
                    }
                    // CALL cc: taken=17, not-taken=10
                    case 0xC4: case 0xCC: case 0xD4: case 0xDC:
                    case 0xE4: case 0xEC: case 0xF4: case 0xFC: {
                        uint8_t cond = (opcode >> 3) & 0x07;
                        bool taken = false;
                        switch (cond) {
                            case 0: taken = !(shadowF & FLAG_Z); break;
                            case 1: taken =  (shadowF & FLAG_Z); break;
                            case 2: taken = !(shadowF & FLAG_C); break;
                            case 3: taken =  (shadowF & FLAG_C); break;
                            case 4: taken = !(shadowF & FLAG_PV); break;
                            case 5: taken =  (shadowF & FLAG_PV); break;
                            case 6: taken = !(shadowF & FLAG_S); break;
                            case 7: taken =  (shadowF & FLAG_S); break;
                        }
                        guard_ticks = taken ? 17 : 10;
                        break;
                    }
                    // JR cc: taken=12, not-taken=7
                    case 0x20: case 0x28: case 0x30: case 0x38: {
                        uint8_t cond = (opcode - 0x20) >> 3;
                        bool taken = false;
                        switch (cond) {
                            case 0: taken = !(shadowF & FLAG_Z); break;
                            case 1: taken =  (shadowF & FLAG_Z); break;
                            case 2: taken = !(shadowF & FLAG_C); break;
                            case 3: taken =  (shadowF & FLAG_C); break;
                        }
                        guard_ticks = taken ? 12 : 7;
                        break;
                    }
                    // DJNZ: taken=13, not-taken=8
                    case 0x10: {
                        // B is decremented as part of DJNZ; predict using (B-1)
                        bool taken = ((uint8_t)(shadowB - 1)) != 0;
                        guard_ticks = taken ? 13 : 8;
                        break;
                    }
                }

                m1Guard = clock_cycle_count + guard_ticks;
            }

            prevM1Addr = softPC;

            // Milestone tracking (no debug prints)

#if outputDEBUG
            // Start recording M1 trace when we reach the loading loop setup
            if (!m1DbgActive && softPC >= 0x0028 && softPC <= 0x0048) {
                m1DbgActive = true;
            }
            if (m1DbgActive && m1DbgCount < M1_DBG_MAX)
                m1Dbg[m1DbgCount++] = {clock_cycle_count, softPC, opcode,
                                       shadowA, shadowH, shadowL, shadowB,
                                       guard_ticks, softSP};
#endif

            // Advance softPC for next instruction
            softPC = computeNextPC(softPC, opcode);
        }
    }
    else {
        // Memory Write (WR active)
        // Due to TXB0108 address bus propagation delay, the address
        // read here (uP_ADDR) is wrong — it lags by 1-3 ticks.
        // All memory writes are pre-written at M1 detection time using
        // softPC-derived correct addresses, so we suppress bus writes
        // to prevent corrupting z80RAM at lagged addresses.
        setDataBusInput();
        // z80RAM[uP_ADDR] = readDataBus();  // DISABLED: pre-writes handle this
    }

    CLK_LOW;
    clock_cycle_count++;
}

////////////////////////////////////////////////////////////////////
// Load boot.bin from sector server into Z80 RAM at address 0x0000
////////////////////////////////////////////////////////////////////
bool loadBootLoader() {
    Serial.print("Boot:       ");

    // Open boot.bin on server
    netSendFileCommand(DISK_CMD_OPEN_READ, "boot.bin");
    if (netReadStatus() != 0) {
        Serial.println("FAILED (boot.bin not found on server)");
        return false;
    }

    // Read blocks into z80RAM at 0x0000
    uint32_t addr = 0;
    const int MAX_BLOCKS = 32;  // Up to 4KB
    for (int i = 0; i < MAX_BLOCKS; i++) {
        server.write(NET_CMD_READ_BLOCK);
        if (netReadStatus() != 0) break;

        uint8_t buf[DISK_BLOCK_SIZE];
        if (!netReadExact(buf, DISK_BLOCK_SIZE)) break;

        // Check for all-zero block (past EOF)
        bool allZero = true;
        for (int j = 0; j < DISK_BLOCK_SIZE; j++) {
            if (buf[j] != 0) { allZero = false; break; }
        }

        memcpy(&z80RAM[addr], buf, DISK_BLOCK_SIZE);
        addr += DISK_BLOCK_SIZE;

        if (allZero) break;
    }

    // Close file
    server.write(DISK_CMD_CLOSE);
    netReadStatus();
    diskFileOpen = false;

    Serial.print("OK (");
    Serial.print(addr);
    Serial.println(" bytes loaded)");

    return addr > 0;
}

////////////////////////////////////////////////////////////////////
// Setup
////////////////////////////////////////////////////////////////////
void setup() {
    Serial.begin(115200);
    while (!Serial) {}  // Wait for serial connection

    Serial.println();
    Serial.println("======================================");
    Serial.println("RetroShield Z80 CP/M 2.2");
    Serial.println("Arduino Giga R1 WiFi");
    Serial.println("======================================");
    Serial.println();

    // Clear Z80 RAM
    Serial.print("Z80 RAM:    ");
    memset(z80RAM, 0, sizeof(z80RAM));
    Serial.println("OK (64KB in SRAM)");

    // Connect to WiFi
    Serial.print("WiFi:       ");
    delay(1000);
    WiFi.disconnect();
    delay(500);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    unsigned long wifiStart = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        if (millis() - wifiStart > 30000) {
            Serial.println(" TIMEOUT!");
            Serial.println("Retrying...");
            WiFi.disconnect();
            delay(1000);
            WiFi.begin(WIFI_SSID, WIFI_PASS);
            wifiStart = millis();
        }
    }
    Serial.print("OK (");
    Serial.print(WiFi.localIP());
    Serial.println(")");

    // Connect to sector server
    Serial.print("Server:     ");
    if (server.connect(SERVER_IP, SERVER_PORT)) {
        Serial.print("OK (");
        Serial.print(SERVER_IP);
        Serial.print(":");
        Serial.print(SERVER_PORT);
        Serial.println(")");
    } else {
        Serial.println("FAILED!");
        Serial.print("  Cannot reach ");
        Serial.print(SERVER_IP);
        Serial.print(":");
        Serial.println(SERVER_PORT);
        while (1) {}  // Halt
    }

    // Load boot loader from sector server
    if (!loadBootLoader()) {
        Serial.println("Cannot continue without boot.bin");
        while (1) {}  // Halt
    }

    // Boot code hex dump disabled for clean output

    // Initialize processor GPIO
    uP_init();

    // Initialize ACIA
    mc6850_init();

    // Reset processor
    uP_assert_reset();
    for (int i = 0; i < 25; i++) cpu_tick();

    Serial.println();
    Serial.println("Starting Z80...");
    Serial.println();

    // Release reset — enable M1 detection after a few ticks
    uP_release_reset();
    m1Guard = clock_cycle_count + 5;  // Z80 starts M1 ~3-4 ticks after reset
    softPC = 0x0000;                   // reset vector

}

////////////////////////////////////////////////////////////////////
// Loop
////////////////////////////////////////////////////////////////////
static unsigned long loopCounter = 0;
// static unsigned long perfLastTime = 0;
// static unsigned long perfLastCycle = 0;

void loop() {
    // Run a batch of ticks in a tight loop to avoid loop() framework overhead
    for (int batch = 0; batch < 4096; batch++) {
        cpu_tick();
    }

    // Yield to RTOS so USB serial stack stays alive
    if (++loopCounter >= 8) {
        loopCounter = 0;
        yield();
    }

}
