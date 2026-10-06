# CompactIRReceiver

A palm-sized, battery-powered infrared (IR) signal receiver and logger based on the WCH CH32V003 RISC-V microcontroller. It decodes incoming IR signals, measures raw pulse lengths, displays the output on an I2C OLED display, and optionally logs raw timing data to a FAT32-formatted MicroSD card.

## Technical Specifications

| Component | Specification / Part Number |
| --- | --- |
| Microcontroller | WCH CH32V003A4M6 (RISC-V, 48 MHz, 2 KB RAM, 16 KB Flash) |
| IR Receiver Module | IRM-3638T (38 kHz) |
| Display | 0.91" 128x32 Monochrome I2C OLED (White) |
| Battery & Power | 300 mAh 3.7V LiPo battery |
| Charge Controller | LP4054 (USB-C charging) |
| Battery Protection | XB5352A (Overcharge, over-discharge, overcurrent protection) |
| Voltage Regulator | AP2112K-3.3 LDO (3.3V system power) |
| Power Switch | Latching push button |
| User Interface | 2x Push buttons (Confirm / Cancel) |
| Storage | MicroSD slot (Software SPI mode) |

## System Architecture & Memory Constraints

The CH32V003 has 2 KB of SRAM. To operate within these limits, the firmware shares a single 512-byte buffer (`buf[512]`):
- **UI State:** Used as the monochrome OLED display framebuffer (128x32 / 8 = 512 bytes).

MicroSD communications are handled via a lightweight, bit-banged SPI implementation with custom FAT32 root directory scanning and file writing functions.

## Operating Flow

1. **Boot:** Power on via the latching switch. The OLED displays the splash screen (`IR Receiver`).
2. **Listen:** Press **Confirm** to enter listening mode. The screen displays `Listening` with a loading animation.
   - Pressing **Confirm** or **Cancel** at any time while listening aborts the capture and returns to the boot screen.
3. **Capture:** When a signal is captured, the device displays:
   - Decoded hex value (e.g., `0x12345678`).
   - Total raw pulse count (e.g., `raw length: 32`).
   - Confirm (Check icon) and Cancel (Cross icon) indicators on screen.
4. **Save / Discard:**
   - Press **Confirm** to save the capture to the SD card.
   - Press **Cancel** to discard the capture without writing to disk.
5. **Feedback:** Shows status output (`Saved!`, `No SD card`, `Not FAT32`, `SD wr err`, or `Cancel X`) for 3 seconds before returning to the boot screen.

## SD Card Logging Format

Saved files are stored in the root directory under the 8.3 naming convention: `SIGxxxxx.TXT` (incrementing from `SIG00001.TXT` to `SIG99999.TXT`).

Format inside the text file:
```text
{HEX_VALUE};{RAW_COUNT};{{RAW_PULSE_MICROSECONDS_ARRAY}}
```

Example:
```text
A55A1234;12;{9000,4500,560,1690,560,560,560,1690,560,560,560,560}
```

## Toolchain & Compilation

Built using **MounRiver Studio II** targeting the CH32V003 platform.