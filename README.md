# ESP32P4 + ESP32C6 Retro-game application (Built using OpenAI GPT5.3-Codex and Oh-my-opencode)

<img src="https://github.com/zenixls2/retro-p4-touch/blob/main/demo.gif" alt="demo" width="400"/>

This is a retro-game application for ESP32P4 (host) and ESP32C6 (co-processor). It includes a simple game that can be played on a small display connected to the microcontroller. We make use of the `JC48804C_I_W` board from GUITON.

## Architecture Overview

- **ESP32P4** - Host MCU: Handles display, touch input, UI, and main application logic
- **ESP32C6** - Co-processor: Handles BLE and Wi-Fi communication
- **Communication**: Uses ESP-Hosted RPC framework for communication between host and co-processor

### Core Framework
- ✅ **ESP-Hosted Integration**: Proper initialization and communication with ESP32C6 co-processor
- ✅ **Display Support**: 480x800 ST7701 MIPI DSI display initialization
- ✅ **Touch Input**: GT911 touch controller support
- ✅ **Wi-Fi Connectivity**: On-device AP scan + STA connect UI (via ESP32C6)
- ✅ **BLE Scanning**: On-device BLE device scan UI (via ESP32C6)
- ✅ **SD Card Storage**: FAT/exFAT mount at `/sdcard` + ROM loading + game saves
- ✅ **RTC/Time Sync**: Timezone lookup + SNTP time sync for real-time clock support
- ✅ **Pin Configuration**: ESP32P4-specific pin definitions

### Development Environment
- **ESP-IDF**: ESP32 development framework (>= v5.4.0)
- **Toolchain**: ESP32 toolchain for building and flashing the application

### Preparation

#### ROM (two options)
1. **Embedded ROM (requires reflash)**: Put the GBA ROM image in the project root and name it `pokemon.gba`. The ROM will be bundled into the firmware image.
2. **SD card ROM (no reflash)**: Format an SD card as FAT32 or exFAT and copy `pokemon.gba` to the card root. When available, the app loads it from `/sdcard/pokemon.gba`.

#### Save data
- When the SD card is mounted, the battery save is loaded from `/sdcard/pokemon.sav` (if present) and written back automatically while playing.

## Useful Commands
- **Build the project**: `idf.py build`
- **Flash the project**: `idf.py flash`
- **Monitor the output**: `idf.py monitor`
- **Clean the build**: `idf.py clean`
