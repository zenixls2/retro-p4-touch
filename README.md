# ESP32P4 + ESP32C6 Retro-game application (Built using OpenAI GPT5.3-Codex and Oh-my-opencode)

<img src="https://github.com/zenixls2/retro-p4-touch/blob/main/demo.gif" alt="demo" width="400"/>

This is a retro-game application for ESP32P4 and ESP32C6 microcontrollers. It includes a simple game that can be played on a small display connected to the microcontroller. We make use of the `JC48804C_I_W` board from GUITON. The example and all the necessary files are included in `/Users/hanshenghuang/esp/JC4880P443C_I_W/` and the example project is located in `/Users/hanshenghuang/esp/JC4880P443C_I_W/1-Demo/idf_examples/ESP-IDF/xiaozhi-esp32`.

## Architecture Overview

- **ESP32P4** - Host MCU: Handles display, touch input, UI, and main application logic
- **ESP32C6** - Co-processor: Handles BLE and WIFI commnunication. 
- **Communication**: Uses ESP-Hosted RPC framework for communication between host and co-processor

### Core Framework
- ✅ **ESP-Hosted Integration**: Proper initialization and communication with ESP32C6 co-processor
- ✅ **Display Support**: 480x800 ST7701 MIPI DSI display initialization
- ✅ **Touch Input**: GT911 touch controller support
- ✅ **Pin Configuration**: ESP32P4-specific pin definitions

### Development Environment
- **ESP-IDF**: ESP32 development framework
- **Toolchain**: ESP32 toolchain for building and flashing the application

### Preparation
Put the GBA ROM image in the project root and name it as `pokemon.gba`. The image will be burnt into the flash and used in the emulation.

## Useful Commands
- **Build the project**: `idf.py build`
- **Flash the project**: `idf.py flash`
- **Monitor the output**: `idf.py monitor`
- **Clean the build**: `idf.py clean`
