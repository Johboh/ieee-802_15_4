# Ieee 802.15.4
[![PlatformIO CI](https://github.com/Johboh/ieee-802_15_4/actions/workflows/platformio.yaml/badge.svg)](https://registry.platformio.org/libraries/johboh/ieee-802_15_4)
[![ESP-IDF CI](https://github.com/Johboh/ieee-802_15_4/actions/workflows/espidf.yaml/badge.svg)](https://components.espressif.com/components/johboh/ieee-802_15_4)
[![Arduino IDE](https://github.com/Johboh/ieee-802_15_4/actions/workflows/arduino_cli.yaml/badge.svg)](https://github.com/Johboh/ieee-802_15_4/actions/workflows/arduino_cli.yaml)
[![GitHub release](https://img.shields.io/github/release/Johboh/ieee-802_15_4.svg)](https://github.com/Johboh/ieee-802_15_4/releases)
[![Clang-format](https://github.com/Johboh/ieee-802_15_4/actions/workflows/clang-format.yaml/badge.svg)](https://github.com/Johboh/ieee-802_15_4)

Arduino (using Arduino IDE) and ESP-IDF (using Espressif IoT Development Framework or PlatformIO) compatible library for sending and reciving messages over 802.15.4 for ESP32-C6 and ESP32-H2 (from ESP-IDF 5.1).

### Installation
#### PlatformIO (Arduino or ESP-IDF):
Add the following to `libs_deps`:
```
   Johboh/ieee-802_15_4
```
#### Arduino IDE:
Search for `ieee-802_15_4` by `johboh` in the library manager.
#### Espressif IoT Development Framework:
In your existing `idf_component.yml` or in a new `idf_component.yml` next to your main component:
```
dependencies:
  johboh/ieee-802_15_4:
    version: ">=0.5.3"
```

#### Arduino IDE:
Search for `ieee-802_15_4` by `johboh` in the library manager. See note about version above.

### Examples
- [Using Arduino IDE/CLI or Platform IO Arduino host](examples/arduino/host/host.ino)
- [Using Arduino IDE/CLI or Platform IO Arduino node](examples/arduino/node/node.ino)
- [ESP-IDF framework host](examples/espidf/host/main/main.cpp)
- [ESP-IDF framework node](examples/espidf/node/main/main.cpp)

### Compatibility
- As of now, ESP32-C6 and ESP32-H2 are the only ones supporting 802.15.4, but might be more in the future.
- Requires at last ESP-IDF 5.1.0
- Can be uses as an Arduino library when using Arduino IDE, or the ESP-IDF framework with Arduino core.
- Can be used as an ESP-IDF component when using ESP-IDF framework.
- For PlatformIO, can only be used when using the ESP-IDF framework, as PlatformIO Arduino verion is too old (4.4.7, end of life))
