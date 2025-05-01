# Ieee 802.15.4
[![PlatformIO CI](https://github.com/Johboh/ieee-802_15_4/actions/workflows/platformio.yaml/badge.svg)](https://registry.platformio.org/libraries/johboh/ieee-802_15_4)
[![ESP-IDF CI](https://github.com/Johboh/ieee-802_15_4/actions/workflows/espidf.yaml/badge.svg)](https://components.espressif.com/components/johboh/ieee-802_15_4)
[![Arduino IDE](https://github.com/Johboh/ieee-802_15_4/actions/workflows/arduino_cli.yaml/badge.svg)](https://github.com/Johboh/ieee-802_15_4/actions/workflows/arduino_cli.yaml)
[![GitHub release](https://img.shields.io/github/release/Johboh/ieee-802_15_4.svg)](https://github.com/Johboh/ieee-802_15_4/releases)
[![Clang-format](https://github.com/Johboh/ieee-802_15_4/actions/workflows/clang-format.yaml/badge.svg)](https://github.com/Johboh/ieee-802_15_4)

Arduino (using Arduino IDE or PlatformIO) and ESP-IDF (using Espressif IoT Development Framework or PlatformIO) compatible library for sending and reciving messages over 802.15.4

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
    version: ">=1.0.0"
```

#### Arduino IDE:
Search for `ieee-802_15_4` by `johboh` in the library manager. See note about version above.

### Examples
- [Using Arduino IDE/CLI or Platform IO Arduino host](examples/arduino/host/host.ino)
- [Using Arduino IDE/CLI or Platform IO Arduino node](examples/arduino/node/node.ino)
- [ESP-IDF framework host](examples/espidf/host/main/main.cpp)
- [ESP-IDF framework node](examples/espidf/node/main/main.cpp)

### Compatibility
- ESP32 only
