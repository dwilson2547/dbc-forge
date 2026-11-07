# SD CAN logger (the first dongle)

The passive CAN logger the dongle started as: XIAO ESP32-S3 + TJA1051 + SPI microSD, logging every
frame to `CAN_NNN.TXT` for reading later, with a hold-up capacitor to finish the last SD write when
the car is switched off. Kept for reference; `esp32_can_ble_bridge` is its descendant.

Copied from robo-services `capture_device_configs/can_capture_dongle_esp32_c6/esp32_can_logger.ino`
(robo-services e706852) on 2026-11-07, renamed so the sketch compiles from its own folder. Despite
the old folder name, the header says it targets the XIAO ESP32-S3.
