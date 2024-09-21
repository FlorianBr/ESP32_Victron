# ESP32 Victron Display

## What is it?

For the [Victron](https://www.victronenergy.de/) devices I have in my camper van I want some low-power display to see the stats like battery capacity and solar panel power. Currently supported:

- 500A Smart Shunt
- Smart Solar MPPT 75/15

### The Hardware

A [Heltec Vision Master E290](https://heltec.org/project/vision-master-e290/) with a ESP32, 2.90inch E-Ink Display, LoRa and battery connector. At the moment without any additional hardware.

I will also add a 3D printed case some time in the future.

### The Software

Libraries used:

- ESP-IDF v5.3
- LVGL 9.2

## Features

- [x] Basic operating system
- [x] EINK Driver
- [x] Basic LVGL setup
- [x] Bluetooth: Device detection
- [x] Victron: Driver
- [x] Victron: Decryption
- [x] Victron: Receive smart shunt data
- [ ] Victron: Receive solar charger data
- [ ] Victron: Receive charger data
- [ ] Victron: Refactoring
- [ ] LVGL: Change Theme to Monochrome
- [ ] LVGL: Fix display orientation
- [ ] Display current states
- [ ] Display battery capacity, voltage and current
- [ ] Display solar panel power
- [ ] Display battery graph
- [ ] Display solar graph

### Maybe

- [ ] WiFi
- [ ] Sending Data to a MQTT broker
- [ ] Sending Data via LoRa to my LoRa-Central
- [ ] Store keys in NVS
- [ ] Add and Remove keys with MQTT commands

## Notes

- LV_COLOR_FORMAT_ARGB8888 must be enabled for rotations
- Unuse Bits/Bytes in the Victron structs are NOT transfered!
- For some reason the Android App showed the first byte of the encryption key of SmartSolar incorrectly. Instead of b5 it showed only 5.

## Useful links

- Fabian Schmidts [ESPHome Component](https://github.com/Fabian-Schmidt/esphome-victron_ble)
- Victrons [Blogpost](https://communityarchive.victronenergy.com/questions/187303/victron-bluetooth-advertising-protocol.html) about the BLE Protocol
- Victron Spec [Extra Manufacturer Data](https://communityarchive.victronenergy.com/storage/attachments/48745-extra-manufacturer-data-2022-12-14.pdf)
- Victron Spec [VRegs](https://www.victronenergy.com/upload/documents/VE.Can-registers-public.pdf)
