# ESP32 Victron Display

## What is it?

For the victron devices I have in my camper van I want some low-power display to see the stats like battery capacity and solar panel power.

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
- [ ] Bluetooth: Device detection
- [ ] Victron: Driver
- [ ] Victron: Receive smart shunt data
- [ ] Victron: Receive solar charger data
- [ ] Victron: Receive charger data
- [ ] LVGL: Change Theme to Monochrome
- [ ] LVGL: Fix display orientation
- [ ] Display current states
- [ ] Display battery capacity, voltage and current
- [ ] Display solar panel power
- [ ] Display battery graph
- [ ] Display solar graph

### Maybe

- WiFi
- Sending Data to a MQTT broker
- Sending Data via LoRa to my LoRa-Central

## Notes

- LV_COLOR_FORMAT_ARGB8888 must be enabled for rotations
