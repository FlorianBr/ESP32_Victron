/**
 ******************************************************************************
 *  file           : sysconfig.h
 *  brief          : The systems configuration
 ******************************************************************************
 *  Copyright (C) 2024 Florian Brandner
 */

#ifndef __SYSCONFIG_H_
#define __SYSCONFIG_H_

#define PIN_MISO -1    // EInk: Unused
#define PIN_MOSI 1     // EInk: SDA/D1 - SDI
#define PIN_CLK 2      // EInk: SCL/D0 - Eink-CLK
#define PIN_CS 3       // EInk: CS#    - Eink-CS
#define PIN_DC 4       // EInk: D/C#   - D/C
#define PIN_RST 5      // EInk: RES#   - Eink-RST
#define PIN_BUSY 6     // EInk: BUSY   - Eink-BUSY
#define PIN_VE_CTRL 18 // EInk: Voltage Enable
#define PIN_LED 45     // GPIO or the Blink-LED

#define EINK_SIZE_X 128 // E-Ink width in pixels
#define EINK_SIZE_Y 296 // E-Ink height in pixels

#endif // __SYSCONFIG_H_
