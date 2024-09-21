/**
 ******************************************************************************
 *  file           : bluetooth.h
 *  brief          : Bluetooth component for the Victron Display
 ******************************************************************************
 *  Copyright (C) 2024 Florian Brandner
 */

#ifndef __BLUETOOTH_H_
#define __BLUETOOTH_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stdint.h"

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

typedef void (*blue_sshunt_cb_t)(const uint16_t volt, const int32_t curr, const uint16_t soc, const uint16_t temp,
                                 const uint16_t remain);
typedef void (*blue_ssolar_cb_t)(const uint8_t state, const uint16_t volt, const uint16_t curr);
typedef void (*blue_dcdc_cb_t)(const uint8_t state, const uint16_t inp, const uint16_t outp, const uint32_t offr);

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Init bluetooth
 */
void blue_init();

/**
 * @brief Set the Callback for SmartShunt Data
 */
void blue_setcb_sshunt(blue_sshunt_cb_t cb);

/**
 * @brief Set the Callback for SmartSolar Data
 */
void blue_setcb_ssolar(blue_ssolar_cb_t cb);

/**
 * @brief Set the Callback for DC/DC Data
 */
void blue_setcb_dcdc(blue_dcdc_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif // __BLUETOOTH_H_
