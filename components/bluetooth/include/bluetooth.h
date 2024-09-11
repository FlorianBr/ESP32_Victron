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

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Init bluetooth
 */
void blue_init();

#ifdef __cplusplus
}
#endif

#endif // __BLUETOOTH_H_
