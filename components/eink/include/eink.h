/**
 ******************************************************************************
 *  file           : eink.h
 *  brief          : E-Ink Driver for the Vision Master E290
 ******************************************************************************
 *  Copyright (C) 2024 Florian Brandner
 */

#ifndef __EINK_H_
#define __EINK_H_

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
 * @brief Init the E-Ink display interface
 */
void eink_init(void);

/**
 * @brief Update the display with the buffer content
 */
void eink_update(void);

/**
 * @brief Fill the screen with a value
 *
 * @param value The value to fill the buffer with
 */
void eink_fillscreen(const uint8_t value);

/**
 * @brief Set the pointer to the display buffer
 *
 * @param pBuffer Pointer to the buffer
 * @param len buffers length in byte
 */
void eink_setbuffer(uint8_t* pBuffer, const uint16_t len);

#ifdef __cplusplus
}
#endif

#endif // __EINK_H_
