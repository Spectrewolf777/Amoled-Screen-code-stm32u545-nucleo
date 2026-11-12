/**
  * @file    cst820.h
  * @brief   CST820 capacitive touch controller driver (I2C1, polled).
  */
#ifndef CST820_H
#define CST820_H

#include "amoled_board.h"
#include <stdbool.h>

#define CST820_I2C_ADDR         0x15U   /* 7-bit address */

/* Gesture codes reported in CST820_Touch.gesture */
#define CST820_GESTURE_NONE         0x00U
#define CST820_GESTURE_SWIPE_UP     0x01U
#define CST820_GESTURE_SWIPE_DOWN   0x02U
#define CST820_GESTURE_SWIPE_LEFT   0x03U
#define CST820_GESTURE_SWIPE_RIGHT  0x04U
#define CST820_GESTURE_CLICK        0x05U
#define CST820_GESTURE_DOUBLE_CLICK 0x0BU
#define CST820_GESTURE_LONG_PRESS   0x0CU

typedef struct
{
  bool     pressed;   /* true while a finger is on the panel */
  uint16_t x;         /* display coordinates, valid when pressed */
  uint16_t y;
  uint8_t  gesture;
} CST820_Touch;

/** Reset the controller, check that it answers and disable its auto sleep. */
HAL_StatusTypeDef CST820_Init(void);

/** Chip ID register (A7h); a CST820 reports B7h. */
HAL_StatusTypeDef CST820_ReadChipID(uint8_t *id);

/** Read the current touch state. Call it every 10..20 ms. */
HAL_StatusTypeDef CST820_Read(CST820_Touch *touch);

#endif /* CST820_H */
