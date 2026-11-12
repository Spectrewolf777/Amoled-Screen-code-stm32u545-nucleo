/**
  * @file    amoled_board.h
  * @brief   Wiring / panel configuration for the 1.73" 466x466 AMOLED module
  *          (CO5300 display driver on QSPI + CST820 touch controller on I2C).
  *
  * The QSPI and I2C pins come from CubeMX (OCTOSPI1 + I2C1). The plain GPIOs
  * below (resets, optional power enable) are configured by the drivers
  * themselves, so they only need to be changed HERE to match your wiring.
  */
#ifndef AMOLED_BOARD_H
#define AMOLED_BOARD_H

#include "main.h"
#include <stdbool.h>

/* ---------------------------------------------------------------- panel -- */
#define AMOLED_WIDTH            466U
#define AMOLED_HEIGHT           466U

/* Offset of the visible area inside the CO5300 frame memory. 466x466 panels
   normally sit 6 columns in. If the image is shifted sideways or you see a
   garbage stripe on one edge, try 0 here. Must be even (CO5300 restriction). */
#define AMOLED_COL_OFFSET       6U
#define AMOLED_ROW_OFFSET       0U

/* Brightness after CO5300_Init(), 0x00..0xFF. Kept low while the supply is
   suspect: a bright full-screen image draws well over 100 mA. */
#define AMOLED_DEFAULT_BRIGHTNESS   0x30U

/* ------------------------------------------------------- display GPIOs --- */
/* QSPI pins, must match the OCTOSPI1 pinout in CubeMX. Only used by the
   bit-banged register read-back (CO5300_ReadReg). */
#define LCD_CS_PORT             GPIOA
#define LCD_CS_PIN              GPIO_PIN_0
#define LCD_CLK_PORT            GPIOA
#define LCD_CLK_PIN             GPIO_PIN_3
#define LCD_IO0_PORT            GPIOB
#define LCD_IO0_PIN             GPIO_PIN_1
#define LCD_IO1_PORT            GPIOB
#define LCD_IO1_PIN             GPIO_PIN_0

/* Display reset (RESX / LCD_RST), active low. Comment out both lines if the
   module has no reset pin wired: a software reset is used instead. */
#define LCD_RST_PORT            GPIOA
#define LCD_RST_PIN             GPIO_PIN_8

/* Optional panel power enable (LCD_EN / PWR_EN), active high. Uncomment if
   your module has one. */
/* #define LCD_EN_PORT          GPIOB */
/* #define LCD_EN_PIN           GPIO_PIN_5 */

/* --------------------------------------------------------- touch GPIOs --- */
/* Touch reset (TP_RST), active low. Comment out both lines if not wired
   (e.g. shared with the display reset on the module). */
#define TP_RST_PORT             GPIOA
#define TP_RST_PIN              GPIO_PIN_4

/* TP_INT is not needed: the touch controller is polled over I2C. */

/* Touch-to-display orientation fixes, set to 1 if needed. */
#define TP_SWAP_XY              0
#define TP_INVERT_X             0
#define TP_INVERT_Y             0

/* ------------------------------------------------------------- helper ---- */
static inline void AMOLED_GpioClockEnable(const GPIO_TypeDef *port)
{
  if (port == GPIOA)      { __HAL_RCC_GPIOA_CLK_ENABLE(); }
  else if (port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
  else if (port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
  else if (port == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
  else if (port == GPIOH) { __HAL_RCC_GPIOH_CLK_ENABLE(); }
}

static inline void AMOLED_GpioOutputInit(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState level)
{
  GPIO_InitTypeDef gpio = {0};

  AMOLED_GpioClockEnable(port);
  HAL_GPIO_WritePin(port, pin, level);
  gpio.Pin   = pin;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(port, &gpio);
}

#endif /* AMOLED_BOARD_H */
