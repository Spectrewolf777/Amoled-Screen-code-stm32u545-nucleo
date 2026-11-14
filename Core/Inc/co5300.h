/**
  * @file    co5300.h
  * @brief   CO5300 AMOLED driver (QSPI, RGB565) for STM32U5 OCTOSPI1.
  *
  * There is no frame buffer in the MCU (466*466*2 = 434 kB does not fit in the
  * STM32U545's RAM): everything is streamed straight into the panel's GRAM.
  */
#ifndef CO5300_H
#define CO5300_H

#include "amoled_board.h"

#define CO5300_WIDTH            AMOLED_WIDTH
#define CO5300_HEIGHT           AMOLED_HEIGHT

/* RGB565 helpers */
#define CO5300_RGB(r, g, b)     ((uint16_t)((((r) & 0xF8U) << 8) | (((g) & 0xFCU) << 3) | ((b) >> 3)))
#define CO5300_BLACK            0x0000U
#define CO5300_WHITE            0xFFFFU
#define CO5300_RED              0xF800U
#define CO5300_GREEN            0x07E0U
#define CO5300_BLUE             0x001FU
#define CO5300_YELLOW           0xFFE0U
#define CO5300_CYAN             0x07FFU
#define CO5300_MAGENTA          0xF81FU

/** Reset the panel and run the init sequence. Screen is cleared to black. */
HAL_StatusTypeDef CO5300_Init(void);

/** Brightness 0x00 (off) .. 0xFF (max). */
HAL_StatusTypeDef CO5300_SetBrightness(uint8_t level);

HAL_StatusTypeDef CO5300_DisplayOn(void);
HAL_StatusTypeDef CO5300_DisplayOff(void);

/**
  * Test mode: the panel shows solid white without using the frame memory
  * (command 23h); false returns to the normal picture (13h).
  */
HAL_StatusTypeDef CO5300_AllPixelsOn(bool on);

/** Low power: display off + sleep in / sleep out + display on. */
HAL_StatusTypeDef CO5300_Sleep(void);
HAL_StatusTypeDef CO5300_Wakeup(void);

/**
  * Fill a rectangle. The rectangle is clipped to the screen.
  * NOTE: the CO5300 only accepts windows with even x and even width, so the
  * rectangle is widened by one pixel where necessary.
  */
HAL_StatusTypeDef CO5300_FillRect(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t color);
HAL_StatusTypeDef CO5300_FillScreen(uint16_t color);

/**
  * Draw a w*h image of native (little endian) RGB565 pixels.
  * x and w must be even and the image must be completely on screen.
  */
HAL_StatusTypeDef CO5300_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *pixels);

/**
  * Streaming interface, e.g. for LVGL flush callbacks:
  *   CO5300_SetWindow(x, y, w, h);          (x and w even)
  *   CO5300_WritePixels(buf, n); ...        (any number of calls, until
  *                                           exactly w*h pixels were sent)
  * The pixels go out as one QSPI frame that is closed when the window is full.
  */
HAL_StatusTypeDef CO5300_SetWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
HAL_StatusTypeDef CO5300_WritePixels(const uint16_t *pixels, uint32_t count);

/**
  * Fastest path, for pixel data that is already in wire order (RGB565, high
  * byte first; see CO5300_WIRE()). After CO5300_SetWindow() send exactly
  * w*h*2 bytes in one or more calls.
  * The data goes out by DMA in the background: the call returns immediately
  * and the buffer must not be modified until the next driver call (which
  * waits for it) or CO5300_WaitIdle() has returned. Rendering into a second
  * buffer meanwhile is what makes this fast.
  */
HAL_StatusTypeDef CO5300_WriteRaw(const uint8_t *wireBytes, uint32_t len);
HAL_StatusTypeDef CO5300_WaitIdle(void);

/** RGB565 value as it has to sit in memory for CO5300_WriteRaw(). */
#define CO5300_WIRE(c)          ((uint16_t)((((c) & 0xFFU) << 8) | (((c) >> 8) & 0xFFU)))

/**
  * Diagnostic register read (e.g. 04h ID, 0Ah power mode, 52h brightness).
  * The CO5300 answers on SIO0, which the OCTOSPI peripheral cannot receive on
  * in 1-wire mode, so this temporarily takes over CS/CLK/IO0/IO1 as GPIOs and
  * bit-bangs the read slowly. What is seen on IO0 and on IO1 is returned
  * separately (either pointer may be NULL); a line nobody drives reads 00.
  */
void CO5300_ReadReg(uint8_t cmd, uint8_t *io0Data, uint8_t *io1Data, uint32_t len);

#endif /* CO5300_H */
