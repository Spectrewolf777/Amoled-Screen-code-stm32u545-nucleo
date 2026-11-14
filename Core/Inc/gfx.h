/**
  * @file    gfx.h
  * @brief   Small anti-aliased drawing primitives for band rendering.
  *
  * There is no full frame buffer. A GfxCanvas is a small window somewhere on
  * the screen backed by a pixel buffer; primitives take SCREEN coordinates
  * and only touch the pixels that fall inside the canvas. To update part of
  * the screen, point a canvas at it, draw the whole scene, send the buffer.
  *
  * Colours passed to the functions are plain RGB565. Pixels in the canvas are
  * stored in display wire order (high byte first), ready for CO5300_WriteRaw.
  */
#ifndef GFX_H
#define GFX_H

#include <stdint.h>

#define GFX_RGB(r, g, b)  ((uint16_t)((((r) & 0xF8U) << 8) | (((g) & 0xFCU) << 3) | ((b) >> 3)))

typedef struct
{
  uint16_t *px;     /* w*h pixels, wire order */
  int16_t   x;      /* screen position of the canvas */
  int16_t   y;
  int16_t   w;
  int16_t   h;
} GfxCanvas;

void Gfx_Clear(const GfxCanvas *c, uint16_t color);
void Gfx_FillRect(const GfxCanvas *c, int32_t x, int32_t y, int32_t w, int32_t h, uint16_t color);

/** Filled circle. */
void Gfx_Disc(const GfxCanvas *c, float cx, float cy, float r, uint16_t color);

/** Thick line with round ends ("capsule"); r is half the thickness. */
void Gfx_Capsule(const GfxCanvas *c, float x0, float y0, float x1, float y1, float r, uint16_t color);

/**
  * Ring segment with round ends. Angles in degrees, clockwise, 0 = 12 o'clock;
  * drawn from a0 to a1 (a1 >= a0, a span of 360 or more gives a full ring).
  */
void Gfx_Arc(const GfxCanvas *c, float cx, float cy, float rIn, float rOut,
             float a0, float a1, uint16_t color);

/** Seven-segment style digit (0..9) in a w*h box, stroke thickness t. */
void Gfx_Digit(const GfxCanvas *c, float x, float y, float w, float h, float t,
               uint32_t digit, uint16_t color);

/** 5x7 font text (space, 0-9, A-Z and : % - . /), each font pixel scale*scale. */
void Gfx_Text(const GfxCanvas *c, int32_t x, int32_t y, const char *text, int32_t scale, uint16_t color);
int32_t Gfx_TextWidth(const char *text, int32_t scale);

/** Mix two RGB565 colours, t = 0 gives a, t = 256 gives b. */
uint16_t Gfx_Mix(uint16_t a, uint16_t b, uint32_t t);

#endif /* GFX_H */
