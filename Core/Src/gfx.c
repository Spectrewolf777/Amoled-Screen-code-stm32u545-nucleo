/**
  * @file    gfx.c
  * @brief   Small anti-aliased drawing primitives for band rendering.
  */
#include "gfx.h"
#include "font5x7.h"
#include <math.h>
#include <stddef.h>

#define DEG2RAD   0.01745329252f

static inline uint16_t Swap16(uint16_t v)
{
  return (uint16_t)((v << 8) | (v >> 8));
}

uint16_t Gfx_Mix(uint16_t a, uint16_t b, uint32_t t)
{
  /* spread the three fields apart (g in the upper half) so that one multiply
     can scale them all without the fractions spilling into a neighbour */
  uint32_t fa = ((uint32_t)a | ((uint32_t)a << 16)) & 0x07E0F81FU;
  uint32_t fb = ((uint32_t)b | ((uint32_t)b << 16)) & 0x07E0F81FU;
  uint32_t t5 = (t + 4U) >> 3;                      /* 0..32 */
  uint32_t m  = (fa + (((fb - fa) * t5) >> 5)) & 0x07E0F81FU;

  return (uint16_t)(m | (m >> 16));
}

/* Blend 'color' over one canvas pixel with coverage 0..1 */
static inline void BlendPx(uint16_t *p, uint16_t color, uint16_t colorWire, float coverage)
{
  if (coverage >= 0.996f)
  {
    *p = colorWire;
  }
  else if (coverage > 0.004f)
  {
    *p = Swap16(Gfx_Mix(Swap16(*p), color, (uint32_t)(coverage * 256.0f)));
  }
}

/* Intersect a screen-space box with the canvas; false if nothing is left.
   x1/y1 are exclusive. */
static int ClipBox(const GfxCanvas *c, int32_t *x0, int32_t *y0, int32_t *x1, int32_t *y1)
{
  if (*x0 < c->x)        { *x0 = c->x; }
  if (*y0 < c->y)        { *y0 = c->y; }
  if (*x1 > c->x + c->w) { *x1 = c->x + c->w; }
  if (*y1 > c->y + c->h) { *y1 = c->y + c->h; }
  return (*x0 < *x1) && (*y0 < *y1);
}

void Gfx_Clear(const GfxCanvas *c, uint16_t color)
{
  uint16_t wire = Swap16(color);
  int32_t  n = (int32_t)c->w * c->h;

  for (int32_t i = 0; i < n; i++)
  {
    c->px[i] = wire;
  }
}

void Gfx_FillRect(const GfxCanvas *c, int32_t x, int32_t y, int32_t w, int32_t h, uint16_t color)
{
  uint16_t wire = Swap16(color);
  int32_t  x0 = x, y0 = y, x1 = x + w, y1 = y + h;

  if (!ClipBox(c, &x0, &y0, &x1, &y1))
  {
    return;
  }
  for (int32_t py = y0; py < y1; py++)
  {
    uint16_t *row = &c->px[(py - c->y) * c->w + (x0 - c->x)];

    for (int32_t i = 0; i < (x1 - x0); i++)
    {
      row[i] = wire;
    }
  }
}

void Gfx_Capsule(const GfxCanvas *c, float ax, float ay, float bx, float by, float r, uint16_t color)
{
  uint16_t wire = Swap16(color);
  float minX = (ax < bx) ? ax : bx, maxX = (ax < bx) ? bx : ax;
  float minY = (ay < by) ? ay : by, maxY = (ay < by) ? by : ay;
  int32_t x0 = (int32_t)floorf(minX - r - 1.0f), x1 = (int32_t)ceilf(maxX + r + 1.0f) + 1;
  int32_t y0 = (int32_t)floorf(minY - r - 1.0f), y1 = (int32_t)ceilf(maxY + r + 1.0f) + 1;
  float dx = bx - ax, dy = by - ay;
  float len2 = dx * dx + dy * dy;
  float invLen2 = (len2 > 0.0001f) ? (1.0f / len2) : 0.0f;
  float rInner = (r > 0.5f) ? (r - 0.5f) : 0.0f;
  float full2 = rInner * rInner;
  float none2 = (r + 0.5f) * (r + 0.5f);

  if (!ClipBox(c, &x0, &y0, &x1, &y1))
  {
    return;
  }
  for (int32_t py = y0; py < y1; py++)
  {
    uint16_t *row = &c->px[(py - c->y) * c->w - c->x];
    float qy = (float)py - ay;

    for (int32_t px = x0; px < x1; px++)
    {
      float qx = (float)px - ax;
      float t = (qx * dx + qy * dy) * invLen2;
      float ex, ey, d2;

      if (t < 0.0f) { t = 0.0f; }
      if (t > 1.0f) { t = 1.0f; }
      ex = qx - t * dx;
      ey = qy - t * dy;
      d2 = ex * ex + ey * ey;

      if (d2 <= full2)
      {
        row[px] = wire;
      }
      else if (d2 < none2)
      {
        BlendPx(&row[px], color, wire, r + 0.5f - sqrtf(d2));
      }
    }
  }
}

void Gfx_Disc(const GfxCanvas *c, float cx, float cy, float r, uint16_t color)
{
  Gfx_Capsule(c, cx, cy, cx, cy, r, color);
}

void Gfx_Arc(const GfxCanvas *c, float cx, float cy, float rIn, float rOut,
             float a0, float a1, uint16_t color)
{
  uint16_t wire = Swap16(color);
  float span = a1 - a0;
  int   fullRing = (span >= 359.99f);
  float sx = sinf(a0 * DEG2RAD), sy = -cosf(a0 * DEG2RAD);
  float ex = sinf(a1 * DEG2RAD), ey = -cosf(a1 * DEG2RAD);
  float rMid = 0.5f * (rIn + rOut);
  float rCap = 0.5f * (rOut - rIn);
  float in2  = (rIn - 0.5f) * (rIn - 0.5f);
  float out2 = (rOut + 0.5f) * (rOut + 0.5f);
  float fullIn2  = (rIn + 0.5f) * (rIn + 0.5f);
  float fullOut2 = (rOut - 0.5f) * (rOut - 0.5f);
  int32_t x0 = (int32_t)floorf(cx - rOut - 1.0f), x1 = (int32_t)ceilf(cx + rOut + 1.0f) + 1;
  int32_t y0 = (int32_t)floorf(cy - rOut - 1.0f), y1 = (int32_t)ceilf(cy + rOut + 1.0f) + 1;

  if (span < 0.0f)
  {
    return;
  }
  if (!ClipBox(c, &x0, &y0, &x1, &y1))
  {
    return;
  }

  for (int32_t py = y0; py < y1; py++)
  {
    uint16_t *row = &c->px[(py - c->y) * c->w - c->x];
    float dy = (float)py - cy;

    for (int32_t px = x0; px < x1; px++)
    {
      float dx = (float)px - cx;
      float d2 = dx * dx + dy * dy;

      if ((d2 <= in2) || (d2 >= out2))
      {
        continue;
      }
      if (!fullRing)
      {
        /* clockwise of the start direction / counter-clockwise of the end */
        int afterStart = (sx * dy - sy * dx) >= 0.0f;
        int beforeEnd  = (dx * ey - dy * ex) >= 0.0f;
        int inside;

        if (span <= 180.0f)
        {
          /* also on the arc's side of the centre (matters for very short arcs) */
          inside = afterStart && beforeEnd && ((dx * (sx + ex) + dy * (sy + ey)) >= 0.0f);
        }
        else
        {
          inside = afterStart || beforeEnd;
        }

        if (!inside)
        {
          continue;
        }
      }
      if ((d2 >= fullIn2) && (d2 <= fullOut2))
      {
        row[px] = wire;
      }
      else
      {
        float d = sqrtf(d2);
        float cov = d - (rIn - 0.5f);
        float covOut = (rOut + 0.5f) - d;

        if (covOut < cov) { cov = covOut; }
        BlendPx(&row[px], color, wire, cov);
      }
    }
  }

  if (!fullRing)
  {
    /* round ends (they also hide the hard radial edges) */
    Gfx_Disc(c, cx + sx * rMid, cy + sy * rMid, rCap, color);
    Gfx_Disc(c, cx + ex * rMid, cy + ey * rMid, rCap, color);
  }
}

void Gfx_Digit(const GfxCanvas *c, float x, float y, float w, float h, float t,
               uint32_t digit, uint16_t color)
{
  /* segment bits: a=0 (top), b, c, d (bottom), e, f, g=6 (middle) */
  static const uint8_t segs[10] =
  {
    0x3FU, 0x06U, 0x5BU, 0x4FU, 0x66U, 0x6DU, 0x7DU, 0x07U, 0x7FU, 0x6FU
  };
  float r = 0.5f * t;
  float L = x + r, R = x + w - r;
  float T = y + r, M = y + 0.5f * h, B = y + h - r;
  float g = r + 2.0f;   /* gap between neighbouring segment ends */
  uint8_t s;
  int32_t bx0 = (int32_t)x - 1, by0 = (int32_t)y - 1;
  int32_t bx1 = (int32_t)(x + w) + 2, by1 = (int32_t)(y + h) + 2;

  if ((digit > 9U) || !ClipBox(c, &bx0, &by0, &bx1, &by1))
  {
    return;
  }
  s = segs[digit];
  if (s & 0x01U) { Gfx_Capsule(c, L + g, T, R - g, T, r, color); }
  if (s & 0x02U) { Gfx_Capsule(c, R, T + g, R, M - g, r, color); }
  if (s & 0x04U) { Gfx_Capsule(c, R, M + g, R, B - g, r, color); }
  if (s & 0x08U) { Gfx_Capsule(c, L + g, B, R - g, B, r, color); }
  if (s & 0x10U) { Gfx_Capsule(c, L, M + g, L, B - g, r, color); }
  if (s & 0x20U) { Gfx_Capsule(c, L, T + g, L, M - g, r, color); }
  if (s & 0x40U) { Gfx_Capsule(c, L + g, M, R - g, M, r, color); }
}

int32_t Gfx_TextWidth(const char *text, int32_t scale)
{
  int32_t n = 0;

  while (text[n] != '\0')
  {
    n++;
  }
  return (n > 0) ? ((n * 6 - 1) * scale) : 0;
}

void Gfx_Text(const GfxCanvas *c, int32_t x, int32_t y, const char *text, int32_t scale, uint16_t color)
{
  int32_t bx0 = x, by0 = y, bx1 = x + Gfx_TextWidth(text, scale), by1 = y + 7 * scale;

  if (!ClipBox(c, &bx0, &by0, &bx1, &by1))
  {
    return;
  }
  for (; *text != '\0'; text++, x += 6 * scale)
  {
    uint32_t ch = (uint8_t)*text;
    const uint8_t *glyph;

    if ((ch >= 'a') && (ch <= 'z'))
    {
      ch -= 32U;
    }
    if ((ch < FONT5X7_FIRST) || (ch > FONT5X7_LAST))
    {
      continue;
    }
    glyph = font5x7[ch - FONT5X7_FIRST];
    for (int32_t row = 0; row < 7; row++)
    {
      for (int32_t col = 0; col < 5; col++)
      {
        if (glyph[row] & (0x10U >> col))
        {
          Gfx_FillRect(c, x + col * scale, y + row * scale, scale, scale, color);
        }
      }
    }
  }
}
