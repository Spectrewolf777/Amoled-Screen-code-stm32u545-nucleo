/**
  * @file    ui.c
  * @brief   Demo user interface: watch face + bouncing orbs.
  *
  * Rendering model: nothing is kept in a frame buffer. Every frame the scene
  * state is advanced, the screen areas that changed are collected as "dirty"
  * rectangles, and each rectangle is re-rendered from scratch in bands of a
  * few rows that are streamed to the panel. Two band buffers are used in
  * turn so the next band is rendered while DMA is still sending the last one.
  */
#include "ui.h"
#include "gfx.h"
#include "co5300.h"
#include <math.h>

#define SCREEN_W        466
#define SCREEN_H        466
#define CX              233.0f
#define CY              233.0f

/* ---------------------------------------------------------- band buffers -- */
#define BAND_PIXELS     (SCREEN_W * 20)

static uint16_t s_band[2][BAND_PIXELS];
static uint32_t s_bandIdx;

/* -------------------------------------------------------- dirty rectangles */
typedef struct { int32_t x0, y0, x1, y1; } Rect;    /* x1/y1 exclusive */

#define MAX_DIRTY       24
static Rect     s_dirty[MAX_DIRTY];
static int32_t  s_dirtyCount;
static uint32_t s_framePixels;

static int32_t RectArea(const Rect *r)
{
  return (r->x1 - r->x0) * (r->y1 - r->y0);
}

static Rect RectUnion(const Rect *a, const Rect *b)
{
  Rect u;

  u.x0 = (a->x0 < b->x0) ? a->x0 : b->x0;
  u.y0 = (a->y0 < b->y0) ? a->y0 : b->y0;
  u.x1 = (a->x1 > b->x1) ? a->x1 : b->x1;
  u.y1 = (a->y1 > b->y1) ? a->y1 : b->y1;
  return u;
}

static void Dirty(int32_t x0, int32_t y0, int32_t x1, int32_t y1)
{
  Rect r;

  if (x0 < 0)        { x0 = 0; }
  if (y0 < 0)        { y0 = 0; }
  if (x1 > SCREEN_W) { x1 = SCREEN_W; }
  if (y1 > SCREEN_H) { y1 = SCREEN_H; }
  x0 &= ~1;                 /* CO5300: even start column, even width */
  x1 = (x1 + 1) & ~1;
  if ((x0 >= x1) || (y0 >= y1))
  {
    return;
  }
  r.x0 = x0; r.y0 = y0; r.x1 = x1; r.y1 = y1;

  /* merge with a rectangle it overlaps (or nearly), or when the list is full */
  for (int32_t i = 0; i < s_dirtyCount; i++)
  {
    Rect u = RectUnion(&s_dirty[i], &r);

    if ((RectArea(&u) <= (RectArea(&s_dirty[i]) + RectArea(&r) + 64)) || (s_dirtyCount == MAX_DIRTY))
    {
      s_dirty[i] = u;
      return;
    }
  }
  s_dirty[s_dirtyCount++] = r;
}

static void DirtyBox(float cx, float cy, float halfW, float halfH)
{
  Dirty((int32_t)floorf(cx - halfW), (int32_t)floorf(cy - halfH),
        (int32_t)ceilf(cx + halfW) + 1, (int32_t)ceilf(cy + halfH) + 1);
}

/* Area swept by a round ring end moving from angle 'from' to angle 'to' */
static void DirtyArcSweep(float rMid, float halfThick, float from, float to)
{
  float delta = fmodf(to - from, 360.0f);
  int32_t steps;

  if (delta > 180.0f)  { delta -= 360.0f; }
  if (delta < -180.0f) { delta += 360.0f; }
  steps = (int32_t)(fabsf(delta) / 3.0f) + 1;

  for (int32_t i = 0; i <= steps; i++)
  {
    float a = (from + delta * (float)i / (float)steps) * 0.01745329252f;

    DirtyBox(CX + sinf(a) * rMid, CY - cosf(a) * rMid, halfThick + 2.0f, halfThick + 2.0f);
  }
}

/* ============================================================ watch face == */

#define RING1_IN        213.0f      /* seconds "comet" */
#define RING1_OUT       229.0f
#define RING2_IN        192.0f
#define RING2_OUT       208.0f
#define RING3_IN        171.0f
#define RING3_OUT       187.0f
#define COMET_LEN       48.0f       /* degrees */
#define RING2_VALUE     0.72f
#define RING3_VALUE     0.46f

#define COL_BG          GFX_RGB(0, 0, 0)
#define COL_RING1       GFX_RGB(255, 45, 85)
#define COL_RING1_DIM   GFX_RGB(44, 8, 16)
#define COL_RING2       GFX_RGB(164, 255, 0)
#define COL_RING2_DIM   GFX_RGB(24, 40, 0)
#define COL_RING3       GFX_RGB(0, 220, 255)
#define COL_RING3_DIM   GFX_RGB(0, 34, 42)
#define COL_TIME        GFX_RGB(255, 255, 255)
#define COL_DATE        GFX_RGB(140, 150, 175)
#define COL_BPM         GFX_RGB(200, 200, 212)

#define DIGIT_W         50
#define DIGIT_H         92
#define DIGIT_T         11.0f
#define TIME_Y          175
static const int32_t s_digitX[4] = { 105, 167, 249, 311 };
#define COLON_X         233.0f

#define ECG_X0          153
#define ECG_X1          313
#define ECG_MID         306.0f
#define ECG_PERIOD      96.0f       /* pixels per heart beat */
#define ECG_SPEED       70.0f       /* pixels per second */

#define BPM_Y           336

typedef struct
{
  float    cometHead;     /* degrees 0..360 */
  float    cometLen;
  float    ring2End;      /* degrees */
  float    ring3End;
  int32_t  hours;
  int32_t  minutes;
  bool     colon;
  int32_t  bpm;
  float    ecgScroll;
} WatchView;

static WatchView s_watch;

static float EcgValue(float u)
{
  /* P wave, Q, R, S, T wave as narrow bumps along one beat (u = 0..1) */
  static const float bumps[5][3] =
  {
    { 0.20f, 0.035f,  0.12f },
    { 0.36f, 0.012f, -0.15f },
    { 0.39f, 0.014f,  1.00f },
    { 0.42f, 0.014f, -0.28f },
    { 0.62f, 0.050f,  0.22f },
  };
  float v = 0.0f;

  for (int32_t i = 0; i < 5; i++)
  {
    float d = (u - bumps[i][0]) / bumps[i][1];

    if ((d > -3.0f) && (d < 3.0f))
    {
      v += bumps[i][2] * expf(-d * d);
    }
  }
  return v;
}

static float EcgY(float x, float scroll)
{
  float u = (x + scroll) / ECG_PERIOD;

  u -= floorf(u);
  return ECG_MID - 18.0f * EcgValue(u);
}

static void WatchCompute(WatchView *v, uint32_t nowMs, uint32_t sceneMs)
{
  /* the demo clock starts at 10:09:30 */
  uint32_t totalMs = nowMs + (((10U * 60U + 9U) * 60U + 30U) * 1000U);
  uint32_t secs = totalMs / 1000U;
  float entry = (sceneMs >= 1200U) ? 1.0f : ((float)sceneMs / 1200.0f);
  static const uint8_t bpmTable[8] = { 72U, 74U, 71U, 69U, 73U, 76U, 72U, 70U };

  entry = 1.0f - (1.0f - entry) * (1.0f - entry) * (1.0f - entry);    /* ease out */

  v->cometHead = (float)(totalMs % 60000U) * 0.006f;
  v->cometLen  = COMET_LEN * entry;
  v->ring2End  = 360.0f * RING2_VALUE * entry;
  v->ring3End  = 360.0f * RING3_VALUE * entry;
  v->hours     = (int32_t)((secs / 3600U) % 24U);
  v->minutes   = (int32_t)((secs / 60U) % 60U);
  v->colon     = ((totalMs % 1000U) < 500U);
  v->bpm       = bpmTable[(nowMs / 4000U) % 8U];
  v->ecgScroll = fmodf((float)(nowMs % 960000U) * (ECG_SPEED / 1000.0f), ECG_PERIOD * 100.0f);
}

static void BpmText(char *out, int32_t bpm)
{
  out[0] = (char)('0' + (bpm / 10) % 10);
  out[1] = (char)('0' + bpm % 10);
  out[2] = ' '; out[3] = 'B'; out[4] = 'P'; out[5] = 'M'; out[6] = '\0';
}

static void WatchDraw(const GfxCanvas *c)
{
  const WatchView *v = &s_watch;
  char bpm[8];
  int32_t x0, x1;

  /* rings: dim track, then the bright part */
  Gfx_Arc(c, CX, CY, RING1_IN, RING1_OUT, 0.0f, 360.0f, COL_RING1_DIM);
  Gfx_Arc(c, CX, CY, RING2_IN, RING2_OUT, 0.0f, 360.0f, COL_RING2_DIM);
  Gfx_Arc(c, CX, CY, RING3_IN, RING3_OUT, 0.0f, 360.0f, COL_RING3_DIM);
  Gfx_Arc(c, CX, CY, RING1_IN, RING1_OUT, v->cometHead - v->cometLen, v->cometHead, COL_RING1);
  Gfx_Arc(c, CX, CY, RING2_IN, RING2_OUT, 0.0f, v->ring2End, COL_RING2);
  Gfx_Arc(c, CX, CY, RING3_IN, RING3_OUT, 0.0f, v->ring3End, COL_RING3);

  /* date */
  Gfx_Text(c, 174, 128, "SAT 14 NOV", 2, COL_DATE);

  /* time */
  Gfx_Digit(c, (float)s_digitX[0], TIME_Y, DIGIT_W, DIGIT_H, DIGIT_T, (uint32_t)v->hours / 10U, COL_TIME);
  Gfx_Digit(c, (float)s_digitX[1], TIME_Y, DIGIT_W, DIGIT_H, DIGIT_T, (uint32_t)v->hours % 10U, COL_TIME);
  Gfx_Digit(c, (float)s_digitX[2], TIME_Y, DIGIT_W, DIGIT_H, DIGIT_T, (uint32_t)v->minutes / 10U, COL_TIME);
  Gfx_Digit(c, (float)s_digitX[3], TIME_Y, DIGIT_W, DIGIT_H, DIGIT_T, (uint32_t)v->minutes % 10U, COL_TIME);
  if (v->colon)
  {
    Gfx_Disc(c, COLON_X, TIME_Y + 30.0f, 6.0f, COL_TIME);
    Gfx_Disc(c, COLON_X, TIME_Y + 62.0f, 6.0f, COL_TIME);
  }

  /* heart beat trace, fading in from the left */
  x0 = (c->x > ECG_X0) ? c->x : ECG_X0;
  x1 = ((c->x + c->w) < ECG_X1) ? (c->x + c->w) : ECG_X1;
  if ((c->y < 334) && ((c->y + c->h) > 280))
  {
    for (int32_t x = x0; x < x1; x++)
    {
      float ya = EcgY((float)x, v->ecgScroll);
      float yb = EcgY((float)x + 1.0f, v->ecgScroll);
      int32_t top = (int32_t)((ya < yb) ? ya : yb);
      int32_t bot = (int32_t)((ya < yb) ? yb : ya);
      uint32_t fade = (uint32_t)(x - ECG_X0) * 256U / 70U;

      if (fade > 256U) { fade = 256U; }
      Gfx_FillRect(c, x, top - 1, 1, bot - top + 3, Gfx_Mix(COL_BG, COL_RING1, fade));
    }
  }

  BpmText(bpm, v->bpm);
  Gfx_Text(c, 198, BPM_Y, bpm, 2, COL_BPM);
}

static void WatchUpdate(uint32_t nowMs, uint32_t sceneMs)
{
  WatchView n;
  float r1Mid = 0.5f * (RING1_IN + RING1_OUT), r1Half = 0.5f * (RING1_OUT - RING1_IN);

  WatchCompute(&n, nowMs, sceneMs);

  /* comet: its head and its tail moved */
  DirtyArcSweep(r1Mid, r1Half, s_watch.cometHead, n.cometHead);
  DirtyArcSweep(r1Mid, r1Half, s_watch.cometHead - s_watch.cometLen, n.cometHead - n.cometLen);
  if (n.ring2End != s_watch.ring2End)
  {
    DirtyArcSweep(0.5f * (RING2_IN + RING2_OUT), 0.5f * (RING2_OUT - RING2_IN), s_watch.ring2End, n.ring2End);
  }
  if (n.ring3End != s_watch.ring3End)
  {
    DirtyArcSweep(0.5f * (RING3_IN + RING3_OUT), 0.5f * (RING3_OUT - RING3_IN), s_watch.ring3End, n.ring3End);
  }
  if (n.hours != s_watch.hours)
  {
    Dirty(s_digitX[0] - 1, TIME_Y - 1, s_digitX[1] + DIGIT_W + 1, TIME_Y + DIGIT_H + 1);
  }
  if (n.minutes != s_watch.minutes)
  {
    Dirty(s_digitX[2] - 1, TIME_Y - 1, s_digitX[3] + DIGIT_W + 1, TIME_Y + DIGIT_H + 1);
  }
  if (n.colon != s_watch.colon)
  {
    Dirty(225, TIME_Y + 22, 241, TIME_Y + 70);
  }
  if (n.bpm != s_watch.bpm)
  {
    Dirty(198, BPM_Y, 198 + 24, BPM_Y + 14);
  }
  Dirty(ECG_X0, 284, ECG_X1, 328);    /* the trace scrolls every frame */

  s_watch = n;
}

/* ============================================================ orbs scene == */

#define ORB_COUNT       8
#define ORB_ARENA_R     226.0f
#define ORB_MAX_SPEED   520.0f

typedef struct
{
  float    x, y, vx, vy, r;
  uint16_t color;
} Orb;

static Orb s_orbs[ORB_COUNT];

static void OrbsInit(void)
{
  static const uint16_t colors[ORB_COUNT] =
  {
    GFX_RGB(255, 45, 85),  GFX_RGB(255, 149, 0), GFX_RGB(255, 214, 10), GFX_RGB(52, 199, 89),
    GFX_RGB(0, 220, 255),  GFX_RGB(10, 132, 255), GFX_RGB(175, 82, 222), GFX_RGB(255, 255, 255),
  };

  for (int32_t i = 0; i < ORB_COUNT; i++)
  {
    float a = (float)i * (6.2831853f / (float)ORB_COUNT);

    s_orbs[i].r     = 14.0f + (float)((i * 5) % 12);
    s_orbs[i].x     = CX + cosf(a) * 120.0f;
    s_orbs[i].y     = CY + sinf(a) * 120.0f;
    s_orbs[i].vx    = cosf(a * 2.3f + 1.0f) * (140.0f + 22.0f * (float)i);
    s_orbs[i].vy    = sinf(a * 1.7f + 0.5f) * (140.0f + 22.0f * (float)i);
    s_orbs[i].color = colors[i];
  }
}

static void OrbDirty(const Orb *o)
{
  DirtyBox(o->x, o->y, o->r + 2.0f, o->r + 2.0f);
}

static void OrbsUpdate(float dt, bool pressed, float tx, float ty)
{
  for (int32_t i = 0; i < ORB_COUNT; i++)
  {
    Orb *o = &s_orbs[i];
    float dx, dy, dist, limit, speed;

    OrbDirty(o);

    if (pressed)
    {
      /* a finger on the glass pulls the orbs towards it */
      float ax = tx - o->x, ay = ty - o->y;
      float d = sqrtf(ax * ax + ay * ay) + 1.0f;

      o->vx += ax / d * 900.0f * dt;
      o->vy += ay / d * 900.0f * dt;
    }
    o->x += o->vx * dt;
    o->y += o->vy * dt;

    /* bounce off the round edge of the screen */
    dx = o->x - CX;
    dy = o->y - CY;
    dist = sqrtf(dx * dx + dy * dy);
    limit = ORB_ARENA_R - o->r;
    if (dist > limit)
    {
      float nx = dx / dist, ny = dy / dist;
      float vn = o->vx * nx + o->vy * ny;

      if (vn > 0.0f)
      {
        o->vx -= 2.0f * vn * nx;
        o->vy -= 2.0f * vn * ny;
      }
      o->x = CX + nx * limit;
      o->y = CY + ny * limit;
    }

    speed = sqrtf(o->vx * o->vx + o->vy * o->vy);
    if (speed > ORB_MAX_SPEED)
    {
      o->vx *= ORB_MAX_SPEED / speed;
      o->vy *= ORB_MAX_SPEED / speed;
    }
  }

  /* orbs bounce off each other (equal masses: swap the speed along the contact line) */
  for (int32_t i = 0; i < ORB_COUNT; i++)
  {
    for (int32_t j = i + 1; j < ORB_COUNT; j++)
    {
      Orb *a = &s_orbs[i], *b = &s_orbs[j];
      float dx = b->x - a->x, dy = b->y - a->y;
      float d2 = dx * dx + dy * dy;
      float minD = a->r + b->r;

      if ((d2 < minD * minD) && (d2 > 0.01f))
      {
        float d = sqrtf(d2);
        float nx = dx / d, ny = dy / d;
        float rel = (b->vx - a->vx) * nx + (b->vy - a->vy) * ny;
        float push = 0.5f * (minD - d);

        if (rel < 0.0f)
        {
          a->vx += rel * nx; a->vy += rel * ny;
          b->vx -= rel * nx; b->vy -= rel * ny;
        }
        a->x -= nx * push; a->y -= ny * push;
        b->x += nx * push; b->y += ny * push;
      }
    }
  }

  for (int32_t i = 0; i < ORB_COUNT; i++)
  {
    OrbDirty(&s_orbs[i]);
  }
}

static void OrbsDraw(const GfxCanvas *c)
{
  Gfx_Arc(c, CX, CY, 228.0f, 231.0f, 0.0f, 360.0f, GFX_RGB(48, 48, 72));
  Gfx_Text(c, 156, 226, "TAP TO SWITCH", 2, GFX_RGB(56, 56, 80));

  for (int32_t i = 0; i < ORB_COUNT; i++)
  {
    const Orb *o = &s_orbs[i];

    Gfx_Disc(c, o->x, o->y, o->r, o->color);
    /* small highlight, up and to the left, for a glossy look */
    Gfx_Disc(c, o->x - o->r * 0.32f, o->y - o->r * 0.32f, o->r * 0.30f,
             Gfx_Mix(o->color, GFX_RGB(255, 255, 255), 150U));
  }
}

/* ================================================================= core == */

typedef enum { SCENE_WATCH = 0, SCENE_ORBS, SCENE_COUNT } Scene;

static Scene    s_scene;
static uint32_t s_sceneStart;
static uint32_t s_lastMs;
static bool     s_wasPressed;
static uint32_t s_pressStart;
static int32_t  s_pressX, s_pressY;
static int32_t  s_lastX, s_lastY;

static void SceneDraw(const GfxCanvas *c)
{
  Gfx_Clear(c, COL_BG);
  if (s_scene == SCENE_WATCH)
  {
    WatchDraw(c);
  }
  else
  {
    OrbsDraw(c);
  }
}

/* Re-render every dirty rectangle and send it to the panel */
static void Flush(void)
{
  s_framePixels = 0U;

  for (int32_t i = 0; i < s_dirtyCount; i++)
  {
    const Rect *r = &s_dirty[i];
    int32_t w = r->x1 - r->x0;
    int32_t h = r->y1 - r->y0;
    int32_t bandRows = BAND_PIXELS / w;

    if (CO5300_SetWindow((uint16_t)r->x0, (uint16_t)r->y0, (uint16_t)w, (uint16_t)h) != HAL_OK)
    {
      continue;
    }
    for (int32_t y = r->y0; y < r->y1; y += bandRows)
    {
      GfxCanvas c;

      c.px = s_band[s_bandIdx];
      c.x  = (int16_t)r->x0;
      c.y  = (int16_t)y;
      c.w  = (int16_t)w;
      c.h  = (int16_t)(((r->y1 - y) < bandRows) ? (r->y1 - y) : bandRows);

      SceneDraw(&c);
      /* returns while DMA is sending; the other buffer is free by then */
      (void)CO5300_WriteRaw((const uint8_t *)c.px, (uint32_t)c.w * (uint32_t)c.h * 2U);
      s_bandIdx ^= 1U;
    }
    s_framePixels += (uint32_t)(w * h);
  }
  s_dirtyCount = 0;
  (void)CO5300_WaitIdle();
}

static void SceneEnter(Scene scene, uint32_t nowMs)
{
  s_scene = scene;
  s_sceneStart = nowMs;
  if (scene == SCENE_WATCH)
  {
    WatchCompute(&s_watch, nowMs, 0U);
  }
  else
  {
    OrbsInit();
  }
  s_dirtyCount = 0;
  Dirty(0, 0, SCREEN_W, SCREEN_H);
}

void UI_Init(uint32_t nowMs)
{
  s_lastMs = nowMs;
  s_wasPressed = false;
  SceneEnter(SCENE_WATCH, nowMs);
  Flush();
}

void UI_Frame(uint32_t nowMs, bool pressed, int32_t touchX, int32_t touchY)
{
  float dt = (float)(nowMs - s_lastMs) * 0.001f;

  if (dt > 0.05f)
  {
    dt = 0.05f;   /* keep the physics stable after a slow frame */
  }
  s_lastMs = nowMs;

  /* a short press without much movement is a tap: next scene */
  if (pressed)
  {
    if (!s_wasPressed)
    {
      s_pressStart = nowMs;
      s_pressX = touchX;
      s_pressY = touchY;
    }
    s_lastX = touchX;
    s_lastY = touchY;
  }
  else if (s_wasPressed)
  {
    int32_t dx = s_lastX - s_pressX, dy = s_lastY - s_pressY;

    if (((nowMs - s_pressStart) < 350U) && ((dx * dx + dy * dy) < (30 * 30)))
    {
      SceneEnter((Scene)((s_scene + 1) % SCENE_COUNT), nowMs);
    }
  }
  s_wasPressed = pressed;

  if (s_scene == SCENE_WATCH)
  {
    WatchUpdate(nowMs, nowMs - s_sceneStart);
  }
  else
  {
    OrbsUpdate(dt, pressed, (float)touchX, (float)touchY);
  }
  Flush();
}

uint32_t UI_LastFramePixels(void)
{
  return s_framePixels;
}
