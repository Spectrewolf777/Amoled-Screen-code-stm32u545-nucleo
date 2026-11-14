/**
  * @file    ui.h
  * @brief   Demo user interface for the round 466x466 AMOLED:
  *          a watch face and a bouncing-orbs animation. Tap to switch.
  */
#ifndef UI_H
#define UI_H

#include <stdint.h>
#include <stdbool.h>

/** Draw the first screen. nowMs is a free-running millisecond counter. */
void UI_Init(uint32_t nowMs);

/**
  * Advance the animation to nowMs and redraw what changed. Call it once per
  * frame with the current touch state (x/y only matter while pressed).
  */
void UI_Frame(uint32_t nowMs, bool pressed, int32_t touchX, int32_t touchY);

/** Pixels sent to the display by the last UI_Frame() call (for statistics). */
uint32_t UI_LastFramePixels(void);

#endif /* UI_H */
