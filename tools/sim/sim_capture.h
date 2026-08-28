/* Framebuffer the stubbed COG driver last received, for the renderer. */
#pragma once

#include <stdint.h>
#include "epd_cog.h"

extern uint8_t sim_frame[EPD_FRAME_SIZE];  /* frame pushed to the panel */
extern uint8_t sim_prev[EPD_FRAME_SIZE];   /* previous frame, fast update only */
extern int     sim_update_mode;            /* EPD_UPDATE_NORMAL / _FAST */
extern int8_t  sim_temp_reg;               /* value handed to the waveform reg */
