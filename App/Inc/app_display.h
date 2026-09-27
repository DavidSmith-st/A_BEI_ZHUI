#ifndef __APP_DISPLAY_H
#define __APP_DISPLAY_H

#include <stdint.h>

void AppDisplay_Init(void);
void AppDisplay_Task(void);
void AppDisplay_SetMode(const char *text);
void AppDisplay_SetVisionState(const char *text);
void AppDisplay_SetSpot(int16_t x_01mm, int16_t y_01mm);

#endif
