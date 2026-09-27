#include "app_display.h"
#include "oled.h"
#include "main.h"
#include <stdio.h>

static uint32_t last_oled_ms;
static char mode_text[8] = "本地";
static char state_text[8] = "等待";
static char spot_x_text[8] = "----";
static char spot_y_text[8] = "----";

void AppDisplay_SetMode(const char *text) {
  (void)snprintf(mode_text, sizeof(mode_text), "%s", text);
}

void AppDisplay_SetVisionState(const char *text) {
  (void)snprintf(state_text, sizeof(state_text), "%s", text);
}

static void format_mm(int16_t v01, char *out, size_t n) {
  uint8_t negative = (v01 < 0) ? 1U : 0U;
  int16_t whole = v01 / 10;
  int16_t frac = v01 % 10;
  if (whole < 0) whole = -whole;
  if (frac < 0) frac = -frac;
  (void)snprintf(out, n, "%s%d.%d", negative ? "-" : "", whole, frac);
}

void AppDisplay_SetSpot(int16_t x_01mm, int16_t y_01mm) {
  format_mm(x_01mm, spot_x_text, sizeof(spot_x_text));
  format_mm(y_01mm, spot_y_text, sizeof(spot_y_text));
}

void AppDisplay_Init(void) {
  HAL_Delay(30);
  OLED_Init();
  last_oled_ms = 0U;
}

void AppDisplay_Task(void) {
  uint32_t now = HAL_GetTick();
  if ((now - last_oled_ms) < 100U) return;
  last_oled_ms = now;

  OLED_NewFrame();
  OLED_PrintString(0, 0, "运动目标云台", &font16x16, OLED_COLOR_NORMAL);
  OLED_PrintString(0, 16, mode_text, &font16x16, OLED_COLOR_NORMAL);
  OLED_PrintString(40, 16, state_text, &font16x16, OLED_COLOR_NORMAL);
  OLED_PrintString(0, 32, "水平", &font16x16, OLED_COLOR_NORMAL);
  OLED_PrintASCIIString(40, 32, spot_x_text, &afont16x8, OLED_COLOR_NORMAL);
  OLED_PrintString(0, 48, "俯仰", &font16x16, OLED_COLOR_NORMAL);
  OLED_PrintASCIIString(40, 48, spot_y_text, &afont16x8, OLED_COLOR_NORMAL);
  OLED_ShowFrame();
}

