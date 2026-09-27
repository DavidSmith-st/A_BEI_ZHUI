#include "app_kinematics.h"
#include <math.h>

#define KIN_PI 3.14159265358979323846

void Kin_Init(void) {}

void Kin_ScreenToAnglesDeg(int16_t x_01mm, int16_t y_01mm,
                           double *pan_deg, double *tilt_deg) {
  double x = (double)x_01mm / 10.0;
  double y = (double)y_01mm / 10.0;
  double d = KIN_SCREEN_DIST_MM;
  if (pan_deg != NULL) *pan_deg = atan(x / d) * (180.0 / KIN_PI);
  if (tilt_deg != NULL) *tilt_deg = atan(y / d) * (180.0 / KIN_PI);
}

void Kin_ScreenToAimPulses(int16_t x_01mm, int16_t y_01mm,
                           int32_t *pan_pulses, int32_t *tilt_pulses) {
  double pan_deg = 0.0, tilt_deg = 0.0;
  Kin_ScreenToAnglesDeg(x_01mm, y_01mm, &pan_deg, &tilt_deg);
  if (pan_pulses != NULL) {
    double v = pan_deg * KIN_PULSES_PER_DEG * KIN_PAN_GAIN;
    *pan_pulses = (v >= 0.0) ? (int32_t)(v + 0.5) : (int32_t)(v - 0.5);
  }
  if (tilt_pulses != NULL) {
    double v = tilt_deg * KIN_PULSES_PER_DEG * KIN_TILT_GAIN;
    *tilt_pulses = (v >= 0.0) ? (int32_t)(v + 0.5) : (int32_t)(v - 0.5);
  }
}
