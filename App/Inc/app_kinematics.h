#ifndef __APP_KINEMATICS_H
#define __APP_KINEMATICS_H

#include <stdint.h>

/*
 * 屏幕坐标 -> 云台脉冲 解算
 *   相机到屏幕距离 KIN_SCREEN_DIST_MM（默认 1000mm，需标定）
 *   pan 角  = atan(x / D)，tilt 角 = atan(y / D)
 *   电机 1.8°，16 细分 = 3200 脉冲/圈 = 8.8889 脉冲/度
 *   KIN_PAN_GAIN / KIN_TILT_GAIN 用于实际标定修正（默认 1.0）
 */
#define KIN_SCREEN_DIST_MM 1000.0
#define KIN_PULSES_PER_DEG (3200.0 / 360.0)
#define KIN_PAN_GAIN       1.0
#define KIN_TILT_GAIN      1.0

void Kin_Init(void);

/* 屏幕 0.1mm 定点坐标 -> 相对云台零位的绝对目标脉冲 */
void Kin_ScreenToAimPulses(int16_t x_01mm, int16_t y_01mm,
                           int32_t *pan_pulses, int32_t *tilt_pulses);

/* 屏幕坐标 -> 角度（调试用） */
void Kin_ScreenToAnglesDeg(int16_t x_01mm, int16_t y_01mm,
                           double *pan_deg, double *tilt_deg);

#endif
