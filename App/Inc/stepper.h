#ifndef __STEPPER_H
#define __STEPPER_H

#include "main.h"
#include "tim.h"
#include <stdint.h>

/*
 * CubeMX pin assignment for the first gimbal test:
 *   PAN_STEP  = TIM1_CH2 / PE11
 *   TILT_STEP = TIM8_CH1 / PC6
 *   PAN_DIR   = PE12, PAN_EN   = PD13
 *   TILT_DIR  = PD6, TILT_EN = PC7
 * Both PD42S1 drivers must be configured as pulse mode, EN active high.
 */
#define PAN_DIR_GPIO_Port   GPIOE
#define PAN_DIR_Pin         GPIO_PIN_12
#define PAN_EN_GPIO_Port    GPIOD
#define PAN_EN_Pin          GPIO_PIN_13
#define TILT_DIR_GPIO_Port  GPIOD
#define TILT_DIR_Pin        GPIO_PIN_6
#define TILT_EN_GPIO_Port   GPIOC
#define TILT_EN_Pin         GPIO_PIN_7

/* PD42S1 PLC + common-cathode wiring: EN high = motor enabled. */
#define STEPPER_EN_ACTIVE_LEVEL GPIO_PIN_SET
#define STEPPER_EN_IDLE_LEVEL   GPIO_PIN_RESET
/* Keep the driver enabled whenever no STEP pulse is being generated. */
#define STEPPER_HOLD_WHEN_IDLE  1U
#define STEPPER_DIR_POS_LEVEL   GPIO_PIN_SET
#define STEPPER_DIR_NEG_LEVEL   GPIO_PIN_RESET

#define STEPPER_PAN_TIMER   (&htim1)
#define STEPPER_TILT_TIMER  (&htim8)
#define STEPPER_PAN_CHANNEL  TIM_CHANNEL_2
#define STEPPER_TILT_CHANNEL TIM_CHANNEL_1

#define STEPPER_MIN_FREQ_HZ 50U
#define STEPPER_MAX_FREQ_HZ 20000U

/* Call once after MX_TIM1_Init/MX_TIM8_Init. */
void Stepper_Init(void);
/* Start a relative move. Positive/negative steps select direction. */
HAL_StatusTypeDef Stepper_MoveRelative(uint8_t axis, int32_t steps, uint32_t freq_hz);
void Stepper_StopAll(void);
/* Stop STEP pulses but keep EN active so the motor holds its position. */
void Stepper_HoldAll(void);
void Stepper_Pause(void);
void Stepper_Resume(void);
void Stepper_ZeroSoftwarePosition(void);
void Stepper_Service(void);

uint8_t Stepper_IsBusy(void);
uint8_t Stepper_IsPaused(void);
int32_t Stepper_GetPosition(uint8_t axis);
int32_t Stepper_GetRemaining(uint8_t axis);

/* Implemented by the application; called after a move finishes. */
void Stepper_OnMoveFinished(uint8_t axis);

#endif



