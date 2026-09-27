#include "stepper.h"
#include <stdlib.h>

#define AXIS_PAN  0U
#define AXIS_TILT 1U

typedef struct {
  TIM_HandleTypeDef *htim;
  uint32_t channel;
  GPIO_TypeDef *dir_port;
  uint16_t dir_pin;
  GPIO_TypeDef *en_port;
  uint16_t en_pin;
  volatile int32_t position;
  volatile int32_t remaining;
  volatile int32_t paused_remaining;
  volatile uint8_t active;
  volatile uint8_t paused;
  int8_t direction;
  uint32_t freq_hz;
} StepperAxis;

static StepperAxis axes[2] = {
  {STEPPER_PAN_TIMER, STEPPER_PAN_CHANNEL, PAN_DIR_GPIO_Port, PAN_DIR_Pin,
   PAN_EN_GPIO_Port, PAN_EN_Pin, 0, 0, 0, 0, 0, 1, 1000},
  {STEPPER_TILT_TIMER, STEPPER_TILT_CHANNEL, TILT_DIR_GPIO_Port, TILT_DIR_Pin,
   TILT_EN_GPIO_Port, TILT_EN_Pin, 0, 0, 0, 0, 0, 1, 1000}
};

static StepperAxis *axis_from_timer(TIM_HandleTypeDef *htim) {
  for (uint8_t i = 0; i < 2; ++i) {
    if (axes[i].htim == htim) return &axes[i];
  }
  return NULL;
}

static void timer_set_frequency(StepperAxis *a, uint32_t freq_hz) {
  uint32_t timer_clk = 1000000U; /* PSC=167 -> 1 MHz timer counter */
  uint32_t period = timer_clk / freq_hz;
  if (period < 2U) period = 2U;
  if (period > 65536U) period = 65536U;
  __HAL_TIM_SET_PRESCALER(a->htim, 167U);
  __HAL_TIM_SET_AUTORELOAD(a->htim, period - 1U);
  __HAL_TIM_SET_COMPARE(a->htim, a->channel, period / 2U);
  __HAL_TIM_SET_COUNTER(a->htim, 0U);
  a->htim->Instance->EGR = TIM_EGR_UG;
  a->freq_hz = timer_clk / period;
}

static void axis_stop_pulses(StepperAxis *a) {
  (void)HAL_TIM_PWM_Stop_IT(a->htim, a->channel);
  a->active = 0U;
#if STEPPER_HOLD_WHEN_IDLE
  /* Do not drop EN: the PD42S1 must keep current and hold the position. */
  HAL_GPIO_WritePin(a->en_port, a->en_pin, STEPPER_EN_ACTIVE_LEVEL);
#else
  HAL_GPIO_WritePin(a->en_port, a->en_pin, STEPPER_EN_IDLE_LEVEL);
#endif
}

void Stepper_Init(void) {
  for (uint8_t i = 0; i < 2; ++i) {
    HAL_GPIO_WritePin(axes[i].dir_port, axes[i].dir_pin, STEPPER_DIR_POS_LEVEL);
#if STEPPER_HOLD_WHEN_IDLE
    HAL_GPIO_WritePin(axes[i].en_port, axes[i].en_pin, STEPPER_EN_ACTIVE_LEVEL);
#else
    HAL_GPIO_WritePin(axes[i].en_port, axes[i].en_pin, STEPPER_EN_IDLE_LEVEL);
#endif
    timer_set_frequency(&axes[i], 1000U);
  }
}

HAL_StatusTypeDef Stepper_MoveRelative(uint8_t axis, int32_t steps, uint32_t freq_hz) {
  if (axis > AXIS_TILT || steps == 0) return HAL_ERROR;
  if (freq_hz < STEPPER_MIN_FREQ_HZ || freq_hz > STEPPER_MAX_FREQ_HZ) return HAL_ERROR;
  StepperAxis *a = &axes[axis];
  if (a->active || a->paused) return HAL_BUSY;

  a->direction = (steps >= 0) ? 1 : -1;
  a->remaining = (steps >= 0) ? steps : -steps;
  HAL_GPIO_WritePin(a->dir_port, a->dir_pin,
                    (a->direction > 0) ? STEPPER_DIR_POS_LEVEL : STEPPER_DIR_NEG_LEVEL);
  timer_set_frequency(a, freq_hz);
  HAL_GPIO_WritePin(a->en_port, a->en_pin, STEPPER_EN_ACTIVE_LEVEL);
  a->active = 1U;
  a->paused = 0U;
  if (HAL_TIM_PWM_Start_IT(a->htim, a->channel) != HAL_OK) {
    axis_stop_pulses(a);
    return HAL_ERROR;
  }
  return HAL_OK;
}

void Stepper_HoldAll(void) {
  for (uint8_t i = 0; i < 2; ++i) {
    axis_stop_pulses(&axes[i]);
    axes[i].paused = 0U;
  }
}

void Stepper_StopAll(void) {
  Stepper_HoldAll();
  for (uint8_t i = 0; i < 2; ++i) {
    axes[i].remaining = 0;
    axes[i].paused_remaining = 0;
  }
}

void Stepper_Pause(void) {
  for (uint8_t i = 0; i < 2; ++i) {
    if (axes[i].active) {
      (void)HAL_TIM_PWM_Stop_IT(axes[i].htim, axes[i].channel);
      HAL_GPIO_WritePin(axes[i].en_port, axes[i].en_pin, STEPPER_EN_ACTIVE_LEVEL);
      axes[i].paused_remaining = axes[i].remaining;
      axes[i].active = 0U;
      axes[i].paused = 1U;
    }
  }
}

void Stepper_Resume(void) {
  for (uint8_t i = 0; i < 2; ++i) {
    if (axes[i].paused && axes[i].paused_remaining > 0) {
      axes[i].remaining = axes[i].paused_remaining;
      axes[i].paused_remaining = 0;
      HAL_GPIO_WritePin(axes[i].dir_port, axes[i].dir_pin,
                        (axes[i].direction > 0) ? STEPPER_DIR_POS_LEVEL : STEPPER_DIR_NEG_LEVEL);
      timer_set_frequency(&axes[i], axes[i].freq_hz);
      HAL_GPIO_WritePin(axes[i].en_port, axes[i].en_pin, STEPPER_EN_ACTIVE_LEVEL);
      axes[i].paused = 0U;
      axes[i].active = 1U;
      (void)HAL_TIM_PWM_Start_IT(axes[i].htim, axes[i].channel);
    }
  }
}

void Stepper_ZeroSoftwarePosition(void) {
  __disable_irq();
  axes[0].position = 0;
  axes[1].position = 0;
  __enable_irq();
}

void Stepper_Service(void) { /* reserved for future acceleration/limit handling */ }

uint8_t Stepper_IsBusy(void) {
  return (axes[0].active || axes[0].paused || axes[1].active || axes[1].paused) ? 1U : 0U;
}
uint8_t Stepper_IsPaused(void) { return (axes[0].paused || axes[1].paused) ? 1U : 0U; }
int32_t Stepper_GetPosition(uint8_t axis) { return (axis < 2U) ? axes[axis].position : 0; }
int32_t Stepper_GetRemaining(uint8_t axis) { return (axis < 2U) ? axes[axis].remaining : 0; }

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
  StepperAxis *a = axis_from_timer(htim);
  if (a == NULL || !a->active) return;
  if (a->remaining > 0) {
    a->remaining--;
    a->position += a->direction;
  }
  if (a->remaining <= 0) {
    uint8_t axis = (a == &axes[AXIS_PAN]) ? AXIS_PAN : AXIS_TILT;
    axis_stop_pulses(a);
    a->paused = 0U;
    Stepper_OnMoveFinished(axis);
  }
}

__weak void Stepper_OnMoveFinished(uint8_t axis) {
  (void)axis;
}
