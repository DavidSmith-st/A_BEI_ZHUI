#include "app.h"
#include "app_display.h"
#include "app_kinematics.h"
#include "app_protocol.h"
#include "stepper.h"
#include "main.h"
#include <stdlib.h>
#include <string.h>

#define TRACE_EDGE_FREQ_HZ       50U
#define HOME_FREQ_HZ             100U
#define TRACE_DEADBAND           3
#define VISION_TIMEOUT_MS        200U
#define VISION_INVALID_LIMIT     8U
#define SCREEN_LIMIT_01MM        3500
#define SPOT_FILTER_ALPHA_N      1
#define SPOT_FILTER_ALPHA_D      4
#define KEY_DEBOUNCE_MS          50U

/* A4 closed-loop parameters. */
#define CL_SEGMENT_PULSES        12U
#define CL_GAIN_N                1
#define CL_GAIN_D                2
#define CL_MAX_CORR_PULSES       10
#define CL_ERROR_DEADBAND_01MM   10
#define CL_FEEDBACK_MAX_AGE_MS   160U

/* 0.5 m x 0.5 m screen square at 1 m distance. */
#define SQUARE_HALF_PULSES       125
#define KEY_ACTIVE_LEVEL         GPIO_PIN_RESET

typedef enum {
  MODE_FIXED_SQUARE = 0,
  MODE_A4_VISION
} WorkMode;

typedef enum {
  TASK_IDLE = 0,
  TASK_FIXED_TO_TL,
  TASK_FIXED_TOP,
  TASK_FIXED_RIGHT,
  TASK_FIXED_BOTTOM,
  TASK_FIXED_LEFT,
  TASK_VISION_TO_FIRST,
  TASK_VISION_EDGE,
  TASK_WAIT_VISION,
  TASK_RETURN_CENTER,
  TASK_HOMING,
  TASK_DONE,
  TASK_ERROR
} TaskState;

typedef struct {
  int32_t pan;
  int32_t tilt;
} Pulse2D;

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
  uint8_t raw;
  uint8_t stable;
  uint32_t changed_ms;
} KeyState;

static WorkMode work_mode;
static TaskState task_state;
static uint8_t task_paused;
static uint8_t vision_armed;
static uint8_t vision_invalid_count;

static ProtoPoint vision_points[4];
static Pulse2D vision_vertices[4];
static uint8_t vision_edge_from;
static uint8_t vision_edge_to;
static uint8_t vision_segment_count;
static uint8_t vision_segment_done;

static int32_t spot_filt_x;
static int32_t spot_filt_y;
static uint8_t spot_filter_init;
static ProtoPoint latest_red_spot;
static uint8_t latest_red_valid;
static uint32_t latest_red_ms;

static KeyState key_home   = {KEY_HOME_GPIO_Port,   KEY_HOME_Pin,   0U, 0U, 0U};
static KeyState key_start  = {KEY_START_GPIO_Port,  KEY_START_Pin,  0U, 0U, 0U};
static KeyState key_pause  = {KEY_PAUSE_GPIO_Port,  KEY_PAUSE_Pin,  0U, 0U, 0U};
static KeyState key_resume = {KEY_RESUME_GPIO_Port, KEY_RESUME_Pin, 0U, 0U, 0U};
static KeyState key_mode   = {KEY_MODE_GPIO_Port,   KEY_MODE_Pin,   0U, 0U, 0U};

static int16_t Clamp01mm(int16_t value) {
  if (value > SCREEN_LIMIT_01MM) return SCREEN_LIMIT_01MM;
  if (value < -SCREEN_LIMIT_01MM) return -SCREEN_LIMIT_01MM;
  return value;
}

static int32_t ClampI32(int32_t value, int32_t limit) {
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

static int16_t Filter01mm(int32_t *state, int16_t raw, uint8_t init) {
  int32_t target = (int32_t)raw * 10;
  if (init) {
    *state = target;
  } else {
    *state += (target - *state) * SPOT_FILTER_ALPHA_N / SPOT_FILTER_ALPHA_D;
  }
  return (*state >= 0) ? (int16_t)((*state + 5) / 10)
                       : (int16_t)((*state - 5) / 10);
}

static HAL_StatusTypeDef MoveToPulse(int32_t target_pan, int32_t target_tilt,
                                     uint32_t freq_hz) {
  int32_t d_pan = target_pan - Stepper_GetPosition(0U);
  int32_t d_tilt = target_tilt - Stepper_GetPosition(1U);

  if (abs(d_pan) <= TRACE_DEADBAND) d_pan = 0;
  if (abs(d_tilt) <= TRACE_DEADBAND) d_tilt = 0;
  if (d_pan == 0 && d_tilt == 0) return HAL_OK;

  if (d_pan != 0 && Stepper_MoveRelative(0U, d_pan, freq_hz) != HAL_OK) {
    return HAL_ERROR;
  }
  if (d_tilt != 0 && Stepper_MoveRelative(1U, d_tilt, freq_hz) != HAL_OK) {
    Stepper_StopAll();
    return HAL_ERROR;
  }
  return HAL_OK;
}

static uint8_t TaskIsRunning(void) {
  switch (task_state) {
    case TASK_FIXED_TO_TL:
    case TASK_FIXED_TOP:
    case TASK_FIXED_RIGHT:
    case TASK_FIXED_BOTTOM:
    case TASK_FIXED_LEFT:
    case TASK_VISION_TO_FIRST:
    case TASK_VISION_EDGE:
    case TASK_RETURN_CENTER:
    case TASK_HOMING:
      return 1U;
    default:
      return 0U;
  }
}

static void SetModeDisplay(void) {
  AppDisplay_SetMode((work_mode == MODE_FIXED_SQUARE) ? "本地" : "追踪");
}

static void ResetFeedback(void) {
  spot_filter_init = 1U;
  latest_red_valid = 0U;
  latest_red_ms = 0U;
}

static void ClearTask(void) {
  Stepper_StopAll();
  task_state = TASK_IDLE;
  task_paused = 0U;
  vision_armed = 0U;
  vision_invalid_count = 0U;
  ResetFeedback();
}

static void SelectMode(WorkMode mode) {
  ClearTask();
  work_mode = mode;
  SetModeDisplay();
  AppDisplay_SetVisionState("等待");
}

static void StartHome(void) {
  ClearTask();
  task_state = TASK_HOMING;
  if (MoveToPulse(0, 0, HOME_FREQ_HZ) != HAL_OK) {
    task_state = TASK_ERROR;
    AppDisplay_SetVisionState("故障");
    return;
  }
  AppDisplay_SetVisionState("复位");
}

static void StartFixedSquare(void) {
  ClearTask();
  task_state = TASK_FIXED_TO_TL;
  if (MoveToPulse(-SQUARE_HALF_PULSES, SQUARE_HALF_PULSES,
                  TRACE_EDGE_FREQ_HZ) != HAL_OK) {
    task_state = TASK_ERROR;
    AppDisplay_SetVisionState("故障");
    return;
  }
  AppDisplay_SetVisionState("运动");
}

static void StartVisionWait(void) {
  ClearTask();
  task_state = TASK_WAIT_VISION;
  vision_armed = 1U;
  AppDisplay_SetVisionState("等待");
}

static void StartCurrentMode(void) {
  if (task_state == TASK_HOMING) return;
  if (work_mode == MODE_FIXED_SQUARE) StartFixedSquare();
  else StartVisionWait();
}

static void PauseTask(void) {
  if (!TaskIsRunning() || task_paused) return;
  Stepper_Pause();
  task_paused = 1U;
  AppDisplay_SetVisionState("暂停");
}

static void ResumeTask(void) {
  if (!task_paused) return;
  task_paused = 0U;
  Stepper_Resume();
  AppDisplay_SetVisionState("运动");
}

static uint8_t KeyReadPressed(const KeyState *key) {
  return (HAL_GPIO_ReadPin(key->port, key->pin) == KEY_ACTIVE_LEVEL) ? 1U : 0U;
}

static void KeyInit(KeyState *key) {
  key->raw = KeyReadPressed(key);
  key->stable = key->raw;
  key->changed_ms = HAL_GetTick();
}

static uint8_t KeyPressed(KeyState *key) {
  uint32_t now = HAL_GetTick();
  uint8_t raw = KeyReadPressed(key);

  if (raw != key->raw) {
    key->raw = raw;
    key->changed_ms = now;
  }
  if (key->stable != key->raw && (now - key->changed_ms) >= KEY_DEBOUNCE_MS) {
    key->stable = key->raw;
    return key->stable;
  }
  return 0U;
}

static void KeysInit(void) {
  KeyInit(&key_home);
  KeyInit(&key_start);
  KeyInit(&key_pause);
  KeyInit(&key_resume);
  KeyInit(&key_mode);
}

static void KeysTask(void) {
  if (KeyPressed(&key_home)) {
    StartHome();
    return;
  }
  if (KeyPressed(&key_mode)) {
    SelectMode((work_mode == MODE_FIXED_SQUARE) ? MODE_A4_VISION
                                                : MODE_FIXED_SQUARE);
    return;
  }
  if (KeyPressed(&key_pause)) {
    PauseTask();
    return;
  }
  if (KeyPressed(&key_resume)) {
    ResumeTask();
    return;
  }
  if (KeyPressed(&key_start)) StartCurrentMode();
}

static ProtoPoint InterpolatePoint(const ProtoPoint *from, const ProtoPoint *to,
                                   uint8_t numerator, uint8_t denominator) {
  ProtoPoint result;
  result.x_01mm = (int16_t)((int32_t)from->x_01mm +
      ((int32_t)(to->x_01mm - from->x_01mm) * numerator) / denominator);
  result.y_01mm = (int16_t)((int32_t)from->y_01mm +
      ((int32_t)(to->y_01mm - from->y_01mm) * numerator) / denominator);
  return result;
}

static void UpdateRedFeedback(const ProtoVisionFrame *frame) {
  if (!frame->red_valid) {
    latest_red_valid = 0U;
    return;
  }

  int16_t x = Clamp01mm(frame->red_spot.x_01mm);
  int16_t y = Clamp01mm(frame->red_spot.y_01mm);
  x = Filter01mm(&spot_filt_x, x, spot_filter_init);
  y = Filter01mm(&spot_filt_y, y, spot_filter_init);
  spot_filter_init = 0U;

  latest_red_spot.x_01mm = x;
  latest_red_spot.y_01mm = y;
  latest_red_valid = 1U;
  latest_red_ms = HAL_GetTick();
  AppDisplay_SetSpot(x, y);
}

static void ComputeClosedLoopCorrection(const ProtoPoint *expected,
                                        int32_t *corr_pan, int32_t *corr_tilt) {
  *corr_pan = 0;
  *corr_tilt = 0;
  if (!latest_red_valid || (HAL_GetTick() - latest_red_ms) > CL_FEEDBACK_MAX_AGE_MS) return;

  int32_t err_x = (int32_t)latest_red_spot.x_01mm - expected->x_01mm;
  int32_t err_y = (int32_t)latest_red_spot.y_01mm - expected->y_01mm;
  if (abs(err_x) <= CL_ERROR_DEADBAND_01MM) err_x = 0;
  if (abs(err_y) <= CL_ERROR_DEADBAND_01MM) err_y = 0;

  int32_t expected_pan, expected_tilt, actual_pan, actual_tilt;
  Kin_ScreenToAimPulses(expected->x_01mm, expected->y_01mm,
                        &expected_pan, &expected_tilt);
  Kin_ScreenToAimPulses(latest_red_spot.x_01mm, latest_red_spot.y_01mm,
                        &actual_pan, &actual_tilt);

  if (err_x != 0) {
    *corr_pan = -((actual_pan - expected_pan) * CL_GAIN_N / CL_GAIN_D);
    *corr_pan = ClampI32(*corr_pan, CL_MAX_CORR_PULSES);
  }
  if (err_y != 0) {
    *corr_tilt = -((actual_tilt - expected_tilt) * CL_GAIN_N / CL_GAIN_D);
    *corr_tilt = ClampI32(*corr_tilt, CL_MAX_CORR_PULSES);
  }
}

static HAL_StatusTypeDef CommandVisionSegment(void) {
  const ProtoPoint *from = &vision_points[vision_edge_from];
  const ProtoPoint *to = &vision_points[vision_edge_to];
  ProtoPoint expected_current = InterpolatePoint(from, to,
                                                  vision_segment_done,
                                                  vision_segment_count);
  int32_t corr_pan, corr_tilt;
  ComputeClosedLoopCorrection(&expected_current, &corr_pan, &corr_tilt);

  if (vision_segment_done >= vision_segment_count) return HAL_OK;

  vision_segment_done++;
  ProtoPoint target = InterpolatePoint(from, to, vision_segment_done,
                                       vision_segment_count);
  int32_t pan, tilt;
  Kin_ScreenToAimPulses(target.x_01mm, target.y_01mm, &pan, &tilt);
  return MoveToPulse(pan + corr_pan, tilt + corr_tilt, TRACE_EDGE_FREQ_HZ);
}

static void StartVisionEdge(uint8_t from, uint8_t to) {
  vision_edge_from = from;
  vision_edge_to = to;
  vision_segment_done = 0U;

  int32_t dp = vision_vertices[to].pan - vision_vertices[from].pan;
  int32_t dt = vision_vertices[to].tilt - vision_vertices[from].tilt;
  int32_t max_delta = (abs(dp) > abs(dt)) ? abs(dp) : abs(dt);
  uint32_t count = ((uint32_t)max_delta + CL_SEGMENT_PULSES - 1U) /
                   CL_SEGMENT_PULSES;
  if (count < 1U) count = 1U;
  if (count > 255U) count = 255U;
  vision_segment_count = (uint8_t)count;
  task_state = TASK_VISION_EDGE;

  if (CommandVisionSegment() != HAL_OK) {
    task_state = TASK_ERROR;
    AppDisplay_SetVisionState("故障");
  }
}

static void StartVisionPath(const ProtoVisionFrame *frame) {
  for (uint8_t i = 0U; i < 4U; ++i) {
    vision_points[i].x_01mm = Clamp01mm(frame->vertices[i].x_01mm);
    vision_points[i].y_01mm = Clamp01mm(frame->vertices[i].y_01mm);
    Kin_ScreenToAimPulses(vision_points[i].x_01mm, vision_points[i].y_01mm,
                          &vision_vertices[i].pan, &vision_vertices[i].tilt);
  }

  vision_armed = 0U;
  task_state = TASK_VISION_TO_FIRST;
  if (MoveToPulse(vision_vertices[0].pan, vision_vertices[0].tilt,
                  TRACE_EDGE_FREQ_HZ) != HAL_OK) {
    task_state = TASK_ERROR;
    AppDisplay_SetVisionState("故障");
    return;
  }
  AppDisplay_SetVisionState("运动");
}

static void HandleVisionFrame(void) {
  ProtoVisionFrame frame;
  if (!AppProtocol_GetVision(&frame)) return;
  if (work_mode != MODE_A4_VISION || task_state == TASK_HOMING) return;

  UpdateRedFeedback(&frame);

  if (!frame.valid) {
    if (task_paused) return;
    if (vision_invalid_count < 255U) vision_invalid_count++;
    if (vision_invalid_count >= VISION_INVALID_LIMIT &&
        (vision_armed || TaskIsRunning())) {
      Stepper_StopAll();
      task_state = TASK_WAIT_VISION;
      task_paused = 0U;
      vision_armed = 1U;
      AppDisplay_SetVisionState("停止");
    }
    return;
  }

  vision_invalid_count = 0U;
  if (task_paused) return;
  if (vision_armed && task_state == TASK_WAIT_VISION && frame.vertex_count == 4U) {
    StartVisionPath(&frame);
  }
}

static void VisionWatchdogTask(void) {
  if (work_mode != MODE_A4_VISION || task_paused || task_state == TASK_HOMING) return;
  if (!(vision_armed || TaskIsRunning())) return;

  if ((HAL_GetTick() - AppProtocol_GetLastVisionMs()) > VISION_TIMEOUT_MS) {
    Stepper_StopAll();
    task_state = TASK_WAIT_VISION;
    task_paused = 0U;
    vision_armed = 1U;
    latest_red_valid = 0U;
    AppDisplay_SetVisionState("故障");
  }
}

static void TaskService(void) {
  if (task_paused) return;

  switch (task_state) {
    case TASK_FIXED_TO_TL:
      if (!Stepper_IsBusy()) {
        task_state = TASK_FIXED_TOP;
        (void)MoveToPulse(SQUARE_HALF_PULSES, SQUARE_HALF_PULSES,
                          TRACE_EDGE_FREQ_HZ);
      }
      break;

    case TASK_FIXED_TOP:
      if (!Stepper_IsBusy()) {
        task_state = TASK_FIXED_RIGHT;
        (void)MoveToPulse(SQUARE_HALF_PULSES, -SQUARE_HALF_PULSES,
                          TRACE_EDGE_FREQ_HZ);
      }
      break;

    case TASK_FIXED_RIGHT:
      if (!Stepper_IsBusy()) {
        task_state = TASK_FIXED_BOTTOM;
        (void)MoveToPulse(-SQUARE_HALF_PULSES, -SQUARE_HALF_PULSES,
                          TRACE_EDGE_FREQ_HZ);
      }
      break;

    case TASK_FIXED_BOTTOM:
      if (!Stepper_IsBusy()) {
        task_state = TASK_FIXED_LEFT;
        (void)MoveToPulse(-SQUARE_HALF_PULSES, SQUARE_HALF_PULSES,
                          TRACE_EDGE_FREQ_HZ);
      }
      break;

    case TASK_FIXED_LEFT:
      if (!Stepper_IsBusy()) {
        task_state = TASK_RETURN_CENTER;
        (void)MoveToPulse(0, 0, HOME_FREQ_HZ);
      }
      break;

    case TASK_VISION_TO_FIRST:
      if (!Stepper_IsBusy()) StartVisionEdge(0U, 1U);
      break;

    case TASK_VISION_EDGE:
      if (!Stepper_IsBusy()) {
        if (vision_segment_done < vision_segment_count) {
          if (CommandVisionSegment() != HAL_OK) task_state = TASK_ERROR;
        } else if (vision_edge_to == 0U) {
          task_state = TASK_RETURN_CENTER;
          (void)MoveToPulse(0, 0, HOME_FREQ_HZ);
        } else {
          uint8_t next_from = vision_edge_to;
          uint8_t next_to = (uint8_t)((vision_edge_to + 1U) % 4U);
          StartVisionEdge(next_from, next_to);
        }
      }
      break;

    case TASK_RETURN_CENTER:
      if (!Stepper_IsBusy()) {
        task_state = TASK_DONE;
        vision_armed = 0U;
        AppDisplay_SetVisionState("完成");
      }
      break;

    case TASK_HOMING:
      if (!Stepper_IsBusy()) {
        task_state = TASK_IDLE;
        AppDisplay_SetVisionState("等待");
      }
      break;

    case TASK_ERROR:
      Stepper_StopAll();
      AppDisplay_SetVisionState("故障");
      break;

    case TASK_IDLE:
    case TASK_WAIT_VISION:
    case TASK_DONE:
    default:
      break;
  }
}

void App_Init(void) {
  Stepper_Init();
  Kin_Init();
  AppDisplay_Init();
  AppProtocol_Init();
  KeysInit();
  work_mode = MODE_FIXED_SQUARE;
  ClearTask();
  SetModeDisplay();
  AppDisplay_SetVisionState("等待");
}

void App_Loop(void) {
  Stepper_Service();
  AppProtocol_Task();
  KeysTask();

  ProtoAppCmd cmd;
  if (AppProtocol_GetAppCmd(&cmd)) {
    if (strcmp(cmd.name, "home") == 0) StartHome();
    else if (strcmp(cmd.name, "vision") == 0) {
      SelectMode(MODE_A4_VISION);
      StartVisionWait();
    }
  }

  HandleVisionFrame();
  VisionWatchdogTask();
  TaskService();
  AppDisplay_Task();
}
