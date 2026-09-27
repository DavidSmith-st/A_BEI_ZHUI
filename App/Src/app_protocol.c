#include "app_protocol.h"
#include "stepper.h"
#include "usart.h"
#include "main.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * 视觉-云台串口协议（USART1, 115200-8-N-1, NDJSON 一行一帧）
 *   支持 VISION_RESULT / TRACK_RESULT / 调试指令 cmd
 *   同 seq 重复帧自动去重；valid=0 或超时由控制层停止并保持。
 */
#define RX_LINE_MAX  256U
#define RX_RING_SIZE 512U
#define RX_RING_MASK (RX_RING_SIZE - 1U)

static uint8_t rx_byte;
static volatile uint8_t rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head;   /* 中断写入 */
static volatile uint16_t rx_tail;   /* 主循环读取 */
static char rx_line[RX_LINE_MAX];
static uint16_t rx_index;

static uint32_t last_rx_ms;
static uint32_t last_vision_ms;
static uint32_t frame_count;

static ProtoVisionFrame vision_frame;
static uint8_t vision_new;
static ProtoTrackFrame track_frame;
static uint8_t track_new;
static ProtoAppCmd app_cmd;
static uint8_t app_cmd_new;

/* ------------------------- JSON 工具 ------------------------- */

static const char *skip_ws(const char *p) {
  while (*p != '\0' && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')) ++p;
  return p;
}

static const char *find_key(const char *json, const char *key) {
  char pat[40];
  (void)snprintf(pat, sizeof(pat), "\"%s\"", key);
  const char *p = strstr(json, pat);
  if (p == NULL) return NULL;
  p += strlen(pat);
  p = strchr(p, ':');
  if (p == NULL) return NULL;
  return skip_ws(p + 1);
}

static const char *parse_number(const char *p, double *out) {
  char *end = NULL;
  double v = strtod(p, &end);
  if (end == p) return NULL;
  *out = v;
  return end;
}

static int16_t mm_to_01mm(double mm) {
  double v = mm * 10.0;
  if (v >= 0.0) return (int16_t)(v + 0.5);
  return (int16_t)(v - 0.5);
}

static uint8_t json_get_u32(const char *json, const char *key, uint32_t *out) {
  const char *p = find_key(json, key);
  if (p == NULL) return 0U;
  double v = 0.0;
  const char *q = parse_number(p, &v);
  if (q == NULL || v < 0.0 || v > 4294967295.0) return 0U;
  *out = (uint32_t)v;
  return 1U;
}

static uint8_t json_get_i32(const char *json, const char *key, int32_t *out) {
  const char *p = find_key(json, key);
  if (p == NULL) return 0U;
  double v = 0.0;
  const char *q = parse_number(p, &v);
  if (q == NULL || v < -2147483648.0 || v > 2147483647.0) return 0U;
  *out = (int32_t)v;
  return 1U;
}

static uint8_t json_get_u8(const char *json, const char *key, uint8_t *out) {
  uint32_t v;
  if (!json_get_u32(json, key, &v)) return 0U;
  *out = (uint8_t)(v & 0xFFU);
  return 1U;
}

static uint8_t json_get_string(const char *json, const char *key, char *out, size_t out_size) {
  const char *p = find_key(json, key);
  if (p == NULL || *p != '"' || out_size == 0U) return 0U;
  ++p;
  size_t n = 0U;
  while (*p != '\0' && *p != '"' && (n + 1U) < out_size) out[n++] = *p++;
  if (*p != '"') return 0U;
  out[n] = '\0';
  return 1U;
}

/* 解析 {"x_mm":..,"y_mm":..} 对象 */
static uint8_t parse_point(const char *json, const char *key, ProtoPoint *out) {
  const char *p = find_key(json, key);
  if (p == NULL || *p != '{') return 0U;
  const char *end = p + 1;
  int depth = 1;
  while (*end != '\0' && depth > 0) {
    if (*end == '{') ++depth;
    else if (*end == '}') --depth;
    if (depth > 0) ++end;
  }
  if (depth != 0) return 0U;
  char obj[96];
  size_t len = (size_t)(end - p + 1U);
  if (len >= sizeof(obj)) len = sizeof(obj) - 1U;
  memcpy(obj, p, len);
  obj[len] = '\0';
  const char *vx = find_key(obj, "x_mm");
  const char *vy = find_key(obj, "y_mm");
  double x = 0.0, y = 0.0;
  if (vx == NULL || vy == NULL) return 0U;
  if (parse_number(vx, &x) == NULL) return 0U;
  if (parse_number(vy, &y) == NULL) return 0U;
  out->x_01mm = mm_to_01mm(x);
  out->y_01mm = mm_to_01mm(y);
  return 1U;
}

/* 解析 vertices_mm: [[x,y],[x,y],...] */
static uint8_t parse_vertices(const char *json, ProtoPoint *out, uint8_t max_n) {
  const char *p = find_key(json, "vertices_mm");
  if (p == NULL) return 0U;
  p = skip_ws(p);
  if (*p != '[') return 0U;
  p = skip_ws(p + 1U);
  uint8_t n = 0U;
  while (*p != '\0' && *p != ']' && n < max_n) {
    p = skip_ws(p);
    if (*p != '[') break;
    p = skip_ws(p + 1U);
    double x = 0.0, y = 0.0;
    const char *q = parse_number(p, &x);
    if (q == NULL) break;
    q = skip_ws(q);
    if (*q != ',') break;
    q = skip_ws(q + 1U);
    const char *r = parse_number(q, &y);
    if (r == NULL) break;
    r = skip_ws(r);
    if (*r != ']') break;
    out[n].x_01mm = mm_to_01mm(x);
    out[n].y_01mm = mm_to_01mm(y);
    ++n;
    p = skip_ws(r + 1U);
    if (*p == ',') p = skip_ws(p + 1U);
  }
  return n;
}

/* ------------------------- 帧解析 ------------------------- */

static void handle_vision(const char *json) {
  last_vision_ms = HAL_GetTick();
  ProtoVisionFrame f;
  memset(&f, 0, sizeof(f));
  (void)json_get_u32(json, "seq", &f.seq);
  (void)json_get_u8(json, "valid", &f.valid);
  uint8_t has_red_valid = json_get_u8(json, "red_valid", &f.red_valid);
  (void)json_get_u8(json, "vertex_count", &f.vertex_count);
  (void)parse_point(json, "red_spot", &f.red_spot);
  if (!has_red_valid) {
    f.red_valid = (f.red_spot.x_01mm != 0 || f.red_spot.y_01mm != 0) ? 1U : 0U;
  }
  uint8_t n = parse_vertices(json, f.vertices, PROTO_MAX_VERTICES);
  if (f.vertex_count > PROTO_MAX_VERTICES) f.vertex_count = PROTO_MAX_VERTICES;
  if (n < f.vertex_count) f.vertex_count = n;

  /* 同 seq 有效帧去重 */
  if (f.valid != 0U && f.seq != 0U && f.seq == vision_frame.seq) return;

  vision_frame = f;
  vision_new = 1U;
  ++frame_count;
}

static void handle_track(const char *json) {
  ProtoTrackFrame f;
  memset(&f, 0, sizeof(f));
  (void)json_get_u32(json, "seq", &f.seq);
  (void)json_get_u8(json, "valid", &f.valid);
  (void)parse_point(json, "red_spot", &f.red_spot);
  (void)parse_point(json, "green_spot", &f.green_spot);

  if (f.valid != 0U && f.seq != 0U && f.seq == track_frame.seq) return;

  track_frame = f;
  track_new = 1U;
  ++frame_count;
}

/* ------------------------- 调试指令（保留第一阶段测试指令） ------------------------- */

static void uart_write(const char *text) {
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 50U);
}

static void send_ack(uint32_t seq, const char *cmd, uint8_t ok, const char *err) {
  char tx[160];
  if (ok) {
    (void)snprintf(tx, sizeof(tx),
                   "{\"type\":\"ack\",\"seq\":%lu,\"ok\":true,\"cmd\":\"%s\"}\r\n",
                   (unsigned long)seq, cmd);
  } else {
    (void)snprintf(tx, sizeof(tx),
                   "{\"type\":\"ack\",\"seq\":%lu,\"ok\":false,\"cmd\":\"%s\",\"err\":\"%s\"}\r\n",
                   (unsigned long)seq, cmd, err);
  }
  uart_write(tx);
}

static void handle_cmd(const char *json) {
  char cmd[16] = {0};
  uint32_t seq = 0;
  (void)json_get_u32(json, "seq", &seq);
  if (!json_get_string(json, "cmd", cmd, sizeof(cmd))) {
    send_ack(seq, "unknown", 0U, "bad_json");
    return;
  }

  if (strcmp(cmd, "move") == 0) {
    char axis_name[12];
    int32_t steps;
    uint32_t freq;
    if (!json_get_string(json, "axis", axis_name, sizeof(axis_name))) {
      send_ack(seq, cmd, 0U, "bad_axis");
      return;
    }
    if (!json_get_u32(json, "freq_hz", &freq)) freq = 500U;
    if (!json_get_i32(json, "steps", &steps) || steps == 0) {
      send_ack(seq, cmd, 0U, "bad_value");
      return;
    }
    uint8_t axis;
    if (strcmp(axis_name, "pan") == 0) axis = 0U;
    else if (strcmp(axis_name, "tilt") == 0) axis = 1U;
    else {
      send_ack(seq, cmd, 0U, "bad_axis");
      return;
    }
    HAL_StatusTypeDef result = Stepper_MoveRelative(axis, steps, freq);
    if (result == HAL_BUSY) send_ack(seq, cmd, 0U, "busy");
    else if (result != HAL_OK) send_ack(seq, cmd, 0U, "bad_value");
    else send_ack(seq, cmd, 1U, NULL);
  } else if (strcmp(cmd, "stop") == 0) {
    Stepper_StopAll();
    send_ack(seq, cmd, 1U, NULL);
  } else if (strcmp(cmd, "pause") == 0) {
    Stepper_Pause();
    send_ack(seq, cmd, 1U, NULL);
  } else if (strcmp(cmd, "resume") == 0) {
    Stepper_Resume();
    send_ack(seq, cmd, 1U, NULL);
  } else if (strcmp(cmd, "zero") == 0) {
    Stepper_ZeroSoftwarePosition();
    send_ack(seq, cmd, 1U, NULL);
  } else if (strcmp(cmd, "vision") == 0 || strcmp(cmd, "home") == 0) {
    /* 控制主程序处理的指令 */
    app_cmd_new = 1U;
    (void)snprintf(app_cmd.name, sizeof(app_cmd.name), "%s", cmd);
    app_cmd.i0 = 0;
    send_ack(seq, cmd, 1U, NULL);
  } else {
    send_ack(seq, cmd, 0U, "bad_cmd");
  }
}

static void process_line(const char *json) {
  char type[24] = {0};
  if (json_get_string(json, "type", type, sizeof(type))) {
    if (strcmp(type, "VISION_RESULT") == 0) handle_vision(json);
    else if (strcmp(type, "TRACK_RESULT") == 0) handle_track(json);
    else send_ack(0, "unknown", 0U, "bad_type");
    return;
  }
  handle_cmd(json);
}

/* ------------------------- 对外接口 ------------------------- */

void AppProtocol_Init(void) {
  rx_head = 0U;
  rx_tail = 0U;
  rx_index = 0U;
  last_rx_ms = HAL_GetTick();
  last_vision_ms = HAL_GetTick();
  frame_count = 0U;
  vision_new = 0U;
  track_new = 0U;
  app_cmd_new = 0U;
  memset(&vision_frame, 0, sizeof(vision_frame));
  memset(&track_frame, 0, sizeof(track_frame));

  /* USART1 全局中断（若 CubeMX 未勾选，这里补开） */
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
  uart_write("{\"type\":\"hello\",\"device\":\"pd42s1_gimbal\",\"protocol\":\"0.2\"}\r\n");
}

void AppProtocol_Task(void) {
  while (rx_tail != rx_head) {
    uint8_t ch = rx_ring[rx_tail];
    rx_tail = (rx_tail + 1U) & RX_RING_MASK;

    if (ch == '\n') {
      if (rx_index > 0U) {
        if (rx_line[rx_index - 1U] == '\r') --rx_index;
        rx_line[rx_index] = '\0';
        rx_index = 0U;
        process_line(rx_line);
      }
    } else if (rx_index < (RX_LINE_MAX - 1U)) {
      rx_line[rx_index++] = (char)ch;
    } else {
      rx_index = 0U; /* 单行超长，丢弃并等下一个换行重新同步 */
    }
  }
}

uint8_t AppProtocol_GetVision(ProtoVisionFrame *out) {
  if (!vision_new) return 0U;
  vision_new = 0U;
  if (out != NULL) *out = vision_frame;
  return 1U;
}

uint8_t AppProtocol_GetTrack(ProtoTrackFrame *out) {
  if (!track_new) return 0U;
  track_new = 0U;
  if (out != NULL) *out = track_frame;
  return 1U;
}

uint8_t AppProtocol_GetAppCmd(ProtoAppCmd *out) {
  if (!app_cmd_new) return 0U;
  app_cmd_new = 0U;
  if (out != NULL) *out = app_cmd;
  return 1U;
}

uint32_t AppProtocol_GetRxAgeMs(void) { return HAL_GetTick() - last_rx_ms; }
uint32_t AppProtocol_GetLastVisionMs(void) { return last_vision_ms; }
uint32_t AppProtocol_GetFrameCount(void) { return frame_count; }

/* ------------------------- 中断回调 ------------------------- */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance != USART1) return;
  last_rx_ms = HAL_GetTick();
  uint16_t next = (rx_head + 1U) & RX_RING_MASK;
  if (next != rx_tail) {
    rx_ring[rx_head] = rx_byte;
    rx_head = next;
  }
  (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    rx_head = 0U;
    rx_tail = 0U;
    rx_index = 0U;
    (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
  }
}
