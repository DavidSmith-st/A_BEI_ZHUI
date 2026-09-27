#ifndef __APP_PROTOCOL_H
#define __APP_PROTOCOL_H

#include <stdint.h>

#define PROTO_MAX_VERTICES 4U

/* 0.1mm 定点坐标（屏幕中心为原点，+x 右，+y 上） */
typedef struct {
  int16_t x_01mm;
  int16_t y_01mm;
} ProtoPoint;

/* VISION_RESULT：红色运动目标 + 方形顶点 */
typedef struct {
  uint32_t seq;
  uint8_t  valid;
  uint8_t  red_valid;
  uint8_t  vertex_count;
  ProtoPoint red_spot;
  ProtoPoint vertices[PROTO_MAX_VERTICES];
} ProtoVisionFrame;

/* TRACK_RESULT：红色/绿色光斑（绿光追踪备用） */
typedef struct {
  uint32_t seq;
  uint8_t  valid;
  ProtoPoint red_spot;
  ProtoPoint green_spot;
} ProtoTrackFrame;

/* 串口调试指令（square/vision/home 等，由控制主程序处理） */
typedef struct {
  char     name[16];
  int32_t  i0;
} ProtoAppCmd;

void AppProtocol_Init(void);
void AppProtocol_Task(void);

/* 取走最新一帧，返回 1 表示有新帧（取走后内部清除） */
uint8_t AppProtocol_GetVision(ProtoVisionFrame *out);
uint8_t AppProtocol_GetTrack(ProtoTrackFrame *out);
uint8_t AppProtocol_GetAppCmd(ProtoAppCmd *out);

uint32_t AppProtocol_GetRxAgeMs(void);
uint32_t AppProtocol_GetLastVisionMs(void);
uint32_t AppProtocol_GetFrameCount(void);

#endif
