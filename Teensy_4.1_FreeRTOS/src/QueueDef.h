#ifndef QUEUEDEF_H
#define QUEUEDEF_H

#include <Arduino.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

// ==================== 运动指令结构体 ====================
typedef struct {
    int axis;       // 0~5
    long steps;     // 正=正转，负=反转
} MotionCmd_t;

// ==================== 队列句柄声明 ====================
extern QueueHandle_t xMotionQueue;
extern QueueHandle_t xUartQueue;
extern SemaphoreHandle_t xEStopSemaphore;

#endif