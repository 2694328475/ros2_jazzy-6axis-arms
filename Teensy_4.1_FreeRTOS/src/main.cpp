#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "Common.h"
#include "QueueDef.h"
#include "MotionTask.h"
#include "CmdTask.h"
#include "MonitorTask.h"

// ==================== 定义队列句柄 ====================
QueueHandle_t xMotionQueue;
QueueHandle_t xUartQueue;
SemaphoreHandle_t xEStopSemaphore;

// ==================== 串口中断处理 ====================
static void processUartISR() {
    char c;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    while (Serial.available()) {
        c = Serial.read();
        xQueueSendFromISR(xUartQueue, &c, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ==================== setup ====================
void setup() {
    Serial.begin(115200);
    delay(100);
    
    setupPins();
    attachInterrupt(digitalPinToInterrupt(estopPin), emergencyStopISR, LOW);
    
    Serial.println("========================================");
    Serial.println("  FreeRTOS ARM Controller - Teensy 4.1  ");
    Serial.println("========================================");
    
    // 创建队列和信号量
    xMotionQueue = xQueueCreate(10, sizeof(MotionCmd_t));
    xUartQueue = xQueueCreate(64, sizeof(char));
    xEStopSemaphore = xSemaphoreCreateBinary();
    
    if (xMotionQueue == NULL || xUartQueue == NULL || xEStopSemaphore == NULL) {
        Serial.println("ERR: Failed to create queues/semaphore!");
        while (1);
    }
    
    // 创建任务（优先级：数字越大优先级越高）
    xTaskCreate(MotionTask, "Motion", 2048, NULL, 3, NULL);
    xTaskCreate(CmdTask, "Command", 2048, NULL, 2, NULL);
    xTaskCreate(MonitorTask, "Monitor", 1024, NULL, 1, NULL);
    
    Serial.println("Tasks created, starting scheduler...");
    
    vTaskStartScheduler();
    
    while (1);
}

// ==================== loop（必须存在但为空） ====================
void loop() {
    // FreeRTOS 调度器接管了一切
}

// ==================== 串口中断服务 ====================
void serialEvent() {
    processUartISR();
}