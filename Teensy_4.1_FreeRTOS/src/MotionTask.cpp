#include "MotionTask.h"
#include "Common.h"
#include "QueueDef.h"

void MotionTask(void *pvParameters) {
    (void)pvParameters;  // 消除未使用参数警告
    
    MotionCmd_t cmd;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    Serial.println("[MotionTask] Started");
    
    while (1) {
        if (xQueueReceive(xMotionQueue, &cmd, 0) == pdPASS) {
            startMoveInternal(cmd.axis, cmd.steps);
            Serial.print("[Motion] Axis ");
            Serial.print(cmd.axis + 1);
            Serial.print(" steps ");
            Serial.println(cmd.steps);
        }
        
        if (activeAxis != -1 && moving[activeAxis]) {
            updateMotion();
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}