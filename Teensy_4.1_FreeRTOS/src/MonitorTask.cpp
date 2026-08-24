#include "MonitorTask.h"
#include "Common.h"
#include "QueueDef.h"

void MonitorTask(void *pvParameters) {
    (void)pvParameters;
    
    Serial.println("[MonitorTask] Started");
    unsigned long lastPrint = 0;
    
    while (1) {
        // 1. 检查急停信号量（由 ISR 释放）
        if (xSemaphoreTake(xEStopSemaphore, 0) == pdPASS) {
            emergencyStopInternal();
            Serial.println("ESTOP triggered by ISR");
        }
        
        // 2. 检查限位
        for (int i = 0; i < 6; i++) {
            if (checkLimit(i)) {
                if (moving[i]) {
                    moving[i] = false;
                    activeAxis = -1;
                    Serial.print("[Monitor] Limit triggered on axis ");
                    Serial.println(i + 1);
                }
            }
        }
        
        // 3. 每秒打印一次状态（默认关闭，取消注释可启用）
        // if (millis() - lastPrint > 1000) {
        //     reportPosition();
        //     lastPrint = millis();
        // }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}