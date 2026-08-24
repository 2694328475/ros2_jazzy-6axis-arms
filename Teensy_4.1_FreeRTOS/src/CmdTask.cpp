#include "CmdTask.h"
#include "Common.h"
#include "QueueDef.h"

extern QueueHandle_t xMotionQueue;
extern QueueHandle_t xUartQueue;

static void parseAndExecute(char* cmd);

void CmdTask(void *pvParameters) {
    (void)pvParameters;
    
    char cmdBuffer[64];
    unsigned int index = 0;
    
    Serial.println("[CmdTask] Started");
    
    while (1) {
        char c;
        if (xQueueReceive(xUartQueue, &c, portMAX_DELAY) == pdPASS) {
            if (c == '\n' || c == '\r') {
                if (index > 0) {
                    cmdBuffer[index] = '\0';
                    parseAndExecute(cmdBuffer);
                    index = 0;
                }
            } else if (index < 63) {
                cmdBuffer[index++] = c;
            }
        }
    }
}

static void parseAndExecute(char* cmd) {
    String cmdStr = String(cmd);
    cmdStr.trim();
    cmdStr.toUpperCase();
    
    if (cmdStr.length() == 0) return;
    
    // ---------- 急停 ----------
    if (cmdStr == "ESTOP") {
        emergencyStopInternal();
        Serial.println("ESTOP_ACK");
        return;
    }
    
    if (cmdStr == "ESRESET") {
        estopTriggered = false;
        Serial.println("ESTOP_RESET");
        return;
    }
    
    // ---------- 查询位置 ----------
    if (cmdStr == "POS") {
        reportPosition();
        return;
    }
    
    // ---------- 单轴运动：J1 1000 ----------
    if (cmdStr[0] == 'J' && isDigit(cmdStr[1])) {
        int spaceIdx = cmdStr.indexOf(' ');
        if (spaceIdx == -1) {
            Serial.println("ERR: Missing steps");
            return;
        }
        int axis = cmdStr.substring(1, spaceIdx).toInt() - 1;
        long steps = cmdStr.substring(spaceIdx + 1).toInt();
        if (axis >= 0 && axis < 6) {
            MotionCmd_t motionCmd = {axis, steps};
            xQueueSend(xMotionQueue, &motionCmd, 0);
            Serial.println("OK");
        } else {
            Serial.println("ERR: Invalid axis");
        }
        return;
    }
    
    // ---------- 多轴命令：RJ A10 B20 C30 ... ----------
    if (cmdStr.startsWith("RJ")) {
        float targetAngles[6] = {NAN, NAN, NAN, NAN, NAN, NAN};
        unsigned int idx = 2;
        while (idx < cmdStr.length()) {
            char axisChar = cmdStr[idx];
            idx++;
            String numStr = "";
            while (idx < cmdStr.length() && (isDigit(cmdStr[idx]) || cmdStr[idx] == '.' || cmdStr[idx] == '-')) {
                numStr += cmdStr[idx];
                idx++;
            }
            if (numStr.length() > 0) {
                float val = numStr.toFloat();
                if (axisChar >= 'A' && axisChar <= 'F') {
                    targetAngles[axisChar - 'A'] = val;
                }
            }
        }
        
        int sentCount = 0;
        for (int i = 0; i < 6; i++) {
            if (!isnan(targetAngles[i])) {
                float stepsPerDeg = stepsPerRev[i] / 360.0;
                float currentAngleDeg = currentPos[i] / stepsPerDeg;
                float deltaAngle = targetAngles[i] - currentAngleDeg;
                long deltaSteps = round(deltaAngle * stepsPerDeg);
                
                if (deltaSteps != 0) {
                    MotionCmd_t motionCmd = {i, deltaSteps};
                    xQueueSend(xMotionQueue, &motionCmd, 0);
                    sentCount++;
                }
            }
        }
        Serial.print("OK RJ (");
        Serial.print(sentCount);
        Serial.println(" axes)");
        return;
    }
    
    Serial.println("ERR: Unknown command");
}