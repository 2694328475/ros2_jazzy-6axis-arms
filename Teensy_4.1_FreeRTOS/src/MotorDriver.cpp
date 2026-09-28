#include "Common.h"
#include "QueueDef.h"

// ==================== 引脚定义 ====================
const int stepPin[6]  = {0,  2,  4,  6,  8,  10};
const int dirPin[6]   = {1,  3,  5,  7,  9,  11};
const int limitPin[6] = {26, 27, 28, 29, 30, 31};
const int estopPin    = 39;

// ==================== 每转步数 ====================
const int stepsPerRev[6] = {400, 400, 400, 800, 400, 400};

// ==================== 运动参数 ====================
float maxSpeed     = 4000.0;
float acceleration = 1000.0;

// ==================== 全局状态 ====================
volatile long  currentPos[6]    = {0};
volatile bool  moving[6]        = {false};
volatile int   activeAxis       = -1;
volatile bool  estopTriggered   = false;

// ==================== 运动规划结构体 ====================
struct MotionProfile {
    long totalSteps;
    long stepsDone;
    long accelSteps;
    long decelSteps;
    long constSteps;
    int direction;
    float currentSpeed;
    unsigned long lastPulseMicros;
    unsigned long nextPulseDelayUs;
};

static MotionProfile motion[6];

// ==================== 实现函数 ====================

void setupPins() {
    for (int i = 0; i < 6; i++) {
        pinMode(stepPin[i], OUTPUT);
        pinMode(dirPin[i], OUTPUT);
        digitalWrite(stepPin[i], HIGH);
        pinMode(limitPin[i], INPUT_PULLUP);
        
        motion[i].totalSteps = 0;
        motion[i].stepsDone = 0;
        motion[i].currentSpeed = 0;
    }
    pinMode(estopPin, INPUT_PULLUP);
}

void startMoveInternal(int axis, long steps) {
    if (steps == 0) return;
    if (estopTriggered) {
        Serial.println("ERR: Emergency stop active");
        return;
    }
    if (steps < 0 && checkLimit(axis)) {
        Serial.print("ERR: Axis ");
        Serial.print(axis + 1);
        Serial.println(" at negative limit");
        return;
    }

    moving[axis] = false;

    activeAxis = axis;
    moving[axis] = true;
    motion[axis].direction = (steps > 0) ? 1 : -1;
    motion[axis].totalSteps = abs(steps);
    motion[axis].stepsDone = 0;

    digitalWrite(dirPin[axis], (motion[axis].direction == 1) ? HIGH : LOW);
    delayMicroseconds(10);

    float v_max = maxSpeed;
    float a = acceleration;
    float accelDist = (v_max * v_max) / (2.0 * a);
    motion[axis].accelSteps = (long)ceil(accelDist);
    motion[axis].decelSteps = motion[axis].accelSteps;

    if (motion[axis].totalSteps <= (motion[axis].accelSteps + motion[axis].decelSteps)) {
        float v_peak = sqrt(a * motion[axis].totalSteps);
        motion[axis].accelSteps = (long)ceil((v_peak * v_peak) / (2.0 * a));
        motion[axis].decelSteps = motion[axis].accelSteps;
        motion[axis].constSteps = 0;
    } else {
        motion[axis].constSteps = motion[axis].totalSteps - motion[axis].accelSteps - motion[axis].decelSteps;
    }

    motion[axis].currentSpeed = 200.0;
    motion[axis].lastPulseMicros = micros();
    motion[axis].nextPulseDelayUs = 1000000.0 / motion[axis].currentSpeed;
}

void updateMotion() {
    if (activeAxis == -1) return;
    int axis = activeAxis;
    if (!moving[axis]) return;
    if (estopTriggered) {
        moving[axis] = false;
        activeAxis = -1;
        return;
    }

    unsigned long now = micros();
    if (now - motion[axis].lastPulseMicros >= motion[axis].nextPulseDelayUs) {
        digitalWrite(stepPin[axis], LOW);
        delayMicroseconds(2);
        digitalWrite(stepPin[axis], HIGH);

        currentPos[axis] += motion[axis].direction;
        motion[axis].stepsDone++;

        if (checkLimit(axis)) {
            Serial.print("Limit hit on axis ");
            Serial.println(axis + 1);
            moving[axis] = false;
            activeAxis = -1;
            reportPosition();
            return;
        }

        if (motion[axis].stepsDone >= motion[axis].totalSteps) {
            moving[axis] = false;
            activeAxis = -1;
            return;
        }

        long done = motion[axis].stepsDone;
        if (done <= motion[axis].accelSteps) {
            motion[axis].currentSpeed = (maxSpeed / motion[axis].accelSteps) * done;
            if (motion[axis].currentSpeed < 10.0) motion[axis].currentSpeed = 10.0;
        } else if (done <= motion[axis].accelSteps + motion[axis].constSteps) {
            motion[axis].currentSpeed = maxSpeed;
        } else {
            long decelDone = done - (motion[axis].accelSteps + motion[axis].constSteps);
            long decelTotal = motion[axis].decelSteps;
            if (decelTotal > 0) {
                motion[axis].currentSpeed = maxSpeed * (1.0 - (float)decelDone / decelTotal);
            }
            if (motion[axis].currentSpeed < 10.0) motion[axis].currentSpeed = 10.0;
        }

        if (motion[axis].currentSpeed > 0) {
            motion[axis].nextPulseDelayUs = (unsigned long)(1000000.0 / motion[axis].currentSpeed);
            if (motion[axis].nextPulseDelayUs < 20) motion[axis].nextPulseDelayUs = 20;
        } else {
            motion[axis].nextPulseDelayUs = 1000;
        }
        motion[axis].lastPulseMicros = now;
    }
}

bool checkLimit(int axis) {
    return (digitalRead(limitPin[axis]) == LOW);
}

void emergencyStopInternal() {
    estopTriggered = true;
    for (int i = 0; i < 6; i++) {
        moving[i] = false;
    }
    activeAxis = -1;
    Serial.println("Emergency stop");
}

void reportPosition() {
    String msg = "POS:";
    for (int i = 0; i < 6; i++) {
        msg += " J" + String(i+1) + "=" + String(currentPos[i]);
    }
    Serial.println(msg);
}

void setMaxSpeed(float sp) {
    if (sp < 10) sp = 10;
    maxSpeed = sp;
}

void setAcceleration(float acc) {
    if (acc < 10) acc = 10;
    acceleration = acc;
}

// ==================== 急停中断服务 ====================
void emergencyStopISR() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xEStopSemaphore != NULL) {
        xSemaphoreGiveFromISR(xEStopSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}