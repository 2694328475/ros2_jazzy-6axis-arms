#ifndef COMMON_H
#define COMMON_H

#include <Arduino.h>

// ==================== 引脚定义 ====================
extern const int stepPin[6];
extern const int dirPin[6];
extern const int limitPin[6];
extern const int estopPin;

// ==================== 每转步数 ====================
extern const int stepsPerRev[6];

// ==================== 运动参数 ====================
extern float maxSpeed;
extern float acceleration;

// ==================== 全局状态变量 ====================
extern volatile long currentPos[6];
extern volatile bool moving[6];
extern volatile int activeAxis;
extern volatile bool estopTriggered;

// ==================== 函数声明 ====================
void setupPins();
void startMoveInternal(int axis, long steps);
void updateMotion();
void emergencyStopInternal();
bool checkLimit(int axis);
void reportPosition();
void setMaxSpeed(float sp);
void setAcceleration(float acc);

// ==================== 急停ISR ====================
void emergencyStopISR();

#endif