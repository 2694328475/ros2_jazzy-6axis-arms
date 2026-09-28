#include <Arduino.h>

// 引脚定义
const int stepPin[6]  = {0,  2,  4,  6,  8,  10};
const int dirPin[6]   = {1,  3,  5,  7,  9,  11};
const int limitPin[6] = {26, 27, 28, 29, 30, 31};
const int estopPin    = 39;

// 每转步数（根据驱动器细分设置）
// 示例：无细分400，有细分800。请根据实际修改！
const int stepsPerRev[6] = {400, 400, 400, 800, 400, 400};  // 第4轴800

// 运动参数
float maxSpeed     = 4000.0;   // 步/秒
float acceleration = 1000.0;   // 步/秒²

// 全局状态
volatile long  currentPos[6]    = {0};
volatile bool  moving[6]        = {false};
volatile int   activeAxis       = -1;
volatile bool  estopTriggered   = false;

// 限位去抖动
const int limitDebounceCount = 2;
int limitDebounce[6] = {0};

// 运动规划变量
volatile long  moveTotalSteps    = 0;
volatile long  moveStepsDone     = 0;
volatile long  moveAccelSteps    = 0;
volatile long  moveDecelSteps    = 0;
volatile long  moveConstSteps    = 0;
volatile int   moveDirection     = 0;
volatile float currentSpeed      = 0.0;
volatile unsigned long lastPulseMicros    = 0;
volatile unsigned long nextPulseDelayUs  = 0;

// 非阻塞多轴任务队列
const int MAX_TASKS = 6;
int   taskAxis[MAX_TASKS];
long  taskSteps[MAX_TASKS];
int   taskCount = 0;
int   taskIndex = 0;
bool  tasksActive = false;

// 命令缓冲区
String inputString = "";
const int MAX_INPUT_LENGTH = 64;

// 函数声明
void setupPins();
void handleCommand(String cmd);
void startMove(int axis, long steps);
void updateMotion();
void emergencyStop();
void reportPosition();
bool checkLimit(int axis);
void homeAxis(int axis);
void stopAxis(int axis);
void setMaxSpeed(float sp);
void setAcceleration(float acc);
void emergencyStopISR();
void clearTaskQueue();
void processTasks();

void setup() {
  Serial.begin(115200);
  delay(100);
  setupPins();

  pinMode(estopPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(estopPin), emergencyStopISR, LOW);

  setMaxSpeed(4000.0);
  setAcceleration(1000.0);

  Serial.println("6-Axis Robot Ready (Final)");
}

void setupPins() {
  for (int i = 0; i < 6; i++) {
    pinMode(stepPin[i], OUTPUT);
    pinMode(dirPin[i], OUTPUT);
    digitalWrite(stepPin[i], HIGH);
    pinMode(limitPin[i], INPUT_PULLUP);
  }
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (inputString.length() > 0) {
        handleCommand(inputString);
        inputString = "";
      }
    } else {
      if (inputString.length() < MAX_INPUT_LENGTH) {
        inputString += c;
      }
    }
  }

  if (activeAxis != -1 && moving[activeAxis] && !estopTriggered) {
    updateMotion();
  }

  if (!estopTriggered && activeAxis == -1 && tasksActive) {
    processTasks();
  }
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() == 0) return;

  if (cmd == "ESTOP") {
    emergencyStop();
    Serial.println("ESTOP_ACK");
    return;
  }
  if (cmd == "ESRESET") {
    estopTriggered = false;
    Serial.println("ESTOP_RESET");
    return;
  }
  if (cmd == "POS") {
    reportPosition();
    return;
  }
  if (cmd == "RP") {
    String resp = "RP";
    for (int i = 0; i < 6; i++) {
      char tag = 'A' + i;
      float stepsPerDeg = stepsPerRev[i] / 360.0;
      float angle = currentPos[i] / stepsPerDeg;
      resp += tag;
      resp += String(angle, 2);
    }
    Serial.println(resp);
    return;
  }
  if (cmd.startsWith("SPEED")) {
    float sp = cmd.substring(5).toFloat();
    setMaxSpeed(sp);
    Serial.print("Max speed set to ");
    Serial.println(sp);
    return;
  }
  if (cmd.startsWith("ACC")) {
    float ac = cmd.substring(3).toFloat();
    setAcceleration(ac);
    Serial.print("Acceleration set to ");
    Serial.println(ac);
    return;
  }
  if (cmd.startsWith("SP")) {
    float pct = cmd.substring(2).toFloat();
    if (pct < 1.0) pct = 1.0;
    if (pct > 100.0) pct = 100.0;
    setMaxSpeed(4000.0 * pct / 100.0);
    Serial.print("Speed set to ");
    Serial.print(pct);
    Serial.println("%");
    return;
  }
  if (cmd.startsWith("PIN")) {
    int ax = cmd.substring(3).toInt() - 1;
    if (ax >= 0 && ax < 6) {
      Serial.print("Limit J");
      Serial.print(ax + 1);
      Serial.print(" = ");
      Serial.println(digitalRead(limitPin[ax]));
    } else {
      Serial.println("ERR: Invalid axis");
    }
    return;
  }
  if (cmd.startsWith("HOME")) {
    int axis = cmd.substring(4).toInt() - 1;
    if (axis >= 0 && axis < 6) {
      homeAxis(axis);
    } else {
      Serial.println("ERR: Invalid axis");
    }
    return;
  }
  if (cmd[0] == 'J' && isDigit(cmd[1])) {
    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx == -1) {
      Serial.println("ERR: Missing steps");
      return;
    }
    int axis = cmd.substring(1, spaceIdx).toInt() - 1;
    long steps = cmd.substring(spaceIdx + 1).toInt();
    if (axis >= 0 && axis < 6) {
      clearTaskQueue();
      if (activeAxis != -1 && moving[activeAxis]) {
        stopAxis(activeAxis);
      }
      startMove(axis, steps);
    } else {
      Serial.println("ERR: Invalid axis");
    }
    return;
  }

  // RJ 命令处理（修正版：强制最小步数，且正确加入队列）
  if (cmd.startsWith("RJ")) {
    pinMode(LED_BUILTIN, OUTPUT);
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
    }
    Serial.println("RJ received");
    float targetAngles[6] = {NAN, NAN, NAN, NAN, NAN, NAN};
    int idx = 2;
    while (idx < cmd.length()) {
      char axisChar = cmd[idx];
      idx++;
      String numStr = "";
      while (idx < cmd.length() && (isDigit(cmd[idx]) || cmd[idx] == '.' || cmd[idx] == '-')) {
        numStr += cmd[idx];
        idx++;
      }
      if (numStr.length() > 0) {
        float val = numStr.toFloat();
        if (axisChar >= 'A' && axisChar <= 'F') {
          targetAngles[axisChar - 'A'] = val;
        }
      }
    }

    if (activeAxis != -1 && moving[activeAxis]) {
      stopAxis(activeAxis);
    }
    clearTaskQueue();

    for (int i = 0; i < 6; i++) {
      if (!isnan(targetAngles[i])) {
        float stepsPerDeg = stepsPerRev[i] / 360.0;
        float currentAngleDeg = currentPos[i] / stepsPerDeg;
        float deltaAngle = targetAngles[i] - currentAngleDeg;
        long deltaSteps = round(deltaAngle * stepsPerDeg);

        // 强制最小步数：如果绝对值小于5且角度差大于0.01度，则设为5或-5
        if (abs(deltaSteps) < 5 && abs(deltaAngle) > 0.01) {
          deltaSteps = (deltaAngle > 0) ? 5 : -5;
          Serial.print("Force move axis "); Serial.print(i+1);
          Serial.print(" deltaSteps="); Serial.println(deltaSteps);
        }

        // 调试输出（打印最终使用的 deltaSteps）
        Serial.print("Axis "); Serial.print(i+1);
        Serial.print(" cur_deg="); Serial.print(currentAngleDeg);
        Serial.print(" target_deg="); Serial.print(targetAngles[i]);
        Serial.print(" deltaSteps="); Serial.println(deltaSteps);

        if (deltaSteps != 0) {
          if (deltaSteps < 0 && checkLimit(i)) {
            Serial.print("ERR: Axis ");
            Serial.print(i + 1);
            Serial.println(" at negative limit");
            continue;
          }
          taskAxis[taskCount] = i;
          taskSteps[taskCount] = deltaSteps;
          taskCount++;
          if (taskCount >= MAX_TASKS) break;
        }
      }
    }

    if (taskCount > 0) {
      taskIndex = 0;
      startMove(taskAxis[0], taskSteps[0]);
      taskIndex = 1;
      tasksActive = true;
      Serial.println("OK RJ");
    } else {
      Serial.println("OK RJ (no movement)");
    }
    return;
  }

  Serial.println("ERR: Unknown command");
}

void clearTaskQueue() {
  taskCount = 0;
  taskIndex = 0;
  tasksActive = false;
}

void processTasks() {
  if (taskIndex < taskCount) {
    startMove(taskAxis[taskIndex], taskSteps[taskIndex]);
    taskIndex++;
  } else {
    tasksActive = false;
    clearTaskQueue();
  }
}

void startMove(int axis, long steps) {
  if (steps == 0) return;
  if (estopTriggered) {
    Serial.println("ERR: Emergency stop active");
    return;
  }
  limitDebounce[axis] = 0;
  delayMicroseconds(500);
  if (steps < 0 && checkLimit(axis)) {
    Serial.print("ERR: Axis ");
    Serial.print(axis + 1);
    Serial.println(" negative limit");
    return;
  }
  activeAxis = axis;
  moving[axis] = true;
  moveDirection = (steps > 0) ? 1 : -1;
  moveTotalSteps = abs(steps);
  moveStepsDone = 0;
  digitalWrite(dirPin[axis], (moveDirection == 1) ? HIGH : LOW);
  delayMicroseconds(10);

  float v_max = maxSpeed;
  float a = acceleration;
  float accelDist = (v_max * v_max) / (2.0 * a);
  moveAccelSteps = (long)ceil(accelDist);
  moveDecelSteps = moveAccelSteps;
  if (moveTotalSteps <= (moveAccelSteps + moveDecelSteps)) {
    float v_peak = sqrt(a * moveTotalSteps);
    moveAccelSteps = (long)ceil((v_peak * v_peak) / (2.0 * a));
    moveDecelSteps = moveAccelSteps;
    moveConstSteps = 0;
  } else {
    moveConstSteps = moveTotalSteps - moveAccelSteps - moveDecelSteps;
  }
  currentSpeed = 200.0;
  lastPulseMicros = micros();
  nextPulseDelayUs = 1000000.0 / currentSpeed;
}

void updateMotion() {
  int axis = activeAxis;
  if (axis == -1 || !moving[axis]) return;
  if (estopTriggered) {
    moving[axis] = false;
    activeAxis = -1;
    return;
  }
  unsigned long now = micros();
  if (now - lastPulseMicros >= nextPulseDelayUs) {
    digitalWrite(stepPin[axis], LOW);
    delayMicroseconds(2);
    digitalWrite(stepPin[axis], HIGH);
    currentPos[axis] += moveDirection;
    moveStepsDone++;
    if (checkLimit(axis)) {
      Serial.print("Limit hit on axis ");
      Serial.println(axis + 1);
      moving[axis] = false;
      activeAxis = -1;
      reportPosition();
      return;
    }
    if (moveStepsDone >= moveTotalSteps) {
      moving[axis] = false;
      activeAxis = -1;
      return;
    }
    long stepsDoneNow = moveStepsDone;
    if (stepsDoneNow <= moveAccelSteps) {
      currentSpeed = (maxSpeed / moveAccelSteps) * stepsDoneNow;
      if (currentSpeed < 10.0) currentSpeed = 10.0;
    } else if (stepsDoneNow <= moveAccelSteps + moveConstSteps) {
      currentSpeed = maxSpeed;
    } else {
      long decelDone = stepsDoneNow - (moveAccelSteps + moveConstSteps);
      long decelTotal = moveDecelSteps;
      if (decelTotal > 0) {
        currentSpeed = maxSpeed * (1.0 - (float)decelDone / decelTotal);
      }
      if (currentSpeed < 10.0) currentSpeed = 10.0;
    }
    if (currentSpeed > 0) {
      nextPulseDelayUs = (unsigned long)(1000000.0 / currentSpeed);
      if (nextPulseDelayUs < 20) nextPulseDelayUs = 20;
    } else {
      nextPulseDelayUs = 1000;
    }
    lastPulseMicros = now;
  }
}

bool checkLimit(int axis) {
  int pinState = digitalRead(limitPin[axis]);
  bool triggered = (pinState == LOW);
  if (triggered) {
    limitDebounce[axis]++;
    if (limitDebounce[axis] >= limitDebounceCount) {
      return true;
    }
  } else {
    limitDebounce[axis] = 0;
  }
  return false;
}

void stopAxis(int axis) {
  if (activeAxis == axis) {
    moving[axis] = false;
    activeAxis = -1;
  }
}

void emergencyStop() {
  estopTriggered = true;
  for (int i = 0; i < 6; i++) moving[i] = false;
  activeAxis = -1;
  clearTaskQueue();
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

void homeAxis(int axis) {
  if (estopTriggered) {
    Serial.println("ERR: Emergency stop active");
    return;
  }
  if (activeAxis != -1 && moving[activeAxis]) {
    Serial.println("ERR: Another axis moving");
    return;
  }
  clearTaskQueue();

  limitDebounce[axis] = 0;
  delayMicroseconds(500);
  bool initiallyAtLimit = checkLimit(axis);
  if (initiallyAtLimit) {
    Serial.println("Home: already at limit, backing off...");
    digitalWrite(dirPin[axis], HIGH);
    delayMicroseconds(10);
    while (checkLimit(axis)) {
      if (estopTriggered) return;
      digitalWrite(stepPin[axis], LOW);
      delayMicroseconds(2);
      digitalWrite(stepPin[axis], HIGH);
      currentPos[axis]++;
      delayMicroseconds(2000);
    }
    delay(200);
  }
  Serial.print("Homing axis ");
  Serial.println(axis + 1);
  digitalWrite(dirPin[axis], LOW);
  delayMicroseconds(10);
  activeAxis = axis;
  moving[axis] = true;
  moveDirection = -1;
  moveTotalSteps = 100000;
  moveStepsDone = 0;
  currentSpeed = 400.0;
  nextPulseDelayUs = 1000000.0 / currentSpeed;
  lastPulseMicros = micros();

  while (moving[axis] && !estopTriggered) {
    updateMotion();
    delayMicroseconds(100);
  }
  if (estopTriggered) {
    Serial.println("Homing aborted");
    return;
  }
  delay(10);
  digitalWrite(dirPin[axis], HIGH);
  for (int i = 0; i < 100; i++) {
    digitalWrite(stepPin[axis], LOW);
    delayMicroseconds(2);
    digitalWrite(stepPin[axis], HIGH);
    currentPos[axis]++;
    delayMicroseconds(2000);
  }
  currentPos[axis] = 0;
  moving[axis] = false;
  activeAxis = -1;
  Serial.print("Axis ");
  Serial.print(axis + 1);
  Serial.println(" homed, position set to 0");
  reportPosition();
}

void emergencyStopISR() {
  estopTriggered = true;
  for (int i = 0; i < 6; i++) moving[i] = false;
  activeAxis = -1;
  clearTaskQueue();
}