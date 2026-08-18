#include <Arduino.h>

const int stepPin[6]  = {0,  2,  4,  6,  8,  10};
const int dirPin[6]   = {1,  3,  5,  7,  9,  11};
const int limitPin[6] = {26, 27, 28, 29, 30, 31};
const int estopPin    = 39;

const int stepsPerRev[6] = {800, 800, 800, 800, 800, 800};

float maxSpeed     = 2000.0;
float acceleration = 1000.0;

const double gearRatio[6] = {10.0, 50.0, 50.0, 14.0, 1.0, 19.0};

float home_backoff_deg_joint[6] = {6255.0, 2070.0, 6120.0, 3150.0, 975.0, 3150.0};

const long homeOffsetSteps[6] = {0, 0, 0, 0, 0, 0};

volatile long  currentPos[6]    = {0};
volatile bool  moving[6]        = {false};
volatile int   activeAxis       = -1;
volatile bool  estopTriggered   = false;
bool           estopPending     = false;

const int limitDebounceCount = 3;
int limitDebounce[6] = {0};

volatile long  moveTotalSteps    = 0;
volatile long  moveStepsDone     = 0;
volatile long  moveAccelSteps    = 0;
volatile long  moveDecelSteps    = 0;
volatile long  moveConstSteps    = 0;
volatile int   moveDirection     = 0;
volatile float currentSpeed      = 0.0;
volatile unsigned long lastPulseMicros    = 0;
volatile unsigned long nextPulseDelayUs  = 0;

const int MAX_TASKS = 6;
int   taskAxis[MAX_TASKS];
long  taskSteps[MAX_TASKS];
int   taskCount = 0;
int   taskIndex = 0;
bool  tasksActive = false;

String inputString = "";
const int MAX_INPUT_LENGTH = 128;

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

  setMaxSpeed(3000.0);
  setAcceleration(1000.0);

  Serial.println("AR3 Robot Ready (Scheme 2: Motor angle commands, joint angle feedback)");
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

  if (estopPending) {
    emergencyStop();
    estopPending = false;
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
      double motor_deg = currentPos[i] * (360.0 / stepsPerRev[i]);
      double joint_deg = motor_deg / gearRatio[i];
      resp += tag;
      resp += String(joint_deg, 2);
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
    setMaxSpeed(20000.0 * pct / 100.0);
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

  if (cmd.startsWith("RJ")) {
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
        // 当前电机角度（度）
        double currentMotorDeg = currentPos[i] * (360.0 / stepsPerRev[i]);
        double deltaMotorDeg = targetAngles[i] - currentMotorDeg;
        long deltaSteps = round(deltaMotorDeg / (360.0 / stepsPerRev[i]));

        if (abs(deltaSteps) < 3 && abs(deltaMotorDeg) > 0.05) {
          double minMotorDeg = 0.1;
          long minSteps = max(1, (long)round(minMotorDeg / (360.0 / stepsPerRev[i])));
          deltaSteps = (deltaMotorDeg > 0) ? minSteps : -minSteps;
          Serial.print("Force move axis "); Serial.print(i+1);
          Serial.print(" deltaSteps="); Serial.println(deltaSteps);
        }

        Serial.print("Axis "); Serial.print(i+1);
        Serial.print(" cur_motor_deg="); Serial.print(currentMotorDeg);
        Serial.print(" target_motor_deg="); Serial.print(targetAngles[i]);
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
      clearTaskQueue();
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

void emergencyStopISR() {
  estopPending = true;
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
  if (checkLimit(axis)) {
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

  Serial.print("Homing axis "); Serial.println(axis + 1);
  digitalWrite(dirPin[axis], LOW);
  delayMicroseconds(10);
  activeAxis = axis;
  moving[axis] = true;
  moveDirection = -1;

  float old_maxSpeed = maxSpeed;
  float old_acc = acceleration;
  maxSpeed = 2000.0;
  acceleration = 1000.0;

  moveTotalSteps = 200000000;
  moveStepsDone = 0;
  currentSpeed = 200.0;
  nextPulseDelayUs = 1000000.0 / currentSpeed;
  lastPulseMicros = micros();

  while (moving[axis] && !estopTriggered) {
    updateMotion();
  }

  maxSpeed = old_maxSpeed;
  acceleration = old_acc;

  if (estopTriggered) {
    Serial.println("Homing aborted");
    return;
  }

  float backoff_deg_joint = home_backoff_deg_joint[axis];
  double backoff_deg_motor = backoff_deg_joint;
  long backoff_steps = (long)round(backoff_deg_motor / (360.0 / stepsPerRev[axis]));
  if (backoff_steps > 0) {
    Serial.print("Backing off axis "); Serial.print(axis+1);
    Serial.print(" by "); Serial.print(backoff_deg_joint); Serial.print(" joint degrees -> ");
    Serial.print(backoff_deg_motor); Serial.println(" motor degrees");
    digitalWrite(dirPin[axis], HIGH);
    delayMicroseconds(10);
    for (long i = 0; i < backoff_steps; i++) {
      if (estopTriggered) return;
      digitalWrite(stepPin[axis], LOW);
      delayMicroseconds(2);
      digitalWrite(stepPin[axis], HIGH);
      currentPos[axis]++;
      delayMicroseconds(2000);
    }
  }

  currentPos[axis] = homeOffsetSteps[axis];
  moving[axis] = false;
  activeAxis = -1;

  double motor_deg = currentPos[axis] * (360.0 / stepsPerRev[axis]);
  double joint_deg = motor_deg / gearRatio[axis];
  Serial.print("Axis "); Serial.print(axis+1);
  Serial.print(" homed -> motor steps = "); Serial.print(currentPos[axis]);
  Serial.print(", joint angle = "); Serial.print(joint_deg); Serial.println(" deg");
  reportPosition();
}