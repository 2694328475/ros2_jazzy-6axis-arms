/***********************************************
 * 力控夹爪控制程序 v4.0（36:1 减速比适配）
 * 硬件：Teensy 4.1 + SN65HVD230 + C610电调 + M2006电机（减速比36）
 * 功能：低速接近(输出轴20rpm可设) → 碰物 → 恒力夹持 → 输出轴行程保护
 * 串口命令：GRIP=1, STOP=1, OPEN=1, 参数调节等
 ***********************************************/

#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// ===================== 硬件参数 =====================
const int MOTOR_ID = 1;                     // 电调ID
const float GEAR_RATIO = 36.0;             // 减速比（电机转子 : 输出轴）
const unsigned long UPDATE_PERIOD = 1;      // 1kHz 控制
const unsigned long SERIAL_PERIOD = 100;    // 调试输出周期(ms)
const unsigned long WATCHDOG_TIMEOUT = 100; // CAN看门狗(ms)

// ===================== 可调参数（默认值） =====================
int FORCE_THRESHOLD = 400;          // 力控目标电流 (2500~10000)
int MAX_SPEED_RPM = 1;             // 输出轴最高接近转速 (rpm)
int RAMP_TIME_MS = 300;             // 电流斜坡时间 (ms)
float MAX_TRAVEL_ANGLE = 120.0;     // 输出轴最大允许旋转角度 (度)
int CLOSE_SPEED = 40;               // 回正(张开)电流值 (正值=张开方向)

// ===================== PI 速度控制参数（基于输出轴转速） =====================
float g_speed_integral = 0.0;
const float SPEED_KP = 0.4;         // 比例系数
const float SPEED_KI = 0.02;        // 积分系数
const float SPEED_INTEGRAL_MAX = 1500.0; // 积分限幅

// ===================== 控制状态变量 =====================
bool g_is_gripping = false;         // 是否正在夹取
bool g_is_opening = false;          // 是否正在回正（张开）
bool g_is_debug = false;            // 定时调试输出
bool g_force_reached = false;       // 是否已达目标力
bool g_angle_overflow = false;      // 是否因超行程而终止
unsigned long g_last_update_us = 0;
unsigned long g_last_serial_ms = 0;
unsigned long g_last_feedback_ms = 0;
unsigned long g_stall_start_ms = 0;
bool g_stall_flag = false;
unsigned long g_log_count = 0;

// 行程累积（输出轴角度）
float g_output_travel_deg = 0.0;    // 从夹取启动累积的输出轴角度变化（度）

float g_current_ramp = 0.0;         // 斜坡当前值
unsigned long g_ramp_start_ms = 0;  // 斜坡开始时间

// 用于处理转子绝对角度溢出（单圈传感器）
uint16_t g_last_angle_raw = 0;
bool g_angle_first = true;          // 第一次读取标志

// ===================== 反馈数据结构 =====================
struct MotorFeedback {
  uint16_t angle_raw;        // 转子原始角度值 (0~8191)
  float rotor_angle_deg;     // 转子绝对角度 (°)
  int16_t rotor_rpm;         // 转子转速 (rpm)
  int16_t torque_raw;        // 转矩电流

  float output_rpm;          // 输出轴转速 (rpm)
  float output_angle_delta;  // 本次反馈周期内输出轴转过的角度 (°)

  uint16_t timestamp;
};
MotorFeedback g_fb;

// ===================== 函数声明 =====================
void parse_serial_command(const String &cmd);
void send_command_to_motor(int16_t current_val);
void read_motor_feedback(uint8_t *rx_data, uint8_t rx_len, uint32_t msg_id);
void force_grip_action();
void print_feedback();
void print_help();
void stop_motor();
void save_config();

// ================================================================
// 初始化
// ================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("=== 力控夹爪控制系统 v4.0 (36:1减速比) ===");
  Serial.println("硬件: Teensy4.1 + C610 + M2006 (减速比36)");
  Can1.begin();
  Can1.setBaudRate(1000000);
  Serial.println("CAN1 初始化完成 (1Mbps)");
  Serial.print("默认力控阈值: "); Serial.println(FORCE_THRESHOLD);
  Serial.print("输出轴最高转速: "); Serial.print(MAX_SPEED_RPM); Serial.println(" rpm");
  Serial.print("电流斜坡时间: "); Serial.print(RAMP_TIME_MS); Serial.println(" ms");
  Serial.print("输出轴最大行程: "); Serial.print(MAX_TRAVEL_ANGLE); Serial.println("°");
  Serial.println("输入 HELP 查看命令");
  g_last_update_us = micros();
}

// ================================================================
// 主循环
// ================================================================
void loop() {
  unsigned long now_us = micros();
  unsigned long now_ms = millis();

  // ---------- 1kHz 控制命令发送 ----------
  if (now_us - g_last_update_us >= UPDATE_PERIOD * 1000UL) {
    g_last_update_us = now_us;

    if (g_is_gripping || g_is_opening) {
      int16_t target_current = 0;

      if (g_is_opening) {
        // 回正模式：输出固定张开电流
        target_current = CLOSE_SPEED;
      } else if (g_is_gripping) {
        // ---------- 夹取模式 ----------
        // 电流斜坡
        if (g_ramp_start_ms == 0) g_ramp_start_ms = now_ms;
        unsigned long elapsed = now_ms - g_ramp_start_ms;
        if (elapsed < RAMP_TIME_MS) {
          g_current_ramp = (float)FORCE_THRESHOLD * elapsed / RAMP_TIME_MS;
        } else {
          g_current_ramp = FORCE_THRESHOLD;
        }

        target_current = (int16_t)g_current_ramp;

        // 速度 PI 控制（基于输出轴转速，仅在未达力且未超行程时启用）
        if (!g_force_reached && !g_angle_overflow) {
          float output_speed = fabs(g_fb.output_rpm);   // 输出轴转速绝对值
          if (output_speed > MAX_SPEED_RPM) {
            float speed_error = output_speed - MAX_SPEED_RPM;
            // 比例项
            float reduction = speed_error * SPEED_KP;
            // 积分项
            g_speed_integral += speed_error * SPEED_KI;
            if (g_speed_integral > SPEED_INTEGRAL_MAX) g_speed_integral = SPEED_INTEGRAL_MAX;
            if (g_speed_integral < -SPEED_INTEGRAL_MAX) g_speed_integral = -SPEED_INTEGRAL_MAX;
            reduction += g_speed_integral;

            target_current = g_current_ramp - reduction;
            if (target_current < 0) target_current = 0;
          } else {
            g_speed_integral = 0.0f;
          }

          // 行程角度保护（输出轴累积角度）
          if (g_output_travel_deg >= MAX_TRAVEL_ANGLE) {
            g_angle_overflow = true;
            g_force_reached = true;
            target_current = 0;
            Serial.print("输出轴行程超限保护：已转过 "); Serial.print(g_output_travel_deg); Serial.println("°");
          }
        } else {
          // 力已达或行程超限，清除积分，保持目标力/停止
          g_speed_integral = 0.0f;
          target_current = (g_angle_overflow) ? 0 : FORCE_THRESHOLD;
        }
      }

      target_current = constrain(target_current, -10000, 10000);
      send_command_to_motor(target_current);
    } else {
      send_command_to_motor(0);
      g_ramp_start_ms = 0;
      g_speed_integral = 0.0f;
    }
  }

  // ---------- CAN 反馈接收 ----------
  CAN_message_t rx_msg;
  uint8_t feedback_raw[8];
  while (Can1.read(rx_msg)) {
    if (rx_msg.id == (uint32_t)(0x200 + MOTOR_ID)) {
      uint8_t len = min(rx_msg.len, (uint8_t)8);
      memcpy(feedback_raw, rx_msg.buf, len);
      g_last_feedback_ms = now_ms;

      read_motor_feedback(feedback_raw, len, rx_msg.id);   // 更新反馈（含输出轴转换）

      if (g_is_gripping) {
        force_grip_action();
      }
    }
  }

  // ---------- CAN 看门狗 ----------
  if ((g_is_gripping || g_is_opening) && (now_ms - g_last_feedback_ms > WATCHDOG_TIMEOUT)) {
    Serial.println("CAN通信中断，紧急停机！");
    g_is_gripping = false;
    g_is_opening = false;
    send_command_to_motor(0);
  }

  // ---------- 串口命令 ----------
  if (Serial.available() > 0) {
    String cmd_line = Serial.readStringUntil('\n');
    cmd_line.trim();
    if (cmd_line.length() > 0) parse_serial_command(cmd_line);
  }

  // ---------- 定时调试输出 ----------
  if (g_is_debug && (now_ms - g_last_serial_ms >= SERIAL_PERIOD)) {
    g_last_serial_ms = now_ms;
    g_log_count++;
    Serial.print("[#"); Serial.print(g_log_count); Serial.print("] ");
    print_feedback();
    if (g_is_gripping) {
      Serial.print(" 目标力:"); Serial.print(FORCE_THRESHOLD);
      Serial.print(" 输出轴行程:"); Serial.print(g_output_travel_deg); Serial.print("/"); Serial.print(MAX_TRAVEL_ANGLE);
    }
    Serial.println();
  }
}

// ================================================================
// 发送电流命令到C610电调
// ================================================================
void send_command_to_motor(int16_t current_val) {
  CAN_message_t tx_msg;
  tx_msg.id = 0x200;
  tx_msg.len = 8;
  uint16_t cmd = (uint16_t)constrain(current_val, -10000, 10000);
  tx_msg.buf[0] = (cmd >> 8) & 0xFF;
  tx_msg.buf[1] = cmd & 0xFF;
  memset(&tx_msg.buf[2], 0, 6);
  Can1.write(tx_msg);
}

// ================================================================
// 解析电机反馈数据（含减速比转换 & 角度累积）
// ================================================================
void read_motor_feedback(uint8_t *rx_data, uint8_t rx_len, uint32_t msg_id) {
  uint16_t angle_raw = ((uint16_t)rx_data[0] << 8) | rx_data[1];
  int16_t rpm = ((int16_t)rx_data[2] << 8) | rx_data[3];
  int16_t torque = ((int16_t)rx_data[4] << 8) | rx_data[5];

  g_fb.angle_raw = angle_raw;
  g_fb.rotor_angle_deg = angle_raw * 360.0f / 8191.0f;
  g_fb.rotor_rpm = rpm;
  g_fb.torque_raw = torque;
  g_fb.timestamp = (uint16_t)(millis() % 65536);

  // ---------- 计算输出轴转速 ----------
  g_fb.output_rpm = rpm / GEAR_RATIO;

  // ---------- 计算转子角度变化量（处理单圈溢出） ----------
  if (g_angle_first) {
    g_last_angle_raw = angle_raw;
    g_angle_first = false;
    g_fb.output_angle_delta = 0.0f;
    return;
  }

  int16_t diff = (int16_t)(angle_raw - g_last_angle_raw);
  // 处理 0/8191 边界：如果差值超过半圈范围（±4096），视为溢出
  if (diff > 4096) {
    diff -= 8192;   // 负向溢出（从高位跳到低位）
  } else if (diff < -4096) {
    diff += 8192;   // 正向溢出（从低位跳到高位）
  }

  // 差分值转换为转子转过的角度 (°)
  float rotor_delta_deg = (float)diff * 360.0f / 8191.0f;

  // 输出轴本次周期转过的角度
  g_fb.output_angle_delta = rotor_delta_deg / GEAR_RATIO;

  // 累积输出轴总行程（仅在夹取状态下累积，但我们总是累积也无妨，GRIP时重置）
  g_output_travel_deg += fabs(g_fb.output_angle_delta);   // 用绝对值保证行程只增不减

  g_last_angle_raw = angle_raw;
}

// ================================================================
// 力控夹取判定（失速检测，维持100ms以上判定）
// ================================================================
void force_grip_action() {
  if (g_force_reached) return;

  int16_t t = g_fb.torque_raw;
  float output_rpm_abs = fabs(g_fb.output_rpm);

  // 力达到阈值90%且输出轴转速很低（接近停转）
  if (abs(t) >= abs(FORCE_THRESHOLD) * 0.9f && output_rpm_abs < 5.0f) {  // 输出轴低速阈值5rpm
    if (!g_stall_flag) {
      g_stall_flag = true;
      g_stall_start_ms = millis();
    } else if (millis() - g_stall_start_ms >= 150) {
      g_force_reached = true;
      g_stall_flag = false;
      Serial.print("夹取完成！力电流="); Serial.print(t);
      Serial.print(" 输出轴行程="); Serial.print(g_output_travel_deg); Serial.println("°");
    }
  } else {
    g_stall_flag = false;
  }
}

// ================================================================
// 停止所有运动
// ================================================================
void stop_motor() {
  g_is_gripping = false;
  g_is_opening = false;
  g_force_reached = false;
  g_stall_flag = false;
  g_angle_overflow = false;
  g_speed_integral = 0.0f;
  send_command_to_motor(0);
  Serial.println("电机已停止");
}

// ================================================================
// 串口命令解析
// ================================================================
void parse_serial_command(const String &cmd_line) {
  String upper_cmd = cmd_line;
  upper_cmd.toUpperCase();
  upper_cmd.trim();

  int eq_pos = upper_cmd.indexOf('=');
  String cmd, val_str;
  if (eq_pos > 0) {
    cmd = upper_cmd.substring(0, eq_pos);
    val_str = upper_cmd.substring(eq_pos + 1);
  } else {
    cmd = upper_cmd;
    val_str = "";
  }

  if (cmd == "GRIP" && val_str == "1") {
    stop_motor();                  // 先清空状态
    g_is_gripping = true;
    g_output_travel_deg = 0.0f;   // 重置输出轴行程累积
    g_angle_first = true;         // 重新初始化转子角度基准
    g_ramp_start_ms = millis();
    g_speed_integral = 0.0f;
    Serial.println("夹取启动！输出轴行程已清零");
  }
  else if (cmd == "OPEN" && val_str == "1") {
    stop_motor();
    g_is_opening = true;
    Serial.println("回正（张开）启动");
  }
  else if (cmd == "STOP" && val_str == "1") {
    stop_motor();
  }
  else if (cmd == "FORCE_THR") {
    int v = val_str.toInt();
    if (v >= 2500 && v <= 10000) {
      FORCE_THRESHOLD = v;
      Serial.print("力控阈值更新: "); Serial.println(v);
    } else Serial.println("范围 2500~10000");
  }
  else if (cmd == "MAX_SPEED") {
    int v = val_str.toInt();
    if (v >= 1 && v <= 200) {            // 输出轴转速范围1~200rpm
      MAX_SPEED_RPM = v;
      Serial.print("输出轴最高转速更新: "); Serial.print(v); Serial.println(" rpm");
    } else Serial.println("输出轴转速范围 1~200 rpm（对应转子36~7200 rpm）");
  }
  else if (cmd == "RAMP_TIME") {
    int v = val_str.toInt();
    if (v >= 10 && v <= 1000) {
      RAMP_TIME_MS = v;
      Serial.print("斜坡时间更新: "); Serial.print(v); Serial.println(" ms");
    } else Serial.println("范围 10~1000 ms");
  }
  else if (cmd == "MAX_ANGLE") {
    float v = val_str.toFloat();
    if (v >= 10.0 && v <= 360.0) {
      MAX_TRAVEL_ANGLE = v;
      Serial.print("输出轴最大行程更新: "); Serial.print(v); Serial.println("°");
    } else Serial.println("范围 10~360°");
  }
  else if (cmd == "CLOSE_SPEED") {
    int v = val_str.toInt();
    if (abs(v) <= 10000) {
      CLOSE_SPEED = v;
      Serial.print("回正电流更新: "); Serial.println(v);
    }
  }
  else if (cmd == "DEBUG") {
    g_is_debug = (val_str == "1");
    Serial.print("调试输出: "); Serial.println(g_is_debug ? "开启" : "关闭");
  }
  else if (cmd == "CHECK") {
    print_feedback();
  }
  else if (cmd == "SAVE") {
    save_config();
  }
  else if (cmd == "HELP") {
    print_help();
  }
  else {
    Serial.print("未知命令: "); Serial.println(cmd_line);
  }
}

// ================================================================
void print_feedback() {
  Serial.print("输出轴:"); Serial.print(g_fb.output_rpm, 1);
  Serial.print("rpm 行程累积:"); Serial.print(g_output_travel_deg, 1);
  Serial.print("° 力矩:"); Serial.print(g_fb.torque_raw);
}

void save_config() {
  Serial.println("⚙️ 当前配置:");
  Serial.print("  FORCE_THR = "); Serial.println(FORCE_THRESHOLD);
  Serial.print("  MAX_SPEED = "); Serial.print(MAX_SPEED_RPM); Serial.println(" rpm (输出轴)");
  Serial.print("  RAMP_TIME = "); Serial.print(RAMP_TIME_MS); Serial.println(" ms");
  Serial.print("  MAX_ANGLE = "); Serial.print(MAX_TRAVEL_ANGLE); Serial.println("° (输出轴)");
  Serial.print("  CLOSE_SPEED = "); Serial.println(CLOSE_SPEED);
}

void print_help() {
  Serial.println("\n===== 力控夹爪命令列表（36:1减速比） =====");
  Serial.println("GRIP=1         启动力控夹取（低速接近 + 行程保护）");
  Serial.println("OPEN=1         回正（张开），电流由CLOSE_SPEED设定");
  Serial.println("STOP=1         停止电机（零力矩）");
  Serial.println("FORCE_THR=8000 力控目标电流 (2500~10000)");
  Serial.println("MAX_SPEED=20   输出轴最高转速 (rpm, 1~200)");
  Serial.println("RAMP_TIME=300  电流斜坡时间 (ms, 10~1000)");
  Serial.println("MAX_ANGLE=180  输出轴最大允许角度 (10~360°)");
  Serial.println("CLOSE_SPEED=80 回正电流值 (-10000~10000)");
  Serial.println("DEBUG=1        开启/关闭定时调试输出");
  Serial.println("CHECK=1        查看实时反馈数据");
  Serial.println("SAVE           显示当前配置");
  Serial.println("HELP           显示本帮助");
  Serial.println("================================\n");
}
