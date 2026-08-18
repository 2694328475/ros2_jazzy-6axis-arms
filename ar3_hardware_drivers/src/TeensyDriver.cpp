#include "ar3_hardware_drivers/TeensyDriver.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <sys/select.h>
#include <chrono>
#include <cmath>

namespace ar3_hardware_drivers {

TeensyDriver::TeensyDriver(const rclcpp::NodeOptions& options)
  : Node("teensy_driver", options),
    serial_fd_(-1), num_joints_(0), running_(false),
    speed_percentage_(100.0)
{
}

TeensyDriver::~TeensyDriver()
{
  stopReading();
  if (serial_fd_ >= 0) close(serial_fd_);
}

bool TeensyDriver::init(const std::string& port, int baudrate, int num_joints)
{
  serial_fd_ = -1;
  num_joints_ = num_joints;
  latest_positions_.assign(num_joints_, 0.0);
  last_rp_time_ = std::chrono::steady_clock::now();

  if (!openSerialPort(port, baudrate)) return false;
  RCLCPP_INFO(get_logger(), "TeensyDriver initialized on %s baud %d", port.c_str(), baudrate);
  return true;
}

bool TeensyDriver::startReading()
{
  if (!running_ && serial_fd_ >= 0) {
    running_ = true;
    read_thread_ = std::thread(&TeensyDriver::readLoop, this);
    RCLCPP_INFO(get_logger(), "Started reading thread");
    return true;
  }
  return false;
}

void TeensyDriver::stopReading()
{
  running_ = false;
  if (read_thread_.joinable()) read_thread_.join();
}

void TeensyDriver::setSpeedPercentage(double percentage)
{
  if (percentage < 1.0) percentage = 1.0;
  if (percentage > 100.0) percentage = 100.0;
  speed_percentage_ = percentage;
  std::string cmd = "SP" + std::to_string((int)percentage) + "\n";
  sendCommand(cmd);
  RCLCPP_INFO(get_logger(), "Set speed to %.0f%%", percentage);
}

bool TeensyDriver::sendPositionCommands(const std::vector<double>& position_commands_deg)
{
  if (serial_fd_ < 0 || position_commands_deg.size() != (size_t)num_joints_) {
    RCLCPP_ERROR(get_logger(), "sendPositionCommands: invalid state (fd=%d, size=%zu, expected=%d)",
                 serial_fd_, position_commands_deg.size(), num_joints_);
    return false;
  }

  std::string cmd = "RJ";
  for (int i = 0; i < num_joints_ && i < 6; ++i) {
    cmd += char('A' + i);
    cmd += std::to_string(position_commands_deg[i]);
  }
  cmd += "\n";

  bool ret = sendCommand(cmd);
  if (!ret) {
    RCLCPP_ERROR(get_logger(), "sendPositionCommands failed to send: %s", cmd.c_str());
  } else {
    RCLCPP_DEBUG(get_logger(), "Sent RJ command: %s", cmd.c_str());
  }
  return ret;
}

bool TeensyDriver::emergencyStop()
{
  return sendCommand("estop\n");
}

std::vector<double> TeensyDriver::getLatestJointPositions() const
{
  std::lock_guard<std::mutex> lock(pos_mutex_);
  return latest_positions_;
}

bool TeensyDriver::homeAxis(int axis)
{
  if (axis < 0 || axis >= num_joints_) return false;
  std::string cmd = "home " + std::to_string(axis + 1) + "\n";
  return sendCommand(cmd);
}

bool TeensyDriver::readLimitSwitch(int axis, bool& triggered)
{
  if (axis < 0 || axis >= num_joints_) return false;
  std::string cmd = "pin " + std::to_string(axis + 1) + "\n";
  std::string response = sendAndReadLine(cmd);
  if (response.empty()) return false;
  size_t eq = response.find('=');
  if (eq != std::string::npos) {
    try {
      int val = std::stoi(response.substr(eq + 1));
      triggered = (val == 1);
      return true;
    } catch (...) {}
  }
  return false;
}

std::string TeensyDriver::sendAndReadLine(const std::string& cmd, int timeout_ms)
{
  if (serial_fd_ < 0) return "";
  sync_read_active_ = true;
  std::lock_guard<std::mutex> lock(serial_mutex_);
  ssize_t written = write(serial_fd_, cmd.c_str(), cmd.length());
  if (written < 0) {
    sync_read_active_ = false;
    return "";
  }
  tcdrain(serial_fd_);
  std::string line;
  char ch;
  fd_set readfds;
  struct timeval tv;
  auto start = std::chrono::steady_clock::now();
  while (true) {
    FD_ZERO(&readfds);
    FD_SET(serial_fd_, &readfds);
    tv.tv_sec = 0;
    tv.tv_usec = 10000;
    int ret = select(serial_fd_ + 1, &readfds, NULL, NULL, &tv);
    if (ret > 0 && FD_ISSET(serial_fd_, &readfds)) {
      if (read(serial_fd_, &ch, 1) == 1) {
        if (ch == '\n') break;
        line += ch;
      }
    }
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start);
    if (elapsed.count() > timeout_ms) break;
  }
  sync_read_active_ = false;
  return line;
}

bool TeensyDriver::openSerialPort(const std::string& port, int baudrate)
{
  serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to open %s", port.c_str());
    return false;
  }
  return configureSerialPort(serial_fd_, baudrate);
}

bool TeensyDriver::configureSerialPort(int fd, int baudrate)
{
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd, &tty)) return false;
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_lflag &= ~(ICANON | ECHO | ISIG);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 10;
  return (tcsetattr(fd, TCSANOW, &tty) == 0);
}

bool TeensyDriver::sendCommand(const std::string& command)
{
  if (serial_fd_ < 0) return false;
  sync_read_active_ = true;
  std::lock_guard<std::mutex> lock(serial_mutex_);
  ssize_t written = write(serial_fd_, command.c_str(), command.size());
  if (written < 0 || written != (ssize_t)command.size()) {
    RCLCPP_ERROR(get_logger(), "sendCommand: wrote %ld of %zu bytes", written, command.size());
    sync_read_active_ = false;
    return false;
  }
  tcdrain(serial_fd_);
  sync_read_active_ = false;
  return true;
}

void TeensyDriver::readLoop()
{
  char buf[256];
  std::string line;
  fd_set readfds;
  struct timeval tv;
  while (running_) {
    if (sync_read_active_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    FD_ZERO(&readfds);
    FD_SET(serial_fd_, &readfds);
    tv.tv_sec = 0;
    tv.tv_usec = 50000;
    int ret = select(serial_fd_ + 1, &readfds, NULL, NULL, &tv);
    if (ret > 0 && FD_ISSET(serial_fd_, &readfds)) {
      ssize_t n = read(serial_fd_, buf, sizeof(buf) - 1);
      if (n > 0) {
        buf[n] = '\0';
        line += buf;
        size_t pos;
        while ((pos = line.find('\n')) != std::string::npos) {
          parseFeedback(line.substr(0, pos));
          line.erase(0, pos + 1);
        }
      }
    }
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_rp_time_);
    if (elapsed.count() > 100) {
      sendCommand("RP\n");
      last_rp_time_ = now;
    }
  }
}

void TeensyDriver::parseFeedback(const std::string& line)
{
  if (line.rfind("RP", 0) != 0) return;
  RCLCPP_DEBUG(get_logger(), "Received RP: %s", line.c_str());
  std::lock_guard<std::mutex> lock(pos_mutex_);
  for (int i = 0; i < 6 && i < num_joints_; ++i) {
    char tag = 'A' + i;
    size_t idx = line.find(tag);
    if (idx != std::string::npos) {
      size_t end = line.find_first_of("ABCDEF", idx + 1);
      std::string val_str = line.substr(idx + 1, end - idx - 1);
      try {
        double deg = std::stod(val_str);
        latest_positions_[i] = deg * M_PI / 180.0;   // 度 -> 弧度
        RCLCPP_DEBUG(get_logger(), "Parsed joint %d: %.2f deg -> %.4f rad", i, deg, latest_positions_[i]);
      } catch (...) {
        RCLCPP_WARN(get_logger(), "Failed to parse value for joint %d", i);
      }
    }
  }
}

} // namespace ar3_hardware_drivers