#ifndef TEENSYDRIVER_HPP
#define TEENSYDRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>

namespace ar3_hardware_drivers {

class TeensyDriver : public rclcpp::Node
{
public:
  explicit TeensyDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~TeensyDriver();

  bool init(const std::string& port, int baudrate, int num_joints);
  bool startReading();
  void stopReading();
  bool sendPositionCommands(const std::vector<double>& position_commands_deg);
  std::vector<double> getLatestJointPositions() const;
  bool emergencyStop();

  void setSpeedPercentage(double percentage);

  bool homeAxis(int axis);
  bool readLimitSwitch(int axis, bool& triggered);

private:
  int serial_fd_;
  int num_joints_;
  std::vector<double> latest_positions_;
  mutable std::mutex pos_mutex_;
  std::thread read_thread_;
  std::atomic<bool> running_;

  double speed_percentage_;
  mutable std::mutex serial_mutex_;
  std::chrono::steady_clock::time_point last_rp_time_;

  std::atomic<bool> sync_read_active_{false};

  bool openSerialPort(const std::string&, int);
  bool configureSerialPort(int, int);
  bool sendCommand(const std::string&);
  void readLoop();
  void parseFeedback(const std::string&);
  std::string sendAndReadLine(const std::string& cmd, int timeout_ms = 500);
};

} // namespace ar3_hardware_drivers

#endif