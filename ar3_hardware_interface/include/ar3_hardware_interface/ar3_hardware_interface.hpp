#ifndef AR3_HARDWARE_INTERFACE__AR3_HARDWARE_INTERFACE_HPP_
#define AR3_HARDWARE_INTERFACE__AR3_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ar3_hardware_drivers/TeensyDriver.hpp"

namespace ar3_hardware_interface
{

class AR3HardwareInterface : public hardware_interface::SystemInterface
{
public:
  AR3HardwareInterface();
  ~AR3HardwareInterface() override;

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams& params) override;
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  std::unique_ptr<ar3_hardware_drivers::TeensyDriver> driver_;
  std::vector<std::string> joint_names_;

  std::vector<double> joint_position_states_;
  std::vector<double> joint_velocity_states_;
  std::vector<double> joint_effort_states_;
  std::vector<double> joint_position_commands_;

  std::vector<double> gear_ratios_;        // 保留但可能不再用于状态（取决于固件）
  std::vector<double> position_lower_limits_;
  std::vector<double> position_upper_limits_;
  std::vector<double> velocity_limits_;
  std::vector<double> acceleration_limits_;

  std::string serial_port_;
  int baudrate_;

  bool hardware_initialized_;
  bool hardware_connected_;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_all_service_;

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;

  bool parseHardwareParameters(const hardware_interface::HardwareInfo& info);
  void enforceJointLimits();
  void publishJointStates();

  // 调试标志，可选择性地打印日志
  bool debug_print_;
};

} // namespace ar3_hardware_interface

#endif