#include "ar3_hardware_interface/ar3_hardware_interface.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace ar3_hardware_interface
{

AR3HardwareInterface::AR3HardwareInterface()
    : hardware_initialized_(false),
      hardware_connected_(false),
      debug_print_(false)   // 开启调试输出，便于排查
{
}

AR3HardwareInterface::~AR3HardwareInterface()
{
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

hardware_interface::CallbackReturn AR3HardwareInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.clear();
  for (const auto& joint : info_.joints) {
    joint_names_.push_back(joint.name);
  }
  size_t num_joints = joint_names_.size();

  joint_position_states_.resize(num_joints, 0.0);
  joint_velocity_states_.resize(num_joints, 0.0);
  joint_effort_states_.resize(num_joints, 0.0);
  joint_position_commands_.resize(num_joints, 0.0);

  if (!parseHardwareParameters(info_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool AR3HardwareInterface::parseHardwareParameters(const hardware_interface::HardwareInfo& info)
{
  try {
    size_t num_joints = info.joints.size();

    gear_ratios_.resize(num_joints, 1.0);
    position_lower_limits_.resize(num_joints, -std::numeric_limits<double>::max());
    position_upper_limits_.resize(num_joints, std::numeric_limits<double>::max());
    velocity_limits_.resize(num_joints, std::numeric_limits<double>::max());

    serial_port_ = info.hardware_parameters.count("serial_port") ?
                   info.hardware_parameters.at("serial_port") : "/dev/ttyACM0";
    baudrate_ = info.hardware_parameters.count("baudrate") ?
                std::stoi(info.hardware_parameters.at("baudrate")) : 115200;

    for (size_t i = 0; i < num_joints; ++i) {
      std::string param_name = "gear_ratio_joint_" + std::to_string(i+1);
      if (info.hardware_parameters.count(param_name)) {
        gear_ratios_[i] = std::stod(info.hardware_parameters.at(param_name));
      }
    }

    for (size_t i = 0; i < num_joints; ++i) {
      const auto& joint = info.joints[i];
      if (joint.parameters.count("min_position"))
        position_lower_limits_[i] = std::stod(joint.parameters.at("min_position"));
      if (joint.parameters.count("max_position"))
        position_upper_limits_[i] = std::stod(joint.parameters.at("max_position"));
      if (joint.parameters.count("max_velocity"))
        velocity_limits_[i] = std::stod(joint.parameters.at("max_velocity"));
    }
    return true;
  } catch (...) {
    return false;
  }
}

hardware_interface::CallbackReturn AR3HardwareInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  node_ = std::make_shared<rclcpp::Node>("ar3_hardware_node");
  
  // 使用 SystemDefaultsQoS 以兼容 MoveIt 的订阅
  joint_state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SystemDefaultsQoS());

  driver_ = std::make_unique<ar3_hardware_drivers::TeensyDriver>();
  hardware_connected_ = driver_->init(serial_port_, baudrate_, joint_names_.size());
  if (!hardware_connected_) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize TeensyDriver");
    return hardware_interface::CallbackReturn::ERROR;
  }

  home_all_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/home_all",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;
        RCLCPP_INFO(node_->get_logger(), "Home service called. Sending home commands...");
        if (!hardware_connected_) {
          response->success = false;
          response->message = "Hardware not connected";
          return;
        }
        for (size_t i = 0; i < joint_names_.size(); ++i) {
          if (!driver_->homeAxis(i)) {
            response->success = false;
            response->message = "Failed to send home for axis " + std::to_string(i+1);
            return;
          }
        }
        response->success = true;
        response->message = "Home commands sent.";
      });

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() {
    executor_->spin();
  });

  hardware_initialized_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AR3HardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!hardware_initialized_) return hardware_interface::CallbackReturn::ERROR;
  driver_->startReading();
  driver_->setSpeedPercentage(25.0);
  RCLCPP_INFO(node_->get_logger(), "Hardware activated, speed set to 25%%");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AR3HardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (driver_ && hardware_connected_) {
    driver_->stopReading();
    driver_->emergencyStop();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AR3HardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  executor_.reset();

  if (driver_) {
    driver_->stopReading();
    driver_.reset();
  }
  joint_state_publisher_.reset();
  node_.reset();
  hardware_initialized_ = false;
  hardware_connected_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AR3HardwareInterface::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  return on_cleanup(rclcpp_lifecycle::State());
}

hardware_interface::CallbackReturn AR3HardwareInterface::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (driver_ && hardware_connected_) {
    driver_->stopReading();
    driver_->emergencyStop();
  }
  return hardware_interface::CallbackReturn::ERROR;
}

std::vector<hardware_interface::StateInterface> AR3HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION,
                                  &joint_position_states_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AR3HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION,
                                    &joint_position_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type AR3HardwareInterface::read(
    const rclcpp::Time& time, const rclcpp::Duration& period)
{
  (void)time;
  if (!hardware_connected_ || !driver_) return hardware_interface::return_type::ERROR;

  // 从 Teensy 获取当前关节角度（弧度），TeensyDriver 已经将度转换为弧度
  std::vector<double> joint_angles_rad = driver_->getLatestJointPositions();
  
  if (debug_print_) {
    RCLCPP_INFO(node_->get_logger(), "read: joint_angles_rad size=%zu", joint_angles_rad.size());
    for (size_t i = 0; i < joint_angles_rad.size(); ++i) {
      RCLCPP_INFO(node_->get_logger(), "  joint[%zu] rad=%.4f deg=%.2f",
                  i, joint_angles_rad[i], joint_angles_rad[i] * 180.0 / M_PI);
    }
  }

  if (joint_angles_rad.size() == joint_position_states_.size()) {
    // 计算速度（如果周期有效）
    if (period.seconds() > 0.001) {
      for (size_t i = 0; i < joint_position_states_.size(); ++i) {
        double prev = joint_position_states_[i];
        double curr = joint_angles_rad[i];
        joint_velocity_states_[i] = (curr - prev) / period.seconds();
      }
    }
    // 更新位置状态
    joint_position_states_ = joint_angles_rad;
  } else {
    RCLCPP_WARN(node_->get_logger(), "read: joint_angles_rad size mismatch, expected %zu got %zu",
                joint_position_states_.size(), joint_angles_rad.size());
  }

  publishJointStates();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AR3HardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  if (!hardware_connected_ || !driver_) return hardware_interface::return_type::ERROR;

  enforceJointLimits();

  // 将关节指令转换为电机角度（度）
  std::vector<double> motor_commands_deg(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    double motor_rad = joint_position_commands_[i] * gear_ratios_[i];
    motor_commands_deg[i] = motor_rad * 180.0 / M_PI;
  }

  // 检查是否所有指令都接近 0（例如小于 0.1 度）
  bool all_near_zero = true;
  for (double deg : motor_commands_deg) {
    if (std::abs(deg) > 0.1) {
      all_near_zero = false;
      break;
    }
  }

  if (all_near_zero) {
    // 忽略全零命令，避免无用串口通信
    RCLCPP_DEBUG(node_->get_logger(), "Skipping near-zero motor commands");
    return hardware_interface::return_type::OK;
  }

  // 正常发送非零命令
  bool success = driver_->sendPositionCommands(motor_commands_deg);
  if (!success) {
    RCLCPP_ERROR(node_->get_logger(), "sendPositionCommands failed");
  }
  return success ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

void AR3HardwareInterface::enforceJointLimits()
{
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    if (joint_position_commands_[i] < position_lower_limits_[i]) {
      joint_position_commands_[i] = position_lower_limits_[i];
    }
    if (joint_position_commands_[i] > position_upper_limits_[i]) {
      joint_position_commands_[i] = position_upper_limits_[i];
    }
  }
}

void AR3HardwareInterface::publishJointStates()
{
  if (!joint_state_publisher_ || !node_) return;
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_->now();
  msg.name = joint_names_;
  msg.position = joint_position_states_;
  msg.velocity = joint_velocity_states_;
  msg.effort = joint_effort_states_;
  joint_state_publisher_->publish(msg);
}

} // namespace ar3_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    ar3_hardware_interface::AR3HardwareInterface,
    hardware_interface::SystemInterface
)