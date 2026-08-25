#include "dc_motor_controller/dc_motor_system_hardware.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <stdexcept>
#include <thread>
#include <unistd.h>

#include <sys/ioctl.h>

#ifdef __linux__
#include <linux/i2c-dev.h>
#endif

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

constexpr std::uint8_t kMode1Register = 0x00;
constexpr std::uint8_t kMode2Register = 0x01;
constexpr std::uint8_t kLed0OnLowRegister = 0x06;
constexpr std::uint8_t kPrescaleRegister = 0xFE;
constexpr std::uint8_t kMode1AutoIncrement = 0x20;
constexpr std::uint8_t kMode1Sleep = 0x10;
constexpr std::uint8_t kMode1Restart = 0x80;
constexpr std::uint8_t kMode2OutputDriver = 0x04;
constexpr double kPca9685ClockHz = 25000000.0;
constexpr double kPwmResolution = 4096.0;
constexpr int kChannelCount = 16;

const auto kLogger = rclcpp::get_logger("dc_motor_controller");

}  // namespace

namespace dc_motor_controller
{

hardware_interface::CallbackReturn DCMotorSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    const auto get_hardware_parameter =
      [this](const std::string & name) -> const std::string & {
        const auto parameter = info_.hardware_parameters.find(name);
        if (parameter == info_.hardware_parameters.end()) {
          throw std::invalid_argument("Missing hardware parameter '" + name + "'");
        }
        return parameter->second;
      };

    i2c_device_ = get_hardware_parameter("i2c_device");
    i2c_address_ = parse_integer(get_hardware_parameter("i2c_address"), "i2c_address");
    pwm_frequency_ = parse_integer(
      get_hardware_parameter("pwm_frequency"), "pwm_frequency");
    min_pwm_percent_ = parse_double(
      get_hardware_parameter("min_pwm_percent"), "min_pwm_percent");
    max_pwm_percent_ = parse_double(
      get_hardware_parameter("max_pwm_percent"), "max_pwm_percent");
    max_wheel_speed_rad_s_ = parse_double(
      get_hardware_parameter("max_wheel_speed_rad_s"), "max_wheel_speed_rad_s");
    command_deadband_rad_s_ = parse_double(
      get_hardware_parameter("command_deadband_rad_s"), "command_deadband_rad_s");
    linear_velocity_state_scale_ = parse_double(
      get_hardware_parameter("linear_velocity_state_scale"),
      "linear_velocity_state_scale");
    dry_run_ = parse_bool(get_hardware_parameter("dry_run"));

    if (i2c_address_ < 0x03 || i2c_address_ > 0x77) {
      throw std::invalid_argument("i2c_address must be between 0x03 and 0x77");
    }
    if (pwm_frequency_ < 24 || pwm_frequency_ > 1526) {
      throw std::invalid_argument("pwm_frequency must be between 24 and 1526 Hz");
    }
    if (min_pwm_percent_ < 0.0 || min_pwm_percent_ > 100.0) {
      throw std::invalid_argument("min_pwm_percent must be between 0 and 100");
    }
    if (max_pwm_percent_ < min_pwm_percent_ || max_pwm_percent_ > 100.0) {
      throw std::invalid_argument(
              "max_pwm_percent must be between min_pwm_percent and 100");
    }
    if (!std::isfinite(max_wheel_speed_rad_s_) || max_wheel_speed_rad_s_ <= 0.0) {
      throw std::invalid_argument("max_wheel_speed_rad_s must be positive");
    }
    if (command_deadband_rad_s_ < 0.0 ||
      command_deadband_rad_s_ >= max_wheel_speed_rad_s_)
    {
      throw std::invalid_argument(
              "command_deadband_rad_s must be non-negative and below max wheel speed");
    }
    if (linear_velocity_state_scale_ <= 0.0) {
      throw std::invalid_argument("linear_velocity_state_scale must be positive");
    }

    motors_.clear();
    motors_.reserve(info_.joints.size());
    std::array<bool, kChannelCount> used_channels{};

    for (const auto & joint : info_.joints) {
      if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        throw std::invalid_argument(
                "Joint '" + joint.name + "' must have exactly one velocity command interface");
      }
      if (joint.state_interfaces.size() != 2) {
        throw std::invalid_argument(
                "Joint '" + joint.name + "' must have position and velocity state interfaces");
      }

      bool has_position = false;
      bool has_velocity = false;
      for (const auto & interface : joint.state_interfaces) {
        has_position |= interface.name == hardware_interface::HW_IF_POSITION;
        has_velocity |= interface.name == hardware_interface::HW_IF_VELOCITY;
      }
      if (!has_position || !has_velocity) {
        throw std::invalid_argument(
                "Joint '" + joint.name + "' must expose position and velocity state interfaces");
      }

      const auto get_joint_parameter =
        [&joint](const std::string & name) -> const std::string & {
          const auto parameter = joint.parameters.find(name);
          if (parameter == joint.parameters.end()) {
            throw std::invalid_argument(
                    "Joint '" + joint.name + "' is missing parameter '" + name + "'");
          }
          return parameter->second;
        };

      const int forward_channel = parse_integer(
        get_joint_parameter("forward_channel"), joint.name + ".forward_channel");
      const int reverse_channel = parse_integer(
        get_joint_parameter("reverse_channel"), joint.name + ".reverse_channel");
      const double direction = parse_double(
        get_joint_parameter("direction"), joint.name + ".direction");

      if (forward_channel < 0 || forward_channel >= kChannelCount ||
        reverse_channel < 0 || reverse_channel >= kChannelCount)
      {
        throw std::invalid_argument("PCA9685 channel numbers must be between 0 and 15");
      }
      if (forward_channel == reverse_channel ||
        used_channels[forward_channel] || used_channels[reverse_channel])
      {
        throw std::invalid_argument("PCA9685 motor channels must be unique");
      }
      if (direction != -1.0 && direction != 1.0) {
        throw std::invalid_argument(
                "Joint '" + joint.name + "' direction must be -1 or 1");
      }

      used_channels[forward_channel] = true;
      used_channels[reverse_channel] = true;
      motors_.push_back({forward_channel, reverse_channel, direction});
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(kLogger, "Invalid hardware configuration: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const double unset = std::numeric_limits<double>::quiet_NaN();
  velocity_commands_.assign(info_.joints.size(), unset);
  velocity_states_.assign(info_.joints.size(), unset);
  position_states_.assign(info_.joints.size(), unset);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DCMotorSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(info_.joints.size() * 2);
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_POSITION,
      &position_states_[index]);
    interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_VELOCITY,
      &velocity_states_[index]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
DCMotorSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(info_.joints.size());
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_VELOCITY,
      &velocity_commands_[index]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn DCMotorSystemHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!open_device()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!initialize_pca9685() || !stop_all()) {
    stop_all();
    close_device();
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
  std::fill(velocity_states_.begin(), velocity_states_.end(), 0.0);
  std::fill(position_states_.begin(), position_states_.end(), 0.0);
  RCLCPP_INFO(
    kLogger, "Configured PCA9685 motor hardware%s",
    dry_run_ ? " in dry-run mode" : "");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DCMotorSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  const bool stopped = stop_all();
  close_device();
  return stopped ?
         hardware_interface::CallbackReturn::SUCCESS :
         hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn DCMotorSystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
  std::fill(velocity_states_.begin(), velocity_states_.end(), 0.0);
  if (!stop_all()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(kLogger, "Activated DC motor hardware");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DCMotorSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
  std::fill(velocity_states_.begin(), velocity_states_.end(), 0.0);
  if (!stop_all()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(kLogger, "Deactivated DC motor hardware");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DCMotorSystemHardware::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  const bool stopped = stop_all();
  close_device();
  return stopped ?
         hardware_interface::CallbackReturn::SUCCESS :
         hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn DCMotorSystemHardware::on_error(
  const rclcpp_lifecycle::State &)
{
  const bool stopped = stop_all();
  close_device();
  return stopped ?
         hardware_interface::CallbackReturn::SUCCESS :
         hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::return_type DCMotorSystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  const double seconds = period.seconds();
  if (!std::isfinite(seconds) || seconds < 0.0) {
    return hardware_interface::return_type::ERROR;
  }

  // Cassandra currently has no wheel encoders. Scale only the common wheel
  // component (forward/backward motion), preserving the differential component
  // used for turning.
  double linear_component = 0.0;
  for (const double command : velocity_commands_) {
    linear_component += command;
  }
  if (!velocity_commands_.empty()) {
    linear_component /= static_cast<double>(velocity_commands_.size());
  }
  const double linear_correction =
    linear_component * (1.0 - linear_velocity_state_scale_);

  for (std::size_t index = 0; index < velocity_states_.size(); ++index) {
    velocity_states_[index] = velocity_commands_[index] - linear_correction;
    position_states_[index] += velocity_states_[index] * seconds;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DCMotorSystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  for (std::size_t index = 0; index < motors_.size(); ++index) {
    const double command = velocity_commands_[index];
    if (!std::isfinite(command)) {
      RCLCPP_ERROR(
        kLogger, "Non-finite velocity command for joint %s",
        info_.joints[index].name.c_str());
      stop_all();
      return hardware_interface::return_type::ERROR;
    }
    const double filtered_command =
      std::abs(command) <= command_deadband_rad_s_ ? 0.0 : command;
    if (!set_motor(motors_[index], filtered_command / max_wheel_speed_rad_s_)) {
      stop_all();
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

bool DCMotorSystemHardware::open_device()
{
  if (dry_run_) {
    return true;
  }
#ifndef __linux__
  RCLCPP_ERROR(kLogger, "Real I2C access is supported only on Linux; use dry_run=true");
  return false;
#else
  file_descriptor_ = ::open(i2c_device_.c_str(), O_RDWR);
  if (file_descriptor_ < 0) {
    RCLCPP_ERROR(
      kLogger, "Cannot open %s: %s", i2c_device_.c_str(), std::strerror(errno));
    return false;
  }
  if (::ioctl(file_descriptor_, I2C_SLAVE, i2c_address_) < 0) {
    RCLCPP_ERROR(
      kLogger, "Cannot select I2C address 0x%02X: %s",
      i2c_address_, std::strerror(errno));
    close_device();
    return false;
  }
  return true;
#endif
}

void DCMotorSystemHardware::close_device()
{
  if (file_descriptor_ >= 0) {
    ::close(file_descriptor_);
    file_descriptor_ = -1;
  }
}

bool DCMotorSystemHardware::initialize_pca9685()
{
  if (dry_run_) {
    return true;
  }

  const int prescale = std::clamp(
    static_cast<int>(std::lround(kPca9685ClockHz / (kPwmResolution * pwm_frequency_) - 1.0)),
    3, 255);
  if (!write_register(kMode1Register, kMode1Sleep | kMode1AutoIncrement) ||
    !write_register(kPrescaleRegister, static_cast<std::uint8_t>(prescale)) ||
    !write_register(kMode2Register, kMode2OutputDriver) ||
    !write_register(kMode1Register, kMode1AutoIncrement))
  {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  return write_register(kMode1Register, kMode1Restart | kMode1AutoIncrement);
}

bool DCMotorSystemHardware::stop_all()
{
  if (dry_run_ || file_descriptor_ < 0) {
    return true;
  }
  bool success = true;
  for (const auto & motor : motors_) {
    success = set_channel_pwm(motor.forward_channel, 0) && success;
    success = set_channel_pwm(motor.reverse_channel, 0) && success;
  }
  return success;
}

bool DCMotorSystemHardware::set_motor(const Motor & motor, double normalized_command)
{
  const double limited = std::clamp(normalized_command * motor.direction, -1.0, 1.0);
  const double magnitude = std::abs(limited);
  const double pwm_percent = magnitude == 0.0 ?
    0.0 :
    min_pwm_percent_ + magnitude * (max_pwm_percent_ - min_pwm_percent_);
  const double scaled = pwm_percent / 100.0;
  const auto duty_cycle = static_cast<std::uint16_t>(
    std::lround(std::clamp(scaled, 0.0, 1.0) * 4095.0));

  if (limited >= 0.0) {
    return set_channel_pwm(motor.reverse_channel, 0) &&
           set_channel_pwm(motor.forward_channel, duty_cycle);
  }
  return set_channel_pwm(motor.forward_channel, 0) &&
         set_channel_pwm(motor.reverse_channel, duty_cycle);
}

bool DCMotorSystemHardware::set_channel_pwm(int channel, std::uint16_t duty_cycle)
{
  if (dry_run_) {
    return true;
  }
  const std::uint16_t limited = std::min<std::uint16_t>(duty_cycle, 4095);
  const std::uint8_t values[] = {
    0,
    0,
    static_cast<std::uint8_t>(limited & 0xFF),
    static_cast<std::uint8_t>((limited >> 8) & 0x0F),
  };
  return write_registers(
    static_cast<std::uint8_t>(kLed0OnLowRegister + 4 * channel), values, sizeof(values));
}

bool DCMotorSystemHardware::write_register(
  std::uint8_t register_address, std::uint8_t value)
{
  return write_registers(register_address, &value, 1);
}

bool DCMotorSystemHardware::write_registers(
  std::uint8_t start_register, const std::uint8_t * values, std::size_t count)
{
  if (dry_run_) {
    return true;
  }
  if (file_descriptor_ < 0 || count > 4) {
    return false;
  }

  std::array<std::uint8_t, 5> buffer{};
  buffer[0] = start_register;
  std::copy(values, values + count, buffer.begin() + 1);
  const auto expected = static_cast<ssize_t>(count + 1);
  if (::write(file_descriptor_, buffer.data(), count + 1) != expected) {
    RCLCPP_ERROR(
      kLogger, "I2C write to register 0x%02X failed: %s",
      start_register, std::strerror(errno));
    return false;
  }
  return true;
}

bool DCMotorSystemHardware::parse_bool(const std::string & value)
{
  if (value == "true" || value == "1") {
    return true;
  }
  if (value == "false" || value == "0") {
    return false;
  }
  throw std::invalid_argument("Boolean parameter must be true/false or 1/0");
}

int DCMotorSystemHardware::parse_integer(
  const std::string & value, const std::string & name)
{
  std::size_t parsed_characters = 0;
  const int result = std::stoi(value, &parsed_characters, 0);
  if (parsed_characters != value.size()) {
    throw std::invalid_argument(name + " is not a valid integer");
  }
  return result;
}

double DCMotorSystemHardware::parse_double(
  const std::string & value, const std::string & name)
{
  std::size_t parsed_characters = 0;
  const double result = std::stod(value, &parsed_characters);
  if (parsed_characters != value.size() || !std::isfinite(result)) {
    throw std::invalid_argument(name + " is not a finite number");
  }
  return result;
}

}  // namespace dc_motor_controller

PLUGINLIB_EXPORT_CLASS(
  dc_motor_controller::DCMotorSystemHardware,
  hardware_interface::SystemInterface)
