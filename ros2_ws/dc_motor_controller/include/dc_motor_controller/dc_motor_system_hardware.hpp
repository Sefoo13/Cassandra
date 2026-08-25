#ifndef DC_MOTOR_CONTROLLER__DC_MOTOR_SYSTEM_HARDWARE_HPP_
#define DC_MOTOR_CONTROLLER__DC_MOTOR_SYSTEM_HARDWARE_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace dc_motor_controller
{

class DCMotorSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DCMotorSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Motor
  {
    int forward_channel;
    int reverse_channel;
    double direction;
  };

  bool open_device();
  void close_device();
  bool initialize_pca9685();
  bool stop_all();
  bool set_motor(const Motor & motor, double normalized_command);
  bool set_channel_pwm(int channel, std::uint16_t duty_cycle);
  bool write_register(std::uint8_t register_address, std::uint8_t value);
  bool write_registers(
    std::uint8_t start_register, const std::uint8_t * values, std::size_t count);

  static bool parse_bool(const std::string & value);
  static int parse_integer(const std::string & value, const std::string & name);
  static double parse_double(const std::string & value, const std::string & name);

  std::vector<Motor> motors_;
  std::vector<double> velocity_commands_;
  std::vector<double> velocity_states_;
  std::vector<double> position_states_;

  std::string i2c_device_;
  int i2c_address_{0x40};
  int pwm_frequency_{1000};
  double min_pwm_percent_{8.0};
  double max_pwm_percent_{15.0};
  double max_wheel_speed_rad_s_{16.6666666667};
  double command_deadband_rad_s_{0.05};
  double linear_velocity_state_scale_{0.5};
  bool dry_run_{false};
  int file_descriptor_{-1};
};

}  // namespace dc_motor_controller

#endif  // DC_MOTOR_CONTROLLER__DC_MOTOR_SYSTEM_HARDWARE_HPP_
