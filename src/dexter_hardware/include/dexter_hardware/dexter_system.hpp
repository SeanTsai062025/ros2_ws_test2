#ifndef DEXTER_HARDWARE__DEXTER_SYSTEM_HPP_
#define DEXTER_HARDWARE__DEXTER_SYSTEM_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "dexter_hardware/control_cycle_guard.hpp"
#include "dexter_hardware/mks_can_client.hpp"
#include "dexter_hardware/mks_protocol.hpp"

namespace dexter_hardware
{

class DexterSystem final : public hardware_interface::SystemInterface
{
public:
  DexterSystem() = default;
  ~DexterSystem() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

private:
  bool validate_hardware_description();
  bool parse_parameters();
  bool cache_interfaces();
  bool configure_part5();
  bool read_all_encoders(bool initializing);
  bool seed_commands_from_measurements();
  void invalidate_exported_states() noexcept;
  void safe_stop(const std::string & reason, bool fault) noexcept;
  void close_can() noexcept;
  void publish_diagnostics(std::uint8_t level, const std::string & message, bool force);

  std::string can_interface_{"can0"};
  std::uint32_t can_bitrate_{1000000U};
  std::chrono::microseconds encoder_batch_timeout_{7000};
  std::chrono::microseconds startup_quiet_period_{20000};
  std::chrono::microseconds startup_max_wait_{250000};
  std::uint16_t max_speed_field_{3000U};
  std::uint16_t min_speed_field_{10U};
  std::uint16_t fallback_speed_field_{300U};
  std::uint16_t part5_min_speed_field_{1U};
  std::uint16_t part5_fallback_speed_field_{1U};
  std::uint8_t acceleration_field_{0U};
  std::uint8_t part5_acceleration_field_{0U};
  double velocity_filter_alpha_{0.25};
  double max_command_step_rad_{0.05};
  bool configure_part5_driver_{true};

  std::uint8_t part5_work_mode_{4U};
  std::uint16_t part5_working_current_ma_{800U};
  std::uint16_t part5_subdivisions_{128U};

  std::vector<MotorCalibration> calibrations_;
  std::vector<hardware_interface::StateInterface::SharedPtr> position_state_handles_;
  std::vector<hardware_interface::StateInterface::SharedPtr> velocity_state_handles_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> position_command_handles_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> velocity_command_handles_;

  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> last_accepted_commands_;
  std::vector<std::optional<std::int32_t>> last_sent_ticks_;
  std::vector<std::optional<std::uint16_t>> last_sent_speeds_;
  std::vector<std::chrono::steady_clock::time_point> last_encoder_rx_;
  std::vector<bool> have_encoder_sample_;

  std::unique_ptr<MksCanClient> can_client_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  std::chrono::steady_clock::time_point last_diagnostic_publish_{};
  std::size_t successful_read_cycles_{0U};
  std::size_t failed_read_cycles_{0U};
  double last_encoder_batch_ms_{0.0};
  double max_encoder_batch_ms_{0.0};
  ControlCycleGuard cycle_guard_;
  bool write_enabled_{false};
  bool stop_sent_{false};
};

}  // namespace dexter_hardware

#endif  // DEXTER_HARDWARE__DEXTER_SYSTEM_HPP_
