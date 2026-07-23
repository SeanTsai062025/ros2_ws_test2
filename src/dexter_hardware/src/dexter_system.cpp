#include "dexter_hardware/dexter_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace dexter_hardware
{
namespace
{
using CallbackReturn = hardware_interface::CallbackReturn;
using ReturnType = hardware_interface::return_type;

template<typename T>
bool parse_unsigned_parameter(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name, T & output, const std::uint64_t maximum,
  const rclcpp::Logger & logger)
{
  const auto found = parameters.find(name);
  if (found == parameters.end())
  {
    return true;
  }
  try
  {
    const auto value = std::stoull(found->second);
    if (value > maximum)
    {
      throw std::out_of_range("value exceeds allowed range");
    }
    output = static_cast<T>(value);
    return true;
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(logger, "Invalid hardware parameter %s='%s': %s", name.c_str(),
      found->second.c_str(), exception.what());
    return false;
  }
}

bool parse_double_parameter(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name, double & output, const double minimum,
  const double maximum, const rclcpp::Logger & logger)
{
  const auto found = parameters.find(name);
  if (found == parameters.end())
  {
    return true;
  }
  try
  {
    const double value = std::stod(found->second);
    if (!std::isfinite(value) || value < minimum || value > maximum)
    {
      throw std::out_of_range("value exceeds allowed range");
    }
    output = value;
    return true;
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(logger, "Invalid hardware parameter %s='%s': %s", name.c_str(),
      found->second.c_str(), exception.what());
    return false;
  }
}

bool parse_bool(const std::string & value, bool & output)
{
  if (value == "true" || value == "1")
  {
    output = true;
    return true;
  }
  if (value == "false" || value == "0")
  {
    output = false;
    return true;
  }
  return false;
}

diagnostic_msgs::msg::KeyValue diagnostic_value(
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}
}  // namespace

DexterSystem::~DexterSystem()
{
  safe_stop("hardware plugin destruction", false);
  close_can();
}

CallbackReturn DexterSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  if (!validate_hardware_description() || !parse_parameters())
  {
    return CallbackReturn::ERROR;
  }

  const auto size = calibrations_.size();
  positions_.assign(size, std::numeric_limits<double>::quiet_NaN());
  velocities_.assign(size, std::numeric_limits<double>::quiet_NaN());
  last_accepted_commands_.assign(size, std::numeric_limits<double>::quiet_NaN());
  last_sent_ticks_.assign(size, std::nullopt);
  last_sent_speeds_.assign(size, std::nullopt);
  last_encoder_rx_.resize(size);
  have_encoder_sample_.assign(size, false);

  RCLCPP_INFO(
    get_logger(),
    "Dexter real hardware initialized: six physical encoder states, direct position+velocity "
    "JTC command forwarding, SocketCAN %s (interface bitrate must already be %u)",
    can_interface_.c_str(), can_bitrate_);
  return CallbackReturn::SUCCESS;
}

bool DexterSystem::validate_hardware_description()
{
  const auto defaults = default_motor_calibrations();
  const auto & joints = get_hardware_info().joints;
  if (joints.size() != defaults.size())
  {
    RCLCPP_ERROR(get_logger(), "Dexter hardware requires exactly six joints; URDF supplied %zu",
      joints.size());
    return false;
  }

  calibrations_.clear();
  for (const auto & joint : joints)
  {
    const auto calibration = std::find_if(
      defaults.begin(), defaults.end(), [&joint](const auto & candidate) {
        return candidate.joint_name == joint.name;
      });
    if (calibration == defaults.end())
    {
      RCLCPP_ERROR(get_logger(), "Unknown Dexter hardware joint '%s'", joint.name.c_str());
      return false;
    }
    const auto has_interface = [](const auto & interfaces, const std::string & name) {
      return std::count_if(interfaces.begin(), interfaces.end(), [&name](const auto & interface) {
        return interface.name == name;
      }) == 1;
    };
    if (
      joint.command_interfaces.size() != 2U || joint.state_interfaces.size() != 2U ||
      !has_interface(joint.command_interfaces, hardware_interface::HW_IF_POSITION) ||
      !has_interface(joint.command_interfaces, hardware_interface::HW_IF_VELOCITY) ||
      !has_interface(joint.state_interfaces, hardware_interface::HW_IF_POSITION) ||
      !has_interface(joint.state_interfaces, hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' must expose position+velocity command interfaces and position+velocity state "
        "interfaces", joint.name.c_str());
      return false;
    }
    calibrations_.push_back(*calibration);
  }
  return true;
}

bool DexterSystem::parse_parameters()
{
  const auto & parameters = get_hardware_info().hardware_parameters;
  if (const auto found = parameters.find("can_interface"); found != parameters.end())
  {
    can_interface_ = found->second;
  }

  std::uint64_t encoder_timeout_us =
    static_cast<std::uint64_t>(encoder_batch_timeout_.count());
  std::uint64_t request_spacing_us =
    static_cast<std::uint64_t>(encoder_request_spacing_.count());
  std::uint64_t quiet_us = static_cast<std::uint64_t>(startup_quiet_period_.count());
  std::uint64_t max_wait_us = static_cast<std::uint64_t>(startup_max_wait_.count());
  bool ok = true;
  ok &= parse_unsigned_parameter(parameters, "can_bitrate", can_bitrate_, 10000000U, get_logger());
  ok &= parse_unsigned_parameter(
    parameters, "encoder_request_window", encoder_request_window_, 6U, get_logger());
  ok &= parse_unsigned_parameter(
    parameters, "encoder_request_spacing_us", request_spacing_us, 2000U, get_logger());
  ok &= parse_unsigned_parameter(
    parameters, "encoder_timeout_us", encoder_timeout_us, 1000000U, get_logger());
  ok &= parse_unsigned_parameter(
    parameters, "startup_quiet_period_us", quiet_us, 1000000U, get_logger());
  ok &= parse_unsigned_parameter(
    parameters, "startup_max_wait_us", max_wait_us, 5000000U, get_logger());
  ok &= parse_unsigned_parameter(
    parameters, "max_speed_field", max_speed_field_, kProtocolMaxSpeed, get_logger());
  ok &= parse_unsigned_parameter(
    parameters, "min_speed_field", min_speed_field_, kProtocolMaxSpeed, get_logger());
  ok &= parse_unsigned_parameter(
    parameters, "fallback_speed_field", fallback_speed_field_, kProtocolMaxSpeed, get_logger());
  ok &= parse_unsigned_parameter(
    parameters, "part5_min_speed_field", part5_min_speed_field_, kProtocolMaxSpeed, get_logger());
  ok &= parse_unsigned_parameter(
    parameters, "part5_fallback_speed_field", part5_fallback_speed_field_,
    kProtocolMaxSpeed, get_logger());
  ok &= parse_unsigned_parameter(
    parameters, "acceleration_field", acceleration_field_, 255U, get_logger());
  ok &= parse_unsigned_parameter(
    parameters, "part5_acceleration_field", part5_acceleration_field_, 255U, get_logger());
  ok &= parse_unsigned_parameter(
    parameters, "part5_work_mode", part5_work_mode_, 255U, get_logger());
  ok &= parse_unsigned_parameter(
    parameters, "part5_working_current_ma", part5_working_current_ma_, 65535U, get_logger());
  ok &= parse_unsigned_parameter(
    parameters, "part5_subdivisions", part5_subdivisions_, 65535U, get_logger());
  ok &= parse_double_parameter(
    parameters, "velocity_filter_alpha", velocity_filter_alpha_, 0.0, 1.0, get_logger());
  ok &= parse_double_parameter(
    parameters, "max_command_step_rad", max_command_step_rad_, 0.001, 3.141592653589793,
    get_logger());

  if (const auto found = parameters.find("configure_part5_driver"); found != parameters.end())
  {
    if (!parse_bool(found->second, configure_part5_driver_))
    {
      RCLCPP_ERROR(get_logger(), "Invalid configure_part5_driver='%s'", found->second.c_str());
      ok = false;
    }
  }
  if (
    min_speed_field_ > max_speed_field_ || fallback_speed_field_ > max_speed_field_ ||
    part5_min_speed_field_ > max_speed_field_ ||
    part5_fallback_speed_field_ > max_speed_field_ || encoder_timeout_us == 0U ||
    encoder_request_window_ == 0U || quiet_us == 0U || max_wait_us < quiet_us)
  {
    RCLCPP_ERROR(get_logger(), "Inconsistent Dexter hardware speed/timing limits");
    ok = false;
  }
  encoder_batch_timeout_ = std::chrono::microseconds{encoder_timeout_us};
  encoder_request_spacing_ = std::chrono::microseconds{request_spacing_us};
  startup_quiet_period_ = std::chrono::microseconds{quiet_us};
  startup_max_wait_ = std::chrono::microseconds{max_wait_us};
  return ok;
}

bool DexterSystem::cache_interfaces()
{
  position_state_handles_.clear();
  velocity_state_handles_.clear();
  position_command_handles_.clear();
  velocity_command_handles_.clear();
  try
  {
    for (const auto & calibration : calibrations_)
    {
      position_state_handles_.push_back(get_state_interface_handle(
        calibration.joint_name + "/" + hardware_interface::HW_IF_POSITION));
      velocity_state_handles_.push_back(get_state_interface_handle(
        calibration.joint_name + "/" + hardware_interface::HW_IF_VELOCITY));
      position_command_handles_.push_back(get_command_interface_handle(
        calibration.joint_name + "/" + hardware_interface::HW_IF_POSITION));
      velocity_command_handles_.push_back(get_command_interface_handle(
        calibration.joint_name + "/" + hardware_interface::HW_IF_VELOCITY));
    }
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(get_logger(), "Failed to cache ros2_control interfaces: %s", exception.what());
    return false;
  }
  return true;
}

CallbackReturn DexterSystem::on_configure(const rclcpp_lifecycle::State &)
{
  close_can();
  cycle_guard_.reset();
  last_encoder_batch_ms_ = 0.0;
  max_encoder_batch_ms_ = 0.0;
  write_enabled_ = false;
  stop_sent_ = false;
  if (!cache_interfaces())
  {
    return CallbackReturn::ERROR;
  }
  try
  {
    can_client_ = std::make_unique<MksCanClient>(make_socketcan_transport(can_interface_));
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(get_logger(), "Failed to open SocketCAN: %s", exception.what());
    return CallbackReturn::ERROR;
  }

  diagnostics_publisher_ = get_node()->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::SystemDefaultsQoS());
  std::string error;
  if (!can_client_->resynchronize(startup_quiet_period_, startup_max_wait_, error))
  {
    RCLCPP_ERROR(get_logger(), "CAN startup synchronization failed: %s", error.c_str());
    publish_diagnostics(diagnostic_msgs::msg::DiagnosticStatus::ERROR, error, true);
    close_can();
    return CallbackReturn::ERROR;
  }
  if (configure_part5_driver_ && !configure_part5())
  {
    safe_stop("Part 5 configuration failed", true);
    close_can();
    return CallbackReturn::ERROR;
  }
  if (!read_all_encoders(true) || !seed_commands_from_measurements())
  {
    safe_stop("initial encoder acquisition failed", true);
    close_can();
    return CallbackReturn::ERROR;
  }
  publish_diagnostics(
    diagnostic_msgs::msg::DiagnosticStatus::OK,
    "all six encoders initialized; writes remain disabled", true);
  RCLCPP_INFO(get_logger(), "All six physical encoders initialized; hardware is safe/inactive");
  return CallbackReturn::SUCCESS;
}

bool DexterSystem::configure_part5()
{
  constexpr std::uint32_t motor_id = 6U;
  constexpr auto timeout = std::chrono::milliseconds{200};
  const auto verify = [this, motor_id, timeout](
    const std::uint8_t parameter, const std::uint32_t desired,
    const std::vector<std::uint8_t> & payload, const char * label) {
      std::string error;
      auto value = can_client_->read_parameter(motor_id, parameter, timeout, error);
      if (!value)
      {
        RCLCPP_ERROR(get_logger(), "Part 5 %s read failed: %s", label, error.c_str());
        return false;
      }
      if (*value != desired)
      {
        RCLCPP_INFO(get_logger(), "Part 5 %s: %u -> %u", label, *value, desired);
        if (!can_client_->set_parameter(motor_id, parameter, payload, timeout, error))
        {
          RCLCPP_ERROR(get_logger(), "Part 5 %s write failed: %s", label, error.c_str());
          return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{50});
        value = can_client_->read_parameter(motor_id, parameter, timeout, error);
      }
      if (!value || *value != desired)
      {
        RCLCPP_ERROR(get_logger(), "Part 5 %s verification failed", label);
        return false;
      }
      return true;
    };

  if (!verify(0x82U, part5_work_mode_, {part5_work_mode_}, "work mode (SR_CLOSE)"))
  {
    return false;
  }
  if (!verify(
      0x83U, part5_working_current_ma_,
      {static_cast<std::uint8_t>((part5_working_current_ma_ >> 8U) & 0xFFU),
        static_cast<std::uint8_t>(part5_working_current_ma_ & 0xFFU)},
      "working current"))
  {
    return false;
  }
  return verify(
    0x84U, part5_subdivisions_,
    {static_cast<std::uint8_t>(part5_subdivisions_ & 0xFFU)}, "subdivisions");
}

CallbackReturn DexterSystem::on_activate(const rclcpp_lifecycle::State &)
{
  // Re-read immediately before allowing commands. This closes the window where the arm
  // could have been moved by hand between configure and controller activation.
  if (!read_all_encoders(true) || !seed_commands_from_measurements())
  {
    safe_stop("activation encoder acquisition failed", true);
    return CallbackReturn::ERROR;
  }
  for (std::size_t index = 0; index < calibrations_.size(); ++index)
  {
    last_sent_ticks_[index] = radians_to_ticks(positions_[index], calibrations_[index]).ticks;
    const bool is_part5 = calibrations_[index].joint_name == "part5";
    last_sent_speeds_[index] = is_part5 ?
      part5_fallback_speed_field_ : fallback_speed_field_;
  }
  write_enabled_ = true;
  stop_sent_ = false;
  cycle_guard_.require_post_switch_read();
  publish_diagnostics(
    diagnostic_msgs::msg::DiagnosticStatus::OK,
    "active; commands seeded from measured encoder positions", true);
  RCLCPP_INFO(
    get_logger(),
    "Dexter hardware active; command positions were seeded from current physical encoders");
  return CallbackReturn::SUCCESS;
}

bool DexterSystem::seed_commands_from_measurements()
{
  const auto seed = make_activation_command_seed(positions_);
  for (std::size_t index = 0; index < calibrations_.size(); ++index)
  {
    if (!std::isfinite(seed.positions[index]))
    {
      return false;
    }
    set_command(position_command_handles_[index], seed.positions[index], true);
    set_command(velocity_command_handles_[index], seed.velocities[index], true);
    last_accepted_commands_[index] = seed.positions[index];
    last_sent_ticks_[index] = radians_to_ticks(seed.positions[index], calibrations_[index]).ticks;
    const bool is_part5 = calibrations_[index].joint_name == "part5";
    last_sent_speeds_[index] = is_part5 ?
      part5_fallback_speed_field_ : fallback_speed_field_;
  }
  return true;
}

bool DexterSystem::read_all_encoders(const bool initializing)
{
  if (!can_client_ || !can_client_->synchronized())
  {
    RCLCPP_ERROR(get_logger(), "Refusing encoder read: CAN transaction stream is unsynchronized");
    return false;
  }

  std::vector<std::uint32_t> motor_ids;
  motor_ids.reserve(calibrations_.size());
  for (const auto & calibration : calibrations_)
  {
    motor_ids.push_back(calibration.can_id);
  }

  std::string error;
  const auto batch_start = std::chrono::steady_clock::now();
  const auto samples = can_client_->read_encoders(
    motor_ids, encoder_batch_timeout_, error, encoder_request_window_, encoder_request_spacing_);
  last_encoder_batch_ms_ = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - batch_start).count();
  max_encoder_batch_ms_ = std::max(max_encoder_batch_ms_, last_encoder_batch_ms_);
  if (!samples)
  {
    ++failed_read_cycles_;
    RCLCPP_ERROR(get_logger(), "Fresh six-axis encoder batch failed: %s", error.c_str());
    publish_diagnostics(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "fresh six-axis encoder batch failed: " + error, true);
    return false;
  }

  std::vector<double> candidate_positions(calibrations_.size());
  std::vector<std::chrono::steady_clock::time_point> receive_times(calibrations_.size());
  for (std::size_t index = 0; index < calibrations_.size(); ++index)
  {
    const auto & sample = (*samples)[index];
    receive_times[index] = sample.received_at;
    candidate_positions[index] = ticks_to_radians(sample.ticks, calibrations_[index]);
  }

  for (std::size_t index = 0; index < calibrations_.size(); ++index)
  {
    double velocity = 0.0;
    if (!initializing && have_encoder_sample_[index])
    {
      const double dt = std::chrono::duration<double>(
        receive_times[index] - last_encoder_rx_[index]).count();
      if (dt <= 0.0 || !std::isfinite(dt))
      {
        RCLCPP_ERROR(get_logger(), "Non-monotonic encoder timestamp for %s",
          calibrations_[index].joint_name.c_str());
        return false;
      }
      const double measured_velocity = (candidate_positions[index] - positions_[index]) / dt;
      velocity = velocity_filter_alpha_ * measured_velocity +
        (1.0 - velocity_filter_alpha_) * velocities_[index];
    }
    positions_[index] = candidate_positions[index];
    velocities_[index] = velocity;
    last_encoder_rx_[index] = receive_times[index];
    have_encoder_sample_[index] = true;
  }

  for (std::size_t index = 0; index < calibrations_.size(); ++index)
  {
    if (
      !set_state(position_state_handles_[index], positions_[index], true) ||
      !set_state(velocity_state_handles_[index], velocities_[index], true))
    {
      RCLCPP_ERROR(get_logger(), "Failed to commit encoder state interfaces");
      return false;
    }
  }
  ++successful_read_cycles_;
  cycle_guard_.record_read();
  publish_diagnostics(
    diagnostic_msgs::msg::DiagnosticStatus::OK, "all encoder feedback fresh", false);
  return true;
}

ReturnType DexterSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!read_all_encoders(false))
  {
    invalidate_exported_states();
    safe_stop("encoder feedback lost", true);
    return ReturnType::ERROR;
  }
  return ReturnType::OK;
}

ReturnType DexterSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!write_enabled_)
  {
    return ReturnType::OK;
  }
  if (!can_client_ || !can_client_->synchronized())
  {
    safe_stop("write attempted while CAN transaction stream was unsynchronized", true);
    return ReturnType::ERROR;
  }

  const auto write_decision = cycle_guard_.evaluate_write();
  if (write_decision == WriteCycleDecision::WAITING_FOR_READ)
  {
    // Controller manager may call write() in the same update cycle in which it
    // activates JTC. Suppress that expected call so no target is sent until a
    // complete encoder batch has arrived after the mode switch.
    return ReturnType::OK;
  }
  if (write_decision == WriteCycleDecision::STALE)
  {
    safe_stop("write attempted without a fresh encoder cycle", true);
    return ReturnType::ERROR;
  }

  std::vector<double> position_commands(calibrations_.size());
  std::vector<double> velocity_commands(calibrations_.size());
  for (std::size_t index = 0; index < calibrations_.size(); ++index)
  {
    if (
      !get_command(position_command_handles_[index], position_commands[index], false) ||
      !get_command(velocity_command_handles_[index], velocity_commands[index], false) ||
      !std::isfinite(position_commands[index]) || !std::isfinite(velocity_commands[index]))
    {
      safe_stop("non-finite or unavailable JTC command", true);
      return ReturnType::ERROR;
    }
    if (
      std::isfinite(last_accepted_commands_[index]) &&
      std::abs(position_commands[index] - last_accepted_commands_[index]) > max_command_step_rad_)
    {
      RCLCPP_ERROR(
        get_logger(), "Command step rejected for %s: %.6f -> %.6f rad exceeds %.6f rad",
        calibrations_[index].joint_name.c_str(), last_accepted_commands_[index],
        position_commands[index], max_command_step_rad_);
      safe_stop("position command step limit exceeded", true);
      return ReturnType::ERROR;
    }
  }

  for (std::size_t index = 0; index < calibrations_.size(); ++index)
  {
    const auto target = radians_to_ticks(position_commands[index], calibrations_[index]);
    if (target.clamped)
    {
      RCLCPP_ERROR(
        get_logger(), "0xF5 axis target rejected for %s: %.6f rad exceeds signed 24-bit limit",
        calibrations_[index].joint_name.c_str(), position_commands[index]);
      safe_stop("0xF5 target limit exceeded", true);
      return ReturnType::ERROR;
    }
    last_accepted_commands_[index] = position_commands[index];

    const bool is_part5 = calibrations_[index].joint_name == "part5";
    const auto speed = velocity_to_speed_field(
      velocity_commands[index], calibrations_[index],
      is_part5 ? part5_min_speed_field_ : min_speed_field_,
      is_part5 ? part5_fallback_speed_field_ : fallback_speed_field_, max_speed_field_);
    const auto acceleration = is_part5 ? part5_acceleration_field_ : acceleration_field_;
    // An absolute F5 target remains active inside the driver. Retransmit for a changed
    // target or speed profile while moving, but never repeat an unchanged idle command.
    if (!absolute_command_changed(
        last_sent_ticks_[index], last_sent_speeds_[index], target.ticks, speed))
    {
      continue;
    }
    std::string error;
    if (!can_client_->send_absolute(
        calibrations_[index].can_id, speed, acceleration, target.ticks, error))
    {
      RCLCPP_ERROR(get_logger(), "0xF5 send failed for %s: %s",
        calibrations_[index].joint_name.c_str(), error.c_str());
      safe_stop("CAN command write failed", true);
      return ReturnType::ERROR;
    }
    last_sent_ticks_[index] = target.ticks;
    last_sent_speeds_[index] = speed;
  }
  cycle_guard_.record_write();
  return ReturnType::OK;
}

void DexterSystem::invalidate_exported_states() noexcept
{
  const double unavailable = std::numeric_limits<double>::quiet_NaN();
  for (std::size_t index = 0; index < position_state_handles_.size(); ++index)
  {
    try
    {
      std::ignore = set_state(position_state_handles_[index], unavailable, false);
      std::ignore = set_state(velocity_state_handles_[index], unavailable, false);
    }
    catch (const std::exception & exception)
    {
      RCLCPP_ERROR(get_logger(), "Failed to invalidate stale state: %s", exception.what());
    }
  }
}

void DexterSystem::safe_stop(const std::string & reason, const bool fault) noexcept
{
  write_enabled_ = false;
  if (!can_client_ || stop_sent_)
  {
    return;
  }
  if (fault)
  {
    RCLCPP_ERROR(get_logger(), "Dexter fault stop: %s", reason.c_str());
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Dexter controlled stop: %s", reason.c_str());
  }
  for (const auto & calibration : calibrations_)
  {
    std::string error;
    if (!can_client_->send_stop(calibration.can_id, error))
    {
      RCLCPP_ERROR(get_logger(), "Safe-stop send failed for %s: %s",
        calibration.joint_name.c_str(), error.c_str());
    }
  }
  stop_sent_ = true;
  publish_diagnostics(
    fault ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
    diagnostic_msgs::msg::DiagnosticStatus::WARN,
    reason, true);
}

ReturnType DexterSystem::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  const auto is_ours = [this](const std::string & interface_name) {
      return std::any_of(calibrations_.begin(), calibrations_.end(), [&interface_name](const auto & motor) {
        return interface_name == motor.joint_name + "/" + hardware_interface::HW_IF_POSITION ||
               interface_name == motor.joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
      });
    };
  if (
    std::any_of(start_interfaces.begin(), start_interfaces.end(), is_ours))
  {
    for (const auto & motor : calibrations_)
    {
      const auto has_position = std::find(
        start_interfaces.begin(), start_interfaces.end(),
        motor.joint_name + "/" + hardware_interface::HW_IF_POSITION) != start_interfaces.end();
      const auto has_velocity = std::find(
        start_interfaces.begin(), start_interfaces.end(),
        motor.joint_name + "/" + hardware_interface::HW_IF_VELOCITY) != start_interfaces.end();
      if (!has_position || !has_velocity)
      {
        RCLCPP_ERROR(
          get_logger(), "Dexter only accepts all six position+velocity command pairs");
        return ReturnType::ERROR;
      }
    }
  }
  (void)stop_interfaces;
  return ReturnType::OK;
}

ReturnType DexterSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  const auto is_ours = [this](const std::string & interface_name) {
      return std::any_of(calibrations_.begin(), calibrations_.end(), [&interface_name](const auto & motor) {
        return interface_name == motor.joint_name + "/" + hardware_interface::HW_IF_POSITION ||
               interface_name == motor.joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
      });
    };
  if (std::any_of(stop_interfaces.begin(), stop_interfaces.end(), is_ours))
  {
    safe_stop("motion command interfaces released", false);
  }
  if (std::any_of(start_interfaces.begin(), start_interfaces.end(), is_ours))
  {
    if (!seed_commands_from_measurements())
    {
      safe_stop("could not seed commands during controller activation", true);
      return ReturnType::ERROR;
    }
    write_enabled_ = true;
    stop_sent_ = false;
    cycle_guard_.require_post_switch_read();
  }
  return ReturnType::OK;
}

void DexterSystem::close_can() noexcept
{
  if (can_client_)
  {
    can_client_->close();
    can_client_.reset();
  }
}

CallbackReturn DexterSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  safe_stop("hardware deactivated", false);
  return CallbackReturn::SUCCESS;
}

CallbackReturn DexterSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  safe_stop("hardware cleanup", false);
  close_can();
  return CallbackReturn::SUCCESS;
}

CallbackReturn DexterSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  safe_stop("hardware shutdown", false);
  close_can();
  return CallbackReturn::SUCCESS;
}

CallbackReturn DexterSystem::on_error(const rclcpp_lifecycle::State &)
{
  safe_stop("hardware lifecycle error", true);
  close_can();
  return CallbackReturn::SUCCESS;
}

void DexterSystem::publish_diagnostics(
  const std::uint8_t level, const std::string & message, const bool force)
{
  if (!diagnostics_publisher_)
  {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  if (
    !force && last_diagnostic_publish_.time_since_epoch().count() != 0 &&
    now - last_diagnostic_publish_ < std::chrono::seconds{1})
  {
    return;
  }
  last_diagnostic_publish_ = now;

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = get_clock()->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = level;
  status.name = "Dexter ros2_control hardware";
  status.hardware_id = can_interface_;
  status.message = message;
  status.values.push_back(diagnostic_value(
    "successful_read_cycles", std::to_string(successful_read_cycles_)));
  status.values.push_back(diagnostic_value(
    "failed_read_cycles", std::to_string(failed_read_cycles_)));
  status.values.push_back(diagnostic_value(
    "writes_enabled", write_enabled_ ? "true" : "false"));
  status.values.push_back(diagnostic_value(
    "awaiting_post_switch_read",
    cycle_guard_.awaiting_post_switch_read() ? "true" : "false"));
  status.values.push_back(diagnostic_value(
    "read_generation", std::to_string(cycle_guard_.read_generation())));
  status.values.push_back(diagnostic_value(
    "write_generation", std::to_string(cycle_guard_.write_generation())));
  status.values.push_back(diagnostic_value(
    "last_encoder_batch_ms", std::to_string(last_encoder_batch_ms_)));
  status.values.push_back(diagnostic_value(
    "max_encoder_batch_ms", std::to_string(max_encoder_batch_ms_)));
  if (can_client_)
  {
    const auto & counters = can_client_->counters();
    status.values.push_back(diagnostic_value("rx_f5_status_ignored", std::to_string(counters.f5_status)));
    status.values.push_back(diagnostic_value("rx_unrelated_ignored", std::to_string(counters.unrelated)));
    status.values.push_back(diagnostic_value("rx_bad_checksum", std::to_string(counters.bad_checksum)));
    status.values.push_back(diagnostic_value(
      "rx_malformed_encoder", std::to_string(counters.malformed_encoder)));
    status.values.push_back(diagnostic_value(
      "rx_duplicate_encoder", std::to_string(counters.duplicate_encoder)));
    status.values.push_back(diagnostic_value(
      "rx_post_deadline_drained", std::to_string(counters.post_deadline_drained)));
  }
  for (std::size_t index = 0; index < calibrations_.size(); ++index)
  {
    const std::string age = have_encoder_sample_[index] ?
      std::to_string(std::chrono::duration<double, std::milli>(
        now - last_encoder_rx_[index]).count()) : "never";
    status.values.push_back(diagnostic_value(
      calibrations_[index].joint_name + ".encoder_age_ms", age));
  }
  array.status.push_back(std::move(status));
  diagnostics_publisher_->publish(array);
}

}  // namespace dexter_hardware

PLUGINLIB_EXPORT_CLASS(dexter_hardware::DexterSystem, hardware_interface::SystemInterface)
