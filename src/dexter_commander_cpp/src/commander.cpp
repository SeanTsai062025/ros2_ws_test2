#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <dexter_interfaces/msg/command_result.hpp>
#include <dexter_interfaces/msg/device_command.hpp>
#include <dexter_interfaces/msg/joint_command.hpp>
#include <dexter_interfaces/msg/pose_command.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <sstream>
#include <string>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using CommandResult = dexter_interfaces::msg::CommandResult;
using DeviceCommand = dexter_interfaces::msg::DeviceCommand;
using JointCmd = dexter_interfaces::msg::JointCommand;
using PoseCmd = dexter_interfaces::msg::PoseCommand;

using namespace std::placeholders;

namespace
{

constexpr double kMinSpeedScaling = 0.2;
constexpr double kMaxSpeedScaling = 1.0;
constexpr double kDefaultSpeedScaling = 0.5;
constexpr double kLegacySpeedScaling = kDefaultSpeedScaling;

struct ExecutionOutcome
{
  std::string status;
  std::string detail;
};

class BusyGuard
{
public:
  explicit BusyGuard(std::atomic_bool & busy) : busy_(busy) {}
  ~BusyGuard() {busy_.store(false);}

private:
  std::atomic_bool & busy_;
};

}  // namespace

class Commander
{
public:
  explicit Commander(std::shared_ptr<rclcpp::Node> node)
  : node_(std::move(node))
  {
    arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
    arm_->setMaxVelocityScalingFactor(kDefaultSpeedScaling);
    arm_->setMaxAccelerationScalingFactor(kDefaultSpeedScaling);
    arm_->setPlanningTime(10.0);
    arm_->setNumPlanningAttempts(5);

    command_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    cancel_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions command_options;
    command_options.callback_group = command_group_;
    rclcpp::SubscriptionOptions cancel_options;
    cancel_options.callback_group = cancel_group_;

    joint_cmd_sub_ = node_->create_subscription<JointCmd>(
      "joint_command", 10, std::bind(&Commander::jointCmdCallback, this, _1),
      command_options);
    legacy_joint_cmd_sub_ = node_->create_subscription<FloatArray>(
      "joint_command_legacy", 10,
      std::bind(&Commander::legacyJointCmdCallback, this, _1), command_options);
    pose_cmd_sub_ = node_->create_subscription<PoseCmd>(
      "pose_command", 10, std::bind(&Commander::poseCmdCallback, this, _1),
      command_options);
    cancel_cmd_sub_ = node_->create_subscription<DeviceCommand>(
      "arm_cancel", 10, std::bind(&Commander::cancelCmdCallback, this, _1),
      cancel_options);
    emergency_stop_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
      "arm_stop", 10, std::bind(&Commander::emergencyStopCallback, this, _1),
      cancel_options);

    result_pub_ = node_->create_publisher<CommandResult>("arm_command_result", 10);
    auto ready_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    ready_pub_ = node_->create_publisher<std_msgs::msg::Bool>("commander/ready", ready_qos);
    std_msgs::msg::Bool ready;
    ready.data = true;
    ready_pub_->publish(ready);
  }

  std::string getPlanningFrame() const {return arm_->getPlanningFrame();}
  std::string getEndEffectorLink() const {return arm_->getEndEffectorLink();}

private:
  bool claimArm()
  {
    bool expected = false;
    return busy_.compare_exchange_strong(expected, true);
  }

  bool applySpeedScaling(double requested_scaling, const char * command_name)
  {
    if (!std::isfinite(requested_scaling) ||
      requested_scaling < kMinSpeedScaling ||
      requested_scaling > kMaxSpeedScaling)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Rejected %s: speed_scaling must be in [%.1f, %.1f] (received %.3f)",
        command_name, kMinSpeedScaling, kMaxSpeedScaling, requested_scaling);
      return false;
    }

    arm_->setMaxVelocityScalingFactor(requested_scaling);
    arm_->setMaxAccelerationScalingFactor(requested_scaling);
    RCLCPP_INFO(
      node_->get_logger(),
      "%s speed_scaling=%.2f (velocity and acceleration)",
      command_name, requested_scaling);
    return true;
  }

  ExecutionOutcome planAndExecute(
    const std::shared_ptr<MoveGroupInterface> & interface)
  {
    if (cancel_requested_.load()) {
      return {"CANCELLED", "cancelled before planning"};
    }

    const auto planning_started = std::chrono::steady_clock::now();
    MoveGroupInterface::Plan plan;
    const auto planning_result = interface->plan(plan);
    const auto planning_elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - planning_started).count();

    if (cancel_requested_.load()) {
      return {"CANCELLED", "cancelled while planning"};
    }

    if (planning_result != moveit::core::MoveItErrorCode::SUCCESS) {
      std::ostringstream detail;
      detail << "planning failed after " << planning_elapsed
             << " s (MoveIt code " << planning_result.val << ")";
      RCLCPP_ERROR(node_->get_logger(), "%s", detail.str().c_str());
      return {"FAILED", detail.str()};
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "Planning succeeded in %.3f s; submitting trajectory for execution...",
      planning_elapsed);
    const auto execution_started = std::chrono::steady_clock::now();
    const auto execution_result = interface->execute(plan);
    const auto execution_elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - execution_started).count();

    if (cancel_requested_.load()) {
      std::ostringstream detail;
      detail << "execution cancelled after " << execution_elapsed << " s";
      RCLCPP_WARN(node_->get_logger(), "%s", detail.str().c_str());
      return {"CANCELLED", detail.str()};
    }

    if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
      std::ostringstream detail;
      detail << "execution failed after " << execution_elapsed
             << " s (MoveIt code " << execution_result.val << ")";
      RCLCPP_ERROR(node_->get_logger(), "%s", detail.str().c_str());
      return {"FAILED", detail.str()};
    }

    std::ostringstream detail;
    detail << "trajectory execution completed in " << execution_elapsed << " s";
    RCLCPP_INFO(node_->get_logger(), "%s", detail.str().c_str());
    return {"SUCCEEDED", detail.str()};
  }

  void publishResult(
    const JointCmd & command, const std::string & status,
    const std::string & detail)
  {
    CommandResult result;
    result.show_run_id = command.show_run_id;
    result.node_id = command.node_id;
    result.command_id = command.command_id;
    result.attempt = command.attempt;
    result.target = "arm";
    result.status = status;
    result.detail = detail;
    result_pub_->publish(result);
  }

  void setActiveCommand(const JointCmd & command)
  {
    std::lock_guard<std::mutex> lock(active_mutex_);
    active_show_run_id_ = command.show_run_id;
    active_command_id_ = command.command_id;
  }

  void clearActiveCommand()
  {
    std::lock_guard<std::mutex> lock(active_mutex_);
    active_show_run_id_.clear();
    active_command_id_.clear();
  }

  void jointCmdCallback(const JointCmd & msg)
  {
    if (!claimArm()) {
      publishResult(msg, "REJECTED", "arm is busy");
      return;
    }
    BusyGuard busy_guard(busy_);
    cancel_requested_.store(false);
    setActiveCommand(msg);

    RCLCPP_INFO(
      node_->get_logger(),
      "Joint command received: run=%s node=%s command=%s speed=%.2f",
      msg.show_run_id.c_str(), msg.node_id.c_str(), msg.command_id.c_str(),
      msg.speed_scaling);

    for (const auto position : msg.positions) {
      if (!std::isfinite(position)) {
        publishResult(msg, "REJECTED", "all six joint positions must be finite");
        clearActiveCommand();
        return;
      }
    }
    if (!applySpeedScaling(msg.speed_scaling, "joint command")) {
      publishResult(msg, "REJECTED", "speed_scaling must be in [0.2, 1.0]");
      clearActiveCommand();
      return;
    }

    arm_->setStartStateToCurrentState();
    const std::vector<double> joints(msg.positions.begin(), msg.positions.end());
    if (!arm_->setJointValueTarget(joints)) {
      publishResult(msg, "REJECTED", "joint target is invalid");
      clearActiveCommand();
      return;
    }

    const auto outcome = planAndExecute(arm_);
    publishResult(msg, outcome.status, outcome.detail);
    clearActiveCommand();
  }

  void legacyJointCmdCallback(const FloatArray & msg)
  {
    if (msg.data.size() != 6) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Rejected legacy joint command: expected 6 positions, received %zu",
        msg.data.size());
      return;
    }
    if (!claimArm()) {
      RCLCPP_ERROR(node_->get_logger(), "Rejected legacy joint command: arm is busy");
      return;
    }
    BusyGuard busy_guard(busy_);
    cancel_requested_.store(false);
    RCLCPP_WARN_ONCE(
      node_->get_logger(),
      "/joint_command_legacy is deprecated; use dexter_interfaces/msg/JointCommand");
    applySpeedScaling(kLegacySpeedScaling, "legacy joint command");
    arm_->setStartStateToCurrentState();
    arm_->setJointValueTarget(msg.data);
    planAndExecute(arm_);
  }

  void poseCmdCallback(const PoseCmd & msg)
  {
    if (!claimArm()) {
      RCLCPP_ERROR(node_->get_logger(), "Rejected pose command: arm is busy");
      return;
    }
    BusyGuard busy_guard(busy_);
    cancel_requested_.store(false);

    if (!applySpeedScaling(msg.speed_scaling, "pose command")) {
      return;
    }

    tf2::Quaternion q;
    q.setRPY(
      msg.roll * M_PI / 180.0,
      msg.pitch * M_PI / 180.0,
      msg.yaw * M_PI / 180.0);
    q.normalize();

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base";
    target_pose.pose.position.x = msg.x;
    target_pose.pose.position.y = msg.y;
    target_pose.pose.position.z = msg.z;
    target_pose.pose.orientation.x = q.getX();
    target_pose.pose.orientation.y = q.getY();
    target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = q.getW();

    arm_->setStartStateToCurrentState();
    if (!msg.cartesian_path) {
      arm_->setPoseTarget(target_pose);
      planAndExecute(arm_);
      return;
    }

    std::vector<geometry_msgs::msg::Pose> waypoints{target_pose.pose};
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory);
    if (fraction != 1.0) {
      RCLCPP_ERROR(
        node_->get_logger(), "Cartesian planning incomplete (fraction %.3f)", fraction);
      return;
    }
    const auto result = arm_->execute(trajectory);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(
        node_->get_logger(), "Cartesian trajectory execution failed (MoveIt code %d)",
        result.val);
    }
  }

  void cancelCmdCallback(const DeviceCommand & msg)
  {
    {
      std::lock_guard<std::mutex> lock(active_mutex_);
      if (!busy_.load() ||
        msg.show_run_id != active_show_run_id_ ||
        msg.command_id != active_command_id_)
      {
        RCLCPP_WARN(
          node_->get_logger(),
          "Ignoring stale arm cancel: run=%s command=%s",
          msg.show_run_id.c_str(), msg.command_id.c_str());
        return;
      }
    }
    cancel_requested_.store(true);
    arm_->stop();
    RCLCPP_WARN(
      node_->get_logger(), "Arm cancel requested: run=%s command=%s",
      msg.show_run_id.c_str(), msg.command_id.c_str());
  }

  void emergencyStopCallback(const std_msgs::msg::Empty &)
  {
    cancel_requested_.store(true);
    arm_->stop();
    RCLCPP_WARN(node_->get_logger(), "Uncorrelated emergency arm stop requested");
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<MoveGroupInterface> arm_;

  std::atomic_bool busy_{false};
  std::atomic_bool cancel_requested_{false};
  std::mutex active_mutex_;
  std::string active_show_run_id_;
  std::string active_command_id_;

  rclcpp::CallbackGroup::SharedPtr command_group_;
  rclcpp::CallbackGroup::SharedPtr cancel_group_;
  rclcpp::Subscription<JointCmd>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<FloatArray>::SharedPtr legacy_joint_cmd_sub_;
  rclcpp::Subscription<PoseCmd>::SharedPtr pose_cmd_sub_;
  rclcpp::Subscription<DeviceCommand>::SharedPtr cancel_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr emergency_stop_sub_;
  rclcpp::Publisher<CommandResult>::SharedPtr result_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("commander");
  auto commander = std::make_shared<Commander>(node);

  RCLCPP_INFO(
    node->get_logger(), "Commander ready. Planning frame: %s, End effector: %s",
    commander->getPlanningFrame().c_str(), commander->getEndEffectorLink().c_str());
  RCLCPP_INFO(
    node->get_logger(),
    "Listening on /joint_command, /pose_command, and /joint_command_legacy; "
    "publishing correlated results on /arm_command_result");

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
