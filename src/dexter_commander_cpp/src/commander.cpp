#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <dexter_interfaces/msg/joint_command.hpp>
#include <dexter_interfaces/msg/pose_command.hpp>

#include <chrono>
#include <cmath>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using JointCmd = dexter_interfaces::msg::JointCommand;
using PoseCmd = dexter_interfaces::msg::PoseCommand;

using namespace std::placeholders;

namespace
{

constexpr double kMinSpeedScaling = 0.2;
constexpr double kMaxSpeedScaling = 1.0;
constexpr double kDefaultSpeedScaling = 0.5;
constexpr double kLegacySpeedScaling = kDefaultSpeedScaling;

}  // namespace

class Commander
{
public:
  Commander(std::shared_ptr<rclcpp::Node> node)
  {
    node_ = node;
    arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
    arm_->setMaxVelocityScalingFactor(kDefaultSpeedScaling);
    arm_->setMaxAccelerationScalingFactor(kDefaultSpeedScaling);
    arm_->setPlanningTime(10.0);
    arm_->setNumPlanningAttempts(5);

    joint_cmd_sub_ = node_->create_subscription<JointCmd>(
      "joint_command", 10, std::bind(&Commander::jointCmdCallback, this, _1));
    legacy_joint_cmd_sub_ = node_->create_subscription<FloatArray>(
      "joint_command_legacy", 10,
      std::bind(&Commander::legacyJointCmdCallback, this, _1));
    pose_cmd_sub_ = node_->create_subscription<PoseCmd>(
      "pose_command", 10, std::bind(&Commander::poseCmdCallback, this, _1));
  }

  void goToNamedTarget(
    const std::string &name, double speed_scaling = kDefaultSpeedScaling)
  {
    if (!applySpeedScaling(speed_scaling, "named target")) {
      return;
    }
    arm_->setStartStateToCurrentState();
    arm_->setNamedTarget(name);
    planAndExecute(arm_);
  }

  void goToJointTarget(
    const std::vector<double> &joints, double speed_scaling)
  {
    if (!applySpeedScaling(speed_scaling, "joint command")) {
      return;
    }
    arm_->setStartStateToCurrentState();
    arm_->setJointValueTarget(joints);
    planAndExecute(arm_);
  }

  std::string getPlanningFrame() const { return arm_->getPlanningFrame(); }
  std::string getEndEffectorLink() const { return arm_->getEndEffectorLink(); }

  void goToPoseTarget(double x, double y, double z,
                      double roll, double pitch, double yaw,
                      bool cartesian_path, double speed_scaling)
  {
    if (!applySpeedScaling(speed_scaling, "pose command")) {
      return;
    }

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q = q.normalize();

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base";
    target_pose.pose.position.x = x;
    target_pose.pose.position.y = y;
    target_pose.pose.position.z = z;
    target_pose.pose.orientation.x = q.getX();
    target_pose.pose.orientation.y = q.getY();
    target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = q.getW();

    arm_->setStartStateToCurrentState();

    if(!cartesian_path){
      arm_->setPoseTarget(target_pose);
      planAndExecute(arm_);
    }else{
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(target_pose.pose);

      moveit_msgs::msg::RobotTrajectory trajectory;

      double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory);

      if (fraction == 1) {
        arm_->execute(trajectory);
      }
    }
  }

private:

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

  void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
  {
    const auto planning_started = std::chrono::steady_clock::now();
    MoveGroupInterface::Plan plan;
    auto error_code = interface->plan(plan);
    bool success = (error_code == moveit::core::MoveItErrorCode::SUCCESS);
    const auto planning_elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - planning_started).count();

    if (success) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Planning succeeded in %.3f s; submitting trajectory for execution...",
        planning_elapsed);
      const auto execution_started = std::chrono::steady_clock::now();
      const auto execution_result = interface->execute(plan);
      const auto execution_elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - execution_started).count();
      if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Trajectory execution failed after %.3f s with error code: %d",
          execution_elapsed, execution_result.val);
      } else {
        RCLCPP_INFO(
          node_->get_logger(),
          "Trajectory execution completed in %.3f s",
          execution_elapsed);
      }
    } else {
      RCLCPP_ERROR(node_->get_logger(),
        "Planning FAILED after %.3f s with error code: %d. "
        "Check that the target pose is reachable and collision-free.",
        planning_elapsed, error_code.val);
    }
  }

  void jointCmdCallback(const JointCmd &msg)
  {
    RCLCPP_INFO(
      node_->get_logger(),
      "Joint command received; speed=%.2f",
      msg.speed_scaling);
    const std::vector<double> joints(msg.positions.begin(), msg.positions.end());
    goToJointTarget(joints, msg.speed_scaling);
  }

  void legacyJointCmdCallback(const FloatArray &msg)
  {
    if (msg.data.size() == 6) {
      RCLCPP_WARN_ONCE(
        node_->get_logger(),
        "/joint_command_legacy is deprecated; use dexter_interfaces/msg/JointCommand on "
        "/joint_command to select speed. Legacy commands retain the old 1 rad/s maximum");
      goToJointTarget(msg.data, kLegacySpeedScaling);
    } else {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Rejected legacy joint command: expected 6 positions, received %zu",
        msg.data.size());
    }
  }

  void poseCmdCallback(const PoseCmd &msg)
  {
    // RPY comes in as degrees from the topic — convert to radians
    double roll_rad  = msg.roll  * M_PI / 180.0;
    double pitch_rad = msg.pitch * M_PI / 180.0;
    double yaw_rad   = msg.yaw   * M_PI / 180.0;

    RCLCPP_INFO(node_->get_logger(),
      "Pose command: pos=(%.4f, %.4f, %.4f) rpy=(%.1f°, %.1f°, %.1f°) speed=%.2f",
      msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, msg.speed_scaling);

    goToPoseTarget(
      msg.x,
      msg.y, 
      msg.z,
      roll_rad,
      pitch_rad,
      yaw_rad,
      msg.cartesian_path,
      msg.speed_scaling
    );
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<MoveGroupInterface> arm_;

  rclcpp::Subscription<JointCmd>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<FloatArray>::SharedPtr legacy_joint_cmd_sub_;
  rclcpp::Subscription<PoseCmd>::SharedPtr pose_cmd_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("commander");
  auto commander = Commander(node);

  RCLCPP_INFO(node->get_logger(), "Commander ready. Planning frame: %s, End effector: %s",
    commander.getPlanningFrame().c_str(), commander.getEndEffectorLink().c_str());
  RCLCPP_INFO(
    node->get_logger(),
    "Listening on /joint_command, /pose_command, and /joint_command_legacy; "
    "speed_scaling range is [%.1f, %.1f]",
    kMinSpeedScaling, kMaxSpeedScaling);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
