#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <dexter_interfaces/msg/pose_command.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <rclcpp/rclcpp.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using PoseCmd = dexter_interfaces::msg::PoseCommand;
using RobotState = moveit_msgs::msg::RobotState;
using GetPositionIK = moveit_msgs::srv::GetPositionIK;

using namespace std::placeholders;

namespace
{

constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
constexpr double kIkTimeoutSeconds = 5.0;
constexpr char kCommandFrame[] = "base";
constexpr char kRvizGoalStateTopic[] = "/rviz/moveit/update_custom_goal_state";

bool allFinite(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {
             return std::isfinite(value);
  });
}

}  // namespace

class Commander
{
public:
  explicit Commander(std::shared_ptr<rclcpp::Node> node)
  {
    node_ = node;
    arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
    arm_->setMaxVelocityScalingFactor(1.0);
    arm_->setMaxAccelerationScalingFactor(1.0);
    arm_->setPlanningTime(10.0);
    arm_->setNumPlanningAttempts(5);

    rviz_goal_state_pub_ = node_->create_publisher<RobotState>(
      kRvizGoalStateTopic, rclcpp::SystemDefaultsQoS());
    ik_client_ = node_->create_client<GetPositionIK>("compute_ik");

    joint_cmd_sub_ = node_->create_subscription<FloatArray>(
      "joint_command", 10, std::bind(&Commander::jointCmdCallback, this, _1));
    pose_cmd_sub_ = node_->create_subscription<PoseCmd>(
      "pose_command", 10, std::bind(&Commander::poseCmdCallback, this, _1));
    joint_goal_cmd_sub_ = node_->create_subscription<FloatArray>(
      "joint_goal_command", 10, std::bind(&Commander::jointGoalCmdCallback, this, _1));
    pose_goal_cmd_sub_ = node_->create_subscription<PoseCmd>(
      "pose_goal_command", 10, std::bind(&Commander::poseGoalCmdCallback, this, _1));
  }

  void goToNamedTarget(const std::string & name)
  {
    arm_->setStartStateToCurrentState();
    arm_->setNamedTarget(name);
    planAndExecute(arm_);
  }

  void goToJointTarget(const std::vector<double> & joints)
  {
    arm_->setStartStateToCurrentState();
    arm_->setJointValueTarget(joints);
    planAndExecute(arm_);
  }

  std::string getPlanningFrame() const {return arm_->getPlanningFrame();}
  std::string getEndEffectorLink() const {return arm_->getEndEffectorLink();}

  void goToPoseTarget(
    double x, double y, double z,
    double roll, double pitch, double yaw, bool cartesian_path = false)
  {
    const auto target_pose = makeTargetPose(x, y, z, roll, pitch, yaw);

    arm_->setStartStateToCurrentState();

    if (!cartesian_path) {
      arm_->setPoseTarget(target_pose);
      planAndExecute(arm_);
    } else {
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
  geometry_msgs::msg::PoseStamped makeTargetPose(
    double x, double y, double z, double roll, double pitch, double yaw) const
  {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.stamp = node_->now();
    target_pose.header.frame_id = kCommandFrame;
    target_pose.pose.position.x = x;
    target_pose.pose.position.y = y;
    target_pose.pose.position.z = z;
    target_pose.pose.orientation.x = q.getX();
    target_pose.pose.orientation.y = q.getY();
    target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = q.getW();
    return target_pose;
  }

  void publishRvizGoalState(const RobotState & goal_state)
  {
    if (rviz_goal_state_pub_->get_subscription_count() == 0) {
      RCLCPP_WARN(node_->get_logger(),
        "No RViz goal-state subscriber is connected. In the MotionPlanning display, "
        "enable 'Allow External Program Communication'.");
    }
    rviz_goal_state_pub_->publish(goal_state);
  }

  void publishRvizJointGoal(const std::vector<double> & joints)
  {
    RobotState goal_state;
    goal_state.is_diff = true;
    goal_state.joint_state.header.stamp = node_->now();
    goal_state.joint_state.name = arm_->getJointNames();
    goal_state.joint_state.position = joints;
    publishRvizGoalState(goal_state);
  }

  bool validateJointValues(
    const std::vector<double> & joints, const char * description) const
  {
    const auto expected_count = arm_->getJointNames().size();
    if (joints.size() != expected_count) {
      RCLCPP_ERROR(node_->get_logger(),
        "%s contains %zu values; planning group '%s' requires %zu.",
        description, joints.size(), arm_->getName().c_str(), expected_count);
      return false;
    }
    if (!allFinite(joints)) {
      RCLCPP_ERROR(node_->get_logger(), "%s contains a non-finite value.", description);
      return false;
    }
    return true;
  }

  void planAndExecute(const std::shared_ptr<MoveGroupInterface> & interface)
  {
    MoveGroupInterface::Plan plan;
    auto error_code = interface->plan(plan);
    bool success = (error_code == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(node_->get_logger(), "Planning succeeded! Executing...");
      interface->execute(plan);
    } else {
      RCLCPP_ERROR(node_->get_logger(),
        "Planning FAILED with error code: %d. "
        "Check that the target pose is reachable and collision-free.",
        error_code.val);
    }
  }

  void jointCmdCallback(const FloatArray & msg)
  {
    auto joints = msg.data;

    if (joints.size() == 6) {
      goToJointTarget(joints);
    }
  }

  void poseCmdCallback(const PoseCmd & msg)
  {
    // RPY comes in as degrees from the topic — convert to radians
    double roll_rad = msg.roll * kDegreesToRadians;
    double pitch_rad = msg.pitch * kDegreesToRadians;
    double yaw_rad = msg.yaw * kDegreesToRadians;

    RCLCPP_INFO(node_->get_logger(),
      "Pose command: pos=(%.4f, %.4f, %.4f) rpy=(%.1f°, %.1f°, %.1f°)",
      msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw);

    goToPoseTarget(
      msg.x,
      msg.y,
      msg.z,
      roll_rad,
      pitch_rad,
      yaw_rad,
      msg.cartesian_path
    );
  }

  void jointGoalCmdCallback(const FloatArray & msg)
  {
    if (!validateJointValues(msg.data, "Joint goal command")) {
      return;
    }

    // Ensure an older Cartesian IK response cannot overwrite this newer command.
    ++pose_goal_request_sequence_;
    publishRvizJointGoal(msg.data);
    RCLCPP_INFO(node_->get_logger(),
      "Updated the RViz joint goal state; no motion was planned or executed.");
  }

  void poseGoalCmdCallback(const PoseCmd & msg)
  {
    const std::vector<double> pose_values = {
      msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw};
    if (!allFinite(pose_values)) {
      RCLCPP_ERROR(node_->get_logger(), "Pose goal command contains a non-finite value.");
      return;
    }
    if (!ik_client_->service_is_ready()) {
      RCLCPP_ERROR(node_->get_logger(),
        "MoveIt's /compute_ik service is not available; the RViz pose goal was not updated.");
      return;
    }

    const auto current_joints = arm_->getCurrentJointValues();
    if (!validateJointValues(current_joints, "Current joint state for Cartesian goal IK")) {
      return;
    }

    auto request = std::make_shared<GetPositionIK::Request>();
    request->ik_request.group_name = arm_->getName();
    request->ik_request.robot_state.is_diff = true;
    request->ik_request.robot_state.joint_state.header.stamp = node_->now();
    request->ik_request.robot_state.joint_state.name = arm_->getJointNames();
    request->ik_request.robot_state.joint_state.position = current_joints;
    request->ik_request.avoid_collisions = false;
    request->ik_request.ik_link_name = arm_->getEndEffectorLink();
    // cartesian_path is intentionally ignored: this interface only computes a goal state.
    request->ik_request.pose_stamped = makeTargetPose(
      msg.x,
      msg.y,
      msg.z,
      msg.roll * kDegreesToRadians,
      msg.pitch * kDegreesToRadians,
      msg.yaw * kDegreesToRadians);
    request->ik_request.timeout = rclcpp::Duration::from_seconds(kIkTimeoutSeconds);

    const auto request_sequence = ++pose_goal_request_sequence_;
    ik_client_->async_send_request(
      request,
      [this, request_sequence](rclcpp::Client<GetPositionIK>::SharedFuture future) {
        if (request_sequence != pose_goal_request_sequence_.load()) {
          RCLCPP_DEBUG(node_->get_logger(), "Ignoring a superseded Cartesian goal IK response.");
          return;
        }

        GetPositionIK::Response::SharedPtr response;
        try {
          response = future.get();
        } catch (const std::exception & error) {
          RCLCPP_ERROR(node_->get_logger(), "Cartesian goal IK request failed: %s", error.what());
          return;
        }
        if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
          RCLCPP_ERROR(node_->get_logger(),
            "Cartesian goal IK failed with MoveIt error code %d; the RViz goal was not updated.",
            response->error_code.val);
          return;
        }

        publishRvizGoalState(response->solution);
        RCLCPP_INFO(node_->get_logger(),
          "Updated the RViz Cartesian goal state; no motion was planned or executed.");
      });
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<MoveGroupInterface> arm_;
  rclcpp::Publisher<RobotState>::SharedPtr rviz_goal_state_pub_;
  rclcpp::Client<GetPositionIK>::SharedPtr ik_client_;

  rclcpp::Subscription<FloatArray>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<PoseCmd>::SharedPtr pose_cmd_sub_;
  rclcpp::Subscription<FloatArray>::SharedPtr joint_goal_cmd_sub_;
  rclcpp::Subscription<PoseCmd>::SharedPtr pose_goal_cmd_sub_;
  std::atomic<std::uint64_t> pose_goal_request_sequence_{0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("commander");
  auto commander = Commander(node);

  RCLCPP_INFO(node->get_logger(), "Commander ready. Planning frame: %s, End effector: %s",
    commander.getPlanningFrame().c_str(), commander.getEndEffectorLink().c_str());
  RCLCPP_INFO(node->get_logger(),
    "Motion topics: /joint_command and /pose_command");
  RCLCPP_INFO(node->get_logger(),
    "RViz goal-only topics: /joint_goal_command and /pose_goal_command");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
