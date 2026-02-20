#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <dexter_interfaces/msg/pose_command.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using PoseCmd = dexter_interfaces::msg::PoseCommand;

using namespace std::placeholders;

class Commander
{
public:
  Commander(std::shared_ptr<rclcpp::Node> node)
  {
    node_ = node;
    arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
    arm_->setMaxVelocityScalingFactor(1.0);
    arm_->setMaxAccelerationScalingFactor(1.0);
    arm_->setPlanningTime(10.0);
    arm_->setNumPlanningAttempts(5);

    joint_cmd_sub_ = node_->create_subscription<FloatArray>(
      "joint_command", 10, std::bind(&Commander::jointCmdCallback, this, _1));
    pose_cmd_sub_ = node_->create_subscription<PoseCmd>(
      "pose_command", 10, std::bind(&Commander::poseCmdCallback, this, _1));
  }

  void goToNamedTarget(const std::string &name)
  {
    arm_->setStartStateToCurrentState();
    arm_->setNamedTarget(name);
    planAndExecute(arm_);
  }

  void goToJointTarget(const std::vector<double> &joints)
  {
    arm_->setStartStateToCurrentState();
    arm_->setJointValueTarget(joints);
    planAndExecute(arm_);
  }

  std::string getPlanningFrame() const { return arm_->getPlanningFrame(); }
  std::string getEndEffectorLink() const { return arm_->getEndEffectorLink(); }

  void goToPoseTarget(double x, double y, double z,
                      double roll, double pitch, double yaw, bool cartesian_path=false)
  {
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

  void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
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

  void jointCmdCallback(const FloatArray &msg)
  {
    auto joints = msg.data;

    if (joints.size() == 6) {
      goToJointTarget(joints);
    }
  }

  void poseCmdCallback(const PoseCmd &msg)
  {
    // RPY comes in as degrees from the topic — convert to radians
    double roll_rad  = msg.roll  * M_PI / 180.0;
    double pitch_rad = msg.pitch * M_PI / 180.0;
    double yaw_rad   = msg.yaw   * M_PI / 180.0;

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

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<MoveGroupInterface> arm_;

  rclcpp::Subscription<FloatArray>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<PoseCmd>::SharedPtr pose_cmd_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("commander");
  auto commander = Commander(node);

  RCLCPP_INFO(node->get_logger(), "Commander ready. Planning frame: %s, End effector: %s",
    commander.getPlanningFrame().c_str(), commander.getEndEffectorLink().c_str());
  RCLCPP_INFO(node->get_logger(), "Listening on /joint_command and /pose_command");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
