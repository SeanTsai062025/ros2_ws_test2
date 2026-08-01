#include "dexter_commander_cpp/robot_pose_panel.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>

#include <QApplication>
#include <QClipboard>
#include <QGroupBox>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSizePolicy>
#include <QTimer>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace dexter_commander_cpp
{
namespace
{

constexpr std::array<const char *, 6> kJointNames = {
  "base", "part1", "part2", "part3", "part4", "part5"};
constexpr char kBaseFrame[] = "base";
constexpr char kTipFrame[] = "gripper_tip";
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

QString formatNumber(double value)
{
  // Avoid displaying negative zero, then keep up to six useful decimal places.
  if (std::abs(value) < 0.0000005) {
    value = 0.0;
  }

  QString text = QString::number(value, 'f', 6);
  while (text.endsWith('0')) {
    text.chop(1);
  }
  if (text.endsWith('.')) {
    text.append('0');
  }
  return text;
}

void setTextIfChanged(QPlainTextEdit * field, const QString & text)
{
  if (field->toPlainText() != text) {
    field->setPlainText(text);
  }
}

}  // namespace

class RobotPosePanel::Impl
{
public:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription;

  std::mutex joint_mutex;
  std::array<double, 6> joints{};
  std::array<bool, 6> joints_received{};
  std::chrono::steady_clock::time_point last_joint_update{};
};

RobotPosePanel::RobotPosePanel(QWidget * parent)
: rviz_common::Panel(parent),
  impl_(std::make_unique<Impl>())
{
  auto * panel_layout = new QVBoxLayout(this);

  auto * heading = new QLabel("<b>Current Robot Pose</b>", this);
  auto * description = new QLabel(
    "Live values from <code>/joint_states</code> and TF. Joint angles are radians; "
    "Cartesian angles are degrees.", this);
  description->setWordWrap(true);
  panel_layout->addWidget(heading);
  panel_layout->addWidget(description);

  auto * joint_command_group = new QGroupBox("Joint-space RViz goal command", this);
  auto * joint_command_layout = new QVBoxLayout(joint_command_group);
  joint_command_ = new QPlainTextEdit("Waiting for /joint_states...", joint_command_group);
  joint_command_->setReadOnly(true);
  joint_command_->setLineWrapMode(QPlainTextEdit::WidgetWidth);
  joint_command_->setMinimumHeight(108);
  joint_command_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
  copy_joint_command_button_ = new QPushButton("Copy joint-space command", joint_command_group);
  copy_joint_command_button_->setEnabled(false);
  joint_command_layout->addWidget(joint_command_);
  joint_command_layout->addWidget(copy_joint_command_button_);
  panel_layout->addWidget(joint_command_group);

  auto * cartesian_command_group = new QGroupBox("Cartesian-space motion command", this);
  auto * cartesian_command_layout = new QVBoxLayout(cartesian_command_group);
  cartesian_command_ = new QPlainTextEdit(
    "Waiting for base -> gripper_tip TF...", cartesian_command_group);
  cartesian_command_->setReadOnly(true);
  cartesian_command_->setLineWrapMode(QPlainTextEdit::WidgetWidth);
  cartesian_command_->setMinimumHeight(108);
  cartesian_command_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
  cartesian_command_->setToolTip(
    "Publishing this /pose_command command requests planned robot motion.");
  copy_cartesian_command_button_ = new QPushButton(
    "Copy Cartesian command", cartesian_command_group);
  copy_cartesian_command_button_->setEnabled(false);
  copy_cartesian_command_button_->setToolTip(
    "Publishing the copied /pose_command command requests planned robot motion.");
  cartesian_command_layout->addWidget(cartesian_command_);
  cartesian_command_layout->addWidget(copy_cartesian_command_button_);
  panel_layout->addWidget(cartesian_command_group);

  auto * joint_group = new QGroupBox("Joint space (radians)", this);
  auto * joint_layout = new QVBoxLayout(joint_group);
  joint_values_ = new QPlainTextEdit("Waiting for /joint_states...", joint_group);
  joint_values_->setReadOnly(true);
  joint_values_->setLineWrapMode(QPlainTextEdit::WidgetWidth);
  joint_values_->setMinimumHeight(62);
  joint_values_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
  copy_joint_button_ = new QPushButton("Copy joint values", joint_group);
  copy_joint_button_->setEnabled(false);
  joint_layout->addWidget(joint_values_);
  joint_layout->addWidget(copy_joint_button_);
  panel_layout->addWidget(joint_group);

  auto * cartesian_group = new QGroupBox("Cartesian space (base frame)", this);
  auto * cartesian_layout = new QVBoxLayout(cartesian_group);
  cartesian_values_ = new QPlainTextEdit("Waiting for base -> gripper_tip TF...", cartesian_group);
  cartesian_values_->setReadOnly(true);
  cartesian_values_->setLineWrapMode(QPlainTextEdit::WidgetWidth);
  cartesian_values_->setMinimumHeight(86);
  cartesian_values_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
  copy_cartesian_button_ = new QPushButton("Copy Cartesian values", cartesian_group);
  copy_cartesian_button_->setEnabled(false);
  cartesian_layout->addWidget(cartesian_values_);
  cartesian_layout->addWidget(copy_cartesian_button_);
  panel_layout->addWidget(cartesian_group);

  status_label_ = new QLabel("Initializing ROS connections...", this);
  status_label_->setWordWrap(true);
  panel_layout->addWidget(status_label_);
  panel_layout->addStretch();

  update_timer_ = new QTimer(this);
  update_timer_->setInterval(100);

  connect(update_timer_, &QTimer::timeout, this, &RobotPosePanel::updateDisplay);
  connect(copy_joint_command_button_, &QPushButton::clicked, this, [this]() {
      copyText(joint_command_, copy_joint_command_button_, "Copy joint-space command");
  });
  connect(copy_cartesian_command_button_, &QPushButton::clicked, this, [this]() {
      copyText(cartesian_command_, copy_cartesian_command_button_, "Copy Cartesian command");
  });
  connect(copy_joint_button_, &QPushButton::clicked, this, [this]() {
      copyText(joint_values_, copy_joint_button_, "Copy joint values");
  });
  connect(copy_cartesian_button_, &QPushButton::clicked, this, [this]() {
      copyText(cartesian_values_, copy_cartesian_button_, "Copy Cartesian values");
  });
}

RobotPosePanel::~RobotPosePanel() = default;

void RobotPosePanel::onInitialize()
{
  auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    status_label_->setText("RViz ROS node is unavailable.");
    status_label_->setStyleSheet("color: #e57373;");
    return;
  }

  impl_->node = node_abstraction->get_raw_node();
  impl_->tf_buffer = std::make_shared<tf2_ros::Buffer>(impl_->node->get_clock());
  // RViz already spins this node, so the listener must not create a second executor thread.
  impl_->tf_listener = std::make_shared<tf2_ros::TransformListener>(
    *impl_->tf_buffer, impl_->node, false);

  impl_->joint_state_subscription =
    impl_->node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
      std::lock_guard<std::mutex> lock(impl_->joint_mutex);
      const auto count = std::min(message->name.size(), message->position.size());
      bool found_joint = false;

      for (std::size_t message_index = 0; message_index < count; ++message_index) {
        const auto wanted = std::find_if(
          kJointNames.begin(), kJointNames.end(),
          [&message, message_index](const char * name) {
            return message->name[message_index] == name;
          });
        if (wanted == kJointNames.end()) {
          continue;
        }

        const auto joint_index = static_cast<std::size_t>(
          std::distance(kJointNames.begin(), wanted));
        impl_->joints[joint_index] = message->position[message_index];
        impl_->joints_received[joint_index] = true;
        found_joint = true;
      }

      if (found_joint) {
        impl_->last_joint_update = std::chrono::steady_clock::now();
      }
    });

  status_label_->setText("Waiting for live robot state...");
  status_label_->setStyleSheet("color: #ffb74d;");
  update_timer_->start();
}

void RobotPosePanel::updateDisplay()
{
  std::array<double, 6> joints{};
  bool have_all_joints = false;
  bool joint_state_is_fresh = false;
  {
    std::lock_guard<std::mutex> lock(impl_->joint_mutex);
    joints = impl_->joints;
    have_all_joints = std::all_of(
      impl_->joints_received.begin(), impl_->joints_received.end(),
      [](bool received) {return received;});
    if (have_all_joints) {
      joint_state_is_fresh =
        std::chrono::steady_clock::now() - impl_->last_joint_update < std::chrono::seconds(2);
    }
  }

  if (have_all_joints) {
    QStringList values;
    for (double joint : joints) {
      values.append(formatNumber(joint));
    }
    setTextIfChanged(joint_values_, "[" + values.join(", ") + "]");
    const QString joint_command = QString(
      "ros2 topic pub -1 /rviz/moveit/update_custom_goal_state "
      "moveit_msgs/msg/RobotState "
      "\"{joint_state: {name: [base, part1, part2, part3, part4, part5], "
      "position: [%1]}}\"")
      .arg(values.join(", "));
    setTextIfChanged(joint_command_, joint_command);
    copy_joint_button_->setEnabled(true);
    copy_joint_command_button_->setEnabled(true);
  }

  bool have_cartesian_pose = false;
  if (impl_->tf_buffer) {
    try {
      const auto transform = impl_->tf_buffer->lookupTransform(
        kBaseFrame, kTipFrame, tf2::TimePointZero);
      const auto & translation = transform.transform.translation;
      const auto & rotation = transform.transform.rotation;

      tf2::Quaternion quaternion(rotation.x, rotation.y, rotation.z, rotation.w);
      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

      const QString x = formatNumber(translation.x);
      const QString y = formatNumber(translation.y);
      const QString z = formatNumber(translation.z);
      const QString roll_degrees = formatNumber(roll * kRadiansToDegrees);
      const QString pitch_degrees = formatNumber(pitch * kRadiansToDegrees);
      const QString yaw_degrees = formatNumber(yaw * kRadiansToDegrees);
      const QString pose = QString(
        "{x: %1, y: %2, z: %3, roll: %4, pitch: %5, yaw: %6}")
        .arg(x)
        .arg(y)
        .arg(z)
        .arg(roll_degrees)
        .arg(pitch_degrees)
        .arg(yaw_degrees);
      setTextIfChanged(cartesian_values_, pose);
      const QString cartesian_command = QString(
        "ros2 topic pub -1 /pose_command dexter_interfaces/msg/PoseCommand "
        "\"{x: %1, y: %2, z: %3, roll: %4, pitch: %5, yaw: %6, "
        "cartesian_path: false, speed_scaling: 0.5}\"")
        .arg(x)
        .arg(y)
        .arg(z)
        .arg(roll_degrees)
        .arg(pitch_degrees)
        .arg(yaw_degrees);
      setTextIfChanged(cartesian_command_, cartesian_command);
      copy_cartesian_button_->setEnabled(true);
      copy_cartesian_command_button_->setEnabled(true);
      have_cartesian_pose = true;
    } catch (const tf2::TransformException &) {
      // TF commonly takes a moment to become available during launch.
    }
  }

  if (have_all_joints && have_cartesian_pose && joint_state_is_fresh) {
    status_label_->setText("Live: /joint_states | base -> gripper_tip");
    status_label_->setStyleSheet("color: #81c784;");
  } else if (have_all_joints && have_cartesian_pose) {
    status_label_->setText(
        "Values shown, but /joint_states has not updated in the last 2 seconds.");
    status_label_->setStyleSheet("color: #ffb74d;");
  } else if (!have_all_joints && !have_cartesian_pose) {
    status_label_->setText("Waiting for /joint_states and base -> gripper_tip TF...");
    status_label_->setStyleSheet("color: #ffb74d;");
  } else if (!have_all_joints) {
    status_label_->setText("Waiting for all six arm joints on /joint_states...");
    status_label_->setStyleSheet("color: #ffb74d;");
  } else {
    status_label_->setText("Waiting for base -> gripper_tip TF...");
    status_label_->setStyleSheet("color: #ffb74d;");
  }
}

void RobotPosePanel::copyText(
  QPlainTextEdit * source, QPushButton * button, const QString & button_text)
{
  QApplication::clipboard()->setText(source->toPlainText());
  button->setText("Copied!");
  QTimer::singleShot(900, button, [button, button_text]() {
      button->setText(button_text);
  });
}

}  // namespace dexter_commander_cpp

PLUGINLIB_EXPORT_CLASS(dexter_commander_cpp::RobotPosePanel, rviz_common::Panel)
