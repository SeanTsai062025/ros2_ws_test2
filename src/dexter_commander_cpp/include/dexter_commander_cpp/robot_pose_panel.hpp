#ifndef DEXTER_COMMANDER_CPP__ROBOT_POSE_PANEL_HPP_
#define DEXTER_COMMANDER_CPP__ROBOT_POSE_PANEL_HPP_

#include <memory>

#include <rviz_common/panel.hpp>

class QLabel;
class QPlainTextEdit;
class QPushButton;
class QTimer;
class QWidget;

namespace dexter_commander_cpp
{

class RobotPosePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit RobotPosePanel(QWidget * parent = nullptr);
  ~RobotPosePanel() override;

  void onInitialize() override;

private:
  class Impl;

  void updateDisplay();
  void copyText(QPlainTextEdit * source, QPushButton * button, const QString & button_text);

  std::unique_ptr<Impl> impl_;

  QPlainTextEdit * joint_command_;
  QPlainTextEdit * cartesian_command_;
  QPlainTextEdit * joint_values_;
  QPlainTextEdit * cartesian_values_;
  QPushButton * copy_joint_command_button_;
  QPushButton * copy_cartesian_command_button_;
  QPushButton * copy_joint_button_;
  QPushButton * copy_cartesian_button_;
  QLabel * status_label_;
  QTimer * update_timer_;
};

}  // namespace dexter_commander_cpp

#endif  // DEXTER_COMMANDER_CPP__ROBOT_POSE_PANEL_HPP_
