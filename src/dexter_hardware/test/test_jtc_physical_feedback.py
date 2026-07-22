#!/usr/bin/env python3

import pathlib
import time
import unittest

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint


ROBOT_DESCRIPTION = """
<robot name="feedback_test_robot">
  <link name="base_link"/>
  <link name="moving_link"/>
  <joint name="physical_joint" type="revolute">
    <parent link="base_link"/>
    <child link="moving_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.0" upper="2.0" velocity="2.0" effort="1.0"/>
  </joint>
  <ros2_control name="NonFollowingPhysicalSystem" type="system">
    <hardware>
      <plugin>mock_components/GenericSystem</plugin>
      <param name="position_state_following_offset">0.5</param>
    </hardware>
    <joint name="physical_joint">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity">
        <param name="initial_value">0.0</param>
      </state_interface>
    </joint>
  </ros2_control>
</robot>
"""


@pytest.mark.launch_test
def generate_test_description():
    controller_config = str(
        pathlib.Path(__file__).with_name('test_jtc_feedback_controllers.yaml'))
    control_node = launch_ros.actions.Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config],
        output='screen',
    )
    description_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ROBOT_DESCRIPTION}],
        output='screen',
    )
    controller_spawner = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager-timeout', '10'],
        output='screen',
    )
    return launch.LaunchDescription([
        control_node,
        description_publisher,
        controller_spawner,
        launch_testing.actions.ReadyToTest(),
    ])


class TestJtcUsesPhysicalFeedback(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('jtc_physical_feedback_test')

    def tearDown(self):
        self.node.destroy_node()

    def test_goal_cannot_succeed_when_state_does_not_follow(self):
        list_client = self.node.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.assertTrue(list_client.wait_for_service(timeout_sec=10.0))
        deadline = time.monotonic() + 10.0
        controller_active = False
        while time.monotonic() < deadline and not controller_active:
            list_future = list_client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(
                self.node, list_future, timeout_sec=1.0)
            if list_future.result() is not None:
                controller_active = any(
                    controller.name == 'arm_controller' and
                    controller.state == 'active'
                    for controller in list_future.result().controller)
            if not controller_active:
                time.sleep(0.05)
        self.assertTrue(controller_active)

        client = ActionClient(
            self.node, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')
        self.assertTrue(client.wait_for_server(timeout_sec=10.0))

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['physical_joint']
        point = JointTrajectoryPoint()
        point.positions = [1.0]
        point.velocities = [0.0]
        point.time_from_start.sec = 1
        goal.trajectory.points = [point]

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()
        self.assertIsNotNone(goal_handle)
        self.assertTrue(goal_handle.accepted)

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=5.0)
        response = result_future.result()
        self.assertIsNotNone(response)
        self.assertEqual(response.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(
            response.result.error_code,
            FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED)
