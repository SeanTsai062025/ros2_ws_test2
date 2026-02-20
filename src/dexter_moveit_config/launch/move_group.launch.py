from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("dexter_simplify", package_name="dexter_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Collect all MoveIt parameter dicts
    move_group_params = [
        moveit_config.to_dict(),
        # Trajectory execution monitoring — give real hardware extra time.
        # allowed_execution_duration_scaling: multiplier on planned duration
        #   before move_group considers the execution timed out.
        # execution_duration_monitoring: false disables the abort entirely.
        {
            "trajectory_execution.allowed_execution_duration_scaling": 2.0,
            "trajectory_execution.allowed_goal_duration_margin": 1.0,
            "trajectory_execution.execution_duration_monitoring": False,
        },
        # Publish SRDF on /robot_description_semantic so RViz can load it.
        {
            "publish_robot_description_semantic": True,
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
        },
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )

    return LaunchDescription([move_group_node])
