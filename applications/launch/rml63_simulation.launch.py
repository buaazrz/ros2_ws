#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ============================================================
    # 1. 启动参数
    # ============================================================
    use_rviz = LaunchConfiguration("use_rviz")

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to start RViz2",
    )

    # ============================================================
    # 2. 读取 RML63 URDF
    # ============================================================
    description_package_path = Path(
        get_package_share_directory("rml63_description")
    )

    urdf_file = (
        description_package_path
        / "urdf"
        / "drone_rml63.urdf"
    )

    if not urdf_file.exists():
        raise FileNotFoundError(
            f"RML63 URDF file does not exist: {urdf_file}"
        )

    robot_description_content = urdf_file.read_text(
        encoding="utf-8"
    )

    robot_description = {
        "robot_description": robot_description_content
    }

    # ============================================================
    # 3. SimulationRobot 和控制器参数
    # ============================================================
    simulation_params = PathJoinSubstitution(
        [
            FindPackageShare("applications"),
            "config",
            "rml63",
            "simulation_params.yaml",
        ]
    )

    # ============================================================
    # 4. RViz 配置
    # ============================================================
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("applications"),
            "config",
            "rml63",
            "view_robot.rviz",
        ]
    )

    # ============================================================
    # 5. robot_state_publisher
    # ============================================================
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
        ],
    )

    # ============================================================
    # 6. 自己框架中的 control_node
    # ============================================================
    control_node = Node(
        package="control_node",
        executable="control_node",
        # name="control_node",
        output="both",
        parameters=[
            simulation_params,
            robot_description,
        ],
    )

    # ============================================================
    # 7. 机器人状态监控节点
    # ============================================================
    robot_monitor_node = Node(
        package="robot_monitor",
        executable="robot_monitor",
        name="robot_monitor",
        output="both",
    )

    # ============================================================
    # 8. 机器人控制界面
    # ============================================================
    robot_gui_node = Node(
        package="robot_gui",
        executable="robot_gui",
        name="robot_gui",
        output="both",
        parameters=[
            robot_description,
        ],
    )

    # ============================================================
    # 9. RViz2
    # ============================================================
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "--display-config",
            rviz_config,
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            declare_use_rviz,

            robot_state_publisher_node,
            control_node,
            robot_monitor_node,
            robot_gui_node,
            rviz_node,
        ]
    )