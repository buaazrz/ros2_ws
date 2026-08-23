# Copyright 2023 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch the custom FR3 control stack with the MuJoCo hardware backend.

MuJoCo is started by hardwares::MujocoRobot inside the control_node process.
The associated parameter file must therefore select:

  simulation: true
  simulation_backend: mujoco
  robot: hardwares::MujocoRobot
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_rviz = LaunchConfiguration("use_rviz")
    use_robot_monitor = LaunchConfiguration("use_robot_monitor")
    use_robot_gui = LaunchConfiguration("use_robot_gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    mujoco_gl = LaunchConfiguration("mujoco_gl")

    default_params_file = PathJoinSubstitution(
        [
            FindPackageShare("applications"),
            "config",
            "fr3",
            "mujoco_params.yaml",
        ]
    )

    rviz_file = PathJoinSubstitution(
        [
            FindPackageShare("applications"),
            "config",
            "fr3",
            "visualize_franka.rviz",
        ]
    )

    robot_xacro_file = PathJoinSubstitution(
        [
            FindPackageShare("franka_description"),
            "robots",
            "fr3",
            "fr3.urdf.xacro",
        ]
    )

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                robot_xacro_file,
                " hand:=false",
                " ee_id:=frank_hand",
            ]
        ),
        value_type=str,
    )

    common_parameters = {
        "robot_description": robot_description,
        "use_sim_time": use_sim_time,
    }

    control_node = Node(
        package="control_node",
        executable="control_node",
        name="control_node",
        output="screen",
        emulate_tty=True,
        parameters=[params_file, common_parameters],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[common_parameters],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["--display-config", rviz_file],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    robot_monitor = Node(
        package="robot_monitor",
        executable="robot_monitor",
        name="robot_monitor",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_robot_monitor),
    )

    robot_gui = Node(
        package="robot_gui",
        executable="robot_gui",
        name="robot_gui",
        output="screen",
        parameters=[common_parameters],
        condition=IfCondition(use_robot_gui),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Absolute path to the MuJoCo control parameter YAML file",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2",
            ),
            DeclareLaunchArgument(
                "use_robot_monitor",
                default_value="true",
                description="Start robot_monitor",
            ),
            DeclareLaunchArgument(
                "use_robot_gui",
                default_value="true",
                description="Start robot_gui",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description=(
                    "Use ROS /clock. Keep false unless the MuJoCo backend publishes /clock"
                ),
            ),
            DeclareLaunchArgument(
                "mujoco_gl",
                default_value="glfw",
                description="MuJoCo rendering backend: glfw for desktop, egl for headless",
            ),
            SetEnvironmentVariable("MUJOCO_GL", mujoco_gl),
            robot_state_publisher,
            control_node,
            rviz_node,
            robot_monitor,
            robot_gui,
        ]
    )
