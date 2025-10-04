# Copyright 2025 Christopher Newport University - CNU Robotics
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---------------------------
    # Declare launch arguments
    # ---------------------------
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "joint_state_publisher_gui",
            default_value="true",
            description="Start joint_state_publisher_gui for manual sliders.",
        )
    )

    # ---------------------------
    # Launch configurations
    # ---------------------------
    gui = LaunchConfiguration("gui")
    joint_state_publisher_gui = LaunchConfiguration("joint_state_publisher_gui")

    # ---------------------------
    # Robot description (URDF via Xacro)
    # ---------------------------
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("pinc_open_driver"), "urdf", "gripper.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # ---------------------------
    # RViz config file
    # ---------------------------
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("pinc_open_driver"), "rviz", "pinc_gripper.rviz"]
    )

    # ---------------------------
    # Nodes
    # ---------------------------
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(joint_state_publisher_gui),
    )

    # ---------------------------
    # Launch description
    # ---------------------------
    nodes = [
        robot_state_pub_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
