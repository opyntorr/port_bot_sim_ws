# F1 — Lado "cerebro/visualizacion" para la sim de Isaac (AGV solo).
# Arranca robot_state_publisher (TF base->ruedas/sensores desde el URDF), RViz y
# teleop holonomico (-> /cmd_vel). La sim (isaac/scene_agv.py) publica /odom, TF
# odom->base_footprint, /joint_states y /clock.
#
# Correr (con isaac/scene_agv.py ya en Play, y isaac_env.sh sourceado p/ dominio 30):
#   ros2 launch /home/opyntorr/agv_uav_project_jetauto/isaac/jetauto_isaac_view.launch.py
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "assets", "jetauto.urdf")
RVIZ = os.path.join(HERE, "jetauto_f1.rviz")


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_teleop = LaunchConfiguration("use_teleop")
    joy_dev = LaunchConfiguration("joy_dev")

    with open(URDF, "r") as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_teleop", default_value="true"),
        DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),

        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="robot_state_publisher", output="screen",
            parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_description}],
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", RVIZ],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(use_rviz),
        ),
        # --- teleop holonomico (mismo mapeo que jetauto_teleop) ---
        Node(
            package="joy_linux", executable="joy_linux_node", name="joy_linux_node",
            output="screen", parameters=[{"device_name": joy_dev, "use_sim_time": use_sim_time}],
            condition=IfCondition(use_teleop),
        ),
        Node(
            package="teleop_twist_joy", executable="teleop_node", name="teleop_twist_joy_node",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "require_enable_button": True,
                "enable_button": 4,           # LB hombre-muerto
                "axis_linear.x": 1, "axis_linear.y": 0,
                "scale_linear.x": 0.15, "scale_linear.y": 0.15,
                "axis_angular.yaw": 3,
                "scale_angular.yaw": 0.4,     # conservador (igual que el robot real)
                "publish_stamped_twist": False,
            }],
            remappings=[("/cmd_vel", "/cmd_vel")],
            condition=IfCondition(use_teleop),
        ),
    ])
