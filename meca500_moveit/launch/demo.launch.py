from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument("use_mock_hardware", default_value="true",
            description="Use mock hardware (true) or real Meca500 (false)"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.0.100",
            description="IP address of the Meca500 robot"),
        DeclareLaunchArgument("control_port", default_value="10000",
            description="Meca500 control port"),
        DeclareLaunchArgument("monitoring_port", default_value="10001",
            description="Meca500 monitoring port"),
        DeclareLaunchArgument("use_rviz", default_value="true",
            description="Launch RViz (true) or headless (false)"),
    ]

    moveit_config = (
        MoveItConfigsBuilder("meca500", package_name="meca500_moveit")
        .robot_description(mappings={
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
            "robot_ip":          LaunchConfiguration("robot_ip"),
            "control_port":      LaunchConfiguration("control_port"),
            "monitoring_port":   LaunchConfiguration("monitoring_port"),
        })
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="pilz_industrial_motion_planner",
        )
        .to_moveit_configs()
    )

    demo_launch = generate_demo_launch(moveit_config)
    return LaunchDescription(declared_args + demo_launch.entities)