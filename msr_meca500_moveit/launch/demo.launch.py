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
        DeclareLaunchArgument("use_extruder", default_value="true",
            description="Attach the extruder end-effector meshes (true) or bare flange (false)"),
        DeclareLaunchArgument("extruder_urdf", default_value="ender3_extruder.urdf",
            description="Extruder urdf filename, relative to msr_meca500_robot/urdf/"),
        DeclareLaunchArgument("use_background", default_value="true",
            description="Attach the Ender3 background/environment mesh (true) or bare robot (false)"),
        DeclareLaunchArgument("background_urdf", default_value="ender3_environment.urdf",
            description="Background environment urdf filename, relative to msr_meca500_robot/urdf/"),
        # Mount offset of the background environment relative to link_0__base,
        # recovered from the combined onshape-to-robot export (see
        # msr_meca500_robot/urdf/ender3_environment.urdf for the derivation).
        DeclareLaunchArgument("env_x", default_value="-0.1034567"),
        DeclareLaunchArgument("env_y", default_value="-0.0740003"),
        DeclareLaunchArgument("env_z", default_value="-0.08937016"),
        DeclareLaunchArgument("env_roll", default_value="0"),
        DeclareLaunchArgument("env_pitch", default_value="0"),
        DeclareLaunchArgument("env_yaw", default_value="3.14159"),
    ]
    moveit_config = (
        MoveItConfigsBuilder("meca500", package_name="msr_meca500_moveit")
        .robot_description(mappings={
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
            "robot_ip":          LaunchConfiguration("robot_ip"),
            "control_port":      LaunchConfiguration("control_port"),
            "monitoring_port":   LaunchConfiguration("monitoring_port"),
            "use_extruder":      LaunchConfiguration("use_extruder"),
            "extruder_urdf":     LaunchConfiguration("extruder_urdf"),
            "use_background":    LaunchConfiguration("use_background"),
            "background_urdf":   LaunchConfiguration("background_urdf"),
            "env_x":             LaunchConfiguration("env_x"),
            "env_y":             LaunchConfiguration("env_y"),
            "env_z":             LaunchConfiguration("env_z"),
            "env_roll":          LaunchConfiguration("env_roll"),
            "env_pitch":         LaunchConfiguration("env_pitch"),
            "env_yaw":           LaunchConfiguration("env_yaw"),
        })
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl",
        )
        .to_moveit_configs()
    )

    demo_launch = generate_demo_launch(moveit_config)
    return LaunchDescription(declared_args + demo_launch.entities)