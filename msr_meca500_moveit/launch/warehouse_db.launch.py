from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_warehouse_db_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("meca500", package_name="msr_meca500_moveit").to_moveit_configs()
    return generate_warehouse_db_launch(moveit_config)
