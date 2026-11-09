from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    workspace_root = Path(__file__).resolve().parents[5]

    config_file = workspace_root / "config" / "vehicle.yaml"

    if not config_file.is_file():
        raise FileNotFoundError(
            f"Vehicle configuration not found: {config_file}"
        )

    return LaunchDescription([
        Node(
            package="putm_controller",
            executable="controller",
            name="controller",
            parameters=[str(config_file)],
            output="screen",
        )
    ])