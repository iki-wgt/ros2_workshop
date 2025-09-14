from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("pointcloud_segmentation"))
    default_params = pkg_share / "config" / "params.yaml"

    params_file = DeclareLaunchArgument(
        "params_file",
        default_value=str(default_params),
        description="Path to a YAML file with grouped ROS parameters.",
    )

    comp = ComposableNode(
        package="pointcloud_segmentation",
        plugin="pointcloud_segmentation::SegmentationComponent",
        name="segmentation_component",
        parameters=[LaunchConfiguration("params_file")],
    )

    container = ComposableNodeContainer(
        name="segmentation_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[comp],
    )

    return LaunchDescription([params_file, container])
