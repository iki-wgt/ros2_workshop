import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    share = get_package_share_directory("room_sim")
    world_path = f"{share}/worlds/room_two_rgbd.sdf"
    bridge_cfg = f"{share}/config/bridge_rgbd.yaml"

    if shutil.which("gz"):
        sim_cmd = [
            "gz",
            "sim",
            "-r",
            "-s",
            world_path,
            "--render-engine",
            "ogre2",
            "--headless-rendering",
        ]
    elif shutil.which("ign"):
        sim_cmd = [
            "ign",
            "gazebo",
            "-r",
            "-s",
            world_path,
            "--render-engine",
            "ogre2",
            "--headless-rendering",
        ]
    else:
        sim_cmd = ["bash", "-lc", "echo 'No ign/gz CLI in PATH' && false"]

    gz_bridge = ComposableNode(
        package="ros_gz_bridge",
        plugin="ros_gz_bridge::RosGzBridge",
        name="gz_bridge",
        parameters=[{"config_file": bridge_cfg}],
    )

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=sim_cmd,
                output="screen",
                additional_env={"QT_QPA_PLATFORM": "offscreen"},
            ),
            ComposableNodeContainer(
                name="room_sim_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                composable_node_descriptions=[gz_bridge],
            ),
        ]
    )
