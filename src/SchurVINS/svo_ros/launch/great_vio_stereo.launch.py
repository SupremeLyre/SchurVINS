from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _launch_setup(context, *args, **kwargs):
    share_dir = Path(get_package_share_directory("svo_ros"))
    calib_file = LaunchConfiguration("calib_file").perform(context)
    trace_dir = Path(LaunchConfiguration("trace_dir").perform(context)).expanduser()
    trace_dir.mkdir(parents=True, exist_ok=True)
    params_file = Path(LaunchConfiguration("params_file").perform(context))
    rviz_config = share_dir / "rviz_config.rviz"
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in (
        "1",
        "true",
        "yes",
        "on",
    )

    params = _load_yaml(params_file)
    params.update(
        {
            "cam0_topic": LaunchConfiguration("cam0_topic").perform(context),
            "cam1_topic": LaunchConfiguration("cam1_topic").perform(context),
            "imu_topic": LaunchConfiguration("imu_topic").perform(context),
            "calib_file": calib_file,
            "trace_dir": str(trace_dir),
            "runlc": False,
            "use_sim_time": use_sim_time,
        }
    )

    return [
        Node(
            package="svo_ros",
            executable="svo_node",
            name="svo",
            output="screen",
            parameters=[params],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="vis",
            output="screen",
            arguments=["-d", str(rviz_config)],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]


def generate_launch_description():
    share_dir = Path(get_package_share_directory("svo_ros"))
    default_calib = share_dir / "param" / "calib" / "great_stereo.yaml"
    default_params = share_dir / "param" / "vio_stereo_great.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument("calib_file", default_value=str(default_calib)),
            DeclareLaunchArgument("params_file", default_value=str(default_params)),
            DeclareLaunchArgument("cam0_topic", default_value="/img0_raw"),
            DeclareLaunchArgument("cam1_topic", default_value="/img1_raw"),
            DeclareLaunchArgument("imu_topic", default_value="/imu_raw"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("trace_dir", default_value="/tmp/svo_trace"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
