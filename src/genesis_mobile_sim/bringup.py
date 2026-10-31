"""Extend the installed Jazzy bringup; leave SLAM, planners, and lifecycle upstream."""

import sys
import tempfile
from pathlib import Path

from .policy import MAX_ANGULAR, MAX_LINEAR
from .robot_model import BASE_FRAME, robot_description


def robot_frames(value):
    if isinstance(value, dict):
        return {key: robot_frames(item) for key, item in value.items()}
    if isinstance(value, list):
        return [robot_frames(item) for item in value]
    return (
        BASE_FRAME if isinstance(value, str) and value in {"base_link", "base_footprint"} else value
    )


def navigation_parameters(nav: dict, slam: dict) -> dict:
    """Adapt upstream defaults to this robot and its real planar lidar."""
    parameters = nav["controller_server"]["ros__parameters"]
    # The Jazzy default is MPPI. Its trajectories and costmaps run on the CPU.
    follow = parameters["FollowPath"]
    follow.update(
        vx_max=MAX_LINEAR, vx_min=-MAX_LINEAR, wz_max=MAX_ANGULAR, batch_size=500, visualize=False
    )
    for name in ("local_costmap", "global_costmap"):
        costmap = nav[name][name]["ros__parameters"]
        costmap["robot_radius"] = 0.45
        costmap["plugins"] = (["static_layer"] if name == "global_costmap" else []) + [
            "obstacle_layer",
            "inflation_layer",
        ]
        costmap.pop("voxel_layer", None)
        costmap["obstacle_layer"] = {
            "plugin": "nav2_costmap_2d::ObstacleLayer",
            "observation_sources": "scan",
            "scan": {
                "topic": "/scan",
                "data_type": "LaserScan",
                "marking": True,
                "clearing": True,
                "inf_is_valid": True,
                "obstacle_max_range": 9.5,
                "raytrace_max_range": 10.0,
            },
        }
        costmap["always_send_full_costmap"] = True
    nav["local_costmap"]["local_costmap"]["ros__parameters"]["rolling_window"] = True
    smoother = nav["velocity_smoother"]["ros__parameters"]
    smoother["max_velocity"] = [MAX_LINEAR, 0.0, MAX_ANGULAR]
    smoother["min_velocity"] = [-MAX_LINEAR, 0.0, -MAX_ANGULAR]
    slam_params = slam["slam_toolbox"]["ros__parameters"]
    slam_params.update(
        use_sim_time=True,
        base_frame=BASE_FRAME,
        scan_topic="/scan",
        min_laser_range=0.08,
        max_laser_range=10.0,
        minimum_time_interval=0.1,
        map_update_interval=1.0,
        minimum_travel_distance=0.1,
        minimum_travel_heading=0.1,
    )
    nav["slam_toolbox"] = slam["slam_toolbox"]
    return robot_frames(nav)


def main() -> None:
    import yaml
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription, LaunchService
    from launch.actions import (
        EmitEvent,
        ExecuteProcess,
        IncludeLaunchDescription,
        RegisterEventHandler,
    )
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import Node

    nav_dir = Path(get_package_share_directory("nav2_bringup"))
    slam_dir = Path(get_package_share_directory("slam_toolbox"))
    nav = yaml.safe_load((nav_dir / "params/nav2_params.yaml").read_text())
    slam = yaml.safe_load((slam_dir / "config/mapper_params_online_async.yaml").read_text())
    failed = []
    service = LaunchService()
    with tempfile.TemporaryDirectory(prefix="genesis-mobile-") as directory:
        params = Path(directory) / "navigation.yaml"
        params.write_text(yaml.safe_dump(navigation_parameters(nav, slam)))
        simulation = ExecuteProcess(
            cmd=[sys.executable, "-m", "genesis_mobile_sim.ros_bridge"], output="screen"
        )

        def simulation_exited(event, context):
            if event.returncode:
                failed.append(event.returncode)
            return [EmitEvent(event=Shutdown(reason="Mobile simulation exited"))]

        service.include_launch_description(
            LaunchDescription(
                [
                    simulation,
                    RegisterEventHandler(
                        OnProcessExit(target_action=simulation, on_exit=simulation_exited)
                    ),
                    Node(
                        package="robot_state_publisher",
                        executable="robot_state_publisher",
                        parameters=[
                            {
                                "robot_description": robot_description(),
                                "use_sim_time": True,
                            }
                        ],
                        output="screen",
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(str(nav_dir / "launch/bringup_launch.py")),
                        launch_arguments={
                            "slam": "True",
                            "use_sim_time": "True",
                            "autostart": "True",
                            "use_composition": "False",
                            "params_file": str(params),
                        }.items(),
                    ),
                ]
            )
        )
        result = service.run()
    raise SystemExit(1 if failed else result)


if __name__ == "__main__":
    main()
