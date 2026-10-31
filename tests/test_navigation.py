from genesis_mobile_sim.bringup import navigation_parameters
from genesis_mobile_sim.policy import MAX_ANGULAR, MAX_LINEAR


def test_navigation_uses_live_lidar_instead_of_a_missing_depth_pointcloud():
    nav = {
        "controller_server": {
            "ros__parameters": {"FollowPath": {}, "robot_base_frame": "base_link"}
        },
        "velocity_smoother": {"ros__parameters": {}},
        **{
            name: {name: {"ros__parameters": {"voxel_layer": {}}}}
            for name in ("local_costmap", "global_costmap")
        },
    }
    config = navigation_parameters(nav, {"slam_toolbox": {"ros__parameters": {}}})
    for name in ("local_costmap", "global_costmap"):
        params = config[name][name]["ros__parameters"]
        assert "voxel_layer" not in params
        assert params["robot_radius"] == 0.45
        scan = params["obstacle_layer"]["scan"]
        assert scan["topic"] == "/scan"
        assert scan["marking"] and scan["clearing"] and scan["inf_is_valid"]
    assert config["local_costmap"]["local_costmap"]["ros__parameters"]["rolling_window"]
    assert config["slam_toolbox"]["ros__parameters"]["use_sim_time"]
    assert config["slam_toolbox"]["ros__parameters"]["minimum_time_interval"] == 0.1
    assert config["velocity_smoother"]["ros__parameters"]["max_velocity"] == [
        MAX_LINEAR,
        0,
        MAX_ANGULAR,
    ]

    assert config["controller_server"]["ros__parameters"]["robot_base_frame"] == "base"
    assert config["slam_toolbox"]["ros__parameters"]["base_frame"] == "base"
