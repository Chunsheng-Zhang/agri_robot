import importlib.util
from pathlib import Path

import yaml


def test_vendor_launch_selection_without_starting_devices(tmp_path, monkeypatch):
    import launch.logging
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
    from launch_ros.actions import Node, SetParameter
    from launch.utilities import perform_substitutions
    monkeypatch.setattr(launch.logging.launch_config, '_log_dir', str(tmp_path))
    root = Path(__file__).resolve().parents[1]
    file = root / 'agri_robot_ws/src/agri_bringup/launch/system.launch.py'
    spec = importlib.util.spec_from_file_location('device_launch', file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(start_mgx23='false', start_joy='true', start_lidar='true',
                                         start_camera='true', start_video='true', chassis='yahboom_x3')
    actions = module.nodes(context)
    packages = [node.node_package if isinstance(node.node_package, str)
                else perform_substitutions(context, node.node_package)
                for node in actions if isinstance(node, Node)]
    assert packages.count('yahboomcar_bringup') == 1
    assert packages.count('joy') == 1
    assert 'ydlidar_ros2_driver' in packages and 'web_video_server' in packages
    assert 'yahboomcar_base_node' not in packages
    assert 'agri_waypoint' in packages and 'agri_path_tracking' in packages and 'agri_path_planning' in packages
    context.launch_configurations['start_planner'] = 'false'
    minimal = module.nodes(context)
    minimal_packages = [perform_substitutions(context, node.node_package) if not isinstance(node.node_package, str)
                        else node.node_package for node in minimal if isinstance(node, Node)]
    assert 'agri_path_planning' not in minimal_packages and 'opennav_coverage' not in minimal_packages
    assert 'agri_path_tracking' in minimal_packages
    camera_groups = [action for action in actions if isinstance(action, GroupAction)]
    group = next(g for g in camera_groups if any(isinstance(a, SetParameter) for a in g.get_sub_entities()))
    include = next(a for a in group.get_sub_entities() if isinstance(a, IncludeLaunchDescription))
    description = include.launch_description_source.get_launch_description(context)
    assert description.entities  # XML parsed successfully, no node executed.
    assert 'astro_pro_plus.launch.xml' in str(include.launch_description_source.location)
    config = yaml.safe_load((root / 'agri_robot_ws/src/agri_bringup/config/system.yaml').read_text())
    assert config['safety']['ros__parameters']['require_lidar']
    assert config['joy_adapter']['ros__parameters']['joy_profile'] == 'jetson'
