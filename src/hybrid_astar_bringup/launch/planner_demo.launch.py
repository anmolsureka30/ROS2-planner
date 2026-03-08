"""
Launch file for Hybrid A* Planner demo.
Launches: map_server + planner_server + manual lifecycle transitions + (optional) rviz2

Usage:
  ros2 launch hybrid_astar_bringup planner_demo.launch.py
  ros2 launch hybrid_astar_bringup planner_demo.launch.py use_rviz:=true
  ros2 launch hybrid_astar_bringup planner_demo.launch.py map:=/path/to/map.yaml

Note: Uses manual lifecycle transitions via shell script instead of
nav2_lifecycle_manager, which has timing issues under Docker/Rosetta.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('hybrid_astar_bringup')

    # Launch arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'test_field.yaml'),
        description='Full path to map yaml file')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
        description='Full path to Nav2 params file')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 (requires X11 display)')

    # Static transform: map -> base_link (for testing without a robot)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_base',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'base_link'])

    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'yaml_filename': LaunchConfiguration('map'),
             'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])

    # Planner server with our Hybrid A* plugin
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])

    # Manual lifecycle transitions (delayed 8s for DDS discovery)
    # This replaces nav2_lifecycle_manager which has timeout issues in Docker.
    # Uses state-checking loop: verifies the node reaches the target state,
    # retrying transitions until it does. This handles stale DDS nodes correctly.
    lifecycle_bash = (
        'ensure_state() { '
        '  local node=$1 target_id=$2 transition=$3; '
        '  for attempt in 1 2 3 4 5; do '
        '    state=$(ros2 lifecycle get "$node" 2>&1 | head -1); '
        '    state_id=$(echo "$state" | grep -oP "\\[\\K[0-9]+"); '
        '    echo "  $node: state=$state (want id=$target_id)"; '
        '    if [ "$state_id" = "$target_id" ]; then '
        '      echo "  $node: OK"; return 0; '
        '    fi; '
        '    echo "  $node: applying $transition (attempt $attempt)..."; '
        '    ros2 lifecycle set "$node" "$transition" 2>&1; '
        '    sleep 3; '
        '  done; '
        '  echo "  WARNING: $node failed"; return 1; '
        '}; '
        'echo "=== Bringing up nodes ==="; '
        'ensure_state /map_server 2 configure; '
        'ensure_state /map_server 3 activate; '
        'ensure_state /planner_server 2 configure; '
        'ensure_state /planner_server 3 activate; '
        'echo "=== Final states ==="; '
        'ros2 lifecycle get /map_server 2>&1 | head -1; '
        'ros2 lifecycle get /planner_server 2>&1 | head -1; '
        'echo "=== All nodes ready ==="'
    )
    lifecycle_script = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c', lifecycle_bash],
                output='screen')
        ])

    # RViz (optional — only if display is available)
    rviz_config = os.path.join(bringup_dir, 'config', 'planner_only.rviz')
    rviz = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [])

    return LaunchDescription([
        map_arg,
        params_arg,
        use_sim_time_arg,
        use_rviz_arg,
        static_tf,
        map_server,
        planner_server,
        lifecycle_script,
        rviz,
    ])
