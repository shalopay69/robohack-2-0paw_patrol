from pathlib import Path

from launch import LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    root = Path(__file__).resolve().parent
    namespace = 't5_nav2'
    params = RewrittenYaml(
        source_file=str(root / 't5_nav2.yaml'), root_key=namespace,
        param_rewrites={}, convert_types=True,
    )
    processes = [
        ExecuteProcess(cmd=['python3', str(root / file)], output='screen')
        for file in ('t4_node.py', 't5_map.py', 't5_aruco.py', 't5_goal.py', 't5_velocity_gate.py')
    ]
    processes += [
        Node(package=package, executable=name, name=name, namespace=namespace, parameters=[params],
             remappings=[('cmd_vel', '/t5_cmd_vel_requested')], output='screen')
        for package, name in (('nav2_controller', 'controller_server'), ('nav2_planner', 'planner_server'),
                              ('nav2_behaviors', 'behavior_server'))
    ]
    navigator = {'default_nav_to_pose_bt_xml': str(root / 't5_navigate.xml'),
                 'default_nav_through_poses_bt_xml': str(root / 't5_navigate_through.xml')}
    processes += [
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             namespace=namespace, parameters=[params, navigator],
             output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', namespace=namespace, parameters=[params],
             output='screen'),
    ]
    watch = [
        RegisterEventHandler(OnProcessExit(
            target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='A navigation process exited'))],
        ))
        for process in processes
    ]
    return LaunchDescription([*watch, *processes])
