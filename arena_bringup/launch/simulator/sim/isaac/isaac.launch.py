import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import time
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
import launch.event_handlers
import launch.actions


def generate_launch_description():
    logger = launch.substitutions.LaunchConfiguration("log_level")
    sim_lock = LaunchConfiguration('sim_lock', default='/tmp/arena_sim.lock')

    def _create_lock(context):
        lock_path = launch.utilities.perform_substitutions(context, [sim_lock])
        try:
            with open(lock_path, 'w') as f:
                f.write(f"{os.getpid()} {int(time.time())}\n")
        except Exception:
            pass

    def _cleanup_lock(context):
        lock_path = launch.utilities.perform_substitutions(context, [sim_lock])
        try:
            if os.path.exists(lock_path):
                os.remove(lock_path)
        except Exception:
            pass

    create_lock_action = OpaqueFunction(function=_create_lock)
    cleanup_handler = launch.event_handlers.RegisterEventHandler(
        launch.event_handlers.OnShutdown(on_shutdown=[OpaqueFunction(function=_cleanup_lock)])
    )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            "log_level",
            default_value=["debug"],
            description="Logging level",
        ),
        launch.actions.DeclareLaunchArgument(
            name='sim_lock',
            default_value='/tmp/arena_sim.lock',
            description='Path to simulator lock file'
        ),
        create_lock_action,
        Node(
            package='ros2isaacsim',
            executable='run_isaacsim',
            # output='screen',
            # arguments=['--ros-args', '--log-level', logger]
        ),
        cleanup_handler,
    ])
