from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    ld = LaunchDescription()

    robot_node = ExecuteProcess(
        cmd=[
            'taskset', '-c', '4',  # This sets the affinity to CPU core 0
            'ros2', 'run', 'robot', 'run',
            # '--ros-args',
            '-p', 'Kp:=[20.0, 20.0, 00.0, 00.0, 00.0, 00.0]',  # Correct syntax for setting parameter
            '-p', 'Ki:=[00.0, 00.0, 00.0, 00.0, 00.0, 00.0]',  # Correct syntax for setting parameter
            '--remap', '__node:=robot_node'
        ],
        output='screen',
    )

    ld.add_action(robot_node)
    return ld
