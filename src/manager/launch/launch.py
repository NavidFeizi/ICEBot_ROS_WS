from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    ld = LaunchDescription()

    manager_node = ExecuteProcess(
        cmd=[
            'taskset', '-c', '6',  # This sets the affinity to CPU core 2
            'ros2', 'run', 'manager', 'manage',
            '--ros-args',
            '-p', 'sample_time:=0.004',  
            '-p', 'task_space_mode:=false',  
            '--remap', '__node:=Manager_node'
        ],
        output='screen',
    )

    task_target_sim_node = ExecuteProcess(
        cmd=[
            'taskset', '-c', '8',  # This sets the affinity to CPU core 4
            'ros2', 'run', 'manager', 'simulator_koopman',
            '--ros-args',
            '-p', 'sample_time:=0.002', 
            '--remap', '__node:=catheter_target_sim_node'
        ],
        output='screen',
    )

    recorder_node = ExecuteProcess(
        cmd=[
            'taskset', '-c', '7',  # This sets the affinity to CPU core 3
            'ros2', 'run', 'manager', 'record',
            '--ros-args',
            '-p', 'sample_time:=0.004',  
            '--remap', '__node:=Recorder_node'
        ],
        output='screen',
    )

    # Delay the start of the node
    delay_task_target_sim_node = TimerAction(
        period=1.0,  # Delay in seconds
        actions=[task_target_sim_node],
    )

    # Delay the start of the node
    delay_manager_node = TimerAction(
        period=3.0,  # Delay in seconds
        actions=[manager_node],
    )

    ld.add_action(delay_manager_node)
    ld.add_action(recorder_node)
    # ld.add_action(delay_task_target_sim_node)
    return ld
