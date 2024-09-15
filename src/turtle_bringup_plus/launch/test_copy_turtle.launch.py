from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument,ExecuteProcess,TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    launch_description = LaunchDescription()
    
    """Define"""
    turtle_bringup_package = 'turtle_bringup_plus'
    test_arry_ns = ['test1','test2']
    turtle_name = ['FIBO','BIBO']

    turtlesim_node = Node(
            package='turtlesim_plus',
            namespace=test_arry_ns[0],
            executable='turtlesim_plus_node.py',
            name='copy_windows'
        )
    launch_description.add_action(turtlesim_node)

    kill_turtle1 = ExecuteProcess(
            cmd=[
                'ros2 service call ',
                f'/{test_arry_ns[0]}/remove_turtle ',
                'turtlesim/srv/Kill ',
                "\"{name: 'turtle1'}\""  # Ensures that this service call is correct
            ],
            shell=True
        )
    launch_description.add_action(kill_turtle1)

    spawn_turtle = ExecuteProcess(
            cmd=[
                'ros2 service call ',
                f'/{test_arry_ns[0]}/spawn_turtle ',
                'turtlesim/srv/Spawn ',
                f'"{{x: 0.1, y: 0.1, theta: 0.0, name: {turtle_name[0]}}}"'
            ],
            shell=True
        )
    launch_description.add_action(spawn_turtle)

    test_copy_schedule_node = Node(
        package=turtle_bringup_package,
        name='test_copy_schedule',
        namespace=test_arry_ns[0],
        executable='copy_schedule.py',
        parameters=[
            {'turtltsim_namespace':test_arry_ns[0]},
            {'turtle_name':turtle_name[0]},
            {'my_copy_index':2}
        ]
    )
    launch_description.add_action(test_copy_schedule_node)

    test_controller_node = Node(
        package=turtle_bringup_package,
        name='test_controller_node',
        namespace=test_arry_ns[0],
        executable='controller_node.py',
        parameters=[
            {'turtltsim_namespace':test_arry_ns[0]},
            {'turtle_name':turtle_name[0]}
        ]
    )
    launch_description.add_action(test_controller_node)


    return launch_description