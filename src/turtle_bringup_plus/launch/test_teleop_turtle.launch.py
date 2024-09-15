from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument,ExecuteProcess,TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    launch_description = LaunchDescription()
    
    """Define"""
    turtle_bringup_package = 'turtle_bringup_plus'
    teleop_namespace = 'tempate_windows'
    turtle_name = ['FIBO','BIBO']

    turtlesim_node = Node(
            package='turtlesim_plus',
            namespace=teleop_namespace,
            executable='turtlesim_plus_node.py',
            name='copy_windows',
            # remappings=[
            #     ('/custom_key/cmd_vel', f'/{teleop_namespace}/{turtle_name[0]}/cmd_vel')
            # ]
        )
    launch_description.add_action(turtlesim_node)

    kill_turtle1 = ExecuteProcess(
            cmd=[
                'ros2 service call ',
                f'/{teleop_namespace}/remove_turtle ',
                'turtlesim/srv/Kill ',
                "\"{name: 'turtle1'}\""  # Ensures that this service call is correct
            ],
            shell=True
        )
    launch_description.add_action(kill_turtle1)

    spawn_turtle = ExecuteProcess(
                cmd=[
                    'ros2 service call ',
                    f'/{teleop_namespace}/spawn_turtle ',
                    'turtlesim/srv/Spawn ',
                    f'"{{x: 0.1, y: 0.1, theta: 0.0, name: {turtle_name[0]}}}"'
                ],
                shell=True
            )
    launch_description.add_action(spawn_turtle)

    teleop_schedule_node = Node(
            package=turtle_bringup_package,
            name=f'teleop_scheduler_{turtle_name[0]}',
            namespace=teleop_namespace,
            executable='teleop_scheduler.py',
            parameters=[
                {'turtle_name':turtle_name[0]},
            ]
        )
    launch_description.add_action(teleop_schedule_node)

    controller_node = Node(
            package=turtle_bringup_package,
            name=f'controller_node_{turtle_name[0]}',
            namespace=teleop_namespace,
            executable='controller_node.py',
            parameters=[
                {'turtle_name':turtle_name[0]},
                {'isteleop':True}
            ],
            # remappings=[
            #     ('/custom_key/cmd_vel', f'/{teleop_namespace}/{turtle_name[0]}/cmd_vel')
            # ]
        )
    launch_description.add_action(controller_node)
    


    return launch_description