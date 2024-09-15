from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument,ExecuteProcess,TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    launch_description = LaunchDescription()
    
    """Define"""
    turtle_bringup_package = 'turtle_bringup_plus'
    copy_namespace = 'copy_windows'
    turtle_name = ['iron','humble','foxy','neotic']

    turtlesim_node = Node(
            package='turtlesim_plus',
            namespace=copy_namespace,
            executable='turtlesim_plus_node.py',
            name='copy_windows'
        )
    launch_description.add_action(turtlesim_node)

    kill_turtle1 = ExecuteProcess(
            cmd=[
                'ros2 service call ',
                f'/{copy_namespace}/remove_turtle ',
                'turtlesim/srv/Kill ',
                "\"{name: 'turtle1'}\""  # Ensures that this service call is correct
            ],
            shell=True
        )
    launch_description.add_action(kill_turtle1)

    for i in range (len(turtle_name)):
        spawn_turtle = ExecuteProcess(
                cmd=[
                    'ros2 service call ',
                    f'/{copy_namespace}/spawn_turtle ',
                    'turtlesim/srv/Spawn ',
                    f'"{{x: 0.1, y: 0.1, theta: 0.0, name: {turtle_name[i]}}}"'
                ],
                shell=True
            )
        launch_description.add_action(spawn_turtle)

        copy_schedule_node = Node(
            package=turtle_bringup_package,
            name=f'copy_schedule_{turtle_name[i]}',
            namespace=copy_namespace,
            executable='copy_schedule.py',
            parameters=[
                {'turtle_name':turtle_name[i]},
                {'my_copy_index':i+1}
            ]
        )
        launch_description.add_action(copy_schedule_node)

        controller_node = Node(
            package=turtle_bringup_package,
            name=f'controller_node_{turtle_name[i]}',
            namespace=copy_namespace,
            executable='controller_node.py',
            parameters=[
                # {'turtltsim_namespace':test_arry_ns[0]},
                {'turtle_name':turtle_name[i]}
            ]
        )
        launch_description.add_action(controller_node)

    return launch_description