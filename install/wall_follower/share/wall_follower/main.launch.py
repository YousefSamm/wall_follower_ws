import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    
    
    node1 = TimerAction(period = 0.0,
     actions=[
     Node(
        package='wall_follower',
        executable='wall_finder',
        name='follower',
        output='screen'
    
    )
    ]
    )
    node2= TimerAction(period=0.0,
        actions=[
            Node(
            package='wall_follower',
            executable='action_server',
            name='action',
            output='screen'
    )
    ]
    )
    node3 = TimerAction(
        period=2.0, 
        actions=[
            Node(
                package='wall_follower',
                executable='wall_follower_node',
                name='finder',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        node1,
        node2,
        node3
        
    ])
