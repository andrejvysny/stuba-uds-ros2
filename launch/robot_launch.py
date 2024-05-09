import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='control_package',
            executable='main',
            name='main',
            output='screen',
        
        ),
        launch_ros.actions.Node(
            package='control_package',
            executable='manual',
            name='manual',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='control_package',
            executable='auto',
            name='auto',
            output='screen'
        )
    ])
