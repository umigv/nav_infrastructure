from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    planner_server_node = Node(
        package='controller_server',
        executable='controller_server',
        name='controller_server',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('controller_server'),
                'config',
                'controller_server_params.yaml'
            ])
        ],
        output='screen',    # Show output in terminal
        emulate_tty=True    # Enable colors and formatting
    )

    nodes = [planner_server_node]
    return LaunchDescription(nodes)