from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params = [
        PathJoinSubstitution([
        FindPackageShare('infra_launch'),
        'config',
        'infra_params.yaml'
    ])]

    planner_server_node = Node(
        package='planner_server',
        executable='planner_server',
        name='planner_server',
        parameters=params,
        output='screen',    # Show output in terminal
        emulate_tty=True    # Enable colors and formatting
    )

    nodes = [planner_server_node]
    return LaunchDescription(nodes)