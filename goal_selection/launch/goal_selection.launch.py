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

    goal_selection_node = Node(
        package='goal_selection',
        executable='goal_selection_node',
        name='goal_selection',
        parameters=params,
        output='screen',    # Show output in terminal
        emulate_tty=True    # Enable colors and formatting
    )

    nodes = [goal_selection_node]
    return LaunchDescription(nodes)