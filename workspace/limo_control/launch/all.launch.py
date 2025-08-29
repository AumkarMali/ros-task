from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    xg = DeclareLaunchArgument('xg', default_value='2.0')
    yg = DeclareLaunchArgument('yg', default_value='1.0')
    thetag = DeclareLaunchArgument('thetag', default_value='1.57')
    odom_topic = DeclareLaunchArgument('odom_topic', default_value='/odom')
    cmd_vel_topic = DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('limo_simulation'), 'launch', 'limo.launch.py')
        )
    )

    controller = Node(
        package='limo_control',
        executable='goto_controller',
        name='goto_controller',
        output='screen',
        parameters=[{
            'x_goal': LaunchConfiguration('xg'),
            'y_goal': LaunchConfiguration('yg'),
            'theta_goal': LaunchConfiguration('thetag'),
            'k_rho': 1.0,
            'k_alpha': 2.5,
            'k_beta': -0.5,
            'v_max': 0.5,
            'w_max': 1.5,
            'pos_tol': 0.05,
            'yaw_tol': 0.1,
            'odom_topic': LaunchConfiguration('odom_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
        }]
    )

    return LaunchDescription([xg, yg, thetag, odom_topic, cmd_vel_topic, sim_launch, controller])

