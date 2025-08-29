from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('xg', default_value='1.0'),
        DeclareLaunchArgument('yg', default_value='0.0'),
        DeclareLaunchArgument('thetag', default_value='0.0'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        Node(
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
    ])
