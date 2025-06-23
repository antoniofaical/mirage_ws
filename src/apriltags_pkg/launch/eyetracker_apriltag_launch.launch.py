from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'height',
            default_value='800',
            description='Altura da interface'
        ),

        DeclareLaunchArgument(
            'width',
            default_value='1300',
            description='largura da interface'
        ),
        DeclareLaunchArgument(
            'AP',
            default_value='0.3',
            description='Proporcao da apriltag'
        ),
        DeclareLaunchArgument(
            'DP',
            default_value='0.05',
            description='Proporcao da distancia'
        ),
        DeclareLaunchArgument(
            'GP',
            default_value='0.05',
            description='Proporcao do gripper'
        ),
        DeclareLaunchArgument(
            'GCP',
            default_value='0.05',
            description='Proporcao do circle'
        ),
        
        Node(
            package='apriltags_pkg',
            executable='einstein_visual_interface',
            output='screen',
            parameters=[{
                'interface_height': LaunchConfiguration('height'),
                'interface_width': LaunchConfiguration('width'),
                'apriltag_proportion': LaunchConfiguration('AP'),
                'distance_proportion': LaunchConfiguration('DP'),
                'gripper_proportion': LaunchConfiguration('GP'),
                'gaze_circle_proportion': LaunchConfiguration('GCP'),
            }]
        ),

        Node(
            package='apriltags_pkg',
            executable='real_eyetracker_publisher',
            output='screen',
            parameters=[{
                'apriltag_reference': 'apriltag_TAG36H11',
                'apriltag_target': 'apriltag_TAG16H5',
                'interface_height': LaunchConfiguration('height'),
                'interface_width': LaunchConfiguration('width'),
                'apriltag_proportion': LaunchConfiguration('AP'),
                'distance_proportion': LaunchConfiguration('DP'),
            }]
        ),
    ])
