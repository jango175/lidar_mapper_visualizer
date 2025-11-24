from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
  lidar_mapper_rviz_node = Node(
    package = 'lidar_mapper_visualiser',
    executable = 'lidar_mapper_rviz',
    name = 'lidar_mapper_rviz',
    output = 'screen',
    parameters = [
      {'timestamp_diff_threshold': 0.05},
      {'lidar_pitch_deg': -30.0},
      {'play_bag': True},
      {'use_ned': True}
    ]
  )

  rviz_config_file = PathJoinSubstitution([
    FindPackageShare('lidar_mapper_visualiser'),
    'config',
    'lidar_mapper_config.rviz'
  ])

  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file]
  )

  ld = LaunchDescription()
  ld.add_action(lidar_mapper_rviz_node)
  ld.add_action(rviz_node)

  return ld
