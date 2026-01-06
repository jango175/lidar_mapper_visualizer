from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  lidar_mapper_rviz_node = Node(
    package = 'lidar_mapper_visualiser',
    executable = 'lidar_mapper_rviz',
    name = 'lidar_mapper_rviz',
    output = 'screen',
    parameters = [
      {'timestamp_diff_threshold': 0.05},
      {'interpolation_timestamp_threshold': 0.25},
      {'use_ned': True}
    ]
  )

  # read the URDF file content
  pkg_share = get_package_share_directory('lidar_mapper_visualiser')
  urdf_file = os.path.join(pkg_share, 'urdf', 'lidar_mapper_drone.urdf')

  with open(urdf_file, 'r') as infp:
    robot_desc = infp.read()

  rsp_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{'robot_description': robot_desc}]
  )

  # initialize static transform publisher from 'map' to 'drone_base'
  static_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='static_transform_publisher',
    arguments=['0', '0', '0', '0', '0', '3.1416', 'map', 'drone_base']
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
  ld.add_action(rsp_node)
  ld.add_action(static_tf_node)
  ld.add_action(rviz_node)

  return ld
