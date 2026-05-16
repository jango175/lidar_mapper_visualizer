from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():
  use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value = 'true',
    description = 'Use simulation (bag) clock'
  )

  world_link_arg = DeclareLaunchArgument(
    'world_link',
    default_value = 'map',
    description = 'World link name'
  )

  drone_link_arg = DeclareLaunchArgument(
    'drone_link',
    default_value = 'base_link',
    description = 'Drone link name'
  )

  lidar_link_arg = DeclareLaunchArgument(
    'lidar_link',
    default_value = 'ldlidar_link',
    description = 'LiDAR link name'
  )

  use_sim_time_param = LaunchConfiguration('use_sim_time')
  world_link = LaunchConfiguration('world_link')
  drone_link = LaunchConfiguration('drone_link')
  lidar_link = LaunchConfiguration('lidar_link')

  lidar_mapper_rviz_node = Node(
    package = 'lidar_mapper_visualizer',
    executable = 'lidar_mapper_rviz',
    name = 'lidar_mapper_rviz',
    output = 'screen',
    # prefix = ['gnome-terminal -- gdb -ex run --args'], # debugger
    parameters = [{
      'lidar_mount_roll_deg': 0.0,
      'lidar_mount_pitch_deg': 30.0,
      'lidar_mount_yaw_deg': 0.0,
      'lidar_mount_offset_x': 0.088,
      'lidar_mount_offset_y': 0.0,
      'lidar_mount_offset_z': 0.073,
      'world_link': world_link,
      'drone_link': drone_link,
      'lidar_link': lidar_link,
      'mf_timeout': 0.25,
      'timestamp_tolerance': 0.11, # should be smaller than mf_timeout
      'window_size': 10.0,
      'sor_mean_k': 50,
      'sor_std_dev_mult': 1.0,
      'fake_3d_lidar_scan_num': 15,
      'fake_3d_lidar_overlap_scan_num': 10, # should be smaller than fake_3d_lidar_scan_num
      'simulate_noise': False,
      'use_sim_time': use_sim_time_param
    }]
  )

  # read the URDF file content
  pkg_share = get_package_share_directory('lidar_mapper_visualizer')
  urdf_file = os.path.join(pkg_share, 'urdf', 'lidar_mapper_drone.urdf')

  with open(urdf_file, 'r') as infp:
    robot_desc = infp.read()

  rsp_node = Node(
    package = 'robot_state_publisher',
    executable = 'robot_state_publisher',
    name = 'robot_state_publisher',
    output = 'screen',
    parameters = [{
      'robot_description': robot_desc,
      'use_sim_time': use_sim_time_param
    }]
  )

  # initialize static transform publisher from 'map' to 'base_link'
  static_tf_node = Node(
    package = 'tf2_ros',
    executable = 'static_transform_publisher',
    name = 'static_transform_publisher',
    arguments = ['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'base_link'],
    parameters=[{
      'use_sim_time': use_sim_time_param
    }]
  )

  rviz_config_file = PathJoinSubstitution([
    FindPackageShare('lidar_mapper_visualizer'),
    'config',
    'lidar_mapper_config.rviz'
  ])

  rviz_node = Node(
    package = 'rviz2',
    executable = 'rviz2',
    name = 'rviz2',
    output = 'screen',
    arguments = ['-d', rviz_config_file],
    parameters = [{
      'use_sim_time': use_sim_time_param
    }]
  )

  octomap_node = Node(
    package = 'octomap_server',
    executable = 'octomap_server_node',
    name = 'octomap_server',
    output = 'screen',
    parameters = [{
      'resolution': 0.15,
      'frame_id': world_link,
      'base_frame_id': drone_link,
      'sensor_model.max_range': 12.0,
      'latch': True,
      'use_sim_time': use_sim_time_param
    }],
    remappings = [
      ('cloud_in', '/lidar_mapper_visualizer/sync_slice_point_cloud')
    ]
  )

  ld = LaunchDescription()

  ld.add_action(use_sim_time_arg)
  ld.add_action(world_link_arg)
  ld.add_action(drone_link_arg)
  ld.add_action(lidar_link_arg)

  ld.add_action(lidar_mapper_rviz_node)
  ld.add_action(rsp_node)
  # ld.add_action(static_tf_node) # for static debug
  ld.add_action(rviz_node)
  ld.add_action(octomap_node)

  return ld
