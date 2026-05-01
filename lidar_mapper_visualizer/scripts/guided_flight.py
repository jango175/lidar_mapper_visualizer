#!/usr/bin/env python3

"""
Guided flight ROS 2 package.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.qos import qos_profile_sensor_data


class GuidedFlight(Node):
  """
  Guided Flight class.
  """

  def __init__(self):
    """
    GuidedFlight constructor.
    """

    super().__init__('guided_flight')

    self.timer_period: float = 0.1
    self.position: PoseStamped = PoseStamped()
    self.waypoints: list[Pose] = []
    self.waypoint_num: int = 0

    self.generate_waypoints()

    # Odometry subscription
    self.odom_sub: Subscription = self.create_subscription(
      Odometry,
      '/mavros/local_position/odom',
      self.odom_callback,
      qos_profile_sensor_data
    )

    # Setpoint publisher
    self.setpoint_pub: Publisher = self.create_publisher(
      PoseStamped,
      '/mavros/setpoint_position/local',
      qos_profile_sensor_data
    )

    self.setpoint_timer: Timer = self.create_timer(
      self.timer_period,
      self.timer_callback
    )

    self.get_logger().info('Guided flight active!')


  def odom_callback(self, msg: Odometry):
    """
    Odometry callback.

    :param msg: Odometry message.
    """

    self.position.header = msg.header
    self.position.pose.position.x = msg.pose.pose.position.x
    self.position.pose.position.y = msg.pose.pose.position.y
    self.position.pose.position.z = msg.pose.pose.position.z
    self.position.pose.orientation.w = msg.pose.pose.orientation.w
    self.position.pose.orientation.x = msg.pose.pose.orientation.x
    self.position.pose.orientation.y = msg.pose.pose.orientation.y
    self.position.pose.orientation.z = msg.pose.pose.orientation.z


  def generate_waypoints(self):
    """
    Generate new waypoints.
    """

    init_x: float = 0.0
    init_y: float = 0.0
    init_z: float = 2.5
    R: float = 3.0
    R_z: float = 1.5
    div: int = 10
    div_z: int = 4

    for i in range(div):
      theta = i * 2.0 * math.pi / div

      for j in range(div_z):
        theta_z = j * 2.0 * math.pi / div_z

        pos: Pose = Pose()
        pos.position.x = init_x + R * math.cos(theta)
        pos.position.y = init_y + R * math.sin(theta)
        pos.position.z = init_z + R_z * math.sin(theta_z)
        pos.orientation.w = math.cos((theta + math.pi) / 2.0)
        pos.orientation.x = 0.0
        pos.orientation.y = 0.0
        pos.orientation.z = math.sin((theta + math.pi) / 2.0)

        self.waypoints.append(pos)

    # pos_1: Pose = Pose()
    # pos_1.position.x = 10.0
    # pos_1.position.y = 10.0
    # pos_1.position.z = 10.0
    # pos_1.orientation.w = 1.0
    # pos_1.orientation.x = 0.0
    # pos_1.orientation.y = 0.0
    # pos_1.orientation.z = 0.0
    # self.waypoints.append(pos_1)

    # pos_2: Pose = Pose()
    # pos_2.position.x = 5.0
    # pos_2.position.y = 5.0
    # pos_2.position.z = 5.0
    # pos_2.orientation.w = -1.0
    # pos_2.orientation.x = 0.0
    # pos_2.orientation.y = 0.0
    # pos_2.orientation.z = 0.0
    # self.waypoints.append(pos_2)

    # pos_3: Pose = Pose()
    # pos_3.position.x = 0.0
    # pos_3.position.y = 0.0
    # pos_3.position.z = 0.0
    # pos_3.orientation.w = 1.0
    # pos_3.orientation.x = 0.0
    # pos_3.orientation.y = 0.0
    # pos_3.orientation.z = 0.0
    # self.waypoints.append(pos_3)


  def set_waypoints(self, new_waypoints: list[Pose]):
    """
    Set new waypoints to follow.

    :param new_waypoints: List of new waypoints.
    """

    self.waypoint_num = 0
    self.waypoints = new_waypoints.copy()


  def timer_callback(self):
    """
    Timer callback.
    """

    if self.waypoints == []:
      return

    if (self.position.pose.position.x - self.waypoints[self.waypoint_num].position.x)**2 + \
       (self.position.pose.position.y - self.waypoints[self.waypoint_num].position.y)**2 + \
       (self.position.pose.position.z - self.waypoints[self.waypoint_num].position.z)**2 > 0.025:

      waypoint_msg: PoseStamped = PoseStamped()
      waypoint_msg.header.stamp = self.get_clock().now().to_msg()
      waypoint_msg.header.frame_id = self.position.header.frame_id
      waypoint_msg.pose.position.x = self.waypoints[self.waypoint_num].position.x
      waypoint_msg.pose.position.y = self.waypoints[self.waypoint_num].position.y
      waypoint_msg.pose.position.z = self.waypoints[self.waypoint_num].position.z
      waypoint_msg.pose.orientation.w = self.waypoints[self.waypoint_num].orientation.w
      waypoint_msg.pose.orientation.x = self.waypoints[self.waypoint_num].orientation.x
      waypoint_msg.pose.orientation.y = self.waypoints[self.waypoint_num].orientation.y
      waypoint_msg.pose.orientation.z = self.waypoints[self.waypoint_num].orientation.z

      self.setpoint_pub.publish(waypoint_msg)
    else:
      if self.waypoint_num < len(self.waypoints) - 1:
        self.get_logger().info(f"Waypoint {self.waypoint_num + 1} reached!")
        self.waypoint_num = self.waypoint_num + 1
      else:
        self.waypoint_num = 0


def main(args: list[str] | None = None):
  """
  Main function.

  :param args: Main arguments.
  """

  rclpy.init(args=args)
  node = GuidedFlight()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


# Entry point
if __name__ == '__main__':
  main()
