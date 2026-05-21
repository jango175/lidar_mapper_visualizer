import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class PoseToOdomRelay(Node):
  def __init__(self):
    super().__init__('pose_to_odom_relay')

    # Subscribe to the PoseStamped topic
    self.subscription = self.create_subscription(
      PoseStamped,
      '/mavros/local_position/pose',
      self.pose_callback,
      10
    )

    # Publish to the Odometry topic
    self.publisher = self.create_publisher(
      Odometry,
      '/mavros/local_position/odom',
      10
    )

    self.get_logger().info('Relaying: /mavros/local_position/pose to: /mavros/local_position/odom')


  def pose_callback(self, pose_msg):
    odom_msg = Odometry()

    odom_msg.header = pose_msg.header
    odom_msg.child_frame_id = 'base_link'
    odom_msg.pose.pose = pose_msg.pose

    self.publisher.publish(odom_msg)


def main(args=None):
  rclpy.init(args=args)
  node = PoseToOdomRelay()

  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
