import rclpy
from rclpy.node import Node
from tf2_ros import TransformException, Buffer, TransformListener

# Change these to the frames you want to track
FROM_FRAME = 'drone_base_link' 
TO_FRAME = 'ldlidar_link'


class ExactTfEcho(Node):
  def __init__(self):
    super().__init__('exact_tf_echo')
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
    # Check the transform every 1 second
    self.timer = self.create_timer(1.0, self.on_timer)


  def on_timer(self):
    try:
      # Look up the transform
      t = self.tf_buffer.lookup_transform(FROM_FRAME, TO_FRAME, rclpy.time.Time())
      trans = t.transform.translation
      rot = t.transform.rotation

      # Print with 8 decimal places (change the .8f to whatever you need!)
      self.get_logger().info(f"\nTranslation: x: {trans.x:.5f}, y: {trans.y:.5f}, z: {trans.z:.5f}")
      self.get_logger().info(f"Rotation(q): x: {rot.x:.12f}, y: {rot.y:.12f}, z: {rot.z:.12f}, w: {rot.w:.12f}")

    except TransformException as ex:
        self.get_logger().info(f"Waiting for transform... {ex}")


def main():
  rclpy.init()
  node = ExactTfEcho()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
