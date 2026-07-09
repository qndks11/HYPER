import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg: Image):
        self.get_logger().info(
            f'Received image: {msg.width}x{msg.height}, encoding={msg.encoding}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
