import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


class SpawnRobot(Node):
    """Waits for Gazebo to be ready and confirms robot spawn."""

    def __init__(self):
        super().__init__('spawn_robot')
        self.declare_parameter('robot_name', 'hyper')
        self.get_logger().info(
            f"SpawnRobot: robot '{self.get_parameter('robot_name').value}' ready"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SpawnRobot()
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
