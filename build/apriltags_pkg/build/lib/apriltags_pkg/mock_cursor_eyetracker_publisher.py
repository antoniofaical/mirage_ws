import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class MockCursorEyetrackerPublisher(Node):
    def __init__(self):
        super().__init__('mock_cursor_eyetracker_publisher')
        self.publisher_ = self.create_publisher(PointStamped, '/target_info/position', 10)
        self.subscription = self.create_subscription(
            PointStamped,
            '/mouse/relative_position',
            self.cursor_callback,
            10
        )
        self.get_logger().info("MockCursorEyetrackerPublisher está rodando.")

    def cursor_callback(self, msg):
        new_msg = PointStamped()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = msg.header.frame_id
        new_msg.point.x = msg.point.x
        new_msg.point.y = msg.point.y
        new_msg.point.z = msg.point.z

        self.publisher_.publish(new_msg)
        self.get_logger().info(
            f"Replicando cursor: x={msg.point.x:.1f}px, y={msg.point.y:.1f}px"
        )

def main(args=None):
    rclpy.init(args=args)
    node = MockCursorEyetrackerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
