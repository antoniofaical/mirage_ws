import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped

class MockRobotPosePublisher(Node):
    def __init__(self):
        super().__init__('mock_robot_pose_publisher')

        self.publisher_ = self.create_publisher(PoseStamped, '/cartesian_pose', 10)

        self.subscription = self.create_subscription(
            PointStamped,
            '/mouse/relative_position',
            self.cursor_callback,
            10
        )

        self.get_logger().info('Mock Robot Pose Publisher (subscriber) iniciado.')

    def cursor_callback(self, msg: PointStamped):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'base'

        pose.pose.position.x = msg.point.x
        pose.pose.position.y = msg.point.y
        pose.pose.position.z = 0.0

        # Orientação unitária (nenhuma rotação)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f"Publicando pose do robô mock: x={pose.pose.position.x:.1f}, y={pose.pose.position.y:.1f}"
        )

        self.publisher_.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = MockRobotPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
