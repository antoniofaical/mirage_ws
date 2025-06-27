# mock_robot_pose_publisher.py (ajustes: frame_id='base' + orientacao valida)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import pyautogui
import subprocess
import time

class MockRobotPosePublisher(Node):
    def __init__(self):
        super().__init__('mock_robot_pose_publisher')

        self.publisher_ = self.create_publisher(PoseStamped, '/cartesian_pose', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz
        self.window_rect = self.get_interface_window_geometry()
        self.get_logger().info('Mock Robot Pose Publisher started.')

    def get_interface_window_geometry(self):
        try:
            result = subprocess.check_output([
                'xdotool', 'search', '--name', 'Interface'
            ]).decode().strip()

            if not result:
                raise RuntimeError("Janela 'Interface' não encontrada")

            win_id = result.split('\n')[0]
            geometry = subprocess.check_output([
                'xdotool', 'getwindowgeometry', '--shell', win_id
            ]).decode()

            geom_dict = {}
            for line in geometry.split('\n'):
                if '=' in line:
                    key, value = line.strip().split('=')
                    geom_dict[key] = int(value)

            x = geom_dict['X']
            y = geom_dict['Y']
            width = geom_dict['WIDTH']
            height = geom_dict['HEIGHT']

            return (x, y, x + width, y + height)
        except Exception as e:
            self.get_logger().error(f"Erro ao obter geometria da janela: {e}")
            screen_width, screen_height = pyautogui.size()
            return (0, 0, screen_width, screen_height)  # fallback dinâmico

    def timer_callback(self):
        x, y = pyautogui.position()
        x0, y0, _, _ = self.window_rect

        px = x - x0
        py = y - y0

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'base'
        pose.pose.position.x = float(px)
        pose.pose.position.y = float(py)
        pose.pose.position.z = 0.0

        # Orientacao unitária (sem rotação)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f"Publicando pose: x={px:.1f}px, y={py:.1f}px, z=0.0"
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