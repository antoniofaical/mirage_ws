# mock_cursor_eyetracker_publisher.py (adaptado para escala em pixels puros)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import pyautogui
import subprocess

class MockCursorEyetrackerPublisher(Node):
    def __init__(self):
        super().__init__('mock_cursor_eyetracker_publisher')

        self.publisher_ = self.create_publisher(PointStamped, '/target_info/position', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz
        self.window_rect = self.get_interface_window_geometry()
        self.get_logger().info('Mock Cursor Eyetracker Publisher started.')

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
            return (0, 0, screen_width, screen_height)

    def timer_callback(self):
        x, y = pyautogui.position()
        x0, y0, _, _ = self.window_rect

        # Converte coordenada absoluta da tela para coordenada relativa à janela Interface
        px = x - x0
        py = y - y0

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base'
        msg.point.x = float(px)
        msg.point.y = float(py)
        msg.point.z = 0.0

        self.get_logger().info(
            f"Publicando ponto: x={px:.1f}px, y={py:.1f}px"
        )

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockCursorEyetrackerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
