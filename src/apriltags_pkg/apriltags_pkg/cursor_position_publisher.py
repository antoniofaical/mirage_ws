import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import pyautogui
import subprocess

class CursorPositionPublisher(Node):
    def __init__(self):
        super().__init__('cursor_position_publisher')

        self.publisher_ = self.create_publisher(PointStamped, '/mouse/relative_position', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

        self.window_rect = self.get_interface_window_geometry()
        self.get_logger().info('Cursor Position Publisher iniciado.')

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
        x0, y0, x1, y1 = self.window_rect

        # Verifica se o mouse está dentro da janela
        if not (x0 <= x <= x1 and y0 <= y <= y1):
            self.get_logger().info("Mouse fora da janela, ignorando publicação.")
            return

        # Offset equivalente à barra superior da interface
        offset_y = 80 #px

        # Posição relativa à janela
        px = x - x0
        py = y - y0 - offset_y

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'interface_window'
        msg.point.x = float(px)
        msg.point.y = float(py)
        msg.point.z = 0.0

        self.publisher_.publish(msg)

        self.get_logger().info(f"Mouse relativo: x={px}, y={py}")

def main(args=None):
    rclpy.init(args=args)
    node = CursorPositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
