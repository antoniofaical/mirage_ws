import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class YoloListener(Node):
    def __init__(self):
        super().__init__('yolo_listener')
        self.subscription = self.create_subscription(
            String,
            '/yolo/detections',
            self.detection_callback,
            10)
        self.get_logger().info('YOLO Listener iniciado! Escutando /yolo/detections...')

    def detection_callback(self, msg):
        self.get_logger().info(f'Detecção recebida: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
