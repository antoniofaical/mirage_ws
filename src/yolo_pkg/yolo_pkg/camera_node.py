import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# a função desse nó é conectar com a câmera do dispositivo, 
# ler e publicar os frames do vídeo num tópico,
# e desconectar da câmera ao fim do uso.

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) 

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error('OpenCV2 VideoCapture não conseguiu abrir a câmera.')
        else:
            self.get_logger().info('OpenCV2 VideoCapture inicializou a câmera com sucesso.')

    def timer_callback(self):
    # "ret" = retorna True ou False se o frame foi capturado corretamente, e "frame" é o frame em si
        ret, frame = self.cap.read()
        if ret:
        # aq o cvbridge converte a imagem do frame com encoding bgr8 para uma msg do tipo Image em ROS2
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        else:
            self.get_logger().warning('OpenCV2 VideoCapture não conseguiu ler o frame.')

    def destroy_node(self):
    # desligando a câmera
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
