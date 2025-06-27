import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

# a função desse nó é ler os frames da camera (/camera/image_raw) e as predictions do YOLO (/yolo/detections), extrair as infos de prediction, 
# confiança e coordenadas da bounding box, plotar as boxes sobre os frames, e mostrar frame a frame em uma taxa de update, possibilitando live feedback

class YOLOVisualizerNode(Node):
    def __init__(self):
        super().__init__('yolo_visualizer_node')
        
        # inicializa o CvBridge, a var image e a lista detections
        self.bridge = CvBridge()
        self.image = None
        self.detections = []

        # sub no topic da câmera
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # sub no topic do YOLO
        self.create_subscription(
            String,
            '/yolo/detections',
            self.detections_callback,
            10
        )

        self.create_timer(0.1, self.visualization_callback)

        self.get_logger().info('YOLO Visualizer Node iniciado.')

    # converte as msg do /camera/image_raw para imagens e armazena no self.image
    def image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Erro ao converter imagem: {e}')


    # extrai, da msg, o rótulo de detecção e confiança (label) e coordenadas da bounding box
        # espera que a mensagem de deteção seja uma string tipo:
        # "person (0.89) [100,200,300,400]; cat (0.75) [400,500,600,700]; ..."
    def detections_callback(self, msg):
        if msg.data.strip() != "":
            for detection in msg.data.split(';'):
                detection = detection.strip()
                if detection:
                    try:
                        label, box_part = detection.rsplit('[', 1)
                        x1, y1, x2, y2 = map(int, box_part.strip('[]').split(','))
                        self.detections.append((label.strip(), (x1, y1, x2, y2)))
                    except Exception as e:
                        self.get_logger().warn(f'Erro ao parsear detecção: "{detection}" - {e}')

    # mostra os frames com as bounding boxes desenhadas
    def visualization_callback(self):
        if self.image is None:
            return

        frame = self.image.copy()

        # desenha as bounding boxes
        for label, (x1, y1, x2, y2) in self.detections:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # mostra a imagem
        cv2.imshow('YOLO Visualizer', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YOLOVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
