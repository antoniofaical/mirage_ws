import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO

# a função desse nó é ler as imagens do /camera/image_raw (vindo do camera_node), processá-las pelo yolov8n
# e publicar os resultados da predição em formato de string no topic /yolo/detections

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
    # subscribes no tópico do camera_node para pegar os frames do vídeo e rodar no yolov8n
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
    # cria um publisher para publicar as predictions
        self.publisher_ = self.create_publisher(
            String,
            '/yolo/detections',
            10)
    # inicializa o cvbridge e o YOLO
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLO Node iniciado! Escutando /camera/image_raw...')

    def image_callback(self, msg):
        try:
            # converte a msg Image do /camera/image_raw de volta para uma imagem usando a mesma encoding bgr8
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # joga a imagem convertida no YOLO para fazer as predições
            results = self.model.predict(frame, conf=0.5, verbose=False)

            # inicializa uma var de msg para armazenar as predictions
            msg_out = ''
            
            # armazena na var acima os resultados em formato de string, no formato "prediction_1 (confiança) [x1,y1,x2,y2]; ... ; prediction_n (confiança) [x1,y1,x2,y2];", 
            # publicando essa message como string no topic /yolo/detections
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    cls_id = int(box.cls[0])
                    cls_name = self.model.names[cls_id]
                    conf = float(box.conf[0])

                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

                    msg_out += f'{cls_name} ({conf:.2f}) [{x1},{y1},{x2},{y2}]; '

            msg_out = msg_out.strip()
            self.publisher_.publish(String(data=msg_out))


        except Exception as e:
            self.get_logger().error(f'Erro ao processar frame: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
