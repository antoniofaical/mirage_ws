import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(
            String,
            '/yolo/detections',
            10)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLO Node iniciado! Escutando /camera/image_raw...')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model.predict(frame, conf=0.5, verbose=False)

            detection_list = []
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    cls_id = int(box.cls[0])
                    cls_name = self.model.names[cls_id]
                    conf = float(box.conf[0])
                    detection_list.append(f"{cls_name} ({conf:.2f})")

            msg_out = ''
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
