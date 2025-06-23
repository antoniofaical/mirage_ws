from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster,TransformListener

from tf2_ros import TransformException 
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 


class DynamicFrameBroadcaster(Node):

    def __init__(self):
        super().__init__('dynamic_frame_tf2_broadcaster')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_broadcaster_transform = TransformBroadcaster(self)


        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.broadcast_timer_callback)

    def broadcast_timer_callback(self):

        #Transformada
        from_frame_rel = 'apriltag_TAG16H5' #TAG16H5 #PONTO A SER TRANSFORMADO
        to_frame_rel = 'apriltag_TAG36H11' #TAG36H11 #ORIGEM (REFERÊNCIA)
    
        trans = None

        timeout = rclpy.duration.Duration(seconds=0.1)

        available_frames = self.tf_buffer.all_frames_as_string()
        self.get_logger().info(f'Frames disponíveis após inicialização: {available_frames}')
        
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        now,timeout=timeout)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        print(trans.transform.translation.x)
        print(trans.transform.translation.y)
        print(trans.transform.translation.z)

        #Plotando Frame no rviz2
        t_point = TransformStamped()
        t_point.header.stamp = self.get_clock().now().to_msg()
        t_point.header.frame_id = 'apriltag_TAG36H11'
        t_point.child_frame_id = 'TRANSFORMADO'
        t_point.transform.translation.x = float(trans.transform.translation.x) 
        t_point.transform.translation.y = float(trans.transform.translation.y)
        t_point.transform.translation.z = float(trans.transform.translation.z)
        t_point.transform.rotation.x = 0.0
        t_point.transform.rotation.y = 0.0
        t_point.transform.rotation.z = 0.0
        t_point.transform.rotation.w = 1.0

        self.tf_broadcaster_transform.sendTransform(t_point)




def main():
    rclpy.init()
    node = DynamicFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()