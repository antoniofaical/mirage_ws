import rclpy
import rclpy.duration
from rclpy.node import Node
import rclpy.parameter

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PoseStamped
from rclpy.exceptions import ParameterNotDeclaredException


FREQ_TF = 80 # Hz

class TransformNode(Node):
    def __init__(self):
        super().__init__('transform_frame')
        #Variáveis para transformada e input
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        timer_period = 1/FREQ_TF
        self.timer = self.create_timer(timer_period, self.transform_point_target)

        self.trans_control=0
        self.target_pose_msg = PoseStamped()

        # Publisher to /target_pose topic
        self.target_pose_publisher = self.create_publisher(
            PoseStamped,
            '/target_pose_in_apriltag_frame',
            1
        )

        try:
            # Get parameters
            self.frame_reference = self.get_parameter('frame_reference').get_parameter_value().string_value
            self.frame_target = self.get_parameter('frame_target').get_parameter_value().string_value
            
            self.get_logger().info(f'AprilTag Reference: {self.frame_reference}')
            self.get_logger().info(f'AprilTag Target: {self.frame_target}')
        except ParameterNotDeclaredException as e:
            self.get_logger().error(f'Error: {e}')
            self.frame_reference = 'apriltag_TAG36H11'
            self.frame_target = 'apriltag_TAG16H5'


    def transform_point_target(self):
        #Transformada
        from_frame_rel = self.frame_target # 'apriltag_TAG16H5'
        to_frame_rel =  self.frame_reference # 'apriltag_TAG36H11' #world

        timeout = rclpy.duration.Duration(seconds=0.1)

        # available_frames = self.tf_buffer.all_frames_as_string()
        # self.get_logger().info(f'Frames disponíveis após inicialização: {available_frames}')
        
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

        self.target_pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.target_pose_msg.header.frame_id = from_frame_rel
        self.target_pose_msg.pose.position.x = trans.transform.translation.x# + pos_robot_relative_to_apriltag.pose.position.x
        self.target_pose_msg.pose.position.y = trans.transform.translation.y# + pos_robot_relative_to_apriltag.pose.position.y
        self.target_pose_msg.pose.position.z = trans.transform.translation.z# + pos_robot_relative_to_apriltag.pose.position.z
        
        # print(trans.transform.translation.x*(-1))
        # print("X - FROM CAMERA: ", trans.transform.translation.x)
        # print("Y - FROM CAMERA: ", trans.transform.translation.y)
        # print("X - FROM ROBOT:", pos_robot_relative_to_apriltag.pose.position.x)
        # print("Y - FROM ROBOT:", pos_robot_relative_to_apriltag.pose.position.y)
        # self.x_trans = trans.transform.translation.x + pos_robot_relative_to_apriltag.pose.position.x
        # self.y_trans = trans.transform.translation.y + pos_robot_relative_to_apriltag.pose.position.y
        # self.z_trans = trans.transform.translation.z*0 + pos_robot_relative_to_apriltag.pose.position.z

        self.trans_control =+ 1
        if self.trans_control%FREQ_TF == 0 and self.trans_control != 0:
            print('\n\n FINAL POSITION:', self.target_pose_msg.pose.position.x, 
                                          self.target_pose_msg.pose.position.y,
                                          self.target_pose_msg.pose.position.z, '\n\n')
            self.trans_control == 0
        
        self.target_pose_publisher.publish(self.target_pose_msg)


def main(args=None):
    rclpy.init(args=args)
    transform_frame = TransformNode()
    #Node da transformada
    rclpy.spin(transform_frame)

if __name__ == '__main__':
    main()