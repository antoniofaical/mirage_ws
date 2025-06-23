import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import roboticstoolbox as rtb
import spatialmath as sm
from numpy import pi

class CartesianPosePublisher(Node):
    def __init__(self):
        super().__init__('cartesian_pose_publisher')
        
        # Subscriber to /joint_states topic
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            1
        )
        
        # Publisher to /cartesian_position topic
        self.cartesian_position_publisher = self.create_publisher(
            PoseStamped,
            '/cartesian_pose',
            1
        )
        
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]

        self.UR20= rtb.DHRobot([
            rtb.RevoluteDH(d=0.2363,alpha=pi/2),
            rtb.RevoluteDH(a=-0.8620),
            rtb.RevoluteDH(a=-0.7287),
            rtb.RevoluteDH(d=0.2010,alpha=pi/2),
            rtb.RevoluteDH(d=0.1593,alpha=-pi/2),
            rtb.RevoluteDH(d=0.1543)
        ],name="ur20")

    def joint_state_callback(self, msg):
        joint_positions = {}
        for name, position in zip(msg.name, msg.position):
            if name in self.joint_names:
                joint_positions[name] = position

        # Process the joint positions to compute Cartesian position
        cartesian_position = self.compute_cartesian_position([joint_positions[name] for name in self.joint_names])
        
        # Publish the Cartesian position
        self.cartesian_position_publisher.publish(cartesian_position)

    def compute_cartesian_position(self, joint_positions):
        fk = self.UR20.fkine(joint_positions)
       
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'base'  # Replace with appropriate frame
        
        pose_stamped.pose.position.x = fk.t[0]
        pose_stamped.pose.position.y = fk.t[1]
        pose_stamped.pose.position.z = fk.t[2]

        quat = sm.UnitQuaternion(fk.R).vec

        # Assuming fk_result contains orientation as a quaternion
        pose_stamped.pose.orientation.x = quat[1]
        pose_stamped.pose.orientation.y = quat[2]
        pose_stamped.pose.orientation.z = quat[3]
        pose_stamped.pose.orientation.w = quat[0]

        return pose_stamped

def main(args=None):
    rclpy.init(args=args)
    node = CartesianPosePublisher()
    rclpy.spin_once(node)
    while rclpy.ok():
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()