import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class TfBroadcasterNode(Node):
    def __init__(self):
        super().__init__('tf_broadcaster_node')
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(PoseStamped, '/zx200/base_link/pose', self.pose_callback, 10)

    def pose_callback(self, msg):
        transform = TransformStamped()
        transform.header.stamp =  self.get_clock().now().to_msg() 
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = msg.pose.position.x
        transform.transform.translation.y = msg.pose.position.y
        transform.transform.translation.z = msg.pose.position.z
        transform.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = TfBroadcasterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
