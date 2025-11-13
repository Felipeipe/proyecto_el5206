import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist

class FollowAndAvoid(Node):
    def __init__(self):
        super().__init__('follower_and_avoider')
        self.bb_sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.listener_callback,
            10)
        self.vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel', # TODO: modify
            10,
        )

    def listener_callback(self, msg: Detection2DArray):
        vel = Twist()
        for detection in msg.detections:
            for results in detection.results:
                label = self.id_parser(results.id)
                
                
                
    def id_parser(self, val):
        if val == 1:
            return 'ball'
        else:
            return 'person'
def main(args=None):
    rclpy.init(args=args)
    node = FollowAndAvoid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()