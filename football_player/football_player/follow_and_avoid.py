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

        self.declare_parameter('ball_proportional_gain',    1.0)
        self.declare_parameter('ball_integral_gain',        1.0)
        self.declare_parameter('ball_derivative_gain',      1.0)
        self.declare_parameter('person_proportional_gain',  1.0)
        self.declare_parameter('person_integral_gain',      1.0)
        self.declare_parameter('person_derivative_gain',    1.0)
        self.declare_parameter('thresh_before_turning',   10000)

        ball_K_p = self.get_parameter('ball_proportional_gain').value
        ball_K_i = self.get_parameter('ball_integral_gain').value
        ball_K_d = self.get_parameter('ball_derivative_gain').value
        person_K_p = self.get_parameter('person_proportional_gain').value
        person_K_i = self.get_parameter('person_integral_gain').value
        person_K_d = self.get_parameter('person_derivative_gain').value
        turning_thresh = self.get_parameter('thresh_before_turning').value

    def listener_callback(self, msg: Detection2DArray):
        vel = Twist()
        for detection in msg.detections:
            for results in detection.results:
                label = self.id_parser(results.id)
                if label == 'ball':
                    pass
                elif label == 'person':
                    pass 
                else:
                    pass
    def human_commands(self, pose):
        linear_vel, angular_vel = (0,0)
        return linear_vel, angular_vel

    def ball_commands(self, pose):
        linear_vel, angular_vel = (0,0)
        return linear_vel, angular_vel
        
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