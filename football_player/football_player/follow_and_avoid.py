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

        ball_pose = None
        person_pose = None

        for detection in msg.detections:
            for results in detection.results:
                label = self.id_parser(results.id)
                cx = detection.bbox.center.position.x
                area = detection.bbox.size_x * detection.bbox.size_y

                if label == 'ball':
                    ball_pose = [cx, area]
                elif label == 'person':
                    person_pose = [cx, area]

        #  FOLLOW BALL > AVOID PERSON
        if ball_pose is not None:
            lin, ang = self.ball_commands(ball_pose)
        elif person_pose is not None:
            lin, ang = self.human_commands(person_pose)
        else:
            lin, ang = (0.0, 0.0)

        vel.linear.x = float(lin)
        vel.angular.z = float(ang)
        self.vel_pub.publish(vel)


    def human_commands(self, pose):
        error = self.img_center_x - pose

        # Control Proporcional (girar lejos del humano)
        angular_vel = -(self.person_K_p * error) * 0.002  

        # Retrocede
        linear_vel = -0.15/max(pose[1], 1.0)  # Más cerca, más rápido retrocede

        return linear_vel, angular_vel
    
    def ball_commands(self, pose):
        error = self.img_center_x - pose

        # Control Proporcional (girar hacia pelota)
        angular_vel = (self.ball_K_p * error) * 0.002  

        # Avanzar
        linear_vel = 0.20/max(pose[1], 1.0)  # Más cerca, más rápido avanza

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