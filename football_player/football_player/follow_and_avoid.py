import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist


class FollowAndAvoid(Node):
    def __init__(self):
        super().__init__('follower_and_avoider')

        self.img_center_x = 640  

        # Memoria de posiciones previas
        self.ball_pose = None
        self.person_pose = None

        # PID memory 
        self.ball_integral = 0.0
        self.ball_prev_error = 0.0

        self.dist_integral = 0.0
        self.dist_prev_error = 0.0

        self.prev_time = self.get_clock().now().nanoseconds


        # Subs y pubs
        self.bb_sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.listener_callback,
            10
        )
        self.vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel_nav',
            10,
        )

        # Parámetros
        self.declare_parameter('reference_area', 20000.0)        

        self.declare_parameter('max_angular', 2.0)
        self.declare_parameter('max_linear', 1.5)


        self.declare_parameter('linear_ball_proportional_gain',    1.0)
        self.declare_parameter('linear_ball_integral_gain',        1.0)
        self.declare_parameter('linear_ball_derivative_gain',      1.0)
        self.declare_parameter('linear_person_proportional_gain',  1.0)
        self.declare_parameter('linear_person_integral_gain',      1.0)
        self.declare_parameter('linear_person_derivative_gain',    1.0)
        self.declare_parameter('angular_ball_proportional_gain',    1.0)
        self.declare_parameter('angular_ball_integral_gain',        1.0)
        self.declare_parameter('angular_ball_derivative_gain',      1.0)
        self.declare_parameter('angular_person_proportional_gain',  1.0)
        self.declare_parameter('angular_person_integral_gain',      1.0)
        self.declare_parameter('angular_person_derivative_gain',    1.0)

        self.max_angular = self.get_parameter('max_angular').value
        self.max_linear = self.get_parameter('max_linear').value
        self.area_ref    = self.get_parameter('reference_area').value

        self.angular_ball_K_p = self.get_parameter('angular_ball_proportional_gain').value
        self.angular_ball_K_i = self.get_parameter('angular_ball_integral_gain').value
        self.angular_ball_K_d = self.get_parameter('angular_ball_derivative_gain').value
        self.linear_ball_K_p = self.get_parameter('linear_ball_proportional_gain').value
        self.linear_ball_K_i = self.get_parameter('linear_ball_integral_gain').value
        self.linear_ball_K_d = self.get_parameter('linear_ball_derivative_gain').value

        self.angular_person_K_p = self.get_parameter('angular_person_proportional_gain').value
        self.angular_person_K_i = self.get_parameter('angular_person_integral_gain').value
        self.angular_person_K_d = self.get_parameter('angular_person_derivative_gain').value
        self.linear_person_K_p = self.get_parameter('linear_person_proportional_gain').value
        self.linear_person_K_i = self.get_parameter('linear_person_integral_gain').value
        self.linear_person_K_d = self.get_parameter('linear_person_derivative_gain').value


    # ------------------------
    #        CALLBACK
    # ------------------------
    def listener_callback(self, msg: Detection2DArray):
        vel = Twist()

        # Guardar previas
        prev_ball_pose = self.ball_pose
        prev_person_pose = self.person_pose

        # Reset actuales
        ball_pose = None
        person_pose = None

        # Procesar detecciones
        for detection in msg.detections:
            for result in detection.results:
                label = result.hypothesis.class_id
                conf = result.hypothesis.score

                cx = detection.bbox.center.position.x
                area = detection.bbox.size_x * detection.bbox.size_y

                if label == 'ball':
                    ball_pose = [cx, area, conf]
                elif label == 'person':
                    pass
                    # person_pose = [cx, area, conf]

        # Lógica de comportamiento:
        # 1) Si hay pelota -> seguir
        # 2) Si no hay pelota pero sí persona -> evitar
        # 3) Si no hay nada -> stop

        if ball_pose is not None:
            lin, ang = self.ball_commands(ball_pose, prev_ball_pose)
        elif person_pose is not None:
            pass
            # lin, ang = self.human_commands(person_pose, prev_person_pose)
        else:
            lin, ang = (0.0, 0.0)

        vel.linear.x = float(lin)
        vel.angular.z = float(ang)
        self.vel_pub.publish(vel)

        self.ball_pose = ball_pose
        self.person_pose = person_pose


    def vel_limiter(self, linear, angular):
        mod_linear = linear
        mod_angular = angular

        if abs(mod_linear) > self.max_linear:
            
            if mod_angular > 0:
                mod_linear = self.max_linear
            else:
                mod_linear = -self.max_linear

        if abs(mod_angular) > self.max_angular:
            if mod_angular > 0:
                mod_angular = self.max_angular
            else:
                mod_angular = -self.max_angular

        return mod_linear, mod_angular


    def human_commands(self, pose, prev_pose):
        if prev_pose is None:
            prev_pose = pose  # evitar None en primer frame

        error = self.img_center_x//2 - pose[0]
        prev_error = self.img_center_x - prev_pose[0]
        delta_error = error - prev_error
        error_sum = error + prev_error

        angular_vel = -(
            self.angular_ball_K_p * error +
            self.angular_person_K_d * delta_error +
            self.angular_person_K_i * error_sum
        )

        linear_vel = 0.0  # retroceso opcional

        return self.vel_limiter(linear_vel, angular_vel)


    def ball_commands(self, pose, prev_pose):
        if prev_pose is None:
            prev_pose = pose 

        # Modulo tempotal para dt
        now = self.get_clock().now().nanoseconds
        dt = (now - self.prev_time) * 1e-9
        dt = max(dt, 1e-6)
        self.prev_time = now

        #  CONTROL ANGULAR
        error = self.img_center_x//2 - pose[0]
        
         # Integral with anti-windup
        self.ball_integral += error * dt
        self.ball_integral = max(min(self.ball_integral, 1000), -1000)

        # DERIVATIVO
        deriv = (error - self.ball_prev_error) / dt
        self.ball_prev_error = error

        # Girar hacia la pelota
        angular_vel = (
            self.angular_ball_K_p * error +
            self.angular_ball_K_i * self.ball_integral +
            self.angular_ball_K_d * deriv
        )

        # ---------------------------
        #     LINEAR PID (distance)
        # ---------------------------
        area = pose[1]
        dist_error = self.area_ref - area

        self.dist_integral += dist_error * dt
        self.dist_integral = max(min(self.dist_integral, 50000), -50000)

        dist_deriv = (dist_error - self.dist_prev_error) / dt
        self.dist_prev_error = dist_error

        linear_vel = (
            self.linear_ball_K_p * dist_error +
            self.linear_ball_K_i * self.dist_integral +
            self.linear_ball_K_d * dist_deriv
        )

        # Do not go backwards on noise
        confidence = pose[2]
        if confidence <= 0.65:
            linear_vel = 0.0


        return self.vel_limiter(linear_vel, angular_vel)


def main(args=None):
    rclpy.init(args=args)
    node = FollowAndAvoid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
