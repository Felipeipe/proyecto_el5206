import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist, Vector3


class FollowAndAvoid(Node):
    def __init__(self):
        super().__init__('follower_and_avoider')

        self.img_center_x = 640  

        # Memoria de posiciones previas
        self.ball_pose = None
        self.person_pose = None
        self.ball_prev_area = None
        self.person_prev_area = None
        # PID memory 
        self.ball_integral = 0.0
        self.ball_prev_error = 0.0
        self.ball_dist_integral = 0.0
        self.ball_dist_prev_error = 0.0

        self.person_integral = 0.0
        self.person_prev_error = 0.0
        self.person_dist_integral = 0.0
        self.person_dist_prev_error = 0.0

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

        self.person_error_pub = self.create_publisher(
            Vector3,
            '/person/control_errors',
            10
        )
        self.ball_error_pub = self.create_publisher(
            Vector3,
            '/ball/control_errors',
            10
        )

        # Parámetros
        self.declare_parameter('ball_reference_area', 20000.0)        
        self.declare_parameter('person_reference_area', 200000.0)        

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
        self.ball_area_ref    = self.get_parameter('ball_reference_area').value
        self.person_area_ref  = self.get_parameter('person_reference_area').value

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
                    person_pose = [cx, area, conf]

        # Lógica de comportamiento
        if ball_pose is not None:
            lin, ang = self.ball_commands(ball_pose, prev_ball_pose)
        elif person_pose is not None:
            lin, ang = self.human_commands(person_pose, prev_person_pose)
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
            mod_linear = self.max_linear if mod_linear > 0 else -self.max_linear

        if abs(mod_angular) > self.max_angular:
            mod_angular = self.max_angular if mod_angular > 0 else -self.max_angular

        return mod_linear, mod_angular


    # ------------------------
    #  HUMAN COMMANDS
    # ------------------------
    def human_commands(self, pose, prev_pose):
        if prev_pose is None:
            prev_pose = pose  

        person_error = self.img_center_x//2 - pose[0]
        person_prev_error = self.img_center_x//2 - prev_pose[0]
        person_delta_error = person_error - person_prev_error

        self.person_integral += person_error
        self.person_integral = max(min(self.person_integral, 1000), -1000)

        angular_vel = (
            self.angular_person_K_p * person_error +
            self.angular_person_K_d * person_delta_error +
            self.angular_person_K_i * self.person_integral
        )

        area = pose[1]
        if self.ball_prev_area is None:
            area_prom = area
        else:
            area_prom = (area + self.ball_prev_area) / 2

        dist_error = self.person_area_ref - area_prom

        self.person_dist_integral += dist_error
        self.person_dist_integral = max(min(self.person_dist_integral, 20000), -20000)

        dist_deriv = (dist_error - self.person_dist_prev_error)

        linear_vel_prev = (
            self.linear_person_K_p * dist_error +
            self.linear_person_K_i * self.person_dist_integral +
            self.linear_person_K_d * dist_deriv
        )
        linear_vel = 0.0 if dist_error > 0 else linear_vel_prev

        confidence = pose[2]
        if confidence <= 0.6:
            linear_vel = 0.0
            angular_vel = 0.0

        # 👉 NUEVO: publicar errores de persona
        err_msg = Vector3()
        err_msg.x = float(dist_error)        # error lineal
        err_msg.y = float(person_error)      # error angular
        err_msg.z = float(confidence)        # confianza (opcional)
        self.person_error_pub.publish(err_msg)

        self.person_dist_prev_error = dist_error
        self.person_prev_area = area

        return self.vel_limiter(linear_vel, angular_vel)


    # ------------------------
    #  BALL COMMANDS
    # ------------------------
    def ball_commands(self, pose, prev_pose):
        if prev_pose is None:
            prev_pose = pose 

        error = self.img_center_x//2 - pose[0]

        self.ball_integral += error
        self.ball_integral = max(min(self.ball_integral, 1000), -1000)

        deriv = (error - self.ball_prev_error)
        self.ball_prev_error = error

        angular_vel = (
            self.angular_ball_K_p * error +
            self.angular_ball_K_i * self.ball_integral +
            self.angular_ball_K_d * deriv
        )

        area = pose[1]

        if self.ball_prev_area is None:
            area_prom = area
        else:
            area_prom = (area + self.ball_prev_area) / 2

        dist_error = self.ball_area_ref - area_prom

        self.ball_dist_integral += dist_error
        self.ball_dist_integral = max(min(self.ball_dist_integral, 20000), -20000)

        dist_deriv = (dist_error - self.ball_dist_prev_error)

        linear_vel = (
            self.linear_ball_K_p * dist_error +
            self.linear_ball_K_i * self.ball_dist_integral +
            self.linear_ball_K_d * dist_deriv
        )

        confidence = pose[2]
        if confidence <= 0.6:
            linear_vel = 0.0
            angular_vel = 0.0

        # 👉 NUEVO: publicar errores de pelota
        err_msg = Vector3()
        err_msg.x = float(dist_error)        # error lineal
        err_msg.y = float(error)             # error angular
        err_msg.z = float(confidence)        # confianza
        self.ball_error_pub.publish(err_msg)

        self.ball_dist_prev_error = dist_error
        self.ball_prev_area = area

        return self.vel_limiter(linear_vel, angular_vel)


def main(args=None):
    rclpy.init(args=args)
    node = FollowAndAvoid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
