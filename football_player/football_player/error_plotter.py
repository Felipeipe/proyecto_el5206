import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

from ament_index_python.packages import get_package_share_directory
import os

import matplotlib.pyplot as plt
import time


class ControlErrorPlotter(Node):
    def __init__(self):
        super().__init__('control_error_plotter_static_dual')

        self.ball_sub = self.create_subscription(
            Vector3,
            '/ball/control_errors',
            self.ball_callback,
            10
        )

        self.person_sub = self.create_subscription(
            Vector3,
            '/person/control_errors',
            self.person_callback,
            10
        )

        self.t0 = time.time()

        self.time_ball = []
        self.ball_err_lin = []
        self.ball_err_ang = []

        self.time_person = []
        self.person_err_lin = []
        self.person_err_ang = []

        self.last_msg_time = time.time()
        self.timeout_sec = 3.0
        self.timer = self.create_timer(0.2, self.check_timeout)

        self.get_logger().info(
            'Listening to /ball/control_errors and /person/control_errors'
        )

    def ball_callback(self, msg: Vector3):
        t = time.time() - self.t0
        self.time_ball.append(t)
        self.ball_err_lin.append(msg.x)
        self.ball_err_ang.append(msg.y)
        self.last_msg_time = time.time()

    def person_callback(self, msg: Vector3):
        t = time.time() - self.t0
        self.time_person.append(t)
        self.person_err_lin.append(msg.x)
        self.person_err_ang.append(msg.y)
        self.last_msg_time = time.time()

    def check_timeout(self):
        if time.time() - self.last_msg_time > self.timeout_sec:
            self.get_logger().info('No messages received, finishing plotter')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ControlErrorPlotter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # -------- PLOT --------
    fig, (ax_lin, ax_ang) = plt.subplots(
        2, 1, figsize=(11, 8), sharex=True
    )

    # -------- LINEAR ERROR --------
    ax_lin.plot(
        node.time_ball,
        node.ball_err_lin,
        color='#1f77b4',
        label='Ball linear error'
    )
    ax_lin.plot(
        node.time_person,
        node.person_err_lin,
        color='#d62728',
        label='Person linear error'
    )

    ax_lin.set_ylabel('Area error [px²]')
    ax_lin.set_title('Linear Control Error')
    ax_lin.grid(True)
    ax_lin.legend()

    # -------- ANGULAR ERROR --------
    ax_ang.plot(
        node.time_ball,
        node.ball_err_ang,
        color='#005f73',
        linestyle='--',
        label='Ball angular error'
    )
    ax_ang.plot(
        node.time_person,
        node.person_err_ang,
        color='#bc3908',
        linestyle='--',
        label='Person angular error'
    )

    ax_ang.set_xlabel('Time [s]')
    ax_ang.set_ylabel('Angular error [px]')
    ax_ang.set_title('Angular Control Error')
    ax_ang.grid(True)
    ax_ang.legend()

    plt.tight_layout()

    # -------- SAVE --------
    pkg_path = get_package_share_directory('football_player')
    plot_dir = os.path.join(pkg_path, 'plots')
    os.makedirs(plot_dir, exist_ok=True)

    plot_path = os.path.join(
        plot_dir,
        'control_errors_ball_person.pdf'
    )

    plt.savefig(plot_path)
    plt.close()

    node.get_logger().info(f'Plot saved to: {plot_path}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
