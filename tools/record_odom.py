#!/usr/bin/env python3
import math, csv, os, argparse
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

def wrap_to_pi(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

def yaw_from_quat(x, y, z, w):
    s = 2.0*(w*z + x*y)
    c = 1.0 - 2.0*(y*y + z*z)
    return math.atan2(s, c)

class OdomLogger(Node):
    def __init__(self, out_csv, xg, yg, thetag, pos_tol, yaw_tol, odom_topic):
        super().__init__('odom_logger')
        self.out_csv = os.path.expanduser(out_csv)
        os.makedirs(os.path.dirname(self.out_csv), exist_ok=True)
        self.f = open(self.out_csv, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t_sec','x','y','yaw','rho','yaw_err'])

        self.goal = (xg, yg, thetag)
        self.tol = (pos_tol, yaw_tol)
        self.ok_streak = 0
        self.start = self.get_clock().now()

        self.create_subscription(Odometry, odom_topic, self.cb, 20)
        self.get_logger().info(
            f'Logging to {self.out_csv} (goal=({xg:.2f},{yg:.2f},{thetag:.2f}), '
            f'tol=({pos_tol} m, {yaw_tol} rad), topic="{odom_topic}")')

    def cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        xg, yg, thetag = self.goal
        pos_tol, yaw_tol = self.tol
        rho = math.hypot(xg - x, yg - y)
        yaw_err = wrap_to_pi(thetag - yaw)

        t = (self.get_clock().now() - self.start).nanoseconds * 1e-9
        self.w.writerow([f'{t:.6f}', f'{x:.6f}', f'{y:.6f}', f'{yaw:.6f}', f'{rho:.6f}', f'{yaw_err:.6f}'])

        # stop after 1s of being within both tolerances
        if rho < pos_tol and abs(yaw_err) < yaw_tol:
            self.ok_streak += 1
        else:
            self.ok_streak = 0

        if self.ok_streak >= 20:
            self.get_logger().info('Reached goal within tolerances for ~1s, stopping.')
            self.f.flush(); self.f.close()
            rclpy.shutdown()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', default='~/cl2_logs/run1.csv')
    ap.add_argument('--xg', type=float, default=2.0)
    ap.add_argument('--yg', type=float, default=1.0)
    ap.add_argument('--thetag', type=float, default=1.57)
    ap.add_argument('--pos_tol', type=float, default=0.05)
    ap.add_argument('--yaw_tol', type=float, default=0.10)
    ap.add_argument('--odom_topic', default='/odom')
    args = ap.parse_args()

    rclpy.init()
    node = OdomLogger(args.csv, args.xg, args.yg, args.thetag,
                      args.pos_tol, args.yaw_tol, args.odom_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
