#!/usr/bin/env python3
"""Board-safe open-loop driving: timed /cmd_vel bursts, no encoder reads.

Why this exists: `DriveRelative` is closed-loop, so it interleaves motor
writes with encoder reads on the same I2C transaction stream — and that is
what hangs the 0x34 driver off the bus (observed three times; each needs a
physical power cycle). The motor node runs `use_encoder_feedback:=False`, so
driving from /cmd_vel is pure writes and does not trigger the contention.

Accuracy is open-loop, so verify the result by eye or with the camera rather
than trusting the commanded distance. Forward motion under-delivers ~26%
(the closed-loop controller compensates with a 0.74 factor), which is applied
here as EFFICIENCY.

  python3 scripts/drive_open_loop.py --dx 0.10          # forward 10 cm
  python3 scripts/drive_open_loop.py --dyaw 0.30        # rotate +0.30 rad
"""
import argparse
import time

import rclpy
from geometry_msgs.msg import Twist

EFFICIENCY = 0.74            # measured forward/strafe delivery
RATE_HZ = 20.0
MAX_DIST = 0.6               # refuse absurd single commands
MAX_YAW = 1.6


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dx', type=float, default=0.0, help='forward metres')
    ap.add_argument('--dy', type=float, default=0.0, help='left metres')
    ap.add_argument('--dyaw', type=float, default=0.0, help='CCW radians')
    ap.add_argument('--speed', type=float, default=0.14, help='m/s')
    ap.add_argument('--yaw-speed', type=float, default=0.5, help='rad/s')
    args = ap.parse_args()

    if abs(args.dx) > MAX_DIST or abs(args.dy) > MAX_DIST or abs(args.dyaw) > MAX_YAW:
        raise SystemExit('command exceeds single-move bounds')

    rclpy.init()
    node = rclpy.create_node('drive_open_loop')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    time.sleep(0.7)                       # let the subscription match

    def burst(vx, vy, wz, seconds):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        end = time.time() + seconds
        while time.time() < end:
            pub.publish(msg)
            time.sleep(1.0 / RATE_HZ)
        stop = Twist()
        for _ in range(8):                # ramp-down + hold zero
            pub.publish(stop)
            time.sleep(1.0 / RATE_HZ)

    if args.dyaw:
        t = abs(args.dyaw) / args.yaw_speed
        print(f'yaw {args.dyaw:+.2f} rad -> {t:.2f}s at {args.yaw_speed} rad/s')
        burst(0.0, 0.0, args.yaw_speed * (1 if args.dyaw > 0 else -1), t)
        time.sleep(0.5)

    dist = (args.dx ** 2 + args.dy ** 2) ** 0.5
    if dist > 0:
        t = dist / (args.speed * EFFICIENCY)
        ux, uy = args.dx / dist, args.dy / dist
        print(f'translate {dist:.3f} m -> {t:.2f}s at {args.speed} m/s '
              f'(efficiency {EFFICIENCY})')
        burst(ux * args.speed, uy * args.speed, 0.0, t)

    print('done (open-loop; verify visually)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
