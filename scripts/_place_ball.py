#!/usr/bin/env python3
"""One-shot: carry the held ball inward, touchdown-release at (278, 0)."""
import sys
import rclpy
sys.path.insert(0, '/scripts')
from sock_cycle import Cycle, GRIP_OPEN

rclpy.init()
c = Cycle()
assert c.wait_ready()
c.grip_cmd = 2.703                      # match the teleop hold, do not disturb
c.move(355, 37, -40, speed=45.0)
ok, th, ta, gap = c.held()
print(f'carry : torH={th} claw={ta:.3f}')
c.move(278, 0, -40, speed=55.0)
z, contact = -40.0, None
while z > -196.0:
    z -= 4.0
    c.send(278, 0, z, c.grip_cmd)
    c.spin(0.4)
    p = c.pose()
    if p[2] - z > 7.0:
        contact = p[2]
        break
print(f'touchdown at wrist z {contact}')
zset = (contact + 2.0) if contact is not None else -190.0
c.move(278, 8, zset + 3.0, t=GRIP_OPEN, speed=25.0, grip_ramp=True)
c.spin(0.5)
c.move(278, 8, -60, speed=45.0)
print('released; parked above')
c.destroy_node()
rclpy.shutdown()
