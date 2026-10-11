import math, sys, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

def yaw(q):
    return math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

class P(Node):
    def __init__(self):
        super().__init__('a7_pose_probe')
        self.got = {}
        for t in ('/odom_raw', '/odom_rf2o', '/odom'):
            self.create_subscription(Odometry, t, lambda m, t=t: self.got.setdefault(t, m), 10)
        self.buf = Buffer(); self.tl = TransformListener(self.buf, self)

rclpy.init(); n = P()
end = n.get_clock().now().nanoseconds + 4e9
while n.get_clock().now().nanoseconds < end:
    rclpy.spin_once(n, timeout_sec=0.1)
for t in ('/odom_raw', '/odom_rf2o', '/odom'):
    m = n.got.get(t)
    if m is None: print(f'{t:12s} SIN DATO'); continue
    p = m.pose.pose
    print(f'{t:12s} x={p.position.x:+.3f} y={p.position.y:+.3f} yaw={yaw(p.orientation):+.1f}')
try:
    tr = n.buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
    print(f'{"slam(map)":12s} x={tr.translation.x:+.3f} y={tr.translation.y:+.3f} yaw={yaw(tr.rotation):+.1f}')
except Exception as e:
    print('slam(map)    SIN DATO', e)
