#!/usr/bin/env python3
# Relative pose error (RPE) of an odometry trajectory against the SLAM
# trajectory recorded in a bag (stage 1).
#
# Reference: map -> base_footprint, composed from the bag's /tf
# (map -> odom from slam_toolbox, odom -> base_footprint from the live EKF).
# slam_toolbox closed within 2.6 cm in both stage runs, so it is a usable
# "approximate ground truth". For every window of --window seconds, the
# motion the odometry reports is compared with the motion SLAM reports over
# the same window; errors are averaged over all windows. Unlike the closure
# error (one start/end pair), errors cannot cancel out by luck.
#
# Usage (inside MentorPi, ROS sourced):
#   python3 rpe.py <bag> --csv traj.csv      # trajectory from eval_ekf.py --out
#   python3 rpe.py <bag> --topic /odom_rf2o  # an Odometry topic in the bag
import argparse
import bisect
import math

import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def compose(a, b):
    """a (+) b for 2D poses (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def relative(a, b):
    """Pose of b expressed in the frame of a: a^-1 (+) b."""
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return (c * dx + s * dy, -s * dx + c * dy, wrap(b[2] - a[2]))


def open_bag(uri, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r


def reference(uri, transform_timeout=0.2):
    """[(t, (x, y, yaw))] of map -> base_footprint, sampled only when SLAM
    publishes a new correction.

    Between corrections, map -> base_footprint just follows whatever EKF ran
    live, which would bias the reference toward that configuration. So a
    sample is taken only when map -> odom changes, composed with the live
    odom -> base_footprint at that moment. slam_toolbox future-dates map ->
    odom by transform_timeout (0.2 s in config/slam_toolbox.yaml), which is
    subtracted to get the time the correction belongs to.
    """
    r = open_bag(uri, ['/tf'])
    corrections = []  # (t, map->odom) when it changes
    odom_base = []     # (t, odom->base_footprint)
    last = None
    while r.has_next():
        _, data, _ = r.read_next()
        for tf in deserialize_message(data, TFMessage).transforms:
            tr = tf.transform
            pose = (tr.translation.x, tr.translation.y, yaw(tr.rotation))
            st = tf.header.stamp
            t = st.sec + st.nanosec * 1e-9
            if tf.header.frame_id == 'map' and tf.child_frame_id == 'odom':
                if last is None or max(abs(pose[0] - last[0]), abs(pose[1] - last[1]),
                                       abs(wrap(pose[2] - last[2]))) > 1e-6:
                    corrections.append((t - transform_timeout, pose))
                    last = pose
            elif tf.header.frame_id == 'odom' and tf.child_frame_id == 'base_footprint':
                odom_base.append((t, pose))
    ob_t = [t for t, _ in odom_base]
    out = []
    for t, map_odom in corrections:
        ob = at(odom_base, ob_t, t, tol=0.05)
        if ob is not None:
            out.append((t, compose(map_odom, ob)))
    return out


def from_topic(uri, topic):
    r = open_bag(uri, [topic])
    out = []
    while r.has_next():
        _, data, _ = r.read_next()
        m = deserialize_message(data, Odometry)
        p = m.pose.pose
        st = m.header.stamp
        out.append((st.sec + st.nanosec * 1e-9, (p.position.x, p.position.y, yaw(p.orientation))))
    return out


def from_csv(path):
    out = []
    for line in open(path):
        t, x, y, th = map(float, line.split(','))
        out.append((t, (x, y, th)))
    return out


def at(traj, times, t, tol=0.1):
    """Pose of traj nearest to t (None if farther than tol seconds)."""
    i = bisect.bisect_left(times, t)
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(traj) and (best is None or abs(times[j] - t) < abs(times[best] - t)):
            best = j
    if best is None or abs(times[best] - t) > tol:
        return None
    return traj[best][1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--csv')
    src.add_argument('--topic')
    ap.add_argument('--window', type=float, default=10.0, help='seconds')
    ap.add_argument('--label', default='')
    ap.add_argument('--verbose', action='store_true')
    a = ap.parse_args()

    ref = reference(a.bag)
    cand = from_csv(a.csv) if a.csv else from_topic(a.bag, a.topic)
    ref_t = [t for t, _ in ref]
    cand_t = [t for t, _ in cand]

    print(f'# referencia: {len(ref)} correcciones del SLAM') if a.verbose else None
    t0 = max(ref_t[0], cand_t[0])
    t_end = min(ref_t[-1], cand_t[-1])
    trans_err, yaw_err, ref_dist = [], [], 0.0
    t = t0
    while t + a.window <= t_end:
        r0, r1 = at(ref, ref_t, t, tol=0.3), at(ref, ref_t, t + a.window, tol=0.3)
        c0, c1 = at(cand, cand_t, t), at(cand, cand_t, t + a.window)
        if None not in (r0, r1, c0, c1):
            dr, dc = relative(r0, r1), relative(c0, c1)
            trans_err.append(math.hypot(dc[0] - dr[0], dc[1] - dr[1]))
            yaw_err.append(abs(wrap(dc[2] - dr[2])))
            ref_dist += math.hypot(dr[0], dr[1])
        t += 0.5  # overlapping windows, every 0.5 s

    n = len(trans_err)
    if n == 0:
        print(f'{a.label:28s} sin ventanas comparables')
        return
    rmse = math.sqrt(sum(e * e for e in trans_err) / n)
    per_m = sum(trans_err) / ref_dist * 100 if ref_dist else float('nan')
    print(f'{a.label:28s} RPE({a.window:.0f}s): tras {sum(trans_err) / n * 100:5.1f} cm '
          f'(RMSE {rmse * 100:5.1f}), {per_m:5.1f} cm por m recorrido, '
          f'yaw {math.degrees(sum(yaw_err) / n):4.2f} deg  [{n} ventanas]')


if __name__ == '__main__':
    main()
