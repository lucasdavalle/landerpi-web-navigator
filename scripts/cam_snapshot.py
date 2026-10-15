#!/usr/bin/env python3
# Copyright 2026 ldavalle
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Stage 6a helper: one RGB + depth snapshot from the Aurora930 and the arm's
current (commanded) pulses, to choose the navigation posture.

Writes <out>/rgb.png and <out>/depth.png (colourised, with the distance in
metres written on a grid) and prints depth statistics. No cv_bridge needed.
"""

import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from servo_controller_msgs.msg import ServoStateList

OUT = sys.argv[1] if len(sys.argv) > 1 else '/home/ubuntu/shared/a7_cam'


def to_array(msg):
    if msg.encoding in ('rgb8', 'bgr8'):
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img.copy()
    if msg.encoding in ('mono16', '16UC1'):
        return np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width).copy()
    raise ValueError(msg.encoding)


def main():
    rclpy.init()
    n = Node('cam_snapshot')
    got = {}
    n.create_subscription(Image, '/ascamera/camera_publisher/rgb0/image',
                          lambda m: got.setdefault('rgb', m), 2)
    n.create_subscription(Image, '/ascamera/camera_publisher/depth0/image_raw',
                          lambda m: got.setdefault('depth', m), 2)
    n.create_subscription(ServoStateList, '/controller_manager/servo_states',
                          lambda m: got.setdefault('servos', m), 2)
    end = time.time() + 10
    while len(got) < 3 and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.2)
    import os
    os.makedirs(OUT, exist_ok=True)
    if 'servos' in got:
        print('pulsos del brazo (comandados):',
              {s.id: s.position for s in got['servos'].servo_state})
    if 'rgb' in got:
        cv2.imwrite(f'{OUT}/rgb.png', to_array(got['rgb']))
    if 'depth' not in got:
        print('sin imagen de profundidad')
        return
    d = to_array(got['depth']).astype(np.float32) / 1000.0  # mm -> m
    valid = d > 0
    print('profundidad %dx%d: validos %.0f %%, min %.2f m, mediana %.2f m, max %.2f m' % (
        d.shape[1], d.shape[0], 100 * valid.mean(), d[valid].min(), np.median(d[valid]),
        d[valid].max()))
    vis = np.zeros((*d.shape, 3), np.uint8)
    norm = np.clip(d / 3.0, 0, 1)
    vis[valid] = cv2.applyColorMap((255 * (1 - norm[valid])).astype(np.uint8).reshape(-1, 1),
                                   cv2.COLORMAP_JET).reshape(-1, 3)
    h, w = d.shape
    for gy in range(1, 5):
        for gx in range(1, 6):
            y, x = gy * h // 5, gx * w // 6
            patch = d[max(0, y - 4):y + 4, max(0, x - 4):x + 4]
            pv = patch[patch > 0]
            txt = '%.2f' % np.median(pv) if pv.size else '--'
            cv2.putText(vis, txt, (x - 20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.putText(vis, txt, (x - 20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    cv2.imwrite(f'{OUT}/depth.png', vis)
    print(f'guardado en {OUT}/rgb.png y depth.png')


if __name__ == '__main__':
    main()
