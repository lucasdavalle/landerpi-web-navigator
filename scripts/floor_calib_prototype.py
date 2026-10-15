#!/usr/bin/env python3
# Stage 6b prototype (2026-10-15): floor-plane extrinsic calibration of the
# depth camera for the current arm posture. RANSAC plane on the lower half
# of the depth image -> camera height, pitch and roll; compares with the
# URDF chain. Superseded by the C++ FloorCalibrator once written.
import math, time
import numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = Node('floor_calib'); got = {}
n.create_subscription(Image, '/ascamera/camera_publisher/depth0/image_raw', lambda m: got.setdefault('d', m), 2)
n.create_subscription(CameraInfo, '/ascamera/camera_publisher/depth0/camera_info', lambda m: got.setdefault('k', m), 2)
buf = Buffer(); TransformListener(buf, n)
end = time.time() + 8
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.1)
d = np.frombuffer(got['d'].data, np.uint16).reshape(got['d'].height, got['d'].width).astype(float) / 1000.0
K = got['k'].k; fx, fy, cx, cy = K[0], K[4], K[2], K[5]
v, u = np.mgrid[0:d.shape[0]:4, 0:d.shape[1]:4]; z = d[::4, ::4]
ok = z > 0
# optical convention: x right, y down, z forward
P = np.stack([(u[ok] - cx) * z[ok] / fx, (v[ok] - cy) * z[ok] / fy, z[ok]], 1)
# floor = lower half of the image (rows below the horizon), RANSAC plane
low = P[(v[ok] > d.shape[0] * 0.55)]
rng = np.random.default_rng(0); best = None
for _ in range(300):
    s = low[rng.choice(len(low), 3, replace=False)]
    nrm = np.cross(s[1] - s[0], s[2] - s[0]); 
    if np.linalg.norm(nrm) < 1e-9: continue
    nrm /= np.linalg.norm(nrm); dist = np.abs((low - s[0]) @ nrm)
    inl = dist < 0.01
    if best is None or inl.sum() > best[0]: best = (inl.sum(), nrm, s[0])
_, nrm, p0 = best
inl = low[np.abs((low - p0) @ nrm) < 0.01]
c = inl.mean(0); _, _, vt = np.linalg.svd(inl - c); nrm = vt[2]
if nrm[1] > 0: nrm = -nrm   # floor normal points "up" = -y in optical
print(f'piso: {len(inl)} puntos de {len(low)}; normal optica {np.round(nrm,3)}')
tf = buf.lookup_transform('base_footprint', 'depth_cam_link', rclpy.time.Time()).transform
q = tf.rotation; t = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
x, y, zq, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+zq*zq), 2*(x*y-zq*w), 2*(x*zq+y*w)], [2*(x*y+zq*w), 1-2*(x*x+zq*zq), 2*(y*zq-x*w)], [2*(x*zq-y*w), 2*(y*zq+x*w), 1-2*(x*x+y*y)]])
R_opt = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])  # body <- optical (x=z_opt, y=-x_opt, z=-y_opt)
for name, Rc in (('A: depth_camera_link optico (rot estandar)', R @ R_opt), ('B: identico a depth_cam_link', R)):
    n_b = Rc @ nrm; c_b = Rc @ c + t
    tilt = math.degrees(math.acos(min(1, abs(n_b[2]))))
    print(f'{name}: normal en robot {np.round(n_b,3)} (inclinacion {tilt:.1f} deg), altura del piso {c_b[2]*100:+.1f} cm')
# pitch error estimate for candidate A: angle between camera-predicted floor normal and true up
Rc = R @ R_opt; n_b = Rc @ nrm
print('correccion de pitch sugerida (A): %.1f deg' % math.degrees(math.atan2(n_b[0], n_b[2])))
# --- floor-based extrinsics: height, pitch, roll of the optical frame ---
H = abs(nrm @ c)                       # camera distance to the floor plane
pitch = math.degrees(math.atan2(-nrm[2], -nrm[1]))   # down-tilt of the optical axis
roll = math.degrees(math.atan2(nrm[0], -nrm[1]))
print(f'CALIBRADO contra el piso: altura camara {H*100:.1f} cm, pitch {pitch:.1f} deg hacia abajo, roll {roll:.1f} deg')
print(f'URDF decia: altura {t[2]*100:.1f} cm, pitch 32.8 deg')
# check: rotate so the measured floor normal becomes "up" (minimal rotation),
# then floor points must sit at height ~0.
b = R_opt @ nrm                      # floor normal in an un-tilted body frame
up = np.array([0.0, 0.0, 1.0]); ax = np.cross(b, up); sa = np.linalg.norm(ax); ca = b @ up
Kx = np.array([[0,-ax[2],ax[1]],[ax[2],0,-ax[0]],[-ax[1],ax[0],0]]) / (sa if sa > 1e-12 else 1)
Rmin = np.eye(3) + sa * Kx + (1 - ca) * (Kx @ Kx)
lvl = (Rmin @ R_opt @ inl.T).T
hz = lvl[:, 2] + H
print('verificacion: alturas del piso re-proyectadas: media %.1f cm, desvio %.1f cm, rango 2-98%% [%.1f, %.1f] cm' % (hz.mean()*100, hz.std()*100, np.percentile(hz,2)*100, np.percentile(hz,98)*100))
allp = (Rmin @ R_opt @ P.T).T; allh = allp[:, 2] + H
print('toda la imagen: %.0f %% de los puntos a menos de 3 cm (piso), %.0f %% entre 3 y 31 cm, %.0f %% arriba de 31 cm' % (100*(allh<0.03).mean(), 100*((allh>=0.03)&(allh<0.31)).mean(), 100*(allh>=0.31).mean()))
