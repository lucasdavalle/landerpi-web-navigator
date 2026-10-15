import type * as THREE_NS from 'three';

type Three = typeof THREE_NS;

/**
 * Arm joint angles in radians, in the model's own convention: the shoulder
 * and elbow tilt the arm forward when negative (0 = straight up), the wrist
 * tilts the camera forward when negative, the base turns the whole arm.
 */
export interface ArmAngles {
  base: number;
  shoulder: number;
  elbow: number;
  wrist: number;
}

const deg = Math.PI / 180;

/**
 * The postures the robot really uses (posture_manager.yaml), as they look:
 * not a kinematic model of the servos (the stock URDF is ~10 deg off and
 * the servos give no feedback), but angles tuned so each pose matches what
 * was measured on the robot.
 */
export const POSES: Record<string, ArmAngles> = {
  // 33 cm tall, camera the highest point looking ahead 12 deg down.
  high: { base: 0, shoulder: 35 * deg, elbow: -40 * deg, wrist: -97 * deg },
  // 20 cm tall, arm straight ahead, sticking out 23 cm, camera level.
  low: { base: 0, shoulder: -85 * deg, elbow: 0, wrist: -10 * deg },
  // 19 cm, folded down with the camera looking straight up.
  look_up: { base: 0, shoulder: -100 * deg, elbow: 150 * deg, wrist: -50 * deg },
  // Power-on pose of the stock bringup, before any posture.
  none: { base: 0, shoulder: 20 * deg, elbow: -60 * deg, wrist: -40 * deg },
};

export function lerpAngles(a: ArmAngles, b: ArmAngles, t: number): ArmAngles {
  // The servos all start and stop together (one ServosPosition message):
  // a smooth ease looks like the real move.
  const k = t <= 0 ? 0 : t >= 1 ? 1 : t * t * (3 - 2 * t);
  return {
    base: a.base + (b.base - a.base) * k,
    shoulder: a.shoulder + (b.shoulder - a.shoulder) * k,
    elbow: a.elbow + (b.elbow - a.elbow) * k,
    wrist: a.wrist + (b.wrist - a.wrist) * k,
  };
}

export interface RobotModel {
  root: THREE_NS.Group;
  setArm(a: ArmAngles): void;
  /** Spins the lidar head and its sweep (call every frame with seconds). */
  animate(t: number): void;
  showSweep(on: boolean): void;
}

/**
 * The LanderPi built from simple shapes. Three.js has Y up; the robot's x
 * (forward) is +X, its z (up) is +Y, its y (left) is -Z. Metres.
 */
export function buildRobot(THREE: Three): RobotModel {
  const root = new THREE.Group();

  const body = new THREE.MeshStandardMaterial({ color: 0x2b3440, metalness: 0.4, roughness: 0.45 });
  const plate = new THREE.MeshStandardMaterial({ color: 0x3d4a5a, metalness: 0.5, roughness: 0.35 });
  const accent = new THREE.MeshStandardMaterial({ color: 0x4fa3ff, metalness: 0.2, roughness: 0.3, emissive: 0x0b2a4d });
  const tyre = new THREE.MeshStandardMaterial({ color: 0x15181d, roughness: 0.9 });
  const roller = new THREE.MeshStandardMaterial({ color: 0x9aa4b2, metalness: 0.6, roughness: 0.3 });
  const servo = new THREE.MeshStandardMaterial({ color: 0x1b1f25, metalness: 0.3, roughness: 0.6 });
  const link = new THREE.MeshStandardMaterial({ color: 0xc9d1db, metalness: 0.7, roughness: 0.25 });
  const lens = new THREE.MeshStandardMaterial({ color: 0x0a0c10, metalness: 0.9, roughness: 0.1, emissive: 0x101826 });

  const box = (w: number, h: number, d: number, m: THREE_NS.Material) =>
    new THREE.Mesh(new THREE.BoxGeometry(w, h, d), m);

  // Chassis: 22 x 18 cm (measured), ~7 cm deep, on 4 wheels.
  const chassis = box(0.22, 0.06, 0.16, body);
  chassis.position.y = 0.075;
  root.add(chassis);
  const top = box(0.2, 0.012, 0.15, plate);
  top.position.y = 0.111;
  root.add(top);
  const stripe = box(0.222, 0.008, 0.162, accent);
  stripe.position.y = 0.06;
  root.add(stripe);

  // Mecanum wheels: a tyre with rollers at 45 degrees around it.
  for (const [x, z] of [[0.075, 0.1], [0.075, -0.1], [-0.075, 0.1], [-0.075, -0.1]]) {
    const wheel = new THREE.Group();
    const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.038, 0.038, 0.03, 24), tyre);
    hub.rotation.x = Math.PI / 2;
    wheel.add(hub);
    const sign = x * z > 0 ? 1 : -1;
    for (let i = 0; i < 10; i++) {
      const a = (i / 10) * Math.PI * 2;
      const r = new THREE.Mesh(new THREE.CapsuleGeometry(0.006, 0.022, 4, 8), roller);
      r.position.set(Math.cos(a) * 0.038, Math.sin(a) * 0.038, 0);
      r.rotation.z = a;
      r.rotateX((sign * Math.PI) / 4);
      wheel.add(r);
    }
    wheel.position.set(x, 0.04, z);
    root.add(wheel);
  }

  // Lidar on the back of the top plate, with a spinning head.
  const lidarBase = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.018, 32), servo);
  lidarBase.position.set(-0.045, 0.126, 0);
  root.add(lidarBase);
  const lidarHead = new THREE.Group();
  const head = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.032, 0.016, 32), plate);
  lidarHead.add(head);
  const window_ = box(0.012, 0.008, 0.02, accent);
  window_.position.set(0.028, 0, 0);
  lidarHead.add(window_);
  lidarHead.position.set(-0.045, 0.143, 0);
  root.add(lidarHead);

  // The lidar's sweep: a translucent wedge turning with the head.
  const sweep = new THREE.Mesh(
    new THREE.CircleGeometry(0.9, 48, 0, Math.PI / 5),
    new THREE.MeshBasicMaterial({
      color: 0x5cff9d, transparent: true, opacity: 0.18, side: THREE.DoubleSide, depthWrite: false,
    }),
  );
  sweep.rotation.x = -Math.PI / 2;
  const sweepPivot = new THREE.Group();
  sweepPivot.position.set(-0.045, 0.143, 0);
  sweepPivot.add(sweep);
  sweepPivot.visible = false;
  root.add(sweepPivot);

  // Arm: base turret, shoulder, upper arm, elbow, forearm, wrist, camera.
  const turret = new THREE.Group();
  turret.position.set(0.06, 0.117, 0);
  root.add(turret);
  const turretMesh = new THREE.Mesh(new THREE.CylinderGeometry(0.028, 0.032, 0.022, 24), servo);
  turretMesh.position.y = 0.011;
  turret.add(turretMesh);

  const joint = (parent: THREE_NS.Object3D, y: number) => {
    const g = new THREE.Group();
    g.position.y = y;
    parent.add(g);
    const s = box(0.03, 0.03, 0.04, servo);
    g.add(s);
    return g;
  };
  const segment = (parent: THREE_NS.Object3D, length: number) => {
    const m = box(0.022, length, 0.03, link);
    m.position.y = length / 2;
    parent.add(m);
  };

  const shoulder = joint(turret, 0.03);
  segment(shoulder, 0.085);
  const elbow = joint(shoulder, 0.085);
  segment(elbow, 0.085);
  const wrist = joint(elbow, 0.085);

  // Depth camera (Aurora930) at the wrist, looking along the wrist's +Y.
  const cam = new THREE.Group();
  cam.position.y = 0.03;
  wrist.add(cam);
  const camBody = box(0.026, 0.03, 0.09, body);
  cam.add(camBody);
  const camFace = box(0.002, 0.024, 0.08, accent);
  camFace.position.set(0, 0.016, 0);
  camFace.rotation.z = Math.PI / 2;
  cam.add(camFace);
  for (const z of [-0.025, 0, 0.025]) {
    const l = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, 0.004, 16), lens);
    l.position.set(0, 0.017, z);
    cam.add(l);
  }
  // Gripper fingers under the camera (the gripper's servo is burned: static).
  for (const z of [-0.012, 0.012]) {
    const f = box(0.008, 0.03, 0.006, servo);
    f.position.set(-0.016, 0.02, z);
    cam.add(f);
  }

  // Rotations: the arm bends in the robot's x-z plane, i.e. about three's Z.
  // Negative model angles lean forward (+X), which is a negative rotation
  // about Z in three.js (right-handed, Y up).
  const setArm = (a: ArmAngles) => {
    turret.rotation.y = a.base;
    shoulder.rotation.z = a.shoulder;
    elbow.rotation.z = a.elbow;
    wrist.rotation.z = a.wrist;
  };
  setArm(POSES['none']);

  root.traverse((o) => {
    const mesh = o as THREE_NS.Mesh;
    if (mesh.isMesh && mesh !== sweep) {
      mesh.castShadow = true;
      mesh.receiveShadow = true;
    }
  });

  return {
    root,
    setArm,
    animate: (t: number) => {
      lidarHead.rotation.y = t * 6;
      sweepPivot.rotation.y = t * 6;
    },
    showSweep: (on: boolean) => {
      sweepPivot.visible = on;
    },
  };
}
