// Messages from the gateway (see landerpi_web_nav/message_codec.hpp).
// JSON for small things, little-endian binary frames for the map and scan.

export interface Pose {
  x: number;
  y: number;
  yaw: number;
  /** Position covariance [xx, xy, yy] in m^2, once SLAM has published one. */
  cov?: [number, number, number];
  cov_yaw?: number;
  t: number;
}

/** A wall: [x1, y1, x2, y2] in metres, map frame. */
export type Wall = [number, number, number, number];

export interface Status {
  battery_mv: number | null;
  sensors: Record<string, number | null>;  // seconds since last message
  slam_running: boolean;
  clients: number;
}

export interface DriveState {
  driver: number | null;  // client id of whoever is driving
  gate: string;           // velocity_gate reason: idle, no_heartbeat, teleop...
  posture?: string;       // arm: none, moving, calibrating, high, low (stage 6)
}

export interface NavState {
  /** idle, pending, active, succeeded, aborted, canceled, rejected, unavailable */
  state: string;
  goal: [number, number] | null;
  distance: number | null;   // m remaining
  eta: number | null;        // s remaining (Nav2's estimate)
  recoveries: number;
}

/** Planned path from Nav2, map frame. */
export type PathPoints = Array<[number, number]>;

export interface GridMap {
  width: number;
  height: number;
  resolution: number;
  originX: number;
  originY: number;
  /** -1 unknown, 0..100 occupancy; row 0 is the lowest y. */
  cells: Int8Array;
}

export interface Scan {
  robotX: number;
  robotY: number;
  robotYaw: number;
  /** Interleaved x, y in the map frame. */
  points: Float32Array;
}

const MAP = 1;
const SCAN = 2;
const VERSION = 1;

export function decodeBinary(buf: ArrayBuffer): { map?: GridMap; scan?: Scan } {
  const v = new DataView(buf);
  if (buf.byteLength < 4 || v.getUint8(1) !== VERSION) {
    return {};
  }
  const type = v.getUint8(0);
  if (type === MAP) {
    const width = v.getUint32(4, true);
    const height = v.getUint32(8, true);
    return {
      map: {
        width,
        height,
        resolution: v.getFloat32(12, true),
        originX: v.getFloat32(16, true),
        originY: v.getFloat32(20, true),
        cells: new Int8Array(buf, 24, width * height),
      },
    };
  }
  if (type === SCAN) {
    const count = v.getUint32(4, true);
    // Copy instead of a view: Float32Array views need 4-byte alignment and
    // the points start at byte 20, which is aligned, but copying keeps it
    // independent of the buffer anyway.
    const points = new Float32Array(count * 2);
    for (let i = 0; i < count * 2; i++) {
      points[i] = v.getFloat32(20 + 4 * i, true);
    }
    return {
      scan: {
        robotX: v.getFloat32(8, true),
        robotY: v.getFloat32(12, true),
        robotYaw: v.getFloat32(16, true),
        points,
      },
    };
  }
  return {};
}
