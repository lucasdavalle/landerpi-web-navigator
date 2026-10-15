import { Injectable, effect, inject, signal } from '@angular/core';

import { AuthService } from './auth.service';
import {
  DriveState, GridMap, MissionState, NavState, PathPoints, Pose, Scan, Status, Wall,
  decodeBinary,
} from './protocol';

export type Link = 'disconnected' | 'connecting' | 'connected';

/**
 * The single WebSocket to the gateway. Authenticates with the first
 * message, turns every incoming frame into a signal, and reconnects with
 * backoff while logged in.
 */
@Injectable({ providedIn: 'root' })
export class RobotSocket {
  private readonly auth = inject(AuthService);
  private ws: WebSocket | null = null;
  private retryMs = 500;
  private retryTimer: ReturnType<typeof setTimeout> | null = null;

  readonly link = signal<Link>('disconnected');
  readonly clientId = signal<number | null>(null);
  readonly map = signal<GridMap | null>(null);
  /** What the depth camera found (stage 6c): crouchable cells, low obstacles. */
  readonly depthGrid = signal<GridMap | null>(null);
  readonly walls = signal<Wall[]>([]);
  readonly scan = signal<Scan | null>(null);
  readonly pose = signal<Pose | null>(null);
  readonly status = signal<Status | null>(null);
  readonly drive = signal<DriveState>({ driver: null, gate: 'unknown' });
  readonly nav = signal<NavState>({
    state: 'idle', goal: null, distance: null, eta: null, recoveries: 0,
  });
  readonly path = signal<PathPoints>([]);
  readonly mission = signal<MissionState | null>(null);

  constructor() {
    // Connect while there is a token; close when it goes away.
    effect(() => {
      if (this.auth.token()) {
        this.connect();
      } else {
        this.close();
      }
    }, { allowSignalWrites: true });
  }

  send(msg: object): void {
    if (this.ws?.readyState === WebSocket.OPEN && this.link() === 'connected') {
      this.ws.send(JSON.stringify(msg));
    }
  }

  private connect(): void {
    if (this.ws) {
      return;
    }
    const token = this.auth.token();
    if (!token) {
      return;
    }
    this.link.set('connecting');
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    const ws = new WebSocket(`${proto}://${location.host}/ws`);
    ws.binaryType = 'arraybuffer';
    this.ws = ws;

    ws.onopen = () => ws.send(JSON.stringify({ type: 'auth', token }));
    ws.onmessage = (ev) => this.onMessage(ev.data);
    ws.onclose = () => {
      const wasConnected = this.link() === 'connected';
      this.ws = null;
      this.link.set('disconnected');
      this.clientId.set(null);
      if (!wasConnected && this.auth.token()) {
        // Closed before auth_ok: most likely an expired token.
        this.auth.forget();
        return;
      }
      this.scheduleReconnect();
    };
  }

  private scheduleReconnect(): void {
    if (!this.auth.token() || this.retryTimer) {
      return;
    }
    this.retryTimer = setTimeout(() => {
      this.retryTimer = null;
      this.connect();
    }, this.retryMs);
    this.retryMs = Math.min(this.retryMs * 2, 8000);
  }

  private close(): void {
    if (this.retryTimer) {
      clearTimeout(this.retryTimer);
      this.retryTimer = null;
    }
    this.ws?.close();
    this.ws = null;
    this.link.set('disconnected');
  }

  private onMessage(data: string | ArrayBuffer): void {
    if (data instanceof ArrayBuffer) {
      const { map, scan, depthGrid } = decodeBinary(data);
      if (map) this.map.set(map);
      if (scan) this.scan.set(scan);
      if (depthGrid) this.depthGrid.set(depthGrid);
      return;
    }
    const msg = JSON.parse(data);
    switch (msg.type) {
      case 'auth_ok':
        this.link.set('connected');
        this.clientId.set(msg.client);
        this.retryMs = 500;
        break;
      case 'pose':
        this.pose.set(msg as Pose);
        break;
      case 'walls':
        this.walls.set(msg.segments as Wall[]);
        break;
      case 'status':
        this.status.set(msg as Status);
        break;
      case 'drive':
        this.drive.set({
          driver: msg.driver, gate: msg.gate, posture: msg.posture,
          arm_target: msg.arm_target, stand_check: msg.stand_check,
          crouch_status: msg.crouch_status,
        });
        break;
      case 'nav':
        this.nav.set(msg as NavState);
        if (msg.state !== 'pending' && msg.state !== 'active') this.path.set([]);
        break;
      case 'mission':
        this.mission.set(msg as MissionState);
        break;
      case 'path':
        if (this.nav().state === 'pending' || this.nav().state === 'active') {
          this.path.set(msg.points as PathPoints);
        }
        break;
    }
  }
}
