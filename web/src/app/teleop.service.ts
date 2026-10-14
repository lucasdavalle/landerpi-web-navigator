import { Injectable, computed, inject, signal } from '@angular/core';

import { RobotSocket } from './robot-socket.service';

interface Command { vx: number; vy: number; wz: number; }

const KEYS: Record<string, Command> = {
  w: { vx: 1, vy: 0, wz: 0 },
  s: { vx: -1, vy: 0, wz: 0 },
  a: { vx: 0, vy: 1, wz: 0 },   // mecanum: sideways left
  d: { vx: 0, vy: -1, wz: 0 },
  q: { vx: 0, vy: 0, wz: 1 },   // turn left
  e: { vx: 0, vy: 0, wz: -1 },
};

/**
 * Keyboard driving. While this browser is the driver it sends a heartbeat
 * every 200 ms (the deadman in velocity_gate trips after 500 ms without
 * one) and the teleop command every 100 ms. Losing the keyboard (window
 * blur, tab hidden) releases every key, so the robot never keeps going
 * with nobody pressing.
 */
@Injectable({ providedIn: 'root' })
export class TeleopService {
  private readonly socket = inject(RobotSocket);
  private readonly pressed = new Set<string>();
  private timer: ReturnType<typeof setInterval> | null = null;
  private tick = 0;

  /** Fraction (0.2 .. 1) of the velocity_gate limits. */
  readonly speed = signal(0.5);
  readonly command = signal<Command>({ vx: 0, vy: 0, wz: 0 });

  readonly isDriver = computed(() => {
    const id = this.socket.clientId();
    return id !== null && this.socket.drive().driver === id;
  });
  readonly someoneElseDrives = computed(() => {
    const driver = this.socket.drive().driver;
    return driver !== null && driver !== this.socket.clientId();
  });

  constructor() {
    window.addEventListener('keydown', (e) => this.onKey(e, true));
    window.addEventListener('keyup', (e) => this.onKey(e, false));
    window.addEventListener('blur', () => this.releaseAll());
    document.addEventListener('visibilitychange', () => {
      if (document.hidden) this.releaseAll();
    });
  }

  enable(): void {
    this.socket.send({ type: 'drive', enable: true });
    this.startLoop();
  }

  disable(): void {
    this.releaseAll();
    this.socket.send({ type: 'teleop', vx: 0, vy: 0, wz: 0 });
    this.socket.send({ type: 'drive', enable: false });
    this.stopLoop();
  }

  private onKey(e: KeyboardEvent, down: boolean): void {
    const k = e.key.toLowerCase();
    if (!(k in KEYS)) return;
    // Do not steal keys from a text field (e.g. the login box).
    if ((e.target as HTMLElement | null)?.tagName === 'INPUT') return;
    e.preventDefault();
    if (down) this.pressed.add(k); else this.pressed.delete(k);
    this.updateCommand();
  }

  private releaseAll(): void {
    this.pressed.clear();
    this.updateCommand();
  }

  private updateCommand(): void {
    const s = this.speed();
    const c: Command = { vx: 0, vy: 0, wz: 0 };
    for (const k of this.pressed) {
      c.vx += KEYS[k].vx;
      c.vy += KEYS[k].vy;
      c.wz += KEYS[k].wz;
    }
    // velocity_gate clamps to its limits (0.25 m/s, 1.0 rad/s by default):
    // sending full scale times the speed fraction lands inside them.
    this.command.set({ vx: c.vx * 0.25 * s, vy: c.vy * 0.25 * s, wz: c.wz * 1.0 * s });
  }

  private startLoop(): void {
    if (this.timer) return;
    this.tick = 0;
    this.timer = setInterval(() => {
      if (!this.isDriver()) return;
      if (this.tick++ % 2 === 0) this.socket.send({ type: 'heartbeat' });
      this.socket.send({ type: 'teleop', ...this.command() });
    }, 100);
  }

  private stopLoop(): void {
    if (this.timer) {
      clearInterval(this.timer);
      this.timer = null;
    }
  }
}
