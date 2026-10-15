import { Component, computed, inject } from '@angular/core';

import { AuthService } from './auth.service';
import { RobotSocket } from './robot-socket.service';

/** Header: brand, link, battery, sensors and the arm's posture as chips. */
@Component({
  selector: 'a7-status-bar',
  standalone: true,
  template: `
    <div class="brand">
      <svg viewBox="0 0 32 32" width="26" height="26" aria-hidden="true">
        <rect x="4" y="13" width="24" height="11" rx="3" fill="#4fa3ff"/>
        <circle cx="10" cy="26" r="3" fill="#e6ebf2"/><circle cx="22" cy="26" r="3" fill="#e6ebf2"/>
        <path d="M18 13 L18 7 L24 4" stroke="#e6ebf2" stroke-width="2.4" fill="none" stroke-linecap="round"/>
        <circle cx="11" cy="10" r="2.6" fill="#3ecf8e"/>
      </svg>
      <div>
        <b>LanderPi</b>
        <small>Web Navigator</small>
      </div>
    </div>
    <div class="chips">
      <span class="chip" [class]="linkClass()">{{ linkText() }}</span>
      @if (battery(); as b) {
        <span class="chip" [class.ok]="b.pct >= 30" [class.warn]="b.pct < 30 && b.pct >= 15"
              [class.bad]="b.pct < 15" title="Batería">
          {{ b.volts }} V · {{ b.pct }} %
        </span>
      }
      @for (s of sensors(); track s.name) {
        <span class="chip" [class.ok]="s.ok" [class.bad]="!s.ok" [title]="s.title">{{ s.name }}</span>
      }
      <span class="chip" [class]="postureClass()" title="Postura del brazo">{{ postureText() }}</span>
    </div>
    <button class="ghost" (click)="auth.logout()">Salir</button>
  `,
  styles: [`
    :host { display: flex; align-items: center; gap: 16px; padding: 12px 16px; }
    .brand { display: flex; align-items: center; gap: 10px; }
    .brand b { display: block; font-size: 15px; letter-spacing: 0.01em; }
    .brand small { display: block; color: var(--muted); font-size: 11px; margin-top: -1px; }
    .chips { flex: 1; display: flex; flex-wrap: wrap; gap: 6px; justify-content: flex-end; }
  `],
})
export class StatusBarComponent {
  readonly auth = inject(AuthService);
  private readonly socket = inject(RobotSocket);

  readonly linkText = computed(() => ({
    connected: 'conectado', connecting: 'conectando…', disconnected: 'sin conexión',
  })[this.socket.link()]);
  readonly linkClass = computed(() =>
    'chip ' + (this.socket.link() === 'connected' ? 'ok'
      : this.socket.link() === 'connecting' ? 'warn' : 'bad'));

  // 2S Li-ion: 6.6 V empty, 8.4 V full (linear: a rough guide only).
  readonly battery = computed(() => {
    const mv = this.socket.status()?.battery_mv;
    if (mv == null) return null;
    const pct = Math.round(Math.min(100, Math.max(0, ((mv - 6600) / 1800) * 100)));
    return { volts: (mv / 1000).toFixed(2), pct };
  });

  readonly sensors = computed(() => {
    const s = this.socket.status()?.sensors ?? {};
    const names: Record<string, string> = { scan: 'lidar', odom: 'odometría', map: 'mapa' };
    return Object.entries(names).map(([key, name]) => {
      const age = s[key];
      // The map updates about once a second; the others ~10+ Hz.
      const limit = key === 'map' ? 5 : 1;
      return { name, ok: age != null && age < limit,
        title: age == null ? 'sin datos' : `hace ${age.toFixed(1)} s` };
    });
  });

  private readonly postureTexts: Record<string, string> = {
    none: 'brazo sin postura', moving: 'brazo moviéndose', calibrating: 'calibrando cámara',
    high: 'parado', low: 'agachado', looking_up: 'mirando arriba', checking: 'mirando arriba',
  };
  readonly postureText = computed(() => {
    const p = this.socket.drive().posture ?? 'none';
    return this.postureTexts[p] ?? p;
  });
  readonly postureClass = computed(() => {
    const p = this.socket.drive().posture ?? 'none';
    return 'chip ' + (p === 'high' ? 'ok' : p === 'low' ? 'crouch' : p === 'none' ? '' : 'info');
  });
}
