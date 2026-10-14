import { Component, computed, inject } from '@angular/core';

import { AuthService } from './auth.service';
import { RobotSocket } from './robot-socket.service';

@Component({
  selector: 'a7-status-bar',
  standalone: true,
  template: `
    <span class="title">LanderPi Navigator</span>
    <span class="item" [class]="linkClass()">● {{ linkText() }}</span>
    @if (battery(); as b) {
      <span class="item" [class.warn]="b.pct < 30" [class.bad]="b.pct < 15">
        Batería {{ b.volts }} V (~{{ b.pct }} %)
      </span>
    }
    @for (s of sensors(); track s.name) {
      <span class="item" [class.bad]="!s.ok" [title]="s.title">{{ s.name }}</span>
    }
    <span class="spacer"></span>
    <button (click)="auth.logout()">Salir</button>
  `,
  styles: [`
    :host { display: flex; align-items: center; gap: 14px; padding: 8px 14px;
            background: var(--panel); border-bottom: 1px solid var(--border); }
    .title { font-weight: 600; }
    .item { color: var(--muted); }
    .ok { color: var(--ok); }
    .warn { color: var(--warn); }
    .bad { color: var(--danger); }
    .spacer { flex: 1; }
  `],
})
export class StatusBarComponent {
  readonly auth = inject(AuthService);
  private readonly socket = inject(RobotSocket);

  readonly linkText = computed(() => ({
    connected: 'conectado', connecting: 'conectando…', disconnected: 'sin conexión',
  })[this.socket.link()]);
  readonly linkClass = computed(() =>
    this.socket.link() === 'connected' ? 'ok' : this.socket.link() === 'connecting' ? 'warn' : 'bad');

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
}
