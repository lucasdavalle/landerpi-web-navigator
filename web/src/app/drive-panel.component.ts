import { Component, computed, inject } from '@angular/core';

import { RobotSocket } from './robot-socket.service';
import { TeleopService } from './teleop.service';

/** Side panel: motion, navigation, arm and keyboard, as cards. */
@Component({
  selector: 'a7-drive-panel',
  standalone: true,
  template: `
    <section class="card">
      <h2>Movimiento</h2>
      @if (teleop.isDriver()) {
        <button class="danger wide" (click)="teleop.disable()">Deshabilitar movimiento</button>
      } @else {
        <button class="primary wide" (click)="teleop.enable()"
                [disabled]="teleop.someoneElseDrives() || socket.link() !== 'connected'">
          Habilitar movimiento
        </button>
      }
      @if (teleop.someoneElseDrives()) {
        <p class="note">Otro navegador está manejando.</p>
      }
      <div class="row">
        <span class="muted">Estado</span>
        <span class="chip" [class]="gateClass()">{{ gateText() }}</span>
      </div>
    </section>

    <section class="card">
      <h2>Navegación</h2>
      <div class="row">
        <span class="big">{{ navText() }}</span>
        @if (navActive() && socket.nav().distance != null) {
          <span class="muted">{{ socket.nav().distance!.toFixed(2) }} m
            @if (socket.nav().eta) { · ~{{ socket.nav().eta!.toFixed(0) }} s }</span>
        }
      </div>
      @if (navActive()) {
        <div class="progress"><div [style.width.%]="progress()"></div></div>
        <button class="danger wide" (click)="socket.send({ type: 'cancel' })">Cancelar</button>
      } @else {
        <p class="note">Click en el mapa → "Ir acá". Cualquier tecla cancela.</p>
      }
    </section>

    <section class="card">
      <h2>Brazo y altura</h2>
      <div class="row">
        <span class="muted">Postura</span>
        <span class="chip" [class]="postureClass()">{{ postureText() }}</span>
      </div>
      @if (socket.drive().crouch_status; as c) {
        <p class="note">{{ c }}</p>
      }
      @if (verdict(); as v) {
        <p class="note">Último chequeo arriba: <b [class]="v.cls">{{ v.text }}</b></p>
      }
    </section>

    <section class="card">
      <h2>Teclado</h2>
      <label>
        <span class="row"><span class="muted">Velocidad</span>
          <b>{{ (teleop.speed() * 100).toFixed(0) }} %</b></span>
        <input type="range" min="0.2" max="1" step="0.1"
               [value]="teleop.speed()" (input)="setSpeed($event)">
      </label>
      <div class="keys" [class.active]="teleop.isDriver()">
        <kbd>Q</kbd><kbd>W</kbd><kbd>E</kbd>
        <kbd>A</kbd><kbd>S</kbd><kbd>D</kbd>
      </div>
      <p class="note">W/S adelante/atrás · A/D de costado · Q/E girar · soltar frena.</p>
      @if (teleop.isDriver()) {
        <p class="cmd">vx {{ teleop.command().vx.toFixed(2) }} · vy {{ teleop.command().vy.toFixed(2) }}
          · wz {{ teleop.command().wz.toFixed(2) }}</p>
      }
    </section>
  `,
  styles: [`
    :host { display: flex; flex-direction: column; gap: 12px; width: 300px; overflow-y: auto; }
    .wide { width: 100%; }
    .row { display: flex; align-items: center; justify-content: space-between; gap: 8px;
           margin-top: 10px; }
    .card > .row:first-of-type { margin-top: 0; }
    .big { font-size: 16px; font-weight: 600; }
    .note, .cmd { color: var(--muted); margin: 10px 0 0; font-size: 12px; line-height: 1.45; }
    .cmd { font-family: ui-monospace, monospace; }
    label { display: flex; flex-direction: column; gap: 8px; }
    label .row { margin-top: 0; }
    .progress { height: 6px; border-radius: 3px; background: var(--bg-2); overflow: hidden;
                margin: 12px 0; }
    .progress div { height: 100%; background: linear-gradient(90deg, var(--accent), var(--accent-2));
                    transition: width 0.5s; }
    .keys { display: grid; grid-template-columns: repeat(3, 44px); gap: 6px;
            justify-content: center; margin-top: 12px; opacity: 0.35; transition: opacity 0.2s; }
    .keys.active { opacity: 1; }
    kbd { display: grid; place-items: center; height: 40px; border: 1px solid var(--border);
          border-bottom-width: 3px; border-radius: 8px; background: var(--panel-2);
          font-family: inherit; font-weight: 600; }
  `],
})
export class DrivePanelComponent {
  readonly teleop = inject(TeleopService);
  readonly socket = inject(RobotSocket);

  private readonly texts: Record<string, string> = {
    idle: 'deshabilitado', no_heartbeat: 'sin señal (deadman)', no_command: 'listo, quieto',
    teleop: 'manejando', nav: 'navegando', arm_moving: 'quieto: brazo moviéndose',
    unknown: 'sin datos',
  };
  readonly gateText = computed(() => this.texts[this.socket.drive().gate] ?? this.socket.drive().gate);
  readonly gateClass = computed(() => {
    const g = this.socket.drive().gate;
    return 'chip ' + (g === 'teleop' || g === 'nav' ? 'ok'
      : g === 'no_heartbeat' || g === 'unknown' ? 'bad'
        : g === 'arm_moving' ? 'info' : 'warn');
  });

  private readonly postureTexts: Record<string, string> = {
    none: 'sin postura', moving: 'moviéndose…', calibrating: 'calibrando cámara…',
    high: 'parado (33 cm)', low: 'agachado (20 cm)', looking_up: 'mirando arriba…',
    checking: 'mirando arriba…', unknown: 'sin datos',
  };
  readonly postureText = computed(() => {
    const p = this.socket.drive().posture ?? 'unknown';
    return this.postureTexts[p] ?? p;
  });
  readonly postureClass = computed(() => {
    const p = this.socket.drive().posture ?? 'none';
    return 'chip ' + (p === 'high' ? 'ok' : p === 'low' ? 'crouch' : p === 'none' ? '' : 'info');
  });

  readonly verdict = computed(() => {
    const v = this.socket.drive().stand_check;
    if (!v) return null;
    const kind = v.split(':')[0];
    return {
      text: kind === 'clear' ? 'libre' : kind === 'blocked' ? 'techo cerca' : 'sin datos',
      cls: kind === 'clear' ? 'ok' : 'warn',
    };
  });

  private readonly navTexts: Record<string, string> = {
    idle: 'Sin objetivo', pending: 'Enviando…', active: 'Navegando',
    succeeded: 'Llegó', aborted: 'No pudo llegar', canceled: 'Cancelada',
    rejected: 'Objetivo rechazado', unavailable: 'Nav2 no está corriendo',
  };
  readonly navText = computed(() => this.navTexts[this.socket.nav().state] ?? this.socket.nav().state);
  readonly navActive = computed(() => ['pending', 'active'].includes(this.socket.nav().state));

  // Progress: the distance left against the largest distance seen for this goal.
  private maxDistance = 0;
  private lastGoal = '';
  readonly progress = computed(() => {
    const n = this.socket.nav();
    const key = JSON.stringify(n.goal);
    if (key !== this.lastGoal) {
      this.lastGoal = key;
      this.maxDistance = 0;
    }
    const d = n.distance ?? 0;
    this.maxDistance = Math.max(this.maxDistance, d);
    return this.maxDistance > 0 ? Math.round((1 - d / this.maxDistance) * 100) : 0;
  });

  setSpeed(e: Event): void {
    this.teleop.speed.set(Number((e.target as HTMLInputElement).value));
  }
}
