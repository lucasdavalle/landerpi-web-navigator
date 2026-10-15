import { Component, computed, inject } from '@angular/core';

import { RobotSocket } from './robot-socket.service';
import { TeleopService } from './teleop.service';

@Component({
  selector: 'a7-drive-panel',
  standalone: true,
  template: `
    <h2>Manejo</h2>
    @if (teleop.isDriver()) {
      <button class="stop" (click)="teleop.disable()">Deshabilitar movimiento</button>
    } @else {
      <button class="go" (click)="teleop.enable()"
              [disabled]="teleop.someoneElseDrives() || socket.link() !== 'connected'">
        Habilitar movimiento
      </button>
    }
    @if (teleop.someoneElseDrives()) {
      <p class="note">Otro navegador está manejando.</p>
    }

    <p class="gate">Estado: <b [class]="gateClass()">{{ gateText() }}</b></p>
    <p class="gate">Brazo: <b>{{ postureText() }}</b></p>

    <h2>Navegación</h2>
    <p class="gate">{{ navText() }}
      @if (navActive() && socket.nav().distance != null) {
        <br><small>faltan {{ socket.nav().distance!.toFixed(2) }} m
        @if (socket.nav().eta) { (~{{ socket.nav().eta!.toFixed(0) }} s) }</small>
      }
    </p>
    @if (navActive()) {
      <button class="stop" (click)="socket.send({ type: 'cancel' })">Cancelar navegación</button>
    } @else {
      <p class="help">Click en el mapa → "Ir acá". Cualquier tecla cancela la navegación.</p>
    }

    <label>
      Velocidad: {{ (teleop.speed() * 100).toFixed(0) }} %
      <input type="range" min="0.2" max="1" step="0.1"
             [value]="teleop.speed()" (input)="setSpeed($event)">
    </label>

    <div class="keys" [class.active]="teleop.isDriver()">
      <span></span><kbd>W</kbd><span></span>
      <kbd>A</kbd><kbd>S</kbd><kbd>D</kbd>
      <kbd>Q</kbd><span></span><kbd>E</kbd>
    </div>
    <p class="help">W/S adelante/atrás · A/D de costado · Q/E girar.
      Soltar las teclas frena.</p>

    @if (teleop.isDriver()) {
      <p class="cmd">vx {{ teleop.command().vx.toFixed(2) }} m/s ·
        vy {{ teleop.command().vy.toFixed(2) }} m/s ·
        wz {{ teleop.command().wz.toFixed(2) }} rad/s</p>
    }
  `,
  styles: [`
    :host { display: flex; flex-direction: column; gap: 12px; padding: 14px; width: 260px;
            background: var(--panel); border-left: 1px solid var(--border); }
    h2 { font-size: 15px; margin: 0; }
    .go { border-color: var(--ok); }
    .stop { border-color: var(--danger); color: var(--danger); font-weight: 600; }
    .note, .help, .cmd { color: var(--muted); margin: 0; font-size: 12px; }
    .gate { margin: 0; }
    .ok { color: var(--ok); } .warn { color: var(--warn); } .bad { color: var(--danger); }
    label { display: flex; flex-direction: column; gap: 6px; }
    .keys { display: grid; grid-template-columns: repeat(3, 40px); gap: 4px;
            justify-content: center; opacity: 0.4; }
    .keys.active { opacity: 1; }
    kbd { display: grid; place-items: center; height: 36px; border: 1px solid var(--border);
          border-radius: 6px; background: var(--bg); font-family: inherit; }
  `],
})
export class DrivePanelComponent {
  readonly teleop = inject(TeleopService);
  readonly socket = inject(RobotSocket);

  private readonly texts: Record<string, string> = {
    idle: 'deshabilitado', no_heartbeat: 'sin señal (deadman)', no_command: 'listo, quieto',
    teleop: 'manejando', nav: 'navegando', unknown: 'velocity_gate sin datos',
  };
  readonly gateText = computed(() => this.texts[this.socket.drive().gate] ?? this.socket.drive().gate);
  readonly gateClass = computed(() => {
    const g = this.socket.drive().gate;
    return g === 'teleop' || g === 'nav' ? 'ok' : g === 'no_heartbeat' || g === 'unknown' ? 'bad' : 'warn';
  });

  private readonly postureTexts: Record<string, string> = {
    none: 'sin postura', moving: 'moviéndose…', calibrating: 'calibrando cámara…',
    high: 'alto (navegación)', low: 'agachado', unknown: 'sin datos',
  };
  readonly postureText = computed(() => {
    const p = this.socket.drive().posture ?? 'unknown';
    return this.postureTexts[p] ?? p;
  });

  private readonly navTexts: Record<string, string> = {
    idle: 'sin objetivo', pending: 'enviando objetivo…', active: 'navegando',
    succeeded: 'llegó', aborted: 'no pudo llegar', canceled: 'cancelada',
    rejected: 'objetivo rechazado', unavailable: 'Nav2 no está corriendo',
  };
  readonly navText = computed(() => this.navTexts[this.socket.nav().state] ?? this.socket.nav().state);
  readonly navActive = computed(() => ['pending', 'active'].includes(this.socket.nav().state));

  setSpeed(e: Event): void {
    this.teleop.speed.set(Number((e.target as HTMLInputElement).value));
  }
}
