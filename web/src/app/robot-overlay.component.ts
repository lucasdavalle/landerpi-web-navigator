import {
  Component, ElementRef, OnDestroy, ViewChild, computed, effect, inject, signal,
} from '@angular/core';
import type * as THREE_NS from 'three';

import { ArmAngles, POSES, RobotModel, buildRobot, lerpAngles } from './robot-model';
import { RobotSocket } from './robot-socket.service';

const BUSY = ['moving', 'calibrating', 'looking_up', 'checking'];
const MOVE_SECONDS = 3.0;     // posture_manager's move_duration
const LINGER_MS = 2500;       // keep the card up after the arm settles

/**
 * A floating card with the robot in 3D while something worth watching
 * happens: the arm changing posture (animated from where it was to where it
 * is going, the same 3 s the servos take) or an exploration (the robot
 * turning with its lidar sweeping). Three.js is loaded only the first time
 * the card shows, so the page itself stays light.
 */
@Component({
  selector: 'a7-robot-overlay',
  standalone: true,
  template: `
    @if (visible()) {
      <div class="card" [class.explore]="exploring()">
        <canvas #canvas width="300" height="210"></canvas>
        <div class="caption">
          <span class="dot"></span>
          <div>
            <b>{{ title() }}</b>
            @if (subtitle()) { <small>{{ subtitle() }}</small> }
          </div>
        </div>
      </div>
    }
  `,
  styles: [`
    :host { position: absolute; left: 14px; bottom: 14px; pointer-events: none; z-index: 5; }
    .card { width: 300px; border-radius: 14px; overflow: hidden;
            background: linear-gradient(160deg, rgba(31, 41, 55, 0.92), rgba(17, 22, 30, 0.92));
            border: 1px solid var(--border); box-shadow: 0 12px 32px rgba(0, 0, 0, 0.45);
            backdrop-filter: blur(6px); animation: pop 0.25s ease-out; }
    canvas { display: block; width: 300px; height: 210px; }
    .caption { display: flex; gap: 10px; align-items: center; padding: 10px 14px 12px; }
    .caption b { display: block; font-size: 14px; }
    .caption small { display: block; color: var(--muted); margin-top: 2px; }
    .dot { width: 10px; height: 10px; border-radius: 50%; background: var(--accent);
           box-shadow: 0 0 0 0 rgba(79, 163, 255, 0.6); animation: pulse 1.4s infinite; flex: none; }
    .explore .dot { background: var(--ok); }
    @keyframes pulse { 70% { box-shadow: 0 0 0 10px rgba(79, 163, 255, 0); } }
    @keyframes pop { from { transform: translateY(12px); opacity: 0; } }
  `],
})
export class RobotOverlayComponent implements OnDestroy {
  @ViewChild('canvas') set canvasRef(ref: ElementRef<HTMLCanvasElement> | undefined) {
    this.canvas = ref?.nativeElement ?? null;
    if (this.canvas) void this.start(this.canvas);
  }

  private readonly socket = inject(RobotSocket);
  private canvas: HTMLCanvasElement | null = null;

  private readonly posture = computed(() => this.socket.drive().posture ?? 'none');
  private readonly target = computed(() => this.socket.drive().arm_target ?? '');
  readonly exploring = computed(() => {
    const m = this.socket.mission();
    return m?.kind === 'explore' && m.state === 'active';
  });
  private readonly lingering = signal(false);
  readonly visible = computed(() =>
    this.exploring() || BUSY.includes(this.posture()) || this.lingering());

  // The last look-up verdict and when it arrived, to explain what follows.
  private verdict = '';
  private verdictAt = 0;
  private readonly verdictText = signal('');

  readonly title = computed(() => {
    if (this.exploring()) return 'Explorando la pieza';
    switch (this.posture()) {
      case 'looking_up': return 'Miro hacia arriba';
      case 'checking': return '¿Hay lugar arriba?';
      case 'calibrating': return 'Calibro la cámara contra el piso';
      case 'moving': return this.target() === 'high' ? 'Me levanto' : 'Me agacho';
      case 'high': return 'Listo, parado';
      case 'low': return 'Listo, agachado';
      default: return '';
    }
  });
  readonly subtitle = computed(() => {
    if (this.exploring()) {
      const m = this.socket.mission();
      return m?.message ?? `${m?.index ?? 0} lugares visitados`;
    }
    return this.verdictText() || this.socket.drive().crouch_status || '';
  });

  // Animation state.
  private three: typeof THREE_NS | null = null;
  private renderer: THREE_NS.WebGLRenderer | null = null;
  private scene: THREE_NS.Scene | null = null;
  private camera: THREE_NS.PerspectiveCamera | null = null;
  private model: RobotModel | null = null;
  private frame = 0;
  private shown: ArmAngles = POSES['none'];
  private from: ArmAngles = POSES['none'];
  private to: ArmAngles = POSES['none'];
  private moveStart = 0;
  private lingerTimer: ReturnType<typeof setTimeout> | null = null;

  constructor() {
    // A new arm move: animate from what is shown now to its target.
    effect(() => {
      const p = this.posture();
      const target = p === 'looking_up' ? 'look_up' : this.target();
      if ((p === 'moving' || p === 'looking_up') && POSES[target]) {
        this.from = this.shown;
        this.to = POSES[target];
        this.moveStart = performance.now();
      }
      if (p === 'high' || p === 'low') {
        // Settled: show it, and keep the card a moment to say so.
        this.to = POSES[p];
        if (this.lingerTimer) clearTimeout(this.lingerTimer);
        this.lingering.set(true);
        this.lingerTimer = setTimeout(() => this.lingering.set(false), LINGER_MS);
      }
    }, { allowSignalWrites: true });

    // The look-up verdict explains the move that follows it.
    effect(() => {
      const v = this.socket.drive().stand_check ?? '';
      if (v && v !== this.verdict) {
        this.verdict = v;
        this.verdictAt = performance.now();
        const kind = v.split(':')[0];
        this.verdictText.set(
          kind === 'clear' ? 'Hay lugar: me levanto'
            : kind === 'blocked' ? 'Hay techo cerca: me quedo agachado'
              : 'No pude ver arriba: me quedo agachado');
      }
      if (this.posture() === 'high' || this.posture() === 'low' || this.posture() === 'none') {
        if (performance.now() - this.verdictAt > 8000) this.verdictText.set('');
      }
    }, { allowSignalWrites: true });
  }

  ngOnDestroy(): void {
    cancelAnimationFrame(this.frame);
    if (this.lingerTimer) clearTimeout(this.lingerTimer);
    this.renderer?.dispose();
  }

  private async start(canvas: HTMLCanvasElement): Promise<void> {
    if (!this.three) {
      this.three = await import('three');
    }
    const THREE = this.three;
    this.renderer?.dispose();
    const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
    renderer.setPixelRatio(devicePixelRatio);
    renderer.setSize(300, 210, false);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer = renderer;

    if (!this.scene) {
      const scene = new THREE.Scene();
      scene.add(new THREE.HemisphereLight(0xcfe3ff, 0x1a1f26, 1.1));
      const sun = new THREE.DirectionalLight(0xffffff, 2.2);
      sun.position.set(0.6, 1.2, 0.4);
      sun.castShadow = true;
      sun.shadow.mapSize.set(1024, 1024);
      sun.shadow.camera.near = 0.1;
      sun.shadow.camera.far = 3;
      for (const k of ['left', 'bottom'] as const) sun.shadow.camera[k] = -0.6;
      for (const k of ['right', 'top'] as const) sun.shadow.camera[k] = 0.6;
      scene.add(sun);
      const rim = new THREE.DirectionalLight(0x4fa3ff, 1.2);
      rim.position.set(-0.8, 0.5, -0.6);
      scene.add(rim);

      const floor = new THREE.Mesh(
        new THREE.CircleGeometry(0.7, 64),
        new THREE.MeshStandardMaterial({ color: 0x1c232c, roughness: 0.95 }),
      );
      floor.rotation.x = -Math.PI / 2;
      floor.receiveShadow = true;
      scene.add(floor);
      const grid = new THREE.PolarGridHelper(0.7, 8, 6, 64, 0x2f3a48, 0x2f3a48);
      grid.position.y = 0.001;
      scene.add(grid);

      this.model = buildRobot(THREE);
      scene.add(this.model.root);
      this.scene = scene;
      this.camera = new THREE.PerspectiveCamera(32, 300 / 210, 0.05, 10);
    }
    cancelAnimationFrame(this.frame);
    const loop = () => {
      this.render();
      if (this.visible()) this.frame = requestAnimationFrame(loop);
    };
    this.frame = requestAnimationFrame(loop);
  }

  private render(): void {
    if (!this.renderer || !this.scene || !this.camera || !this.model) return;
    const now = performance.now();
    const t = now / 1000;
    const exploring = this.exploring();

    const k = Math.min(1, (now - this.moveStart) / (MOVE_SECONDS * 1000));
    this.shown = lerpAngles(this.from, this.to, k);
    this.model.setArm(this.shown);
    this.model.animate(t);
    this.model.showSweep(exploring);

    if (exploring) {
      // Mapping: the robot turns slowly, seen from above at an angle.
      this.model.root.rotation.y = t * 0.6;
      this.camera.position.set(0.9, 0.75, 0.9);
      this.camera.lookAt(0, 0.08, 0);
    } else {
      // Arm moves: a three-quarter view from the robot's left, so the arm's
      // profile reads clearly, swaying a little to show depth.
      this.model.root.rotation.y = 0;
      const a = -1.2 + Math.sin(t * 0.5) * 0.15;
      this.camera.position.set(Math.cos(a) * 0.75 + 0.05, 0.42, Math.sin(a) * 0.75);
      this.camera.lookAt(0.06, 0.15, 0);
    }
    this.renderer.render(this.scene, this.camera);
  }
}
