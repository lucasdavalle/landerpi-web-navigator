import {
  AfterViewInit, Component, ElementRef, OnDestroy, ViewChild, effect, inject, signal,
} from '@angular/core';

import { CAMERA_OBSTACLE, CROUCHABLE, GridMap } from './protocol';
import { RobotSocket } from './robot-socket.service';
import { TeleopService } from './teleop.service';

/**
 * The live map on a canvas: occupancy grid, walls, laser points, robot and
 * its uncertainty ellipse. Wheel = zoom, drag = pan, "Seguir" keeps the
 * robot centred. A click picks a map point; the driver can send the robot
 * there ("Ir acá", Nav2). The goal and the planned path are drawn, and what
 * the depth camera found on top (stage 6c): where the robot has to crouch
 * and obstacles too low for the lidar.
 */
@Component({
  selector: 'a7-map-view',
  standalone: true,
  template: `
    <canvas #canvas
            (wheel)="onWheel($event)"
            (pointerdown)="onDown($event)" (pointermove)="onMove($event)"
            (pointerup)="onUp($event)" (pointerleave)="dragging = false"></canvas>
    <div class="tools">
      <button (click)="follow.set(!follow())" [class.on]="follow()">Seguir robot</button>
      <button (click)="fit()">Ver todo</button>
    </div>
    <div class="legend">
      <span><i class="wall"></i>paredes</span>
      <span><i class="lidar"></i>lidar</span>
      <span><i class="crouch"></i>agacharse</span>
      <span><i class="low"></i>obstáculo bajo</span>
    </div>
    @if (clicked(); as c) {
      <div class="goal">
        <span class="muted">({{ c.x.toFixed(2) }}, {{ c.y.toFixed(2) }}) m</span>
        <button class="primary" (click)="goHere(c.x, c.y)" [disabled]="!teleop.isDriver()"
                [title]="teleop.isDriver() ? 'Navegar hasta este punto' : 'Primero habilitá el movimiento'">
          Ir acá
        </button>
        <button class="ghost" (click)="clicked.set(null)">×</button>
      </div>
    }
  `,
  styles: [`
    :host { position: relative; display: block; width: 100%; height: 100%; }
    canvas { width: 100%; height: 100%; display: block; cursor: grab; touch-action: none; }
    .tools { position: absolute; top: 12px; right: 12px; display: flex; gap: 6px; }
    .tools button { background: rgba(23, 29, 38, 0.85); backdrop-filter: blur(4px); }
    .tools .on { border-color: var(--accent); color: var(--accent); }
    .legend { position: absolute; bottom: 12px; right: 12px; display: flex; gap: 12px;
              padding: 6px 12px; border-radius: 999px; font-size: 12px; color: var(--muted);
              background: rgba(23, 29, 38, 0.85); border: 1px solid var(--border); }
    .legend span { display: flex; align-items: center; gap: 6px; }
    .legend i { width: 12px; height: 12px; border-radius: 3px; display: inline-block; }
    .legend .wall { background: #e6ebf2; height: 3px; }
    .legend .lidar { background: #5cff9d; width: 6px; height: 6px; border-radius: 50%; }
    .legend .crouch { background: rgba(176, 124, 255, 0.8); }
    .legend .low { background: rgba(255, 146, 72, 0.9); }
    .goal { position: absolute; top: 12px; left: 50%; transform: translateX(-50%);
            display: flex; gap: 10px; align-items: center; padding: 6px 6px 6px 14px;
            background: rgba(23, 29, 38, 0.92); border: 1px solid var(--border);
            border-radius: 999px; box-shadow: 0 8px 24px rgba(0, 0, 0, 0.4); }
  `],
})
export class MapViewComponent implements AfterViewInit, OnDestroy {
  @ViewChild('canvas', { static: true }) canvasRef!: ElementRef<HTMLCanvasElement>;

  private readonly socket = inject(RobotSocket);
  readonly teleop = inject(TeleopService);
  readonly follow = signal(true);
  readonly clicked = signal<{ x: number; y: number } | null>(null);

  // View: world point at the canvas centre and pixels per metre.
  private cx = 0;
  private cy = 0;
  private scale = 80;
  dragging = false;
  private dragMoved = false;
  private lastX = 0;
  private lastY = 0;

  // The grid is rasterised once per map update into an offscreen canvas.
  private mapCanvas: HTMLCanvasElement | null = null;
  private mapInfo: GridMap | null = null;
  private depthCanvas: HTMLCanvasElement | null = null;
  private depthInfo: GridMap | null = null;
  private frame = 0;
  private resizeObs?: ResizeObserver;

  constructor() {
    effect(() => {
      const map = this.socket.map();
      if (map) this.rasterise(map);
    });
    effect(() => {
      const grid = this.socket.depthGrid();
      if (grid) this.rasteriseDepth(grid);
    });
  }

  ngAfterViewInit(): void {
    const canvas = this.canvasRef.nativeElement;
    this.resizeObs = new ResizeObserver(() => {
      canvas.width = canvas.clientWidth * devicePixelRatio;
      canvas.height = canvas.clientHeight * devicePixelRatio;
    });
    this.resizeObs.observe(canvas);
    const loop = () => {
      this.draw();
      this.frame = requestAnimationFrame(loop);
    };
    this.frame = requestAnimationFrame(loop);
  }

  ngOnDestroy(): void {
    cancelAnimationFrame(this.frame);
    this.resizeObs?.disconnect();
  }

  goHere(x: number, y: number): void {
    this.socket.send({ type: 'goal', x, y });
    this.clicked.set(null);
  }

  fit(): void {
    const m = this.mapInfo;
    const canvas = this.canvasRef.nativeElement;
    if (!m) return;
    this.follow.set(false);
    this.cx = m.originX + (m.width * m.resolution) / 2;
    this.cy = m.originY + (m.height * m.resolution) / 2;
    const sx = canvas.clientWidth / (m.width * m.resolution);
    const sy = canvas.clientHeight / (m.height * m.resolution);
    this.scale = Math.max(10, Math.min(sx, sy) * 0.9);
  }

  // --- input -----------------------------------------------------------
  onWheel(e: WheelEvent): void {
    e.preventDefault();
    this.scale = Math.min(400, Math.max(10, this.scale * (e.deltaY < 0 ? 1.15 : 1 / 1.15)));
  }

  onDown(e: PointerEvent): void {
    this.dragging = true;
    this.dragMoved = false;
    this.lastX = e.clientX;
    this.lastY = e.clientY;
    (e.target as HTMLElement).setPointerCapture(e.pointerId);
  }

  onMove(e: PointerEvent): void {
    if (!this.dragging) return;
    const dx = e.clientX - this.lastX;
    const dy = e.clientY - this.lastY;
    if (Math.abs(dx) + Math.abs(dy) > 2) {
      this.dragMoved = true;
      this.follow.set(false);
    }
    this.cx -= dx / this.scale;
    this.cy += dy / this.scale;  // screen y grows down, map y grows up
    this.lastX = e.clientX;
    this.lastY = e.clientY;
  }

  onUp(e: PointerEvent): void {
    this.dragging = false;
    if (!this.dragMoved) {
      const r = this.canvasRef.nativeElement.getBoundingClientRect();
      this.clicked.set(this.toWorld(e.clientX - r.left, e.clientY - r.top));
    }
  }

  // --- drawing ---------------------------------------------------------
  private toScreen(x: number, y: number): [number, number] {
    const c = this.canvasRef.nativeElement;
    return [c.clientWidth / 2 + (x - this.cx) * this.scale,
      c.clientHeight / 2 - (y - this.cy) * this.scale];
  }

  private toWorld(px: number, py: number): { x: number; y: number } {
    const c = this.canvasRef.nativeElement;
    return { x: this.cx + (px - c.clientWidth / 2) / this.scale,
      y: this.cy - (py - c.clientHeight / 2) / this.scale };
  }

  private rasterise(m: GridMap): void {
    const off = document.createElement('canvas');
    off.width = m.width;
    off.height = m.height;
    const ctx = off.getContext('2d')!;
    const img = ctx.createImageData(m.width, m.height);
    for (let r = 0; r < m.height; r++) {
      for (let c = 0; c < m.width; c++) {
        const v = m.cells[r * m.width + c];
        // Grid row 0 is the lowest y; image row 0 is the top: flip.
        const i = ((m.height - 1 - r) * m.width + c) * 4;
        if (v < 0) {                         // unknown: transparent, the page shows
          img.data[i + 3] = 0;
          continue;
        }
        if (v >= 65) {                       // occupied
          img.data[i] = 120; img.data[i + 1] = 134; img.data[i + 2] = 156;
        } else {                             // free (darker = less certain)
          const g = 44 - Math.round(v * 0.2);
          img.data[i] = g; img.data[i + 1] = g + 8; img.data[i + 2] = g + 20;
        }
        img.data[i + 3] = 255;
      }
    }
    ctx.putImageData(img, 0, 0);
    const first = !this.mapInfo;
    this.mapCanvas = off;
    this.mapInfo = m;
    if (first && !this.socket.pose()) this.fit();
  }

  /** The camera's grid: crouchable in violet, low obstacles in orange. */
  private rasteriseDepth(m: GridMap): void {
    const off = document.createElement('canvas');
    off.width = m.width;
    off.height = m.height;
    const ctx = off.getContext('2d')!;
    const img = ctx.createImageData(m.width, m.height);
    for (let r = 0; r < m.height; r++) {
      for (let c = 0; c < m.width; c++) {
        const v = m.cells[r * m.width + c];
        const i = ((m.height - 1 - r) * m.width + c) * 4;
        if (v === CROUCHABLE) {
          img.data[i] = 176; img.data[i + 1] = 124; img.data[i + 2] = 255; img.data[i + 3] = 150;
        } else if (v === CAMERA_OBSTACLE) {
          img.data[i] = 255; img.data[i + 1] = 146; img.data[i + 2] = 72; img.data[i + 3] = 210;
        }
      }
    }
    ctx.putImageData(img, 0, 0);
    this.depthCanvas = off;
    this.depthInfo = m;
  }

  private drawGrid(ctx: CanvasRenderingContext2D, m: GridMap, img: HTMLCanvasElement): void {
    const [x0, y0] = this.toScreen(m.originX, m.originY + m.height * m.resolution);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(img, x0, y0,
      m.width * m.resolution * this.scale, m.height * m.resolution * this.scale);
  }

  private draw(): void {
    const canvas = this.canvasRef.nativeElement;
    const ctx = canvas.getContext('2d')!;
    ctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
    ctx.fillStyle = '#131820';
    ctx.fillRect(0, 0, canvas.clientWidth, canvas.clientHeight);
    this.drawMetreGrid(ctx, canvas.clientWidth, canvas.clientHeight);

    const pose = this.socket.pose();
    if (pose && this.follow()) {
      this.cx = pose.x;
      this.cy = pose.y;
    }

    // Occupancy grid, scaled with nearest neighbour (crisp cells).
    if (this.mapInfo && this.mapCanvas) this.drawGrid(ctx, this.mapInfo, this.mapCanvas);
    // What the depth camera found (stage 6c), over the floor.
    if (this.depthInfo && this.depthCanvas) this.drawGrid(ctx, this.depthInfo, this.depthCanvas);

    // Walls (stage 2): the floor plan.
    ctx.strokeStyle = '#e6ebf2';
    ctx.lineWidth = 2.5;
    ctx.lineCap = 'round';
    ctx.beginPath();
    for (const [x1, y1, x2, y2] of this.socket.walls()) {
      const [a, b] = this.toScreen(x1, y1);
      const [c, d] = this.toScreen(x2, y2);
      ctx.moveTo(a, b);
      ctx.lineTo(c, d);
    }
    ctx.stroke();

    // Laser points.
    const scan = this.socket.scan();
    if (scan) {
      ctx.fillStyle = 'rgba(92, 255, 157, 0.9)';
      for (let i = 0; i < scan.points.length; i += 2) {
        const [px, py] = this.toScreen(scan.points[i], scan.points[i + 1]);
        ctx.beginPath();
        ctx.arc(px, py, 1.8, 0, 2 * Math.PI);
        ctx.fill();
      }
    }

    // Planned path and goal (stage 5).
    const path = this.socket.path();
    if (path.length > 1) {
      ctx.strokeStyle = '#4fa3ff';
      ctx.lineWidth = 3;
      ctx.setLineDash([2, 6]);
      ctx.beginPath();
      path.forEach(([x, y], i) => {
        const [sx, sy] = this.toScreen(x, y);
        if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
      });
      ctx.stroke();
      ctx.setLineDash([]);
    }
    const nav = this.socket.nav();
    if (nav.goal && (nav.state === 'pending' || nav.state === 'active')) {
      const [gx, gy] = this.toScreen(nav.goal[0], nav.goal[1]);
      const pulse = 10 + 4 * Math.sin(performance.now() / 250);
      ctx.fillStyle = 'rgba(79, 163, 255, 0.18)';
      ctx.beginPath();
      ctx.arc(gx, gy, pulse, 0, 2 * Math.PI);
      ctx.fill();
      ctx.fillStyle = '#4fa3ff';
      ctx.strokeStyle = '#e6ebf2';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(gx, gy, 6, 0, 2 * Math.PI);
      ctx.fill();
      ctx.stroke();
    }

    if (pose) this.drawRobot(ctx, pose.x, pose.y, pose.yaw, pose.cov);
  }

  /** Faint lines every metre, so distances read at a glance. */
  private drawMetreGrid(ctx: CanvasRenderingContext2D, w: number, h: number): void {
    const tl = this.toWorld(0, 0);
    const br = this.toWorld(w, h);
    ctx.strokeStyle = 'rgba(138, 150, 168, 0.08)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = Math.ceil(tl.x); x <= br.x; x++) {
      const [sx] = this.toScreen(x, 0);
      ctx.moveTo(sx, 0); ctx.lineTo(sx, h);
    }
    for (let y = Math.ceil(br.y); y <= tl.y; y++) {
      const [, sy] = this.toScreen(0, y);
      ctx.moveTo(0, sy); ctx.lineTo(w, sy);
    }
    ctx.stroke();
  }

  private drawRobot(
    ctx: CanvasRenderingContext2D, x: number, y: number, yaw: number,
    cov?: [number, number, number],
  ): void {
    const [px, py] = this.toScreen(x, y);

    // 2-sigma uncertainty ellipse from the position covariance [xx, xy, yy]:
    // its axes are the eigenvectors, the radii 2*sqrt(eigenvalues).
    if (cov) {
      const [a, b, c] = cov;
      const tr = (a + c) / 2;
      const det = Math.sqrt(Math.max(0, ((a - c) / 2) ** 2 + b * b));
      const l1 = tr + det;
      const l2 = Math.max(0, tr - det);
      const angle = 0.5 * Math.atan2(2 * b, a - c);
      const r1 = Math.max(2, 2 * Math.sqrt(l1) * this.scale);
      const r2 = Math.max(2, 2 * Math.sqrt(l2) * this.scale);
      ctx.save();
      ctx.translate(px, py);
      ctx.rotate(-angle);
      ctx.beginPath();
      ctx.ellipse(0, 0, r1, r2, 0, 0, 2 * Math.PI);
      ctx.fillStyle = 'rgba(79, 163, 255, 0.18)';
      ctx.strokeStyle = 'rgba(79, 163, 255, 0.8)';
      ctx.lineWidth = 1;
      ctx.fill();
      ctx.stroke();
      ctx.restore();
    }

    // The robot: a 22 x 18 cm body with an arrow for its heading; crouched,
    // the arm sticks out 23 cm in front (drawn, it is part of its footprint).
    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(-yaw);
    const s = this.scale;
    if (this.socket.drive().posture === 'low') {
      ctx.fillStyle = 'rgba(176, 124, 255, 0.85)';
      ctx.fillRect(0.06 * s, -0.025 * s, 0.28 * s, 0.05 * s);
    }
    ctx.fillStyle = '#4fa3ff';
    ctx.strokeStyle = '#e6ebf2';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.roundRect(-0.11 * s, -0.09 * s, 0.22 * s, 0.18 * s, 0.03 * s);
    ctx.fill();
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(0.16 * s, 0);
    ctx.lineTo(0.04 * s, -0.06 * s);
    ctx.lineTo(0.04 * s, 0.06 * s);
    ctx.closePath();
    ctx.fillStyle = '#e3e7ec';
    ctx.fill();
    ctx.restore();
  }
}
