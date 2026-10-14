import {
  AfterViewInit, Component, ElementRef, OnDestroy, ViewChild, effect, inject, signal,
} from '@angular/core';

import { GridMap } from './protocol';
import { RobotSocket } from './robot-socket.service';

/**
 * The live map on a canvas: occupancy grid, walls, laser points, robot and
 * its uncertainty ellipse. Wheel = zoom, drag = pan, "Seguir" keeps the
 * robot centred. A click shows the clicked map coordinates; sending the
 * robot there is stage 5 (Nav2), prepared but disabled.
 */
@Component({
  selector: 'a7-map-view',
  standalone: true,
  template: `
    <canvas #canvas
            (wheel)="onWheel($event)"
            (pointerdown)="onDown($event)" (pointermove)="onMove($event)"
            (pointerup)="onUp($event)" (pointerleave)="dragging = false"></canvas>
    <div class="overlay">
      <button (click)="follow.set(!follow())" [class.on]="follow()">Seguir robot</button>
      <button (click)="fit()">Ver todo</button>
    </div>
    @if (clicked(); as c) {
      <div class="goal">
        Punto ({{ c.x.toFixed(2) }}, {{ c.y.toFixed(2) }}) m
        <button disabled title="Navegación autónoma: etapa 5">Ir acá</button>
        <button (click)="clicked.set(null)">×</button>
      </div>
    }
  `,
  styles: [`
    :host { position: relative; display: block; width: 100%; height: 100%; }
    canvas { width: 100%; height: 100%; display: block; cursor: grab; touch-action: none; }
    .overlay { position: absolute; top: 10px; right: 10px; display: flex; gap: 6px; }
    .overlay .on { border-color: var(--accent); color: var(--accent); }
    .goal { position: absolute; bottom: 10px; left: 50%; transform: translateX(-50%);
            display: flex; gap: 8px; align-items: center; padding: 6px 10px;
            background: var(--panel); border: 1px solid var(--border); border-radius: 8px; }
  `],
})
export class MapViewComponent implements AfterViewInit, OnDestroy {
  @ViewChild('canvas', { static: true }) canvasRef!: ElementRef<HTMLCanvasElement>;

  private readonly socket = inject(RobotSocket);
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
  private frame = 0;
  private resizeObs?: ResizeObserver;

  constructor() {
    effect(() => {
      const map = this.socket.map();
      if (map) this.rasterise(map);
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
        let g: number;
        if (v < 0) g = 36;                   // unknown: background-ish
        else if (v >= 65) g = 10;            // wall
        else g = 70 - Math.round(v * 0.3);   // free (darker = less certain)
        img.data[i] = g;
        img.data[i + 1] = g + 4;
        img.data[i + 2] = g + 10;
        img.data[i + 3] = 255;
      }
    }
    ctx.putImageData(img, 0, 0);
    const first = !this.mapInfo;
    this.mapCanvas = off;
    this.mapInfo = m;
    if (first && !this.socket.pose()) this.fit();
  }

  private draw(): void {
    const canvas = this.canvasRef.nativeElement;
    const ctx = canvas.getContext('2d')!;
    ctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
    ctx.fillStyle = '#15181d';
    ctx.fillRect(0, 0, canvas.clientWidth, canvas.clientHeight);

    const pose = this.socket.pose();
    if (pose && this.follow()) {
      this.cx = pose.x;
      this.cy = pose.y;
    }

    // Occupancy grid, scaled with nearest neighbour (crisp cells).
    const m = this.mapInfo;
    if (m && this.mapCanvas) {
      const [x0, y0] = this.toScreen(m.originX, m.originY + m.height * m.resolution);
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(this.mapCanvas, x0, y0,
        m.width * m.resolution * this.scale, m.height * m.resolution * this.scale);
    }

    // Walls (stage 2): the floor plan.
    ctx.strokeStyle = '#ff5a4a';
    ctx.lineWidth = 3;
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
      ctx.fillStyle = '#5cff9d';
      for (let i = 0; i < scan.points.length; i += 2) {
        const [px, py] = this.toScreen(scan.points[i], scan.points[i + 1]);
        ctx.fillRect(px - 1.5, py - 1.5, 3, 3);
      }
    }

    if (pose) this.drawRobot(ctx, pose.x, pose.y, pose.yaw, pose.cov);
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

    // The robot: a 22 x 18 cm body with an arrow for its heading.
    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(-yaw);
    const s = this.scale;
    ctx.fillStyle = '#4fa3ff';
    ctx.strokeStyle = '#e3e7ec';
    ctx.lineWidth = 1.5;
    ctx.fillRect(-0.11 * s, -0.09 * s, 0.22 * s, 0.18 * s);
    ctx.strokeRect(-0.11 * s, -0.09 * s, 0.22 * s, 0.18 * s);
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
