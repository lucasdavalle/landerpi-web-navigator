import { Component, inject } from '@angular/core';

import { AuthService } from './auth.service';
import { DrivePanelComponent } from './drive-panel.component';
import { LoginComponent } from './login.component';
import { MapViewComponent } from './map-view.component';
import { RobotOverlayComponent } from './robot-overlay.component';
import { StatusBarComponent } from './status-bar.component';

@Component({
  selector: 'a7-root',
  standalone: true,
  imports: [
    LoginComponent, StatusBarComponent, MapViewComponent, DrivePanelComponent,
    RobotOverlayComponent,
  ],
  template: `
    @if (auth.token()) {
      <a7-status-bar />
      <main>
        <section class="map">
          <a7-map-view />
          <a7-robot-overlay />
        </section>
        <a7-drive-panel />
      </main>
    } @else {
      <a7-login />
    }
  `,
  styles: [`
    :host { display: flex; flex-direction: column; height: 100vh; }
    main { flex: 1; display: flex; gap: 12px; min-height: 0; padding: 0 12px 12px; }
    .map { position: relative; flex: 1; min-width: 0; border-radius: var(--radius);
           overflow: hidden; border: 1px solid var(--border); background: var(--bg-2); }
  `],
})
export class AppComponent {
  readonly auth = inject(AuthService);
}
