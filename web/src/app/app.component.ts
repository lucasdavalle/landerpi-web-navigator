import { Component, inject } from '@angular/core';

import { AuthService } from './auth.service';
import { DrivePanelComponent } from './drive-panel.component';
import { LoginComponent } from './login.component';
import { MapViewComponent } from './map-view.component';
import { StatusBarComponent } from './status-bar.component';

@Component({
  selector: 'a7-root',
  standalone: true,
  imports: [LoginComponent, StatusBarComponent, MapViewComponent, DrivePanelComponent],
  template: `
    @if (auth.token()) {
      <a7-status-bar />
      <main>
        <a7-map-view />
        <a7-drive-panel />
      </main>
    } @else {
      <a7-login />
    }
  `,
  styles: [`
    :host { display: flex; flex-direction: column; height: 100vh; }
    main { flex: 1; display: flex; min-height: 0; }
    a7-map-view { flex: 1; }
  `],
})
export class AppComponent {
  readonly auth = inject(AuthService);
}
