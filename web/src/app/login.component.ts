import { Component, inject, signal } from '@angular/core';
import { FormsModule } from '@angular/forms';

import { AuthService } from './auth.service';

@Component({
  selector: 'a7-login',
  standalone: true,
  imports: [FormsModule],
  template: `
    <form class="card" (ngSubmit)="submit()">
      <div class="logo">
        <svg viewBox="0 0 32 32" width="44" height="44" aria-hidden="true">
          <rect x="4" y="13" width="24" height="11" rx="3" fill="#4fa3ff"/>
          <circle cx="10" cy="26" r="3" fill="#e6ebf2"/><circle cx="22" cy="26" r="3" fill="#e6ebf2"/>
          <path d="M18 13 L18 7 L24 4" stroke="#e6ebf2" stroke-width="2.4" fill="none" stroke-linecap="round"/>
          <circle cx="11" cy="10" r="2.6" fill="#3ecf8e"/>
        </svg>
      </div>
      <h1>LanderPi Navigator</h1>
      <p class="sub">Mapa en vivo, navegación y control del robot</p>
      <input type="password" name="password" placeholder="Contraseña"
             [(ngModel)]="password" autocomplete="current-password" autofocus>
      <button class="primary" type="submit" [disabled]="busy() || !password">Entrar</button>
      @if (error()) {
        <p class="error">{{ error() }}</p>
      }
    </form>
  `,
  styles: [`
    :host { display: grid; place-items: center; height: 100vh; }
    .card { display: flex; flex-direction: column; gap: 12px; width: 320px; padding: 28px;
            box-shadow: 0 20px 50px rgba(0, 0, 0, 0.45); }
    .logo { display: grid; place-items: center; }
    h1 { font-size: 20px; margin: 0; text-align: center; }
    .sub { margin: -4px 0 6px; color: var(--muted); text-align: center; font-size: 13px; }
    .error { color: var(--danger); margin: 0; }
  `],
})
export class LoginComponent {
  private readonly auth = inject(AuthService);
  password = '';
  readonly busy = signal(false);
  readonly error = signal<string | null>(null);

  async submit(): Promise<void> {
    this.busy.set(true);
    this.error.set(null);
    try {
      const err = await this.auth.login(this.password);
      if (err) this.error.set(err === 'wrong password' ? 'Contraseña incorrecta' : err);
    } catch {
      this.error.set('No se pudo contactar al robot');
    } finally {
      this.password = '';
      this.busy.set(false);
    }
  }
}
