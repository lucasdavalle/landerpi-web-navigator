import { Component, inject, signal } from '@angular/core';
import { FormsModule } from '@angular/forms';

import { AuthService } from './auth.service';

@Component({
  selector: 'a7-login',
  standalone: true,
  imports: [FormsModule],
  template: `
    <form class="card" (ngSubmit)="submit()">
      <h1>LanderPi Navigator</h1>
      <input type="password" name="password" placeholder="Contraseña"
             [(ngModel)]="password" autocomplete="current-password" autofocus>
      <button type="submit" [disabled]="busy() || !password">Entrar</button>
      @if (error()) {
        <p class="error">{{ error() }}</p>
      }
    </form>
  `,
  styles: [`
    :host { display: grid; place-items: center; height: 100vh; }
    .card { display: flex; flex-direction: column; gap: 12px; width: 280px;
            padding: 24px; background: var(--panel); border: 1px solid var(--border);
            border-radius: 10px; }
    h1 { font-size: 18px; margin: 0 0 4px; }
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
