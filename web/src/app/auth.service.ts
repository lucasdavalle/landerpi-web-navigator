import { Injectable, signal } from '@angular/core';

const KEY = 'a7_token';

/**
 * Login against POST /api/login. The token lives in sessionStorage: it
 * survives a reload but not closing the tab, and it is never in a URL.
 */
@Injectable({ providedIn: 'root' })
export class AuthService {
  readonly token = signal<string | null>(sessionStorage.getItem(KEY));

  async login(password: string): Promise<string | null> {
    const res = await fetch('/api/login', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ password }),
    });
    const body = await res.json().catch(() => ({}));
    if (!res.ok) {
      return body.error ?? `HTTP ${res.status}`;
    }
    sessionStorage.setItem(KEY, body.token);
    this.token.set(body.token);
    return null;
  }

  async logout(): Promise<void> {
    const token = this.token();
    if (token) {
      await fetch('/api/logout', {
        method: 'POST',
        headers: { Authorization: `Bearer ${token}` },
      }).catch(() => undefined);
    }
    this.forget();
  }

  /** Drop the token locally (expired, or rejected by the server). */
  forget(): void {
    sessionStorage.removeItem(KEY);
    this.token.set(null);
  }
}
