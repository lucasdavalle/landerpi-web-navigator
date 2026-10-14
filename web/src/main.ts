import { bootstrapApplication } from '@angular/platform-browser';
import { provideExperimentalZonelessChangeDetection } from '@angular/core';

import { AppComponent } from './app/app.component';

// Zoneless: the UI is driven by signals updated from the WebSocket, so
// Angular does not need zone.js to detect changes.
bootstrapApplication(AppComponent, {
  providers: [provideExperimentalZonelessChangeDetection()],
}).catch((err) => console.error(err));
