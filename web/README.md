# Web UI (Angular)

Browser interface of the A7 web navigator: login, live map (occupancy grid,
walls, laser points, robot pose with its uncertainty ellipse, and what the
depth camera found: where the robot must crouch and obstacles too low for
the lidar), keyboard driving and "go here" through the robot's
`web_gateway` / `velocity_gate`.

A floating card shows the robot in 3D (Three.js, a model built from simple
shapes in `robot-model.ts`) when its arm changes posture, animated over the
same 3 s the servos take, and while it explores. Three.js is loaded on
demand (a separate chunk), so the page itself stays light.

Built on a PC (Node 20 LTS), not on the robot. Only `dist/` is copied to the
robot, where `web_gateway` serves it on port 8000.

## Develop against the live robot

```
npm install
npm start          # http://localhost:4200, /api and /ws proxied to the robot
```

`proxy.conf.json` points at `192.168.100.181:8000`; change it if the robot's
address differs. The PC must be on the home network (the gateway only
accepts the networks in its `allowed_networks`).

## Build and deploy

```
npm run build      # -> dist/landerpi-ui/browser/
```

Copy `dist/landerpi-ui/browser/` to the robot and point the gateway's
`static_root` parameter at it (see the repository README).
