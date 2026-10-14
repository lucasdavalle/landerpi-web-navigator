# Web UI (Angular)

Browser interface of the A7 web navigator: login, live map (occupancy grid,
walls, laser points, robot pose with its uncertainty ellipse) and keyboard
driving through the robot's `web_gateway` / `velocity_gate`.

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
