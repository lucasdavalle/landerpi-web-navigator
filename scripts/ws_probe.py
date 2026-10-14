#!/usr/bin/env python3
# Copyright 2026 ldavalle
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Command-line WebSocket client for the A7 gateway (stage 3 test tool).

Logs in (or takes a token), opens /ws, authenticates with the first message
and prints what arrives: message counts and rates by type, the latest pose
and the decoded headers of the binary map/scan frames.

  python3 ws_probe.py --host 192.168.100.181 --password ... --seconds 10
  python3 ws_probe.py --token <token> --seconds 30 --follow   # live pose
"""

import argparse
import getpass
import json
import struct
import sys
import time
import urllib.request

import websocket  # websocket-client


def login(host, port, password):
    req = urllib.request.Request(
        f'http://{host}:{port}/api/login', data=json.dumps({'password': password}).encode(),
        method='POST')
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.load(r)['token']


def describe_binary(data):
    kind, version = data[0], data[1]
    if kind == 1:
        w, h, res, ox, oy = struct.unpack_from('<IIfff', data, 4)
        return f'map {w}x{h} @ {res:.2f} m, origin ({ox:.2f}, {oy:.2f}), {len(data)} bytes'
    if kind == 2:
        n, rx, ry, ryaw = struct.unpack_from('<Ifff', data, 4)
        return f'scan {n} points, robot ({rx:.2f}, {ry:.2f}, {ryaw:.2f}), {len(data)} bytes'
    return f'unknown binary type {kind} v{version}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=8000)
    ap.add_argument('--token')
    ap.add_argument('--password')
    ap.add_argument('--seconds', type=float, default=10.0)
    ap.add_argument('--follow', action='store_true', help='print every pose')
    ap.add_argument('--no-auth', action='store_true', help='test: never authenticate')
    a = ap.parse_args()

    token = a.token
    if not token and not a.no_auth:
        token = login(a.host, a.port, a.password or getpass.getpass('Contraseña: '))

    ws = websocket.create_connection(f'ws://{a.host}:{a.port}/ws', timeout=5)
    if not a.no_auth:
        ws.send(json.dumps({'type': 'auth', 'token': token}))

    counts, first, last_pose = {}, {}, None
    end = time.time() + a.seconds
    try:
        while time.time() < end:
            ws.settimeout(max(0.1, end - time.time()))
            try:
                opcode, data = ws.recv_data()
            except websocket.WebSocketTimeoutException:
                break
            if opcode == websocket.ABNF.OPCODE_CLOSE:
                print('el servidor cerró la conexión')
                return 2
            if opcode == websocket.ABNF.OPCODE_BINARY:
                kind = {1: 'map', 2: 'scan'}.get(data[0], 'binary?')
                if kind not in first:
                    first[kind] = describe_binary(data)
            else:
                msg = json.loads(data)
                kind = msg.get('type', '?')
                if kind == 'pose':
                    last_pose = msg
                    if a.follow:
                        print(f"pose x={msg['x']:+.3f} y={msg['y']:+.3f} "
                              f"yaw={msg['yaw']:+.3f} cov={msg.get('cov')}")
                if kind not in first:
                    first[kind] = data[:120]
            counts[kind] = counts.get(kind, 0) + 1
    except websocket.WebSocketConnectionClosedException:
        print('el servidor cerró la conexión')
        return 2
    finally:
        ws.close()

    print(f'mensajes en {a.seconds:.0f} s:')
    for k, n in sorted(counts.items()):
        print(f'  {k:10s} {n:5d}  ({n / a.seconds:5.1f}/s)  ej: {first.get(k)}')
    if last_pose:
        print(f'última pose: {last_pose}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
