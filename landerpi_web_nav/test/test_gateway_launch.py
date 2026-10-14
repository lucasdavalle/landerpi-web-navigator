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
"""Integration test: the real web_gateway node, its REST API and auth."""

import hashlib
import json
import os
import secrets
import socket
import tempfile
import time
import unittest
import urllib.error
import urllib.request

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest

PORT = 18080  # not 8000: a real gateway may be running on the robot
PASSWORD = 'integration-test-password'


def write_password_file():
    salt = secrets.token_bytes(16)
    digest = hashlib.pbkdf2_hmac('sha256', PASSWORD.encode(), salt, 20000, 32)
    fd, path = tempfile.mkstemp(suffix='.a7pw')
    with os.fdopen(fd, 'w') as f:
        f.write(f'pbkdf2-sha256$20000${salt.hex()}${digest.hex()}\n')
    return path


@pytest.mark.launch_test
def generate_test_description():
    gateway = launch_ros.actions.Node(
        package='landerpi_web_nav',
        executable='web_gateway_node',
        name='web_gateway_test',
        parameters=[{
            'port': PORT,
            'password_file': write_password_file(),
            'allowed_networks': ['127.0.0.1'],
        }],
    )
    return launch.LaunchDescription([gateway, launch_testing.actions.ReadyToTest()])


def request(path, method='GET', body=None, token=None):
    req = urllib.request.Request(
        f'http://127.0.0.1:{PORT}{path}', method=method,
        data=json.dumps(body).encode() if body is not None else None)
    if token:
        req.add_header('Authorization', f'Bearer {token}')
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read() or b'{}')
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b'{}')


class TestGateway(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Wait for the server to listen (the node configures + activates).
        deadline = time.time() + 20
        while time.time() < deadline:
            with socket.socket() as s:
                if s.connect_ex(('127.0.0.1', PORT)) == 0:
                    return
            time.sleep(0.2)
        raise AssertionError('gateway never started listening')

    def test_requires_a_token(self):
        status, _ = request('/api/status')
        self.assertEqual(status, 401)

    def test_wrong_password_is_rejected(self):
        status, body = request('/api/login', 'POST', {'password': 'nope'})
        self.assertEqual(status, 401)
        self.assertNotIn('token', body)

    def test_login_then_read_only_api(self):
        status, body = request('/api/login', 'POST', {'password': PASSWORD})
        self.assertEqual(status, 200)
        token = body['token']
        self.assertEqual(len(token), 64)

        status, body = request('/api/status', token=token)
        self.assertEqual(status, 200)
        self.assertEqual(body['type'], 'status')
        self.assertIn('sensors', body)

        status, _ = request('/api/map', token=token)
        self.assertEqual(status, 404)  # no SLAM running in the test

        status, body = request('/api/walls', token=token)
        self.assertEqual(status, 200)
        self.assertEqual(body['segments'], [])

        status, _ = request('/api/status', 'POST', {}, token=token)
        self.assertEqual(status, 405)  # read-only

        status, _ = request('/api/logout', 'POST', {}, token=token)
        self.assertEqual(status, 200)
        status, _ = request('/api/status', token=token)
        self.assertEqual(status, 401)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
