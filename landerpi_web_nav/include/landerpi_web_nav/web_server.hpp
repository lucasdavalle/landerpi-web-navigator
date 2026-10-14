// Copyright 2026 ldavalle
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace landerpi_web_nav
{

// HTTP + WebSocket server on Boost.Beast, with no ROS in it: the gateway
// node plugs its logic in through the callbacks below. Runs its own
// io_context on its own thread (start/stop), so ROS callbacks never block
// on the network and vice versa.
//
// - Every connection is checked against `ip_allowed` first (403 if not).
// - `/api/*` requests go to `api`; anything else is a static file under
//   `static_root` (index.html for "/"), with path traversal rejected.
// - `/ws` upgrades to a WebSocket that must authenticate with its first
//   message, {"type":"auth","token":"..."}, within ws_auth_timeout; only
//   then does it receive `initial_frames` and every broadcast.

struct HttpRequest
{
  std::string method;
  std::string target;        // path + query, e.g. "/api/status"
  std::string body;
  std::string bearer_token;  // from "Authorization: Bearer ...", empty if none
  std::string client_ip;
};

struct HttpResponse
{
  int status = 200;
  std::string content_type = "application/json";
  std::string body;
};

// One WebSocket message: text (JSON) or binary.
struct WsFrame
{
  bool binary = false;
  std::string text;
  std::vector<uint8_t> data;
};

struct WebServerConfig
{
  std::string address = "0.0.0.0";  // IPv4 only, see IpFilter
  uint16_t port = 8000;
  std::string static_root;           // directory with the built web app
  std::chrono::milliseconds ws_auth_timeout{3000};
  // Per-client outgoing queue limit: a slow browser drops the oldest
  // stream frames instead of making the server's memory grow.
  std::size_t ws_max_queue = 32;
};

struct WebServerCallbacks
{
  std::function<HttpResponse(const HttpRequest &)> api;
  std::function<bool(const std::string & token)> token_valid;
  std::function<bool(const std::string & ip)> ip_allowed;
  std::function<std::vector<WsFrame>()> initial_frames;
};

struct WebServerImpl;  // keeps Boost headers out of this header

class WebServer
{
public:
  WebServer(WebServerConfig config, WebServerCallbacks callbacks);
  ~WebServer();
  WebServer(const WebServer &) = delete;
  WebServer & operator=(const WebServer &) = delete;

  // Binds and starts the network thread. Throws on bind failure (port in
  // use, bad address).
  void start();
  // Stops the network thread and closes every connection (the whole
  // io_context is destroyed, so no socket survives). Idempotent; start()
  // can be called again afterwards.
  void stop();

  // Thread-safe: queues the frame for every authenticated WebSocket client.
  void broadcast(const WsFrame & frame);
  std::size_t clients() const;

private:
  std::unique_ptr<WebServerImpl> impl_;
};

}  // namespace landerpi_web_nav
