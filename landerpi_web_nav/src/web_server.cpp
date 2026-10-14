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

#include "landerpi_web_nav/web_server.hpp"

#include <algorithm>
#include <atomic>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <boost/asio/dispatch.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/strand.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>

#include <nlohmann/json.hpp>

namespace landerpi_web_nav
{

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

namespace
{

std::string mimeType(const std::string & path)
{
  const auto dot = path.rfind('.');
  const std::string ext = dot == std::string::npos ? "" : path.substr(dot);
  if (ext == ".html") {return "text/html; charset=utf-8";}
  if (ext == ".js") {return "application/javascript";}
  if (ext == ".css") {return "text/css";}
  if (ext == ".json") {return "application/json";}
  if (ext == ".svg") {return "image/svg+xml";}
  if (ext == ".png") {return "image/png";}
  if (ext == ".ico") {return "image/x-icon";}
  if (ext == ".woff2") {return "font/woff2";}
  return "application/octet-stream";
}

std::string jsonError(const std::string & message)
{
  return nlohmann::json{{"error", message}}.dump();
}

}  // namespace

// ----------------------------------------------------------------------------
// WebSocket session: one per browser tab.
// ----------------------------------------------------------------------------
class WsSession : public std::enable_shared_from_this<WsSession>
{
public:
  WsSession(tcp::socket && socket, WebServerImpl & server);
  void run(http::request<http::string_body> req);
  // Thread-safe: hops onto this session's strand before touching the queue.
  void send(std::shared_ptr<const WsFrame> frame);
  void close();

private:
  void onAccept(beast::error_code ec);
  void readNext();
  void onRead(beast::error_code ec, std::size_t bytes);
  void writeNext();
  void onWrite(beast::error_code ec, std::size_t bytes);
  // Unregisters and tells the gateway, once, that this client is gone.
  void gone();

  websocket::stream<beast::tcp_stream> ws_;
  WsClientId id_ = 0;
  bool gone_ = false;
  WebServerImpl & server_;
  beast::flat_buffer buffer_;
  net::steady_timer auth_timer_;
  bool authenticated_ = false;
  bool writing_ = false;
  std::deque<std::shared_ptr<const WsFrame>> queue_;
};

// ----------------------------------------------------------------------------
// Server internals.
// ----------------------------------------------------------------------------
struct WebServerImpl
{
  WebServerConfig config;
  WebServerCallbacks callbacks;
  // Recreated on every start(): destroying the io_context destroys every
  // pending handler, and with them the last references to the sessions, so
  // stop() really closes every socket.
  std::unique_ptr<net::io_context> ioc;
  std::unique_ptr<tcp::acceptor> acceptor;
  std::thread thread;
  bool running = false;
  std::atomic<WsClientId> next_client_id{1};

  mutable std::mutex sessions_mutex;
  std::set<std::shared_ptr<WsSession>> sessions;  // authenticated only

  void accept();
  void addSession(const std::shared_ptr<WsSession> & s)
  {
    std::lock_guard<std::mutex> lock(sessions_mutex);
    sessions.insert(s);
  }
  void removeSession(const std::shared_ptr<WsSession> & s)
  {
    std::lock_guard<std::mutex> lock(sessions_mutex);
    sessions.erase(s);
  }
};

// ----------------------------------------------------------------------------
// HTTP session: reads requests on one connection until it closes or
// upgrades to a WebSocket.
// ----------------------------------------------------------------------------
class HttpSession : public std::enable_shared_from_this<HttpSession>
{
public:
  HttpSession(tcp::socket && socket, WebServerImpl & server, std::string client_ip)
  : stream_(std::move(socket)), server_(server), client_ip_(std::move(client_ip)) {}

  void run()
  {
    net::dispatch(
      stream_.get_executor(), beast::bind_front_handler(&HttpSession::read, shared_from_this()));
  }

private:
  void read()
  {
    req_ = {};
    stream_.expires_after(std::chrono::seconds(30));
    http::async_read(
      stream_, buffer_, req_, beast::bind_front_handler(&HttpSession::onRead, shared_from_this()));
  }

  void onRead(beast::error_code ec, std::size_t)
  {
    if (ec) {
      return;  // closed, timed out or malformed: just drop the connection
    }
    if (websocket::is_upgrade(req_)) {
      if (req_.target() == "/ws") {
        stream_.expires_never();  // the WebSocket manages its own timeouts
        std::make_shared<WsSession>(stream_.release_socket(), server_)->run(std::move(req_));
      }
      return;
    }
    respond(handle());
  }

  http::response<http::string_body> handle()
  {
    const std::string target(req_.target());
    http::response<http::string_body> res;
    res.version(req_.version());
    res.keep_alive(req_.keep_alive());
    res.set(http::field::server, "landerpi-web-gateway");
    // Never cache API answers; they are live state.
    res.set(http::field::cache_control, "no-store");

    if (target.rfind("/api/", 0) == 0) {
      HttpRequest r;
      r.method = std::string(req_.method_string());
      r.target = target;
      r.body = req_.body();
      r.client_ip = client_ip_;
      const auto auth = req_[http::field::authorization];
      const std::string prefix = "Bearer ";
      if (auth.size() > prefix.size() && std::string(auth.substr(0, prefix.size())) == prefix) {
        r.bearer_token = std::string(auth.substr(prefix.size()));
      }
      HttpResponse out;
      try {
        out = server_.callbacks.api(r);
      } catch (const std::exception & e) {
        out = {500, "application/json", jsonError(e.what())};
      }
      res.result(static_cast<http::status>(out.status));
      res.set(http::field::content_type, out.content_type);
      res.body() = std::move(out.body);
      res.prepare_payload();
      return res;
    }

    // Static files: GET only, no ".." anywhere in the path.
    std::string path = target.substr(0, target.find('?'));
    if (req_.method() != http::verb::get || path.find("..") != std::string::npos) {
      res.result(http::status::bad_request);
      res.set(http::field::content_type, "text/plain");
      res.body() = "bad request";
      res.prepare_payload();
      return res;
    }
    if (path == "/" || path.empty()) {
      path = "/index.html";
    }
    std::ifstream file(server_.config.static_root + path, std::ios::binary);
    if (!file) {
      res.result(http::status::not_found);
      res.set(http::field::content_type, "text/plain");
      res.body() = server_.config.static_root.empty() ?
        "landerpi web gateway: no web app installed yet" : "not found";
      res.prepare_payload();
      return res;
    }
    std::ostringstream content;
    content << file.rdbuf();
    res.set(http::field::content_type, mimeType(path));
    res.body() = content.str();
    res.prepare_payload();
    return res;
  }

  void respond(http::response<http::string_body> && res)
  {
    auto sp = std::make_shared<http::response<http::string_body>>(std::move(res));
    res_ = sp;  // keep it alive until the write completes
    http::async_write(
      stream_, *sp, beast::bind_front_handler(
        &HttpSession::onWrite, shared_from_this(), sp->need_eof()));
  }

  void onWrite(bool close, beast::error_code ec, std::size_t)
  {
    if (ec) {
      return;
    }
    if (close) {
      beast::error_code ignored;
      stream_.socket().shutdown(tcp::socket::shutdown_send, ignored);
      return;
    }
    res_.reset();
    read();
  }

  beast::tcp_stream stream_;
  WebServerImpl & server_;
  std::string client_ip_;
  beast::flat_buffer buffer_;
  http::request<http::string_body> req_;
  std::shared_ptr<void> res_;
};

// Sends a 403 and closes: used for connections from outside the allowlist.
void rejectConnection(tcp::socket socket)
{
  auto stream = std::make_shared<beast::tcp_stream>(std::move(socket));
  auto res = std::make_shared<http::response<http::string_body>>(http::status::forbidden, 11);
  res->set(http::field::content_type, "application/json");
  res->body() = jsonError("forbidden: only the home network may connect");
  res->keep_alive(false);
  res->prepare_payload();
  stream->expires_after(std::chrono::seconds(5));
  http::async_write(
    *stream, *res, [stream, res](beast::error_code, std::size_t) {
      beast::error_code ignored;
      stream->socket().shutdown(tcp::socket::shutdown_both, ignored);
    });
}

void WebServerImpl::accept()
{
  acceptor->async_accept(
    net::make_strand(*ioc), [this](beast::error_code ec, tcp::socket socket) {
      if (ec) {
        if (ec == net::error::operation_aborted) {
          return;  // stop() closed the acceptor
        }
      } else {
        beast::error_code ep_ec;
        const auto endpoint = socket.remote_endpoint(ep_ec);
        const std::string ip = ep_ec ? "" : endpoint.address().to_string();
        if (!ep_ec && callbacks.ip_allowed(ip)) {
          std::make_shared<HttpSession>(std::move(socket), *this, ip)->run();
        } else {
          rejectConnection(std::move(socket));
        }
      }
      accept();
    });
}

// ----------------------------------------------------------------------------
// WsSession implementation.
// ----------------------------------------------------------------------------
WsSession::WsSession(tcp::socket && socket, WebServerImpl & server)
: ws_(std::move(socket)), id_(server.next_client_id++), server_(server),
  auth_timer_(ws_.get_executor()) {}

void WsSession::gone()
{
  if (gone_) {
    return;
  }
  gone_ = true;
  server_.removeSession(shared_from_this());
  if (authenticated_ && server_.callbacks.ws_closed) {
    server_.callbacks.ws_closed(id_);
  }
}

void WsSession::run(http::request<http::string_body> req)
{
  ws_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
  ws_.async_accept(req, beast::bind_front_handler(&WsSession::onAccept, shared_from_this()));
}

void WsSession::onAccept(beast::error_code ec)
{
  if (ec) {
    return;
  }
  // The first message must authenticate before the deadline, or bye.
  auth_timer_.expires_after(server_.config.ws_auth_timeout);
  auth_timer_.async_wait(
    [self = shared_from_this()](beast::error_code tec) {
      if (!tec && !self->authenticated_) {
        self->close();
      }
    });
  readNext();
}

void WsSession::readNext()
{
  ws_.async_read(buffer_, beast::bind_front_handler(&WsSession::onRead, shared_from_this()));
}

void WsSession::onRead(beast::error_code ec, std::size_t)
{
  if (ec) {
    auth_timer_.cancel();
    gone();
    return;
  }
  const std::string msg = beast::buffers_to_string(buffer_.data());
  buffer_.consume(buffer_.size());

  if (!authenticated_) {
    std::string token;
    try {
      const auto j = nlohmann::json::parse(msg);
      if (j.value("type", "") == "auth") {
        token = j.value("token", "");
      }
    } catch (const std::exception &) {
    }
    if (token.empty() || !server_.callbacks.token_valid(token)) {
      close();
      return;
    }
    authenticated_ = true;
    auth_timer_.cancel();
    server_.addSession(shared_from_this());
    send(
      std::make_shared<const WsFrame>(
        WsFrame{false, nlohmann::json{{"type", "auth_ok"}, {"client", id_}}.dump(), {}}));
    for (auto & f : server_.callbacks.initial_frames()) {
      send(std::make_shared<const WsFrame>(std::move(f)));
    }
  } else if (server_.callbacks.ws_message) {
    server_.callbacks.ws_message(id_, msg);
  }
  readNext();
}

void WsSession::send(std::shared_ptr<const WsFrame> frame)
{
  net::post(
    ws_.get_executor(), [self = shared_from_this(), frame = std::move(frame)]() {
      if (self->queue_.size() >= self->server_.config.ws_max_queue) {
        self->queue_.pop_front();  // slow client: drop the oldest
      }
      self->queue_.push_back(frame);
      if (!self->writing_) {
        self->writeNext();
      }
    });
}

void WsSession::writeNext()
{
  if (queue_.empty()) {
    writing_ = false;
    return;
  }
  writing_ = true;
  const auto & f = *queue_.front();
  ws_.binary(f.binary);
  if (f.binary) {
    ws_.async_write(
      net::buffer(f.data), beast::bind_front_handler(&WsSession::onWrite, shared_from_this()));
  } else {
    ws_.async_write(
      net::buffer(f.text), beast::bind_front_handler(&WsSession::onWrite, shared_from_this()));
  }
}

void WsSession::onWrite(beast::error_code ec, std::size_t)
{
  if (ec) {
    gone();
    return;
  }
  queue_.pop_front();
  writeNext();
}

void WsSession::close()
{
  net::post(
    ws_.get_executor(), [self = shared_from_this()]() {
      self->gone();
      self->ws_.async_close(websocket::close_code::policy_error, [self](beast::error_code) {});
    });
}

// ----------------------------------------------------------------------------
// WebServer.
// ----------------------------------------------------------------------------
WebServer::WebServer(WebServerConfig config, WebServerCallbacks callbacks)
: impl_(std::make_unique<WebServerImpl>())
{
  if (!callbacks.api || !callbacks.token_valid || !callbacks.ip_allowed ||
    !callbacks.initial_frames)
  {
    throw std::invalid_argument("WebServer needs all four callbacks");
  }
  impl_->config = std::move(config);
  impl_->callbacks = std::move(callbacks);
}

WebServer::~WebServer() {stop();}

void WebServer::start()
{
  if (impl_->running) {
    return;
  }
  impl_->ioc = std::make_unique<net::io_context>(1);
  impl_->acceptor = std::make_unique<tcp::acceptor>(*impl_->ioc);
  const auto address = net::ip::make_address_v4(impl_->config.address);
  const tcp::endpoint endpoint{address, impl_->config.port};
  impl_->acceptor->open(endpoint.protocol());
  impl_->acceptor->set_option(net::socket_base::reuse_address(true));
  impl_->acceptor->bind(endpoint);  // throws if the port is taken
  impl_->acceptor->listen(net::socket_base::max_listen_connections);
  impl_->accept();
  impl_->running = true;
  impl_->thread = std::thread([this]() {impl_->ioc->run();});
}

void WebServer::stop()
{
  if (!impl_ || !impl_->running) {
    return;
  }
  impl_->running = false;
  impl_->ioc->stop();
  if (impl_->thread.joinable()) {
    impl_->thread.join();
  }
  {
    std::lock_guard<std::mutex> lock(impl_->sessions_mutex);
    impl_->sessions.clear();
  }
  // Acceptor first (it lives on the io_context), then the io_context with
  // every pending handler and the sessions they keep alive.
  impl_->acceptor.reset();
  impl_->ioc.reset();
}

void WebServer::broadcast(const WsFrame & frame)
{
  auto shared = std::make_shared<const WsFrame>(frame);  // one copy for all clients
  std::lock_guard<std::mutex> lock(impl_->sessions_mutex);
  for (const auto & s : impl_->sessions) {
    s->send(shared);
  }
}

std::size_t WebServer::clients() const
{
  std::lock_guard<std::mutex> lock(impl_->sessions_mutex);
  return impl_->sessions.size();
}

}  // namespace landerpi_web_nav
