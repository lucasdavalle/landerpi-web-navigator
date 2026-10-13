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
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace landerpi_web_nav
{

// The stored password: never the password itself, only a PBKDF2-SHA256
// hash with a random salt. One line in the password file:
//   pbkdf2-sha256$<iterations>$<salt hex>$<hash hex>
// (the same format scripts/a7_set_password writes).
struct PasswordHash
{
  uint32_t iterations = 0;
  std::vector<uint8_t> salt;
  std::vector<uint8_t> hash;

  static PasswordHash parse(const std::string & line);  // throws std::invalid_argument
  std::string serialize() const;
  // Hashes a new password with a fresh random salt.
  static PasswordHash create(const std::string & password, uint32_t iterations = 200000);
};

std::vector<uint8_t> pbkdf2Sha256(
  const std::string & password, const std::vector<uint8_t> & salt, uint32_t iterations,
  std::size_t length);

struct AuthConfig
{
  std::chrono::seconds token_ttl{12 * 3600};
  // Brute-force protection: after max_failures wrong passwords from one IP
  // within failure_window, that IP is locked out until the oldest failure
  // leaves the window.
  int max_failures = 5;
  std::chrono::seconds failure_window{60};
};

enum class LoginStatus
{
  kOk,
  kWrongPassword,
  kLockedOut,
};

struct LoginResult
{
  LoginStatus status = LoginStatus::kWrongPassword;
  std::string token;                       // only when kOk
  std::chrono::seconds expires_in{0};      // only when kOk
};

// Password check, token issuing and validation. Thread-safe. Uses OpenSSL
// for PBKDF2, random bytes and constant-time comparison; no ROS.
class AuthManager
{
public:
  using Clock = std::chrono::steady_clock;
  using NowFn = std::function<Clock::time_point()>;

  // `now` is injectable so tests can move time forward.
  AuthManager(PasswordHash password, AuthConfig config, NowFn now = Clock::now);

  LoginResult login(const std::string & password, const std::string & client_ip);
  // True if the token exists and has not expired. Expired tokens are dropped.
  bool validate(const std::string & token);
  void logout(const std::string & token);

  std::size_t activeTokens();

private:
  bool lockedOut(const std::string & client_ip, Clock::time_point now);

  PasswordHash password_;
  AuthConfig config_;
  NowFn now_;
  std::mutex mutex_;
  std::map<std::string, Clock::time_point> tokens_;  // token -> expiry
  std::map<std::string, std::deque<Clock::time_point>> failures_;  // ip -> times
};

}  // namespace landerpi_web_nav
