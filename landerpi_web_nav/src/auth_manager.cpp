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

#include "landerpi_web_nav/auth_manager.hpp"

#include <openssl/crypto.h>
#include <openssl/evp.h>
#include <openssl/rand.h>

#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace landerpi_web_nav
{

namespace
{

std::string toHex(const std::vector<uint8_t> & bytes)
{
  static const char * digits = "0123456789abcdef";
  std::string out;
  out.reserve(bytes.size() * 2);
  for (uint8_t b : bytes) {
    out.push_back(digits[b >> 4]);
    out.push_back(digits[b & 0x0F]);
  }
  return out;
}

std::vector<uint8_t> fromHex(const std::string & hex)
{
  if (hex.empty() || hex.size() % 2 != 0) {
    throw std::invalid_argument("bad hex string");
  }
  auto nibble = [](char c) -> int {
      if (c >= '0' && c <= '9') {
        return c - '0';
      }
      if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
      }
      if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
      }
      throw std::invalid_argument("bad hex digit");
    };
  std::vector<uint8_t> out(hex.size() / 2);
  for (std::size_t i = 0; i < out.size(); ++i) {
    out[i] = static_cast<uint8_t>(nibble(hex[2 * i]) << 4 | nibble(hex[2 * i + 1]));
  }
  return out;
}

std::vector<uint8_t> randomBytes(std::size_t n)
{
  std::vector<uint8_t> out(n);
  // OpenSSL's CSPRNG (seeded from the OS): suitable for salts and tokens,
  // unlike std::rand or std::mt19937.
  if (RAND_bytes(out.data(), static_cast<int>(n)) != 1) {
    throw std::runtime_error("RAND_bytes failed");
  }
  return out;
}

}  // namespace

std::vector<uint8_t> pbkdf2Sha256(
  const std::string & password, const std::vector<uint8_t> & salt, uint32_t iterations,
  std::size_t length)
{
  std::vector<uint8_t> out(length);
  if (PKCS5_PBKDF2_HMAC(
      password.data(), static_cast<int>(password.size()), salt.data(),
      static_cast<int>(salt.size()), static_cast<int>(iterations), EVP_sha256(),
      static_cast<int>(length), out.data()) != 1)
  {
    throw std::runtime_error("PBKDF2 failed");
  }
  return out;
}

PasswordHash PasswordHash::parse(const std::string & line)
{
  std::vector<std::string> parts;
  std::stringstream ss(line);
  std::string part;
  while (std::getline(ss, part, '$')) {
    parts.push_back(part);
  }
  if (parts.size() != 4 || parts[0] != "pbkdf2-sha256") {
    throw std::invalid_argument("password file must be pbkdf2-sha256$iter$salt$hash");
  }
  PasswordHash h;
  try {
    h.iterations = static_cast<uint32_t>(std::stoul(parts[1]));
  } catch (const std::exception &) {
    throw std::invalid_argument("bad iteration count");
  }
  if (h.iterations < 10000) {
    throw std::invalid_argument("too few PBKDF2 iterations (< 10000)");
  }
  h.salt = fromHex(parts[2]);
  // Trailing newline from the file is tolerated.
  std::string hash_hex = parts[3];
  while (!hash_hex.empty() && (hash_hex.back() == '\n' || hash_hex.back() == '\r')) {
    hash_hex.pop_back();
  }
  h.hash = fromHex(hash_hex);
  return h;
}

std::string PasswordHash::serialize() const
{
  return "pbkdf2-sha256$" + std::to_string(iterations) + "$" + toHex(salt) + "$" + toHex(hash);
}

PasswordHash PasswordHash::create(const std::string & password, uint32_t iterations)
{
  PasswordHash h;
  h.iterations = iterations;
  h.salt = randomBytes(16);
  h.hash = pbkdf2Sha256(password, h.salt, iterations, 32);
  return h;
}

AuthManager::AuthManager(PasswordHash password, AuthConfig config, NowFn now)
: password_(std::move(password)), config_(config), now_(std::move(now))
{
  if (password_.hash.empty() || password_.salt.empty() || password_.iterations == 0) {
    throw std::invalid_argument("empty password hash");
  }
  if (config_.token_ttl.count() <= 0 || config_.max_failures < 1 ||
    config_.failure_window.count() <= 0)
  {
    throw std::invalid_argument("invalid auth config");
  }
}

bool AuthManager::lockedOut(const std::string & client_ip, Clock::time_point now)
{
  auto it = failures_.find(client_ip);
  if (it == failures_.end()) {
    return false;
  }
  auto & times = it->second;
  while (!times.empty() && now - times.front() >= config_.failure_window) {
    times.pop_front();
  }
  if (times.empty()) {
    failures_.erase(it);
    return false;
  }
  return static_cast<int>(times.size()) >= config_.max_failures;
}

LoginResult AuthManager::login(const std::string & password, const std::string & client_ip)
{
  const auto now = now_();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (lockedOut(client_ip, now)) {
      // Not even hashed: a locked-out client gets no information and costs
      // no CPU.
      return {LoginStatus::kLockedOut, {}, std::chrono::seconds{0}};
    }
  }

  // Hashing is deliberately slow (that is the point of PBKDF2): done
  // outside the lock so other requests are not blocked meanwhile.
  const auto candidate = pbkdf2Sha256(
    password, password_.salt, password_.iterations, password_.hash.size());
  // CRYPTO_memcmp takes the same time wherever the first difference is, so
  // response timing does not leak how much of the hash matched.
  const bool match = CRYPTO_memcmp(
    candidate.data(), password_.hash.data(), password_.hash.size()) == 0;

  std::lock_guard<std::mutex> lock(mutex_);
  if (!match) {
    failures_[client_ip].push_back(now);
    return {LoginStatus::kWrongPassword, {}, std::chrono::seconds{0}};
  }
  failures_.erase(client_ip);
  const std::string token = toHex(randomBytes(32));  // 256 bits
  tokens_[token] = now + config_.token_ttl;
  return {LoginStatus::kOk, token, config_.token_ttl};
}

bool AuthManager::validate(const std::string & token)
{
  if (token.empty()) {
    return false;
  }
  const auto now = now_();
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = tokens_.find(token);
  if (it == tokens_.end()) {
    return false;
  }
  if (now >= it->second) {
    tokens_.erase(it);
    return false;
  }
  return true;
}

void AuthManager::logout(const std::string & token)
{
  std::lock_guard<std::mutex> lock(mutex_);
  tokens_.erase(token);
}

std::size_t AuthManager::activeTokens()
{
  const auto now = now_();
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto it = tokens_.begin(); it != tokens_.end(); ) {
    it = now >= it->second ? tokens_.erase(it) : std::next(it);
  }
  return tokens_.size();
}

}  // namespace landerpi_web_nav
