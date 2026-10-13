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

#include <gtest/gtest.h>

#include <chrono>
#include <set>
#include <stdexcept>
#include <string>

#include "landerpi_web_nav/auth_manager.hpp"

using landerpi_web_nav::AuthConfig;
using landerpi_web_nav::AuthManager;
using landerpi_web_nav::LoginStatus;
using landerpi_web_nav::PasswordHash;
using std::chrono::seconds;

namespace
{

// A clock the test moves by hand.
struct FakeClock
{
  AuthManager::Clock::time_point t{};
  AuthManager::NowFn fn() {return [this]() {return t;};}
};

// Few iterations: the tests check the logic, not PBKDF2's slowness.
PasswordHash testHash() {return PasswordHash::create("correcta", 1000);}

}  // namespace

TEST(PasswordHash, KnownPbkdf2Vector)
{
  // RFC 7914 section 11, PBKDF2-HMAC-SHA256 test vector.
  const std::string salt = "salt";
  const auto out = landerpi_web_nav::pbkdf2Sha256(
    "passwd", std::vector<uint8_t>(salt.begin(), salt.end()), 1, 64);
  EXPECT_EQ(out[0], 0x55);
  EXPECT_EQ(out[1], 0xac);
  EXPECT_EQ(out[2], 0x04);
  EXPECT_EQ(out[63], 0x83);
}

TEST(PasswordHash, SerializeRoundTrip)
{
  const auto h = PasswordHash::create("x", 20000);
  const auto parsed = PasswordHash::parse(h.serialize() + "\n");
  EXPECT_EQ(parsed.iterations, 20000u);
  EXPECT_EQ(parsed.salt, h.salt);
  EXPECT_EQ(parsed.hash, h.hash);
}

TEST(PasswordHash, SaltIsRandom)
{
  EXPECT_NE(PasswordHash::create("x", 1000).salt, PasswordHash::create("x", 1000).salt);
}

TEST(PasswordHash, RejectsMalformedLines)
{
  EXPECT_THROW(PasswordHash::parse("plaintext"), std::invalid_argument);
  EXPECT_THROW(PasswordHash::parse("md5$1$aa$bb"), std::invalid_argument);
  EXPECT_THROW(PasswordHash::parse("pbkdf2-sha256$100$aa$bb"), std::invalid_argument);
  EXPECT_THROW(PasswordHash::parse("pbkdf2-sha256$20000$zz$bb"), std::invalid_argument);
}

TEST(AuthManager, CorrectPasswordGivesAToken)
{
  FakeClock clock;
  AuthManager auth{testHash(), AuthConfig{}, clock.fn()};
  const auto r = auth.login("correcta", "192.168.100.5");
  ASSERT_EQ(r.status, LoginStatus::kOk);
  EXPECT_EQ(r.token.size(), 64u);  // 256 bits in hex
  EXPECT_EQ(r.expires_in, seconds{12 * 3600});
  EXPECT_TRUE(auth.validate(r.token));
}

TEST(AuthManager, WrongPasswordGivesNothing)
{
  FakeClock clock;
  AuthManager auth{testHash(), AuthConfig{}, clock.fn()};
  const auto r = auth.login("incorrecta", "192.168.100.5");
  EXPECT_EQ(r.status, LoginStatus::kWrongPassword);
  EXPECT_TRUE(r.token.empty());
  EXPECT_FALSE(auth.validate(""));
  EXPECT_FALSE(auth.validate("deadbeef"));
}

TEST(AuthManager, TokensAreUnique)
{
  FakeClock clock;
  AuthManager auth{testHash(), AuthConfig{}, clock.fn()};
  std::set<std::string> tokens;
  for (int i = 0; i < 20; ++i) {
    tokens.insert(auth.login("correcta", "192.168.100.5").token);
  }
  EXPECT_EQ(tokens.size(), 20u);
}

TEST(AuthManager, TokenExpires)
{
  FakeClock clock;
  AuthConfig cfg;
  cfg.token_ttl = seconds{60};
  AuthManager auth{testHash(), cfg, clock.fn()};
  const auto token = auth.login("correcta", "192.168.100.5").token;
  clock.t += seconds{59};
  EXPECT_TRUE(auth.validate(token));
  clock.t += seconds{1};
  EXPECT_FALSE(auth.validate(token));
  EXPECT_EQ(auth.activeTokens(), 0u);
}

TEST(AuthManager, LogoutInvalidatesTheToken)
{
  FakeClock clock;
  AuthManager auth{testHash(), AuthConfig{}, clock.fn()};
  const auto token = auth.login("correcta", "192.168.100.5").token;
  auth.logout(token);
  EXPECT_FALSE(auth.validate(token));
}

TEST(AuthManager, LocksOutAfterTooManyFailures)
{
  FakeClock clock;
  AuthManager auth{testHash(), AuthConfig{}, clock.fn()};  // 5 per minute
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(auth.login("nope", "192.168.100.66").status, LoginStatus::kWrongPassword);
  }
  // Locked: even the right password is refused for now...
  EXPECT_EQ(auth.login("correcta", "192.168.100.66").status, LoginStatus::kLockedOut);
  // ...but only for that IP.
  EXPECT_EQ(auth.login("correcta", "192.168.100.5").status, LoginStatus::kOk);
  // Once the failures leave the 60 s window, it can log in again.
  clock.t += seconds{60};
  EXPECT_EQ(auth.login("correcta", "192.168.100.66").status, LoginStatus::kOk);
}

TEST(AuthManager, SuccessfulLoginClearsFailures)
{
  FakeClock clock;
  AuthManager auth{testHash(), AuthConfig{}, clock.fn()};
  for (int i = 0; i < 4; ++i) {
    auth.login("nope", "192.168.100.66");
  }
  ASSERT_EQ(auth.login("correcta", "192.168.100.66").status, LoginStatus::kOk);
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(auth.login("nope", "192.168.100.66").status, LoginStatus::kWrongPassword);
  }
}

TEST(AuthManager, RejectsInvalidConfig)
{
  AuthConfig bad;
  bad.max_failures = 0;
  EXPECT_THROW(AuthManager(testHash(), bad), std::invalid_argument);
  EXPECT_THROW(AuthManager(PasswordHash{}, AuthConfig{}), std::invalid_argument);
}
