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

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace landerpi_web_nav
{

// "Only from home": an allowlist of IPv4 networks. The gateway listens on
// IPv4 only (the robot also has a public IPv6 address, never used), and
// every request from an address outside these networks gets 403 before the
// password is even looked at.
class IpFilter
{
public:
  // CIDR strings like "192.168.100.0/24" or "127.0.0.1" (= /32). Throws
  // std::invalid_argument on a malformed entry or an empty list.
  explicit IpFilter(const std::vector<std::string> & networks);

  // Dotted IPv4 address; anything that does not parse (IPv6, garbage) is
  // not allowed.
  bool allows(const std::string & address) const;

  // Parses dotted IPv4 ("192.168.0.1") into a host-order integer.
  static std::optional<uint32_t> parseIpv4(const std::string & address);

private:
  struct Network
  {
    uint32_t base;
    uint32_t mask;
  };
  std::vector<Network> networks_;
};

}  // namespace landerpi_web_nav
