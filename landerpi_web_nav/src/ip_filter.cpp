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

#include "landerpi_web_nav/ip_filter.hpp"

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace landerpi_web_nav
{

std::optional<uint32_t> IpFilter::parseIpv4(const std::string & address)
{
  uint32_t value = 0;
  int octets = 0;
  std::size_t i = 0;
  while (octets < 4) {
    if (i >= address.size() || address[i] < '0' || address[i] > '9') {
      return std::nullopt;
    }
    uint32_t octet = 0;
    std::size_t digits = 0;
    while (i < address.size() && address[i] >= '0' && address[i] <= '9') {
      octet = octet * 10 + static_cast<uint32_t>(address[i] - '0');
      ++i;
      if (++digits > 3 || octet > 255) {
        return std::nullopt;
      }
    }
    value = (value << 8) | octet;
    ++octets;
    if (octets < 4) {
      if (i >= address.size() || address[i] != '.') {
        return std::nullopt;
      }
      ++i;
    }
  }
  if (i != address.size()) {
    return std::nullopt;
  }
  return value;
}

IpFilter::IpFilter(const std::vector<std::string> & networks)
{
  if (networks.empty()) {
    throw std::invalid_argument("IpFilter needs at least one network");
  }
  for (const auto & entry : networks) {
    const auto slash = entry.find('/');
    const auto ip = parseIpv4(entry.substr(0, slash));
    int prefix = 32;
    if (slash != std::string::npos) {
      const auto bits = entry.substr(slash + 1);
      if (bits.empty() || bits.size() > 2 || bits.find_first_not_of("0123456789") !=
        std::string::npos)
      {
        throw std::invalid_argument("bad prefix in '" + entry + "'");
      }
      prefix = std::stoi(bits);
    }
    if (!ip || prefix < 0 || prefix > 32) {
      throw std::invalid_argument("bad network '" + entry + "'");
    }
    // prefix 0 would shift by 32 (undefined behaviour): mask is all zeros.
    const uint32_t mask = prefix == 0 ? 0u : (0xFFFFFFFFu << (32 - prefix));
    networks_.push_back({*ip & mask, mask});
  }
}

bool IpFilter::allows(const std::string & address) const
{
  const auto ip = parseIpv4(address);
  if (!ip) {
    return false;
  }
  for (const auto & n : networks_) {
    if ((*ip & n.mask) == n.base) {
      return true;
    }
  }
  return false;
}

}  // namespace landerpi_web_nav
