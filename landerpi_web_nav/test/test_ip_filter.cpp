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

#include <stdexcept>
#include <string>
#include <vector>

#include "landerpi_web_nav/ip_filter.hpp"

using landerpi_web_nav::IpFilter;

namespace
{

IpFilter homeFilter()
{
  return IpFilter{{"192.168.100.0/24", "192.168.149.0/24", "127.0.0.1"}};
}

}  // namespace

TEST(IpFilter, AllowsTheHomeNetworks)
{
  const auto f = homeFilter();
  EXPECT_TRUE(f.allows("192.168.100.181"));  // the robot itself
  EXPECT_TRUE(f.allows("192.168.100.1"));
  EXPECT_TRUE(f.allows("192.168.149.20"));   // fallback hotspot
  EXPECT_TRUE(f.allows("127.0.0.1"));
}

TEST(IpFilter, RejectsEverythingElse)
{
  const auto f = homeFilter();
  EXPECT_FALSE(f.allows("192.168.101.5"));   // neighbouring /24
  EXPECT_FALSE(f.allows("10.0.0.1"));
  EXPECT_FALSE(f.allows("8.8.8.8"));
  EXPECT_FALSE(f.allows("127.0.0.2"));       // only the exact /32
}

TEST(IpFilter, RejectsAnythingThatIsNotIpv4)
{
  const auto f = homeFilter();
  EXPECT_FALSE(f.allows("2803:9800:988a:76ff::1"));  // the robot's public IPv6
  EXPECT_FALSE(f.allows("::ffff:192.168.100.5"));
  EXPECT_FALSE(f.allows(""));
  EXPECT_FALSE(f.allows("192.168.100"));
  EXPECT_FALSE(f.allows("192.168.100.256"));
  EXPECT_FALSE(f.allows("192.168.100.5 "));
  EXPECT_FALSE(f.allows("192.168.0100.5"));
}

TEST(IpFilter, ParsesAddresses)
{
  EXPECT_EQ(IpFilter::parseIpv4("0.0.0.0"), 0u);
  EXPECT_EQ(IpFilter::parseIpv4("255.255.255.255"), 0xFFFFFFFFu);
  EXPECT_EQ(IpFilter::parseIpv4("192.168.1.2"), 0xC0A80102u);
  EXPECT_FALSE(IpFilter::parseIpv4("1.2.3.4.5").has_value());
}

TEST(IpFilter, PrefixZeroAllowsEverything)
{
  const IpFilter any{{"0.0.0.0/0"}};
  EXPECT_TRUE(any.allows("8.8.8.8"));
}

TEST(IpFilter, RejectsMalformedNetworks)
{
  EXPECT_THROW(IpFilter{{}}, std::invalid_argument);
  EXPECT_THROW(IpFilter{{"192.168.1.0/33"}}, std::invalid_argument);
  EXPECT_THROW(IpFilter{{"192.168.1.0/"}}, std::invalid_argument);
  EXPECT_THROW(IpFilter{{"192.168.1/24"}}, std::invalid_argument);
  EXPECT_THROW(IpFilter{{"home"}}, std::invalid_argument);
}
