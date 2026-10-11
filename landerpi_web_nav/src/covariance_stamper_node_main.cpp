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

#include <exception>
#include <memory>

#include "landerpi_web_nav/covariance_stamper_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<landerpi_web_nav::CovarianceStamperNode>());
  } catch (const std::exception & e) {
    // Bad parameters (wrong array size, negative stddev...) end up here: fail
    // loudly instead of republishing with made-up covariances.
    RCLCPP_FATAL(rclcpp::get_logger("covariance_stamper"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
