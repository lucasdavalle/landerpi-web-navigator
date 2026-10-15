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

#include <memory>

#include "landerpi_posture/posture_manager_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<landerpi_posture::PostureManagerNode>();
  // Norm: configure + activate at start. Active = ready; the arm moves only
  // on ~/go_high or ~/go_low.
  if (node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE).id() !=
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE).id() !=
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_ERROR(node->get_logger(), "failed to start posture_manager");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
