// Copyright 2022 Víctor Mayoral-Vilches
// All rights reserved.

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "fpga_harris_node.hpp"
#include "cmd_line_sched.hpp"
#include "sched_utils.hpp"

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  auto options_reader = SchedOptionsReader();
  if (!options_reader.read_options(argc, argv)) {
    options_reader.print_usage();
    return 0;
  }
  auto options = options_reader.get_options();

  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  auto harris_node = std::make_shared<rt_hw_accel_demo::HarrisNodeFPGA>(node_options);

  set_thread_scheduling(pthread_self(), options.policy, options.priority);

  rclcpp::spin(harris_node);
  rclcpp::shutdown();
  return 0;
}
