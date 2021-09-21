// Copyright (c) 2021 Institute for Robotics and Intelligent Machines,
//               Georgia Institute of Technology
// Copyright (c) 2020, Intelligent Robotics Lab
// Copyright (c) 2020, Airelectronics.
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
//
// Author: Christian Llanes <christian.llanes@gatech.edu>
// Author: David Vargas Frutos <david.vargas@urjc.es>


#include <string>
#include <list>
#include <memory>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "optitrack2_driver/optitrack2_driver.hpp"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;
using std::placeholders::_1;

class TestOptiTrackDriver : public OptitrackDriverNode
{
public:
    TestOptiTrackDriver()
  : ref_connection_type_(connection_type_),
    ref_server_address_(server_address_),
    ref_local_address_(local_address_),
    ref_multicast_address_(multicast_address_),
    ref_server_command_port_(server_command_port_),
    ref_server_data_port_(server_data_port_),
    ref_rigid_body_name_(rigid_body_name_),
    ref_lastFrameNumber_(lastFrameNumber_),
    ref_frameCount_(frameCount_),
    ref_droppedFrameCount_(droppedFrameCount_),
    ref_qos_history_policy_(qos_history_policy_),
    ref_qos_reliability_policy_(qos_reliability_policy_),
    ref_qos_depth_(qos_depth_), OptitrackDriverNode(rclcpp::NodeOptions()){
  }

  void test_init_parameters()
  {
    initParameters();
  }

    std::string ref_connection_type_;
    std::string ref_server_address_;
    std::string ref_local_address_;
    std::string ref_multicast_address_;
    uint16_t ref_server_command_port_;
    uint16_t ref_server_data_port_;
    std::string ref_rigid_body_name_;
    int ref_lastFrameNumber_;
    int ref_frameCount_;
    int ref_droppedFrameCount_;
    std::string ref_qos_history_policy_;
    std::string ref_qos_reliability_policy_;
    int ref_qos_depth_;
};

TEST(UtilsTest, test_optitrack2_params)
{
  auto optitrack2_node = std::make_shared<TestOptiTrackDriver>();
  auto test_node = rclcpp::Node::make_shared("optitrack2_test_node");

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::executor::ExecutorArgs(), 4);
  exe.add_node(optitrack2_node->get_node_base_interface());
  exe.add_node(test_node->get_node_base_interface());

  auto set_parameters_results = optitrack2_node->set_parameters(
  {
      rclcpp::Parameter("connection_type", "Unicast"),
      rclcpp::Parameter("server_address", "000.000.000.000"),
      rclcpp::Parameter("local_address", "000.000.000.000"),
      rclcpp::Parameter("multicast_address", "000.000.000.000"),
      rclcpp::Parameter("server_command_port", 0),
      rclcpp::Parameter("server_data_port", 0),
      rclcpp::Parameter("rigid_body_name", "mav1"),
      rclcpp::Parameter("lastFrameNumber", 0),
      rclcpp::Parameter("frameCount", 0),
      rclcpp::Parameter("droppedFrameCount", 0),
      rclcpp::Parameter("qos_history_policy", "keep_all"),
      rclcpp::Parameter("qos_reliability_policy", "best_effort"),
      rclcpp::Parameter("qos_depth", 10),
  });

  optitrack2_node->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE));
  exe.spin_some();

  EXPECT_EQ(State::PRIMARY_STATE_INACTIVE, optitrack2_node->get_current_state().id());

  optitrack2_node->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVATE));

  ASSERT_EQ(optitrack2_node->ref_connection_type_, "Unicast");
  ASSERT_EQ(optitrack2_node->ref_server_address_, "000.000.000.000");
  ASSERT_EQ(optitrack2_node->ref_local_address_, "000.000.000.000");
  ASSERT_EQ(optitrack2_node->ref_multicast_address_, "000.000.000.000");
  ASSERT_EQ(optitrack2_node->ref_server_command_port_, 0);
  ASSERT_EQ(optitrack2_node->ref_server_data_port_, 0);
  ASSERT_EQ(optitrack2_node->ref_rigid_body_name_, "mav1");
  ASSERT_EQ(optitrack2_node->ref_lastFrameNumber_, 0);
  ASSERT_EQ(optitrack2_node->ref_frameCount_, 0);
  ASSERT_EQ(optitrack2_node->ref_droppedFrameCount_, 0);
  ASSERT_EQ(optitrack2_node->ref_qos_history_policy_, "keep_all");
  ASSERT_EQ(optitrack2_node->ref_qos_reliability_policy_, "best_effort");
  ASSERT_EQ(optitrack2_node->ref_qos_depth_, 10);
}


int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
