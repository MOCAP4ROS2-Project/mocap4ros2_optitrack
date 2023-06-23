// Copyright 2021 Institute for Robotics and Intelligent Machines,
//                Georgia Institute of Technology
// Copyright 2019 Intelligent Robotics Lab
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

#ifndef MOCAP_OPTITRACK_DRIVER__MOCAP_OPTITRACK_DRIVER_HPP_
#define MOCAP_OPTITRACK_DRIVER__MOCAP_OPTITRACK_DRIVER_HPP_

#include <iostream>
#include <sstream>
#include <map>
#include <string>
#include <memory>
#include <chrono>
#include <vector>
#include <optional>
#include <map>
#include <vector>

#include "rclcpp/time.hpp"

#include "mocap_msgs/msg/marker.hpp"
#include "mocap_msgs/msg/markers.hpp"
#include "mocap_msgs/msg/rigid_body.hpp"
#include "mocap_msgs/msg/rigid_bodies.hpp"

#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_interfaces/node_logging.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "tf2/buffer_core.h"
#include "tf2_ros/transform_broadcaster.h"

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>

#include "mocap_control/ControlledLifecycleNode.hpp"

namespace mocap_optitrack_driver
{

class OptitrackDriverNode : public mocap_control::ControlledLifecycleNode
{
public:
  OptitrackDriverNode();
  ~OptitrackDriverNode();

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  bool connect_optitrack();
  bool disconnect_optitrack();
  void set_settings_optitrack();
  bool stop_optitrack();
  void initParameters();

  void process_frame(sFrameOfMocapData * data);

protected:
  void control_start(const mocap_control_msgs::msg::Control::SharedPtr msg) override;
  void control_stop(const mocap_control_msgs::msg::Control::SharedPtr msg) override;

  NatNetClient * client;
  sNatNetClientConnectParams client_params;
  sServerDescription server_description;
  sDataDescriptions * data_descriptions{nullptr};
  sFrameOfMocapData latest_data;
  sRigidBodyData latest_body_frame_data;

  rclcpp_lifecycle::LifecyclePublisher<mocap_msgs::msg::Markers>::SharedPtr mocap_markers_pub_;
  rclcpp_lifecycle::LifecyclePublisher<mocap_msgs::msg::RigidBodies>::SharedPtr
    mocap_rigid_body_pub_;

  std::string connection_type_;
  std::string server_address_;
  std::string local_address_;
  std::string multicast_address_;
  uint16_t server_command_port_;
  uint16_t server_data_port_;

  uint32_t frame_number_{0};
};

void NATNET_CALLCONV process_frame_callback(sFrameOfMocapData * data, void * pUserData);

}  // namespace mocap_optitrack_driver

#endif  // MOCAP_OPTITRACK_DRIVER__MOCAP_OPTITRACK_DRIVER_HPP_
