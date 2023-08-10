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

#include <string>
#include <vector>
#include <memory>

#include "mocap_msgs/msg/marker.hpp"
#include "mocap_msgs/msg/markers.hpp"

#include "mocap_optitrack_driver/mocap_optitrack_driver.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace mocap_optitrack_driver
{

using std::placeholders::_1;
using std::placeholders::_2;

OptitrackDriverNode::OptitrackDriverNode()
: ControlledLifecycleNode("mocap_optitrack_driver_node", 
                            rclcpp::NodeOptions()
                              .allow_undeclared_parameters(true)
                              .automatically_declare_parameters_from_overrides(true))
{
  // declare_parameter<std::string>("connection_type", "Unicast");
  // declare_parameter<std::string>("server_address", "000.000.000.000");
  // declare_parameter<std::string>("local_address", "000.000.000.000");
  // declare_parameter<std::string>("multicast_address", "000.000.000.000");
  // declare_parameter<uint16_t>("server_command_port", 0);
  // declare_parameter<uint16_t>("server_data_port", 0);

  client = new NatNetClient();
  client->SetFrameReceivedCallback(process_frame_callback, this);
}

OptitrackDriverNode::~OptitrackDriverNode()
{
}

void OptitrackDriverNode::set_settings_optitrack()
{
  if (connection_type_ == "Multicast") {
    client_params.connectionType = ConnectionType::ConnectionType_Multicast;
    client_params.multicastAddress = multicast_address_.c_str();
  } else if (connection_type_ == "Unicast") {
    client_params.connectionType = ConnectionType::ConnectionType_Unicast;
  } else {
    RCLCPP_FATAL(get_logger(), "Unknown connection type -- options are Multicast, Unicast");
    rclcpp::shutdown();
  }

  client_params.serverAddress = server_address_.c_str();
  client_params.localAddress = local_address_.c_str();
  client_params.serverCommandPort = server_command_port_;
  client_params.serverDataPort = server_data_port_;
}

void OptitrackDriverNode::update_rigid_bodies(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  
  if (data_descriptions) {
      NatNet_FreeDescriptions(data_descriptions);
  }
  RCLCPP_INFO(get_logger(), "Requesting Data Descriptions for Rigid Bodies");
  
  int result = client->GetDataDescriptionList(&data_descriptions);
  if (result != ErrorCode::ErrorCode_OK) {
    response->success = false;
    return;
  }
  // populate rigid body id map
  update_rigid_body_id_map();
  if (publish_tf_) {
    get_rigid_bodies_from_params();
  }
  response->success = true;
}

void OptitrackDriverNode::update_rigid_body_id_map()
{
  // RCLCPP_INFO(get_logger(), "updating rigid body id map");
  for (int i = 0; i < data_descriptions->nDataDescriptions; ++i) {
    if (data_descriptions->arrDataDescriptions[i].type == DataDescriptors::Descriptor_RigidBody) {
      sRigidBodyDescription* rigid_body = data_descriptions->arrDataDescriptions[i].Data.RigidBodyDescription;
      id_rigid_body_map[rigid_body->ID] = rigid_body->szName;
      rigid_body_id_map[rigid_body->szName] = rigid_body->ID;
      // RCLCPP_INFO_STREAM(get_logger(), "Rigid Body: " << rigid_body->szName << " ID: " << rigid_body->ID);
    }
  }
}

void OptitrackDriverNode::get_rigid_bodies_from_params()
{
  // RCLCPP_INFO(get_logger(), "getting parent frame");
  const std::vector<std::string> prefix{"rigid_bodies"};
  const std::string name_with_dot = std::string(this->get_name()) + ".";
  const auto result = this->get_node_parameters_interface()->list_parameters(
    prefix, 0
  );

  for (const std::string & prefix : result.prefixes)
  {
    // RCLCPP_INFO_STREAM(get_logger(), "Reading : " << prefix);
    std::string temp_name;
    if (!get_parameter<std::string>(prefix + ".name", temp_name)) {
      RCLCPP_WARN_STREAM(get_logger(), "Unable to find 'name' sub-parameter in: " << prefix);
      continue;
    }
    if (rigid_body_id_map.count(temp_name) > 0) {
      // RCLCPP_INFO_STREAM(get_logger(), temp_name << " is a rigid body");
      tf_frames_to_publish.insert(temp_name);
    }
    else {
      RCLCPP_WARN_STREAM(get_logger(), "Unable to find '" << temp_name << "' on NatNet server. Check Assets tab in Motive.");
    }
  }
  RCLCPP_INFO(get_logger(), "Able to publish tf of the following rigid bodies:");
  for (const auto & str : tf_frames_to_publish) {
    RCLCPP_INFO_STREAM(get_logger(), "\t" << str);
  }
}

bool OptitrackDriverNode::stop_optitrack()
{
  RCLCPP_INFO(get_logger(), "Disconnecting from optitrack DataStream SDK");

  return true;
}

void
OptitrackDriverNode::control_start(const mocap_control_msgs::msg::Control::SharedPtr msg)
{
  (void)msg;
  trigger_transition(
    rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));
}

void
OptitrackDriverNode::control_stop(const mocap_control_msgs::msg::Control::SharedPtr msg)
{
  (void)msg;
  trigger_transition(
    rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));
}

void NATNET_CALLCONV process_frame_callback(sFrameOfMocapData * data, void * pUserData)
{
  static_cast<OptitrackDriverNode *>(pUserData)->process_frame(data);
}

void
OptitrackDriverNode::process_frame(sFrameOfMocapData * data)
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  frame_number_++;
  std::map<int, std::vector<mocap_msgs::msg::Marker>> marker2rb;

  // Markers
  if (mocap_markers_pub_->get_subscription_count() > 0) {
    mocap_msgs::msg::Markers msg;
    msg.header.frame_id = parent_frame_name_;
    msg.header.stamp = now();
    msg.frame_number = frame_number_;

    for (int i = 0; i < data->nLabeledMarkers; i++) {
      bool Unlabeled = ((data->LabeledMarkers[i].params & 0x10) != 0);
      bool ActiveMarker = ((data->LabeledMarkers[i].params & 0x20) != 0);
      sMarker & marker_data = data->LabeledMarkers[i];
      int modelID, markerID;
      NatNet_DecodeID(marker_data.ID, &modelID, &markerID);

      mocap_msgs::msg::Marker marker;
      marker.id_type = mocap_msgs::msg::Marker::USE_INDEX;
      marker.marker_index = i;
      marker.translation.x = marker_data.x;
      marker.translation.y = marker_data.y;
      marker.translation.z = marker_data.z;
      if (ActiveMarker || Unlabeled) {
        msg.markers.push_back(marker);
      } else {
        marker2rb[modelID].push_back(marker);
      }
    }
    mocap_markers_pub_->publish(msg);
  }

  if (mocap_rigid_body_pub_->get_subscription_count() > 0) {
    mocap_msgs::msg::RigidBodies msg_rb;
    msg_rb.header.frame_id = "map";
    msg_rb.header.stamp = now();
    msg_rb.frame_number = frame_number_;

    for (int i = 0; i < data->nRigidBodies; i++) {
      mocap_msgs::msg::RigidBody rb;

      rb.rigid_body_name = std::to_string(data->RigidBodies[i].ID);
      rb.pose.position.x = data->RigidBodies[i].x;
      rb.pose.position.y = data->RigidBodies[i].y;
      rb.pose.position.z = data->RigidBodies[i].z;
      rb.pose.orientation.x = data->RigidBodies[i].qx;
      rb.pose.orientation.y = data->RigidBodies[i].qy;
      rb.pose.orientation.z = data->RigidBodies[i].qz;
      rb.pose.orientation.w = data->RigidBodies[i].qw;
      rb.markers = marker2rb[data->RigidBodies[i].ID];

      msg_rb.rigidbodies.push_back(rb);
    }

    mocap_rigid_body_pub_->publish(msg_rb);
  }
  
  if (publish_tf_) {
    publish_tf_data(data);
  }
}

void OptitrackDriverNode::publish_tf_data(sFrameOfMocapData * data)
{
  for (int i = 0; i < data->nRigidBodies; i++) {
    if (id_rigid_body_map.count(data->RigidBodies[i].ID) > 0 && 
        tf_frames_to_publish.count(id_rigid_body_map[data->RigidBodies[i].ID]) > 0) {
      geometry_msgs::msg::TransformStamped t;

      t.header.stamp = this->get_clock()->now(); //TODO: get this time from NatNet
      t.header.frame_id = parent_frame_name_;
      t.child_frame_id = id_rigid_body_map[data->RigidBodies[i].ID];

      t.transform.translation.x = data->RigidBodies[i].x;
      t.transform.translation.y = data->RigidBodies[i].y;
      t.transform.translation.z = data->RigidBodies[i].z;
      t.transform.rotation.x = data->RigidBodies[i].qx;
      t.transform.rotation.y = data->RigidBodies[i].qy;
      t.transform.rotation.z = data->RigidBodies[i].qz;
      t.transform.rotation.w = data->RigidBodies[i].qw;

      tf_broadcaster_->sendTransform(t);
    }

  }
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


// The next Callbacks are used to manage behavior in the different states of the lifecycle node.
CallbackReturnT
OptitrackDriverNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  initParameters();

  mocap_markers_pub_ = create_publisher<mocap_msgs::msg::Markers>(
    "markers", rclcpp::QoS(1000));
  mocap_rigid_body_pub_ = create_publisher<mocap_msgs::msg::RigidBodies>(
    "rigid_bodies", rclcpp::QoS(1000));
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  std::string node_name(this->get_name());
  
  update_rigid_bodies_srv_ = this->create_service<std_srvs::srv::Trigger>(
    node_name + "/update_rigid_bodies", 
    std::bind(&OptitrackDriverNode::update_rigid_bodies, this, 
      std::placeholders::_1, std::placeholders::_2));

  connect_optitrack();
  
  update_rigid_body_id_map();
  if (publish_tf_) {
    get_rigid_bodies_from_params();
  }

  RCLCPP_INFO(get_logger(), "Configured!\n");

  return ControlledLifecycleNode::on_configure(state);
}

CallbackReturnT
OptitrackDriverNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  mocap_markers_pub_->on_activate();
  mocap_rigid_body_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "Activated!\n");

  return ControlledLifecycleNode::on_activate(state);
}

CallbackReturnT
OptitrackDriverNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  mocap_markers_pub_->on_deactivate();
  mocap_rigid_body_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "Deactivated!\n");

  return ControlledLifecycleNode::on_deactivate(state);
}

CallbackReturnT
OptitrackDriverNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Cleaned up!\n");

  if (disconnect_optitrack()) {
    return ControlledLifecycleNode::on_cleanup(state);
  } else {
    return CallbackReturnT::FAILURE;
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
OptitrackDriverNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Shutted down!\n");

  if (disconnect_optitrack()) {
    return ControlledLifecycleNode::on_shutdown(state);
  } else {
    return CallbackReturnT::FAILURE;
  }
}

CallbackReturnT
OptitrackDriverNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());

  disconnect_optitrack();

  return ControlledLifecycleNode::on_error(state);
}

bool
OptitrackDriverNode::connect_optitrack()
{
  RCLCPP_INFO(
    get_logger(),
    "Trying to connect to Optitrack NatNET SDK at %s ...", server_address_.c_str());

  client->Disconnect();
  set_settings_optitrack();

  if (client->Connect(client_params) == ErrorCode::ErrorCode_OK) {
    RCLCPP_INFO(get_logger(), "... connected!");

    memset(&server_description, 0, sizeof(server_description));
    client->GetServerDescription(&server_description);
    if (!server_description.HostPresent) {
      RCLCPP_DEBUG(get_logger(), "Unable to connect to server. Host not present.");
      return false;
    }

    if (client->GetDataDescriptionList(&data_descriptions) != ErrorCode_OK || !data_descriptions) {
      RCLCPP_WARN(get_logger(), "[Client] Unable to retrieve Data Descriptions.\n");
    }

    RCLCPP_INFO(get_logger(), "\n[Client] Server application info:\n");
    RCLCPP_INFO(
      get_logger(), "Application: %s (ver. %d.%d.%d.%d)\n",
      server_description.szHostApp, server_description.HostAppVersion[0],
      server_description.HostAppVersion[1], server_description.HostAppVersion[2],
      server_description.HostAppVersion[3]);
    RCLCPP_INFO(
      get_logger(), "NatNet Version: %d.%d.%d.%d\n", server_description.NatNetVersion[0],
      server_description.NatNetVersion[1],
      server_description.NatNetVersion[2], server_description.NatNetVersion[3]);
    RCLCPP_INFO(get_logger(), "Client IP:%s\n", client_params.localAddress);
    RCLCPP_INFO(get_logger(), "Server IP:%s\n", client_params.serverAddress);
    RCLCPP_INFO(get_logger(), "Server Name:%s\n", server_description.szHostComputerName);

    void * pResult;
    int nBytes = 0;

    if (client->SendMessageAndWait("FrameRate", &pResult, &nBytes) == ErrorCode_OK) {
      float fRate = *(static_cast<float *>(pResult));
      RCLCPP_INFO(get_logger(), "Mocap Framerate : %3.2f\n", fRate);
    } else {
      RCLCPP_DEBUG(get_logger(), "Error getting frame rate.\n");
    }
  } else {
    RCLCPP_INFO(get_logger(), "... not connected :( ");
    return false;
  }

  return true;
}

bool
OptitrackDriverNode::disconnect_optitrack()
{
  void * response;
  int nBytes;
  if (client->SendMessageAndWait("Disconnect", &response, &nBytes) == ErrorCode_OK) {
    client->Disconnect();
    RCLCPP_INFO(get_logger(), "[Client] Disconnected");
    return true;
  } else {
    RCLCPP_ERROR(get_logger(), "[Client] Disconnect not successful..");
    return false;
  }
}

void
OptitrackDriverNode::initParameters()
{
  bool valid_params = true;
  valid_params &= get_parameter<std::string>("connection_type", connection_type_);
  valid_params &= get_parameter<std::string>("server_address", server_address_);
  valid_params &= get_parameter<std::string>("local_address", local_address_);
  valid_params &= get_parameter<std::string>("multicast_address", multicast_address_);
  valid_params &= get_parameter<uint16_t>("server_command_port", server_command_port_);
  valid_params &= get_parameter<uint16_t>("server_data_port", server_data_port_);

  if (!valid_params) {
    RCLCPP_ERROR(get_logger(), "Not all required Parameters were set");
  }

  if (!get_parameter<bool>("publish_tf", publish_tf_)) {
    publish_tf_ = false;
  }
  if (publish_tf_) {
    get_parameter<std::string>("parent_frame_name", parent_frame_name_);
  }
}

}  // namespace mocap_optitrack_driver
