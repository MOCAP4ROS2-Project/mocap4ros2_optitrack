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
// Author: David Vargas Frutos <david.vargas@urjc.es>

#ifndef OPTITRACK2_DRIVER__OPTITRACK2_DRIVER_HPP_
#define OPTITRACK2_DRIVER__OPTITRACK2_DRIVER_HPP_

#include <iostream>
#include <sstream>
#include <map>
#include <string>
#include <memory>
#include <chrono>
#include <vector>
#include <optional>

#include "rclcpp/time.hpp"

#include "mocap_msgs/msg/marker.hpp"
#include "mocap_msgs/msg/markers.hpp"
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

#include "device_control/ControlledLifecycleNode.hpp"


class OptitrackDriverNode : public device_control::ControlledLifecycleNode
{
public:
    explicit OptitrackDriverNode(
            const rclcpp::NodeOptions options =
            rclcpp::NodeOptions().parameter_overrides(
                    std::vector<rclcpp::Parameter> {
                            rclcpp::Parameter("use_sim_time", true)
                    }));

    using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
    CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
    CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
    CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
    CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
    CallbackReturnT on_error(const rclcpp_lifecycle::State & state);
    bool connect_optitrack();
    void set_settings_optitrack();
    bool stop_optitrack();
    void initParameters();

protected:
    NatNetClient client;
    sNatNetClientConnectParams client_params;
    sServerDescription server_description;
    sDataDescriptions* data_descriptions{nullptr};
    sFrameOfMocapData latest_data;
    sRigidBodyData latest_body_frame_data;
    // std::shared_ptr<rclcpp::SyncParametersClient> parameters_client;
    rclcpp::Time now_time;
    std::string myParam;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string connection_type_;
    std::string server_address_;
    std::string local_address_;
    std::string multicast_address_;
    uint16_t server_command_port_;
    uint16_t server_data_port_;
    std::string rigid_body_name_;
    int32_t rigid_body_id_{-1};

    int lastFrameNumber_;
    int frameCount_;
    int droppedFrameCount_;
    std::string qos_history_policy_;
    std::string qos_reliability_policy_;
    int qos_depth_;

    static void NATNET_CALLCONV process_frame_callback(sFrameOfMocapData* data, void* pUserData);
    void process_frame(sFrameOfMocapData data);
    void process_rigid_body(const rclcpp::Time & frame_time, unsigned int optitrack_frame_num);

    void control_start(const device_control_msgs::msg::Control::SharedPtr msg) override;
    void control_stop(const device_control_msgs::msg::Control::SharedPtr msg) override;
    void get_latest_body_frame_data();

    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>::SharedPtr update_pub_;
};

static
std::map<std::string, rmw_qos_reliability_policy_t> name_to_reliability_policy_map = {
        {"reliable", RMW_QOS_POLICY_RELIABILITY_RELIABLE},
        {"best_effort", RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT}
};

static
std::map<std::string, rmw_qos_history_policy_t> name_to_history_policy_map = {
        {"keep_last", RMW_QOS_POLICY_HISTORY_KEEP_LAST},
        {"keep_all", RMW_QOS_POLICY_HISTORY_KEEP_ALL}
};

#endif  // OPTITRACK2_DRIVER__OPTITRACK2_DRIVER_HPP_