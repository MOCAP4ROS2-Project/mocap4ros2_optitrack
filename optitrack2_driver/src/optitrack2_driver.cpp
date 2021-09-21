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

#include "optitrack2_driver/optitrack2_driver.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using std::min;
using std::max;
using std::string;
using std::map;
using std::stringstream;

// The Optitrack driver node has differents parameters to initialized with the optitrack2_driver_params.yaml
OptitrackDriverNode::OptitrackDriverNode(const rclcpp::NodeOptions node_options)
        : device_control::ControlledLifecycleNode(static_cast<string>("optitrack2_driver_node"))
{
    declare_parameter<std::string>("connection_type", "Unicast");
    declare_parameter<std::string>("server_address", "000.000.000.000");
    declare_parameter<std::string>("local_address", "000.000.000.000");
    declare_parameter<std::string>("multicast_address", "000.000.000.000");
    declare_parameter<uint16_t>("server_command_port", 0);
    declare_parameter<uint16_t>("server_data_port", 0);
    declare_parameter<std::string>("rigid_body_name", "mav1");
    declare_parameter<int>("lastFrameNumber", 0);
    declare_parameter<int>("frameCount", 0);
    declare_parameter<int>("droppedFrameCount", 0);
    declare_parameter<std::string>("qos_history_policy", "keep_all");
    declare_parameter<std::string>("qos_reliability_policy", "best_effort");
    declare_parameter<int>("qos_depth", 10);
    client = new NatNetClient();
    client->SetFrameReceivedCallback(&OptitrackDriverNode::process_frame_callback, this);	// this function will receive data from the server
}

// In charge of choose the different driver options related and provided by the Optitrack SDK
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


    /*
    if (publish_markers_) {
      marker_pub_ = create_publisher<mocap4ros_msgs::msg::Markers>(
        tracked_frame_suffix_ + "/markers", 100);
      RCLCPP_WARN(get_logger(), "publish_markers_ configured!!!");
    }
    */
}

// Stop the optitrack_driver_node if the lifecycle node state is shutdown.
bool OptitrackDriverNode::stop_optitrack()
{
    RCLCPP_INFO(get_logger(), "Disconnecting from optitrack DataStream SDK");
    void* response;
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

// In charge of the transition of the lifecycle node
void OptitrackDriverNode::control_start(const device_control_msgs::msg::Control::SharedPtr msg) {
    trigger_transition(rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));
}

// In charge of the transition of the lifecycle node
void OptitrackDriverNode::control_stop(const device_control_msgs::msg::Control::SharedPtr msg) {
    trigger_transition(rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));
}

void NATNET_CALLCONV OptitrackDriverNode::process_frame_callback(sFrameOfMocapData* data, void* pUserData)
{
    return reinterpret_cast<OptitrackDriverNode*>(pUserData)->process_frame(*data);
}

// In charge of get the Optitrack information and convert it to optitrack_msgs
void OptitrackDriverNode::process_frame(sFrameOfMocapData data)
{
    latest_data = data;

    static rclcpp::Time lastTime;
    int OutputFrameNum = data.iFrame;


    int frameDiff = 0;
    if (lastFrameNumber_ != 0) {
        frameDiff = OutputFrameNum - lastFrameNumber_;
        frameCount_ += frameDiff;
        if ((frameDiff) > 1) {
            droppedFrameCount_ += frameDiff - 1;
            auto droppedFramePct = static_cast<double>((double)droppedFrameCount_ / (double)frameCount_ * 100.0);

            RCLCPP_DEBUG(
                    get_logger(),
                    "%d more (total %d / %d, %f %%) frame(s) dropped. Consider adjusting rates",
                    frameDiff, droppedFrameCount_, frameCount_, droppedFramePct);
        }
    }
    lastFrameNumber_ = OutputFrameNum;

    if (frameDiff != 0) {
        const uint64_t softwareLatencyHostTicks = data.TransmitTimestamp - data.CameraDataReceivedTimestamp;
//        auto softwareLatencyNano = static_cast<rcl_duration_value_t>(((double)softwareLatencyHostTicks * 1000000000.0) / (double)(server_description.HighResClockFrequency));

//        rclcpp::Duration optitrack_latency(softwareLatencyNano);
        get_latest_body_frame_data();
        now_time = this->get_clock()->now();
        process_rigid_body(now_time, lastFrameNumber_);
        lastTime = now_time;
    }
}

void OptitrackDriverNode::get_latest_body_frame_data()
{
    if (rigid_body_id_ != -1)
    {
        for(int i=0; i < latest_data.nRigidBodies; i++) {
            if (latest_data.RigidBodies[i].ID == rigid_body_id_) {
                latest_body_frame_data = latest_data.RigidBodies[i];
            }
        }
    }
}

void OptitrackDriverNode::process_rigid_body(const rclcpp::Time & frame_time, unsigned int optitrack_frame_num)
{
    int marker_cnt = 0;

    geometry_msgs::msg::PoseStamped poseStamped;

    poseStamped.header.frame_id = std::to_string(rigid_body_id_);
    poseStamped.header.stamp = frame_time;
    poseStamped.pose.position.set__x(latest_body_frame_data.x);
    poseStamped.pose.position.set__y(latest_body_frame_data.y);
    poseStamped.pose.position.set__z(latest_body_frame_data.z);
    poseStamped.pose.orientation.set__w(latest_body_frame_data.qw);
    poseStamped.pose.orientation.set__x(latest_body_frame_data.qx);
    poseStamped.pose.orientation.set__y(latest_body_frame_data.qy);
    poseStamped.pose.orientation.set__z(latest_body_frame_data.qz);

    pose_pub_->publish(poseStamped);
}

using CallbackReturnT =
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


// The next Callbacks are used to manage behavior in the different states of the lifecycle node.
CallbackReturnT
OptitrackDriverNode::on_configure(const rclcpp_lifecycle::State &)
{
    initParameters();

    RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
    RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());

    auto rmw_qos_history_policy = name_to_history_policy_map.find(qos_history_policy_);
    auto rmw_qos_reliability_policy = name_to_reliability_policy_map.find(qos_reliability_policy_);
    auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(
                    // The history policy determines how messages are saved until taken by
                    // the reader.
                    // KEEP_ALL saves all messages until they are taken.
                    // KEEP_LAST enforces a limit on the number of messages that are saved,
                    // specified by the "depth" parameter.
                    rmw_qos_history_policy->second,
                    // Depth represents how many messages to store in history when the
                    // history policy is KEEP_LAST.
                    qos_depth_
            ));
    // The reliability policy can be reliable, meaning that the underlying transport layer will try
    // ensure that every message gets received in order, or best effort, meaning that the transport
    // makes no guarantees about the order or reliability of delivery.
    qos.reliability(rmw_qos_reliability_policy->second);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
            "/optitrack2_driver/change_state");

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/optitrack2_driver/" + rigid_body_name_ + "/PoseStamped", 100);

    update_pub_ = create_publisher<std_msgs::msg::Empty>(
            "/optitrack2_driver/update_notify", qos);

    RCLCPP_INFO(get_logger(), "Configured!\n");

    return CallbackReturnT::SUCCESS;
}

CallbackReturnT
OptitrackDriverNode::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
    RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
    update_pub_->on_activate();
    pose_pub_->on_activate();
    connect_optitrack();
    RCLCPP_INFO(get_logger(), "Activated!\n");

    return CallbackReturnT::SUCCESS;
}

CallbackReturnT
OptitrackDriverNode::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
    RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
    update_pub_->on_deactivate();
    pose_pub_->on_deactivate();
    RCLCPP_INFO(get_logger(), "Deactivated!\n");

    return CallbackReturnT::SUCCESS;
}

CallbackReturnT
OptitrackDriverNode::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
    RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
    /* Clean up stuff */
    RCLCPP_INFO(get_logger(), "Cleaned up!\n");

    return CallbackReturnT::SUCCESS;
}

CallbackReturnT
OptitrackDriverNode::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
    RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());
    /* Shut down stuff */
    RCLCPP_INFO(get_logger(), "Shutted down!\n");

    return CallbackReturnT::SUCCESS;
}

CallbackReturnT
OptitrackDriverNode::on_error(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
    RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());

    return CallbackReturnT::SUCCESS;
}

// In charge of find and connect the driver with the optitrack SDK.
bool OptitrackDriverNode::connect_optitrack()
{
    RCLCPP_WARN(
            get_logger(),
            "Trying to connect to Optitrack NatNET SDK at %s ...", server_address_.c_str());

    client->Disconnect();
    set_settings_optitrack();

    if (client->Connect(client_params) == ErrorCode::ErrorCode_OK) {
        RCLCPP_INFO(get_logger(), "... connected!");

        memset(&server_description, 0, sizeof(server_description));
        client->GetServerDescription(&server_description);
        if(!server_description.HostPresent)
        {
            RCLCPP_DEBUG(get_logger(),"Unable to connect to server. Host not present.");
            return false;
        }

        if (client->GetDataDescriptionList(&data_descriptions) != ErrorCode_OK || !data_descriptions)
        {
            RCLCPP_DEBUG(get_logger(),"[Client] Unable to retrieve Data Descriptions.\n");
        }

        for(int i=0; i < data_descriptions->nDataDescriptions; i++) {
            if (data_descriptions->arrDataDescriptions[i].type == Descriptor_RigidBody) {
                if (strcmp(data_descriptions->arrDataDescriptions[i].Data.RigidBodyDescription->szName,rigid_body_name_.c_str()) == 0){
                    rigid_body_id_ = data_descriptions->arrDataDescriptions[i].Data.RigidBodyDescription->ID;
                }
            }
        }

        RCLCPP_INFO(get_logger(),"\n[Client] Server application info:\n");
        RCLCPP_INFO(get_logger(),"Application: %s (ver. %d.%d.%d.%d)\n", server_description.szHostApp, server_description.HostAppVersion[0],
               server_description.HostAppVersion[1], server_description.HostAppVersion[2], server_description.HostAppVersion[3]);
        RCLCPP_INFO(get_logger(),"NatNet Version: %d.%d.%d.%d\n", server_description.NatNetVersion[0], server_description.NatNetVersion[1],
               server_description.NatNetVersion[2], server_description.NatNetVersion[3]);
        RCLCPP_INFO(get_logger(),"Client IP:%s\n", client_params.localAddress );
        RCLCPP_INFO(get_logger(),"Server IP:%s\n", client_params.serverAddress );
        RCLCPP_INFO(get_logger(),"Server Name:%s\n", server_description.szHostComputerName);

        void* pResult;
        int nBytes = 0;

        if ( client->SendMessageAndWait("FrameRate", &pResult, &nBytes) == ErrorCode_OK)
        {
            float fRate = *((float*)pResult);
            RCLCPP_INFO(get_logger(),"Mocap Framerate : %3.2f\n", fRate);
        }
        else
            RCLCPP_DEBUG(get_logger(),"Error getting frame rate.\n");

    } else {
        RCLCPP_INFO(get_logger(), "... not connected :( ");
        return false;
    }
}

// Init the necessary parameters to use the optitrack SDK.
void OptitrackDriverNode::initParameters()
{
    get_parameter<std::string>("connection_type", connection_type_);
    get_parameter<std::string>("server_address", server_address_);
    get_parameter<std::string>("local_address", local_address_);
    get_parameter<std::string>("multicast_address", multicast_address_);
    get_parameter<uint16_t>("server_command_port", server_command_port_);
    get_parameter<uint16_t>("server_data_port", server_data_port_);
    get_parameter<std::string>("rigid_body_name", rigid_body_name_);
    get_parameter<int>("lastFrameNumber", lastFrameNumber_);
    get_parameter<int>("frameCount", frameCount_);
    get_parameter<int>("droppedFrameCount", droppedFrameCount_);
    get_parameter<std::string>("qos_history_policy", qos_history_policy_);
    get_parameter<std::string>("qos_reliability_policy", qos_reliability_policy_);
    get_parameter<int>("qos_depth", qos_depth_);


    RCLCPP_INFO(
            get_logger(),
            "Param connection_type: %s", connection_type_.c_str());
    RCLCPP_INFO(
            get_logger(),
            "Param server_address: %s", server_address_.c_str());
    RCLCPP_INFO(
            get_logger(),
            "Param local_address: %s", local_address_.c_str());
    RCLCPP_INFO(
            get_logger(),
            "Param multicast_address: %i", multicast_address_.c_str());
    RCLCPP_INFO(
            get_logger(),
            "Param server_command_port: %i", server_command_port_);
    RCLCPP_INFO(
            get_logger(),
            "Param server_data_port: %i", server_data_port_);
    RCLCPP_INFO(
            get_logger(),
            "Param rigid_body_name: %s", rigid_body_name_.c_str());
    RCLCPP_INFO(
            get_logger(),
            "Param lastFrameNumber: %d", lastFrameNumber_);
    RCLCPP_INFO(
            get_logger(),
            "Param frameCount: %d", frameCount_);
    RCLCPP_INFO(
            get_logger(),
            "Param droppedFrameCount: %d", droppedFrameCount_);
    RCLCPP_INFO(
            get_logger(),
            "Param qos_history_policy: %s", qos_history_policy_.c_str());
    RCLCPP_INFO(
            get_logger(),
            "Param qos_reliability_policy: %s", qos_reliability_policy_.c_str());
    RCLCPP_INFO(
            get_logger(),
            "Param qos_depth: %d", qos_depth_);
}