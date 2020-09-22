/*
 * @brief ros_wrapper encapsulates the ROS API and switches ROS specific implementation
 * between ROS 1 and ROS 2.
 *
 * Copyright (C) 2020 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2020 SICK AG, Waldkirch
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of SICK AG nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission
 *     * Neither the name of Ing.-Buero Dr. Michael Lehning nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Authors:
 *         Michael Lehning <michael.lehning@lehning.de>
 *
 *  Copyright 2020 SICK AG
 *  Copyright 2020 Ing.-Buero Dr. Michael Lehning
 *
 */
#ifndef __SIM_LOC_ROS_WRAPPER_H_INCLUDED
#define __SIM_LOC_ROS_WRAPPER_H_INCLUDED

#include <cfloat>
#include <chrono>
#include <future>
#include <memory>

#if defined __ROS_VERSION && __ROS_VERSION == 1
/*
 * Support for ROS 1 API
 */
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#define RCLCPP_DEBUG_STREAM(logger,msgstream) ROS_DEBUG_STREAM(msgstream)
#define RCLCPP_INFO_STREAM(logger,msgstream)  ROS_INFO_STREAM(msgstream)
#define RCLCPP_WARN_STREAM(logger,msgstream)  ROS_WARN_STREAM(msgstream)
#define RCLCPP_ERROR_STREAM(logger,msgstream) ROS_ERROR_STREAM(msgstream)

// Message header
#include "sick_lidar_localization/SickLocColaTelegramMsg.h"
#include "sick_lidar_localization/SickLocDiagnosticMsg.h"
#include "sick_lidar_localization/SickLocResultPortCrcMsg.h"
#include "sick_lidar_localization/SickLocResultPortHeaderMsg.h"
#include "sick_lidar_localization/SickLocResultPortPayloadMsg.h"
#include "sick_lidar_localization/SickLocResultPortTelegramMsg.h"
#include "sick_lidar_localization/SickLocResultPortTestcaseMsg.h"
// Services supported in release 3 and later
#include "sick_lidar_localization/SickLocColaTelegramSrv.h"
#include "sick_lidar_localization/SickLocIsSystemReadySrv.h"
#include "sick_lidar_localization/SickLocRequestResultDataSrv.h"
#include "sick_lidar_localization/SickLocRequestTimestampSrv.h"
#include "sick_lidar_localization/SickLocSetPoseSrv.h"
#include "sick_lidar_localization/SickLocSetResultEndiannessSrv.h"
#include "sick_lidar_localization/SickLocSetResultModeSrv.h"
#include "sick_lidar_localization/SickLocSetResultPortSrv.h"
#include "sick_lidar_localization/SickLocSetResultPoseEnabledSrv.h"
#include "sick_lidar_localization/SickLocSetResultPoseIntervalSrv.h"
#include "sick_lidar_localization/SickLocStartLocalizingSrv.h"
#include "sick_lidar_localization/SickLocStateSrv.h"
#include "sick_lidar_localization/SickLocStopSrv.h"
#include "sick_lidar_localization/SickLocTimeSyncSrv.h"
// Services supported in release 4 and later
#include "sick_lidar_localization/SickDevGetLidarConfigSrv.h"
#include "sick_lidar_localization/SickDevGetLidarIdentSrv.h"
#include "sick_lidar_localization/SickDevGetLidarStateSrv.h"
#include "sick_lidar_localization/SickDevSetLidarConfigSrv.h"
#include "sick_lidar_localization/SickGetSoftwareVersionSrv.h"
#include "sick_lidar_localization/SickLocAutoStartActiveSrv.h"
#include "sick_lidar_localization/SickLocAutoStartSavePoseIntervalSrv.h"
#include "sick_lidar_localization/SickLocAutoStartSavePoseSrv.h"
#include "sick_lidar_localization/SickLocForceUpdateSrv.h"
#include "sick_lidar_localization/SickLocInitializePoseSrv.h"
#include "sick_lidar_localization/SickLocInitialPoseSrv.h"
#include "sick_lidar_localization/SickLocMapSrv.h"
#include "sick_lidar_localization/SickLocMapStateSrv.h"
#include "sick_lidar_localization/SickLocOdometryActiveSrv.h"
#include "sick_lidar_localization/SickLocOdometryPortSrv.h"
#include "sick_lidar_localization/SickLocOdometryRestrictYMotionSrv.h"
#include "sick_lidar_localization/SickLocReflectorsForSupportActiveSrv.h"
#include "sick_lidar_localization/SickLocResultEndiannessSrv.h"
#include "sick_lidar_localization/SickLocResultModeSrv.h"
#include "sick_lidar_localization/SickLocResultPortSrv.h"
#include "sick_lidar_localization/SickLocResultPoseIntervalSrv.h"
#include "sick_lidar_localization/SickLocResultStateSrv.h"
#include "sick_lidar_localization/SickLocRingBufferRecordingActiveSrv.h"
#include "sick_lidar_localization/SickLocSaveRingBufferRecordingSrv.h"
#include "sick_lidar_localization/SickLocSetAutoStartActiveSrv.h"
#include "sick_lidar_localization/SickLocSetAutoStartSavePoseIntervalSrv.h"
#include "sick_lidar_localization/SickLocSetMapSrv.h"
#include "sick_lidar_localization/SickLocSetOdometryActiveSrv.h"
#include "sick_lidar_localization/SickLocSetOdometryPortSrv.h"
#include "sick_lidar_localization/SickLocSetOdometryRestrictYMotionSrv.h"
#include "sick_lidar_localization/SickLocSetReflectorsForSupportActiveSrv.h"
#include "sick_lidar_localization/SickLocSetRingBufferRecordingActiveSrv.h"
#include "sick_lidar_localization/SickLocStartDemoMappingSrv.h"
#include "sick_lidar_localization/SickReportUserMessageSrv.h"
#include "sick_lidar_localization/SickSavePermanentSrv.h"
#include "sick_lidar_localization/SickDevSetIMUActiveSrv.h"
#include "sick_lidar_localization/SickDevIMUActiveSrv.h"

namespace sick_lidar_localization
{
  typedef sensor_msgs::PointCloud2 PointCloud2Msg;
  typedef ros::Publisher PointCloud2MsgPublisher;

  typedef nav_msgs::Odometry OdomMsg;
  typedef ros::Subscriber OdomMsgSubscriber;

  typedef ros::Publisher SickLocResultPortTelegramMsgPublisher;
  typedef ros::Publisher SickLocDiagnosticMsgPublisher;
  typedef ros::Publisher SickLocResultPortTestcaseMsgPublisher;

  typedef ros::Subscriber SickLocResultPortTelegramMsgSubscriber;
  typedef ros::Subscriber SickLocResultPortTestcaseMsgSubscriber;

  // Service types supported in release 3 and later
  typedef ros::ServiceClient SickLocColaTelegramSrvClient;
  typedef ros::ServiceServer SickLocColaTelegramSrvServer;
  typedef ros::ServiceClient SickLocIsSystemReadySrvClient;
  typedef ros::ServiceServer SickLocIsSystemReadySrvServer;
  typedef ros::ServiceClient SickLocRequestResultDataSrvClient;
  typedef ros::ServiceServer SickLocRequestResultDataSrvServer;
  typedef ros::ServiceClient SickLocRequestTimestampSrvClient;
  typedef ros::ServiceServer SickLocRequestTimestampSrvServer;
  typedef ros::ServiceClient SickLocSetPoseSrvClient;
  typedef ros::ServiceServer SickLocSetPoseSrvServer;
  typedef ros::ServiceClient SickLocSetResultEndiannessSrvClient;
  typedef ros::ServiceServer SickLocSetResultEndiannessSrvServer;
  typedef ros::ServiceClient SickLocSetResultModeSrvClient;
  typedef ros::ServiceServer SickLocSetResultModeSrvServer;
  typedef ros::ServiceClient SickLocSetResultPortSrvClient;
  typedef ros::ServiceServer SickLocSetResultPortSrvServer;
  typedef ros::ServiceClient SickLocSetResultPoseEnabledSrvClient;
  typedef ros::ServiceServer SickLocSetResultPoseEnabledSrvServer;
  typedef ros::ServiceClient SickLocSetResultPoseIntervalSrvClient;
  typedef ros::ServiceServer SickLocSetResultPoseIntervalSrvServer;
  typedef ros::ServiceClient SickLocStartLocalizingSrvClient;
  typedef ros::ServiceServer SickLocStartLocalizingSrvServer;
  typedef ros::ServiceClient SickLocStateSrvClient;
  typedef ros::ServiceServer SickLocStateSrvServer;
  typedef ros::ServiceClient SickLocStopSrvClient;
  typedef ros::ServiceServer SickLocStopSrvServer;
  typedef ros::ServiceClient SickLocTimeSyncSrvClient;
  typedef ros::ServiceServer SickLocTimeSyncSrvServer;
  // Service types supported in release 4 and later
  typedef ros::ServiceClient SickDevGetLidarConfigSrvClient;
  typedef ros::ServiceServer SickDevGetLidarConfigSrvServer;
  typedef ros::ServiceClient SickDevGetLidarIdentSrvClient;
  typedef ros::ServiceServer SickDevGetLidarIdentSrvServer;
  typedef ros::ServiceClient SickDevGetLidarStateSrvClient;
  typedef ros::ServiceServer SickDevGetLidarStateSrvServer;
  typedef ros::ServiceClient SickDevSetLidarConfigSrvClient;
  typedef ros::ServiceServer SickDevSetLidarConfigSrvServer;
  typedef ros::ServiceClient SickGetSoftwareVersionSrvClient;
  typedef ros::ServiceServer SickGetSoftwareVersionSrvServer;
  typedef ros::ServiceClient SickLocAutoStartActiveSrvClient;
  typedef ros::ServiceServer SickLocAutoStartActiveSrvServer;
  typedef ros::ServiceClient SickLocAutoStartSavePoseIntervalSrvClient;
  typedef ros::ServiceServer SickLocAutoStartSavePoseIntervalSrvServer;
  typedef ros::ServiceClient SickLocAutoStartSavePoseSrvClient;
  typedef ros::ServiceServer SickLocAutoStartSavePoseSrvServer;
  typedef ros::ServiceClient SickLocForceUpdateSrvClient;
  typedef ros::ServiceServer SickLocForceUpdateSrvServer;
  typedef ros::ServiceClient SickLocInitializePoseSrvClient;
  typedef ros::ServiceServer SickLocInitializePoseSrvServer;
  typedef ros::ServiceClient SickLocInitialPoseSrvClient;
  typedef ros::ServiceServer SickLocInitialPoseSrvServer;
  typedef ros::ServiceClient SickLocMapSrvClient;
  typedef ros::ServiceServer SickLocMapSrvServer;
  typedef ros::ServiceClient SickLocMapStateSrvClient;
  typedef ros::ServiceServer SickLocMapStateSrvServer;
  typedef ros::ServiceClient SickLocOdometryActiveSrvClient;
  typedef ros::ServiceServer SickLocOdometryActiveSrvServer;
  typedef ros::ServiceClient SickLocOdometryPortSrvClient;
  typedef ros::ServiceServer SickLocOdometryPortSrvServer;
  typedef ros::ServiceClient SickLocOdometryRestrictYMotionSrvClient;
  typedef ros::ServiceServer SickLocOdometryRestrictYMotionSrvServer;
  typedef ros::ServiceClient SickLocReflectorsForSupportActiveSrvClient;
  typedef ros::ServiceServer SickLocReflectorsForSupportActiveSrvServer;
  typedef ros::ServiceClient SickLocResultEndiannessSrvClient;
  typedef ros::ServiceServer SickLocResultEndiannessSrvServer;
  typedef ros::ServiceClient SickLocResultModeSrvClient;
  typedef ros::ServiceServer SickLocResultModeSrvServer;
  typedef ros::ServiceClient SickLocResultPortSrvClient;
  typedef ros::ServiceServer SickLocResultPortSrvServer;
  typedef ros::ServiceClient SickLocResultPoseIntervalSrvClient;
  typedef ros::ServiceServer SickLocResultPoseIntervalSrvServer;
  typedef ros::ServiceClient SickLocResultStateSrvClient;
  typedef ros::ServiceServer SickLocResultStateSrvServer;
  typedef ros::ServiceClient SickLocRingBufferRecordingActiveSrvClient;
  typedef ros::ServiceServer SickLocRingBufferRecordingActiveSrvServer;
  typedef ros::ServiceClient SickLocSaveRingBufferRecordingSrvClient;
  typedef ros::ServiceServer SickLocSaveRingBufferRecordingSrvServer;
  typedef ros::ServiceClient SickLocSetAutoStartActiveSrvClient;
  typedef ros::ServiceServer SickLocSetAutoStartActiveSrvServer;
  typedef ros::ServiceClient SickLocSetAutoStartSavePoseIntervalSrvClient;
  typedef ros::ServiceServer SickLocSetAutoStartSavePoseIntervalSrvServer;
  typedef ros::ServiceClient SickLocSetMapSrvClient;
  typedef ros::ServiceServer SickLocSetMapSrvServer;
  typedef ros::ServiceClient SickLocSetOdometryActiveSrvClient;
  typedef ros::ServiceServer SickLocSetOdometryActiveSrvServer;
  typedef ros::ServiceClient SickLocSetOdometryPortSrvClient;
  typedef ros::ServiceServer SickLocSetOdometryPortSrvServer;
  typedef ros::ServiceClient SickLocSetOdometryRestrictYMotionSrvClient;
  typedef ros::ServiceServer SickLocSetOdometryRestrictYMotionSrvServer;
  typedef ros::ServiceClient SickLocSetReflectorsForSupportActiveSrvClient;
  typedef ros::ServiceServer SickLocSetReflectorsForSupportActiveSrvServer;
  typedef ros::ServiceClient SickLocSetRingBufferRecordingActiveSrvClient;
  typedef ros::ServiceServer SickLocSetRingBufferRecordingActiveSrvServer;
  typedef ros::ServiceClient SickLocStartDemoMappingSrvClient;
  typedef ros::ServiceServer SickLocStartDemoMappingSrvServer;
  typedef ros::ServiceClient SickReportUserMessageSrvClient;
  typedef ros::ServiceServer SickReportUserMessageSrvServer;
  typedef ros::ServiceClient SickSavePermanentSrvClient;
  typedef ros::ServiceServer SickSavePermanentSrvServer;
  typedef ros::ServiceClient SickDevSetIMUActiveSrvClient;
  typedef ros::ServiceServer SickDevSetIMUActiveSrvServer;
  typedef ros::ServiceClient SickDevIMUActiveSrvClient;
  typedef ros::ServiceServer SickDevIMUActiveSrvServer;

} // namespace sick_lidar_localization

namespace ROS
{
  using namespace ros; // map namespace ROS to ros in ROS1 and to rclcpp in ROS2, f.e. ROS::ok() to ROS::ok() in ROS1 and rclcpp::ok() in ROS2
  typedef ros::NodeHandle* NodePtr;

  template<typename T> bool param(ROS::NodePtr & node, const std::string & param_name, T& value, const T& default_value)
  {
    return ros::param::param<T>(param_name, value, default_value);
  }
  
  /** ROS1-/ROS2-compatible shortcut for ros::spin(); */
  void spin(ROS:: NodePtr nh = 0);

} // namespace ROS

#define ROS_CREATE_SRV_CLIENT(nh,srv,name) nh->serviceClient<srv>(name)
#define ROS_CREATE_SRV_SERVER(nh,srv,name,cbfunction,cbobject) nh->advertiseService(name,cbfunction,cbobject)

#define ROS_CREATE_PUBLISHER(nh,msgtype,topic) nh->advertise<msgtype>(topic,1);
#define ROS_PUBLISH(publisher,message) publisher.publish(message);

#define ROS_CREATE_SUBSCRIBER(nh,msgtype,topic,cbfunction,cbobject) nh->subscribe(topic,1,cbfunction,cbobject)

#define NSEC nsec // maps nanoseconds in std_msgs::Header::stamp to stamp.nsec in ROS1 resp. stamp.nanosec in ROS2

#elif defined __ROS_VERSION && __ROS_VERSION == 2
/*
 * Support for ROS 2 API
 */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#define ROS_DEBUG_STREAM(msgstream) RCLCPP_DEBUG_STREAM(rclcpp::get_logger("sick_lidar_localization"),msgstream)
#define ROS_INFO_STREAM(msgstream)  RCLCPP_INFO_STREAM(rclcpp::get_logger("sick_lidar_localization"),msgstream)
#define ROS_WARN_STREAM(msgstream)  RCLCPP_WARN_STREAM(rclcpp::get_logger("sick_lidar_localization"),msgstream)
#define ROS_ERROR_STREAM(msgstream) RCLCPP_ERROR_STREAM(rclcpp::get_logger("sick_lidar_localization"),msgstream)

// Message header
#include "sick_lidar_localization/msg/sick_loc_cola_telegram_msg.hpp"
#include "sick_lidar_localization/msg/sick_loc_diagnostic_msg.hpp"
#include "sick_lidar_localization/msg/sick_loc_result_port_crc_msg.hpp"
#include "sick_lidar_localization/msg/sick_loc_result_port_header_msg.hpp"
#include "sick_lidar_localization/msg/sick_loc_result_port_payload_msg.hpp"
#include "sick_lidar_localization/msg/sick_loc_result_port_telegram_msg.hpp"
#include "sick_lidar_localization/msg/sick_loc_result_port_testcase_msg.hpp"
// Services supported in release 3 and later
#include "sick_lidar_localization/srv/sick_loc_cola_telegram_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_is_system_ready_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_request_result_data_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_request_timestamp_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_set_pose_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_set_result_endianness_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_set_result_mode_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_set_result_port_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_set_result_pose_enabled_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_set_result_pose_interval_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_start_localizing_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_state_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_stop_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_time_sync_srv.hpp"
// Services supported in release 4 and later
#include "sick_lidar_localization/srv/sick_dev_get_lidar_config_srv.hpp"
#include "sick_lidar_localization/srv/sick_dev_get_lidar_ident_srv.hpp"
#include "sick_lidar_localization/srv/sick_dev_get_lidar_state_srv.hpp"
#include "sick_lidar_localization/srv/sick_dev_set_lidar_config_srv.hpp"
#include "sick_lidar_localization/srv/sick_get_software_version_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_auto_start_active_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_auto_start_save_pose_interval_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_auto_start_save_pose_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_force_update_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_initialize_pose_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_initial_pose_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_map_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_map_state_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_odometry_active_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_odometry_port_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_odometry_restrict_y_motion_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_reflectors_for_support_active_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_result_endianness_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_result_mode_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_result_port_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_result_pose_interval_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_result_state_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_ring_buffer_recording_active_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_save_ring_buffer_recording_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_set_auto_start_active_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_set_auto_start_save_pose_interval_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_set_map_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_set_odometry_active_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_set_odometry_port_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_set_odometry_restrict_y_motion_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_set_reflectors_for_support_active_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_set_ring_buffer_recording_active_srv.hpp"
#include "sick_lidar_localization/srv/sick_loc_start_demo_mapping_srv.hpp"
#include "sick_lidar_localization/srv/sick_report_user_message_srv.hpp"
#include "sick_lidar_localization/srv/sick_save_permanent_srv.hpp"
#include "sick_lidar_localization/srv/sick_dev_set_imu_active_srv.hpp"
#include "sick_lidar_localization/srv/sick_dev_imu_active_srv.hpp"

namespace geometry_msgs
{
  using namespace msg; // maps ROS2-namespace geometry_msgs::msg to ROS1-namespace geometry_msgs, f.e. geometry_msgs::Point works on both ROS1 and ROS2
}

namespace sensor_msgs
{
  using namespace msg; // maps ROS2-namespace sensor_msgs::msg to ROS1-namespace sensor_msgs, f.e. sensor_msgs::PointCloud2 works on both ROS1 and ROS2
}

namespace nav_msgs
{
  using namespace msg; // maps ROS2-namespace nav_msgs::msg to ROS1-namespace nav_msgs, f.e. nav_msgs::Odometry works on both ROS1 and ROS2
}

namespace std_msgs
{
  using namespace msg; // maps ROS2-namespace std_msgs::msg to ROS1-namespace std_msgs, f.e. std_msgs::Header works on both ROS1 and ROS2
}

namespace sick_lidar_localization
{
  using namespace msg;
  using namespace srv;

  typedef sensor_msgs::msg::PointCloud2 PointCloud2Msg;
  typedef rclcpp::Publisher<PointCloud2Msg>::SharedPtr PointCloud2MsgPublisher;

  typedef nav_msgs::msg::Odometry OdomMsg;
  typedef rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr OdomMsgSubscriber;

  typedef rclcpp::Publisher<sick_lidar_localization::SickLocResultPortTelegramMsg>::SharedPtr SickLocResultPortTelegramMsgPublisher;
  typedef rclcpp::Publisher<sick_lidar_localization::SickLocDiagnosticMsg>::SharedPtr SickLocDiagnosticMsgPublisher;
  typedef rclcpp::Publisher<sick_lidar_localization::SickLocResultPortTestcaseMsg>::SharedPtr SickLocResultPortTestcaseMsgPublisher;

  typedef rclcpp::Subscription<sick_lidar_localization::SickLocResultPortTelegramMsg>::SharedPtr SickLocResultPortTelegramMsgSubscriber;
  typedef rclcpp::Subscription<sick_lidar_localization::SickLocResultPortTestcaseMsg>::SharedPtr SickLocResultPortTestcaseMsgSubscriber;

  // Service types supported in release 3 and later
  typedef rclcpp::Client<sick_lidar_localization::SickLocColaTelegramSrv>::SharedPtr SickLocColaTelegramSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocColaTelegramSrv>::SharedPtr SickLocColaTelegramSrvServer;
  typedef rclcpp::Client<sick_lidar_localization::SickLocIsSystemReadySrv>::SharedPtr SickLocIsSystemReadySrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocIsSystemReadySrv>::SharedPtr SickLocIsSystemReadySrvServer;
  typedef rclcpp::Client<sick_lidar_localization::SickLocRequestResultDataSrv>::SharedPtr SickLocRequestResultDataSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocRequestResultDataSrv>::SharedPtr SickLocRequestResultDataSrvServer;
  typedef rclcpp::Client<sick_lidar_localization::SickLocRequestTimestampSrv>::SharedPtr SickLocRequestTimestampSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocRequestTimestampSrv>::SharedPtr SickLocRequestTimestampSrvServer;
  typedef rclcpp::Client<sick_lidar_localization::SickLocSetPoseSrv>::SharedPtr SickLocSetPoseSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocSetPoseSrv>::SharedPtr SickLocSetPoseSrvServer;
  typedef rclcpp::Client<sick_lidar_localization::SickLocSetResultEndiannessSrv>::SharedPtr SickLocSetResultEndiannessSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocSetResultEndiannessSrv>::SharedPtr SickLocSetResultEndiannessSrvServer;
  typedef rclcpp::Client<sick_lidar_localization::SickLocSetResultModeSrv>::SharedPtr SickLocSetResultModeSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocSetResultModeSrv>::SharedPtr SickLocSetResultModeSrvServer;
  typedef rclcpp::Client<sick_lidar_localization::SickLocSetResultPortSrv>::SharedPtr SickLocSetResultPortSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocSetResultPortSrv>::SharedPtr SickLocSetResultPortSrvServer;
  typedef rclcpp::Client<sick_lidar_localization::SickLocSetResultPoseEnabledSrv>::SharedPtr SickLocSetResultPoseEnabledSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocSetResultPoseEnabledSrv>::SharedPtr SickLocSetResultPoseEnabledSrvServer;
  typedef rclcpp::Client<sick_lidar_localization::SickLocSetResultPoseIntervalSrv>::SharedPtr SickLocSetResultPoseIntervalSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocSetResultPoseIntervalSrv>::SharedPtr SickLocSetResultPoseIntervalSrvServer;
  typedef rclcpp::Client<sick_lidar_localization::SickLocStartLocalizingSrv>::SharedPtr SickLocStartLocalizingSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocStartLocalizingSrv>::SharedPtr SickLocStartLocalizingSrvServer;
  typedef rclcpp::Client<sick_lidar_localization::SickLocStateSrv>::SharedPtr SickLocStateSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocStateSrv>::SharedPtr SickLocStateSrvServer;
  typedef rclcpp::Client<sick_lidar_localization::SickLocStopSrv>::SharedPtr SickLocStopSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocStopSrv>::SharedPtr SickLocStopSrvServer;
  typedef rclcpp::Client<sick_lidar_localization::SickLocTimeSyncSrv>::SharedPtr SickLocTimeSyncSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocTimeSyncSrv>::SharedPtr SickLocTimeSyncSrvServer;
  // Service types supported in release 4 and later
  typedef rclcpp::Client <sick_lidar_localization::SickDevGetLidarConfigSrv>::SharedPtr SickDevGetLidarConfigSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickDevGetLidarConfigSrv>::SharedPtr SickDevGetLidarConfigSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickDevGetLidarIdentSrv>::SharedPtr SickDevGetLidarIdentSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickDevGetLidarIdentSrv>::SharedPtr SickDevGetLidarIdentSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickDevGetLidarStateSrv>::SharedPtr SickDevGetLidarStateSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickDevGetLidarStateSrv>::SharedPtr SickDevGetLidarStateSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickDevSetLidarConfigSrv>::SharedPtr SickDevSetLidarConfigSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickDevSetLidarConfigSrv>::SharedPtr SickDevSetLidarConfigSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickGetSoftwareVersionSrv>::SharedPtr SickGetSoftwareVersionSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickGetSoftwareVersionSrv>::SharedPtr SickGetSoftwareVersionSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocAutoStartActiveSrv>::SharedPtr SickLocAutoStartActiveSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocAutoStartActiveSrv>::SharedPtr SickLocAutoStartActiveSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocAutoStartSavePoseIntervalSrv>::SharedPtr SickLocAutoStartSavePoseIntervalSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocAutoStartSavePoseIntervalSrv>::SharedPtr SickLocAutoStartSavePoseIntervalSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocAutoStartSavePoseSrv>::SharedPtr SickLocAutoStartSavePoseSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocAutoStartSavePoseSrv>::SharedPtr SickLocAutoStartSavePoseSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocForceUpdateSrv>::SharedPtr SickLocForceUpdateSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocForceUpdateSrv>::SharedPtr SickLocForceUpdateSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocInitializePoseSrv>::SharedPtr SickLocInitializePoseSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocInitializePoseSrv>::SharedPtr SickLocInitializePoseSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocInitialPoseSrv>::SharedPtr SickLocInitialPoseSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocInitialPoseSrv>::SharedPtr SickLocInitialPoseSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocMapSrv>::SharedPtr SickLocMapSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocMapSrv>::SharedPtr SickLocMapSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocMapStateSrv>::SharedPtr SickLocMapStateSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocMapStateSrv>::SharedPtr SickLocMapStateSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocOdometryActiveSrv>::SharedPtr SickLocOdometryActiveSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocOdometryActiveSrv>::SharedPtr SickLocOdometryActiveSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocOdometryPortSrv>::SharedPtr SickLocOdometryPortSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocOdometryPortSrv>::SharedPtr SickLocOdometryPortSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocOdometryRestrictYMotionSrv>::SharedPtr SickLocOdometryRestrictYMotionSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocOdometryRestrictYMotionSrv>::SharedPtr SickLocOdometryRestrictYMotionSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocReflectorsForSupportActiveSrv>::SharedPtr SickLocReflectorsForSupportActiveSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocReflectorsForSupportActiveSrv>::SharedPtr SickLocReflectorsForSupportActiveSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocResultEndiannessSrv>::SharedPtr SickLocResultEndiannessSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocResultEndiannessSrv>::SharedPtr SickLocResultEndiannessSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocResultModeSrv>::SharedPtr SickLocResultModeSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocResultModeSrv>::SharedPtr SickLocResultModeSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocResultPortSrv>::SharedPtr SickLocResultPortSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocResultPortSrv>::SharedPtr SickLocResultPortSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocResultPoseIntervalSrv>::SharedPtr SickLocResultPoseIntervalSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocResultPoseIntervalSrv>::SharedPtr SickLocResultPoseIntervalSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocResultStateSrv>::SharedPtr SickLocResultStateSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocResultStateSrv>::SharedPtr SickLocResultStateSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocRingBufferRecordingActiveSrv>::SharedPtr SickLocRingBufferRecordingActiveSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocRingBufferRecordingActiveSrv>::SharedPtr SickLocRingBufferRecordingActiveSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocSaveRingBufferRecordingSrv>::SharedPtr SickLocSaveRingBufferRecordingSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocSaveRingBufferRecordingSrv>::SharedPtr SickLocSaveRingBufferRecordingSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocSetAutoStartActiveSrv>::SharedPtr SickLocSetAutoStartActiveSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocSetAutoStartActiveSrv>::SharedPtr SickLocSetAutoStartActiveSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocSetAutoStartSavePoseIntervalSrv>::SharedPtr SickLocSetAutoStartSavePoseIntervalSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocSetAutoStartSavePoseIntervalSrv>::SharedPtr SickLocSetAutoStartSavePoseIntervalSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocSetMapSrv>::SharedPtr SickLocSetMapSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocSetMapSrv>::SharedPtr SickLocSetMapSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocSetOdometryActiveSrv>::SharedPtr SickLocSetOdometryActiveSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocSetOdometryActiveSrv>::SharedPtr SickLocSetOdometryActiveSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocSetOdometryPortSrv>::SharedPtr SickLocSetOdometryPortSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocSetOdometryPortSrv>::SharedPtr SickLocSetOdometryPortSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocSetOdometryRestrictYMotionSrv>::SharedPtr SickLocSetOdometryRestrictYMotionSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocSetOdometryRestrictYMotionSrv>::SharedPtr SickLocSetOdometryRestrictYMotionSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocSetReflectorsForSupportActiveSrv>::SharedPtr SickLocSetReflectorsForSupportActiveSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocSetReflectorsForSupportActiveSrv>::SharedPtr SickLocSetReflectorsForSupportActiveSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocSetRingBufferRecordingActiveSrv>::SharedPtr SickLocSetRingBufferRecordingActiveSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocSetRingBufferRecordingActiveSrv>::SharedPtr SickLocSetRingBufferRecordingActiveSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickLocStartDemoMappingSrv>::SharedPtr SickLocStartDemoMappingSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickLocStartDemoMappingSrv>::SharedPtr SickLocStartDemoMappingSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickReportUserMessageSrv>::SharedPtr SickReportUserMessageSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickReportUserMessageSrv>::SharedPtr SickReportUserMessageSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickSavePermanentSrv>::SharedPtr SickSavePermanentSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickSavePermanentSrv>::SharedPtr SickSavePermanentSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickDevSetIMUActiveSrv>::SharedPtr SickDevSetIMUActiveSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickDevSetIMUActiveSrv>::SharedPtr SickDevSetIMUActiveSrvServer;
  typedef rclcpp::Client <sick_lidar_localization::SickDevIMUActiveSrv>::SharedPtr SickDevIMUActiveSrvClient;
  typedef rclcpp::Service<sick_lidar_localization::SickDevIMUActiveSrv>::SharedPtr SickDevIMUActiveSrvServer;

} // namespace sick_lidar_localization

namespace ROS
{
  using namespace rclcpp; // map namespace ROS to ros in ROS1 and to rclcpp in ROS2, f.e. ROS::ok() to ros::ok() in ROS1 and rclcpp::ok() in ROS2
  typedef rclcpp::Node::SharedPtr NodePtr;

  template<typename T> bool param(ROS::NodePtr & node, const std::string & param_name, T& value, const T& default_value)
  {
    ROS_INFO_STREAM("ROS::param(" << node->get_name() << "," << param_name << "," << default_value << ")");
    try
    {
      if(!node->has_parameter(param_name))
      {
        node->declare_parameter<T>(param_name, default_value);
      }
    }
    catch(const std::exception& exc)
    {
      ROS_WARN_STREAM("## ERROR ROS::param: declare_parameter(" << param_name << "," << default_value << ") failed, exception " << exc.what());
    }
    try
    {
      return node->get_parameter(param_name, value);
    }
    catch(const std::exception& exc)
    {
      ROS_WARN_STREAM("## ERROR ROS::param: get_parameter(" << param_name << "," << default_value << ") failed, exception " << exc.what());
    }
    return false;
  }

  /** ROS1-/ROS2-compatible shortcut for rclcpp::init(argc, argv); */
  void init(int argc, char** argv, const std::string nodename = "");

} // namespace ROS

#define ROS_CREATE_SRV_CLIENT(nh,srv,name) nh->create_client<srv>(name)
#define ROS_CREATE_SRV_SERVER(nh,srv,name,cbfunction,cbobject) nh->create_service<srv>(name,std::bind(cbfunction,cbobject,std::placeholders::_1,std::placeholders::_2))

#define ROS_CREATE_PUBLISHER(nh,msgtype,topic) nh->create_publisher<msgtype>(topic,rclcpp::SystemDefaultsQoS());
#define ROS_PUBLISH(publisher,message) publisher->publish(message);

#define ROS_CREATE_SUBSCRIBER(nh,msgtype,topic,cbfunction,cbobject) nh->create_subscription<msgtype>(topic,10,std::bind(cbfunction,cbobject,std::placeholders::_1))

#define NSEC nanosec // maps nanoseconds in std_msgs::Header::stamp to stamp.nsec in ROS1 resp. stamp.nanosec in ROS2

#else
#error __ROS_VERSION not defined, build with "--cmake-args -DROS_VERSION=1" or "--cmake-args -DROS_VERSION=2"
#endif

namespace ROS
{
  /** Creates a new ros node, shortcut for new ros::NodeHandle() on ROS1 resp. rclcpp::Node::make_shared(node_name) on ROS2 */
  ROS::NodePtr createNode(const std::string& node_name = "sick_lidar_localization");

  /** Deletes a ros node */
  void deleteNode(ROS::NodePtr & node);

  /** Shortcut to replace ros::Duration(seconds).sleep() by std::thread */
  void sleep(double seconds);

  /** Shortcut to return ros::Time::now() on ROS1 resp. rclcpp::Clock::now() on ROS2 */
  ROS::Time now(void);

  /** Splits a ROS::Duration into seconds and nanoseconds part */
  void splitTime(ROS::Duration time, uint32_t& seconds, uint32_t& nanoseconds);

  /** Splits a ROS::Time into seconds and nanoseconds part */
  void splitTime(ROS::Time time, uint32_t& seconds, uint32_t& nanoseconds);

  /** Shortcut to return ros::Time(msg_header->stamp.sec,msg_header->stamp.nsec) on ROS1 resp. rclcpp::Time(msg_header->stamp.sec,msg_header->stamp.nanosec) on ROS2 */
  ROS::Time timeFromHeader(const std_msgs::Header* msg_header);

  /** Returns a time (type ros::Time on ROS1 resp. rclcpp::Time on ROS2) from a given amount of seconds.
   ** Note: ros::Time(1) initializes 1 second, while rclcpp::Time(1) initializes 1 nanosecond.
   ** Do not use the Time constructor with one parameter! Always use ROS::timeFromSec(seconds)
   ** or ROS::Time(int32_t seconds, uint32_t nanoseconds). */
  ROS::Time timeFromSec(double seconds);

  /** Returns a duration (type ros::Duration on ROS1 resp. rclcpp::Duration on ROS2) from a given amount of seconds.
   ** Note: ros::Duration(1) initializes 1 second, while rclcpp::Duration(1) initializes 1 nanosecond.
   ** Do not use the Duration constructor with one parameter! Always use ROS::durationFromSec(seconds)
   ** or ROS::Duration(int32_t seconds, uint32_t nanoseconds). */
  ROS::Duration durationFromSec(double seconds);
  
  /** Shortcut to return ros::Duration::toSec() on ROS1 resp. rclcpp::Duration::seconds() on ROS2 */
  double seconds(ROS::Duration duration);

  /** Shortcut to return the time in seconds since program start (first node started) */
  double secondsSinceStart(const ROS::Time& time);

  /** Shortcut to return the timestamp in milliseconds from ROS1 resp. ROS2 time */
  uint64_t timestampMilliseconds(const ROS::Time& time);

} // namespace ROS

#endif // __SIM_LOC_ROS_WRAPPER_H_INCLUDED
