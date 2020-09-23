/*
 * @brief cola_services implements the following ros services for sick_lidar_localization:
 *
 * ROS service name               | Definition file                         | Cola request telegram                                | Description
 * "SickLocIsSystemReady"         | srv/SickLocIsSystemReadySrv.srv         | "sMN IsSystemReady"                                  | Check if the system is ready
 * "SickLocState"                 | srv/SickLocStateSrv.srv                 | "sRN LocState"                                       | Read localization state
 * "SickLocStartLocalizing"       | srv/SickLocStartLocalizingSrv.srv       | "sMN LocStartLocalizing"                             | Start localization
 * "SickLocStop"                  | srv/SickLocStopSrv.srv                  | "sMN LocStop"                                        | Stop localization or demo mapping
 * "SickLocSetResultPort"         | srv/SickLocSetResultPortSrv.srv         | "sMN LocSetResultPort <port>"                        | Set TCP-port for result output (default: 2201)
 * "SickLocSetResultMode"         | srv/SickLocSetResultModeSrv.srv         | "sMN LocSetResultMode <mode>"                        | Set mode of result output (default: stream)
 * "SickLocSetResultPoseEnabled"  | srv/SickLocSetResultPoseEnabledSrv.srv  | "sMN LocSetResultPoseEnabled <enabled>"              | Disable/enable result output
 * "SickLocSetResultEndianness"   | srv/SickLocSetResultEndiannessSrv.srv   | "sMN LocSetResultEndianness <endianness>"            | Set endianness of result output
 * "SickLocSetResultPoseInterval" | srv/SickLocSetResultPoseIntervalSrv.srv | "sMN LocSetResultPoseInterval <interval>"            | Set interval of result output
 * "SickLocRequestResultData"     | srv/SickLocRequestResultDataSrv.srv     | "sMN LocRequestResultData"                           | If in poll mode, trigger sending the localization result of the next processed scan via TCP interface.
 * "SickLocSetPose"               | srv/SickLocSetPoseSrv.srv               | "sMN LocSetPose <posex> <posey> <yaw> <uncertainty>" | Initialize vehicle pose
 *
 * See Telegram-Listing-v1.1.0.241R.pdf for further details about Cola telegrams.
 *
 * Copyright (C) 2019 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2019 SICK AG, Waldkirch
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
 *  Copyright 2019 SICK AG
 *  Copyright 2019 Ing.-Buero Dr. Michael Lehning
 *
 */
#include "sick_lidar_localization/ros_wrapper.h"
#include "sick_lidar_localization/cola_encoder.h"
#include "sick_lidar_localization/cola_services.h"

/*!
 * Constructor
 */
sick_lidar_localization::ColaServices::ColaServices(ROS::NodePtr nh, sick_lidar_localization::DriverMonitor* driver_monitor) : m_nh(nh), m_driver_monitor(driver_monitor), m_cola_response_timeout(10.0)
{
  if(nh)
  {
    ROS::param<double>(nh, "/sick_lidar_localization/time_sync/cola_response_timeout", m_cola_response_timeout, m_cola_response_timeout);

    // Advertise ros services supported by release 3 and later
#if defined __ROS_VERSION && __ROS_VERSION == 1
    m_srv_server_01 = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocIsSystemReadySrv, "SickLocIsSystemReady",&sick_lidar_localization::ColaServices::serviceCbIsSystemReady, this);
    m_srv_server_02 = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocStateSrv, "SickLocState",&sick_lidar_localization::ColaServices::serviceCbLocState, this);
    m_srv_server_03 = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocStartLocalizingSrv, "SickLocStartLocalizing",&sick_lidar_localization::ColaServices::serviceCbLocStartLocalizing, this);
    m_srv_server_04 = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocStopSrv, "SickLocStop",&sick_lidar_localization::ColaServices::serviceCbLocStop, this);
    m_srv_server_06 = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetResultPortSrv, "SickLocSetResultPort",&sick_lidar_localization::ColaServices::serviceCbLocSetResultPort, this);
    m_srv_server_07 = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetResultModeSrv, "SickLocSetResultMode",&sick_lidar_localization::ColaServices::serviceCbLocSetResultMode, this);
    m_srv_server_08 = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetResultPoseEnabledSrv, "SickLocSetResultPoseEnabled",&sick_lidar_localization::ColaServices::serviceCbLocSetResultPoseEnabled, this);
    m_srv_server_09 = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetResultEndiannessSrv, "SickLocSetResultEndianness",&sick_lidar_localization::ColaServices::serviceCbLocSetResultEndianness, this);
    m_srv_server_10 = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetResultPoseIntervalSrv, "SickLocSetResultPoseInterval",&sick_lidar_localization::ColaServices::serviceCbLocSetResultPoseInterval, this);
    m_srv_server_11 = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocRequestResultDataSrv, "SickLocRequestResultData",&sick_lidar_localization::ColaServices::serviceCbLocRequestResultData, this);
    m_srv_server_12 = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetPoseSrv, "SickLocSetPose",&sick_lidar_localization::ColaServices::serviceCbLocSetPose, this);
#elif defined __ROS_VERSION && __ROS_VERSION == 2
    m_srv_server_01 = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocIsSystemReadySrv, "SickLocIsSystemReady",&sick_lidar_localization::ColaServices::serviceCbIsSystemReadyROS2, this);
    m_srv_server_02 = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocStateSrv, "SickLocState",&sick_lidar_localization::ColaServices::serviceCbLocStateROS2, this);
    m_srv_server_03 = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocStartLocalizingSrv, "SickLocStartLocalizing",&sick_lidar_localization::ColaServices::serviceCbLocStartLocalizingROS2, this);
    m_srv_server_04 = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocStopSrv, "SickLocStop",&sick_lidar_localization::ColaServices::serviceCbLocStopROS2, this);
    m_srv_server_06 = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetResultPortSrv, "SickLocSetResultPort",&sick_lidar_localization::ColaServices::serviceCbLocSetResultPortROS2, this);
    m_srv_server_07 = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetResultModeSrv, "SickLocSetResultMode",&sick_lidar_localization::ColaServices::serviceCbLocSetResultModeROS2, this);
    m_srv_server_08 = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetResultPoseEnabledSrv, "SickLocSetResultPoseEnabled",&sick_lidar_localization::ColaServices::serviceCbLocSetResultPoseEnabledROS2, this);
    m_srv_server_09 = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetResultEndiannessSrv, "SickLocSetResultEndianness",&sick_lidar_localization::ColaServices::serviceCbLocSetResultEndiannessROS2, this);
    m_srv_server_10 = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetResultPoseIntervalSrv, "SickLocSetResultPoseInterval",&sick_lidar_localization::ColaServices::serviceCbLocSetResultPoseIntervalROS2, this);
    m_srv_server_11 = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocRequestResultDataSrv, "SickLocRequestResultData",&sick_lidar_localization::ColaServices::serviceCbLocRequestResultDataROS2, this);
    m_srv_server_12 = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetPoseSrv, "SickLocSetPose",&sick_lidar_localization::ColaServices::serviceCbLocSetPoseROS2, this);
#endif
    // Advertise ros services supported by release 4 and later
#if defined __ROS_VERSION && __ROS_VERSION == 1
     m_SickDevSetLidarConfigSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickDevSetLidarConfigSrv, "SickDevSetLidarConfig", &sick_lidar_localization::ColaServices::serviceCbDevSetLidarConfig, this);
     m_SickDevGetLidarConfigSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickDevGetLidarConfigSrv, "SickDevGetLidarConfig", &sick_lidar_localization::ColaServices::serviceCbDevGetLidarConfig, this);
     m_SickLocSetMapSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetMapSrv, "SickLocSetMap", &sick_lidar_localization::ColaServices::serviceCbLocSetMap, this);
     m_SickLocMapSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocMapSrv, "SickLocMap", &sick_lidar_localization::ColaServices::serviceCbLocMap, this);
     m_SickLocMapStateSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocMapStateSrv, "SickLocMapState", &sick_lidar_localization::ColaServices::serviceCbLocMapState, this);
     m_SickLocInitializePoseSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocInitializePoseSrv, "SickLocInitializePose", &sick_lidar_localization::ColaServices::serviceCbLocInitializePose, this);
     m_SickLocInitialPoseSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocInitialPoseSrv, "SickLocInitialPose", &sick_lidar_localization::ColaServices::serviceCbLocInitialPose, this);
     m_SickLocSetReflectorsForSupportActiveSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetReflectorsForSupportActiveSrv, "SickLocSetReflectorsForSupportActive", &sick_lidar_localization::ColaServices::serviceCbLocSetReflectorsForSupportActive, this);
     m_SickLocReflectorsForSupportActiveSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocReflectorsForSupportActiveSrv, "SickLocReflectorsForSupportActive", &sick_lidar_localization::ColaServices::serviceCbLocReflectorsForSupportActive, this);
     m_SickLocSetOdometryActiveSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetOdometryActiveSrv, "SickLocSetOdometryActive", &sick_lidar_localization::ColaServices::serviceCbLocSetOdometryActive, this);
     m_SickLocOdometryActiveSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocOdometryActiveSrv, "SickLocOdometryActive", &sick_lidar_localization::ColaServices::serviceCbLocOdometryActive, this);
     m_SickLocSetOdometryPortSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetOdometryPortSrv, "SickLocSetOdometryPort", &sick_lidar_localization::ColaServices::serviceCbLocSetOdometryPort, this);
     m_SickLocOdometryPortSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocOdometryPortSrv, "SickLocOdometryPort", &sick_lidar_localization::ColaServices::serviceCbLocOdometryPort, this);
     m_SickLocSetOdometryRestrictYMotionSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetOdometryRestrictYMotionSrv, "SickLocSetOdometryRestrictYMotion", &sick_lidar_localization::ColaServices::serviceCbLocSetOdometryRestrictYMotion, this);
     m_SickLocOdometryRestrictYMotionSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocOdometryRestrictYMotionSrv, "SickLocOdometryRestrictYMotion", &sick_lidar_localization::ColaServices::serviceCbLocOdometryRestrictYMotion, this);
     m_SickLocSetAutoStartActiveSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetAutoStartActiveSrv, "SickLocSetAutoStartActive", &sick_lidar_localization::ColaServices::serviceCbLocSetAutoStartActive, this);
     m_SickLocAutoStartActiveSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocAutoStartActiveSrv, "SickLocAutoStartActive", &sick_lidar_localization::ColaServices::serviceCbLocAutoStartActive, this);
     m_SickLocSetAutoStartSavePoseIntervalSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetAutoStartSavePoseIntervalSrv, "SickLocSetAutoStartSavePoseInterval", &sick_lidar_localization::ColaServices::serviceCbLocSetAutoStartSavePoseInterval, this);
     m_SickLocAutoStartSavePoseIntervalSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocAutoStartSavePoseIntervalSrv, "SickLocAutoStartSavePoseInterval", &sick_lidar_localization::ColaServices::serviceCbLocAutoStartSavePoseInterval, this);
     m_SickLocSetRingBufferRecordingActiveSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetRingBufferRecordingActiveSrv, "SickLocSetRingBufferRecordingActive", &sick_lidar_localization::ColaServices::serviceCbLocSetRingBufferRecordingActive, this);
     m_SickLocRingBufferRecordingActiveSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocRingBufferRecordingActiveSrv, "SickLocRingBufferRecordingActive", &sick_lidar_localization::ColaServices::serviceCbLocRingBufferRecordingActive, this);
     m_SickDevGetLidarIdentSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickDevGetLidarIdentSrv, "SickDevGetLidarIdent", &sick_lidar_localization::ColaServices::serviceCbDevGetLidarIdent, this);
     m_SickDevGetLidarStateSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickDevGetLidarStateSrv, "SickDevGetLidarState", &sick_lidar_localization::ColaServices::serviceCbDevGetLidarState, this);
     m_SickGetSoftwareVersionSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickGetSoftwareVersionSrv, "SickGetSoftwareVersion", &sick_lidar_localization::ColaServices::serviceCbGetSoftwareVersion, this);
     m_SickLocAutoStartSavePoseSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocAutoStartSavePoseSrv, "SickLocAutoStartSavePose", &sick_lidar_localization::ColaServices::serviceCbLocAutoStartSavePose, this);
     m_SickLocForceUpdateSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocForceUpdateSrv, "SickLocForceUpdate", &sick_lidar_localization::ColaServices::serviceCbLocForceUpdate, this);
     m_SickLocSaveRingBufferRecordingSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSaveRingBufferRecordingSrv, "SickLocSaveRingBufferRecording", &sick_lidar_localization::ColaServices::serviceCbLocSaveRingBufferRecording, this);
     m_SickLocStartDemoMappingSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocStartDemoMappingSrv, "SickLocStartDemoMapping", &sick_lidar_localization::ColaServices::serviceCbLocStartDemoMapping, this);
     m_SickReportUserMessageSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickReportUserMessageSrv, "SickReportUserMessage", &sick_lidar_localization::ColaServices::serviceCbReportUserMessage, this);
     m_SickSavePermanentSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickSavePermanentSrv, "SickSavePermanent", &sick_lidar_localization::ColaServices::serviceCbSavePermanent, this);
     m_SickLocResultPortSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocResultPortSrv, "SickLocResultPort", &sick_lidar_localization::ColaServices::serviceCbLocResultPort, this);
     m_SickLocResultModeSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocResultModeSrv, "SickLocResultMode", &sick_lidar_localization::ColaServices::serviceCbLocResultMode, this);
     m_SickLocResultEndiannessSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocResultEndiannessSrv, "SickLocResultEndianness", &sick_lidar_localization::ColaServices::serviceCbLocResultEndianness, this);
     m_SickLocResultStateSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocResultStateSrv, "SickLocResultState", &sick_lidar_localization::ColaServices::serviceCbLocResultState, this);
     m_SickLocResultPoseIntervalSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocResultPoseIntervalSrv, "SickLocResultPoseInterval", &sick_lidar_localization::ColaServices::serviceCbLocResultPoseInterval, this);
     m_SickDevSetIMUActiveSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickDevSetIMUActiveSrv, "SickDevSetIMUActive", &sick_lidar_localization::ColaServices::serviceCbDevSetIMUActive, this);
     m_SickDevIMUActiveSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickDevIMUActiveSrv, "SickDevIMUActive", &sick_lidar_localization::ColaServices::serviceCbDevIMUActive, this);
#elif defined __ROS_VERSION && __ROS_VERSION == 2
     m_SickDevSetLidarConfigSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickDevSetLidarConfigSrv, "SickDevSetLidarConfig", &sick_lidar_localization::ColaServices::serviceCbDevSetLidarConfigROS2, this);
     m_SickDevGetLidarConfigSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickDevGetLidarConfigSrv, "SickDevGetLidarConfig", &sick_lidar_localization::ColaServices::serviceCbDevGetLidarConfigROS2, this);
     m_SickLocSetMapSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetMapSrv, "SickLocSetMap", &sick_lidar_localization::ColaServices::serviceCbLocSetMapROS2, this);
     m_SickLocMapSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocMapSrv, "SickLocMap", &sick_lidar_localization::ColaServices::serviceCbLocMapROS2, this);
     m_SickLocMapStateSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocMapStateSrv, "SickLocMapState", &sick_lidar_localization::ColaServices::serviceCbLocMapStateROS2, this);
     m_SickLocInitializePoseSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocInitializePoseSrv, "SickLocInitializePose", &sick_lidar_localization::ColaServices::serviceCbLocInitializePoseROS2, this);
     m_SickLocInitialPoseSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocInitialPoseSrv, "SickLocInitialPose", &sick_lidar_localization::ColaServices::serviceCbLocInitialPoseROS2, this);
     m_SickLocSetReflectorsForSupportActiveSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetReflectorsForSupportActiveSrv, "SickLocSetReflectorsForSupportActive", &sick_lidar_localization::ColaServices::serviceCbLocSetReflectorsForSupportActiveROS2, this);
     m_SickLocReflectorsForSupportActiveSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocReflectorsForSupportActiveSrv, "SickLocReflectorsForSupportActive", &sick_lidar_localization::ColaServices::serviceCbLocReflectorsForSupportActiveROS2, this);
     m_SickLocSetOdometryActiveSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetOdometryActiveSrv, "SickLocSetOdometryActive", &sick_lidar_localization::ColaServices::serviceCbLocSetOdometryActiveROS2, this);
     m_SickLocOdometryActiveSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocOdometryActiveSrv, "SickLocOdometryActive", &sick_lidar_localization::ColaServices::serviceCbLocOdometryActiveROS2, this);
     m_SickLocSetOdometryPortSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetOdometryPortSrv, "SickLocSetOdometryPort", &sick_lidar_localization::ColaServices::serviceCbLocSetOdometryPortROS2, this);
     m_SickLocOdometryPortSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocOdometryPortSrv, "SickLocOdometryPort", &sick_lidar_localization::ColaServices::serviceCbLocOdometryPortROS2, this);
     m_SickLocSetOdometryRestrictYMotionSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetOdometryRestrictYMotionSrv, "SickLocSetOdometryRestrictYMotion", &sick_lidar_localization::ColaServices::serviceCbLocSetOdometryRestrictYMotionROS2, this);
     m_SickLocOdometryRestrictYMotionSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocOdometryRestrictYMotionSrv, "SickLocOdometryRestrictYMotion", &sick_lidar_localization::ColaServices::serviceCbLocOdometryRestrictYMotionROS2, this);
     m_SickLocSetAutoStartActiveSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetAutoStartActiveSrv, "SickLocSetAutoStartActive", &sick_lidar_localization::ColaServices::serviceCbLocSetAutoStartActiveROS2, this);
     m_SickLocAutoStartActiveSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocAutoStartActiveSrv, "SickLocAutoStartActive", &sick_lidar_localization::ColaServices::serviceCbLocAutoStartActiveROS2, this);
     m_SickLocSetAutoStartSavePoseIntervalSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetAutoStartSavePoseIntervalSrv, "SickLocSetAutoStartSavePoseInterval", &sick_lidar_localization::ColaServices::serviceCbLocSetAutoStartSavePoseIntervalROS2, this);
     m_SickLocAutoStartSavePoseIntervalSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocAutoStartSavePoseIntervalSrv, "SickLocAutoStartSavePoseInterval", &sick_lidar_localization::ColaServices::serviceCbLocAutoStartSavePoseIntervalROS2, this);
     m_SickLocSetRingBufferRecordingActiveSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSetRingBufferRecordingActiveSrv, "SickLocSetRingBufferRecordingActive", &sick_lidar_localization::ColaServices::serviceCbLocSetRingBufferRecordingActiveROS2, this);
     m_SickLocRingBufferRecordingActiveSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocRingBufferRecordingActiveSrv, "SickLocRingBufferRecordingActive", &sick_lidar_localization::ColaServices::serviceCbLocRingBufferRecordingActiveROS2, this);
     m_SickDevGetLidarIdentSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickDevGetLidarIdentSrv, "SickDevGetLidarIdent", &sick_lidar_localization::ColaServices::serviceCbDevGetLidarIdentROS2, this);
     m_SickDevGetLidarStateSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickDevGetLidarStateSrv, "SickDevGetLidarState", &sick_lidar_localization::ColaServices::serviceCbDevGetLidarStateROS2, this);
     m_SickGetSoftwareVersionSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickGetSoftwareVersionSrv, "SickGetSoftwareVersion", &sick_lidar_localization::ColaServices::serviceCbGetSoftwareVersionROS2, this);
     m_SickLocAutoStartSavePoseSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocAutoStartSavePoseSrv, "SickLocAutoStartSavePose", &sick_lidar_localization::ColaServices::serviceCbLocAutoStartSavePoseROS2, this);
     m_SickLocForceUpdateSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocForceUpdateSrv, "SickLocForceUpdate", &sick_lidar_localization::ColaServices::serviceCbLocForceUpdateROS2, this);
     m_SickLocSaveRingBufferRecordingSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocSaveRingBufferRecordingSrv, "SickLocSaveRingBufferRecording", &sick_lidar_localization::ColaServices::serviceCbLocSaveRingBufferRecordingROS2, this);
     m_SickLocStartDemoMappingSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocStartDemoMappingSrv, "SickLocStartDemoMapping", &sick_lidar_localization::ColaServices::serviceCbLocStartDemoMappingROS2, this);
     m_SickReportUserMessageSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickReportUserMessageSrv, "SickReportUserMessage", &sick_lidar_localization::ColaServices::serviceCbReportUserMessageROS2, this);
     m_SickSavePermanentSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickSavePermanentSrv, "SickSavePermanent", &sick_lidar_localization::ColaServices::serviceCbSavePermanentROS2, this);
     m_SickLocResultPortSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocResultPortSrv, "SickLocResultPort", &sick_lidar_localization::ColaServices::serviceCbLocResultPortROS2, this);
     m_SickLocResultModeSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocResultModeSrv, "SickLocResultMode", &sick_lidar_localization::ColaServices::serviceCbLocResultModeROS2, this);
     m_SickLocResultEndiannessSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocResultEndiannessSrv, "SickLocResultEndianness", &sick_lidar_localization::ColaServices::serviceCbLocResultEndiannessROS2, this);
     m_SickLocResultStateSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocResultStateSrv, "SickLocResultState", &sick_lidar_localization::ColaServices::serviceCbLocResultStateROS2, this);
     m_SickLocResultPoseIntervalSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocResultPoseIntervalSrv, "SickLocResultPoseInterval", &sick_lidar_localization::ColaServices::serviceCbLocResultPoseIntervalROS2, this);
     m_SickDevSetIMUActiveSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickDevSetIMUActiveSrv, "SickDevSetIMUActive", &sick_lidar_localization::ColaServices::serviceCbDevSetIMUActiveROS2, this);
     m_SickDevIMUActiveSrvServer = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickDevIMUActiveSrv, "SickDevIMUActive", &sick_lidar_localization::ColaServices::serviceCbDevIMUActiveROS2, this);
#endif // __ROS_VERSION
    // Clients for ros services "SickLocColaTelegram"
    m_service_client = ROS_CREATE_SRV_CLIENT(nh, sick_lidar_localization::SickLocColaTelegramSrv, "SickLocColaTelegram");
  }
}

/*!
 * Destructor
 */
sick_lidar_localization::ColaServices::~ColaServices()
{

}

/*!
 * Sends a cola telegram using ros service "SickLocColaTelegram", receives and returns the response from localization controller
 * @param[in] cola_ascii_request request (Cola-ASCII, f.e. "sMN IsSystemReady")
 * @return response from localization controller
 */
sick_lidar_localization::SickLocColaTelegramMsg sick_lidar_localization::ColaServices::sendColaTelegram(const std::string & cola_ascii_request)
{
  boost::lock_guard<boost::mutex> service_cb_lockguard(m_service_cb_mutex); // one service request at a time
#if defined __ROS_VERSION && __ROS_VERSION == 1
  sick_lidar_localization::SickLocColaTelegramSrv cola_telegram;
  sick_lidar_localization::SickLocColaTelegramSrv::Request* cola_telegram_request = &cola_telegram.request;
  sick_lidar_localization::SickLocColaTelegramSrv::Response* cola_telegram_response = &cola_telegram.response;
#elif defined __ROS_VERSION && __ROS_VERSION == 2
  std::shared_ptr<sick_lidar_localization::SickLocColaTelegramSrv::Request> cola_telegram_request = std::make_shared<sick_lidar_localization::SickLocColaTelegramSrv::Request>();
  std::shared_ptr<sick_lidar_localization::SickLocColaTelegramSrv::Response> cola_telegram_response = std::make_shared<sick_lidar_localization::SickLocColaTelegramSrv::Response>();
#endif
  cola_telegram_request->cola_ascii_request = cola_ascii_request;
  cola_telegram_request->wait_response_timeout = m_cola_response_timeout;
  try
  {
    ROS::Time start_request_timestamp = ROS::now();
    if(m_driver_monitor == 0)
    {
      ROS_ERROR_STREAM("## ERROR ColaServices::sendColaTelegram(): ColaServices not initialized (driver_monitor == 0, " << __FILE__ << ":" << __LINE__ << ")");
      return sick_lidar_localization::SickLocColaTelegramMsg();
    }
    bool service_call_ok = m_driver_monitor->serviceCbColaTelegram(*cola_telegram_request, *cola_telegram_response);
    if (!service_call_ok || cola_telegram_response->cola_ascii_response.empty())
    {
       ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(): calling ros service \"SickLocColaTelegram\" failed with request: \""
        << cola_telegram_request->cola_ascii_request << "\", response: \"" << cola_telegram_response->cola_ascii_response << "\"" << " after "
        << ROS::seconds(ROS::now() - start_request_timestamp) << " sec (timeout: " << m_cola_response_timeout << " sec, " << __FILE__ << ":" << __LINE__ << ")");
      return sick_lidar_localization::SickLocColaTelegramMsg();
    }
    ROS_DEBUG_STREAM("ColaServices::sendColaTelegram(): request \"" << cola_telegram_request->cola_ascii_request
      << "\", response: \"" << cola_telegram_response->cola_ascii_response << "\" succesfull.");
    // Decode and return response
    return sick_lidar_localization::ColaParser::decodeColaTelegram(cola_telegram_response->cola_ascii_response);
  }
  catch(const std::exception & exc)
  {
    ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_telegram_request->cola_ascii_request << ") failed, exception " << exc.what());
  }
  return sick_lidar_localization::SickLocColaTelegramMsg();
}

/*!
 * Callback for service messages (SickLocIsSystemReadySrv, Check if the system is ready).
 * Sends a cola telegram "sMN IsSystemReady" and receives the response from localization controller
 * using ros service "SickLocColaTelegramSrv".
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors.
 */
bool sick_lidar_localization::ColaServices::serviceCbIsSystemReady(
  sick_lidar_localization::SickLocIsSystemReadySrv::Request &service_request,
  sick_lidar_localization::SickLocIsSystemReadySrv::Response &service_response)
{
  return serviceCbWithSuccessResponse("sMN", "IsSystemReady", "", service_response);
}

/*!
 * Callback for service messages (SickLocStateSrv, Read localization state).
 * Sends a cola telegram "sRN LocState" and receives the response from localization controller
 * using ros service "SickLocColaTelegramSrv".
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors.
 */
bool sick_lidar_localization::ColaServices::serviceCbLocState(
  sick_lidar_localization::SickLocStateSrv::Request &service_request,
  sick_lidar_localization::SickLocStateSrv::Response &service_response)
{
  service_response.success = false;
  std::string cola_ascii = "sRN LocState";
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if(cola_response.command_name == "LocState" && cola_response.parameter.size() == 1)
  {
    service_response.state = sick_lidar_localization::ColaParser::convertColaArg(cola_response.parameter[0], 10, -1);
    service_response.success = (service_response.state != -1);
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(\"" << cola_ascii << "\") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service messages (SickLocStartLocalizingSrv, Stop localization, save settings).
 * Sends a cola telegram "Stop localization, save settings" and receives the response from localization controller
 * using ros service "SickLocColaTelegramSrv".
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors.
 */
bool sick_lidar_localization::ColaServices::serviceCbLocStartLocalizing(
  sick_lidar_localization::SickLocStartLocalizingSrv::Request &service_request,
  sick_lidar_localization::SickLocStartLocalizingSrv::Response &service_response)
{
  return serviceCbWithSuccessResponse("sMN", "LocStartLocalizing", "", service_response);
}

/*!
 * Callback for service messages (SickLocStopSrv, Stop localization, save settings).
 * Sends a cola telegram "Stop localization, save settings" and receives the response from localization controller
 * using ros service "SickLocColaTelegramSrv".
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors.
 */
bool sick_lidar_localization::ColaServices::serviceCbLocStop(
  sick_lidar_localization::SickLocStopSrv::Request &service_request,
  sick_lidar_localization::SickLocStopSrv::Response &service_response)
{
  return serviceCbWithSuccessResponse("sMN", "LocStop", "", service_response);
}

/*!
 * Callback for service messages (SickLocSetResultPortSrv, Set TCP-port for result output, default: 2201).
 * Sends a cola telegram "Set mode of result output (default: stream" and receives the response from localization controller
 * using ros service "SickLocColaTelegramSrv".
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors.
 */
bool sick_lidar_localization::ColaServices::serviceCbLocSetResultPort(
  sick_lidar_localization::SickLocSetResultPortSrv::Request &service_request,
  sick_lidar_localization::SickLocSetResultPortSrv::Response &service_response)
{
  // return serviceCbWithSuccessResponse("sMN", "LocSetResultPort", std::string("+") + std::to_string(service_request.port), service_response);
  service_response.success = false;
  std::string cola_ascii = "sMN LocSetResultPort +" + std::to_string(service_request.port);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if(cola_response.command_name == "LocSetResultPort" && cola_response.parameter.size() > 0) // parameter[0]: Set Result (1: success)
  {
    service_response.success = sick_lidar_localization::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(\"" << cola_ascii << "\") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service messages (SickLocSetResultModeSrv, Set mode of result output, default: stream).
 * Sends a cola telegram "sMN LocSetResultMode <mode>" and receives the response from localization controller
 * using ros service "SickLocColaTelegramSrv".
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors.
 */
bool sick_lidar_localization::ColaServices::serviceCbLocSetResultMode(
  sick_lidar_localization::SickLocSetResultModeSrv::Request &service_request,
  sick_lidar_localization::SickLocSetResultModeSrv::Response &service_response)
{
  return serviceCbWithSuccessResponse("sMN", "LocSetResultMode", std::to_string(service_request.mode), service_response);
}

/*!
 * Callback for service messages (SickLocSetResultPoseEnabledSrv, Disable/enable result output).
 * Sends a cola telegram "sMN LocSetResultPoseEnabled <enabled>" and receives the response from localization controller
 * using ros service "SickLocColaTelegramSrv".
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors.
 */
bool sick_lidar_localization::ColaServices::serviceCbLocSetResultPoseEnabled(
  sick_lidar_localization::SickLocSetResultPoseEnabledSrv::Request &service_request,
  sick_lidar_localization::SickLocSetResultPoseEnabledSrv::Response &service_response)
{
  return serviceCbWithSuccessResponse("sMN", "LocSetResultPoseEnabled", std::to_string(service_request.enabled), service_response);
}

/*!
 * Callback for service messages (SickLocSetResultEndiannessSrv, Set endianness of result output).
 * Sends a cola telegram "sMN LocSetResultEndianness <endianness>" and receives the response from localization controller
 * using ros service "SickLocColaTelegramSrv".
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors.
 */
bool sick_lidar_localization::ColaServices::serviceCbLocSetResultEndianness(
  sick_lidar_localization::SickLocSetResultEndiannessSrv::Request &service_request,
  sick_lidar_localization::SickLocSetResultEndiannessSrv::Response &service_response)
{
  return serviceCbWithSuccessResponse("sMN", "LocSetResultEndianness", std::to_string(service_request.endianness), service_response);
}

/*!
 * Callback for service messages (SickLocSetResultPoseIntervalSrv, Set interval of result output).
 * Sends a cola telegram "sMN LocSetResultPoseInterval <interval>" and receives the response from localization controller
 * using ros service "SickLocColaTelegramSrv".
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors.
 */
bool sick_lidar_localization::ColaServices::serviceCbLocSetResultPoseInterval(
  sick_lidar_localization::SickLocSetResultPoseIntervalSrv::Request &service_request,
  sick_lidar_localization::SickLocSetResultPoseIntervalSrv::Response &service_response)
{
  return serviceCbWithSuccessResponse("sMN", "LocSetResultPoseInterval", std::to_string(service_request.interval), service_response);
}

/*!
 * Callback for service messages (SickLocRequestResultDataSrv,  If in poll mode, trigger sending the localization result of the next processed scan via TCP interface).
 * Sends a cola telegram "sMN LocRequestResultData" and receives the response from localization controller
 * using ros service "SickLocColaTelegramSrv".
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors.
 */
bool sick_lidar_localization::ColaServices::serviceCbLocRequestResultData(
  sick_lidar_localization::SickLocRequestResultDataSrv::Request &service_request,
  sick_lidar_localization::SickLocRequestResultDataSrv::Response &service_response)
{
  return serviceCbWithSuccessResponse("sMN", "LocRequestResultData", "", service_response);
}

/*!
 * Callback for service messages (SickLocSetPoseSrv,  Initialize vehicle pose).
 * Sends a cola telegram "sMN LocSetPose <posex> <posey> <yaw> <uncertainty>" and receives the response from localization controller
 * using ros service "SickLocColaTelegramSrv".
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors.
 */
bool sick_lidar_localization::ColaServices::serviceCbLocSetPose(
  sick_lidar_localization::SickLocSetPoseSrv::Request &service_request,
  sick_lidar_localization::SickLocSetPoseSrv::Response &service_response)
{
  std::stringstream cola_args;
  cola_args << std::showpos << service_request.posex << " " << service_request.posey << " " << service_request.yaw << " " << service_request.uncertainty;
  return serviceCbWithSuccessResponse("sMN", "LocSetPose", cola_args.str(), service_response);
}

/*!
 * Callback for service "SickDevSetLidarConfigSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbDevSetLidarConfig(sick_lidar_localization::SickDevSetLidarConfigSrv::Request& service_request, sick_lidar_localization::SickDevSetLidarConfigSrv::Response& service_response)
{
  service_response.set = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "DevSetLidarConfig" && cola_response.parameter.size() > 0)
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.set)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickDevGetLidarConfigSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbDevGetLidarConfig(sick_lidar_localization::SickDevGetLidarConfigSrv::Request& service_request, sick_lidar_localization::SickDevGetLidarConfigSrv::Response& service_response)
{
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "DevGetLidarConfig" && cola_response.parameter.size() > 0)
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response))
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocSetMapSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocSetMap(sick_lidar_localization::SickLocSetMapSrv::Request& service_request, sick_lidar_localization::SickLocSetMapSrv::Response& service_response)
{
  service_response.set = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocSetMap" && cola_response.parameter.size() > 0)
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.set)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocMapSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocMap(sick_lidar_localization::SickLocMapSrv::Request& service_request, sick_lidar_localization::SickLocMapSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocMap" && cola_response.parameter.size() > 0)
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocMapStateSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocMapState(sick_lidar_localization::SickLocMapStateSrv::Request& service_request, sick_lidar_localization::SickLocMapStateSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocMapState" && cola_response.parameter.size() > 0)
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocInitializePoseSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocInitializePose(sick_lidar_localization::SickLocInitializePoseSrv::Request& service_request, sick_lidar_localization::SickLocInitializePoseSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocInitializePose" && cola_response.parameter.size() > 0)
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocInitialPoseSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocInitialPose(sick_lidar_localization::SickLocInitialPoseSrv::Request& service_request, sick_lidar_localization::SickLocInitialPoseSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocInitialPose")
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocSetReflectorsForSupportActiveSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocSetReflectorsForSupportActive(sick_lidar_localization::SickLocSetReflectorsForSupportActiveSrv::Request& service_request, sick_lidar_localization::SickLocSetReflectorsForSupportActiveSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocSetReflectorsForSupportActive" && cola_response.parameter.size() > 0)
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocReflectorsForSupportActiveSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocReflectorsForSupportActive(sick_lidar_localization::SickLocReflectorsForSupportActiveSrv::Request& service_request, sick_lidar_localization::SickLocReflectorsForSupportActiveSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocReflectorsForSupportActive" && cola_response.parameter.size() > 0)
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocSetOdometryActiveSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocSetOdometryActive(sick_lidar_localization::SickLocSetOdometryActiveSrv::Request& service_request, sick_lidar_localization::SickLocSetOdometryActiveSrv::Response& service_response)
{
  service_response.set = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocSetOdometryActive" && cola_response.parameter.size() > 0)
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.set)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocOdometryActiveSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocOdometryActive(sick_lidar_localization::SickLocOdometryActiveSrv::Request& service_request, sick_lidar_localization::SickLocOdometryActiveSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocOdometryActive" && cola_response.parameter.size() > 0)
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocSetOdometryPortSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocSetOdometryPort(sick_lidar_localization::SickLocSetOdometryPortSrv::Request& service_request, sick_lidar_localization::SickLocSetOdometryPortSrv::Response& service_response)
{
  service_response.set = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocSetOdometryPort" && cola_response.parameter.size() > 0)
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.set)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocOdometryPortSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocOdometryPort(sick_lidar_localization::SickLocOdometryPortSrv::Request& service_request, sick_lidar_localization::SickLocOdometryPortSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocOdometryPort" && cola_response.parameter.size() > 0)
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocSetOdometryRestrictYMotionSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocSetOdometryRestrictYMotion(sick_lidar_localization::SickLocSetOdometryRestrictYMotionSrv::Request& service_request, sick_lidar_localization::SickLocSetOdometryRestrictYMotionSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocSetOdometryRestrictYMotion" && cola_response.parameter.size() > 0)
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocOdometryRestrictYMotionSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocOdometryRestrictYMotion(sick_lidar_localization::SickLocOdometryRestrictYMotionSrv::Request& service_request, sick_lidar_localization::SickLocOdometryRestrictYMotionSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocOdometryRestrictYMotion" && cola_response.parameter.size() > 0)
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocSetAutoStartActiveSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocSetAutoStartActive(sick_lidar_localization::SickLocSetAutoStartActiveSrv::Request& service_request, sick_lidar_localization::SickLocSetAutoStartActiveSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocSetAutoStartActive" && cola_response.parameter.size() > 0)
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocAutoStartActiveSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocAutoStartActive(sick_lidar_localization::SickLocAutoStartActiveSrv::Request& service_request, sick_lidar_localization::SickLocAutoStartActiveSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocAutoStartActive" && cola_response.parameter.size() > 0)
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocSetAutoStartSavePoseIntervalSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocSetAutoStartSavePoseInterval(sick_lidar_localization::SickLocSetAutoStartSavePoseIntervalSrv::Request& service_request, sick_lidar_localization::SickLocSetAutoStartSavePoseIntervalSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocSetAutoStartSavePoseInterval" && cola_response.parameter.size() > 0)
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocAutoStartSavePoseIntervalSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocAutoStartSavePoseInterval(sick_lidar_localization::SickLocAutoStartSavePoseIntervalSrv::Request& service_request, sick_lidar_localization::SickLocAutoStartSavePoseIntervalSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocAutoStartSavePoseInterval" && cola_response.parameter.size() > 0)
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocSetRingBufferRecordingActiveSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocSetRingBufferRecordingActive(sick_lidar_localization::SickLocSetRingBufferRecordingActiveSrv::Request& service_request, sick_lidar_localization::SickLocSetRingBufferRecordingActiveSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocSetRingBufferRecordingActive" && cola_response.parameter.size() > 0)
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocRingBufferRecordingActiveSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocRingBufferRecordingActive(sick_lidar_localization::SickLocRingBufferRecordingActiveSrv::Request& service_request, sick_lidar_localization::SickLocRingBufferRecordingActiveSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocRingBufferRecordingActive" && cola_response.parameter.size() > 0)
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickDevGetLidarIdentSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbDevGetLidarIdent(sick_lidar_localization::SickDevGetLidarIdentSrv::Request& service_request, sick_lidar_localization::SickDevGetLidarIdentSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "DevGetLidarIdent" && cola_response.parameter.size() > 0)
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickDevGetLidarStateSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbDevGetLidarState(sick_lidar_localization::SickDevGetLidarStateSrv::Request& service_request, sick_lidar_localization::SickDevGetLidarStateSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "DevGetLidarState" && cola_response.parameter.size() > 0)
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickGetSoftwareVersionSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbGetSoftwareVersion(sick_lidar_localization::SickGetSoftwareVersionSrv::Request& service_request, sick_lidar_localization::SickGetSoftwareVersionSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "GetSoftwareVersion")
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocAutoStartSavePoseSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocAutoStartSavePose(sick_lidar_localization::SickLocAutoStartSavePoseSrv::Request& service_request, sick_lidar_localization::SickLocAutoStartSavePoseSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocAutoStartSavePose")
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocForceUpdateSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocForceUpdate(sick_lidar_localization::SickLocForceUpdateSrv::Request& service_request, sick_lidar_localization::SickLocForceUpdateSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocForceUpdate")
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocSaveRingBufferRecordingSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocSaveRingBufferRecording(sick_lidar_localization::SickLocSaveRingBufferRecordingSrv::Request& service_request, sick_lidar_localization::SickLocSaveRingBufferRecordingSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocSaveRingBufferRecording" && cola_response.parameter.size() > 0)
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocStartDemoMappingSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocStartDemoMapping(sick_lidar_localization::SickLocStartDemoMappingSrv::Request& service_request, sick_lidar_localization::SickLocStartDemoMappingSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocStartDemoMapping")
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickReportUserMessageSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbReportUserMessage(sick_lidar_localization::SickReportUserMessageSrv::Request& service_request, sick_lidar_localization::SickReportUserMessageSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "ReportUserMessage" && cola_response.parameter.size() > 0)
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickSavePermanentSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbSavePermanent(sick_lidar_localization::SickSavePermanentSrv::Request& service_request, sick_lidar_localization::SickSavePermanentSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "SavePermanent")
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocResultPortSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocResultPort(sick_lidar_localization::SickLocResultPortSrv::Request& service_request, sick_lidar_localization::SickLocResultPortSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocResultPort")
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocResultModeSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocResultMode(sick_lidar_localization::SickLocResultModeSrv::Request& service_request, sick_lidar_localization::SickLocResultModeSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocResultMode")
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocResultEndiannessSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocResultEndianness(sick_lidar_localization::SickLocResultEndiannessSrv::Request& service_request, sick_lidar_localization::SickLocResultEndiannessSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocResultEndianness")
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocResultStateSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocResultState(sick_lidar_localization::SickLocResultStateSrv::Request& service_request, sick_lidar_localization::SickLocResultStateSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocResultState")
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickLocResultPoseIntervalSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbLocResultPoseInterval(sick_lidar_localization::SickLocResultPoseIntervalSrv::Request& service_request, sick_lidar_localization::SickLocResultPoseIntervalSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "LocResultPoseInterval")
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickDevSetIMUActiveSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbDevSetIMUActive(sick_lidar_localization::SickDevSetIMUActiveSrv::Request& service_request, sick_lidar_localization::SickDevSetIMUActiveSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "DevSetIMUActive" && cola_response.parameter.size() > 0)
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}

/*!
 * Callback for service "SickDevIMUActiveSrv"
 * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
 * Uses ros service "SickLocColaTelegramSrv"
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
 */
bool sick_lidar_localization::ColaServices::serviceCbDevIMUActive(sick_lidar_localization::SickDevIMUActiveSrv::Request& service_request, sick_lidar_localization::SickDevIMUActiveSrv::Response& service_response)
{
  service_response.success = false;
  std::string cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request);
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii);
  if (cola_response.command_name == "DevIMUActive")
  {
    if (!sick_lidar_localization::ColaEncoder::parseServiceResponse(cola_response, service_response) || !service_response.success)
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", ColaConverter::parseServiceResponse() failed.");
      return false;
    }
    return true;
  }
  ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << cola_ascii << ") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
  return false;
}
