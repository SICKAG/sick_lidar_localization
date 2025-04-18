/*
 * @brief sick_services implements the ROS services for sick localization.
 * It converts the ROS services to json, sends them to the localization device
 * using its REST API, receives the json response and returns the response data
 * converted to the service definition.
 *
 * Copyright (C) 2021 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2021 SICK AG, Waldkirch
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
 *  Copyright 2021 SICK AG
 *  Copyright 2021 Ing.-Buero Dr. Michael Lehning
 *
 */
#ifndef __SICK_LIDAR_LOCALIZATION_SERVICES_H_INCLUDED
#define __SICK_LIDAR_LOCALIZATION_SERVICES_H_INCLUDED

#include <mutex>
#include <thread>

#include "sick_lidar_localization/sick_common.h"
#include "sick_lidar_localization/json_parser.h"

#if __ROS_VERSION == 1
#include "sick_lidar_localization_msgs/LocAutoStartSavePoseSrv.h"
#include "sick_lidar_localization_msgs/LocClearMapCacheSrv.h"
#include "sick_lidar_localization_msgs/LocGetErrorLevelSrv.h"
#include "sick_lidar_localization_msgs/LocGetMapSrv.h"
#include "sick_lidar_localization_msgs/LocGetSystemStateSrv.h"
#include "sick_lidar_localization_msgs/LocInitializeAtPoseSrv.h"
#include "sick_lidar_localization_msgs/LocIsSystemReadySrv.h"
#include "sick_lidar_localization_msgs/LocLoadMapToCacheSrv.h"
#include "sick_lidar_localization_msgs/LocRequestTimestampSrv.h"
#include "sick_lidar_localization_msgs/LocResumeAtPoseSrv.h"
#include "sick_lidar_localization_msgs/LocSaveRingBufferRecordingSrv.h"
#include "sick_lidar_localization_msgs/LocSetKinematicVehicleModelActiveSrv.h"
#include "sick_lidar_localization_msgs/LocSetLinesForSupportActiveSrv.h"
#include "sick_lidar_localization_msgs/LocSetMappingActiveSrv.h"
#include "sick_lidar_localization_msgs/LocSetMapSrv.h"
#include "sick_lidar_localization_msgs/LocSetOdometryActiveSrv.h"
#include "sick_lidar_localization_msgs/LocSetRecordingActiveSrv.h"
#include "sick_lidar_localization_msgs/LocSetRingBufferRecordingActiveSrv.h"
#include "sick_lidar_localization_msgs/LocStartSrv.h"
#include "sick_lidar_localization_msgs/LocStopSrv.h"
#include "sick_lidar_localization_msgs/LocSwitchMapSrv.h"
#include "sick_lidar_localization_msgs/LocGetLocalizationStatusSrv.h"
#include "sick_lidar_localization_msgs/LocGetSoftwareVersionSrv.h"
#include "sick_lidar_localization_msgs/LocLoadPersistentConfigSrv.h"
//#include "sick_lidar_localization/LocSavePermanentSrv.h"
#elif __ROS_VERSION == 2
#include "sick_lidar_localization_msgs/srv/loc_auto_start_save_pose_srv.hpp"
#include "sick_lidar_localization_msgs/srv/loc_clear_map_cache_srv.hpp"
#include "sick_lidar_localization_msgs/srv/loc_get_error_level_srv.hpp"
#include "sick_lidar_localization_msgs/srv/loc_get_map_srv.hpp"
#include "sick_lidar_localization_msgs/srv/loc_get_system_state_srv.hpp"
#include "sick_lidar_localization_msgs/srv/loc_initialize_at_pose_srv.hpp"
#include "sick_lidar_localization_msgs/srv/loc_is_system_ready_srv.hpp"
#include "sick_lidar_localization_msgs/srv/loc_load_map_to_cache_srv.hpp"
#include "sick_lidar_localization_msgs/srv/loc_request_timestamp_srv.hpp"
#include "sick_lidar_localization_msgs/srv/loc_resume_at_pose_srv.hpp"
#include "sick_lidar_localization_msgs/srv/loc_save_ring_buffer_recording_srv.hpp"
#include "sick_lidar_localization_msgs/srv/loc_set_kinematic_vehicle_model_active_srv.hpp"
#include "sick_lidar_localization_msgs/srv/loc_set_lines_for_support_active_srv.hpp"
#include "sick_lidar_localization_msgs/srv/loc_set_map_srv.hpp"
#include "sick_lidar_localization_msgs/srv/loc_set_mapping_active_srv.hpp"
#include "sick_lidar_localization_msgs/srv/loc_set_odometry_active_srv.hpp"
#include "sick_lidar_localization_msgs/srv/loc_set_recording_active_srv.hpp"
#include "sick_lidar_localization_msgs/srv/loc_set_ring_buffer_recording_active_srv.hpp"
#include "sick_lidar_localization_msgs/srv/loc_start_srv.hpp"
#include "sick_lidar_localization_msgs/srv/loc_stop_srv.hpp"
#include "sick_lidar_localization_msgs/srv/loc_switch_map_srv.hpp"
#include "sick_lidar_localization_msgs/srv/loc_get_localization_status_srv.hpp"
#include "sick_lidar_localization_msgs/srv/loc_get_software_version_srv.hpp"
#include "sick_lidar_localization_msgs/srv/loc_load_persistent_config_srv.hpp"
//#include "sick_lidar_localization/srv/loc_save_permanent_srv.hpp"
namespace sick_lidar_localization_msgs { using namespace srv; }
#endif

namespace sick_lidar_localization
{

    /*
     * @brief Container for LocRequestTimestamp response with send and receive timestamps
     */
    struct LocRequestTimestampResponse
    {
        uint64_t timestamp_lidar_microsec;    // Lidar timestamp in microseconds from LocRequestTimestamp response
        uint64_t mean_time_vehicle_microsec;  // Vehicle mean timestamp in microseconds: (send_time_vehicle + receive_time_vehicle) / 2
        uint64_t delta_time_microsec;         // Time offset: mean_time_vehicle_microsec - timestamp_lidar_microsec
        uint32_t send_time_vehicle_sec;       // Vehicle timestamp when sending LocRequestTimestamp (seconds part of ros timestamp immediately before tcp send)
        uint32_t send_time_vehicle_nsec;      // Vehicle timestamp when sending LocRequestTimestamp (nano seconds part of ros timestamp immediately before tcp send)
        uint32_t receive_time_vehicle_sec;    // Vehicle timestamp when receiving the LocRequestTimestamp response (seconds part of ros timestamp immediately after first response byte received)
        uint32_t receive_time_vehicle_nsec;   // Vehicle timestamp when receiving the LocRequestTimestamp response (nano seconds part of ros timestamp immediately after first response byte received) 
    };

    /*
     * @brief Container for seconds and nanoseconds part of the timestamp calculated by Software PLL
     */
    struct SyncTimeStamp
    {
        uint32_t sec;  // seconds part of the timestamp
        uint32_t nsec; // nanoseconds part of the timestamp
        uint32_t valid;    // timestamp successfully calculated and valid
    };

    inline SyncTimeStamp makeSyncTimeStamp(uint32_t sec = 0, uint32_t nsec = 0,  bool valid = false)
    {
        sick_lidar_localization::SyncTimeStamp sync_time_stamp;
        sync_time_stamp.sec = sec;
        sync_time_stamp.nsec = nsec;
        sync_time_stamp.valid = valid ? 1 : 0;
        return sync_time_stamp;
    }

    /*
     * @brief sick_services implements the ROS services for sick localization.
     * It converts the ROS services to json, sends them to the localization device
     * using its REST API, receives the json response and returns the response data
     * converted to the service definition.
     */
    class SickServices
    {
    public:

        SickServices(rosNodePtr nh, const std::string& lls_device_ip, const int& lls_device_webserver_port, int software_pll_fifo_length, int verbose);

        /*
         * @brief Sends a LocRequestTimestamp, updates the SoftwarePLL and returns LocRequestTimestamp response incl. send and receive timestamps
         */
        LocRequestTimestampResponse requestTimestamp(void);

        /*
         * @brief Start a time sync thread to periodically request timestamps and to update Software PLL
         */
        bool startTimeSyncThread(void);

        /*
         * @brief Stops the time sync thread to periodically request timestamps and to update Software PLL
         */
        void stopTimeSyncThread(void);

        /*
         * @brief Calculates the system time of a vehicle pose from lidar ticks, using a software pll.
         * https://github.com/SICKAG/sick_lidar_localization/blob/1ce55facdb48eb13ac5ef6813b273ee1eaf00ba4/src/time_sync_service.cpp#L276
         * @param[in] lidar_tics_ms lidar tics
         * @param[in] verbose > 0 prints calculated time, otherwise silent mode
         * @return seconds and nanoseconds part of calculated time
         */
        sick_lidar_localization::SyncTimeStamp getSystemTimeFromTics(uint64_t lidar_tics_microsec, int verbose = 0);

        /*
         * @brief Computes the difference between timestamp_sync and the current system time. Prints a warning and returns false, if delta time in seconds exceeds max_delta_time_seconds.
         */
        bool checkDeltaTimestampToNow(uint64_t lidar_tics_microsec, sick_lidar_localization::SyncTimeStamp& timestamp_sync, double max_delta_time_seconds);

        /*
         * @brief Returns default arguments of a service call. Example:
         *   std::string def_command, def_method, def_json;
         *   getDefaultCommand("LocIsSystemReady", def_command, def_method, def_json);
         *   std::cout << def_command << ", " << def_method << ", " << def_json << std::endl;
         * Output for "LocIsSystemReady" is "IsSystemReady, POST, {}"
         *   
         */
        static void getDefaultCommand(const std::string& command, std::string& def_command, std::string& def_method, std::string& def_json);

    protected:

#if __ROS_VERSION > 0 // ROS service callback declarations generated by ros_service_prep.py

        bool serviceCbLocAutoStartSavePoseSrvROS1(sick_lidar_localization_msgs::LocAutoStartSavePoseSrv::Request &service_request, sick_lidar_localization_msgs::LocAutoStartSavePoseSrv::Response &service_response);
        bool serviceCbLocAutoStartSavePoseSrvROS2(std::shared_ptr<sick_lidar_localization_msgs::LocAutoStartSavePoseSrv::Request> service_request, std::shared_ptr<sick_lidar_localization_msgs::LocAutoStartSavePoseSrv::Response> service_response);
        rosServiceServer<sick_lidar_localization_msgs::LocAutoStartSavePoseSrv> m_srv_server_LocAutoStartSavePoseSrv;
        
        bool serviceCbLocClearMapCacheSrvROS1(sick_lidar_localization_msgs::LocClearMapCacheSrv::Request &service_request, sick_lidar_localization_msgs::LocClearMapCacheSrv::Response &service_response);
        bool serviceCbLocClearMapCacheSrvROS2(std::shared_ptr<sick_lidar_localization_msgs::LocClearMapCacheSrv::Request> service_request, std::shared_ptr<sick_lidar_localization_msgs::LocClearMapCacheSrv::Response> service_response);
        rosServiceServer<sick_lidar_localization_msgs::LocClearMapCacheSrv> m_srv_server_LocClearMapCacheSrv;
        
        bool serviceCbLocGetErrorLevelSrvROS1(sick_lidar_localization_msgs::LocGetErrorLevelSrv::Request &service_request, sick_lidar_localization_msgs::LocGetErrorLevelSrv::Response &service_response);
        bool serviceCbLocGetErrorLevelSrvROS2(std::shared_ptr<sick_lidar_localization_msgs::LocGetErrorLevelSrv::Request> service_request, std::shared_ptr<sick_lidar_localization_msgs::LocGetErrorLevelSrv::Response> service_response);
        rosServiceServer<sick_lidar_localization_msgs::LocGetErrorLevelSrv> m_srv_server_LocGetErrorLevelSrv;
        
        bool serviceCbLocGetMapSrvROS1(sick_lidar_localization_msgs::LocGetMapSrv::Request &service_request, sick_lidar_localization_msgs::LocGetMapSrv::Response &service_response);
        bool serviceCbLocGetMapSrvROS2(std::shared_ptr<sick_lidar_localization_msgs::LocGetMapSrv::Request> service_request, std::shared_ptr<sick_lidar_localization_msgs::LocGetMapSrv::Response> service_response);
        rosServiceServer<sick_lidar_localization_msgs::LocGetMapSrv> m_srv_server_LocGetMapSrv;
        
        bool serviceCbLocGetSystemStateSrvROS1(sick_lidar_localization_msgs::LocGetSystemStateSrv::Request &service_request, sick_lidar_localization_msgs::LocGetSystemStateSrv::Response &service_response);
        bool serviceCbLocGetSystemStateSrvROS2(std::shared_ptr<sick_lidar_localization_msgs::LocGetSystemStateSrv::Request> service_request, std::shared_ptr<sick_lidar_localization_msgs::LocGetSystemStateSrv::Response> service_response);
        rosServiceServer<sick_lidar_localization_msgs::LocGetSystemStateSrv> m_srv_server_LocGetSystemStateSrv;
        
        bool serviceCbLocInitializeAtPoseSrvROS1(sick_lidar_localization_msgs::LocInitializeAtPoseSrv::Request &service_request, sick_lidar_localization_msgs::LocInitializeAtPoseSrv::Response &service_response);
        bool serviceCbLocInitializeAtPoseSrvROS2(std::shared_ptr<sick_lidar_localization_msgs::LocInitializeAtPoseSrv::Request> service_request, std::shared_ptr<sick_lidar_localization_msgs::LocInitializeAtPoseSrv::Response> service_response);
        rosServiceServer<sick_lidar_localization_msgs::LocInitializeAtPoseSrv> m_srv_server_LocInitializeAtPoseSrv;
        
        bool serviceCbLocIsSystemReadySrvROS1(sick_lidar_localization_msgs::LocIsSystemReadySrv::Request &service_request, sick_lidar_localization_msgs::LocIsSystemReadySrv::Response &service_response);
        bool serviceCbLocIsSystemReadySrvROS2(std::shared_ptr<sick_lidar_localization_msgs::LocIsSystemReadySrv::Request> service_request, std::shared_ptr<sick_lidar_localization_msgs::LocIsSystemReadySrv::Response> service_response);
        rosServiceServer<sick_lidar_localization_msgs::LocIsSystemReadySrv> m_srv_server_LocIsSystemReadySrv;
        
        bool serviceCbLocLoadMapToCacheSrvROS1(sick_lidar_localization_msgs::LocLoadMapToCacheSrv::Request &service_request, sick_lidar_localization_msgs::LocLoadMapToCacheSrv::Response &service_response);
        bool serviceCbLocLoadMapToCacheSrvROS2(std::shared_ptr<sick_lidar_localization_msgs::LocLoadMapToCacheSrv::Request> service_request, std::shared_ptr<sick_lidar_localization_msgs::LocLoadMapToCacheSrv::Response> service_response);
        rosServiceServer<sick_lidar_localization_msgs::LocLoadMapToCacheSrv> m_srv_server_LocLoadMapToCacheSrv;
        
        bool serviceCbLocRequestTimestampSrvROS1(sick_lidar_localization_msgs::LocRequestTimestampSrv::Request &service_request, sick_lidar_localization_msgs::LocRequestTimestampSrv::Response &service_response);
        bool serviceCbLocRequestTimestampSrvROS2(std::shared_ptr<sick_lidar_localization_msgs::LocRequestTimestampSrv::Request> service_request, std::shared_ptr<sick_lidar_localization_msgs::LocRequestTimestampSrv::Response> service_response);
        rosServiceServer<sick_lidar_localization_msgs::LocRequestTimestampSrv> m_srv_server_LocRequestTimestampSrv;
        
        bool serviceCbLocResumeAtPoseSrvROS1(sick_lidar_localization_msgs::LocResumeAtPoseSrv::Request &service_request, sick_lidar_localization_msgs::LocResumeAtPoseSrv::Response &service_response);
        bool serviceCbLocResumeAtPoseSrvROS2(std::shared_ptr<sick_lidar_localization_msgs::LocResumeAtPoseSrv::Request> service_request, std::shared_ptr<sick_lidar_localization_msgs::LocResumeAtPoseSrv::Response> service_response);
        rosServiceServer<sick_lidar_localization_msgs::LocResumeAtPoseSrv> m_srv_server_LocResumeAtPoseSrv;
        
        bool serviceCbLocSaveRingBufferRecordingSrvROS1(sick_lidar_localization_msgs::LocSaveRingBufferRecordingSrv::Request &service_request, sick_lidar_localization_msgs::LocSaveRingBufferRecordingSrv::Response &service_response);
        bool serviceCbLocSaveRingBufferRecordingSrvROS2(std::shared_ptr<sick_lidar_localization_msgs::LocSaveRingBufferRecordingSrv::Request> service_request, std::shared_ptr<sick_lidar_localization_msgs::LocSaveRingBufferRecordingSrv::Response> service_response);
        rosServiceServer<sick_lidar_localization_msgs::LocSaveRingBufferRecordingSrv> m_srv_server_LocSaveRingBufferRecordingSrv;
        
        bool serviceCbLocSetKinematicVehicleModelActiveSrvROS1(sick_lidar_localization_msgs::LocSetKinematicVehicleModelActiveSrv::Request &service_request, sick_lidar_localization_msgs::LocSetKinematicVehicleModelActiveSrv::Response &service_response);
        bool serviceCbLocSetKinematicVehicleModelActiveSrvROS2(std::shared_ptr<sick_lidar_localization_msgs::LocSetKinematicVehicleModelActiveSrv::Request> service_request, std::shared_ptr<sick_lidar_localization_msgs::LocSetKinematicVehicleModelActiveSrv::Response> service_response);
        rosServiceServer<sick_lidar_localization_msgs::LocSetKinematicVehicleModelActiveSrv> m_srv_server_LocSetKinematicVehicleModelActiveSrv;
        
        bool serviceCbLocSetLinesForSupportActiveSrvROS1(sick_lidar_localization_msgs::LocSetLinesForSupportActiveSrv::Request &service_request, sick_lidar_localization_msgs::LocSetLinesForSupportActiveSrv::Response &service_response);
        bool serviceCbLocSetLinesForSupportActiveSrvROS2(std::shared_ptr<sick_lidar_localization_msgs::LocSetLinesForSupportActiveSrv::Request> service_request, std::shared_ptr<sick_lidar_localization_msgs::LocSetLinesForSupportActiveSrv::Response> service_response);
        rosServiceServer<sick_lidar_localization_msgs::LocSetLinesForSupportActiveSrv> m_srv_server_LocSetLinesForSupportActiveSrv;
        
        bool serviceCbLocSetMappingActiveSrvROS1(sick_lidar_localization_msgs::LocSetMappingActiveSrv::Request &service_request, sick_lidar_localization_msgs::LocSetMappingActiveSrv::Response &service_response);
        bool serviceCbLocSetMappingActiveSrvROS2(std::shared_ptr<sick_lidar_localization_msgs::LocSetMappingActiveSrv::Request> service_request, std::shared_ptr<sick_lidar_localization_msgs::LocSetMappingActiveSrv::Response> service_response);
        rosServiceServer<sick_lidar_localization_msgs::LocSetMappingActiveSrv> m_srv_server_LocSetMappingActiveSrv;
        
        bool serviceCbLocSetMapSrvROS1(sick_lidar_localization_msgs::LocSetMapSrv::Request &service_request, sick_lidar_localization_msgs::LocSetMapSrv::Response &service_response);
        bool serviceCbLocSetMapSrvROS2(std::shared_ptr<sick_lidar_localization_msgs::LocSetMapSrv::Request> service_request, std::shared_ptr<sick_lidar_localization_msgs::LocSetMapSrv::Response> service_response);
        rosServiceServer<sick_lidar_localization_msgs::LocSetMapSrv> m_srv_server_LocSetMapSrv;
        
        bool serviceCbLocSetOdometryActiveSrvROS1(sick_lidar_localization_msgs::LocSetOdometryActiveSrv::Request &service_request, sick_lidar_localization_msgs::LocSetOdometryActiveSrv::Response &service_response);
        bool serviceCbLocSetOdometryActiveSrvROS2(std::shared_ptr<sick_lidar_localization_msgs::LocSetOdometryActiveSrv::Request> service_request, std::shared_ptr<sick_lidar_localization_msgs::LocSetOdometryActiveSrv::Response> service_response);
        rosServiceServer<sick_lidar_localization_msgs::LocSetOdometryActiveSrv> m_srv_server_LocSetOdometryActiveSrv;
        
        bool serviceCbLocSetRecordingActiveSrvROS1(sick_lidar_localization_msgs::LocSetRecordingActiveSrv::Request &service_request, sick_lidar_localization_msgs::LocSetRecordingActiveSrv::Response &service_response);
        bool serviceCbLocSetRecordingActiveSrvROS2(std::shared_ptr<sick_lidar_localization_msgs::LocSetRecordingActiveSrv::Request> service_request, std::shared_ptr<sick_lidar_localization_msgs::LocSetRecordingActiveSrv::Response> service_response);
        rosServiceServer<sick_lidar_localization_msgs::LocSetRecordingActiveSrv> m_srv_server_LocSetRecordingActiveSrv;
        
        bool serviceCbLocSetRingBufferRecordingActiveSrvROS1(sick_lidar_localization_msgs::LocSetRingBufferRecordingActiveSrv::Request &service_request, sick_lidar_localization_msgs::LocSetRingBufferRecordingActiveSrv::Response &service_response);
        bool serviceCbLocSetRingBufferRecordingActiveSrvROS2(std::shared_ptr<sick_lidar_localization_msgs::LocSetRingBufferRecordingActiveSrv::Request> service_request, std::shared_ptr<sick_lidar_localization_msgs::LocSetRingBufferRecordingActiveSrv::Response> service_response);
        rosServiceServer<sick_lidar_localization_msgs::LocSetRingBufferRecordingActiveSrv> m_srv_server_LocSetRingBufferRecordingActiveSrv;
        
        bool serviceCbLocStartSrvROS1(sick_lidar_localization_msgs::LocStartSrv::Request &service_request, sick_lidar_localization_msgs::LocStartSrv::Response &service_response);
        bool serviceCbLocStartSrvROS2(std::shared_ptr<sick_lidar_localization_msgs::LocStartSrv::Request> service_request, std::shared_ptr<sick_lidar_localization_msgs::LocStartSrv::Response> service_response);
        rosServiceServer<sick_lidar_localization_msgs::LocStartSrv> m_srv_server_LocStartSrv;
        
        bool serviceCbLocStopSrvROS1(sick_lidar_localization_msgs::LocStopSrv::Request &service_request, sick_lidar_localization_msgs::LocStopSrv::Response &service_response);
        bool serviceCbLocStopSrvROS2(std::shared_ptr<sick_lidar_localization_msgs::LocStopSrv::Request> service_request, std::shared_ptr<sick_lidar_localization_msgs::LocStopSrv::Response> service_response);
        rosServiceServer<sick_lidar_localization_msgs::LocStopSrv> m_srv_server_LocStopSrv;
        
        bool serviceCbLocSwitchMapSrvROS1(sick_lidar_localization_msgs::LocSwitchMapSrv::Request &service_request, sick_lidar_localization_msgs::LocSwitchMapSrv::Response &service_response);
        bool serviceCbLocSwitchMapSrvROS2(std::shared_ptr<sick_lidar_localization_msgs::LocSwitchMapSrv::Request> service_request, std::shared_ptr<sick_lidar_localization_msgs::LocSwitchMapSrv::Response> service_response);
        rosServiceServer<sick_lidar_localization_msgs::LocSwitchMapSrv> m_srv_server_LocSwitchMapSrv;
        
        bool serviceCbLocGetLocalizationStatusSrvROS1(sick_lidar_localization_msgs::LocGetLocalizationStatusSrv::Request &service_request, sick_lidar_localization_msgs::LocGetLocalizationStatusSrv::Response &service_response);
        bool serviceCbLocGetLocalizationStatusSrvROS2(std::shared_ptr<sick_lidar_localization_msgs::LocGetLocalizationStatusSrv::Request> service_request, std::shared_ptr<sick_lidar_localization_msgs::LocGetLocalizationStatusSrv::Response> service_response);
        rosServiceServer<sick_lidar_localization_msgs::LocGetLocalizationStatusSrv> m_srv_server_LocGetLocalizationStatusSrv;
        
        bool serviceCbLocGetSoftwareVersionSrvROS1(sick_lidar_localization_msgs::LocGetSoftwareVersionSrv::Request &service_request, sick_lidar_localization_msgs::LocGetSoftwareVersionSrv::Response &service_response);
        bool serviceCbLocGetSoftwareVersionSrvROS2(std::shared_ptr<sick_lidar_localization_msgs::LocGetSoftwareVersionSrv::Request> service_request, std::shared_ptr<sick_lidar_localization_msgs::LocGetSoftwareVersionSrv::Response> service_response);
        rosServiceServer<sick_lidar_localization_msgs::LocGetSoftwareVersionSrv> m_srv_server_LocGetSoftwareVersionSrv;
        
        bool serviceCbLocLoadPersistentConfigSrvROS1(sick_lidar_localization_msgs::LocLoadPersistentConfigSrv::Request &service_request, sick_lidar_localization_msgs::LocLoadPersistentConfigSrv::Response &service_response);
        bool serviceCbLocLoadPersistentConfigSrvROS2(std::shared_ptr<sick_lidar_localization_msgs::LocLoadPersistentConfigSrv::Request> service_request, std::shared_ptr<sick_lidar_localization_msgs::LocLoadPersistentConfigSrv::Response> service_response);
        rosServiceServer<sick_lidar_localization_msgs::LocLoadPersistentConfigSrv> m_srv_server_LocLoadPersistentConfigSrv;
        
        //bool serviceCbLocSavePermanentSrvROS1(sick_lidar_localization_msgs::LocSavePermanentSrv::Request &service_request, sick_lidar_localization_msgs::LocSavePermanentSrv::Response &service_response);
        //bool serviceCbLocSavePermanentSrvROS2(std::shared_ptr<sick_lidar_localization_msgs::LocSavePermanentSrv::Request> service_request, std::shared_ptr<sick_lidar_localization_msgs::LocSavePermanentSrv::Response> service_response);
        //rosServiceServer<sick_lidar_localization_msgs::LocSavePermanentSrv> m_srv_server_LocSavePermanentSrv;
        
        std::string convertToJson(bool val);
        std::string convertToJson(sick_lidar_localization_msgs::LocInitializeAtPoseSrv::Request& request);
        std::string convertToJson(sick_lidar_localization_msgs::LocLoadMapToCacheSrv::Request &request);
        std::string convertToJson(sick_lidar_localization_msgs::LocResumeAtPoseSrv::Request &request);
        std::string convertToJson(sick_lidar_localization_msgs::LocSaveRingBufferRecordingSrv::Request &request);
        std::string convertToJson(sick_lidar_localization_msgs::LocSetKinematicVehicleModelActiveSrv::Request &request);
        std::string convertToJson(sick_lidar_localization_msgs::LocSetLinesForSupportActiveSrv::Request &request);
        std::string convertToJson(sick_lidar_localization_msgs::LocSetMappingActiveSrv::Request &request);
        std::string convertToJson(sick_lidar_localization_msgs::LocSetMapSrv::Request &request);
        std::string convertToJson(sick_lidar_localization_msgs::LocSetOdometryActiveSrv::Request &request);
        std::string convertToJson(sick_lidar_localization_msgs::LocSetRecordingActiveSrv::Request &request);
        std::string convertToJson(sick_lidar_localization_msgs::LocSetRingBufferRecordingActiveSrv::Request &request);
        std::string convertToJson(sick_lidar_localization_msgs::LocSwitchMapSrv::Request &request);

#endif // __ROS_VERSION > 0

        bool runTimeSync(void);

        std::map<std::string, sick_lidar_localization::JsonValue> sendJsonRequestGetResponse(const std::string& command, const std::string& method, const std::string&json_data);

        int m_verbose;
        sick_lidar_localization::CurlWrapper m_curl;

        std::mutex m_software_pll_mutex;
        int m_software_pll_fifo_length;
        std::string m_software_pll_name_send_time;
        std::string m_software_pll_name_recv_time;

        double m_time_sync_rate;
        double m_time_sync_initial_rate;
        int m_time_sync_initial_length;
        bool m_time_sync_thread_running;
        std::thread* m_time_sync_thread;

    }; // class SickServices
} // namespace sick_lidar_localization
#endif // __SICK_LIDAR_LOCALIZATION_SERVICES_H_INCLUDED
