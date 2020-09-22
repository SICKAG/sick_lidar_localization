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
#ifndef __SIM_LOC_COLA_SERVICES_H_INCLUDED
#define __SIM_LOC_COLA_SERVICES_H_INCLUDED

#include "sick_lidar_localization/ros_wrapper.h"
#include "sick_lidar_localization/cola_parser.h"
#include "sick_lidar_localization/driver_monitor.h"
#include "sick_lidar_localization/utils.h"

namespace sick_lidar_localization
{
  /*!
   * Class sick_lidar_localization::ColaServices implements the following ros services using
   * cola telegrams:
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
   */
  class ColaServices
  {
  public:
    
    /*!
     * Constructor
     */
    ColaServices(ROS::NodePtr nh = 0, sick_lidar_localization::DriverMonitor* driver_monitor = 0);
    
    /*!
     * Destructor
     */
    virtual ~ColaServices();
  
    /*!
     * Callback for service messages (SickLocIsSystemReadySrv, Check if the system is ready).
     * Sends a cola telegram "sMN IsSystemReady" and receives the response from localization controller
     * using ros service "SickLocColaTelegramSrv".
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors.
     */
    virtual bool serviceCbIsSystemReady(sick_lidar_localization::SickLocIsSystemReadySrv::Request & service_request, sick_lidar_localization::SickLocIsSystemReadySrv::Response & service_response);
    /*! ROS2 version of function serviceCbIsSystemReady */
    virtual bool serviceCbIsSystemReadyROS2(std::shared_ptr<sick_lidar_localization::SickLocIsSystemReadySrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocIsSystemReadySrv::Response> service_response)
    {
      return serviceCbIsSystemReady(*service_request, *service_response);
    }

    /*!
     * Callback for service messages (SickLocStateSrv, Read localization state).
     * Sends a cola telegram "sRN LocState" and receives the response from localization controller
     * using ros service "SickLocColaTelegramSrv".
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors.
     */
    virtual bool serviceCbLocState(sick_lidar_localization::SickLocStateSrv::Request & service_request, sick_lidar_localization::SickLocStateSrv::Response & service_response);
    /*! ROS2 version of function serviceCbLocState */
    virtual bool serviceCbLocStateROS2(std::shared_ptr<sick_lidar_localization::SickLocStateSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocStateSrv::Response> service_response)
    {
      return serviceCbLocState(*service_request, *service_response);
    }
  
    /*!
     * Callback for service messages (SickLocStartLocalizingSrv, Stop localization, save settings).
     * Sends a cola telegram "Stop localization, save settings" and receives the response from localization controller
     * using ros service "SickLocColaTelegramSrv".
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors.
     */
    virtual bool serviceCbLocStartLocalizing(sick_lidar_localization::SickLocStartLocalizingSrv::Request & service_request, sick_lidar_localization::SickLocStartLocalizingSrv::Response & service_response);
    /*! ROS2 version of function serviceCbLocStartLocalizing */
    virtual bool serviceCbLocStartLocalizingROS2(std::shared_ptr<sick_lidar_localization::SickLocStartLocalizingSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocStartLocalizingSrv::Response> service_response)
    {
      return serviceCbLocStartLocalizing(*service_request, *service_response);
    }
  
    /*!
     * Callback for service messages (SickLocStopSrv, Stop localization, save settings).
     * Sends a cola telegram "Stop localization, save settings" and receives the response from localization controller
     * using ros service "SickLocColaTelegramSrv".
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors.
     */
    virtual bool serviceCbLocStop(sick_lidar_localization::SickLocStopSrv::Request & service_request, sick_lidar_localization::SickLocStopSrv::Response & service_response);
    /*! ROS2 version of function serviceCbLocStop */
    virtual bool serviceCbLocStopROS2(std::shared_ptr<sick_lidar_localization::SickLocStopSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocStopSrv::Response> service_response)
    {
      return serviceCbLocStop(*service_request, *service_response);
    }
  
    /*!
     * Callback for service messages (SickLocSetResultPortSrv, Set TCP-port for result output, default: 2201).
     * Sends a cola telegram "Set mode of result output (default: stream" and receives the response from localization controller
     * using ros service "SickLocColaTelegramSrv".
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors.
     */
    virtual bool serviceCbLocSetResultPort(sick_lidar_localization::SickLocSetResultPortSrv::Request & service_request, sick_lidar_localization::SickLocSetResultPortSrv::Response & service_response);
    /*! ROS2 version of function serviceCbLocSetResultPort */
    virtual bool serviceCbLocSetResultPortROS2(std::shared_ptr<sick_lidar_localization::SickLocSetResultPortSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocSetResultPortSrv::Response> service_response)
    {
      return serviceCbLocSetResultPort(*service_request, *service_response);
    }
  
    /*!
     * Callback for service messages (SickLocSetResultModeSrv, Set mode of result output, default: stream).
     * Sends a cola telegram "sMN LocSetResultMode <mode>" and receives the response from localization controller
     * using ros service "SickLocColaTelegramSrv".
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors.
     */
    virtual bool serviceCbLocSetResultMode(sick_lidar_localization::SickLocSetResultModeSrv::Request & service_request, sick_lidar_localization::SickLocSetResultModeSrv::Response & service_response);
    /*! ROS2 version of function serviceCbLocSetResultMode */
    virtual bool serviceCbLocSetResultModeROS2(std::shared_ptr<sick_lidar_localization::SickLocSetResultModeSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocSetResultModeSrv::Response> service_response)
    {
      return serviceCbLocSetResultMode(*service_request, *service_response);
    }
  
    /*!
     * Callback for service messages (SickLocSetResultPoseEnabledSrv, Disable/enable result output).
     * Sends a cola telegram "sMN LocSetResultPoseEnabled <enabled>" and receives the response from localization controller
     * using ros service "SickLocColaTelegramSrv".
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors.
     */
    virtual bool serviceCbLocSetResultPoseEnabled(sick_lidar_localization::SickLocSetResultPoseEnabledSrv::Request & service_request, sick_lidar_localization::SickLocSetResultPoseEnabledSrv::Response & service_response);
    /*! ROS2 version of function serviceCbLocSetResultPoseEnabled */
    virtual bool serviceCbLocSetResultPoseEnabledROS2(std::shared_ptr<sick_lidar_localization::SickLocSetResultPoseEnabledSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocSetResultPoseEnabledSrv::Response> service_response)
    {
      return serviceCbLocSetResultPoseEnabled(*service_request, *service_response);
    }
  
    /*!
     * Callback for service messages (SickLocSetResultEndiannessSrv, Set endianness of result output).
     * Sends a cola telegram "sMN LocSetResultEndianness <endianness>" and receives the response from localization controller
     * using ros service "SickLocColaTelegramSrv".
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors.
     */
    virtual bool serviceCbLocSetResultEndianness(sick_lidar_localization::SickLocSetResultEndiannessSrv::Request & service_request, sick_lidar_localization::SickLocSetResultEndiannessSrv::Response & service_response);
    /*! ROS2 version of function serviceCbLocSetResultEndianness */
    virtual bool serviceCbLocSetResultEndiannessROS2(std::shared_ptr<sick_lidar_localization::SickLocSetResultEndiannessSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocSetResultEndiannessSrv::Response> service_response)
    {
      return serviceCbLocSetResultEndianness(*service_request, *service_response);
    }
  
    /*!
     * Callback for service messages (SickLocSetResultPoseIntervalSrv, Set interval of result output).
     * Sends a cola telegram "sMN LocSetResultPoseInterval <interval>" and receives the response from localization controller
     * using ros service "SickLocColaTelegramSrv".
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors.
     */
    virtual bool serviceCbLocSetResultPoseInterval(sick_lidar_localization::SickLocSetResultPoseIntervalSrv::Request & service_request, sick_lidar_localization::SickLocSetResultPoseIntervalSrv::Response & service_response);
    /*! ROS2 version of function serviceCbLocSetResultPoseInterval */
    virtual bool serviceCbLocSetResultPoseIntervalROS2(std::shared_ptr<sick_lidar_localization::SickLocSetResultPoseIntervalSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocSetResultPoseIntervalSrv::Response> service_response)
    {
      return serviceCbLocSetResultPoseInterval(*service_request, *service_response);
    }
  
    /*!
     * Callback for service messages (SickLocRequestResultDataSrv,  If in poll mode, trigger sending the localization result of the next processed scan via TCP interface).
     * Sends a cola telegram "sMN LocRequestResultData" and receives the response from localization controller
     * using ros service "SickLocColaTelegramSrv".
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors.
     */
    virtual bool serviceCbLocRequestResultData(sick_lidar_localization::SickLocRequestResultDataSrv::Request & service_request, sick_lidar_localization::SickLocRequestResultDataSrv::Response & service_response);
    /*! ROS2 version of function serviceCbLocRequestResultData */
    virtual bool serviceCbLocRequestResultDataROS2(std::shared_ptr<sick_lidar_localization::SickLocRequestResultDataSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocRequestResultDataSrv::Response> service_response)
    {
      return serviceCbLocRequestResultData(*service_request, *service_response);
    }
  
    /*!
     * Callback for service messages (SickLocSetPoseSrv,  Initialize vehicle pose).
     * Sends a cola telegram "sMN LocSetPose <posex> <posey> <yaw> <uncertainty>" and receives the response from localization controller
     * using ros service "SickLocColaTelegramSrv".
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors.
     */
    virtual bool serviceCbLocSetPose(sick_lidar_localization::SickLocSetPoseSrv::Request & service_request, sick_lidar_localization::SickLocSetPoseSrv::Response & service_response);
    /*! ROS2 version of function serviceCbLocSetPose */
    virtual bool serviceCbLocSetPoseROS2(std::shared_ptr<sick_lidar_localization::SickLocSetPoseSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocSetPoseSrv::Response> service_response)
    {
      return serviceCbLocSetPose(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickDevSetLidarConfigSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbDevSetLidarConfig(sick_lidar_localization::SickDevSetLidarConfigSrv::Request& service_request, sick_lidar_localization::SickDevSetLidarConfigSrv::Response& service_response);
    /*! ROS2 version of function serviceCbDevSetLidarConfig */
    virtual bool serviceCbDevSetLidarConfigROS2(std::shared_ptr<sick_lidar_localization::SickDevSetLidarConfigSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickDevSetLidarConfigSrv::Response> service_response)
    {
      return serviceCbDevSetLidarConfig(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickDevGetLidarConfigSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbDevGetLidarConfig(sick_lidar_localization::SickDevGetLidarConfigSrv::Request& service_request, sick_lidar_localization::SickDevGetLidarConfigSrv::Response& service_response);
    /*! ROS2 version of function serviceCbDevGetLidarConfig */
    virtual bool serviceCbDevGetLidarConfigROS2(std::shared_ptr<sick_lidar_localization::SickDevGetLidarConfigSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickDevGetLidarConfigSrv::Response> service_response)
    {
      return serviceCbDevGetLidarConfig(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocSetMapSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocSetMap(sick_lidar_localization::SickLocSetMapSrv::Request& service_request, sick_lidar_localization::SickLocSetMapSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocSetMap */
    virtual bool serviceCbLocSetMapROS2(std::shared_ptr<sick_lidar_localization::SickLocSetMapSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocSetMapSrv::Response> service_response)
    {
      return serviceCbLocSetMap(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocMapSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocMap(sick_lidar_localization::SickLocMapSrv::Request& service_request, sick_lidar_localization::SickLocMapSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocMap */
    virtual bool serviceCbLocMapROS2(std::shared_ptr<sick_lidar_localization::SickLocMapSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocMapSrv::Response> service_response)
    {
      return serviceCbLocMap(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocMapStateSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocMapState(sick_lidar_localization::SickLocMapStateSrv::Request& service_request, sick_lidar_localization::SickLocMapStateSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocMapState */
    virtual bool serviceCbLocMapStateROS2(std::shared_ptr<sick_lidar_localization::SickLocMapStateSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocMapStateSrv::Response> service_response)
    {
      return serviceCbLocMapState(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocInitializePoseSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocInitializePose(sick_lidar_localization::SickLocInitializePoseSrv::Request& service_request, sick_lidar_localization::SickLocInitializePoseSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocInitializePose */
    virtual bool serviceCbLocInitializePoseROS2(std::shared_ptr<sick_lidar_localization::SickLocInitializePoseSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocInitializePoseSrv::Response> service_response)
    {
      return serviceCbLocInitializePose(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocInitialPoseSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocInitialPose(sick_lidar_localization::SickLocInitialPoseSrv::Request& service_request, sick_lidar_localization::SickLocInitialPoseSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocInitialPose */
    virtual bool serviceCbLocInitialPoseROS2(std::shared_ptr<sick_lidar_localization::SickLocInitialPoseSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocInitialPoseSrv::Response> service_response)
    {
      return serviceCbLocInitialPose(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocSetReflectorsForSupportActiveSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocSetReflectorsForSupportActive(sick_lidar_localization::SickLocSetReflectorsForSupportActiveSrv::Request& service_request, sick_lidar_localization::SickLocSetReflectorsForSupportActiveSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocSetReflectorsForSupportActive */
    virtual bool serviceCbLocSetReflectorsForSupportActiveROS2(std::shared_ptr<sick_lidar_localization::SickLocSetReflectorsForSupportActiveSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocSetReflectorsForSupportActiveSrv::Response> service_response)
    {
      return serviceCbLocSetReflectorsForSupportActive(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocReflectorsForSupportActiveSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocReflectorsForSupportActive(sick_lidar_localization::SickLocReflectorsForSupportActiveSrv::Request& service_request, sick_lidar_localization::SickLocReflectorsForSupportActiveSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocReflectorsForSupportActive */
    virtual bool serviceCbLocReflectorsForSupportActiveROS2(std::shared_ptr<sick_lidar_localization::SickLocReflectorsForSupportActiveSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocReflectorsForSupportActiveSrv::Response> service_response)
    {
      return serviceCbLocReflectorsForSupportActive(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocSetOdometryActiveSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocSetOdometryActive(sick_lidar_localization::SickLocSetOdometryActiveSrv::Request& service_request, sick_lidar_localization::SickLocSetOdometryActiveSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocSetOdometryActive */
    virtual bool serviceCbLocSetOdometryActiveROS2(std::shared_ptr<sick_lidar_localization::SickLocSetOdometryActiveSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocSetOdometryActiveSrv::Response> service_response)
    {
      return serviceCbLocSetOdometryActive(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocOdometryActiveSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocOdometryActive(sick_lidar_localization::SickLocOdometryActiveSrv::Request& service_request, sick_lidar_localization::SickLocOdometryActiveSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocOdometryActive */
    virtual bool serviceCbLocOdometryActiveROS2(std::shared_ptr<sick_lidar_localization::SickLocOdometryActiveSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocOdometryActiveSrv::Response> service_response)
    {
      return serviceCbLocOdometryActive(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocSetOdometryPortSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocSetOdometryPort(sick_lidar_localization::SickLocSetOdometryPortSrv::Request& service_request, sick_lidar_localization::SickLocSetOdometryPortSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocSetOdometryPort */
    virtual bool serviceCbLocSetOdometryPortROS2(std::shared_ptr<sick_lidar_localization::SickLocSetOdometryPortSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocSetOdometryPortSrv::Response> service_response)
    {
      return serviceCbLocSetOdometryPort(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocOdometryPortSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocOdometryPort(sick_lidar_localization::SickLocOdometryPortSrv::Request& service_request, sick_lidar_localization::SickLocOdometryPortSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocOdometryPort */
    virtual bool serviceCbLocOdometryPortROS2(std::shared_ptr<sick_lidar_localization::SickLocOdometryPortSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocOdometryPortSrv::Response> service_response)
    {
      return serviceCbLocOdometryPort(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocSetOdometryRestrictYMotionSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocSetOdometryRestrictYMotion(sick_lidar_localization::SickLocSetOdometryRestrictYMotionSrv::Request& service_request, sick_lidar_localization::SickLocSetOdometryRestrictYMotionSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocSetOdometryRestrictYMotion */
    virtual bool serviceCbLocSetOdometryRestrictYMotionROS2(std::shared_ptr<sick_lidar_localization::SickLocSetOdometryRestrictYMotionSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocSetOdometryRestrictYMotionSrv::Response> service_response)
    {
      return serviceCbLocSetOdometryRestrictYMotion(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocOdometryRestrictYMotionSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocOdometryRestrictYMotion(sick_lidar_localization::SickLocOdometryRestrictYMotionSrv::Request& service_request, sick_lidar_localization::SickLocOdometryRestrictYMotionSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocOdometryRestrictYMotion */
    virtual bool serviceCbLocOdometryRestrictYMotionROS2(std::shared_ptr<sick_lidar_localization::SickLocOdometryRestrictYMotionSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocOdometryRestrictYMotionSrv::Response> service_response)
    {
      return serviceCbLocOdometryRestrictYMotion(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocSetAutoStartActiveSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocSetAutoStartActive(sick_lidar_localization::SickLocSetAutoStartActiveSrv::Request& service_request, sick_lidar_localization::SickLocSetAutoStartActiveSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocSetAutoStartActive */
    virtual bool serviceCbLocSetAutoStartActiveROS2(std::shared_ptr<sick_lidar_localization::SickLocSetAutoStartActiveSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocSetAutoStartActiveSrv::Response> service_response)
    {
      return serviceCbLocSetAutoStartActive(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocAutoStartActiveSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocAutoStartActive(sick_lidar_localization::SickLocAutoStartActiveSrv::Request& service_request, sick_lidar_localization::SickLocAutoStartActiveSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocAutoStartActive */
    virtual bool serviceCbLocAutoStartActiveROS2(std::shared_ptr<sick_lidar_localization::SickLocAutoStartActiveSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocAutoStartActiveSrv::Response> service_response)
    {
      return serviceCbLocAutoStartActive(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocSetAutoStartSavePoseIntervalSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocSetAutoStartSavePoseInterval(sick_lidar_localization::SickLocSetAutoStartSavePoseIntervalSrv::Request& service_request, sick_lidar_localization::SickLocSetAutoStartSavePoseIntervalSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocSetAutoStartSavePoseInterval */
    virtual bool serviceCbLocSetAutoStartSavePoseIntervalROS2(std::shared_ptr<sick_lidar_localization::SickLocSetAutoStartSavePoseIntervalSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocSetAutoStartSavePoseIntervalSrv::Response> service_response)
    {
      return serviceCbLocSetAutoStartSavePoseInterval(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocAutoStartSavePoseIntervalSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocAutoStartSavePoseInterval(sick_lidar_localization::SickLocAutoStartSavePoseIntervalSrv::Request& service_request, sick_lidar_localization::SickLocAutoStartSavePoseIntervalSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocAutoStartSavePoseInterval */
    virtual bool serviceCbLocAutoStartSavePoseIntervalROS2(std::shared_ptr<sick_lidar_localization::SickLocAutoStartSavePoseIntervalSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocAutoStartSavePoseIntervalSrv::Response> service_response)
    {
      return serviceCbLocAutoStartSavePoseInterval(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocSetRingBufferRecordingActiveSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocSetRingBufferRecordingActive(sick_lidar_localization::SickLocSetRingBufferRecordingActiveSrv::Request& service_request, sick_lidar_localization::SickLocSetRingBufferRecordingActiveSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocSetRingBufferRecordingActive */
    virtual bool serviceCbLocSetRingBufferRecordingActiveROS2(std::shared_ptr<sick_lidar_localization::SickLocSetRingBufferRecordingActiveSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocSetRingBufferRecordingActiveSrv::Response> service_response)
    {
      return serviceCbLocSetRingBufferRecordingActive(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocRingBufferRecordingActiveSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocRingBufferRecordingActive(sick_lidar_localization::SickLocRingBufferRecordingActiveSrv::Request& service_request, sick_lidar_localization::SickLocRingBufferRecordingActiveSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocRingBufferRecordingActive */
    virtual bool serviceCbLocRingBufferRecordingActiveROS2(std::shared_ptr<sick_lidar_localization::SickLocRingBufferRecordingActiveSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocRingBufferRecordingActiveSrv::Response> service_response)
    {
      return serviceCbLocRingBufferRecordingActive(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickDevGetLidarIdentSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbDevGetLidarIdent(sick_lidar_localization::SickDevGetLidarIdentSrv::Request& service_request, sick_lidar_localization::SickDevGetLidarIdentSrv::Response& service_response);
    /*! ROS2 version of function serviceCbDevGetLidarIdent */
    virtual bool serviceCbDevGetLidarIdentROS2(std::shared_ptr<sick_lidar_localization::SickDevGetLidarIdentSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickDevGetLidarIdentSrv::Response> service_response)
    {
      return serviceCbDevGetLidarIdent(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickDevGetLidarStateSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbDevGetLidarState(sick_lidar_localization::SickDevGetLidarStateSrv::Request& service_request, sick_lidar_localization::SickDevGetLidarStateSrv::Response& service_response);
    /*! ROS2 version of function serviceCbDevGetLidarState */
    virtual bool serviceCbDevGetLidarStateROS2(std::shared_ptr<sick_lidar_localization::SickDevGetLidarStateSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickDevGetLidarStateSrv::Response> service_response)
    {
      return serviceCbDevGetLidarState(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickGetSoftwareVersionSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbGetSoftwareVersion(sick_lidar_localization::SickGetSoftwareVersionSrv::Request& service_request, sick_lidar_localization::SickGetSoftwareVersionSrv::Response& service_response);
    /*! ROS2 version of function serviceCbGetSoftwareVersion */
    virtual bool serviceCbGetSoftwareVersionROS2(std::shared_ptr<sick_lidar_localization::SickGetSoftwareVersionSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickGetSoftwareVersionSrv::Response> service_response)
    {
      return serviceCbGetSoftwareVersion(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocAutoStartSavePoseSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocAutoStartSavePose(sick_lidar_localization::SickLocAutoStartSavePoseSrv::Request& service_request, sick_lidar_localization::SickLocAutoStartSavePoseSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocAutoStartSavePose */
    virtual bool serviceCbLocAutoStartSavePoseROS2(std::shared_ptr<sick_lidar_localization::SickLocAutoStartSavePoseSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocAutoStartSavePoseSrv::Response> service_response)
    {
      return serviceCbLocAutoStartSavePose(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocForceUpdateSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocForceUpdate(sick_lidar_localization::SickLocForceUpdateSrv::Request& service_request, sick_lidar_localization::SickLocForceUpdateSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocForceUpdate */
    virtual bool serviceCbLocForceUpdateROS2(std::shared_ptr<sick_lidar_localization::SickLocForceUpdateSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocForceUpdateSrv::Response> service_response)
    {
      return serviceCbLocForceUpdate(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocSaveRingBufferRecordingSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocSaveRingBufferRecording(sick_lidar_localization::SickLocSaveRingBufferRecordingSrv::Request& service_request, sick_lidar_localization::SickLocSaveRingBufferRecordingSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocSaveRingBufferRecording */
    virtual bool serviceCbLocSaveRingBufferRecordingROS2(std::shared_ptr<sick_lidar_localization::SickLocSaveRingBufferRecordingSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocSaveRingBufferRecordingSrv::Response> service_response)
    {
      return serviceCbLocSaveRingBufferRecording(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocStartDemoMappingSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocStartDemoMapping(sick_lidar_localization::SickLocStartDemoMappingSrv::Request& service_request, sick_lidar_localization::SickLocStartDemoMappingSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocStartDemoMapping */
    virtual bool serviceCbLocStartDemoMappingROS2(std::shared_ptr<sick_lidar_localization::SickLocStartDemoMappingSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocStartDemoMappingSrv::Response> service_response)
    {
      return serviceCbLocStartDemoMapping(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickReportUserMessageSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbReportUserMessage(sick_lidar_localization::SickReportUserMessageSrv::Request& service_request, sick_lidar_localization::SickReportUserMessageSrv::Response& service_response);
    /*! ROS2 version of function serviceCbReportUserMessage */
    virtual bool serviceCbReportUserMessageROS2(std::shared_ptr<sick_lidar_localization::SickReportUserMessageSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickReportUserMessageSrv::Response> service_response)
    {
      return serviceCbReportUserMessage(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickSavePermanentSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbSavePermanent(sick_lidar_localization::SickSavePermanentSrv::Request& service_request, sick_lidar_localization::SickSavePermanentSrv::Response& service_response);
    /*! ROS2 version of function serviceCbSavePermanent */
    virtual bool serviceCbSavePermanentROS2(std::shared_ptr<sick_lidar_localization::SickSavePermanentSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickSavePermanentSrv::Response> service_response)
    {
      return serviceCbSavePermanent(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocResultPortSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocResultPort(sick_lidar_localization::SickLocResultPortSrv::Request& service_request, sick_lidar_localization::SickLocResultPortSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocResultPort */
    virtual bool serviceCbLocResultPortROS2(std::shared_ptr<sick_lidar_localization::SickLocResultPortSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocResultPortSrv::Response> service_response)
    {
      return serviceCbLocResultPort(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocResultModeSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocResultMode(sick_lidar_localization::SickLocResultModeSrv::Request& service_request, sick_lidar_localization::SickLocResultModeSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocResultMode */
    virtual bool serviceCbLocResultModeROS2(std::shared_ptr<sick_lidar_localization::SickLocResultModeSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocResultModeSrv::Response> service_response)
    {
      return serviceCbLocResultMode(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocResultEndiannessSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocResultEndianness(sick_lidar_localization::SickLocResultEndiannessSrv::Request& service_request, sick_lidar_localization::SickLocResultEndiannessSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocResultEndianness */
    virtual bool serviceCbLocResultEndiannessROS2(std::shared_ptr<sick_lidar_localization::SickLocResultEndiannessSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocResultEndiannessSrv::Response> service_response)
    {
      return serviceCbLocResultEndianness(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocResultStateSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocResultState(sick_lidar_localization::SickLocResultStateSrv::Request& service_request, sick_lidar_localization::SickLocResultStateSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocResultState */
    virtual bool serviceCbLocResultStateROS2(std::shared_ptr<sick_lidar_localization::SickLocResultStateSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocResultStateSrv::Response> service_response)
    {
      return serviceCbLocResultState(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickLocResultPoseIntervalSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbLocResultPoseInterval(sick_lidar_localization::SickLocResultPoseIntervalSrv::Request& service_request, sick_lidar_localization::SickLocResultPoseIntervalSrv::Response& service_response);
    /*! ROS2 version of function serviceCbLocResultPoseInterval */
    virtual bool serviceCbLocResultPoseIntervalROS2(std::shared_ptr<sick_lidar_localization::SickLocResultPoseIntervalSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocResultPoseIntervalSrv::Response> service_response)
    {
      return serviceCbLocResultPoseInterval(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickDevSetIMUActiveSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbDevSetIMUActive(sick_lidar_localization::SickDevSetIMUActiveSrv::Request& service_request, sick_lidar_localization::SickDevSetIMUActiveSrv::Response& service_response);
    /*! ROS2 version of function serviceCbDevSetIMUActive */
    virtual bool serviceCbDevSetIMUActiveROS2(std::shared_ptr<sick_lidar_localization::SickDevSetIMUActiveSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickDevSetIMUActiveSrv::Response> service_response)
    {
      return serviceCbDevSetIMUActive(*service_request, *service_response);
    }

    /*!
     * Callback for service "SickDevIMUActiveSrv"
     * Converts the service request to cola telegram, sends the telegram to the localization controller and receives the response
     * Uses ros service "SickLocColaTelegramSrv"
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors (negative response from localization controller, timeout, service or communication error).
     */
    virtual bool serviceCbDevIMUActive(sick_lidar_localization::SickDevIMUActiveSrv::Request& service_request, sick_lidar_localization::SickDevIMUActiveSrv::Response& service_response);
    /*! ROS2 version of function serviceCbDevIMUActive */
    virtual bool serviceCbDevIMUActiveROS2(std::shared_ptr<sick_lidar_localization::SickDevIMUActiveSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickDevIMUActiveSrv::Response> service_response)
    {
      return serviceCbDevIMUActive(*service_request, *service_response);
    }

  protected:
  
    /*!
     * Sends a cola telegram using ros service "SickLocColaTelegram", receives and returns the response from localization controller
     * @param[in] cola_ascii_request request (Cola-ASCII, f.e. "sMN IsSystemReady")
     * @return response from localization controller
     */
    virtual sick_lidar_localization::SickLocColaTelegramMsg sendColaTelegram(const std::string & cola_ascii_request);

    /*!
     * Sends a cola telegram using ros service "SickLocColaTelegram", receives the response from localization controller
     * and sets service_response.success to the parameter value of the recived response. Specialization of function
     * sendColaTelegram() for Cola-ASCII telegrams returning a single success value.
     * @param[in] cola_command_type request type (Cola-ASCII, f.e. "sMN")
     * @param[in] cola_command_name request name (Cola-ASCII, f.e. "IsSystemReady")
     * @param[in] cola_command_args request arguments (Cola-ASCII, f.e. "")
     * @param[out] service_response service_response.success set to response value from localization controller
     * @return true on success, false in case of errors.
     */
    template<typename ResponseType> bool serviceCbWithSuccessResponse(const std::string & cola_command_type, const std::string & cola_command_name,  const std::string & cola_command_args, ResponseType &service_response)
    {
      service_response.success = false;
      std::stringstream cola_ascii;
      cola_ascii << cola_command_type << " " << cola_command_name;
      if(cola_command_args != "")
        cola_ascii << " " << cola_command_args;
      sick_lidar_localization::SickLocColaTelegramMsg cola_response = sendColaTelegram(cola_ascii.str());
      if(cola_response.command_name == cola_command_name && cola_response.parameter.size() == 1)
      {
        service_response.success = sick_lidar_localization::ColaParser::convertColaResponseBool(cola_response.parameter[0], false);
        return true;
      }
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(\"" << cola_ascii.str() << "\") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
      return false;
    }
  
    /*
     * member variables
     */
  
    ROS::NodePtr m_nh;                                                              ///< ros node
    sick_lidar_localization::DriverMonitor* m_driver_monitor;                       ///< implements the cola telegram services
    double m_cola_response_timeout;                                                 ///< Timeout in seconds for cola responses from localization controller, default: 1
    boost::mutex m_service_cb_mutex;                                                ///< Mutex to protect sendColaTelegram (one service request at a time)
    sick_lidar_localization::SickLocColaTelegramSrvClient m_service_client;         ///< ROS client to call service "SickLocColaTelegram" to send cola telegrams and receive cola responses from localization controller
    /** list of ros service provider for services listed above (release 3 and later) */
    sick_lidar_localization::SickLocIsSystemReadySrvServer         m_srv_server_01; ///< "SickLocIsSystemReady",&sick_lidar_localization::ColaServices::serviceCbIsSystemReady
    sick_lidar_localization::SickLocStateSrvServer                 m_srv_server_02; ///< "SickLocState",&sick_lidar_localization::ColaServices::serviceCbLocState
    sick_lidar_localization::SickLocStartLocalizingSrvServer       m_srv_server_03; ///< "SickLocStartLocalizing",&sick_lidar_localization::ColaServices::serviceCbLocStartLocalizing
    sick_lidar_localization::SickLocStopSrvServer                  m_srv_server_04; ///< "SickLocStop",&sick_lidar_localization::ColaServices::serviceCbLocStop
    sick_lidar_localization::SickLocSetResultPortSrvServer         m_srv_server_06; ///< "SickLocSetResultPort",&sick_lidar_localization::ColaServices::serviceCbLocSetResultPort
    sick_lidar_localization::SickLocSetResultModeSrvServer         m_srv_server_07; ///< "SickLocSetResultMode",&sick_lidar_localization::ColaServices::serviceCbLocSetResultMode
    sick_lidar_localization::SickLocSetResultPoseEnabledSrvServer  m_srv_server_08; ///< "SickLocSetResultPoseEnabled",&sick_lidar_localization::ColaServices::serviceCbLocSetResultPoseEnabled
    sick_lidar_localization::SickLocSetResultEndiannessSrvServer   m_srv_server_09; ///< "SickLocSetResultEndianness",&sick_lidar_localization::ColaServices::serviceCbLocSetResultEndianness
    sick_lidar_localization::SickLocSetResultPoseIntervalSrvServer m_srv_server_10; ///< "SickLocSetResultPoseInterval",&sick_lidar_localization::ColaServices::serviceCbLocSetResultPoseInterval
    sick_lidar_localization::SickLocRequestResultDataSrvServer     m_srv_server_11; ///< "SickLocRequestResultData",&sick_lidar_localization::ColaServices::serviceCbLocRequestResultData
    sick_lidar_localization::SickLocSetPoseSrvServer               m_srv_server_12; ///< "SickLocSetPose",&sick_lidar_localization::ColaServices::serviceCbLocSetPose
    /** list of ros service provider for services listed above (release 4 and later) */
    sick_lidar_localization::SickDevSetLidarConfigSrvServer m_SickDevSetLidarConfigSrvServer; ///< service "DevSetLidarConfig", callback &sick_lidar_localization::ColaServices::serviceCbDevSetLidarConfig
    sick_lidar_localization::SickDevGetLidarConfigSrvServer m_SickDevGetLidarConfigSrvServer; ///< service "DevGetLidarConfig", callback &sick_lidar_localization::ColaServices::serviceCbDevGetLidarConfig
    sick_lidar_localization::SickLocSetMapSrvServer m_SickLocSetMapSrvServer; ///< service "LocSetMap", callback &sick_lidar_localization::ColaServices::serviceCbLocSetMap
    sick_lidar_localization::SickLocMapSrvServer m_SickLocMapSrvServer; ///< service "LocMap", callback &sick_lidar_localization::ColaServices::serviceCbLocMap
    sick_lidar_localization::SickLocMapStateSrvServer m_SickLocMapStateSrvServer; ///< service "LocMapState", callback &sick_lidar_localization::ColaServices::serviceCbLocMapState
    sick_lidar_localization::SickLocInitializePoseSrvServer m_SickLocInitializePoseSrvServer; ///< service "LocInitializePose", callback &sick_lidar_localization::ColaServices::serviceCbLocInitializePose
    sick_lidar_localization::SickLocInitialPoseSrvServer m_SickLocInitialPoseSrvServer; ///< service "LocInitialPose", callback &sick_lidar_localization::ColaServices::serviceCbLocInitialPose
    sick_lidar_localization::SickLocSetReflectorsForSupportActiveSrvServer m_SickLocSetReflectorsForSupportActiveSrvServer; ///< service "LocSetReflectorsForSupportActive", callback &sick_lidar_localization::ColaServices::serviceCbLocSetReflectorsForSupportActive
    sick_lidar_localization::SickLocReflectorsForSupportActiveSrvServer m_SickLocReflectorsForSupportActiveSrvServer; ///< service "LocReflectorsForSupportActive", callback &sick_lidar_localization::ColaServices::serviceCbLocReflectorsForSupportActive
    sick_lidar_localization::SickLocSetOdometryActiveSrvServer m_SickLocSetOdometryActiveSrvServer; ///< service "LocSetOdometryActive", callback &sick_lidar_localization::ColaServices::serviceCbLocSetOdometryActive
    sick_lidar_localization::SickLocOdometryActiveSrvServer m_SickLocOdometryActiveSrvServer; ///< service "LocOdometryActive", callback &sick_lidar_localization::ColaServices::serviceCbLocOdometryActive
    sick_lidar_localization::SickLocSetOdometryPortSrvServer m_SickLocSetOdometryPortSrvServer; ///< service "LocSetOdometryPort", callback &sick_lidar_localization::ColaServices::serviceCbLocSetOdometryPort
    sick_lidar_localization::SickLocOdometryPortSrvServer m_SickLocOdometryPortSrvServer; ///< service "LocOdometryPort", callback &sick_lidar_localization::ColaServices::serviceCbLocOdometryPort
    sick_lidar_localization::SickLocSetOdometryRestrictYMotionSrvServer m_SickLocSetOdometryRestrictYMotionSrvServer; ///< service "LocSetOdometryRestrictYMotion", callback &sick_lidar_localization::ColaServices::serviceCbLocSetOdometryRestrictYMotion
    sick_lidar_localization::SickLocOdometryRestrictYMotionSrvServer m_SickLocOdometryRestrictYMotionSrvServer; ///< service "LocOdometryRestrictYMotion", callback &sick_lidar_localization::ColaServices::serviceCbLocOdometryRestrictYMotion
    sick_lidar_localization::SickLocSetAutoStartActiveSrvServer m_SickLocSetAutoStartActiveSrvServer; ///< service "LocSetAutoStartActive", callback &sick_lidar_localization::ColaServices::serviceCbLocSetAutoStartActive
    sick_lidar_localization::SickLocAutoStartActiveSrvServer m_SickLocAutoStartActiveSrvServer; ///< service "LocAutoStartActive", callback &sick_lidar_localization::ColaServices::serviceCbLocAutoStartActive
    sick_lidar_localization::SickLocSetAutoStartSavePoseIntervalSrvServer m_SickLocSetAutoStartSavePoseIntervalSrvServer; ///< service "LocSetAutoStartSavePoseInterval", callback &sick_lidar_localization::ColaServices::serviceCbLocSetAutoStartSavePoseInterval
    sick_lidar_localization::SickLocAutoStartSavePoseIntervalSrvServer m_SickLocAutoStartSavePoseIntervalSrvServer; ///< service "LocAutoStartSavePoseInterval", callback &sick_lidar_localization::ColaServices::serviceCbLocAutoStartSavePoseInterval
    sick_lidar_localization::SickLocSetRingBufferRecordingActiveSrvServer m_SickLocSetRingBufferRecordingActiveSrvServer; ///< service "LocSetRingBufferRecordingActive", callback &sick_lidar_localization::ColaServices::serviceCbLocSetRingBufferRecordingActive
    sick_lidar_localization::SickLocRingBufferRecordingActiveSrvServer m_SickLocRingBufferRecordingActiveSrvServer; ///< service "LocRingBufferRecordingActive", callback &sick_lidar_localization::ColaServices::serviceCbLocRingBufferRecordingActive
    sick_lidar_localization::SickDevGetLidarIdentSrvServer m_SickDevGetLidarIdentSrvServer; ///< service "DevGetLidarIdent", callback &sick_lidar_localization::ColaServices::serviceCbDevGetLidarIdent
    sick_lidar_localization::SickDevGetLidarStateSrvServer m_SickDevGetLidarStateSrvServer; ///< service "DevGetLidarState", callback &sick_lidar_localization::ColaServices::serviceCbDevGetLidarState
    sick_lidar_localization::SickGetSoftwareVersionSrvServer m_SickGetSoftwareVersionSrvServer; ///< service "GetSoftwareVersion", callback &sick_lidar_localization::ColaServices::serviceCbGetSoftwareVersion
    sick_lidar_localization::SickLocAutoStartSavePoseSrvServer m_SickLocAutoStartSavePoseSrvServer; ///< service "LocAutoStartSavePose", callback &sick_lidar_localization::ColaServices::serviceCbLocAutoStartSavePose
    sick_lidar_localization::SickLocForceUpdateSrvServer m_SickLocForceUpdateSrvServer; ///< service "LocForceUpdate", callback &sick_lidar_localization::ColaServices::serviceCbLocForceUpdate
    sick_lidar_localization::SickLocSaveRingBufferRecordingSrvServer m_SickLocSaveRingBufferRecordingSrvServer; ///< service "LocSaveRingBufferRecording", callback &sick_lidar_localization::ColaServices::serviceCbLocSaveRingBufferRecording
    sick_lidar_localization::SickLocStartDemoMappingSrvServer m_SickLocStartDemoMappingSrvServer; ///< service "LocStartDemoMapping", callback &sick_lidar_localization::ColaServices::serviceCbLocStartDemoMapping
    sick_lidar_localization::SickReportUserMessageSrvServer m_SickReportUserMessageSrvServer; ///< service "ReportUserMessage", callback &sick_lidar_localization::ColaServices::serviceCbReportUserMessage
    sick_lidar_localization::SickSavePermanentSrvServer m_SickSavePermanentSrvServer; ///< service "SavePermanent", callback &sick_lidar_localization::ColaServices::serviceCbSavePermanent
    sick_lidar_localization::SickLocResultPortSrvServer m_SickLocResultPortSrvServer; ///< service "LocResultPort", callback &sick_lidar_localization::ColaServices::serviceCbLocResultPort
    sick_lidar_localization::SickLocResultModeSrvServer m_SickLocResultModeSrvServer; ///< service "LocResultMode", callback &sick_lidar_localization::ColaServices::serviceCbLocResultMode
    sick_lidar_localization::SickLocResultEndiannessSrvServer m_SickLocResultEndiannessSrvServer; ///< service "LocResultEndianness", callback &sick_lidar_localization::ColaServices::serviceCbLocResultEndianness
    sick_lidar_localization::SickLocResultStateSrvServer m_SickLocResultStateSrvServer; ///< service "LocResultState", callback &sick_lidar_localization::ColaServices::serviceCbLocResultState
    sick_lidar_localization::SickLocResultPoseIntervalSrvServer m_SickLocResultPoseIntervalSrvServer; ///< service "LocResultPoseInterval", callback &sick_lidar_localization::ColaServices::serviceCbLocResultPoseInterval
    sick_lidar_localization::SickDevSetIMUActiveSrvServer m_SickDevSetIMUActiveSrvServer; ///< service "DevSetIMUActive", callback &sick_lidar_localization::ColaServices::serviceCbDevSetIMUActive
    sick_lidar_localization::SickDevIMUActiveSrvServer m_SickDevIMUActiveSrvServer; ///< service "DevIMUActive", callback &sick_lidar_localization::ColaServices::serviceCbSetIMUActive
  }; // class ColaServices
  
} // namespace sick_lidar_localization
#endif // __SIM_LOC_COLA_SERVICES_H_INCLUDED
