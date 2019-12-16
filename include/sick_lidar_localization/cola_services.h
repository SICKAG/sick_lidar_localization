/*
 * @brief cola_services implements the following ros services for sick_lidar_localization:
 *
 * ROS service name               | Definition file                         | Cola request telegram                                | Description
 * "SickLocIsSystemReady"         | srv/SickLocIsSystemReadySrv.srv         | "sMN IsSystemReady"                                  | Check if the system is ready
 * "SickLocState"                 | srv/SickLocStateSrv.srv                 | "sRN LocState"                                       | Read localization state
 * "SickLocStartLocalizing"       | srv/SickLocStartLocalizingSrv.srv       | "sMN LocStartLocalizing"                             | Start localization
 * "SickLocStop"                  | srv/SickLocStopSrv.srv                  | "sMN LocStop"                                        | Stop localization or demo mapping
 * "SickLocStopAndSave"           | srv/SickLocStopAndSaveSrv.srv           | "sMN LocStopAndSave"                                 | Stop localization, save settings
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

#include "sick_lidar_localization/cola_parser.h"
#include "sick_lidar_localization/SickLocColaTelegramSrv.h"
#include "sick_lidar_localization/SickLocIsSystemReadySrv.h"
#include "sick_lidar_localization/SickLocStateSrv.h"
#include "sick_lidar_localization/SickLocStartLocalizingSrv.h"
#include "sick_lidar_localization/SickLocStopSrv.h"
#include "sick_lidar_localization/SickLocStopAndSaveSrv.h"
#include "sick_lidar_localization/SickLocSetResultPortSrv.h"
#include "sick_lidar_localization/SickLocSetResultModeSrv.h"
#include "sick_lidar_localization/SickLocSetResultPoseEnabledSrv.h"
#include "sick_lidar_localization/SickLocSetResultEndiannessSrv.h"
#include "sick_lidar_localization/SickLocSetResultPoseIntervalSrv.h"
#include "sick_lidar_localization/SickLocRequestResultDataSrv.h"
#include "sick_lidar_localization/SickLocSetPoseSrv.h"
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
   * "SickLocStopAndSave"           | srv/SickLocStopAndSaveSrv.srv           | "sMN LocStopAndSave"                                 | Stop localization, save settings
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
    ColaServices(ros::NodeHandle* nh = 0);
    
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
  
    /*!
     * Callback for service messages (SickLocStateSrv, Read localization state).
     * Sends a cola telegram "sRN LocState" and receives the response from localization controller
     * using ros service "SickLocColaTelegramSrv".
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors.
     */
    virtual bool serviceCbLocState(sick_lidar_localization::SickLocStateSrv::Request & service_request, sick_lidar_localization::SickLocStateSrv::Response & service_response);
  
    /*!
     * Callback for service messages (SickLocStartLocalizingSrv, Stop localization, save settings).
     * Sends a cola telegram "Stop localization, save settings" and receives the response from localization controller
     * using ros service "SickLocColaTelegramSrv".
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors.
     */
    virtual bool serviceCbLocStartLocalizing(sick_lidar_localization::SickLocStartLocalizingSrv::Request & service_request, sick_lidar_localization::SickLocStartLocalizingSrv::Response & service_response);
  
    /*!
     * Callback for service messages (SickLocStopSrv, Stop localization, save settings).
     * Sends a cola telegram "Stop localization, save settings" and receives the response from localization controller
     * using ros service "SickLocColaTelegramSrv".
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors.
     */
    virtual bool serviceCbLocStop(sick_lidar_localization::SickLocStopSrv::Request & service_request, sick_lidar_localization::SickLocStopSrv::Response & service_response);
  
    /*!
     * Callback for service messages (SickLocStopAndSaveSrv, Stop localization, save settings).
     * Sends a cola telegram "sMN LocStopAndSave" and receives the response from localization controller
     * using ros service "SickLocColaTelegramSrv".
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors.
     */
    virtual bool serviceCbLocStopAndSave(sick_lidar_localization::SickLocStopAndSaveSrv::Request & service_request, sick_lidar_localization::SickLocStopAndSaveSrv::Response & service_response);
  
    /*!
     * Callback for service messages (SickLocSetResultPortSrv, Set TCP-port for result output, default: 2201).
     * Sends a cola telegram "Set mode of result output (default: stream" and receives the response from localization controller
     * using ros service "SickLocColaTelegramSrv".
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors.
     */
    virtual bool serviceCbLocSetResultPort(sick_lidar_localization::SickLocSetResultPortSrv::Request & service_request, sick_lidar_localization::SickLocSetResultPortSrv::Response & service_response);
  
    /*!
     * Callback for service messages (SickLocSetResultModeSrv, Set mode of result output, default: stream).
     * Sends a cola telegram "sMN LocSetResultMode <mode>" and receives the response from localization controller
     * using ros service "SickLocColaTelegramSrv".
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors.
     */
    virtual bool serviceCbLocSetResultMode(sick_lidar_localization::SickLocSetResultModeSrv::Request & service_request, sick_lidar_localization::SickLocSetResultModeSrv::Response & service_response);
  
    /*!
     * Callback for service messages (SickLocSetResultPoseEnabledSrv, Disable/enable result output).
     * Sends a cola telegram "sMN LocSetResultPoseEnabled <enabled>" and receives the response from localization controller
     * using ros service "SickLocColaTelegramSrv".
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors.
     */
    virtual bool serviceCbLocSetResultPoseEnabled(sick_lidar_localization::SickLocSetResultPoseEnabledSrv::Request & service_request, sick_lidar_localization::SickLocSetResultPoseEnabledSrv::Response & service_response);
  
    /*!
     * Callback for service messages (SickLocSetResultEndiannessSrv, Set endianness of result output).
     * Sends a cola telegram "sMN LocSetResultEndianness <endianness>" and receives the response from localization controller
     * using ros service "SickLocColaTelegramSrv".
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors.
     */
    virtual bool serviceCbLocSetResultEndianness(sick_lidar_localization::SickLocSetResultEndiannessSrv::Request & service_request, sick_lidar_localization::SickLocSetResultEndiannessSrv::Response & service_response);
  
    /*!
     * Callback for service messages (SickLocSetResultPoseIntervalSrv, Set interval of result output).
     * Sends a cola telegram "sMN LocSetResultPoseInterval <interval>" and receives the response from localization controller
     * using ros service "SickLocColaTelegramSrv".
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors.
     */
    virtual bool serviceCbLocSetResultPoseInterval(sick_lidar_localization::SickLocSetResultPoseIntervalSrv::Request & service_request, sick_lidar_localization::SickLocSetResultPoseIntervalSrv::Response & service_response);
  
    /*!
     * Callback for service messages (SickLocRequestResultDataSrv,  If in poll mode, trigger sending the localization result of the next processed scan via TCP interface).
     * Sends a cola telegram "sMN LocRequestResultData" and receives the response from localization controller
     * using ros service "SickLocColaTelegramSrv".
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors.
     */
    virtual bool serviceCbLocRequestResultData(sick_lidar_localization::SickLocRequestResultDataSrv::Request & service_request, sick_lidar_localization::SickLocRequestResultDataSrv::Response & service_response);
  
    /*!
     * Callback for service messages (SickLocSetPoseSrv,  Initialize vehicle pose).
     * Sends a cola telegram "sMN LocSetPose <posex> <posey> <yaw> <uncertainty>" and receives the response from localization controller
     * using ros service "SickLocColaTelegramSrv".
     * @param[in] service_request ros service request to localization controller
     * @param[out] service_response service response from localization controller
     * @return true on success, false in case of errors.
     */
    virtual bool serviceCbLocSetPose(sick_lidar_localization::SickLocSetPoseSrv::Request & service_request, sick_lidar_localization::SickLocSetPoseSrv::Response & service_response);
  
  protected:
  
    /*!
     * Sends a cola telegram using ros service "SickLocColaTelegram", receives and returns the response from localization controller
     * @param[in] cola_ascii_request request (Cola-ASCII, f.e. "sMN IsSystemReady")
     * @return response from localization controller
     */
    virtual sick_lidar_localization::SickLocColaTelegramMsg sendColaTelegram(const std::string & cola_ascii_request);

    /*!
     * Converts and returns the parameter of a cola ascii telegram into a numeric value.
     * @param[in] cola_arg parameter of a cola ascii telegram
     * @param[in] base numeric base (10 for decimal values or 16 for hex strings)
     * @param[in] default_value default value returned in case of parse errors
     * @return parameter converted to integer value
     */
    virtual int32_t convertColaArg(const std::string & cola_arg, int base = 10, int32_t default_value = 0);
    
    /*!
     * Converts and returns the parameter of a cola ascii response into a boolean value.
     * @param[in] cola_response_arg parameter of a cola ascii response
     * @param[in] default_value default value returned in case of parse errors
     * @return parameter converted to boolean value
     */
    virtual bool convertColaResponseBool(const std::string & cola_response_arg, bool default_value);

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
        service_response.success = convertColaResponseBool(cola_response.parameter[0], false);
        return true;
      }
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(\"" << cola_ascii.str() << "\") failed, invalid response: " << sick_lidar_localization::Utils::flattenToString(cola_response));
      return false;
    }
  
    /*
     * member variables
     */
  
    double m_cola_response_timeout;                    ///< Timeout in seconds for cola responses from localization controller, default: 1
    std::vector<ros::ServiceServer> m_service_server;  ///< list of ros service provider for services listed above
    ros::ServiceClient m_service_client;               ///< client to call ros service "SickLocColaTelegram" to send cola telegrams and receive cola responses from localization controller
  
  }; // class ColaServices
  
} // namespace sick_lidar_localization
#endif // __SIM_LOC_COLA_SERVICES_H_INCLUDED
