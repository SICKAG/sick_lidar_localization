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
#include <ros/ros.h>

#include "sick_lidar_localization/cola_services.h"

/*!
 * Constructor
 */
sick_lidar_localization::ColaServices::ColaServices(ros::NodeHandle *nh) : m_cola_response_timeout(1.0)
{
  if(nh)
  {
    ros::param::param<double>("/sick_lidar_localization/time_sync/cola_response_timeout", m_cola_response_timeout, m_cola_response_timeout);
    // Advertise ros services
    m_service_server.push_back(nh->advertiseService("SickLocIsSystemReady", &sick_lidar_localization::ColaServices::serviceCbIsSystemReady, this));
    m_service_server.push_back(nh->advertiseService("SickLocState", &sick_lidar_localization::ColaServices::serviceCbLocState, this));
    m_service_server.push_back(nh->advertiseService("SickLocStartLocalizing", &sick_lidar_localization::ColaServices::serviceCbLocStartLocalizing, this));
    m_service_server.push_back(nh->advertiseService("SickLocStop", &sick_lidar_localization::ColaServices::serviceCbLocStop, this));
    m_service_server.push_back(nh->advertiseService("SickLocStopAndSave", &sick_lidar_localization::ColaServices::serviceCbLocStopAndSave, this));
    m_service_server.push_back(nh->advertiseService("SickLocSetResultPort", &sick_lidar_localization::ColaServices::serviceCbLocSetResultPort, this));
    m_service_server.push_back(nh->advertiseService("SickLocSetResultMode", &sick_lidar_localization::ColaServices::serviceCbLocSetResultMode, this));
    m_service_server.push_back(nh->advertiseService("SickLocSetResultPoseEnabled", &sick_lidar_localization::ColaServices::serviceCbLocSetResultPoseEnabled, this));
    m_service_server.push_back(nh->advertiseService("SickLocSetResultEndianness", &sick_lidar_localization::ColaServices::serviceCbLocSetResultEndianness, this));
    m_service_server.push_back(nh->advertiseService("SickLocSetResultPoseInterval", &sick_lidar_localization::ColaServices::serviceCbLocSetResultPoseInterval, this));
    m_service_server.push_back(nh->advertiseService("SickLocRequestResultData", &sick_lidar_localization::ColaServices::serviceCbLocRequestResultData, this));
    m_service_server.push_back(nh->advertiseService("SickLocSetPose", &sick_lidar_localization::ColaServices::serviceCbLocSetPose, this));
    // Clients for ros services "SickLocColaTelegram"
    m_service_client = nh->serviceClient<sick_lidar_localization::SickLocColaTelegramSrv>("SickLocColaTelegram");
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
  sick_lidar_localization::SickLocColaTelegramSrv cola_telegram;
  sick_lidar_localization::SickLocColaTelegramMsg cola_response;
  cola_telegram.request.cola_ascii_request = cola_ascii_request;
  cola_telegram.request.wait_response_timeout = m_cola_response_timeout;
  try
  {
    // Send cola telegram using ros service "SickLocColaTelegram", receive response from localization server
    if (!m_service_client.call(cola_telegram) || cola_telegram.response.cola_ascii_response.empty())
    {
      ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(): calling ros service \"SickLocColaTelegram\" failed with request: "
        << sick_lidar_localization::Utils::flattenToString(cola_telegram.request) << " response: " << sick_lidar_localization::Utils::flattenToString(cola_telegram.response));
      return cola_response;
    }
    ROS_DEBUG_STREAM("ColaServices::sendColaTelegram(): request " << sick_lidar_localization::Utils::flattenToString(cola_telegram.request)
      << " response: " << sick_lidar_localization::Utils::flattenToString(cola_telegram.response) << " succesfull.");
    // Decode and return response
    return sick_lidar_localization::ColaParser::decodeColaTelegram(cola_telegram.response.cola_ascii_response);
  }
  catch(const std::exception & exc)
  {
    ROS_WARN_STREAM("## ERROR ColaServices::sendColaTelegram(" << sick_lidar_localization::Utils::flattenToString(cola_telegram.request)
      << "): cola response " << sick_lidar_localization::Utils::flattenToString(cola_response) << ", exception " << exc.what());
  }
  return cola_response;
}

/*!
 * Converts and returns the parameter of a cola ascii telegram into a numeric value.
 * @param[in] cola_arg parameter of a cola ascii telegram
 * @param[in] base numeric base (10 for decimal values or 16 for hex strings)
 * @param[in] default_value default value returned in case of parse errors
 * @return parameter converted to integer value
 */
int32_t sick_lidar_localization::ColaServices::convertColaArg(const std::string & cola_arg, int base, int32_t default_value)
{
  try
  {
    return std::stoi(cola_arg, 0, base);
  }
  catch(const std::exception & exc)
  {
    ROS_WARN_STREAM("## ERROR ColaServices::convertColaArg(" << cola_arg << ") failed, exception " << exc.what());
  }
  return default_value;
  
}

/*!
 * Converts and returns the parameter of a cola ascii response into a boolean value.
 * @param[in] cola_response_arg parameter of a cola ascii response
 * @param[in] default_value default value returned in case of parse errors
 * @return parameter converted to boolean value
 */
bool sick_lidar_localization::ColaServices::convertColaResponseBool(const std::string & cola_response_arg, bool default_value)
{
  return ((convertColaArg(cola_response_arg, 10, (default_value ? 1 : 0)) > 0) ? true : false);
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
    service_response.state = convertColaArg(cola_response.parameter[0], 10, -1);
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
 * Callback for service messages (SickLocStopAndSaveSrv, Stop localization, save settings).
 * Sends a cola telegram "sMN LocStopAndSave" and receives the response from localization controller
 * using ros service "SickLocColaTelegramSrv".
 * @param[in] service_request ros service request to localization controller
 * @param[out] service_response service response from localization controller
 * @return true on success, false in case of errors.
 */
bool sick_lidar_localization::ColaServices::serviceCbLocStopAndSave(
  sick_lidar_localization::SickLocStopAndSaveSrv::Request &service_request,
  sick_lidar_localization::SickLocStopAndSaveSrv::Response &service_response)
{
  return serviceCbWithSuccessResponse("sMN", "LocStopAndSave", "", service_response);
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
    service_response.success = convertColaResponseBool(cola_response.parameter[0], false);
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
