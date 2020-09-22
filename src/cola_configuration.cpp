/*
 * @brief cola_configuration sets the initial SIM result output configuration
 * using ros cola services.
 * Configures the following result output settings from launch file:
 * LocSetResultPort, LocSetResultMode, LocSetResultPoseEnabled, LocSetResultEndianness, LocSetResultPoseInterval,
 * LocRequestResultData
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

#include "sick_lidar_localization/cola_configuration.h"

/*!
 * Constructor
 * @param[in] nh ros node handle
 * @param[in] cola_services cola service callbacks (converts requests to cola telegrams, sends the cola telegrams and decodes the response from localization server)
 */
sick_lidar_localization::ColaConfiguration::ColaConfiguration(ROS::NodePtr nh, sick_lidar_localization::ColaServices* cola_services)
: m_nh(nh), m_cola_services(cola_services), m_configuration_thread_running(false), m_configuration_thread(0)
{
}

/*!
 * Destructor
 */
sick_lidar_localization::ColaConfiguration::~ColaConfiguration()
{
  stop();
}

/*!
 * Starts transmitting the initial result output configuration to the localization controller.
 * Configures the following result output settings from launch file:
 * LocSetResultPort, LocSetResultMode, LocSetResultPoseEnabled, LocSetResultEndianness, LocSetResultPoseInterval,
 * LocRequestResultData
 * @return true on success, false in case of errors.
 */
bool sick_lidar_localization::ColaConfiguration::start(void)
{
  m_configuration_thread_running = true;
  m_configuration_thread = new boost::thread(&sick_lidar_localization::ColaConfiguration::runConfigurationThreadCb, this);
  return true;
}

/*!
 * Stops transmitting the initial result output configuration to the localization controller.
 */
void sick_lidar_localization::ColaConfiguration::stop(void)
{
  m_configuration_thread_running = false;
  if(m_configuration_thread)
  {
    m_configuration_thread->join();
    delete(m_configuration_thread);
    m_configuration_thread = 0;
  }
}



/*!
 * Thread callback, transmits the initial SIM result output configuration using ros cola services.
 * Configures the following result output settings from launch file:
 * LocSetResultPort, LocSetResultMode, LocSetResultPoseEnabled, LocSetResultEndianness, LocSetResultPoseInterval,
 * LocRequestResultData
 */
void sick_lidar_localization::ColaConfiguration::runConfigurationThreadCb(void)
{
  // Get settings from launch file
  std::map<std::string, int> sim_configuration_map = {
    {"SickLocSetResultPort",         -1}, // "LocSetResultPort" value="2201": Set TCP-port for result output (default: 2201)
    {"SickLocSetResultMode",         -1}, // "LocSetResultMode" value="0": Set mode of result output (0=stream or 1=poll, default:0)
    {"SickLocSetResultPoseEnabled",  -1}, // "LocSetResultPoseEnabled" value="1": Disable (0) or enable (1) result output (default: 1, enabled)
    {"SickLocSetResultEndianness",   -1}, // "LocSetResultEndianness" value="0": Set endianness of result output (0 = big endian, 1 = little endian, default: 0)
    {"SickLocSetResultPoseInterval", -1}, // "LocSetResultPoseInterval" value="1": Set interval of result output (0-255, interval in number of scans, 1: result with each processed scan, default: 1)
    {"SickLocRequestResultData",     -1}  // "LocRequestResultData" value="0": If in poll mode, trigger sending the localization result of the next processed scan via TCP interface (default: 0)
  };
  double retry_delay = 1.0;
  ROS::param<double>(m_nh, "/sick_lidar_localization/driver/tcp_connection_retry_delay", retry_delay, retry_delay);
  ROS_INFO_STREAM("ColaConfiguration: configuration thread started");
  if(ROS::ok() && m_nh && m_configuration_thread_running)
  {
    for(std::map<std::string, int>::iterator iter_config = sim_configuration_map.begin(); iter_config != sim_configuration_map.end(); iter_config++)
    {
      ROS::param<int>(m_nh, std::string("/cola_service_node/") + iter_config->first, iter_config->second, iter_config->second);
      if(iter_config->second >= 0)
        ROS_INFO_STREAM("ColaConfiguration: \"" << iter_config->first << "\": " << iter_config->second);
    }
  }
  
  // Transmit SickLocSetResultPort setting to localization
  while(sim_configuration_map["SickLocSetResultPort"] >= 0 && ROS::ok() && m_nh && m_cola_services && m_configuration_thread_running)
  {
    sick_lidar_localization::SickLocSetResultPortSrv::Request service_request;
    sick_lidar_localization::SickLocSetResultPortSrv::Response service_response;
    service_request.port = sim_configuration_map["SickLocSetResultPort"];
    if(m_cola_services->serviceCbLocSetResultPort(service_request, service_response) && service_response.success)
      break; // service request successfull
    ROS_WARN_STREAM("## ERROR ColaConfiguration: service call type " << typeid(service_request).name() << " failed, retrying");
    ROS::sleep(retry_delay);
  }
  
  // Transmit SickLocSetResultMode setting to localization
  while(sim_configuration_map["SickLocSetResultMode"] >= 0 && ROS::ok() && m_nh && m_cola_services && m_configuration_thread_running)
  {
    sick_lidar_localization::SickLocSetResultModeSrv::Request service_request;
    sick_lidar_localization::SickLocSetResultModeSrv::Response service_response;
    service_request.mode = sim_configuration_map["SickLocSetResultMode"];
    if(m_cola_services->serviceCbLocSetResultMode(service_request, service_response) && service_response.success)
      break; // service request successfull
    ROS_WARN_STREAM("## ERROR ColaConfiguration: service call type " << typeid(service_request).name() << " failed, retrying");
    ROS::sleep(retry_delay);
  }
  
  // Transmit SickLocSetResultPoseEnabled setting to localization
  while(sim_configuration_map["SickLocSetResultPoseEnabled"] >= 0 && ROS::ok() && m_nh && m_cola_services && m_configuration_thread_running)
  {
    sick_lidar_localization::SickLocSetResultPoseEnabledSrv::Request service_request;
    sick_lidar_localization::SickLocSetResultPoseEnabledSrv::Response service_response;
    service_request.enabled = sim_configuration_map["SickLocSetResultPoseEnabled"];
    if(m_cola_services->serviceCbLocSetResultPoseEnabled(service_request, service_response) && service_response.success)
      break; // service request successfull
    ROS_WARN_STREAM("## ERROR ColaConfiguration: service call type " << typeid(service_request).name() << " failed, retrying");
    ROS::sleep(retry_delay);
  }
  
  // Transmit SickLocSetResultEndianness setting to localization
  while(sim_configuration_map["SickLocSetResultEndianness"] >= 0 && ROS::ok() && m_nh && m_cola_services && m_configuration_thread_running)
  {
    sick_lidar_localization::SickLocSetResultEndiannessSrv::Request service_request;
    sick_lidar_localization::SickLocSetResultEndiannessSrv::Response service_response;
    service_request.endianness = sim_configuration_map["SickLocSetResultEndianness"];
    if(m_cola_services->serviceCbLocSetResultEndianness(service_request, service_response) && service_response.success)
      break; // service request successfull
    ROS_WARN_STREAM("## ERROR ColaConfiguration: service call type " << typeid(service_request).name() << " failed, retrying");
    ROS::sleep(retry_delay);
  }
  
  // Transmit SickLocSetResultEndianness setting to localization
  while(sim_configuration_map["SickLocSetResultPoseInterval"] >= 0 && ROS::ok() && m_nh && m_cola_services && m_configuration_thread_running)
  {
    sick_lidar_localization::SickLocSetResultPoseIntervalSrv::Request service_request;
    sick_lidar_localization::SickLocSetResultPoseIntervalSrv::Response service_response;
    service_request.interval = sim_configuration_map["SickLocSetResultPoseInterval"];
    if(m_cola_services->serviceCbLocSetResultPoseInterval(service_request, service_response) && service_response.success)
      break; // service request successfull
    ROS_WARN_STREAM("## ERROR ColaConfiguration: service call type " << typeid(service_request).name() << " failed, retrying");
    ROS::sleep(retry_delay);
  }
  
  // Transmit SickLocSetResultEndianness setting to localization
  while(sim_configuration_map["SickLocRequestResultData"] > 0 && ROS::ok() && m_nh && m_cola_services && m_configuration_thread_running)
  {
    sick_lidar_localization::SickLocRequestResultDataSrv::Request service_request;
    sick_lidar_localization::SickLocRequestResultDataSrv::Response service_response;
    if(m_cola_services->serviceCbLocRequestResultData(service_request, service_response) && service_response.success)
      break; // service request successfull
    ROS_WARN_STREAM("## ERROR ColaConfiguration: service call type " << typeid(service_request).name() << " failed, retrying");
    ROS::sleep(retry_delay);
  }
  
  m_configuration_thread_running = false;
  ROS_INFO_STREAM("ColaConfiguration: configuration thread finished");
}
