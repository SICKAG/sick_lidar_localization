/*
 * @brief time_sync_service implements ros services "SickLocRequestTimestamp" and "SickLocTimeSync"
 * for time synchronization.
 *
 * ROS service SickLocRequestTimestamp requests a timestamp from the localization controller
 * by sending cola command LocRequestTimestamp ("sMN LocRequestTimestamp").
 *
 * The service receives and decodes the current timestamp (uint32 timestamp in milliseconds)
 * and calculates the time offset with the following formular:
 *
 * delta_time_ms := mean_time_vehicle_ms - timestamp_lidar_ms
 * mean_time_vehicle_ms := (send_time_vehicle + receive_time_vehicle) / 2
 *                      := vehicles mean timestamp in milliseconds
 * send_time_vehicle    := vehicles timestamp when sending LocRequestTimestamp
 * receive_time_vehicle := vehicles timestamp when receiving the LocRequestTimestamp response
 * timestamp_lidar_ms   := lidar timestamp in milliseconds from LocRequestTimestamp response
 *
 * See Operation-Instruction-v1.1.0.241R.pdf for details about time synchronization and
 * time offset calculation. See Technical_information_Telegram_Listing_NAV_LOC_en_IM0076556.PDF
 * for further details about Cola telegram LocRequestTimestamp.
 *
 * time_sync_service implements a time synchronization thread, running a software pll
 * to estimate system time from lidar timestamp ticks. This synchronization thread can
 * be started by function start() and stopped by function stop(). When running the
 * synchronization thread, ros services "SickLocRequestTimestamp" is called each 10 seconds.
 * The software pll is then updated to estimate the system time from lidar timestamp ticks.
 * The system timestamp of a vehicle pose can be queried using ros service
 * "SickLocTimeSync".
 *
 * Note: The software pll uses a default fifo size of 7 measurements, thus it requires at least 7 successful
 * LocRequestTimestamp requests. Depending on time_sync_rate configured in the launch-file, this initial phase can
 * take several seconds (up to 70 seconds).
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

#include "sick_lidar_localization/cola_parser.h"
#include "sick_lidar_localization/SoftwarePLL.h"
#include "sick_lidar_localization/time_sync_service.h"

/*!
 * Constructor
 */
sick_lidar_localization::TimeSyncService::TimeSyncService(ROS::NodePtr nh, sick_lidar_localization::DriverMonitor* driver_monitor)
: m_nh(nh), m_driver_monitor(driver_monitor), m_time_sync_thread_running(false), m_time_sync_thread(0), m_cola_binary(false), m_cola_binary_mode(0), m_software_pll_fifo_length(7),
  m_time_sync_rate(0.1), m_time_sync_initial_rate(1.0), m_time_sync_initial_length(10), m_cola_response_timeout(10.0)
{
  if(nh)
  {
    // Configuration and parameter
    ROS::param<int>(nh, "/sick_lidar_localization/driver/cola_binary", m_cola_binary_mode, m_cola_binary_mode);
    m_cola_binary = (m_cola_binary_mode == 1) ? true : false; //  0: send Cola-ASCII (default), 1: send Cola-Binary, 2: toggle between Cola-ASCII and Cola-Binary (test and development only!)
    ROS::param<int>(nh, "/sick_lidar_localization/time_sync/software_pll_fifo_length", m_software_pll_fifo_length, m_software_pll_fifo_length);
    double time_sync_rate = 0.1, time_sync_initial_rate = 1.0;
    ROS::param<double>(nh, "/sick_lidar_localization/time_sync/time_sync_rate", time_sync_rate, time_sync_rate);
    ROS::param<double>(nh, "/sick_lidar_localization/time_sync/time_sync_initial_rate", time_sync_initial_rate, time_sync_initial_rate);
    m_time_sync_rate = time_sync_rate;
    m_time_sync_initial_rate = time_sync_initial_rate;
    ROS::param<int>(nh, "/sick_lidar_localization/time_sync/time_sync_initial_length", m_time_sync_initial_length, m_time_sync_initial_length);
    ROS::param<double>(nh, "/sick_lidar_localization/time_sync/cola_response_timeout", m_cola_response_timeout, m_cola_response_timeout);
    // Advertise service "SickLocRequestTimestamp" to send a LocRequestTimestamp, receive the response and to calculate the time offset
    // Advertise service "SickLocTimeSync" to calculate system time from ticks by software pll
#if defined __ROS_VERSION && __ROS_VERSION == 1
    m_timestamp_service_server = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocRequestTimestampSrv, "SickLocRequestTimestamp", &sick_lidar_localization::TimeSyncService::serviceCbRequestTimestamp, this);
    m_timesync_service_server = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocTimeSyncSrv, "SickLocTimeSync", &sick_lidar_localization::TimeSyncService::serviceCbTimeSync, this);
#elif defined __ROS_VERSION && __ROS_VERSION == 2
    m_timestamp_service_server = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocRequestTimestampSrv, "SickLocRequestTimestamp", &sick_lidar_localization::TimeSyncService::serviceCbRequestTimestampROS2, this);
    m_timesync_service_server = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocTimeSyncSrv, "SickLocTimeSync", &sick_lidar_localization::TimeSyncService::serviceCbTimeSyncROS2, this);
#endif
    ROS_INFO_STREAM("TimeSyncService: advertising ros service \"SickLocRequestTimestamp\" for LocRequestTimestamp commands, message type SickLocRequestTimestamp");
    ROS_INFO_STREAM("TimeSyncService: advertising ros service \"SickLocTimeSync\" for time synchronization by software pll, message type SickLocTimeSync");
    // Clients for ros services "SickLocColaTelegram", "SickLocRequestTimestamp" and "SickLocTimeSync", required for time synchronization using a software pll
    m_cola_service_client = ROS_CREATE_SRV_CLIENT(nh, sick_lidar_localization::SickLocColaTelegramSrv, "SickLocColaTelegram");
    // m_request_timestamp_client = ROS_CREATE_SRV_CLIENT(nh, sick_lidar_localization::SickLocRequestTimestampSrv, "SickLocRequestTimestamp");
  }
}

/*!
 * Destructor
 */
sick_lidar_localization::TimeSyncService::~TimeSyncService()
{
  stop();
}

/*!
 * Callback for service messages (SickLocRequestTimestamp). Sends LocRequestTimestamp requests
 * to the localization controller, receives the response and calculates the time offset.
 * @param[in] service_request ros service request
 * @param[out] service_response service response with timestamps and calculated time offset.
 * @return true on success, false in case of errors.
 */
bool sick_lidar_localization::TimeSyncService::serviceCbRequestTimestamp(sick_lidar_localization::SickLocRequestTimestampSrv::Request & service_request, sick_lidar_localization::SickLocRequestTimestampSrv::Response & service_response)
{
  boost::lock_guard<boost::mutex> service_cb_lockguard(m_service_cb_mutex); // one service request at a time
  // Sends cola command "sMN LocRequestTimestamp" and receive timestamp from localization controller using ros service "SickLocColaTelegram"
  m_cola_binary = ((m_cola_binary_mode == 2) ? (!m_cola_binary) : (m_cola_binary)); // m_cola_binary_mode == 0: send Cola-ASCII (default), 1: send Cola-Binary, 2: toggle between Cola-ASCII and Cola-Binary (test and development only!)
#if defined __ROS_VERSION && __ROS_VERSION == 1
  sick_lidar_localization::SickLocColaTelegramSrv cola_telegram;
  sick_lidar_localization::SickLocColaTelegramSrv::Request* cola_telegram_request = &cola_telegram.request;
  sick_lidar_localization::SickLocColaTelegramSrv::Response* cola_telegram_response = &cola_telegram.response;
#elif defined __ROS_VERSION && __ROS_VERSION == 2
  std::shared_ptr<sick_lidar_localization::SickLocColaTelegramSrv::Request> cola_telegram_request = std::make_shared<sick_lidar_localization::SickLocColaTelegramSrv::Request>();
  std::shared_ptr<sick_lidar_localization::SickLocColaTelegramSrv::Response> cola_telegram_response = std::make_shared<sick_lidar_localization::SickLocColaTelegramSrv::Response>();
#endif
  cola_telegram_request->cola_ascii_request = "sMN LocRequestTimestamp";
  cola_telegram_request->wait_response_timeout = m_cola_response_timeout;
  // cola_telegram_request.send_binary = m_cola_binary;
  try
  {
    ROS::Time start_request_timestamp = ROS::now();
    if(m_driver_monitor == 0)
    {
      ROS_ERROR_STREAM("## ERROR TimeSyncService::serviceCbRequestTimestamp(): TimeSyncService not initialized (driver_monitor == 0, " << __FILE__ << ":" << __LINE__ << ")");
      return false;
    }
    bool service_call_ok = m_driver_monitor->serviceCbColaTelegram(*cola_telegram_request, *cola_telegram_response);
    if (!service_call_ok || cola_telegram_response->cola_ascii_response.empty())
    {
      ROS_WARN_STREAM("## ERROR TimeSyncService::serviceCbRequestTimestamp(): calling ros service \"SickLocColaTelegram\" failed with request: \""
        << cola_telegram_request->cola_ascii_request << "\" response: \"" << cola_telegram_response->cola_ascii_response << "\" after "
        << ROS::seconds(ROS::now() - start_request_timestamp) << " sec (status: " << service_call_ok << ", timeout: " << m_cola_response_timeout
        << " sec, " << __FILE__ << ":" << __LINE__ << ")");
      return false;
    }
    ROS_DEBUG_STREAM("TimeSyncService::serviceCbRequestTimestamp(): request \"" << cola_telegram_request->cola_ascii_request << "\" response: \"" << cola_telegram_response->cola_ascii_response << "\" succesfull.");
  }
  catch(const std::exception & exc)
  {
    ROS_WARN_STREAM("## ERROR TimeSyncService::serviceCbRequestTimestamp():  request \"" << cola_telegram_request->cola_ascii_request << "\" failed, exception " << exc.what());
    return false;
  }

  // Decode response, get timestamp_lidar_ms from parameter
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sick_lidar_localization::ColaParser::decodeColaTelegram(cola_telegram_response->cola_ascii_response);
  if(cola_response.command_name != "LocRequestTimestamp" || cola_response.parameter.size() != 1)
  {
    ROS_WARN_STREAM("## ERROR TimeSyncService::serviceCbRequestTimestamp(): invalid or unexpected cola response: \"" << cola_telegram_response->cola_ascii_response
      << "\" decoded to " << sick_lidar_localization::Utils::flattenToString(cola_response));
    return false;
  }
  try
  {
    service_response.timestamp_lidar_ms = (std::stoul(cola_response.parameter[0],nullptr,16) & 0xFFFFFFFF); // Lidar timestamp in milliseconds from LocRequestTimestamp response
  }
  catch(const std::exception & exc)
  {
    ROS_WARN_STREAM("## ERROR TimeSyncService::serviceCbRequestTimestamp(): failed to parse timestamp parameter in cola response " << sick_lidar_localization::Utils::flattenToString(cola_response)
      << ", exception " << exc.what());
    return false;
  }
  if(service_response.timestamp_lidar_ms <= 0 || cola_telegram_response->send_timestamp_sec <= 0 || cola_telegram_response->receive_timestamp_sec <= 0)
  {
    ROS_WARN_STREAM("## ERROR TimeSyncService::serviceCbRequestTimestamp(): invalid timestamps in cola response " << sick_lidar_localization::Utils::flattenToString(cola_response));
    return false;
  }

  // Set timestamps in service_response
  service_response.send_time_vehicle_sec = cola_telegram_response->send_timestamp_sec;          // Vehicle timestamp when sending LocRequestTimestamp (seconds part of ros timestamp immediately before tcp send)
  service_response.send_time_vehicle_nsec = cola_telegram_response->send_timestamp_nsec;        // Vehicle timestamp when sending LocRequestTimestamp (nano seconds part of ros timestamp immediately before tcp send)
  service_response.receive_time_vehicle_sec = cola_telegram_response->receive_timestamp_sec;    // Vehicle timestamp when receiving the LocRequestTimestamp response (seconds part of ros timestamp immediately after first response byte received)
  service_response.receive_time_vehicle_nsec = cola_telegram_response->receive_timestamp_nsec;  // Vehicle timestamp when receiving the LocRequestTimestamp response (nano seconds part of ros timestamp immediately after first response byte received)

  // Calculate time offset
  uint64_t send_time_vehicle_nsec = service_response.send_time_vehicle_sec * 1000000000UL + service_response.send_time_vehicle_nsec;
  uint64_t receive_time_vehicle_nsec = service_response.receive_time_vehicle_sec * 1000000000UL + service_response.receive_time_vehicle_nsec;
  uint64_t mean_time_vehicle_nsec = send_time_vehicle_nsec / 2 + receive_time_vehicle_nsec / 2;
  service_response.mean_time_vehicle_ms = mean_time_vehicle_nsec / 1000000;                                      // Vehicle mean timestamp in milliseconds: (send_time_vehicle + receive_time_vehicle) / 2
  service_response.delta_time_ms = service_response.mean_time_vehicle_ms - service_response.timestamp_lidar_ms;  // Time offset: mean_time_vehicle_ms - timestamp_lidar_ms

  // Update software pll
  updateSoftwarePll(service_response);

  // Get system timestamp from ticks via ros service "SickLocTimeSync"
  sick_lidar_localization::SickLocTimeSyncSrv::Request time_sync_msg_request;
  sick_lidar_localization::SickLocTimeSyncSrv::Response time_sync_msg_response;
  time_sync_msg_request.timestamp_lidar_ms = service_response.timestamp_lidar_ms;
  if(serviceCbTimeSync(time_sync_msg_request, time_sync_msg_response) && time_sync_msg_response.vehicle_time_valid)
    ROS_INFO_STREAM("TimeSyncService::serviceCbRequestTimestamp(): Lidar ticks: " << service_response.timestamp_lidar_ms << ", Systemtime: " << time_sync_msg_response.vehicle_time_sec << "." << time_sync_msg_response.vehicle_time_sec);
  else if(isSoftwarePllInitialized())
    ROS_WARN_STREAM("## ERROR TimeSyncService::serviceCbRequestTimestamp(): service \"SickLocTimeSync\" failed, could not get system time from ticks");
  else
    ROS_INFO_STREAM("TimeSyncService::serviceCbRequestTimestamp(): no system time from ticks, software pll still initializing");

  return true;
}

/*!
 * Update the software pll with lidar ticks and system time from response by "SickLocRequestTimestamp" servcice
 * @param[out] service_response service response by "SickLocRequestTimestamp" servcice
 */
void sick_lidar_localization::TimeSyncService::updateSoftwarePll(sick_lidar_localization::SickLocRequestTimestampSrv::Response & service_response)
{
  boost::lock_guard<boost::mutex> software_pll_lockguard(m_software_pll_mutex);
  SoftwarePLL & software_pll_send_time = SoftwarePLL::Instance("sick_lidar_localization::TimeSyncService::SendTime", m_software_pll_fifo_length);
  SoftwarePLL & software_pll_receive_time = SoftwarePLL::Instance("sick_lidar_localization::TimeSyncService::ReceiveTime", m_software_pll_fifo_length);
  software_pll_send_time.UpdatePLL(service_response.send_time_vehicle_sec, service_response.send_time_vehicle_nsec, service_response.timestamp_lidar_ms);
  software_pll_receive_time.UpdatePLL(service_response.receive_time_vehicle_sec, service_response.receive_time_vehicle_nsec, service_response.timestamp_lidar_ms);
}

/*!
 * Returns true, if the initialization phase of the software pll is completed, otherwise false.
 * @return initialization phase of the software pll completed
 */
bool sick_lidar_localization::TimeSyncService::isSoftwarePllInitialized(void)
{
  boost::lock_guard<boost::mutex> software_pll_lockguard(m_software_pll_mutex);
  SoftwarePLL & software_pll_send_time = SoftwarePLL::Instance("sick_lidar_localization::TimeSyncService::SendTime", m_software_pll_fifo_length);
  SoftwarePLL & software_pll_receive_time = SoftwarePLL::Instance("sick_lidar_localization::TimeSyncService::ReceiveTime", m_software_pll_fifo_length);
  return software_pll_send_time.IsInitialized() && software_pll_receive_time.IsInitialized();
}


/*!
 * Callback for service messages (SickLocTimeSync). Calculates the system time of a vehicle pose from lidar ticks,
 * using a software pll.
 * Note: ros service SickLocTimeSync uses service SickLocRequestTimestamp to query lidar ticks and to update the
 * software pll. Therefore, at least 7 LocRequestTimestamp measurements are required before time sync becomes valid
 * The default fifo size of the software pll is 7 measurements, thus it requires at least 7 successful
 * LocRequestTimestamp requests. Depending on time_sync_rate configured in the launch-file, this initial phase can
 * take several seconds (up to 70 seconds).
 * @param[in] time_sync_request ros service request (input: lidar ticks)
 * @param[out] time_sync_response service response  (output: system time from ticks, calculated by software pll)
 * @return true on success, false in case of errors (software pll still in initialization phase or communication error).
 */
bool sick_lidar_localization::TimeSyncService::serviceCbTimeSync(sick_lidar_localization::SickLocTimeSyncSrv::Request & time_sync_request, sick_lidar_localization::SickLocTimeSyncSrv::Response & time_sync_response)
{
  boost::lock_guard<boost::mutex> software_pll_lockguard(m_software_pll_mutex);
  SoftwarePLL & software_pll_send_time = SoftwarePLL::Instance("sick_lidar_localization::TimeSyncService::SendTime", m_software_pll_fifo_length);
  SoftwarePLL & software_pll_receive_time = SoftwarePLL::Instance("sick_lidar_localization::TimeSyncService::ReceiveTime", m_software_pll_fifo_length);
  time_sync_response.vehicle_time_sec = 0;
  time_sync_response.vehicle_time_nsec = 0;
  time_sync_response.vehicle_time_valid = false;
  uint32_t ticks = time_sync_request.timestamp_lidar_ms;
  uint32_t system_timestamp_1_sec = 0, system_timestamp_1_nsec = 0, system_timestamp_2_sec = 0, system_timestamp_2_nsec = 0;
  if(software_pll_send_time.GetCorrectedTimeStamp(system_timestamp_1_sec, system_timestamp_1_nsec, ticks)
    && software_pll_receive_time.GetCorrectedTimeStamp(system_timestamp_2_sec, system_timestamp_2_nsec, ticks))
  {
    ROS::Time system_timestamp_1(system_timestamp_1_sec, system_timestamp_1_nsec);
    ROS::Time system_timestamp_2(system_timestamp_2_sec, system_timestamp_2_nsec);
    ROS::Time system_timestamp = system_timestamp_1 + ROS::durationFromSec(0.5 * ROS::seconds(system_timestamp_2 - system_timestamp_1));
    ROS::splitTime(system_timestamp, time_sync_response.vehicle_time_sec, time_sync_response.vehicle_time_nsec);
    time_sync_response.vehicle_time_valid = true;
    ROS_INFO_STREAM("TimeSyncService::serviceCbTimeSync(): Lidar ticks: " << ticks << ", Systemtime: " << time_sync_response.vehicle_time_sec << "." << time_sync_response.vehicle_time_nsec);
  }
  else if(software_pll_send_time.IsInitialized() && software_pll_receive_time.IsInitialized())
    ROS_WARN_STREAM("## ERROR TimeSyncService::serviceCbTimeSync(): SoftwarePLL::GetCorrectedTimeStamp() failed");
  else
    ROS_INFO_STREAM("TimeSyncService::serviceCbTimeSync(): no system time from ticks, software pll still initializing");
  return time_sync_response.vehicle_time_valid;
}


/*!
 * Class TimeSyncService implements a time synchronization thread, running a software pll
 * to estimate system time from lidar timestamp ticks. This synchronization thread can
 * be started by function start() and stopped by function stop(). When running the
 * synchronization thread, ros services "SickLocRequestTimestamp" is called each 10 seconds.
 * The software pll is then updated to estimate the system time from lidar timestamp ticks.
 * The system timestamp of a vehicle pose can be queried using ros service
 * "SickLocTimeSync".
 * @return true on success, false in case of errors.
 */
bool sick_lidar_localization::TimeSyncService::start(void)
{
  m_time_sync_thread_running = true;
  m_time_sync_thread = new boost::thread(&sick_lidar_localization::TimeSyncService::runTimeSyncThreadCb, this);
  return (m_time_sync_thread != 0);
}

/*!
 * Stops the time synchronization thread and the software pll.
 * @return true on success, false in case of errors.
 */
bool sick_lidar_localization::TimeSyncService::stop(void)
{
  m_time_sync_thread_running = false;
  if(m_time_sync_thread)
  {
    m_time_sync_thread->join();
    delete(m_time_sync_thread);
    m_time_sync_thread = 0;
  }
  return (m_time_sync_thread == 0);
}

/*!
 * Thread callback, runs time synchronization, calls ros service "SickLocRequestTimestamp" each 10 seconds
 * and updates the software pll.
 */
void sick_lidar_localization::TimeSyncService::runTimeSyncThreadCb(void)
{
  int time_sync_cnt = 0;
  while(ROS::ok() && m_time_sync_thread_running)
  {
    // Run LocRequestTimestamp every second during initialization of software pll, otherwise every 10 seconds
    ROS::Rate time_sync_rate((time_sync_cnt < m_time_sync_initial_length) ? m_time_sync_initial_rate : m_time_sync_rate);
    time_sync_rate.sleep();
    if(ROS::ok() && m_time_sync_thread_running)
    {
      // Call ros service "SickLocRequestTimestamp"
#if defined __ROS_VERSION && __ROS_VERSION == 1
      sick_lidar_localization::SickLocRequestTimestampSrv timestamp_service;
      sick_lidar_localization::SickLocRequestTimestampSrv::Request* timestamp_service_request = &timestamp_service.request;
      sick_lidar_localization::SickLocRequestTimestampSrv::Response* timestamp_service_response = &timestamp_service.response;
      // bool service_call_ok = m_request_timestamp_client.call(timestamp_service);
#elif defined __ROS_VERSION && __ROS_VERSION == 2
      std::shared_ptr<sick_lidar_localization::SickLocRequestTimestampSrv::Request> timestamp_service_request = std::make_shared<sick_lidar_localization::SickLocRequestTimestampSrv::Request>();
      std::shared_ptr<sick_lidar_localization::SickLocRequestTimestampSrv::Response> timestamp_service_response = std::make_shared<sick_lidar_localization::SickLocRequestTimestampSrv::Response>();
#endif
      // ROS_INFO_STREAM("TimeSyncService::runTimeSyncThreadCb(): calling serviceCbRequestTimestamp() ...");
      bool service_call_ok = serviceCbRequestTimestamp(*timestamp_service_request, *timestamp_service_response);
      if (!service_call_ok || timestamp_service_response->timestamp_lidar_ms == 0)
      {
        ROS_WARN_STREAM("## ERROR TimeSyncService::runTimeSyncThreadCb(): calling ros service \"SickLocRequestTimestamp\" failed, response: timestamp_lidar_ms=" << timestamp_service_response->timestamp_lidar_ms
          << " (status: " << service_call_ok << ", timeout: 1 sec, " << __FILE__ << ":" << __LINE__ << ")");
      }
      else
      {
        time_sync_cnt++;
        ROS_INFO_STREAM("TimeSyncService::runTimeSyncThreadCb(): ros service \"SickLocRequestTimestamp\" successfull, response: timestamp_lidar_ms=" << timestamp_service_response->timestamp_lidar_ms);
      }
    }
  }
  m_time_sync_thread_running = false;
}
