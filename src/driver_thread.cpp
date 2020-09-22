/*
 * @brief sim_loc_driver_thread implements the worker thread for the ros driver for sick localization.
 *
 * Class sick_lidar_localization::DriverThread creates and runs two threads:
 * - a tcp thread to connect to the localization controller (f.e. SIM1000FXA)
 *   and receive binary result port telegrams, and
 * - a converter thread to convert the binary telegrams to SickLocResultPortTelegramMsg
 *   and to publish the messages.
 * Both threads uses a threadsafe fifo for data exchange.
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
#include <string>
#include <vector>

#include "sick_lidar_localization/driver_thread.h"
#include "sick_lidar_localization/testcase_generator.h"
#include "sick_lidar_localization/utils.h"


/*
 * Constructor. The driver thread does not start automatically, call start() and stop() to start and stop the driver.
 * @param[in] nh ros node handle
 * @param[in] server_adress ip adress of the localization controller, default: 192.168.0.1
 * @param[in] tcp_port tcp port of the localization controller, default: The localization controller uses IP port number 2201 to send localization results
 */
sick_lidar_localization::DriverThread::DriverThread(ROS::NodePtr nh, const std::string & server_adress, int tcp_port)
: m_initialized(false), m_tcp_connected(false), m_server_adress(server_adress), m_tcp_port(tcp_port), m_tcp_connection_retry_delay(1.0),
  m_tcp_receiver_thread(0), m_tcp_receiver_thread_running(false),
  m_converter_thread(0), m_converter_thread_running(false),
  m_ioservice(), m_tcp_socket(m_ioservice)
{
  if(nh)
  {
    // get config parameter
    std::string result_telegrams_topic = "/sick_lidar_localization/driver/result_telegrams"; // default topic to publish result port telegram messages (type SickLocResultPortTelegramMsg)
    std::string diagnostic_topic = "/sick_lidar_localization/driver/diagnostic"; // default topic to publish diagnostic messages (type SickLocDiagnosticMsg)
    ROS::param<double>(nh, "/sick_lidar_localization/driver/tcp_connection_retry_delay", m_tcp_connection_retry_delay, m_tcp_connection_retry_delay);
    ROS::param<std::string>(nh, "/sick_lidar_localization/driver/result_telegrams_topic", result_telegrams_topic, result_telegrams_topic);
    ROS::param<std::string>(nh, "/sick_lidar_localization/driver/result_telegrams_frame_id", m_result_telegrams_frame_id, "sick_lidar_localization");
    ROS::param<std::string>(nh, "/sick_lidar_localization/driver/diagnostic_topic", diagnostic_topic, diagnostic_topic);
    ROS::param<std::string>(nh, "/sick_lidar_localization/driver/diagnostic_frame_id", m_diagnostic_frame_id, "sick_lidar_localization");
    int software_pll_fifo_length = 7, time_sync_initial_length = 10;
    double time_sync_rate = 0.1, time_sync_initial_rate = 1.0;
    ROS::param<int>(nh, "/sick_lidar_localization/time_sync/software_pll_fifo_length", software_pll_fifo_length, software_pll_fifo_length);
    ROS::param<double>(nh, "/sick_lidar_localization/time_sync/time_sync_rate", time_sync_rate, time_sync_rate);
    ROS::param<double>(nh, "/sick_lidar_localization/time_sync/time_sync_initial_rate", time_sync_initial_rate, time_sync_initial_rate);
    ROS::param<int>(nh, "/sick_lidar_localization/time_sync/time_sync_initial_length", time_sync_initial_length, time_sync_initial_length);
    assert(software_pll_fifo_length > 0 && time_sync_rate > FLT_EPSILON);
    m_software_pll_expected_initialization_duration = (software_pll_fifo_length / time_sync_initial_rate + 1); // expected initialization time for software pll (system time from lidar ticks not yet available)
    // ros publisher for result port telegram messages (type SickLocResultPortTelegramMsg)
    m_result_telegrams_publisher = ROS_CREATE_PUBLISHER(nh, sick_lidar_localization::SickLocResultPortTelegramMsg, result_telegrams_topic);
    // ros publisher for diagnostic messages (type SickLocDiagnosticMsg)
    m_diagnostic_publisher = ROS_CREATE_PUBLISHER(nh, sick_lidar_localization::SickLocDiagnosticMsg, diagnostic_topic);
    // ros service client for SickLocTimeSync
    m_timesync_service_client = ROS_CREATE_SRV_CLIENT(nh, sick_lidar_localization::SickLocTimeSyncSrv, "SickLocTimeSync");
    m_initialized = true;
  }
}

/*
 * Destructor. Stops the driver thread and closes all tcp connections.
 */
sick_lidar_localization::DriverThread::~DriverThread()
{
  stop();
}

/*
 * Starts the driver thread, i.e. starts to receive binary telegrams from the localization controller
 * and starts to publish SickLocResultPortTelegramMsg messages containing the localization data.
 * @return true on success, false on failure.
 */
bool sick_lidar_localization::DriverThread::start(void)
{
  if(!m_initialized) // DriverThread (i.e. its publisher, config parameter, etc.) not properly initialized
  {
    ROS_ERROR_STREAM("## ERROR sick_lidar_localization::DriverThread::start(): DriverThread not initialized");
    publishDiagnosticMessage(CONFIGURATION_ERROR, "## ERROR sim_loc_driver: DriverThread not initialized");
    return false;
  }
  // Create receiver thread to receive telegrams from the localization controller
  m_tcp_receiver_thread_running = true;
  m_tcp_receiver_thread = new boost::thread(&sick_lidar_localization::DriverThread::runReceiverThreadCb, this);
  // Create converter thread to convert and publish localization data
  m_converter_thread_running = true;
  m_converter_thread = new boost::thread(&sick_lidar_localization::DriverThread::runConverterThreadCb, this);
  if(m_tcp_receiver_thread == 0 || m_converter_thread == 0) // DriverThread not properly initialized
  {
    ROS_ERROR_STREAM("## ERROR sick_lidar_localization::DriverThread::start(): failed to create receiver/converter threads");
    publishDiagnosticMessage(INTERNAL_ERROR, "## ERROR sim_loc_driver: failed to create receiver/converter threads");
    return false;
  }
  
  return true;
}

/*
 * Stops the driver thread and closes all tcp connections.
 * @param[in] force_shutdown if true, sockets are immediately forced to shutdown, otherwise stop() waits until all operations finished.
 * @return true on success, false on failure.
 */
bool sick_lidar_localization::DriverThread::stop(bool force_shutdown)
{
  publishDiagnosticMessage(NO_ERROR, "sim_loc_driver: stopping DriverThread");
  ROS_INFO_STREAM("DriverThread::stop(force_shutdown=" << force_shutdown << ")");
  m_tcp_receiver_thread_running = false;
  m_converter_thread_running = false;
  closeTcpConnections(force_shutdown);
  ROS_INFO_STREAM("DriverThread::stop(force_shutdown=" << force_shutdown << "): stopping ioservice");
  m_ioservice.stop();
  if(m_converter_thread)
  {
    ROS_INFO_STREAM("DriverThread::stop(force_shutdown=" << force_shutdown << "): stopping converter thread");
    m_fifo_buffer.push(std::vector<uint8_t>()); // push empty telegram to wake up converter thread waiting for notification
    m_converter_thread->join();
    delete(m_converter_thread);
    m_converter_thread = 0;
  }
  if(m_tcp_receiver_thread)
  {
    ROS_INFO_STREAM("DriverThread::stop(force_shutdown=" << force_shutdown << "): stopping receiver thread");
    m_tcp_receiver_thread->join();
    delete(m_tcp_receiver_thread);
    m_tcp_receiver_thread = 0;
  }
  ROS_INFO_STREAM("DriverThread::stop() finished.");
  publishDiagnosticMessage(NO_ERROR, "sim_loc_driver: DriverThread stopped");
  return true;
}

/*
 * Publishes a diagnostic message.
 * @param[in] error_code one of the error codes enumerated by DRIVER_ERROR_CODES
 * @param[in] message diagnostical message
 */
void sick_lidar_localization::DriverThread::publishDiagnosticMessage(const DRIVER_ERROR_CODES & error_code, const std::string & message)
{
  sick_lidar_localization::SickLocDiagnosticMsg msg;
  msg.header.stamp = ROS::now();
  msg.header.frame_id = m_diagnostic_frame_id;
  msg.error_code = (int32_t)error_code;
  msg.message = message;
  ROS_PUBLISH(m_diagnostic_publisher, msg);
}

/*
 * Returns true, if tcp-connection to the localization controller is established and driver thread is running.
 * @return true if driver thread is running and connected to the localization controller, false otherwise.
 */
bool sick_lidar_localization::DriverThread::isConnected(void)
{
  return m_tcp_receiver_thread_running && m_converter_thread_running && m_tcp_connected;
}

/*
 * Returns true, if driver thread is running (but not necessarily connected to a localization controller).
 * @return true if driver thread is running, false otherwise.
 */
bool sick_lidar_localization::DriverThread::isRunning(void)
{
  return m_tcp_receiver_thread_running && m_converter_thread_running;
}

/*
 * Closes all tcp connections
 * @param[in] force_shutdown if true, sockets are immediately forced to shutdown
 */
void sick_lidar_localization::DriverThread::closeTcpConnections(bool force_shutdown)
{
  m_tcp_connected = false;
  m_tcp_socket.close(force_shutdown);
}

/*
 * Thread callback, connects to the localization controller and receives binary result port telegrams.
 */
void sick_lidar_localization::DriverThread::runReceiverThreadCb(void)
{
  publishDiagnosticMessage(NO_ERROR, "sim_loc_driver: receiver thread started");
  ROS_INFO_STREAM("DriverThread: receiver thread started");
  try
  {
    ROS::Time diagnostic_msg_published;
    // Connect to localization controller
    while(ROS::ok() && m_tcp_receiver_thread_running && !m_tcp_connected)
    {
      if(m_tcp_socket.connect(m_ioservice, m_server_adress, m_tcp_port))
      {
        m_tcp_connected = true;
        publishDiagnosticMessage(NO_ERROR, std::string("sim_loc_driver: tcp connection established to localization controller ") + m_server_adress + ":" + std::to_string(m_tcp_port));
        ROS_INFO_STREAM("DriverThread: tcp connection established to localization controller " << m_server_adress << ":" << m_tcp_port);
      }
      else
      {
        publishDiagnosticMessage(NO_TCP_CONNECTION, std::string("sim_loc_driver: no tcp connection to localization controller ") + m_server_adress + ":" + std::to_string(m_tcp_port));
        ROS_WARN_STREAM("DriverThread: no connection to localization controller " << m_server_adress << ":" << m_tcp_port << ", retry in " << m_tcp_connection_retry_delay << " seconds");
        ROS::sleep(m_tcp_connection_retry_delay);
      }
    }
    // Receive binary telegrams from localization controller
    size_t telegram_size = sick_lidar_localization::TestcaseGenerator::createDefaultResultPortTestcase().binary_data.size(); // 106 byte result port telegrams
    while(ROS::ok() && m_tcp_receiver_thread_running && m_tcp_socket.socket().is_open())
    {
      size_t bytes_received = 0, bytes_required = telegram_size;
      std::vector<uint8_t> receive_buffer(bytes_required, 0);
      boost::system::error_code errorcode;
      std::string error_info("");
      while(ROS::ok() && m_tcp_receiver_thread_running && m_tcp_socket.socket().is_open() && bytes_received < bytes_required)
      {
        size_t bytes_to_read =  bytes_required - bytes_received;
        bytes_received += boost::asio::read(m_tcp_socket.socket(), boost::asio::buffer(&receive_buffer[bytes_received],bytes_to_read), boost::asio::transfer_exactly(bytes_to_read), errorcode);
        if(errorcode)
        {
          std::stringstream error_info_stream;
          error_info_stream << "DriverThread: tcp socket read errorcode " << errorcode.value() << " \"" << errorcode.message() << "\"";
          if(error_info != error_info_stream.str())
          {
            publishDiagnosticMessage(NO_TCP_CONNECTION, std::string("sim_loc_driver: tcp socket read errorcode ") + std::to_string(errorcode.value()) + ", " + errorcode.message());
            ROS_WARN_STREAM(error_info_stream.str());
          }
          error_info = error_info_stream.str();
        }
        else if( ROS::seconds(ROS::now() - diagnostic_msg_published) >= 60)
        {
          publishDiagnosticMessage(NO_ERROR, std::string("sim_loc_driver: tcp connection established to localization controller ") + m_server_adress + ":" + std::to_string(m_tcp_port));
          diagnostic_msg_published = ROS::now();
        }
        if(bytes_received < bytes_required)
          ROS::sleep(0.0001);
      }
      // Copy received telegram to fifo
      if(bytes_received >= bytes_required)
      {
        m_fifo_buffer.push(receive_buffer);
        ROS_DEBUG_STREAM("DriverThread: received " << bytes_received << " byte telegram (hex): " << sick_lidar_localization::Utils::toHexString(receive_buffer));
        ROS::sleep(0.0001);
      }
    }
  }
  catch(std::exception & exc)
  {
    ROS_WARN_STREAM("## ERROR DriverThread::runReceiverThreadCb(): exception " << exc.what());
  }
  m_tcp_receiver_thread_running = false;
  ROS_INFO_STREAM("DriverThread: receiver thread finished");
  publishDiagnosticMessage(NO_ERROR, "sim_loc_driver: receiver thread finished");
}

/*
 * Thread callback, converts the binary result port telegrams to SickLocResultPortTelegramMsg
 * and publishes all localization data.
 */
void sick_lidar_localization::DriverThread::runConverterThreadCb(void)
{
  publishDiagnosticMessage(NO_ERROR, "sim_loc_driver: converter thread started");
  ROS_INFO_STREAM("DriverThread: converter thread started");
  // Decode and publish result port telegrams
  ROS::Time diagnostic_msg_published;
  ROS::Time timestamp_first_telegram;
  sick_lidar_localization::ResultPortParser result_port_parser(m_result_telegrams_frame_id);
  while(ROS::ok() && m_converter_thread_running)
  {
    m_fifo_buffer.waitForElement(); // Wait for at least one element in the fifo buffer
    if(!m_fifo_buffer.empty())
    {
      // Get binary telegram from fifo buffer
      std::vector<uint8_t> binary_telegram = m_fifo_buffer.pop();
      if(!binary_telegram.empty())
      {
        // Decode binary telegram to SickLocResultPortTelegramMsg
        if (!result_port_parser.decode(binary_telegram))
        {
          publishDiagnosticMessage(PARSE_ERROR, std::string("sim_loc_driver: ResultPortParser::decode() failed on ") + std::to_string(binary_telegram.size()) + " byte input (hex):" + sick_lidar_localization::Utils::toHexString(binary_telegram));
          ROS_ERROR_STREAM("## ERROR DriverThread: sick_lidar_localization::ResultPortParser::decode() failed. " << binary_telegram.size() << " byte input (hex):");
          ROS_ERROR_STREAM(sick_lidar_localization::Utils::toHexString(binary_telegram));
          ROS_ERROR_STREAM("## output (decoded): " << sick_lidar_localization::Utils::flattenToString(result_port_parser.getTelegramMsg()));
        }
        else
        {
          sick_lidar_localization::SickLocResultPortTelegramMsg & result_telegram = result_port_parser.getTelegramMsg();
          // Query system time of vehicle pose from lidar tick, using software pll with ros service "SickLocTimeSync"
          result_telegram.vehicle_time_valid = false;
          result_telegram.vehicle_time_sec = 0;
          result_telegram.vehicle_time_nsec = 0;
          if(ROS::secondsSinceStart(timestamp_first_telegram) < FLT_EPSILON)
            timestamp_first_telegram = ROS::now();

#if defined __ROS_VERSION && __ROS_VERSION == 1
          sick_lidar_localization::SickLocTimeSyncSrv time_sync_msg;
          sick_lidar_localization::SickLocTimeSyncSrv::Request* time_sync_msg_request = &time_sync_msg.request;
          sick_lidar_localization::SickLocTimeSyncSrv::Response* time_sync_msg_response = &time_sync_msg.response;
#elif defined __ROS_VERSION && __ROS_VERSION == 2
          std::shared_ptr<sick_lidar_localization::SickLocTimeSyncSrv::Request> time_sync_msg_request = std::make_shared<sick_lidar_localization::SickLocTimeSyncSrv::Request>();
#endif  
          time_sync_msg_request->timestamp_lidar_ms = result_telegram.telegram_payload.timestamp;
#if defined __ROS_VERSION && __ROS_VERSION == 1
          bool service_call_ok = m_timesync_service_client.call(time_sync_msg);
#elif defined __ROS_VERSION && __ROS_VERSION == 2
          auto service_future = m_timesync_service_client->async_send_request(time_sync_msg_request);
          auto future_status = service_future.wait_for(std::chrono::seconds(1)); // wait for the response
          bool service_call_ok = (future_status == std::future_status::ready); // service successfully completed
          std::shared_ptr<sick_lidar_localization::SickLocTimeSyncSrv::Response> time_sync_msg_response = (service_call_ok ? (service_future.get()) : (std::make_shared<sick_lidar_localization::SickLocTimeSyncSrv::Response>()));
#endif
          if (service_call_ok && time_sync_msg_response->vehicle_time_valid)
          {
            result_telegram.vehicle_time_valid = time_sync_msg_response->vehicle_time_valid;
            result_telegram.vehicle_time_sec = time_sync_msg_response->vehicle_time_sec;
            result_telegram.vehicle_time_nsec = time_sync_msg_response->vehicle_time_nsec;
            ROS_DEBUG_STREAM("sim_loc_driver: Lidar ticks: " << result_telegram.telegram_payload.timestamp << ", Systemtime by pll: " << result_telegram.vehicle_time_sec << "." << result_telegram.vehicle_time_sec);
          }
          else if(ROS::seconds(ROS::now() - timestamp_first_telegram) <= m_software_pll_expected_initialization_duration) // software pll still initializing
            ROS_DEBUG_STREAM("sim_loc_driver: no system time from ticks, software pll still initializing");
          else // time sync error
            ROS_WARN_STREAM("## ERROR sim_loc_driver: service \"SickLocTimeSync\" failed, could not get system time from ticks (" << __FILE__ << ":" << __LINE__ << ")");
          // Publish the decoded result port telegram (type SickLocResultPortTelegramMsg)
          ROS_PUBLISH(m_result_telegrams_publisher, result_telegram);
          ROS_INFO_STREAM("DriverThread: result telegram received " << sick_lidar_localization::Utils::toHexString(binary_telegram) << ", published " << sick_lidar_localization::Utils::flattenToString(result_telegram));
          if( ROS::seconds(ROS::now() - diagnostic_msg_published) >= 60)
          {
            publishDiagnosticMessage(NO_ERROR, "sim_loc_driver: status okay, receiving and publishing result telegrams");
            diagnostic_msg_published = ROS::now();
          }
        }
      }
    }
    else
    {
      ROS::sleep(0.001);
    }
  }
  m_converter_thread_running = false;
  ROS_INFO_STREAM("DriverThread: converter thread finished");
  publishDiagnosticMessage(NO_ERROR, "sim_loc_driver: converter thread finished");
}
