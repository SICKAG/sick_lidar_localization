/*
 * @brief sim_loc_driver_monitor monitors the driver thread implemented by sick_lidar_localization::DriverThread
 * and starts a new driver thread after tcp errors (connection lost, socket shutdown, message timeouts etc.).
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

#include "sick_lidar_localization/cola_parser.h"
#include "sick_lidar_localization/driver_monitor.h"

/*!
 * Constructor. The driver monitor does not start automatically, call start() and stop() to start and stop.
 *
 * Default tcp ports: see Operation-Instruction-v1.1.0.241R.pdf, page 51, "IP port number and protocol":
 * For requests and to transmit settings to the localization controller:
 * - IP port number 2111 and 2112 to send telegrams and to request data.
 * - SOPAS CoLa-A or CoLa-B protocols.
 * To transmit the localization results to the vehicle controller, the localization controller uses:
 * - IP port number 2201 to send localization results in a single direction to the external vehicle controller.
 * - Binary result port protocol TCP/IP.
 *
 * @param[in] nh ros node handle
 * @param[in] server_adress ip adress of the localization controller, default: 192.168.0.1
 * @param[in] ip_port_results ip port for result telegrams, default: 2201
 * @param[in] ip_port_cola ip port for command requests and responses, default: 2111
 */
sick_lidar_localization::DriverMonitor::DriverMonitor(ros::NodeHandle * nh, const std::string & server_adress, int ip_port_results, int ip_port_cola)
: m_initialized(false), m_nh(nh), m_server_adress(server_adress), m_ip_port_results(ip_port_results), m_ip_port_cola(ip_port_cola), m_cola_binary(false),
  m_monitoring_thread_running(false), m_monitoring_thread(0), m_monitoring_rate(1.0), m_receive_telegrams_timeout(1.0), m_cola_transmitter(0)
{
  if(m_nh)
  {
    // Query configuration
    int cola_binary_mode = 0;
    ros::param::param<int>("/sick_lidar_localization/driver/cola_binary", cola_binary_mode, cola_binary_mode);
    m_cola_binary = (cola_binary_mode == 1) ? true : false; //  0: send Cola-ASCII (default), 1: send Cola-Binary, 2: toggle between Cola-ASCII and Cola-Binary (test and development only!)
    ros::param::param<double>("/sick_lidar_localization/driver/monitoring_rate", m_monitoring_rate, m_monitoring_rate); // frequency to monitor driver messages, default: once per second
    ros::param::param<double>("/sick_lidar_localization/driver/monitoring_message_timeout", m_receive_telegrams_timeout, m_receive_telegrams_timeout); // timeout for driver messages, shutdown tcp-sockets and reconnect after message timeout, default: 1 second
    ros::param::param<double>("/sick_lidar_localization/time_sync/cola_response_timeout", m_cola_response_timeout, m_cola_response_timeout);
    m_initialized = true;
  }
}

/*!
 * Destructor. Stops the driver thread and monitor and closes all tcp connections.
 */
sick_lidar_localization::DriverMonitor::~DriverMonitor()
{
  stop();
}

/*!
 * Starts the driver monitor and driver thread.
 * @return true on success, false on failure.
 */
bool sick_lidar_localization::DriverMonitor::start(void)
{
  if(!m_initialized) // DriverMonitor not properly initialized
  {
    ROS_ERROR_STREAM("## ERROR sick_lidar_localization::DriverMonitor::start(): DriverMonitor not initialized");
    return false;
  }
  // Create receiver thread to receive telegrams from the localization controller
  m_monitoring_thread_running = true;
  m_monitoring_thread = new boost::thread(&sick_lidar_localization::DriverMonitor::runMonitorThreadCb, this);
  return true;
}

/*!
 * Stops the driver monitor and thread and closes all tcp connections.
 * @return true on success, false on failure.
 */
bool sick_lidar_localization::DriverMonitor::stop(void)
{
  m_monitoring_thread_running = false;
  if(m_monitoring_thread)
  {
    m_monitoring_thread->join();
    delete(m_monitoring_thread);
    m_monitoring_thread = 0;
  }
  stopColaTransmitter();
  return true;
}

/*!
 * Initializes cola sender and receiver for cola requests and responses
 * @param[in] server_adress ip adress of the localization controller, default: 192.168.0.1
 * @param[in] ip_port_cola ip port for command requests and responses, default: 2111
 * @param[in] receive_timeout timeout in seconds for receive functions
 */
bool sick_lidar_localization::DriverMonitor::initColaTransmitter(const std::string & server_adress, int ip_port_cola, double receive_timeout)
{
  if(!m_cola_transmitter)
  {
    m_cola_transmitter = new sick_lidar_localization::ColaTransmitter(server_adress, ip_port_cola, receive_timeout);
    if (!m_cola_transmitter->connect())
    {
      ROS_WARN_STREAM("## ERROR DriverMonitor::serviceCbColaTelegram: can't connect to localization server " << server_adress << ":" << ip_port_cola);
      return false;
    }
    if (!m_cola_transmitter->startReceiverThread())
    {
      ROS_WARN_STREAM("## ERROR DriverMonitor::serviceCbColaTelegram: can't start receiver thread");
      return false;
    }
  }
  return true;
}

/*!
 * Stops cola sender and receiver for cola requests and responses
 * (if started by a cola service request "SickLocColaTelegram")
 */
void sick_lidar_localization::DriverMonitor::stopColaTransmitter(void)
{
  if(m_cola_transmitter)
  {
    delete(m_cola_transmitter);
    m_cola_transmitter = 0;
  }
}

/*!
 * Callback for result telegram messages (SickLocResultPortTelegramMsg) from sim_loc_driver.
 * @param[in] msg result telegram message (SickLocResultPortTelegramMsg)
 */
void sick_lidar_localization::DriverMonitor::messageCbResultPortTelegrams(const sick_lidar_localization::SickLocResultPortTelegramMsg & msg)
{
  m_driver_message_recv_timestamp.set(ros::Time::now());
}

/*!
 * Callback for service messages (SickLocColaTelegramSrv). Handles Cola requests and responses, sends and receives Cola-ASCII telegrams to resp. from
 * the localization controller
 * @param[in] cola_request Cola command request, will be encoded and send to the localization controller
 * @param[out] cola_response Cola command response from localization controller
 */
bool sick_lidar_localization::DriverMonitor::serviceCbColaTelegram(sick_lidar_localization::SickLocColaTelegramSrv::Request & cola_request, sick_lidar_localization::SickLocColaTelegramSrv::Response & cola_response)
{
  ROS_INFO_STREAM("DriverMonitor::serviceCbColaTelegram: starting Cola request { " << sick_lidar_localization::Utils::flattenToString(cola_request) << " }");
  boost::lock_guard<boost::mutex> service_cb_lockguard(m_service_cb_mutex); // one service request at a time
  // initialize cola_response with default values
  cola_response.cola_ascii_response = "";
  const std::string & asciiSTX = sick_lidar_localization::ColaParser::asciiSTX();
  const std::string & asciiETX = sick_lidar_localization::ColaParser::asciiETX();
  // Convert to binary (optional, default: false)
  std::string ascii_request = asciiSTX + cola_request.cola_ascii_request + asciiETX;
  std::vector<uint8_t> binary_request = sick_lidar_localization::ColaAsciiBinaryConverter::ConvertColaAscii(ascii_request);
  if(m_cola_binary)  // cola_request.send_binary
    binary_request = sick_lidar_localization::ColaAsciiBinaryConverter::ColaAsciiToColaBinary(binary_request);
  // Initialize cola sender and receiver, connect to localization controlle
  if(!initColaTransmitter(m_server_adress, m_ip_port_cola, cola_request.wait_response_timeout))
  {
    ROS_WARN_STREAM("## ERROR DriverMonitor::serviceCbColaTelegram: can't initialize cola transmission to localization server " << m_server_adress);
    stopColaTransmitter();
    return false;
  }
  // Send request and wait for response with timeout
  std::vector<uint8_t> binary_response;
  ros::Time send_timestamp, receive_timestamp;
  if(!m_cola_transmitter->send(binary_request, send_timestamp))
  {
    ROS_WARN_STREAM("## ERROR DriverMonitor::serviceCbColaTelegram: send() failed to localization server " << m_server_adress << ":" << m_ip_port_cola);
    stopColaTransmitter();
    return false;
  }
  if(!m_cola_transmitter->waitPopResponse(binary_response, cola_request.wait_response_timeout, receive_timestamp) || binary_response.size() < 2) // at least 2 byte stx and etx
  {
    ROS_WARN_STREAM("## ERROR DriverMonitor::serviceCbColaTelegram: receive() failed by localization server " << m_server_adress << ":" << m_ip_port_cola);
    stopColaTransmitter();
    return false;
  }
  // Convert reponse from controller to Cola-ASCII telegram
  bool is_binary_cola = sick_lidar_localization::ColaAsciiBinaryConverter::IsColaBinary(binary_response);
  ROS_INFO_STREAM("DriverMonitor::serviceCbColaTelegram: " << (is_binary_cola?"cola-binary":"cola-ascii") << " response received (hex): " << sick_lidar_localization::Utils::toHexString(binary_response));
  if(m_cola_binary && is_binary_cola) // if(cola_request.send_binary)
    binary_response = sick_lidar_localization::ColaAsciiBinaryConverter::ColaBinaryToColaAscii(binary_response);
  cola_response.cola_ascii_response = sick_lidar_localization::ColaAsciiBinaryConverter::ConvertColaAscii(binary_response);
  if (cola_response.cola_ascii_response.size() > asciiSTX.size() + asciiETX.size()
    && cola_response.cola_ascii_response.substr(0, asciiSTX.size()) == asciiSTX
    && cola_response.cola_ascii_response.substr(cola_response.cola_ascii_response.size() - asciiETX.size()) == asciiETX)
  {
    // Remove "<STX>" and "<ETX>" start and end tags
    cola_response.cola_ascii_response = cola_response.cola_ascii_response.substr(asciiSTX.size(), cola_response.cola_ascii_response.size() - asciiSTX.size() - asciiETX.size()); // Cola-ASCII response
    cola_response.send_timestamp_sec = send_timestamp.sec;          // Send timestamp (seconds part of ros timestamp immediately before tcp send)
    cola_response.send_timestamp_nsec = send_timestamp.nsec;        // Send timestamp (nano seconds part of ros timestamp immediately before tcp send)
    cola_response.receive_timestamp_sec = receive_timestamp.sec;    // Receive timestamp (seconds part of ros timestamp immediately after first response byte received)
    cola_response.receive_timestamp_nsec = receive_timestamp.nsec;  // Receive timestamp (nano seconds part of ros timestamp immediately after first response byte received)
    ROS_INFO_STREAM("DriverMonitor::serviceCbColaTelegram: finished Cola request { " << sick_lidar_localization::Utils::flattenToString(cola_request) << " } with response { " << sick_lidar_localization::Utils::flattenToString(cola_response) << " }");
    return true;
  }
  else
  {
    ROS_WARN_STREAM("## ERROR DriverMonitor::serviceCbColaTelegram: parse error, response from localization server \"" << cola_response.cola_ascii_response << "\" not enclosed with \"" << asciiSTX << "\" and \"" << asciiETX << "\"");
  }
  stopColaTransmitter();
  return false;
}

/*!
 * Returns true, if result telegrams have been received within configured timeout "monitoring_message_timeout".
 * If no result telegrams have been received within the timeout, the localization status is queried by ros service
 * "SickLocState". If localization is not activated (LocState != 2), this function returns true (no error).
 * Otherwise, result telegrams are missing and false is returned (error).
 */
bool sick_lidar_localization::DriverMonitor::resultTelegramsReceiveStatusIsOk(void)
{
  // Check timestamp of last result telegram
  if((ros::Time::now() - m_driver_message_recv_timestamp.get()).toSec() <= m_receive_telegrams_timeout)
    return true; // OK: result telegram received within timeout

  // Call "sRN LocState" and check state of localization (no result telegrams when localization deactivated)
  sick_lidar_localization::SickLocColaTelegramSrv::Request cola_telegram_request;
  sick_lidar_localization::SickLocColaTelegramSrv::Response cola_telegram_response;
  cola_telegram_request.cola_ascii_request = "sRN LocState";
  cola_telegram_request.wait_response_timeout = m_cola_response_timeout;
  if(!serviceCbColaTelegram(cola_telegram_request, cola_telegram_response))
  {
    ROS_WARN_STREAM("## ERROR DriverMonitor: serviceCbColaTelegram failed, cola request: " << sick_lidar_localization::Utils::flattenToString(cola_telegram_request) << ", cola response: " << sick_lidar_localization::Utils::flattenToString(cola_telegram_response));
    return false; // Error: localization controller not responding
  }
  sick_lidar_localization::SickLocColaTelegramMsg cola_response = sick_lidar_localization::ColaParser::decodeColaTelegram(cola_telegram_response.cola_ascii_response);
  if(cola_response.command_name != "LocState" || cola_response.parameter.size() != 1)
  {
    ROS_WARN_STREAM("## ERROR DriverMonitor: unexpected cola response from localization controller, cola request: " << sick_lidar_localization::Utils::flattenToString(cola_telegram_request) << ", cola response: " << sick_lidar_localization::Utils::flattenToString(cola_telegram_response));
    return false; // Error decoding response from localization
  }
  if(cola_response.parameter[0] != "2")
  {
    ROS_INFO_STREAM("DriverMonitor: localization deactivated, no result telegrams received (cola response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ")");
    return true; // OK: SickLocState != "2": localization deactivated, no result telegrams send or received
  }

  // Call "sRN LocResultState" and check state of result output (result telegrams enabled or disabled)
  cola_telegram_request.cola_ascii_request = "sRN LocResultState";
  cola_telegram_request.wait_response_timeout = m_cola_response_timeout;
  if(!serviceCbColaTelegram(cola_telegram_request, cola_telegram_response))
  {
    ROS_WARN_STREAM("## ERROR DriverMonitor: serviceCbColaTelegram failed, cola request: " << sick_lidar_localization::Utils::flattenToString(cola_telegram_request) << ", cola response: " << sick_lidar_localization::Utils::flattenToString(cola_telegram_response));
    return false; // Error: localization controller not responding
  }
  cola_response = sick_lidar_localization::ColaParser::decodeColaTelegram(cola_telegram_response.cola_ascii_response);
  if(cola_response.command_name != "LocResultState" || cola_response.parameter.size() != 1)
  {
    ROS_WARN_STREAM("## ERROR DriverMonitor: unexpected cola response from localization controller, cola request: " << sick_lidar_localization::Utils::flattenToString(cola_telegram_request) << ", cola response: " << sick_lidar_localization::Utils::flattenToString(cola_telegram_response));
    return false; // Error decoding response from localization
  }
  if(cola_response.parameter[0] == "0")
  {
    ROS_INFO_STREAM("DriverMonitor: result telegrams deactivated, no result telegrams received (cola response: " << sick_lidar_localization::Utils::flattenToString(cola_response) << ")");
    return true; // OK: LocResultState == "0": Result telegrams deactivated
  }
    
  // Timeout error: Localization and result telegrams activated, but no result telegrams received
  ROS_WARN_STREAM("## ERROR DriverMonitor: Localization and result telegrams activated, timeout while waiting for result telegrams");
  return false;
}

/*!
 * Thread callback, implements the monitoring and restarts a DriverThread in case of tcp-errors.
 */
void sick_lidar_localization::DriverMonitor::runMonitorThreadCb(void)
{
  ROS_INFO_STREAM("DriverMonitor: monitoring thread started");
  while(ros::ok() && m_monitoring_thread_running)
  {
    ROS_INFO_STREAM("DriverMonitor: starting connection thread");
    sick_lidar_localization::DriverThread* driver_thread = new sick_lidar_localization::DriverThread(m_nh, m_server_adress, m_ip_port_results);
    if(!driver_thread->start())
    {
      ROS_ERROR_STREAM("## ERROR DriverMonitor: could not start tcp client thread");
    }
    // initial wait until tcp connection established
    while(ros::ok() && m_monitoring_thread_running && driver_thread->isRunning() && !driver_thread->isConnected())
    {
      ROS_INFO_STREAM("DriverMonitor: waiting for connection to localization controller");
      ros::Duration(1).sleep(); // wait for initial tcp connection
    }
    // initial wait until monitoring starts
    ros::Time initial_wait_end = ros::Time::now() + ros::Duration(std::max(m_receive_telegrams_timeout, 1.0/m_monitoring_rate));
    while(ros::ok() && m_monitoring_thread_running && driver_thread->isRunning() && driver_thread->isConnected() && ros::Time::now() < initial_wait_end)
    {
      ros::Duration(1).sleep(); // wait for monitoring start
    }
    // Monitor driver messages
    ros::Duration monitoring_delay(1.0/m_monitoring_rate);
    while(ros::ok()
    && m_monitoring_thread_running
    && driver_thread->isRunning()
    && driver_thread->isConnected()
    && resultTelegramsReceiveStatusIsOk())
    {
      monitoring_delay.sleep();
    }
    if(ros::ok() && m_monitoring_thread_running) // timeout, telegram messages from driver missing
    {
      ROS_WARN_STREAM("DriverMonitor: tcp client thread timeout, closing tcp connections and restarting tcp thread.");
      driver_thread->shutdown();
    }
    delete(driver_thread);
    driver_thread = 0;
  }
  m_monitoring_thread_running = false;
  ROS_INFO_STREAM("DriverMonitor: monitoring thread finished");
}

