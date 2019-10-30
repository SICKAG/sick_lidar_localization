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

#include "sick_lidar_localization/sim_loc_driver_monitor.h"

/*
 * Constructor. The driver monitor does not start automatically, call start() and stop() to start and stop.
 * @param[in] nh ros node handle
 * @param[in] server_adress ip adress of the localization controller, default: 192.168.0.1
 * @param[in] tcp_port tcp port of the localization controller, default: The localization controller uses IP port number 2201 to send localization results
 */
sick_lidar_localization::DriverMonitor::DriverMonitor(ros::NodeHandle * nh, const std::string & server_adress, int tcp_port)
: m_initialized(false), m_nh(nh), m_server_adress(server_adress), m_tcp_port(tcp_port), m_monitoring_thread_running(false),
  m_monitoring_thread(0), m_monitoring_rate(1.0), m_receive_telegrams_timeout(1.0)
{
  if(m_nh)
  {
    // Query configuration
    ros::param::param<double>("/sick_lidar_localization/driver/monitoring_rate", m_monitoring_rate, m_monitoring_rate); // frequency to monitor driver messages, default: once per second
    ros::param::param<double>("/sick_lidar_localization/driver/monitoring_message_timeout", m_receive_telegrams_timeout, m_receive_telegrams_timeout); // timeout for driver messages, shutdown tcp-sockets and reconnect after message timeout, default: 1 second
    m_initialized = true;
  }
}

/*
 * Destructor. Stops the driver thread and monitor and closes all tcp connections.
 */
sick_lidar_localization::DriverMonitor::~DriverMonitor()
{
  stop();
}

/*
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

/*
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
  return true;
}

/*
 * Callback for result telegram messages (SickLocResultPortTelegramMsg) from sim_loc_driver.
 * @param[in] msg result telegram message (SickLocResultPortTelegramMsg)
 */
void sick_lidar_localization::DriverMonitor::messageCbResultPortTelegrams(const sick_lidar_localization::SickLocResultPortTelegramMsg & msg)
{
  m_driver_message_recv_timestamp.set(ros::Time::now());
}

/*
 * Thread callback, implements the monitoring and restarts a DriverThread in case of tcp-errors.
 */
void sick_lidar_localization::DriverMonitor::runMonitorThreadCb(void)
{
  ROS_INFO_STREAM("DriverMonitor: monitoring thread started");
  while(ros::ok() && m_monitoring_thread_running)
  {
    ROS_INFO_STREAM("DriverMonitor: starting connection thread");
    sick_lidar_localization::DriverThread* driver_thread = new sick_lidar_localization::DriverThread(m_nh, m_server_adress, m_tcp_port);
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
    && (ros::Time::now() - m_driver_message_recv_timestamp.get()).toSec() <= m_receive_telegrams_timeout)
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

