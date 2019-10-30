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
#ifndef __SIM_LOC_DRIVER_MONITOR_H_INCLUDED
#define __SIM_LOC_DRIVER_MONITOR_H_INCLUDED

#include "sick_lidar_localization/sim_loc_driver_thread.h"
#include "sick_lidar_localization/sim_loc_utils.h"

namespace sick_lidar_localization
{
  /*!
   * Class sick_lidar_localization::DriverMonitor monitors the driver thread implemented by sick_lidar_localization::DriverThread
   * and starts a new driver thread after tcp errors (connection lost, socket shutdown, message timeouts etc.).
   */
  class DriverMonitor
  {
  public:
    
    /*!
     * Constructor. The driver monitor does not start automatically, call start() and stop() to start and stop.
     * @param[in] nh ros node handle
     * @param[in] server_adress ip adress of the localization controller, default: 192.168.0.1
     * @param[in] tcp_port tcp port of the localization controller, default: The localization controller uses IP port number 2201 to send localization results
     */
    DriverMonitor(ros::NodeHandle * nh = 0, const std::string & server_adress = "192.168.0.1", int tcp_port = 2201);
    
    /*!
     * Destructor. Stops the driver thread and monitor and closes all tcp connections.
     */
    virtual ~DriverMonitor();
    
    /*!
     * Starts the driver monitor and driver thread.
     * @return true on success, false on failure.
     */
    virtual bool start(void);
    
    /*!
     * Stops the driver monitor and thread and closes all tcp connections.
     * @return true on success, false on failure.
     */
    virtual bool stop(void);
  
    /*!
     * Callback for result telegram messages (SickLocResultPortTelegramMsg) from sim_loc_driver.
     * @param[in] msg result telegram message (SickLocResultPortTelegramMsg)
     */
    virtual void messageCbResultPortTelegrams(const sick_lidar_localization::SickLocResultPortTelegramMsg & msg);

  protected:
    
    /*!
     * Thread callback, implements the monitoring and restarts a DriverThread in case of tcp-errors.
     */
    virtual void runMonitorThreadCb(void);
    
    /*
     * member data
     */

    bool m_initialized;                      ///< true after successfull configuration, otherwise false
    ros::NodeHandle* m_nh;                   ///< ros node handle
    std::string m_server_adress;             ///< ip adress of the localization controller, default: 192.168.0.1
    int m_tcp_port;                          ///< tcp port of the localization controller, default: The localization controller uses IP port number 2201 to send localization results
    bool m_monitoring_thread_running;        ///< true: m_monitoring_thread is running, otherwise false
    boost::thread* m_monitoring_thread;      ///< thread to monitor tcp-connection, restarts DriverThread in case of tcp-errors.
    sick_lidar_localization::SetGet<ros::Time> m_driver_message_recv_timestamp; ///< time of the last received driver message (threadsafe)
    double m_monitoring_rate;                ///< frequency to monitor driver messages, default: once per second
    double m_receive_telegrams_timeout;      ///< timeout for driver messages, shutdown tcp-sockets and reconnect after message timeout, default: 1 second
    
  }; // class DriverMonitor
  
} // namespace sick_lidar_localization
#endif // __SIM_LOC_DRIVER_MONITOR_H_INCLUDED
