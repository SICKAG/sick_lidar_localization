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

#include "sick_lidar_localization/cola_transmitter.h"
#include "sick_lidar_localization/driver_thread.h"
#include "sick_lidar_localization/utils.h"

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
    DriverMonitor(ROS::NodePtr nh = 0, const std::string & server_adress = "192.168.0.1", int ip_port_results = 2201, int ip_port_cola = 2111);
    
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
      /*! ROS2 version of function messageCbResultPortTelegrams */
    virtual void messageCbResultPortTelegramsROS2(const std::shared_ptr<sick_lidar_localization::SickLocResultPortTelegramMsg> msg) { messageCbResultPortTelegrams(*msg); }

    /*!
     * Callback for service messages (SickLocColaTelegramSrv). Handles Cola requests and responses, sends and receives Cola-ASCII telegrams to resp. from
     * the localization controller
     * @param[in] cola_request Cola command request, will be encoded and send to the localization controller
     * @param[out] cola_response Cola command response from localization controller
     */
    virtual bool serviceCbColaTelegram(sick_lidar_localization::SickLocColaTelegramSrv::Request & cola_request, sick_lidar_localization::SickLocColaTelegramSrv::Response & cola_response);
    /*! ROS2 version of function serviceCbColaTelegram */
    virtual bool serviceCbColaTelegramROS2(std::shared_ptr<sick_lidar_localization::SickLocColaTelegramSrv::Request> cola_request, std::shared_ptr<sick_lidar_localization::SickLocColaTelegramSrv::Response> cola_response)
    {
      return serviceCbColaTelegram(*cola_request, *cola_response);
    }

  protected:

    /*!
     * Initializes cola sender and receiver for cola requests and responses
     * @param[in] server_adress ip adress of the localization controller, default: 192.168.0.1
     * @param[in] ip_port_cola ip port for command requests and responses, default: 2111
     * @param[in] receive_timeout timeout in seconds for receive functions
     */
    bool initColaTransmitter(const std::string & server_adress, int ip_port_cola, double receive_timeout);
    
    /*!
     * Stops cola sender and receiver for cola requests and responses
     * (if started by a cola service request "SickLocColaTelegram")
     */
    void stopColaTransmitter(void);

    /*!
     * Returns true, if result telegrams have been received within configured timeout "monitoring_message_timeout".
     * If no result telegrams have been received within the timeout, the localization status is queried by ros service
     * "SickLocState". If localization is not activated (LocState != 2), this function returns true (no error).
     * Otherwise, result telegrams are missing and false is returned (error).
     */
    bool resultTelegramsReceiveStatusIsOk(void);
    
    /*!
     * Thread callback, implements the monitoring and restarts a DriverThread in case of tcp-errors.
     */
    virtual void runMonitorThreadCb(void);
    
    /*
     * member data
     */

    bool m_initialized;                      ///< true after successfull configuration, otherwise false
    ROS::NodePtr m_nh;                       ///< ros node handle
    std::string m_server_adress;             ///< ip adress of the localization controller, default: 192.168.0.1
    int m_ip_port_results;                   ///< ip port for result telegrams, default: The localization controller uses IP port number 2201 to send localization results
    int m_ip_port_cola;                      ///< ip port for for command requests and responses, default: The localization controller uses IP port number 2111 and 2112 to send telegrams and to request data
    bool m_cola_binary;                      ///< false: send Cola-ASCII (default), true: send Cola-Binary
    bool m_monitoring_thread_running;        ///< true: m_monitoring_thread is running, otherwise false
    boost::thread* m_monitoring_thread;      ///< thread to monitor tcp-connection, restarts DriverThread in case of tcp-errors.
    sick_lidar_localization::SetGet<ROS::Time> m_driver_message_recv_timestamp; ///< time of the last received driver message (threadsafe)
    double m_monitoring_rate;                ///< frequency to monitor driver messages, default: once per second
    double m_receive_telegrams_timeout;      ///< timeout for driver messages, shutdown tcp-sockets and reconnect after message timeout, default: 1 second
    double m_cola_response_timeout;          ///< timeout in seconds for cola responses from localization controller, default: 1
    sick_lidar_localization::ColaTransmitter* m_cola_transmitter; ///< transmitter for cola commands (send requests, receive responses)
    boost::mutex m_service_cb_mutex;          ///< mutex to protect serviceCbColaTelegram (one service request at a time)
  
  }; // class DriverMonitor
  
} // namespace sick_lidar_localization
#endif // __SIM_LOC_DRIVER_MONITOR_H_INCLUDED
