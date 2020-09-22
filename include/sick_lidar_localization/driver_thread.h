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
#ifndef __SIM_LOC_DRIVER_THREAD_H_INCLUDED
#define __SIM_LOC_DRIVER_THREAD_H_INCLUDED

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <list>

#include "sick_lidar_localization/client_socket.h"
#include "sick_lidar_localization/fifo_buffer.h"

namespace sick_lidar_localization
{
  /*!
   * Class sick_lidar_localization::DriverThread creates and runs two threads:
   * - a tcp thread to connect to the localization controller (f.e. SIM1000FXA)
   *   and receive binary result port telegrams, and
   * - a converter thread to convert the binary telegrams to SickLocResultPortTelegramMsg
   *   and to publish the messages.
   * Both threads uses a threadsafe fifo for data exchange.
   */
  class DriverThread
  {
  public:
    
    /*!
     * Constructor. The driver thread does not start automatically, call start() and stop() to start and stop the driver.
     * @param[in] nh ros node handle
     * @param[in] server_adress ip adress of the localization controller, default: 192.168.0.1
     * @param[in] tcp_port tcp port of the localization controller, default: The localization controller uses IP port number 2201 to send localization results
     */
    DriverThread(ROS::NodePtr nh = 0, const std::string & server_adress = "192.168.0.1", int tcp_port = 2201);
    
    /*!
     * Destructor. Stops the driver thread and closes all tcp connections.
     */
    virtual ~DriverThread();
    
    /*!
     * Starts the driver thread, i.e. starts to receive binary telegrams from the localization controller
     * and starts to publish SickLocResultPortTelegramMsg messages containing the localization data.
     * @return true on success, false on failure.
     */
    virtual bool start(void);
  
    /*!
     * Stops the driver thread and closes all tcp connections.
     * @return true on success, false on failure.
     */
    virtual bool stop(void) { return stop(false); }
  
    /*!
     * Forces a shutdown of all tcp connections and the driver thread.
     * @return true on success, false on failure.
     */
    virtual bool shutdown(void) { return stop(true); }
  
    /*!
     * Returns true, if tcp-connection to the localization controller is established and driver thread is running.
     * @return true if driver thread is running and connected to the localization controller, false otherwise.
     */
    virtual bool isConnected(void);
  
    /*!
     * Returns true, if driver thread is running (but not necessarily connected to a localization controller).
     * @return true if driver thread is running, false otherwise.
     */
    virtual bool isRunning(void);

  protected:
  
    /*!
     * Stops the driver thread and closes all tcp connections.
     * @param[in] force_shutdown if true, sockets are immediately forced to shutdown, otherwise stop() waits until all operations finished.
     * @return true on success, false on failure.
     */
    virtual bool stop(bool force_shutdown);
  
    /*!
     * Closes all tcp connections
     * @param[in] force_shutdown if true, sockets are immediately forced to shutdown
     */
    virtual void closeTcpConnections(bool force_shutdown);
    
    /*!
     * Thread callback, connects to the localization controller and receives binary result port telegrams.
     */
    virtual void runReceiverThreadCb(void);
  
    /*!
     * Thread callback, converts the binary result port telegrams to SickLocResultPortTelegramMsg
     * and publishes all localization data.
     */
    virtual void runConverterThreadCb(void);
  
    /*
     * member data
     */
  
    bool m_initialized;                                     ///< true after successfull initialization (publisher, config parameter, etc.), otherwise false
    bool m_tcp_connected;                                   ///< true if a tcp connection to the localization controller is established, otherwise false
    std::string m_server_adress;                            ///< ip adress of the localization controller, default: 192.168.0.1
    int m_tcp_port;                                         ///< tcp port of the localization controller, default: The localization controller uses IP port number 2201 to send localization results
    double m_tcp_connection_retry_delay;                    ///< delay in seconds to retry to connect to the localization controller, default: 1 second
    boost::asio::io_service m_ioservice;                    ///< boost io service for tcp connections
    sick_lidar_localization::ClientSocket m_tcp_socket;     ///< tcp socket connected to the localization controller
    boost::thread* m_tcp_receiver_thread;                   ///< thread to receive telegrams from the localization controller
    bool m_tcp_receiver_thread_running;                     ///< true: m_tcp_receiver_thread is running, otherwise false
    boost::thread* m_converter_thread;                      ///< thread to convert and publish localization data
    bool m_converter_thread_running;                        ///< true: m_converter_thread is running, otherwise false
    sick_lidar_localization::FifoBuffer<std::vector<uint8_t>, boost::mutex> m_fifo_buffer; ///< fifo buffer to transfer data from receiver thread to converter thread
    sick_lidar_localization::SickLocResultPortTelegramMsgPublisher m_result_telegrams_publisher;            ///< ros publisher for result port telegram messages (type SickLocResultPortTelegramMsg)
    std::string m_result_telegrams_frame_id;                ///< ros frame id of result port telegram messages (type SickLocResultPortTelegramMsg), default: "sick_lidar_localization"
    sick_lidar_localization::SickLocTimeSyncSrvClient m_timesync_service_client;  ///< client to call ros service "SickLocTimeSync" to calculate system time from ticks by software pll
    double m_software_pll_expected_initialization_duration; ///< expected initialization time for software pll (system time from lidar ticks not yet available)
  
    /*
     * configuration and member data for diagnostic messages
     */
    typedef enum DRIVER_ERROR_CODES_ENUM                ///< Enumerates the error codes in driver diagnostic messages
    {
      NO_ERROR,            ///< no error, driver works as expected
      NO_TCP_CONNECTION,   ///< tcp connection to localization controller could not be established
      PARSE_ERROR,         ///< Telegram could not be decoded
      CONFIGURATION_ERROR, ///< invalid driver configuration
      INTERNAL_ERROR       ///< Internal error
    } DRIVER_ERROR_CODES;  ///< Enumeration of error codes in driver diagnostic message

    /*!
     * Publishes a diagnostic message.
     * @param[in] error_code one of the error codes enumerated by DRIVER_ERROR_CODES
     * @param[in] message diagnostical message
     */
    void publishDiagnosticMessage(const DRIVER_ERROR_CODES & error_code, const std::string & message);
    
    sick_lidar_localization::SickLocDiagnosticMsgPublisher m_diagnostic_publisher; ///< ros publisher for diagnostic messages (type SickLocDiagnosticMsg)
    std::string m_diagnostic_frame_id;                                             ///< ros frame id of diagnostic messages (type SickLocDiagnosticMsg), default: "sick_lidar_localization"
    
  }; // class DriverThread
  
} // namespace sick_lidar_localization
#endif // __SIM_LOC_DRIVER_THREAD_H_INCLUDED
