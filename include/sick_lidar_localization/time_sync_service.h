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
#ifndef __SIM_LOC_TIME_SYNC_SERVICE_H_INCLUDED
#define __SIM_LOC_TIME_SYNC_SERVICE_H_INCLUDED

#include "sick_lidar_localization/cola_parser.h"
#include "sick_lidar_localization/driver_monitor.h"
#include "sick_lidar_localization/utils.h"

namespace sick_lidar_localization
{
  /*!
   * Class sick_lidar_localization::TimeSyncService implements ros services "SickLocRequestTimestamp"
   * and "SickLocTimeSync" for time synchronization.
   * It sends LocRequestTimestamp requests to the localization controller, receives the response and
   * calculates the time offset.
   */
  class TimeSyncService
  {
  public:
    
    /*!
     * Constructor
     */
    TimeSyncService(ROS::NodePtr nh = 0, sick_lidar_localization::DriverMonitor* driver_monitor = 0);
    
    /*!
     * Destructor
     */
    virtual ~TimeSyncService();
  
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
    virtual bool start(void);
  
    /*!
     * Stops the time synchronization thread and the software pll.
     * @return true on success, false in case of errors.
     */
    virtual bool stop(void);
    
    /*!
     * Callback for service messages (SickLocRequestTimestamp). Sends LocRequestTimestamp requests
     * to the localization controller, receives the response and calculates the time offset.
     * @param[in] service_request ros service request
     * @param[out] service_response service response with timestamps and calculated time offset.
     * @return true on success, false in case of errors.
     */
    virtual bool serviceCbRequestTimestamp(sick_lidar_localization::SickLocRequestTimestampSrv::Request & service_request, sick_lidar_localization::SickLocRequestTimestampSrv::Response & service_response);
    /*! ROS2 version of function serviceCbRequestTimestamp */
    virtual bool serviceCbRequestTimestampROS2(std::shared_ptr<sick_lidar_localization::SickLocRequestTimestampSrv::Request> service_request, std::shared_ptr<sick_lidar_localization::SickLocRequestTimestampSrv::Response> service_response)
    {
      return serviceCbRequestTimestamp(*service_request, *service_response);
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
    virtual bool serviceCbTimeSync(sick_lidar_localization::SickLocTimeSyncSrv::Request & time_sync_request, sick_lidar_localization::SickLocTimeSyncSrv::Response & time_sync_response);
    /*! ROS2 version of function serviceCbTimeSync */
    virtual bool serviceCbTimeSyncROS2(std::shared_ptr<sick_lidar_localization::SickLocTimeSyncSrv::Request> time_sync_request, std::shared_ptr<sick_lidar_localization::SickLocTimeSyncSrv::Response> time_sync_response)
    {
      return serviceCbTimeSync(*time_sync_request, *time_sync_response);
    }
  
    /*!
     * Thread callback, runs time synchronization, calls ros service "SickLocRequestTimestamp" each 10 seconds
     * and updates the software pll.
     */
    virtual void runTimeSyncThreadCb(void);
    
  protected:

    /*!
     * Update the software pll with lidar ticks and system time from response by "SickLocRequestTimestamp" servcice
     * @param[out] service_response service response by "SickLocRequestTimestamp" servcice
     */
    void updateSoftwarePll(sick_lidar_localization::SickLocRequestTimestampSrv::Response & service_response);

    /*!
     * Returns true, if the initialization phase of the software pll is completed, otherwise false.
     * @return initialization phase of the software pll completed
     */
    bool isSoftwarePllInitialized(void);
    
    ROS::NodePtr m_nh; ///< ros node handle
    sick_lidar_localization::DriverMonitor* m_driver_monitor;                             ///< implements the cola telegram services
    sick_lidar_localization::SickLocRequestTimestampSrvServer m_timestamp_service_server; ///< provides ros service "SickLocRequestTimestamp" to send a LocRequestTimestamp, receive the response and to calculate the time offset
    sick_lidar_localization::SickLocTimeSyncSrvServer m_timesync_service_server;          ///< provides ros service "SickLocTimeSync" to calculate system time from ticks by software pll
    sick_lidar_localization::SickLocColaTelegramSrvClient m_cola_service_client;          ///< client to call ros service "SickLocColaTelegram" to send cola telegrams and receive cola responses from localization controller
    // sick_lidar_localization::SickLocRequestTimestampSrvClient m_request_timestamp_client; ///< client to call ros service "SickLocRequestTimestamp"
    bool m_time_sync_thread_running;               ///< true: m_time_sync_thread is running, otherwise false
    boost::thread* m_time_sync_thread;             ///< thread to synchronize timestamps, runs the software pll
    bool m_cola_binary;                            ///< false: send Cola-ASCII (default), true: send Cola-Binary
    int m_cola_binary_mode;                        ///< 0: send Cola-ASCII (default), 1: send Cola-Binary, 2: toggle between Cola-ASCII and Cola-Binary (test and development only!)
    int m_software_pll_fifo_length;                ///< length of software pll fifo, default: 7
    double m_time_sync_rate;                       ///< frequency to request timestamps using ros service "SickLocRequestTimestamp" and to update software pll, default: 0.1 (LocRequestTimestamp queries every 10 seconds)
    double m_time_sync_initial_rate;               ///< frequency to request timestamps and to update software pll during initialization phase, default: 1.0 (LocRequestTimestamp queries every second)
    int m_time_sync_initial_length;                ///< length of initialization phase with LocRequestTimestamps every second, default: 10 (i.e. 10 LocRequestTimestamp queries every second after start, otherwise LocRequestTimestamp queries every 10 seconds)
    double m_cola_response_timeout;                ///< Timeout in seconds for cola responses from localization controller, default: 1
    boost::mutex m_software_pll_mutex;             ///< mutex to protect access to software pll used in service "SickLocTimeSync
    boost::mutex m_service_cb_mutex;               ///< mutex to protect serviceCbRequestTimestamp (one service request at a time)
  
  }; // class TimeSyncService
  
} // namespace sick_lidar_localization
#endif // __SIM_LOC_TIME_SYNC_SERVICE_H_INCLUDED
