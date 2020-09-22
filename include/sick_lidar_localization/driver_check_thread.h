/*
 * @brief sim_loc_driver_check_thread implements a thread to check
 * sim_loc_driver messages against configured min and max values.
 *
 * This way a scene specific plausibility check of sim_loc_driver messages
 * can be performed when running automated tests against localization controllers
 * like SIM1000FXA. A warning will be logged in case of failures or values out of
 * range.
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
#ifndef __SIM_LOC_DRIVER_CHECK_THREAD_H_INCLUDED
#define __SIM_LOC_DRIVER_CHECK_THREAD_H_INCLUDED

#include <list>
#include <string>
#include <vector>

#include "sick_lidar_localization/ros_wrapper.h"
#include "sick_lidar_localization/fifo_buffer.h"

namespace sick_lidar_localization
{
  /*!
   * class MessageCheckThread implements a thread to check
   * sim_loc_driver messages against configured min and max values.
   * A warning will be logged in case of failures or mismatches.
   */
  class MessageCheckThread
  {
  public:
    
    /*!
     * Constructor, reads the configuration parameter incl. the
     * min and max allowed values in sim_loc_driver messages.
     * @param[in] nh ros node handle
     */
    MessageCheckThread(ROS::NodePtr nh = 0);
    
    /*!
     * Destructor
     */
    virtual ~MessageCheckThread();
    
    /*!
     * Starts the sim_loc_driver message check thread.
     * @return true on success, false on failure.
     */
    virtual bool start(void);
    
    /*!
     * Stops the sim_loc_driver message check thread.
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
    
  protected:
    
    /*!
     * Thread callback, checks sim_loc_driver messages in m_result_port_telegram_fifo
     * against min and max values.
     */
    virtual void runMessageCheckThreadCb(void);

    /*!
     * Reads and returns a result port telegram from yaml configuration file.
     * @param[in] nh ros node handle
     * @param[in] param_section section name in yaml configuration file, f.e. "/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values"
     * @return result port telegram with values initialized from yaml file.
     */
    sick_lidar_localization::SickLocResultPortTelegramMsg readYamlResultPortTelegram(ROS::NodePtr nh, const std::string param_section);

    /*!
     * Checks a result telegram messages (SickLocResultPortTelegramMsg) against min and max values.
     * Returns true, if test passed (all values within their ranges), or false otherwise.
     * @param[in] telegram result telegram message (SickLocResultPortTelegramMsg)
     * @return true, if test passed, false otherwise.
     */
    virtual bool checkTelegram(sick_lidar_localization::SickLocResultPortTelegramMsg & telegram);

    /*!
     * Checks the vehicle time of a result telegram message (system time from ticks by software pll) against min and max
     * allowed difference to ros::Time::now(). Returns true, if test passed (vehicle time  within their range), or false otherwise.
     * @param[in] telegram result telegram message (SickLocResultPortTelegramMsg)
     * @return true, if test passed, false otherwise.
     */
    virtual bool checkVehicleTime(sick_lidar_localization::SickLocResultPortTelegramMsg & telegram);
    
    /*
     * member data
     */
    
    sick_lidar_localization::FifoBuffer<sick_lidar_localization::SickLocResultPortTelegramMsg, boost::mutex> m_result_port_telegram_fifo; ///< fifo buffer for result port telegrams from sim_loc_driver
    sick_lidar_localization::SickLocResultPortTelegramMsg m_result_port_telegram_min_values; ///< min allowed values of result port telegrams
    sick_lidar_localization::SickLocResultPortTelegramMsg m_result_port_telegram_max_values; ///< max allowed values of result port telegrams
    double m_vehicle_time_delta_min;                        ///< min. allowed time diff in seconds between vehicle time (system time from ticks by software pll) and ros::Time::now()
    double m_vehicle_time_delta_max;                        ///< max. allowed time diff in seconds between vehicle time (system time from ticks by software pll) and ros::Time::now()
    bool m_vehicle_time_check_enabled;                      ///< true: check of vehicle time is enabled (default), false in case of simulated network errors (LocRequestTimestamp not available)
    double m_software_pll_expected_initialization_duration; ///< expected initialization time for software pll (system time from lidar ticks not yet available)
    ROS::Time m_timestamp_valid_telegram;      ///< timestamp of a telegram with valid vehicle time
    bool m_message_check_thread_running;       ///< true: m_message_check_thread is running, otherwise false
    boost::thread* m_message_check_thread;     ///< thread to check sim_loc_driver messages
    double m_message_check_frequency;          ///< frequency to check sim_loc_driver messages (default: 100)
    
  };
  
} // namespace sick_lidar_localization
#endif // __SIM_LOC_DRIVER_CHECK_THREAD_H_INCLUDED
