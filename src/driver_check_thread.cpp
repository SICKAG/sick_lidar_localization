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
#include <algorithm>
#include "sick_lidar_localization/ros_wrapper.h"
#include <boost/algorithm/clamp.hpp>

#include "sick_lidar_localization/driver_check_thread.h"
#include "sick_lidar_localization/utils.h"

/*
 * Constructor, reads the configuration parameter incl. the
 * min and max allowed values in sim_loc_driver messages.
 */
sick_lidar_localization::MessageCheckThread::MessageCheckThread(ROS::NodePtr nh)
: m_message_check_thread_running(false), m_message_check_thread(0), m_message_check_frequency(100), m_timestamp_valid_telegram(0)
{
  if(nh) // Read configuation
  {
    int software_pll_fifo_length = 7, time_sync_initial_length = 10;
    double time_sync_rate = 0.1, time_sync_initial_rate = 1.0;
    ROS::param<double>(nh, "/sick_lidar_localization/sim_loc_driver_check/message_check_frequency", m_message_check_frequency, m_message_check_frequency);
    ROS::param<int>(nh, "/sick_lidar_localization/time_sync/software_pll_fifo_length", software_pll_fifo_length, software_pll_fifo_length);
    ROS::param<double>(nh, "/sick_lidar_localization/time_sync/time_sync_rate", time_sync_rate, time_sync_rate);
    ROS::param<double>(nh, "/sick_lidar_localization/time_sync/time_sync_initial_rate", time_sync_initial_rate, time_sync_initial_rate);
    ROS::param<int>(nh, "/sick_lidar_localization/time_sync/time_sync_initial_length", time_sync_initial_length, time_sync_initial_length);
    assert(software_pll_fifo_length > 0 && time_sync_rate > FLT_EPSILON);
    m_software_pll_expected_initialization_duration = (software_pll_fifo_length / time_sync_initial_rate + 1); // expected initialization time for software pll (system time from lidar ticks not yet available)
    // Read min allowed values in a result port telegrams
    m_result_port_telegram_min_values = readYamlResultPortTelegram(nh, "/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values");
    m_result_port_telegram_min_values.vehicle_time_valid = false;
    m_result_port_telegram_min_values.vehicle_time_sec = 0;
    m_result_port_telegram_min_values.vehicle_time_nsec = 0;
    ROS::param<double>(nh, "/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/vehicle_time_delta", m_vehicle_time_delta_min, -1.0);
    ROS::param<bool>(nh, "/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/check_vehicle_time", m_vehicle_time_check_enabled, true);
    // Read max allowed values in a result port telegrams
    m_result_port_telegram_max_values = readYamlResultPortTelegram(nh, "/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values");
    m_result_port_telegram_max_values.vehicle_time_valid = true;
    m_result_port_telegram_max_values.vehicle_time_sec = UINT32_MAX;
    m_result_port_telegram_max_values.vehicle_time_nsec = UINT32_MAX;
    m_result_port_telegram_max_values.telegram_header.fw_version = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    ROS::param<double>(nh, "/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/vehicle_time_delta", m_vehicle_time_delta_max, 1.0);
    ROS::param<bool>(nh, "/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/check_vehicle_time", m_vehicle_time_check_enabled, true);
    ROS_INFO_STREAM("MessageCheckThread: min allowed values in result port telegrams: " << sick_lidar_localization::Utils::flattenToString(m_result_port_telegram_min_values));
    ROS_INFO_STREAM("MessageCheckThread: max allowed values in result port telegrams: " << sick_lidar_localization::Utils::flattenToString(m_result_port_telegram_max_values));
  }
}

/*
 * Destructor
 */
sick_lidar_localization::MessageCheckThread::~MessageCheckThread()
{
  stop();
}

/*
 * Starts the sim_loc_driver message check thread.
 * @return true on success, false on failure.
 */
bool sick_lidar_localization::MessageCheckThread::start(void)
{
  m_message_check_thread_running = true;
  m_message_check_thread = new boost::thread(&sick_lidar_localization::MessageCheckThread::runMessageCheckThreadCb, this);
  return true;
}

/*
 * Stops the sim_loc_driver message check thread.
 * @return true on success, false on failure.
 */
bool sick_lidar_localization::MessageCheckThread::stop(void)
{
  m_message_check_thread_running = false;
  if(m_message_check_thread)
  {
    m_message_check_thread->join();
    delete(m_message_check_thread);
    m_message_check_thread = 0;
  }
  return true;
}

/*
 * Callback for result telegram messages (SickLocResultPortTelegramMsg) from sim_loc_driver.
 * @param[in] msg result telegram message (SickLocResultPortTelegramMsg)
 */
void sick_lidar_localization::MessageCheckThread::messageCbResultPortTelegrams(const sick_lidar_localization::SickLocResultPortTelegramMsg & msg)
{
  m_result_port_telegram_fifo.push(msg);
}

/*
 * Thread callback, checks sim_loc_driver messages in m_result_port_telegram_fifo
 * against min and max values.
 */
void sick_lidar_localization::MessageCheckThread::runMessageCheckThreadCb(void)
{
  ROS_INFO_STREAM("MessageCheckThread: thread to check sim_loc_driver messages started");
  size_t total_message_check_cnt = 0, total_message_check_failed_cnt = 0;
  while(ROS::ok() && m_message_check_thread_running)
  {
    while(ROS::ok() && m_message_check_thread_running && m_result_port_telegram_fifo.empty())
    {
      ROS::sleep(1.0 / m_message_check_frequency);
    }
    if(ROS::ok() && m_message_check_thread_running && !m_result_port_telegram_fifo.empty())
    {
      // Check sim_loc_driver message
      sick_lidar_localization::SickLocResultPortTelegramMsg result_port_telegram = m_result_port_telegram_fifo.pop();
      total_message_check_cnt++;
      if (!checkTelegram(result_port_telegram))
      {
        ROS_WARN_STREAM("## ERROR MessageCheckThread: driver message check failed (" << total_message_check_cnt << ". driver message)");
        ROS_WARN_STREAM("## driver message (received): " << sick_lidar_localization::Utils::flattenToString(result_port_telegram));
        ROS_WARN_STREAM("## MessageCheckThread: min allowed values in result port telegrams: " << sick_lidar_localization::Utils::flattenToString(m_result_port_telegram_min_values));
        ROS_WARN_STREAM("## MessageCheckThread: max allowed values in result port telegrams: " << sick_lidar_localization::Utils::flattenToString(m_result_port_telegram_max_values));
        total_message_check_failed_cnt++;
      }
      else
      {
        ROS_DEBUG_STREAM("MessageCheckThread: " << total_message_check_cnt << ". driver message checked, okay");
        ROS_DEBUG_STREAM("MessageCheckThread: driver message (received): " << sick_lidar_localization::Utils::flattenToString(result_port_telegram));
      }
    }
  }
  ROS_INFO_STREAM("MessageCheckThread: thread to check sim_loc_driver messages finished");
  std::stringstream info_msg;
  info_msg << "MessageCheckThread: check messages thread summary: " << total_message_check_cnt << " messages checked, " << total_message_check_failed_cnt << " failures.";
  if(ROS::ok())
    ROS_INFO_STREAM(info_msg.str());
  else
    std::cout << info_msg.str() << std::endl;
}

/*
 * Reads and returns a result port telegram from yaml configuration file.
 * @param[in] param_section section name in yaml configuration file, f.e. "/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values"
 * @return result port telegram with values initialized from yaml file.
 */
sick_lidar_localization::SickLocResultPortTelegramMsg sick_lidar_localization::MessageCheckThread::readYamlResultPortTelegram(ROS::NodePtr nh, const std::string param_section)
{
  int i_value = 0;
  double d_value = 0;
  std::vector<int> i_vec;
  std::vector<uint8_t> u8_vec;
  sick_lidar_localization::SickLocResultPortTelegramMsg telegram;
  
  // Default values, can be overwritten by parameter settings
  telegram.vehicle_time_valid = false;
  telegram.vehicle_time_sec = 0;
  telegram.vehicle_time_nsec = 0;
  
  // Read ros header
#if defined __ROS_VERSION && __ROS_VERSION == 1  // telegram.header.seq does not exist anymore in ROS2
  ROS::param<double>(nh, param_section+"/header/seq", d_value, 0);
  telegram.header.seq = (uint32_t)boost::algorithm::clamp<double>(d_value, 0, 0xFFFFFFFF); // sequence ID, consecutively increasing ID, uint32, size:= 4 byte
#endif  
  ROS::param<double>(nh, param_section+"/header/stamp", d_value, 0);
  telegram.header.stamp = ROS::timeFromSec(d_value); // time stamp
  ROS::param<std::string>(nh, param_section+"/header/frame_id", telegram.header.frame_id, "");
  
  // Read telegram header, 52 byte
  ROS::param<int>(nh, param_section+"/telegram_header/MagicWord", i_value, 0x5349434B);
  telegram.telegram_header.magicword = (uint32_t)i_value; // Magic word SICK (0x53 0x49 0x43 0x4B). uint32, size:= 4 × UInt8 = 4 byte
  ROS::param<int>(nh, param_section+"/telegram_header/Length", i_value, 106);
  telegram.telegram_header.length = (uint32_t)i_value; // Length of telegram incl. header, payload, and trailer. uint32, size:= UInt32 = 4 byte
  ROS::param<int>(nh, param_section+"/telegram_header/PayloadType", i_value, 0x0642);
  telegram.telegram_header.payloadtype = (uint16_t)i_value; // Payload type, 0x06c2 = Little Endian, 0x0642 = Big Endian. uint16, size:= UInt16 = 2 byte
  ROS::param<int>(nh, param_section+"/telegram_header/PayloadVersion", i_value, 1);
  telegram.telegram_header.payloadversion = (uint16_t)i_value; // Version of PayloadType structure. uint16, size:= UInt16 = 2 byte
  ROS::param<double>(nh, param_section+"/telegram_header/OrderNumber", d_value, 0);
  telegram.telegram_header.ordernumber = (uint32_t)boost::algorithm::clamp<double>(d_value, 0, 0xFFFFFFFF); // Order number of the localization controller. uint32, size:= UInt32 = 4 byte
  ROS::param<double>(nh, param_section+"/telegram_header/SerialNumber", d_value, 0);
  telegram.telegram_header.serialnumber = (uint32_t)boost::algorithm::clamp<double>(d_value, 0, 0xFFFFFFFF); // Serial number of the localization controller. uint32, size:= UInt32 = 4 byte
  // ROS::param<std::vector<int>>(nh, param_section+"/telegram_header/FW_Version", i_vec, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  // u8_vec.reserve(i_vec.size());
  // for(size_t n = 0; n < i_vec.size(); n++)
  //   u8_vec.push_back(i_vec[n]);
  u8_vec = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  telegram.telegram_header.fw_version = u8_vec; // Software version of the localization controller. uint8[], size:= 20 × UInt8 = 20 byte
  ROS::param<double>(nh, param_section+"/telegram_header/TelegramCounter", d_value, 0);
  telegram.telegram_header.telegramcounter = (uint32_t)boost::algorithm::clamp<double>(d_value, 0, 0xFFFFFFFF); // Telegram counter since last start-up. uint32, size:= UInt32 = 4 byte
  ROS::param<double>(nh, param_section+"/telegram_header/SystemTime", d_value, 0);
  telegram.telegram_header.systemtime = ((d_value <= 0)?0:((d_value >= 0xFFFFFFFFFFFFFFFF)?0xFFFFFFFFFFFFFFFF:(uint64_t)(d_value))); // Not used. uint64, size:= NTP = 8 byte
  
  // Read telegram payload, 52 byte
  ROS::param<int>(nh, param_section+"/telegram_payload/ErrorCode", i_value, 0);
  telegram.telegram_payload.errorcode = (uint16_t)i_value; // ErrorCode 0:= OK, ErrorCode 1:= UNKNOWNERROR. uint16, size:= UInt16 = 2 byte
  ROS::param<double>(nh, param_section+"/telegram_payload/ScanCounter", d_value, 0);
  telegram.telegram_payload.scancounter = (uint32_t)boost::algorithm::clamp<double>(d_value, 0, 0xFFFFFFFF); // Counter of related scan data. uint32, size:= UInt32 = 4 byte
  ROS::param<double>(nh, param_section+"/telegram_payload/Timestamp", d_value, 0);
  telegram.telegram_payload.timestamp = (uint32_t)boost::algorithm::clamp<double>(d_value, 0, 0xFFFFFFFF); // Time stamp of the pose [ms]. The time stamp indicates the time at which the pose is calculated. uint32, size:= UInt32 = 4 byte
  ROS::param<int>(nh, param_section+"/telegram_payload/PoseX", telegram.telegram_payload.posex, 0); // Position X of the vehicle on the map in cartesian global coordinates [mm]. int32, size:= Int32 = 4 byte
  ROS::param<int>(nh, param_section+"/telegram_payload/PoseY", telegram.telegram_payload.posey, 0); // Position Y of the vehicle on the map in cartesian global coordinates [mm]. int32, size:= Int32 = 4 byte
  ROS::param<int>(nh, param_section+"/telegram_payload/PoseYaw", telegram.telegram_payload.poseyaw, 0); // Orientation (yaw) of the vehicle on the map [mdeg]. int32, size:= Int32 = 4 byte
  ROS::param<double>(nh, param_section+"/telegram_payload/Reserved1", d_value, 0);
  telegram.telegram_payload.reserved1 = (uint32_t)boost::algorithm::clamp<double>(d_value, 0, 0xFFFFFFFF); // Reserved. uint32, size:= UInt32 = 4 byte
  ROS::param<int>(nh, param_section+"/telegram_payload/Reserved2", telegram.telegram_payload.reserved2, 0); // Reserved. int32, size:= Int32 = 4 byte
  ROS::param<int>(nh, param_section+"/telegram_payload/Quality", i_value, 0);
  telegram.telegram_payload.quality = (uint8_t)i_value; // Quality of pose [0 … 100], 1 = bad pose quality, 100 = good pose quality. uint8, size:= UInt8 = 1 byte
  ROS::param<int>(nh, param_section+"/telegram_payload/OutliersRatio", i_value, 0);
  telegram.telegram_payload.outliersratio = (uint8_t)i_value; // Ratio of beams that cannot be assigned to the current reference map [%]. uint8, size:= UInt8 = 1 byte
  ROS::param<int>(nh, param_section+"/telegram_payload/CovarianceX", telegram.telegram_payload.covariancex, 0); // Covariance c1 of the pose X [mm^2]. int32, size:= Int32 = 4 byte
  ROS::param<int>(nh, param_section+"/telegram_payload/CovarianceY", telegram.telegram_payload.covariancey, 0); // Covariance c5 of the pose X [mm^2]. int32, size:= Int32 = 4 byte
  ROS::param<int>(nh, param_section+"/telegram_payload/CovarianceYaw", telegram.telegram_payload.covarianceyaw, 0); // Covariance c9 of the pose Yaw [mdeg^2]. int32, size:= Int32 = 4 byte
  ROS::param<double>(nh, param_section+"/telegram_payload/Reserved3", d_value, 0);
  telegram.telegram_payload.reserved3 = ((d_value <= 0)?0:((d_value >= 0xFFFFFFFFFFFFFFFF)?0xFFFFFFFFFFFFFFFF:(uint64_t)(d_value))); // Reserved. uint64, size:= UInt64 = 8 byte
  
  // Read telegram trailer, 2 byte
  ROS::param<int>(nh, param_section+"/telegram_trailer/Checksum", i_value, 0);
  telegram.telegram_trailer.checksum = (uint16_t)i_value; // CRC16-CCITT checksum
  
  return telegram;
}

/** Shortcut to check a value against min and max values, returns (telegram.key >= mintelegram.key && telegram.key <= maxtelegram.key) */
#define CHECK_TELEGRAM_VALUE(key,telegram,mintelegram,maxtelegram) ((telegram.key) >= (mintelegram.key) && (telegram.key <= maxtelegram.key))

/** Shortcut to check an array of values against min and max values, returns (telegram.key >= mintelegram.key && telegram.key <= maxtelegram.key) for each element in telegram.key */
#define CHECK_TELEGRAM_ARRAY(key,telegram,mintelegram,maxtelegram) (checkTelegramArray((telegram.key), (mintelegram.key), (maxtelegram.key)))

/** Shortcut to check an array of values against min and max values, returns (telegram.key >= mintelegram.key && telegram.key <= maxtelegram.key) for each element */
template<typename T> bool checkTelegramArray(const T & value, const T & min_value, const T & max_value)
{
  if(value.size() != min_value.size() || value.size() != max_value.size())
    return false;
  for(size_t n = 0; n < value.size(); n++)
  {
    if(!(value[n] >= min_value[n] && value[n] <= max_value[n]))
      return false;
  }
  return true;
}

/*
 * Checks a result telegram messages (SickLocResultPortTelegramMsg) against min and max values.
 * Returns true, if test passed (all values within their ranges), or false otherwise.
 * @param[in] telegram result telegram message (SickLocResultPortTelegramMsg)
 * @return true, if test passed, false otherwise.
 */
bool sick_lidar_localization::MessageCheckThread::checkTelegram(sick_lidar_localization::SickLocResultPortTelegramMsg & telegram)
{
  //telegram.header.stamp.sec
  return
    // Check ros header
    // CHECK_TELEGRAM_VALUE(header.seq, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) && // telegram.header.seq does not exist anymore in ROS2
    // CHECK_TELEGRAM_VALUE(header.stamp.sec, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) &&
    CHECK_TELEGRAM_ARRAY(header.frame_id, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) &&
    // Check telegram header, 52 byte
    CHECK_TELEGRAM_VALUE(telegram_header.magicword, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) &&
    CHECK_TELEGRAM_VALUE(telegram_header.length, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) &&
    CHECK_TELEGRAM_VALUE(telegram_header.payloadtype, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) &&
    CHECK_TELEGRAM_VALUE(telegram_header.payloadversion, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) &&
    CHECK_TELEGRAM_VALUE(telegram_header.ordernumber, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) &&
    CHECK_TELEGRAM_VALUE(telegram_header.serialnumber, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) &&
    CHECK_TELEGRAM_ARRAY(telegram_header.fw_version, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) &&
    CHECK_TELEGRAM_VALUE(telegram_header.telegramcounter, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) &&
    CHECK_TELEGRAM_VALUE(telegram_header.systemtime, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) &&
    // Check telegram payload, 52 byte
    CHECK_TELEGRAM_VALUE(telegram_payload.errorcode, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) &&
    CHECK_TELEGRAM_VALUE(telegram_payload.scancounter, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) &&
    CHECK_TELEGRAM_VALUE(telegram_payload.timestamp, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) &&
    CHECK_TELEGRAM_VALUE(telegram_payload.posex, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) &&
    CHECK_TELEGRAM_VALUE(telegram_payload.posey, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) &&
    CHECK_TELEGRAM_VALUE(telegram_payload.poseyaw, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) &&
    CHECK_TELEGRAM_VALUE(telegram_payload.reserved1, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) &&
    CHECK_TELEGRAM_VALUE(telegram_payload.reserved2, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) &&
    CHECK_TELEGRAM_VALUE(telegram_payload.quality, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) &&
    CHECK_TELEGRAM_VALUE(telegram_payload.outliersratio, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) &&
    CHECK_TELEGRAM_VALUE(telegram_payload.covariancex, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) &&
    CHECK_TELEGRAM_VALUE(telegram_payload.covariancey, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) &&
    CHECK_TELEGRAM_VALUE(telegram_payload.covarianceyaw, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) &&
    CHECK_TELEGRAM_VALUE(telegram_payload.reserved3, telegram, m_result_port_telegram_min_values, m_result_port_telegram_max_values) &&
      (!m_vehicle_time_check_enabled || checkVehicleTime(telegram)); // Check vehicle time (system time from ticks by software pll), if enabled by default
}

/*
 * Checks the vehicle time of a result telegram message (system time from ticks by software pll) against min and max
 * allowed difference to ros::Time::now(). Returns true, if test passed (vehicle time  within their range), or false otherwise.
 * @param[in] telegram result telegram message (SickLocResultPortTelegramMsg)
 * @return true, if test passed, false otherwise.
 */

bool sick_lidar_localization::MessageCheckThread::checkVehicleTime(sick_lidar_localization::SickLocResultPortTelegramMsg & telegram)
{
  if(ROS::secondsSinceStart(m_timestamp_valid_telegram) < FLT_EPSILON)
    m_timestamp_valid_telegram = ROS::now();
  if(telegram.vehicle_time_valid)
  {
    m_timestamp_valid_telegram = ROS::now();
    ROS::Time message_time = ROS::timeFromHeader(&telegram.header);
    //ROS::Time message_time(telegram.header.stamp.sec, telegram.header.stamp.nsec);
    ROS::Time vehicle_time(telegram.vehicle_time_sec,telegram.vehicle_time_nsec);
    ROS::Duration delta_time = message_time - vehicle_time;
    return ROS::seconds(delta_time) >= m_vehicle_time_delta_min && ROS::seconds(delta_time) <= m_vehicle_time_delta_max;
  }
  else if(ROS::seconds(ROS::now() - m_timestamp_valid_telegram) <= m_software_pll_expected_initialization_duration // software pll is initializing
  || (std::abs(m_vehicle_time_delta_min) >= FLT_MAX && std::abs(m_vehicle_time_delta_max) >= FLT_MAX)) // checkVehicleTime disabled for error simulation (unreachable controller etc.)
  {
    return true; // software pll initializing, system time from lidar ticks not yet available
  }
  return false; // vehicle time out of range
}

