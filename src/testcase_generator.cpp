/*
 * @brief sim_loc_testcase_generator generates testcases for SIM Localization driver.
 * The generator creates deterministic and random based result port telegrams.
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
#include "sick_lidar_localization/random_generator.h"
#include "sick_lidar_localization/testcase_generator.h"
#include "sick_lidar_localization/utils.h"


/*!
 * result pose interval, i.e. the interval in number of scans (default: 1, i.e. result telegram with each processed scan)
 */
uint32_t sick_lidar_localization::TestcaseGenerator::s_u32ResultPoseInterval = 1;

/*!
 * test server int32 settings, set by sMN or sRN requests
 */
std::map<std::string, int32_t> sick_lidar_localization::TestcaseGenerator::s_controller_settings = {
  {"IsSystemReady", 1},        // 0:false, 1:true (default)
  {"LocState", 2},             // controller state: 0:BOOTING, 1:IDLE, 2:LOCALIZING, 3:DEMO_MAPPING
  {"LocResultPort", 2201},     // tcp port for result telegrams (default: 2201)
  {"LocResultMode", 0},        // 0:stream (default), 1:poll
  {"LocResultState", 1},       // result output: 0: disabled, 1: enabled
  {"LocResultEndianness", 0},  // 0: big endian (default), 1: little endian
  {"LocMapState", 1},          // map state: 0:not active, 1:active
  {"LocRequestResultData", 1}  // in poll mode, trigger sending the localization result of the next processed scan via TCP interface.
};

/*!
 * test server string settings, set by sMN or sRN requests
 */
std::map<std::string, std::string> sick_lidar_localization::TestcaseGenerator::s_controller_settings_str;

/*!
 * Returns true, if localization is active (default), otherwise false (localization deactivated)
 * @return result telegrams are activated (true) or deactivated
 */
bool sick_lidar_localization::TestcaseGenerator::LocalizationEnabled(void)
{
  return s_controller_settings["LocState"] == 2; // localization on
}

/*!
 * Returns true, if result telegrams are activated (i.e. localization on and result telegrams active), otherwise false (result telegrams deactivated)
 * @return result telegrams are activated (true, default) or deactivated
 */
bool sick_lidar_localization::TestcaseGenerator::ResultTelegramsEnabled(void)
{
  return LocalizationEnabled() && s_controller_settings["LocResultState"] > 0; // localization on and result telegrams activated, otherwise result telegrams deactivated
}

/*!
 * Creates and returns a deterministic default testcase for result port telegrams (binary telegrams and SickLocResultPortTelegramMsg)
 * @return SickLocResultPortTestcaseMsg with the binary telegram and SickLocResultPortTelegramMsg
 */
sick_lidar_localization::SickLocResultPortTestcaseMsg sick_lidar_localization::TestcaseGenerator::createDefaultResultPortTestcase(void)
{
  sick_lidar_localization::SickLocResultPortTestcaseMsg testcase;
  
  // ROS Header with sequence id, timestamp and frame id
  testcase.header.stamp = ROS::now();
  testcase.header.frame_id = "sick_localization_testcase";

  // binary encoded result port telegram (default example)
  testcase.binary_data = {
    0x53, 0x49, 0x43, 0x4B, 0x00, 0x00, 0x00, 0x6A, 0x06, 0x42, 0x00, 0x01, 0x00, 0x10, 0xC0, 0x58, 0x01, 0x22, 0xA2, 0x72,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x4C, 0x53, 0x20, 0x56, 0x30, 0x2E, 0x31, 0x2E, 0x39, 0x2E, 0x78, 0x42,
    0x00, 0x00, 0x02, 0x6D, 0x83, 0xAA, 0x8C, 0x0C, 0x8E, 0x14, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x6F, 0x00, 0x34,
    0xEC, 0xF3, 0x00, 0x00, 0x00, 0x5D, 0x00, 0x00, 0x00, 0x21, 0x00, 0x00, 0x45, 0xE7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x37, 0x00, 0x00, 0x00, 0x80, 0x89, 0x00, 0x00, 0x99, 0x93, 0x00, 0x12, 0x78, 0x9F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x62, 0x11 };

  // decoded result port telegram
  sick_lidar_localization::ResultPortParser result_port_parser(testcase.header.frame_id);
  std::vector<uint8_t> recoded_telegram;
  if(!result_port_parser.decode(testcase.binary_data) || (recoded_telegram = result_port_parser.encode()) != testcase.binary_data)
  {
    ROS_ERROR_STREAM("## ERROR TestcaseGenerator::createDefaultResultPortTestcase: sick_lidar_localization::ResultPortParser::decode() failed. " << testcase.binary_data.size() << " byte input (hex):");
    ROS_ERROR_STREAM(sick_lidar_localization::Utils::toHexString(testcase.binary_data));
    ROS_ERROR_STREAM("## output (decoded): " << sick_lidar_localization::Utils::flattenToString(result_port_parser.getTelegramMsg()));
    ROS_ERROR_STREAM("## recoded:");
    ROS_ERROR_STREAM(sick_lidar_localization::Utils::toHexString(recoded_telegram));
  }
  testcase.telegram_msg = result_port_parser.getTelegramMsg();

  return testcase;
}

/*!
 * Creates and returns a random testcase for result port telegrams (binary telegrams and SickLocResultPortTelegramMsg)
 * @return SickLocResultPortTestcaseMsg with the binary telegram and SickLocResultPortTelegramMsg
 */
sick_lidar_localization::SickLocResultPortTestcaseMsg sick_lidar_localization::TestcaseGenerator::createRandomResultPortTestcase(void)
{
  // Random number generators
  static sick_lidar_localization::UniformRandomInteger random1_generator(0, 1);
  static sick_lidar_localization::UniformRandomInteger random8_generator(0, 255);
  static sick_lidar_localization::UniformRandomInteger random32_generator(-INT32_MAX, INT32_MAX);
  static sick_lidar_localization::UniformRandomInteger random_yaw_generator(-180000, 180000);
  static sick_lidar_localization::UniformRandomInteger random_quality_generator(0, 100);
  static sick_lidar_localization::UniformRandomInteger random_covariance_generator(0, INT32_MAX);
  
  // Create default SickLocResultPortTelegramMsg
  static ROS::Time start_time = ROS::now();
  static sick_lidar_localization::SickLocResultPortTestcaseMsg default_testcase = createDefaultResultPortTestcase();
  sick_lidar_localization::SickLocResultPortTestcaseMsg testcase = default_testcase;
  sick_lidar_localization::SickLocResultPortTelegramMsg & telegram_msg = testcase.telegram_msg;
  
  // Modify SickLocResultPortTelegramMsg with random values
  telegram_msg.telegram_header.payloadtype = ((random1_generator.generate() > 0) ? 0x06c2 : 0x0642); // Payload type: 0x06c2 = Little Endian, 0x0642 = Big Endian. Size: UInt16 = 2 byte
  telegram_msg.telegram_header.ordernumber = (uint32_t)random32_generator.generate();                // Order number of the localization controller. Size: UInt32 = 4 byte
  telegram_msg.telegram_header.serialnumber = (uint32_t)random32_generator.generate();               // Serial number of the localization controller. Size: UInt32 = 4 byte
  for(size_t n = 0; n < telegram_msg.telegram_header.fw_version.size(); n++)
    telegram_msg.telegram_header.fw_version[n] = (uint8_t)random8_generator.generate();              // Software version of the localization controller. Size: 20 × UInt8 = 20 byte
  telegram_msg.telegram_payload.posex = random32_generator.generate();                               // Position X of the vehicle on the map in cartesian global coordinates [mm]. Size: Int32 = 4 byte
  telegram_msg.telegram_payload.posey = random32_generator.generate();                               // Position Y of the vehicle on the map in cartesian global coordinates [mm]. Size: Int32 = 4 byte
  telegram_msg.telegram_payload.poseyaw = random_yaw_generator.generate();                           // Orientation (yaw) of the vehicle on the map [mdeg], range -180 to +180 deg assumed. Size: Int32 = 4 byte
  telegram_msg.telegram_payload.reserved1 = (uint32_t)random32_generator.generate();                 // Reserved. Size: UInt32 = 4 byte
  telegram_msg.telegram_payload.reserved2 = random32_generator.generate();                           // Reserved. Size: Int32 = 4 byte
  telegram_msg.telegram_payload.quality = (uint8_t)random_quality_generator.generate();              // Quality of pose [0 … 100], 1 = bad pose quality, 100 = good pose quality. Size: UInt8 = 1 byte
  telegram_msg.telegram_payload.outliersratio = (uint8_t)random_quality_generator.generate();        // Ratio of beams that cannot be assigned to the current reference map [%]. Size: UInt8 = 1 byte
  telegram_msg.telegram_payload.covariancex = random_covariance_generator.generate();                // Covariance c1 of the pose X [mm^2]. Size: Int32 = 4 byte
  telegram_msg.telegram_payload.covariancey = random_covariance_generator.generate();                // Covariance c5 of the pose Y [mm^2]. Size: Int32 = 4 byte
  telegram_msg.telegram_payload.covarianceyaw = random_covariance_generator.generate();              // Covariance c9 of the pose Yaw [mdeg^2]. Size: Int32 = 4 byte
  telegram_msg.telegram_payload.reserved3 = (((uint64_t)random32_generator.generate() << 32) | (uint64_t)random32_generator.generate()); // Reserved. Size: UInt64 = 8 byte
  
  // Update telegram timestamps
  double delta_time_seconds = ROS::seconds(ROS::now() - start_time);
  telegram_msg.telegram_payload.timestamp = createTimestampTicksMilliSec();    // Time stamp of the pose [ms]. The time stamp indicates the time at which the pose is calculated. Size: UInt32 = 4 byte
  telegram_msg.telegram_header.systemtime += (uint64_t)(delta_time_seconds);   // SystemTime not used. Size: NTP = 8 byte
  
  // Re-encode the modified result port telegram (SickLocResultPortTelegramMsg)
  sick_lidar_localization::ResultPortParser result_port_parser(testcase.header.frame_id);
  result_port_parser.getTelegramMsg() = telegram_msg;
  testcase.binary_data = result_port_parser.encode();
  testcase.telegram_msg = result_port_parser.getTelegramMsg();
  
  // Increment telegram counter for next testcase
  default_testcase.telegram_msg.telegram_header.telegramcounter += 1; // Telegram counter since last start-up. Size: UInt32 = 4 byte
  default_testcase.telegram_msg.telegram_payload.scancounter += 1;    // Counter of related scan data. Size: UInt32 = 4 byte
  
  // Update testcase timestamp
  testcase.header.stamp = ROS::now();
  return testcase;
}

/*!
 * Creates and returns a result port telegrams (binary telegrams and SickLocResultPortTelegramMsg)
 * simulating sensor moving in circles with a current position given by radius in meter and yaw angle
 * in radians
 * @param[in] circle_radius radius of circle in meter
 * @param[in] circle_yaw current angle in radians
 * @return SickLocResultPortTestcaseMsg with the binary telegram and SickLocResultPortTelegramMsg
 */
sick_lidar_localization::SickLocResultPortTestcaseMsg sick_lidar_localization::TestcaseGenerator::createResultPortCircles(double circle_radius, double circle_yaw)
{
  // Create default SickLocResultPortTelegramMsg
  static ROS::Time start_time = ROS::now();
  static sick_lidar_localization::SickLocResultPortTestcaseMsg default_testcase = createDefaultResultPortTestcase();
  sick_lidar_localization::SickLocResultPortTestcaseMsg testcase = default_testcase;
  sick_lidar_localization::SickLocResultPortTelegramMsg & telegram_msg = testcase.telegram_msg;

  // Set current position and orientation
  telegram_msg.telegram_payload.posex = (int32_t)(1000.0 * circle_radius * std::cos(circle_yaw));  // Position X of the vehicle on the map in cartesian global coordinates [mm]. Size: Int32 = 4 byte
  telegram_msg.telegram_payload.posey = (int32_t)(1000.0 * circle_radius * std::sin(circle_yaw));  // Position Y of the vehicle on the map in cartesian global coordinates [mm]. Size: Int32 = 4 byte
  double orientation = sick_lidar_localization::Utils::normalizeAngle(circle_yaw + M_PI_2);        // Orienation := circle_yaw + 90 degree
  telegram_msg.telegram_payload.poseyaw = (int32_t)(1000.0 * orientation * 180.0 / M_PI);          // Orientation (yaw) of the vehicle on the map [mdeg], range -180 to +180 deg assumed. Size: Int32 = 4 byte
  
  // Update telegram timestamps
  double delta_time_seconds = ROS::seconds(ROS::now() - start_time);
  telegram_msg.telegram_payload.timestamp = createTimestampTicksMilliSec();             // Time stamp of the pose [ms]. The time stamp indicates the time at which the pose is calculated. Size: UInt32 = 4 byte
  telegram_msg.telegram_header.systemtime += (uint64_t)(delta_time_seconds);            // SystemTime not used. Size: NTP = 8 byte
  
  // Re-encode the modified result port telegram (SickLocResultPortTelegramMsg)
  sick_lidar_localization::ResultPortParser result_port_parser(testcase.header.frame_id);
  result_port_parser.getTelegramMsg() = telegram_msg;
  testcase.binary_data = result_port_parser.encode();
  testcase.telegram_msg = result_port_parser.getTelegramMsg();
  
  // Increment telegram counter for next testcase
  default_testcase.telegram_msg.telegram_header.telegramcounter += 1; // Telegram counter since last start-up. Size: UInt32 = 4 byte
  default_testcase.telegram_msg.telegram_payload.scancounter += 1;    // Counter of related scan data. Size: UInt32 = 4 byte
  
  // Update testcase timestamp
  testcase.header.stamp = ROS::now();
  return testcase;
}

/*!
 * Creates and returns a synthetical cola response to a cola command request.
 * Note: Just a few cola responses are implemented for test purposes, f.e. responses to "LocRequestTimestamp".
 * By default, a response: "sAN <command_name>" without any parameter is returned (sAN: Response to sMN)
 * @param[in] cola_request Cola request from client
 * @return Synthetical cola response from server
 */
sick_lidar_localization::SickLocColaTelegramMsg sick_lidar_localization::TestcaseGenerator::createColaResponse(const sick_lidar_localization::SickLocColaTelegramMsg & cola_request)
{
  // Generate a synthetical response to LocRequestTimestamp requests: "sAN LocRequestTimestamp <timestamp>" with uint32_t timestamp in hex and ms
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "LocRequestTimestamp")
  {
    static sick_lidar_localization::UniformRandomInteger time_jitter_network_ms(0, 2);
    // Simulate some network latency
    ROS::sleep(0.001 * time_jitter_network_ms.generate());
    // Create current timestamp in ticks
    uint32_t ticks_ms = createTimestampTicksMilliSec();
    // Simulate some network latency
    ROS::sleep(0.001 * time_jitter_network_ms.generate());
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {hexstr(ticks_ms)});
  }
  
  // Set settings from Configuration Telegrams
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "LocStartLocalizing")
  {
    s_controller_settings["LocState"] = 2;
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "LocStop")
  {
    s_controller_settings["LocState"] = 1;
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "LocStopAndSave")
  {
    s_controller_settings["LocState"] = 1;
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "LocSetResultPort" && cola_request.parameter.size() == 1)
  {
    s_controller_settings["LocResultPort"] = std::strtol(cola_request.parameter[0].c_str(), 0, 0);
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1), decstr(1)});
  }
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "LocSetResultMode" && cola_request.parameter.size() == 1)
  {
    s_controller_settings["LocResultMode"] = std::strtol(cola_request.parameter[0].c_str(), 0, 0);
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "LocSetResultPoseEnabled" && cola_request.parameter.size() == 1)
  {
    s_controller_settings["LocResultState"] = std::strtol(cola_request.parameter[0].c_str(), 0, 0);
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "LocSetResultEndianness" && cola_request.parameter.size() == 1)
  {
    s_controller_settings["LocResultEndianness"] = std::strtol(cola_request.parameter[0].c_str(), 0, 0);;
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "LocSetPose" && cola_request.parameter.size() == 4)
  {
    int32_t posex_mm = std::strtol(cola_request.parameter[0].c_str(), 0, 0);
    int32_t posey_mm = std::strtol(cola_request.parameter[1].c_str(), 0, 0);
    int32_t yaw_mdeg = std::strtol(cola_request.parameter[2].c_str(), 0, 0);
    int32_t uncertainty = std::strtol(cola_request.parameter[3].c_str(), 0, 0);
    bool success = (posex_mm >= -300000 && posex_mm <= +300000 && posey_mm >= -300000 && posey_mm <= +300000
      && yaw_mdeg >= -180000 && yaw_mdeg <= +180000 && uncertainty >= 0 && uncertainty < 300000);
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(success?1:0)});
  }

  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "LocSetResultPoseInterval" && cola_request.parameter.size() == 1)
  {
    s_u32ResultPoseInterval = std::strtoul(cola_request.parameter[0].c_str(), 0, 0);
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }

  /* Start of test server responses for new service requests (release 4 or later) */

  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "DevSetLidarConfig" && cola_request.parameter.size() == 15)
  {
    for(size_t n = 0; n < cola_request.parameter.size(); n++)
      s_controller_settings_str["DevSetLidarConfig_"+std::to_string(n)] = cola_request.parameter[n];
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1), decstr(1)});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "DevGetLidarConfig" && cola_request.parameter.size() == 1)
  {
    std::vector<std::string> config_parameter;
    for(size_t n = 1; n < 15; n++)
      // if(n != 9)
        config_parameter.push_back(s_controller_settings_str["DevSetLidarConfig_"+std::to_string(n)]);
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, config_parameter);
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "LocSetMap" && cola_request.parameter.size() == 2)
  {
    s_controller_settings_str["LocSetMap"] = cola_request.parameter[1];
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1), decstr(1)});
  }
  
  if(cola_request.command_name == "LocMap")//if(cola_request.command_type == sick_lidar_localization::ColaParser::sRN && cola_request.command_name == "LocMap" && cola_request.parameter.size() == 0)
  {
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sRA, cola_request.command_name, {s_controller_settings_str["LocSetMap"]});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sRN && cola_request.command_name == "LocMapState" && cola_request.parameter.size() == 0)
  {
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "LocInitializePose" && cola_request.parameter.size() == 4)
  {
    for(size_t n = 0; n < cola_request.parameter.size(); n++)
      s_controller_settings_str["LocInitializePose_"+std::to_string(n)] = cola_request.parameter[n];
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sRN && cola_request.command_name == "LocInitialPose" && cola_request.parameter.size() == 0)
  {
    std::vector<std::string> parameter;
    for(size_t n = 0; n < 4; n++)
      parameter.push_back(s_controller_settings_str["LocInitializePose_"+std::to_string(n)]);
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, parameter);
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "LocSetPoseQualityCovWeight" && cola_request.parameter.size() == 1)
  {
    s_controller_settings_str["LocSetPoseQualityCovWeight"] = cola_request.parameter[0];
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sRN && cola_request.command_name == "LocPoseQualityCovWeight" && cola_request.parameter.size() == 0)
  {
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {s_controller_settings_str["LocSetPoseQualityCovWeight"]});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "LocSetPoseQualityMeanDistWeight" && cola_request.parameter.size() == 1)
  {
    s_controller_settings_str["LocSetPoseQualityMeanDistWeight"] = cola_request.parameter[0];
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sRN && cola_request.command_name == "LocPoseQualityMeanDistWeight" && cola_request.parameter.size() == 0)
  {
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {s_controller_settings_str["LocSetPoseQualityMeanDistWeight"]});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "LocSetReflectorsForSupportActive" && cola_request.parameter.size() == 1)
  {
    s_controller_settings_str["LocSetReflectorsForSupportActive"] = cola_request.parameter[0];
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sRN && cola_request.command_name == "LocReflectorsForSupportActive" && cola_request.parameter.size() == 0)
  {
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {s_controller_settings_str["LocSetReflectorsForSupportActive"]});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "LocSetOdometryActive" && cola_request.parameter.size() == 1)
  {
    s_controller_settings_str["LocSetOdometryActive"] = cola_request.parameter[0];
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1), decstr(1)});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sRN && cola_request.command_name == "LocOdometryActive" && cola_request.parameter.size() == 0)
  {
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {s_controller_settings_str["LocSetOdometryActive"]});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "LocSetOdometryPort" && cola_request.parameter.size() == 1)
  {
    s_controller_settings_str["LocSetOdometryPort"] = cola_request.parameter[0];
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1), decstr(1)});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sRN && cola_request.command_name == "LocOdometryPort" && cola_request.parameter.size() == 0)
  {
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {s_controller_settings_str["LocSetOdometryPort"]});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "LocSetOdometryRestrictYMotion" && cola_request.parameter.size() == 1)
  {
    s_controller_settings_str["LocSetOdometryRestrictYMotion"] = cola_request.parameter[0];
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sRN && cola_request.command_name == "LocOdometryRestrictYMotion" && cola_request.parameter.size() == 0)
  {
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {s_controller_settings_str["LocSetOdometryRestrictYMotion"]});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "LocSetAutoStartActive" && cola_request.parameter.size() == 1)
  {
    s_controller_settings_str["LocSetAutoStartActive"] = cola_request.parameter[0];
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sRN && cola_request.command_name == "LocAutoStartActive" && cola_request.parameter.size() == 0)
  {
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {s_controller_settings_str["LocSetAutoStartActive"]});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "LocSetAutoStartSavePoseInterval" && cola_request.parameter.size() == 1)
  {
    s_controller_settings_str["LocSetAutoStartSavePoseInterval"] = cola_request.parameter[0];
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sRN && cola_request.command_name == "LocAutoStartSavePoseInterval" && cola_request.parameter.size() == 0)
  {
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {s_controller_settings_str["LocSetAutoStartSavePoseInterval"]});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "LocSetRingBufferRecordingActive" && cola_request.parameter.size() == 1)
  {
    s_controller_settings_str["LocSetRingBufferRecordingActive"] = cola_request.parameter[0];
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sRN && cola_request.command_name == "LocRingBufferRecordingActive" && cola_request.parameter.size() == 0)
  {
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {s_controller_settings_str["LocSetRingBufferRecordingActive"]});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "DevGetLidarIdent" && cola_request.parameter.size() == 1)
  {
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {"TestcaseGenerator" + cola_request.parameter[0]});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "DevGetLidarState" && cola_request.parameter.size() == 1)
  {
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(2), decstr(2), decstr(2)});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "GetSoftwareVersion" && cola_request.parameter.size() == 0)
  {
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {"1.0"});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "LocAutoStartSavePose" && cola_request.parameter.size() == 0)
  {
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "LocForceUpdate" && cola_request.parameter.size() == 0)
  {
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "LocSaveRingBufferRecording" && cola_request.parameter.size() == 2)
  {
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(2)});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "LocStartDemoMapping" && cola_request.parameter.size() == 0)
  {
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "ReportUserMessage" && cola_request.parameter.size() == 2)
  {
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "SavePermanent" && cola_request.parameter.size() == 0)
  {
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sRN && cola_request.command_name == "LocResultPort" && cola_request.parameter.size() == 0)
  {
    int32_t port = ((s_controller_settings["LocResultPort"]) > 0 ? s_controller_settings["LocResultPort"] : 2201);
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sRA, cola_request.command_name, {hexstr(port)});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sRN && cola_request.command_name == "LocResultMode" && cola_request.parameter.size() == 0)
  {
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sRA, cola_request.command_name, {decstr(s_controller_settings["LocResultMode"])});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sRN && cola_request.command_name == "LocResultEndianness" && cola_request.parameter.size() == 0)
  {
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sRA, cola_request.command_name, {decstr(s_controller_settings["LocResultEndianness"])});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sRN && cola_request.command_name == "LocResultState" && cola_request.parameter.size() == 0)
  {
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sRA, cola_request.command_name, {decstr(s_controller_settings["LocResultState"])});
  }
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sRN && cola_request.command_name == "LocResultPoseInterval" && cola_request.parameter.size() == 0)
  {
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sRA, cola_request.command_name, {decstr(s_u32ResultPoseInterval)});
  }  
  
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN && cola_request.command_name == "DevSetIMUActive" && cola_request.parameter.size() == 1)
  {
    s_controller_settings_str["DevSetIMUActive"] = cola_request.parameter[0];
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name, {decstr(1)});
  }  

  if(cola_request.command_type == sick_lidar_localization::ColaParser::sRN && cola_request.command_name == "DevIMUActive" && cola_request.parameter.size() == 0)
  {
    return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sRA, cola_request.command_name, {s_controller_settings_str["DevSetIMUActive"]});
  }  

  /* End of test server responses for new service requests (release 4 or later) */

  // Create sAN responses to sMN requests resp. sRA responses to sRN requests
  if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN || cola_request.command_type == sick_lidar_localization::ColaParser::sRN)
  {
    sick_lidar_localization::ColaParser::COLA_SOPAS_COMMAND response_type = sick_lidar_localization::ColaParser::sINVALID;
    if(cola_request.command_type == sick_lidar_localization::ColaParser::sMN)
      response_type = sick_lidar_localization::ColaParser::sAN; // sAN responses to sMN requests
    else if(cola_request.command_type == sick_lidar_localization::ColaParser::sRN)
      response_type = sick_lidar_localization::ColaParser::sRA; // sRA responses to sRN requests
    for(std::map<std::string, int32_t>::iterator iter_settings = s_controller_settings.begin(); iter_settings != s_controller_settings.end(); iter_settings++)
    {
      if(cola_request.command_name == iter_settings->first)
      {
        return sick_lidar_localization::ColaParser::createColaTelegram(response_type, cola_request.command_name, {hexstr(iter_settings->second)});
      }
    }
  }
  
  // Default response: "sAN <command_name>" without parameter (sAN: Response to sMN)
  return sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, cola_request.command_name);
}

/*!
 * Creates and returns a timestamp in milliseconds ticks.
 * To simulate time jitter, network latency and time drift,
 * a random value of +/- 2 milliseconds is added.
 * @return timestamp in milliseconds ticks
 */
uint32_t sick_lidar_localization::TestcaseGenerator::createTimestampTicksMilliSec(void)
{
  static ROS::Time start = ROS::now();
  static sick_lidar_localization::UniformRandomInteger time_jitter_ticks_ms(-2, +2);
  // Create current timestamp in ticks
  ROS::Duration timestamp = (ROS::now() - start);
  uint32_t seconds = 0, nanoseconds = 0;
  ROS::splitTime(timestamp, seconds, nanoseconds);
  uint32_t ticks_ms = (((uint64_t)seconds * 1000 + (uint64_t)nanoseconds/1000000 + 1000) & 0xFFFFFFFF);
  // Create some jitter, simulation network latency and time drift
  ticks_ms += time_jitter_ticks_ms.generate();
  return ticks_ms;
}