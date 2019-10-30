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
#include <ros/ros.h>

#include "sick_lidar_localization/sim_loc_random.h"
#include "sick_lidar_localization/sim_loc_testcase_generator.h"
#include "sick_lidar_localization/sim_loc_utils.h"

/*
 * Creates and returns a deterministic default testcase for result port telegrams (binary telegrams and SickLocResultPortTelegramMsg)
 * @return SickLocResultPortTestcaseMsg with the binary telegram and SickLocResultPortTelegramMsg
 */
sick_lidar_localization::SickLocResultPortTestcaseMsg sick_lidar_localization::TestcaseGenerator::createDefaultResultPortTestcase(void)
{
  sick_lidar_localization::SickLocResultPortTestcaseMsg testcase;
  
  // ROS Header with sequence id, timestamp and frame id
  testcase.header.stamp = ros::Time::now();
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

/*
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
  static sick_lidar_localization::UniformRandomInteger random_quality_generator(1, 100);
  static sick_lidar_localization::UniformRandomInteger random_covariance_generator(0, INT32_MAX);
  
  // Create default SickLocResultPortTelegramMsg
  static ros::Time start_time = ros::Time::now();
  static sick_lidar_localization::SickLocResultPortTestcaseMsg default_testcase = createDefaultResultPortTestcase();
  sick_lidar_localization::SickLocResultPortTestcaseMsg testcase = default_testcase;
  sick_lidar_localization::SickLocResultPortTelegramMsg & telegram_msg = testcase.telegram_msg;
  
  // Modify SickLocResultPortTelegramMsg with random values
  telegram_msg.telegram_header.PayloadType = ((random1_generator.generate() > 0) ? 0x06c2 : 0x0642); // Payload type: 0x06c2 = Little Endian, 0x0642 = Big Endian. Size: UInt16 = 2 byte
  telegram_msg.telegram_header.OrderNumber = (uint32_t)random32_generator.generate();                // Order number of the localization controller. Size: UInt32 = 4 byte
  telegram_msg.telegram_header.SerialNumber = (uint32_t)random32_generator.generate();               // Serial number of the localization controller. Size: UInt32 = 4 byte
  for(size_t n = 0; n < telegram_msg.telegram_header.FW_Version.size(); n++)
    telegram_msg.telegram_header.FW_Version[n] = (uint8_t)random8_generator.generate();              // Software version of the localization controller. Size: 20 × UInt8 = 20 byte
  telegram_msg.telegram_payload.PoseX = random32_generator.generate();                               // Position X of the vehicle on the map in cartesian global coordinates [mm]. Size: Int32 = 4 byte
  telegram_msg.telegram_payload.PoseY = random32_generator.generate();                               // Position Y of the vehicle on the map in cartesian global coordinates [mm]. Size: Int32 = 4 byte
  telegram_msg.telegram_payload.PoseYaw = random_yaw_generator.generate();                           // Orientation (yaw) of the vehicle on the map [mdeg], range -180 to +180 deg assumed. Size: Int32 = 4 byte
  telegram_msg.telegram_payload.Reserved1 = (uint32_t)random32_generator.generate();                 // Reserved. Size: UInt32 = 4 byte
  telegram_msg.telegram_payload.Reserved2 = random32_generator.generate();                           // Reserved. Size: Int32 = 4 byte
  telegram_msg.telegram_payload.Quality = (uint8_t)random_quality_generator.generate();              // Quality of pose [1 … 100], 1 = bad pose quality, 100 = good pose quality. Size: UInt8 = 1 byte
  telegram_msg.telegram_payload.OutliersRatio = (uint8_t)random_quality_generator.generate();        // Ratio of beams that cannot be assigned to the current reference map [%]. Size: UInt8 = 1 byte
  telegram_msg.telegram_payload.CovarianceX = random_covariance_generator.generate();                // Covariance c1 of the pose X [mm^2]. Size: Int32 = 4 byte
  telegram_msg.telegram_payload.CovarianceY = random_covariance_generator.generate();                // Covariance c5 of the pose Y [mm^2]. Size: Int32 = 4 byte
  telegram_msg.telegram_payload.CovarianceYaw = random_covariance_generator.generate();              // Covariance c9 of the pose Yaw [mdeg^2]. Size: Int32 = 4 byte
  telegram_msg.telegram_payload.Reserved3 = (((uint64_t)random32_generator.generate() << 32) | (uint64_t)random32_generator.generate()); // Reserved. Size: UInt64 = 8 byte
  
  // Update telegram timestamps
  double delta_time_seconds = (ros::Time::now() - start_time).toSec();
  telegram_msg.telegram_payload.Timestamp += (uint32_t)(1000.0 * delta_time_seconds); // Time stamp of the pose [ms]. The time stamp indicates the time at which the pose is calculated. Size: UInt32 = 4 byte
  telegram_msg.telegram_header.SystemTime += (uint64_t)(delta_time_seconds);          // SystemTime not used. Size: NTP = 8 byte
  
  // Re-encode the modified result port telegram (SickLocResultPortTelegramMsg)
  sick_lidar_localization::ResultPortParser result_port_parser(testcase.header.frame_id);
  result_port_parser.getTelegramMsg() = telegram_msg;
  testcase.binary_data = result_port_parser.encode();
  testcase.telegram_msg = result_port_parser.getTelegramMsg();
  
  // Increment telegram counter for next testcase
  default_testcase.telegram_msg.telegram_header.TelegramCounter += 1; // Telegram counter since last start-up. Size: UInt32 = 4 byte
  default_testcase.telegram_msg.telegram_payload.ScanCounter += 1;    // Counter of related scan data. Size: UInt32 = 4 byte
  
  // Update testcase timestamp
  testcase.header.stamp = ros::Time::now();
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
  static ros::Time start_time = ros::Time::now();
  static sick_lidar_localization::SickLocResultPortTestcaseMsg default_testcase = createDefaultResultPortTestcase();
  sick_lidar_localization::SickLocResultPortTestcaseMsg testcase = default_testcase;
  sick_lidar_localization::SickLocResultPortTelegramMsg & telegram_msg = testcase.telegram_msg;

  // Set current position and orientation
  telegram_msg.telegram_payload.PoseX = (int32_t)(1000.0 * circle_radius * std::cos(circle_yaw));  // Position X of the vehicle on the map in cartesian global coordinates [mm]. Size: Int32 = 4 byte
  telegram_msg.telegram_payload.PoseY = (int32_t)(1000.0 * circle_radius * std::sin(circle_yaw));  // Position Y of the vehicle on the map in cartesian global coordinates [mm]. Size: Int32 = 4 byte
  double orientation = sick_lidar_localization::Utils::normalizeAngle(circle_yaw + M_PI_2);        // Orienation := circle_yaw + 90 degree
  telegram_msg.telegram_payload.PoseYaw = (int32_t)(1000.0 * orientation * 180.0 / M_PI);          // Orientation (yaw) of the vehicle on the map [mdeg], range -180 to +180 deg assumed. Size: Int32 = 4 byte
  
  // Update telegram timestamps
  double delta_time_seconds = (ros::Time::now() - start_time).toSec();
  telegram_msg.telegram_payload.Timestamp += (uint32_t)(1000.0 * delta_time_seconds); // Time stamp of the pose [ms]. The time stamp indicates the time at which the pose is calculated. Size: UInt32 = 4 byte
  telegram_msg.telegram_header.SystemTime += (uint64_t)(delta_time_seconds);          // SystemTime not used. Size: NTP = 8 byte
  
  // Re-encode the modified result port telegram (SickLocResultPortTelegramMsg)
  sick_lidar_localization::ResultPortParser result_port_parser(testcase.header.frame_id);
  result_port_parser.getTelegramMsg() = telegram_msg;
  testcase.binary_data = result_port_parser.encode();
  testcase.telegram_msg = result_port_parser.getTelegramMsg();
  
  // Increment telegram counter for next testcase
  default_testcase.telegram_msg.telegram_header.TelegramCounter += 1; // Telegram counter since last start-up. Size: UInt32 = 4 byte
  default_testcase.telegram_msg.telegram_payload.ScanCounter += 1;    // Counter of related scan data. Size: UInt32 = 4 byte
  
  // Update testcase timestamp
  testcase.header.stamp = ros::Time::now();
  return testcase;
}
