/*
 * @brief unittest_sim_loc_parser implements a unittest for parsing CoLa
 * and result port telegrams for SIM Localization.
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
#include <string>
#include <vector>

#include "sick_lidar_localization/cola_encoder.h"
#include "sick_lidar_localization/cola_parser.h"
#include "sick_lidar_localization/random_generator.h"
#include "sick_lidar_localization/testcase_generator.h"
#include "sick_lidar_localization/utils.h"

/*!
 * Container for testcase data to convert between Cola-ASCII and Cola-Binary,
 * containing the reference data for both  Cola-ASCII and Cola-Binary telegrams.
 */
class ColaBinaryTestcase
{
public:
  ColaBinaryTestcase(const std::string & _description = std::string(), const std::vector<uint8_t> & _cola_ascii = std::vector<uint8_t>(), const std::vector<uint8_t> & _cola_binary = std::vector<uint8_t>(), bool _parameter_to_ascii = true)
  : description(_description), cola_ascii(_cola_ascii), cola_binary(_cola_binary), parameter_to_ascii(_parameter_to_ascii) {} ///< Constructor with default values
  std::string description;          ///< Descripition, f.e. "<STX>sMN SetAccessMode 3 F4724744<ETX>"
  std::vector<uint8_t> cola_ascii;  ///< Cola-ASCII reference, f.e. { 0x02, 0x73, 0x4D, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x41, 0x63, 0x63,  0x65, 0x73, 0x73, 0x4D, 0x6F, 0x64, 0x65, 0x20, 0x33, 0x20, 0x46, 0x34, 0x37, 0x32, 0x34, 0x37, 0x34, 0x34, 0x03 }
  std::vector<uint8_t> cola_binary; ///< Cola-Binaryreference, f.e. { 0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x17, 0x73, 0x4D, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x41, 0x63, 0x63, 0x65, 0x73, 0x73, 0x4D, 0x6F, 0x64, 0x65, 0x20, 0x03, 0xF4, 0x72, 0x47, 0x44, 0xB3 }
  bool parameter_to_ascii;          ///< Conversion of command parameter to ascii (f.e. if true, a parameter 0x01 will be converted to 0x31 (default), otherwise to 0x01)
};
std::vector<uint8_t> cola_ascii_reference = // "<STX>sMN SetAccessMode 3 F4724744<ETX>"
  { 0x02, 0x73, 0x4D, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x41, 0x63, 0x63,  0x65, 0x73, 0x73, 0x4D, 0x6F, 0x64, 0x65, 0x20, 0x33, 0x20, 0x46, 0x34, 0x37, 0x32, 0x34, 0x37, 0x34, 0x34, 0x03 };
std::vector<uint8_t> cola_binary_reference = // Example from Technical_information_Telegram_Listing_Ranging_sensors_....pdf
  { 0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x17, 0x73, 0x4D, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x41, 0x63, 0x63, 0x65, 0x73, 0x73, 0x4D, 0x6F, 0x64, 0x65, 0x20, 0x03, 0xF4, 0x72, 0x47, 0x44, 0xB3 };

// Run sim_loc_parser unittest for encoding new services to Cola Ascii telegrams. Unittest for new services in release 4 and later
// Testcases taken from examples listed in manual 8024818_TI_LiDAR_Localization_Telegram_Listing_en_9317743_2020-09-11.pdf
static void RunColaEncoderUnitTests(int & testcase_cnt, int & failed_testcase_cnt)
{
  sick_lidar_localization::SickDevSetLidarConfigSrv::Request service_request_01;
  sick_lidar_localization::SickLocSetMapSrv::Request service_request_03;
  sick_lidar_localization::SickLocMapSrv::Request service_request_04;
  sick_lidar_localization::SickLocMapStateSrv::Request service_request_05;
  sick_lidar_localization::SickLocInitializePoseSrv::Request service_request_06;
  sick_lidar_localization::SickLocSetReflectorsForSupportActiveSrv::Request service_request_12;
  sick_lidar_localization::SickLocSetOdometryActiveSrv::Request service_request_14;
  sick_lidar_localization::SickLocSetOdometryPortSrv::Request service_request_16;
  sick_lidar_localization::SickLocSetOdometryRestrictYMotionSrv::Request service_request_18;
  sick_lidar_localization::SickLocSetAutoStartActiveSrv::Request service_request_20;
  sick_lidar_localization::SickLocSetAutoStartSavePoseIntervalSrv::Request service_request_22;
  sick_lidar_localization::SickLocSetRingBufferRecordingActiveSrv::Request service_request_24;
  sick_lidar_localization::SickDevGetLidarStateSrv::Request service_request_27;
  sick_lidar_localization::SickGetSoftwareVersionSrv::Request service_request_28;
  sick_lidar_localization::SickLocAutoStartSavePoseSrv::Request service_request_29;
  sick_lidar_localization::SickLocForceUpdateSrv::Request service_request_30;
  sick_lidar_localization::SickLocSaveRingBufferRecordingSrv::Request service_request_31;
  sick_lidar_localization::SickSavePermanentSrv::Request service_request_34;
  sick_lidar_localization::SickLocResultPortSrv::Request service_request_35;
  sick_lidar_localization::SickLocResultModeSrv::Request service_request_36;
  sick_lidar_localization::SickLocResultEndiannessSrv::Request service_request_37;
  sick_lidar_localization::SickLocResultStateSrv::Request service_request_38;
  sick_lidar_localization::SickDevSetIMUActiveSrv::Request service_request_40;
  std::string cola_ascii;

  service_request_01.index = 0; // Index of the lidar that shall be configured. [0, 1] 
  service_request_01.minrange = 300; // Beams with a range lower than this distance will be discarded. [<min>, 250000] in [mm] 
  service_request_01.maxrange = 100000; // Beams with a range greater than this distance will be discarded. [<min>, 250000] in [mm] 
  service_request_01.minangle = -180000; // Beams with an angle lower than this threshold will be discarded. [-180000, 180000] in [mdeg] 
  service_request_01.maxangle = 180000; // Beams with an angle greater than this threshold will be discarded. [-180000, 180000] in [mdeg] 
  service_request_01.x = 1000; // X position relative to vehicle coordinate system. [-50000, 50000] in [mm] 
  service_request_01.y = -300; // Y position relative to vehicle coordinate system. [-50000, 50000] in [mm] 
  service_request_01.yaw = -45000; // Yaw angle relative to vehicle coordinate system. [-180000, 180000] in [mdeg] 
  service_request_01.upsidedown = 0; // Whether the sensor is mounted upside down. {0 (false), 1 (true)} 
  service_request_01.ip = "192.168.0.30"; // IP address of the sensor. Must be in the same subnet as the port which it is connected to. Max length = 15 
  service_request_01.port = 2122; // COLA Port of the scanner used for communication. This is usually 2111 or 2122. [0, 65535] 
  service_request_01.interfacetype = 0; // The type of the interface to this lidar. {0 (TCP), 1 (SERIAL)} 
  service_request_01.maplayer = 0; // The index of the map layer on which this LiDAR operates. If no map layers are used, set it to 0. NOTE: This feature is not implemented yet but is reserved for later releases. [<min>, <max>] 
  service_request_01.active = 1; // Whether this scanner shall be used for localization. For the LiDAR with the index 0 this should be always 1 (true). {0 (false), 1 (true)} 
  if ((cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request_01)) != "sMN DevSetLidarConfig +0 +300 +100000 -180000 +180000 +1000 -300 -45000 0 +12 192.168.0.30 +2122 0 0 1")
  {
    failed_testcase_cnt++;
    ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest() failed, expected sMN DevSetLidarConfig +0 +300 +100000 -180000 +180000 +1000 -300 -45000 0 +12 192.168.0.30 +2122 0 0 1, returned \"" << cola_ascii << "\"");
  }
  else
  {
    ROS_INFO_STREAM("unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest testcase passed, returned \"" << cola_ascii << "\"");
  }
  testcase_cnt++;
  service_request_03.mapfilename = "our_hall.smap";
  if ((cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request_03)) != "sMN LocSetMap +13 our_hall.smap")
  {
    failed_testcase_cnt++;
    ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest() failed, expected sMN LocSetMap +13 our_hall.smap, returned \"" << cola_ascii << "\"");
  }
  else
  {
    ROS_INFO_STREAM("unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest testcase passed, returned \"" << cola_ascii << "\"");
  }
  testcase_cnt++;
  if ((cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request_04)) != "sRN LocMap")
  {
    failed_testcase_cnt++;
    ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest() failed, expected sRN LocMap, returned \"" << cola_ascii << "\"");
  }
  else
  {
    ROS_INFO_STREAM("unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest testcase passed, returned \"" << cola_ascii << "\"");
  }
  testcase_cnt++;
  if ((cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request_05)) != "sRN LocMapState")
  {
    failed_testcase_cnt++;
    ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest() failed, expected sRN LocMapState, returned \"" << cola_ascii << "\"");
  }
  else
  {
    ROS_INFO_STREAM("unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest testcase passed, returned \"" << cola_ascii << "\"");
  }
  testcase_cnt++;
  service_request_06.x = 10300;
  service_request_06.y = -5200;
  service_request_06.yaw = 30000;
  service_request_06.sigmatranslation = 1000;
  if ((cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request_06)) != "sMN LocInitializePose +10300 -5200 +30000 +1000")
  {
    failed_testcase_cnt++;
    ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest() failed, expected sMN LocInitializePose +10300 -5200 +30000 +1000, returned \"" << cola_ascii << "\"");
  }
  else
  {
    ROS_INFO_STREAM("unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest testcase passed, returned \"" << cola_ascii << "\"");
  }
  testcase_cnt++;
  service_request_12.active = true;
  if ((cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request_12)) != "sMN LocSetReflectorsForSupportActive 1")
  {
    failed_testcase_cnt++;
    ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest() failed, expected sMN LocSetReflectorsForSupportActive 1, returned \"" << cola_ascii << "\"");
  }
  else
  {
    ROS_INFO_STREAM("unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest testcase passed, returned \"" << cola_ascii << "\"");
  }
  testcase_cnt++;
  service_request_14.active = true;
  if ((cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request_14)) != "sMN LocSetOdometryActive 1")
  {
    failed_testcase_cnt++;
    ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest() failed, expected sMN LocSetOdometryActive 1, returned \"" << cola_ascii << "\"");
  }
  else
  {
    ROS_INFO_STREAM("unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest testcase passed, returned \"" << cola_ascii << "\"");
  }
  testcase_cnt++;
  service_request_16.port = 3000;
  if ((cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request_16)) != "sMN LocSetOdometryPort +3000")
  {
    failed_testcase_cnt++;
    ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest() failed, expected sMN LocSetOdometryPort +3000, returned \"" << cola_ascii << "\"");
  }
  else
  {
    ROS_INFO_STREAM("unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest testcase passed, returned \"" << cola_ascii << "\"");
  }
  testcase_cnt++;
  service_request_18.active = true;
  if ((cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request_18)) != "sMN LocSetOdometryRestrictYMotion 1")
  {
    failed_testcase_cnt++;
    ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest() failed, expected sMN LocSetOdometryRestrictYMotion 1, returned \"" << cola_ascii << "\"");
  }
  else
  {
    ROS_INFO_STREAM("unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest testcase passed, returned \"" << cola_ascii << "\"");
  }
  testcase_cnt++;
  service_request_20.active = true;
  if ((cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request_20)) != "sMN LocSetAutoStartActive 1")
  {
    failed_testcase_cnt++;
    ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest() failed, expected sMN LocSetAutoStartActive 1, returned \"" << cola_ascii << "\"");
  }
  else
  {
    ROS_INFO_STREAM("unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest testcase passed, returned \"" << cola_ascii << "\"");
  }
  testcase_cnt++;
  service_request_22.interval = 3;
  if ((cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request_22)) != "sMN LocSetAutoStartSavePoseInterval 3")
  {
    failed_testcase_cnt++;
    ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest() failed, expected sMN LocSetAutoStartSavePoseInterval , returned \"" << cola_ascii << "\"");
  }
  else
  {
    ROS_INFO_STREAM("unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest testcase passed, returned \"" << cola_ascii << "\"");
  }
  testcase_cnt++;
  service_request_24.active = true;
  if ((cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request_24)) != "sMN LocSetRingBufferRecordingActive 1")
  {
    failed_testcase_cnt++;
    ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest() failed, expected sMN LocSetRingBufferRecordingActive 1, returned \"" << cola_ascii << "\"");
  }
  else
  {
    ROS_INFO_STREAM("unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest testcase passed, returned \"" << cola_ascii << "\"");
  }
  testcase_cnt++;
  service_request_27.index = 0;
  if ((cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request_27)) != "sMN DevGetLidarState 0")
  {
    failed_testcase_cnt++;
    ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest() failed, expected sMN DevGetLidarState 0, returned \"" << cola_ascii << "\"");
  }
  else
  {
    ROS_INFO_STREAM("unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest testcase passed, returned \"" << cola_ascii << "\"");
  }
  testcase_cnt++;
  if ((cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request_28)) != "sMN GetSoftwareVersion")
  {
    failed_testcase_cnt++;
    ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest() failed, expected sMN GetSoftwareVersion, returned \"" << cola_ascii << "\"");
  }
  else
  {
    ROS_INFO_STREAM("unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest testcase passed, returned \"" << cola_ascii << "\"");
  }
  testcase_cnt++;
  if ((cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request_29)) != "sMN LocAutoStartSavePose")
  {
    failed_testcase_cnt++;
    ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest() failed, expected sMN LocAutoStartSavePose, returned \"" << cola_ascii << "\"");
  }
  else
  {
    ROS_INFO_STREAM("unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest testcase passed, returned \"" << cola_ascii << "\"");
  }
  testcase_cnt++;
  if ((cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request_30)) != "sMN LocForceUpdate")
  {
    failed_testcase_cnt++;
    ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest() failed, expected sMN LocForceUpdate, returned \"" << cola_ascii << "\"");
  }
  else
  {
    ROS_INFO_STREAM("unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest testcase passed, returned \"" << cola_ascii << "\"");
  }
  testcase_cnt++;
  service_request_31.reason = "YYYY-MM-DD_HH-MM-SS pose quality low";
  if ((cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request_31)) != "sMN LocSaveRingBufferRecording +36 YYYY-MM-DD_HH-MM-SS pose quality low")
  {
    failed_testcase_cnt++;
    ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest() failed, expected sMN LocSaveRingBufferRecording +36 YYYY-MM-DD_HH-MM-SS pose quality low, returned \"" << cola_ascii << "\"");
  }
  else
  {
    ROS_INFO_STREAM("unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest testcase passed, returned \"" << cola_ascii << "\"");
  }
  testcase_cnt++;
  if ((cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request_34)) != "sMN SavePermanent")
  {
    failed_testcase_cnt++;
    ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest() failed, expected sMN SavePermanent, returned \"" << cola_ascii << "\"");
  }
  else
  {
    ROS_INFO_STREAM("unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest testcase passed, returned \"" << cola_ascii << "\"");
  }
  testcase_cnt++;
  if ((cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request_35)) != "sRN LocResultPort")
  {
    failed_testcase_cnt++;
    ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest() failed, expected sRN LocResultPort, returned \"" << cola_ascii << "\"");
  }
  else
  {
    ROS_INFO_STREAM("unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest testcase passed, returned \"" << cola_ascii << "\"");
  }
  testcase_cnt++;
  if ((cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request_36)) != "sRN LocResultMode")
  {
    failed_testcase_cnt++;
    ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest() failed, expected sRN LocResultMode, returned \"" << cola_ascii << "\"");
  }
  else
  {
    ROS_INFO_STREAM("unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest testcase passed, returned \"" << cola_ascii << "\"");
  }
  testcase_cnt++;
  if ((cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request_37)) != "sRN LocResultEndianness")
  {
    failed_testcase_cnt++;
    ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest() failed, expected sRN LocResultEndianness, returned \"" << cola_ascii << "\"");
  }
  else
  {
    ROS_INFO_STREAM("unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest testcase passed, returned \"" << cola_ascii << "\"");
  }
  testcase_cnt++;
  if ((cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request_38)) != "sRN LocResultState")
  {
    failed_testcase_cnt++;
    ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest() failed, expected sRN LocResultState, returned \"" << cola_ascii << "\"");
  }
  else
  {
    ROS_INFO_STREAM("unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest testcase passed, returned \"" << cola_ascii << "\"");
  }
  testcase_cnt++;
  service_request_40.active = true;
  if ((cola_ascii = sick_lidar_localization::ColaEncoder::encodeServiceRequest(service_request_40)) != "sMN DevSetIMUActive 1")
  {
    failed_testcase_cnt++;
    ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest() failed, expected sMN DevSetIMUActive 1, returned \"" << cola_ascii << "\"");
  }
  else
  {
    ROS_INFO_STREAM("unittest_sim_loc_parser: ColaEncoder::encodeServiceRequest testcase passed, returned \"" << cola_ascii << "\"");
  }
  testcase_cnt++;
}


int main(int argc, char** argv)
{
  // Ros configuration and initialization
  ROS::init(argc, argv, "unittest_sim_loc_parser");
  ROS::NodePtr nh = ROS::createNode("unittest_sim_loc_parser");
  ROS_INFO_STREAM("unittest_sim_loc_parser started.");
  
  // Run sim_loc_parser unittest for result port telegrams
  int testcase_cnt = 0, failed_testcase_cnt = 0, number_result_port_testcases = 100; // default: run 100 random based result port testcases
  ROS::param<int>(nh, "/unittest_sim_loc_parser/number_result_port_testcases", number_result_port_testcases, number_result_port_testcases);
  sick_lidar_localization::ResultPortParser result_port_parser("sick_lidar_localization");
  sick_lidar_localization::SickLocResultPortTestcaseMsg testcase = sick_lidar_localization::TestcaseGenerator::createDefaultResultPortTestcase(); // initial testcase is the default testcase
  for(; testcase_cnt < number_result_port_testcases; testcase_cnt++)
  {
    // Decode binary result port telegram, re-encode and check identity
    std::vector<uint8_t> recoded_binary;
    if (!result_port_parser.decode(testcase.binary_data)
      || (recoded_binary = result_port_parser.encode()) != testcase.binary_data
      || !sick_lidar_localization::Utils::identicalByStream(result_port_parser.getTelegramMsg(), testcase.telegram_msg))
    {
      failed_testcase_cnt++;
      ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: sick_lidar_localization::ResultPortParser::decode() failed. " << testcase.binary_data.size() << " byte input (hex):");
      ROS_ERROR_STREAM(sick_lidar_localization::Utils::toHexString(testcase.binary_data));
      ROS_ERROR_STREAM("## output (decoded):  " << sick_lidar_localization::Utils::flattenToString(result_port_parser.getTelegramMsg()));
      ROS_ERROR_STREAM("## recoded:");
      ROS_ERROR_STREAM(sick_lidar_localization::Utils::toHexString(recoded_binary));
      ROS_ERROR_STREAM("## output (expected): " << sick_lidar_localization::Utils::flattenToString(testcase.telegram_msg));
      ROS_ERROR_STREAM("## unittest_sim_loc_parser: " << (testcase_cnt + 1) << ". testcase FAILED.");
    }
    else
    {
      ROS_INFO_STREAM("unittest_sim_loc_parser: " << (testcase_cnt + 1) << ". testcase passed (result port telegram " << sick_lidar_localization::Utils::toHexString(testcase.binary_data) << ")");
      ROS_DEBUG_STREAM("unittest_sim_loc_parser decoded telegram = " << sick_lidar_localization::Utils::flattenToString(result_port_parser.getTelegramMsg())
        << " identical to expected telegram = " << sick_lidar_localization::Utils::flattenToString(testcase.telegram_msg));
      ROS_DEBUG_STREAM("Input (hex):            " << sick_lidar_localization::Utils::toHexString(testcase.binary_data));
      ROS_DEBUG_STREAM("Recoded telegram (hex): " << sick_lidar_localization::Utils::toHexString(recoded_binary));
    }
    // next testcase is a result port telegram with random data
    testcase = sick_lidar_localization::TestcaseGenerator::createRandomResultPortTestcase();
  }
  
  // Run sim_loc_parser unittest for Cola Ascii telegrams
  std::string cola_ascii = "<STX>sMN SetAccessMode 3 F4724744<ETX>";
  std::vector<uint8_t> cola_binary = { 0x02, 0x73, 0x4D, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x41, 0x63, 0x63, 0x65, 0x73, 0x73, 0x4D, 0x6F, 0x64, 0x65, 0x20, 0x33, 0x20, 0x46, 0x34, 0x37, 0x32, 0x34, 0x37, 0x34, 0x34, 0x03 };
  if (sick_lidar_localization::ColaAsciiBinaryConverter::ConvertColaAscii(cola_binary) != cola_ascii
    || sick_lidar_localization::Utils::toHexString(sick_lidar_localization::ColaAsciiBinaryConverter::ConvertColaAscii(cola_ascii)) != sick_lidar_localization::Utils::toHexString(cola_binary))
  {
    failed_testcase_cnt++;
    ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: sick_lidar_localization::ColaAsciiBinaryConverter::convert() failed.");
    ROS_ERROR_STREAM("## Cola ASCII input: " << cola_ascii);
    ROS_ERROR_STREAM("## Converted Cola Binary ouput: " << sick_lidar_localization::Utils::toHexString(sick_lidar_localization::ColaAsciiBinaryConverter::ConvertColaAscii(cola_ascii)));
    ROS_ERROR_STREAM("## Expected Cola Binary ouput:  " << sick_lidar_localization::Utils::toHexString(cola_binary));
    ROS_ERROR_STREAM("## Cola Binary input: " << sick_lidar_localization::Utils::toHexString(cola_binary));
    ROS_ERROR_STREAM("## Converted Cola ASCII ouput: " << sick_lidar_localization::ColaAsciiBinaryConverter::ConvertColaAscii(cola_binary));
    ROS_ERROR_STREAM("## Expected Cola ASCII ouput:  " << cola_ascii);
  }
  else
  {
    ROS_INFO_STREAM("unittest_sim_loc_parser: " << (testcase_cnt + 1) << ". testcase passed (ColaAsciiBinaryConverter: \"" << cola_ascii << "\", " << sick_lidar_localization::Utils::toHexString(cola_binary) << ")");
  }
  testcase_cnt++;
  
  // Run sim_loc_parser unittest for parsing Cola telegram messages
  sick_lidar_localization::UniformRandomInteger random32_generator(0, INT32_MAX);
  std::vector<sick_lidar_localization::SickLocColaTelegramMsg> cola_telegram_tests_input;
  cola_telegram_tests_input.push_back(sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sRN, "SetAccessMode", {"3", "F4724744"}));
  cola_telegram_tests_input.push_back(sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sMN, "LocRequestTimestamp"));
  for(size_t n = 0; n < 10; n++) // Create some "sAN, "LocRequestTimestamp" with timestamp (response to "sMN LocRequestTimestamp")
    cola_telegram_tests_input.push_back(sick_lidar_localization::ColaParser::createColaTelegram(sick_lidar_localization::ColaParser::sAN, "LocRequestTimestamp", { std::to_string(random32_generator.generate())}));
  for(std::vector<sick_lidar_localization::SickLocColaTelegramMsg>::iterator iter_testcase = cola_telegram_tests_input.begin(); iter_testcase != cola_telegram_tests_input.end(); iter_testcase++, testcase_cnt++)
  {
    sick_lidar_localization::SickLocColaTelegramMsg & cola_telegram_input = *iter_testcase;
    std::vector<uint8_t> cola_binary = sick_lidar_localization::ColaParser::encodeColaTelegram(cola_telegram_input);
    sick_lidar_localization::SickLocColaTelegramMsg cola_telegram_output = sick_lidar_localization::ColaParser::decodeColaTelegram(cola_binary);
    cola_telegram_output.header = cola_telegram_input.header; // ignore different message timestamps
    if (cola_telegram_output.command_type == sick_lidar_localization::ColaParser::sINVALID
      || !sick_lidar_localization::Utils::identicalByStream(cola_telegram_input, cola_telegram_output))
    {
      failed_testcase_cnt++;
      ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: sick_lidar_localization::ColaParser::encodeColaTelegram/decodeColaTelegram failed:");
      ROS_ERROR_STREAM("## input:  " << sick_lidar_localization::Utils::flattenToString(cola_telegram_input));
      ROS_ERROR_STREAM("## output: " << sick_lidar_localization::Utils::flattenToString(cola_telegram_output));
    }
    else
    {
      ROS_INFO_STREAM("unittest_sim_loc_parser: " << (testcase_cnt + 1) << ". testcase passed (cola_telegram telegram " << sick_lidar_localization::Utils::flattenToString(cola_telegram_output) << ")");
    }
  }
  
  // Run sim_loc_parser unittest for converting Cola Ascii to Cola Binary telegrams.
  // Testcases taken from examples listed in manual Technical_information_Telegram_Listing_Ranging_sensors_LMS1xx_LMS5xx_TiM5xx_MRS1000_MRS6000_NAV310_LD_OEM15xx_LD_LRS36xx_LMS4000_en_IM0045927.pdf
  std::vector<ColaBinaryTestcase> cola_ascii_to_cola_binary_testcases =
    {
      ColaBinaryTestcase("<STX>sAN SetAccessMode 1<ETX>",
        {0x02, 0x73, 0x41, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x41, 0x63, 0x63, 0x65, 0x73, 0x73, 0x4D, 0x6F, 0x64, 0x65, 0x20, 0x31, 0x03},
        {0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x13, 0x73, 0x41, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x41, 0x63, 0x63, 0x65, 0x73, 0x73, 0x4D, 0x6F, 0x64, 0x65, 0x20, 0x01, 0x38}),
      ColaBinaryTestcase("<STX>sMN SetAccessMode 3 F4724744<ETX>",
        {0x02, 0x73, 0x4D, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x41, 0x63, 0x63, 0x65, 0x73, 0x73, 0x4D, 0x6F, 0x64, 0x65, 0x20, 0x33, 0x20, 0x46, 0x34, 0x37, 0x32, 0x34, 0x37, 0x34, 0x34, 0x03},
        {0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x17, 0x73, 0x4D, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x41, 0x63, 0x63, 0x65, 0x73, 0x73, 0x4D, 0x6F, 0x64, 0x65, 0x20, 0x03, 0xF4, 0x72, 0x47, 0x44, 0xB3}),
      ColaBinaryTestcase("<STX>sWA MMAlignmentMode<ETX>",
        {0x02, 0x73, 0x57, 0x41, 0x20, 0x4D, 0x4D, 0x41, 0x6C, 0x69, 0x67, 0x6E, 0x6D, 0x65, 0x6E, 0x74, 0x4D, 0x6F, 0x64, 0x65, 0x03},
        {0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x13, 0x73, 0x57, 0x41, 0x20, 0x4D, 0x4D, 0x41, 0x6C, 0x69, 0x67, 0x6E, 0x6D, 0x65, 0x6E, 0x74, 0x4D, 0x6F, 0x64, 0x65, 0x39}),
      ColaBinaryTestcase("<STX>sAN SetPassword 1<ETX>",
        {0x02, 0x73, 0x41, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x50, 0x61, 0x73, 0x73, 0x77, 0x6F, 0x72, 0x64, 0x20, 0x31, 0x03},
        {0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x11, 0x73, 0x41, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x50, 0x61, 0x73, 0x73, 0x77, 0x6F, 0x72, 0x64, 0x20, 0x31, 0x30}, false),
      ColaBinaryTestcase("<STX>sMN Run<ETX>",
        {0x02, 0x73, 0x4D, 0x4E, 0x20, 0x52, 0x75, 0x6E, 0x03},
        {0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x07, 0x73, 0x4D, 0x4E, 0x20, 0x52, 0x75, 0x6E, 0x19}),
      ColaBinaryTestcase("<STX>sAN Run 1<ETX>",
        {0x02, 0x73, 0x41, 0x4E, 0x20, 0x52, 0x75, 0x6E, 0x20, 0x31, 0x03},
        {0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x09, 0x73, 0x41, 0x4E, 0x20, 0x52, 0x75, 0x6E, 0x20, 0x01, 0x34}),
      ColaBinaryTestcase("<STX>sWN EIHstCola 0<ETX>",
        {0x02, 0x73, 0x57, 0x4E, 0x20, 0x45, 0x49, 0x48, 0x73, 0x74, 0x43, 0x6F, 0x6C, 0x61, 0x20, 0x30, 0x03},
        {0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x0F, 0x73, 0x57, 0x4E, 0x20, 0x45, 0x49, 0x48, 0x73, 0x74, 0x43, 0x6F, 0x6C, 0x61, 0x20, 0x00, 0x08}),
      ColaBinaryTestcase("<STX>sWA EIHstCola<ETX>",
        {0x02, 0x73, 0x57, 0x41, 0x20, 0x45, 0x49, 0x48, 0x73, 0x74, 0x43, 0x6F, 0x6C, 0x61, 0x03},
        {0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x0D, 0x73, 0x57, 0x41, 0x20, 0x45, 0x49, 0x48, 0x73, 0x74, 0x43, 0x6F, 0x6C, 0x61, 0x27})
        
    };
  for(std::vector<ColaBinaryTestcase>::iterator iter_testcase = cola_ascii_to_cola_binary_testcases.begin(); iter_testcase != cola_ascii_to_cola_binary_testcases.end(); iter_testcase++)
  {
    std::vector<uint8_t> cola_binary_test = sick_lidar_localization::ColaAsciiBinaryConverter::ColaAsciiToColaBinary(iter_testcase->cola_ascii, iter_testcase->parameter_to_ascii);
    if (sick_lidar_localization::Utils::toHexString(cola_binary_test) != sick_lidar_localization::Utils::toHexString(iter_testcase->cola_binary))
    {
      failed_testcase_cnt++;
      ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: sick_lidar_localization::ColaAsciiBinaryConverter::ColaAsciiToColaBinary(" << iter_testcase->description << ") failed.");
      ROS_ERROR_STREAM("## encoded:  " << sick_lidar_localization::Utils::toHexString(cola_binary_test));
      ROS_ERROR_STREAM("## expected: " << sick_lidar_localization::Utils::toHexString(iter_testcase->cola_binary));
    }
    else
    {
      ROS_INFO_STREAM("unittest_sim_loc_parser: " << (testcase_cnt + 1) << ". testcase passed (ColaAsciiToColaBinary, " << iter_testcase->description << ")");
    }
    testcase_cnt++;
  }
  
  // Run sim_loc_parser unittest for converting Cola Binary to Cola Ascii telegrams.
  // Testcases taken from examples listed in manual Technical_information_Telegram_Listing_Ranging_sensors_LMS1xx_LMS5xx_TiM5xx_MRS1000_MRS6000_NAV310_LD_OEM15xx_LD_LRS36xx_LMS4000_en_IM0045927.pdf
  std::vector<ColaBinaryTestcase> cola_binary_to_cola_ascii_testcases =
    {
      ColaBinaryTestcase("<STX>sAN SetAccessMode 1<ETX>",
        {0x02, 0x73, 0x41, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x41, 0x63, 0x63, 0x65, 0x73, 0x73, 0x4D, 0x6F, 0x64, 0x65, 0x20, 0x31, 0x03},
        {0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x13, 0x73, 0x41, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x41, 0x63, 0x63, 0x65, 0x73, 0x73, 0x4D, 0x6F, 0x64, 0x65, 0x20, 0x01, 0x38}),
      ColaBinaryTestcase("<STX>sRA TSCTCmaxoffset 468CA000<ETX>",
        {0x02, 0x73, 0x52, 0x41, 0x20, 0x54, 0x53, 0x43, 0x54, 0x43, 0x6D, 0x61, 0x78, 0x6F, 0x66, 0x66, 0x73, 0x65, 0x74, 0x20, 0x46, 0x8C, 0xA0, 0x00, 0x03},
        {0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x17, 0x73, 0x52, 0x41, 0x20, 0x54, 0x53, 0x43, 0x54, 0x43, 0x6D, 0x61, 0x78, 0x6F, 0x66, 0x66, 0x73, 0x65, 0x74, 0x20, 0x46, 0x8C, 0xA0, 0x00, 0x20}, false),
      ColaBinaryTestcase("<STX>sWA MMAlignmentMode<ETX>",
        {0x02, 0x73, 0x57, 0x41, 0x20, 0x4D, 0x4D, 0x41, 0x6C, 0x69, 0x67, 0x6E, 0x6D, 0x65, 0x6E, 0x74, 0x4D, 0x6F, 0x64, 0x65, 0x03},
        {0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x13, 0x73, 0x57, 0x41, 0x20, 0x4D, 0x4D, 0x41, 0x6C, 0x69, 0x67, 0x6E, 0x6D, 0x65, 0x6E, 0x74, 0x4D, 0x6F, 0x64, 0x65, 0x39}),
      ColaBinaryTestcase("<STX>sAN SetPassword 1<ETX>",
        {0x02, 0x73, 0x41, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x50, 0x61, 0x73, 0x73, 0x77, 0x6F, 0x72, 0x64, 0x20, 0x31, 0x03},
        {0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x11, 0x73, 0x41, 0x4E, 0x20, 0x53, 0x65, 0x74, 0x50, 0x61, 0x73, 0x73, 0x77, 0x6F, 0x72, 0x64, 0x20, 0x31, 0x30}, false),
      ColaBinaryTestcase("<STX>sMN Run<ETX>",
        {0x02, 0x73, 0x4D, 0x4E, 0x20, 0x52, 0x75, 0x6E, 0x03},
        {0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x07, 0x73, 0x4D, 0x4E, 0x20, 0x52, 0x75, 0x6E, 0x19}),
      ColaBinaryTestcase("<STX>sAN Run 1<ETX>",
        {0x02, 0x73, 0x41, 0x4E, 0x20, 0x52, 0x75, 0x6E, 0x20, 0x31, 0x03},
        {0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x09, 0x73, 0x41, 0x4E, 0x20, 0x52, 0x75, 0x6E, 0x20, 0x01, 0x34}),
      ColaBinaryTestcase("<STX>sWN EIHstCola 0<ETX>",
        {0x02, 0x73, 0x57, 0x4E, 0x20, 0x45, 0x49, 0x48, 0x73, 0x74, 0x43, 0x6F, 0x6C, 0x61, 0x20, 0x30, 0x03},
        {0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x0F, 0x73, 0x57, 0x4E, 0x20, 0x45, 0x49, 0x48, 0x73, 0x74, 0x43, 0x6F, 0x6C, 0x61, 0x20, 0x00, 0x08}),
      ColaBinaryTestcase("<STX>sWA EIHstCola<ETX>",
        {0x02, 0x73, 0x57, 0x41, 0x20, 0x45, 0x49, 0x48, 0x73, 0x74, 0x43, 0x6F, 0x6C, 0x61, 0x03},
        {0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x0D, 0x73, 0x57, 0x41, 0x20, 0x45, 0x49, 0x48, 0x73, 0x74, 0x43, 0x6F, 0x6C, 0x61, 0x27})
        
    };
  for(std::vector<ColaBinaryTestcase>::iterator iter_testcase = cola_binary_to_cola_ascii_testcases.begin(); iter_testcase != cola_binary_to_cola_ascii_testcases.end(); iter_testcase++)
  {
    std::vector<uint8_t> cola_ascii_test = sick_lidar_localization::ColaAsciiBinaryConverter::ColaBinaryToColaAscii(iter_testcase->cola_binary, iter_testcase->parameter_to_ascii);
    if (sick_lidar_localization::Utils::toHexString(cola_ascii_test) != sick_lidar_localization::Utils::toHexString(iter_testcase->cola_ascii))
    {
      failed_testcase_cnt++;
      ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser: sick_lidar_localization::ColaAsciiBinaryConverter::ColaBinaryToColaAscii(" << iter_testcase->description << ") failed.");
      ROS_ERROR_STREAM("## encoded:  " << sick_lidar_localization::Utils::toHexString(cola_ascii_test));
      ROS_ERROR_STREAM("## expected: " << sick_lidar_localization::Utils::toHexString(iter_testcase->cola_ascii));
    }
    else
    {
      ROS_INFO_STREAM("unittest_sim_loc_parser: " << (testcase_cnt + 1) << ". testcase passed (ColaBinaryToColaAscii, " << iter_testcase->description << ")");
    }
    testcase_cnt++;
  }

  // Run sim_loc_parser unittest for encoding new services to Cola Ascii telegrams. Unittest for new services in release 4 and later
  // Testcases taken from examples listed in manual 8024818_TI_LiDAR_Localization_Telegram_Listing_en_9317743_2020-09-11.pdf
  RunColaEncoderUnitTests(testcase_cnt, failed_testcase_cnt);

  ROS_INFO_STREAM("unittest_sim_loc_parser finished, " << (testcase_cnt - failed_testcase_cnt) << " of " << testcase_cnt << " testcases passed, " << failed_testcase_cnt << " testcases failed.");
  if(failed_testcase_cnt > 0)
  {
    ROS_ERROR_STREAM("## ERROR unittest_sim_loc_parser finished, " << (testcase_cnt - failed_testcase_cnt) << " of " << testcase_cnt << " testcases passed, " << failed_testcase_cnt << " testcases failed.");
    ROS::spin(nh);
  }
  ROS::deleteNode(nh);
  return 0;
}
