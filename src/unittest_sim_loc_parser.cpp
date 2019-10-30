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
#include <ros/ros.h>
#include <string>
#include <vector>

#include "sick_lidar_localization/sim_loc_testcase_generator.h"
#include "sick_lidar_localization/sim_loc_utils.h"

int main(int argc, char** argv)
{
  // Ros configuration and initialization
  ros::init(argc, argv, "unittest_sim_loc_parser");
  ros::NodeHandle nh;
  
  // Run testcases for sim_loc_parser
  ROS_INFO_STREAM("unittest_sim_loc_parser started.");
  int number_result_port_testcases = 100; // default: run 100 random based result port testcases
  ros::param::param<int>("/unittest_sim_loc_parser/number_result_port_testcases", number_result_port_testcases, number_result_port_testcases);
  sick_lidar_localization::ResultPortParser result_port_parser("sick_lidar_localization");
  sick_lidar_localization::SickLocResultPortTestcaseMsg testcase = sick_lidar_localization::TestcaseGenerator::createDefaultResultPortTestcase(); // initial testcase is the default testcase
  for(int testcase_cnt = 0; testcase_cnt < number_result_port_testcases; testcase_cnt++)
  {
    // Decode binary result port telegram, re-encode and check identity
    std::vector<uint8_t> recoded_binary;
    if (!result_port_parser.decode(testcase.binary_data)
      || (recoded_binary = result_port_parser.encode()) != testcase.binary_data
      || !sick_lidar_localization::Utils::identicalByStream(result_port_parser.getTelegramMsg(), testcase.telegram_msg))
    {
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
      ROS_INFO_STREAM("unittest_sim_loc_parser: " << (testcase_cnt + 1) << ". testcase okay (result port telegram " << sick_lidar_localization::Utils::toHexString(testcase.binary_data) << ")");
      ROS_DEBUG_STREAM("unittest_sim_loc_parser decoded telegram = " << sick_lidar_localization::Utils::flattenToString(result_port_parser.getTelegramMsg())
        << " identical to expected telegram = " << sick_lidar_localization::Utils::flattenToString(testcase.telegram_msg));
      ROS_DEBUG_STREAM("Input (hex):            " << sick_lidar_localization::Utils::toHexString(testcase.binary_data));
      ROS_DEBUG_STREAM("Recoded telegram (hex): " << sick_lidar_localization::Utils::toHexString(recoded_binary));
    }
    // next testcase is a result port telegram with random data
    testcase = sick_lidar_localization::TestcaseGenerator::createRandomResultPortTestcase();
  }
  
  ROS_INFO_STREAM("unittest_sim_loc_parser finished.");
  return 0;
}
