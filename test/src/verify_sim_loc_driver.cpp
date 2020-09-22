/*
 * @brief verify_sim_loc_driver verifies the sim_loc_driver messages
 * against testcases published by sim_loc_test_server.
 *
 * To verify sim_loc_driver, sim_loc_driver runs against sim_loc_test_server.
 * sim_loc_test_server generates and publishes testcases with deterministic
 * and randomly generated result port telegrams. sim_loc_driver receives
 * result port telegrams from the server via TCP, decodes the telegrams
 * and publishes SickLocResultPortTelegramMsg messages.
 *
 * verify_sim_loc_driver subscribes to both sim_loc_driver messages and
 * sim_loc_test_server messages and compares their content. A warning will
 * be logged in case of failures or mismatches.
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

#include "sick_lidar_localization/verifier_thread.h"

int main(int argc, char** argv)
{
  // Ros configuration and initialization
  ROS::init(argc, argv, "verify_sim_loc_driver");
  ROS::NodePtr nh = ROS::createNode("verify_sim_loc_driver");
  ROS_INFO_STREAM("verify_sim_loc_driver started.");
  
  std::string result_telegrams_topic = "/sick_lidar_localization/driver/result_telegrams";      // default topic to publish result port telegram messages (type SickLocResultPortTelegramMsg)
  std::string result_testcases_topic = "/sick_lidar_localization/test_server/result_testcases"; // default topic to publish testcases with result port telegrams (type SickLocResultPortTestcaseMsg)
  ROS::param<std::string>(nh, "/sick_lidar_localization/driver/result_telegrams_topic", result_telegrams_topic, result_telegrams_topic);
  ROS::param<std::string>(nh, "/sick_lidar_localization/test_server/result_testcases_topic", result_testcases_topic, result_testcases_topic);
  
  // Init verifier to compare and check sim_loc_driver and sim_loc_test_server messages
  sick_lidar_localization::VerifierThread verifier(nh);
  
  // Subscribe to sim_loc_driver messages and sim_loc_test_server messages
#if defined __ROS_VERSION && __ROS_VERSION == 1
  sick_lidar_localization::SickLocResultPortTelegramMsgSubscriber result_telegram_subscriber 
    = ROS_CREATE_SUBSCRIBER(nh, sick_lidar_localization::SickLocResultPortTelegramMsg, result_telegrams_topic, &sick_lidar_localization::VerifierThread::messageCbResultPortTelegrams, &verifier);
  sick_lidar_localization::SickLocResultPortTestcaseMsgSubscriber testcase_subscriber
    = ROS_CREATE_SUBSCRIBER(nh, sick_lidar_localization::SickLocResultPortTestcaseMsg, result_testcases_topic, &sick_lidar_localization::VerifierThread::messageCbResultPortTestcases, &verifier);
#elif defined __ROS_VERSION && __ROS_VERSION == 2
  sick_lidar_localization::SickLocResultPortTelegramMsgSubscriber result_telegram_subscriber
    = ROS_CREATE_SUBSCRIBER(nh, sick_lidar_localization::SickLocResultPortTelegramMsg, result_telegrams_topic, &sick_lidar_localization::VerifierThread::messageCbResultPortTelegramsROS2, &verifier);
  sick_lidar_localization::SickLocResultPortTestcaseMsgSubscriber testcase_subscriber
    = ROS_CREATE_SUBSCRIBER(nh, sick_lidar_localization::SickLocResultPortTestcaseMsg, result_testcases_topic, &sick_lidar_localization::VerifierThread::messageCbResultPortTestcasesROS2, &verifier);
#endif


  // Start verification thread
  verifier.start();
  
  // Run ros event loop
  ROS::spin(nh);
  
  std::cout << "verify_sim_loc_driver finished." << std::endl;
  ROS_INFO_STREAM("verify_sim_loc_driver finished.");
  verifier.stop();
  std::cout << "verify_sim_loc_driver exits." << std::endl;
  ROS_INFO_STREAM("verify_sim_loc_driver exits.");
  ROS::deleteNode(nh);
  return 0;
}
