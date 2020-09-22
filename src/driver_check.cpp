/*
 * @brief sim_loc_driver_check subscribes to sim_loc_driver messages
 * and checks the telegram data against configured min and max values.
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
#include "sick_lidar_localization/ros_wrapper.h"

#include "sick_lidar_localization/driver_check_thread.h"

int main(int argc, char** argv)
{
  // Ros configuration and initialization
  ROS::init(argc, argv, "sim_loc_driver_check");
  ROS::NodePtr nh = ROS::createNode("sim_loc_driver_check");
  ROS_INFO_STREAM("sim_loc_driver_check started.");
  
  std::string result_telegrams_topic = "/sick_lidar_localization/driver/result_telegrams"; // default topic to publish result port telegram messages (type SickLocResultPortTelegramMsg)
  ROS::param<std::string>(nh, "/sick_lidar_localization/sim_loc_driver_check/result_telegrams_topic", result_telegrams_topic, result_telegrams_topic);
  
  // Init thread to check sim_loc_driver messages against configured min and max values
  sick_lidar_localization::MessageCheckThread check_thread(nh);
  
  // Subscribe to sim_loc_driver messages
#if defined __ROS_VERSION && __ROS_VERSION == 1
  sick_lidar_localization::SickLocResultPortTelegramMsgSubscriber result_telegram_subscriber 
    = ROS_CREATE_SUBSCRIBER(nh, sick_lidar_localization::SickLocResultPortTelegramMsg, result_telegrams_topic, &sick_lidar_localization::MessageCheckThread::messageCbResultPortTelegrams, &check_thread);
#elif defined __ROS_VERSION && __ROS_VERSION == 2
  sick_lidar_localization::SickLocResultPortTelegramMsgSubscriber result_telegram_subscriber
    = ROS_CREATE_SUBSCRIBER(nh, sick_lidar_localization::SickLocResultPortTelegramMsg, result_telegrams_topic, &sick_lidar_localization::MessageCheckThread::messageCbResultPortTelegramsROS2, &check_thread);
#endif
  
  // Start checking thread
  check_thread.start();
  
  // Run ros event loop
  ROS::spin(nh);
  
  std::cout << "sim_loc_driver_check finished." << std::endl;
  ROS_INFO_STREAM("sim_loc_driver_check finished.");
  check_thread.stop();
  std::cout << "sim_loc_driver_check exits." << std::endl;
  ROS_INFO_STREAM("sim_loc_driver_check exits.");
  ROS::deleteNode(nh);
  return 0;
}
