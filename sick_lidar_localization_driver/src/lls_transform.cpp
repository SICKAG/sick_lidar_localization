/*
 * @brief lls_transform subscribes to lls_loc_driver messages (type sick_lidar_localization::LocResultPortTelegramMsg),
 * converts them to PointCloud2 and publishes PointCloud2 messages on topic "/cloud".
 *
 * It also serves as an usage example for sick_lidar_localization and shows how to subscribe and use the
 * sick_lidar_localization messages.
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
#include "sick_lidar_localization/sick_common.h"
#include "sick_lidar_localization/lls_transform.h"
#if __ROS_VERSION == 2
#include "launchparser/launchparser.h"
#endif

int main(int argc, char** argv)
{
  // Ros configuration and initialization
#if __ROS_VERSION == 1
    ros::init(argc, argv, "lls_transform", ros::init_options::NoSigintHandler);
    ros::NodeHandle node("~");
    rosNodePtr nh = &node;
#elif __ROS_VERSION ==  2
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    //node_options.automatically_declare_initial_parameters(true);
    rosNodePtr nh = rclcpp::Node::make_shared("lls_transform", "", node_options);
    if (!LaunchParser::parseLaunchfileSetParameter(nh, argc, argv))
    {
        ROS_ERROR_STREAM("## ERROR sick_lidar_localization: parseLaunchfileSetParameter() failed, aborting... ");
        return false;
    }
#else
#error __ROS_VERSION not defined, define __ROS_VERSION 1 or 2
#endif

  ROS_INFO_STREAM("lls_transform started.");
  
  std::string result_telegrams_topic = "/localizationcontroller/out/localizationcontroller_result_message_0502"; // default topic to publish result port telegram messages (type LocResultPortTelegramMsg)
  //rosGetParam(nh, "result_telegrams_topic", result_telegrams_topic);
  
  // Init verifier to compare and check lls_loc_driver and lls_loc_test_server messages
  sick_lidar_localization::LLSTransformer lls_transform(nh);
  
  // Subscribe to lls_loc_driver messages
#if __ROS_VERSION == 1
  rosSubscriber<sick_lidar_localization_msgs::LocalizationControllerResultMessage0502> result_telegram_subscriber = rosSubscribe<sick_lidar_localization_msgs::LocalizationControllerResultMessage0502>(nh, 
      result_telegrams_topic, &sick_lidar_localization::LLSTransformer::messageCbResultPortTelegrams, &lls_transform);
#elif __ROS_VERSION == 2
# ifdef _MSC_VER
  auto subscriber = nh->create_subscription<sick_lidar_localization_msgs::LocalizationControllerResultMessage0502>(result_telegrams_topic, 10, std::bind(&sick_lidar_localization::LLSTransformer::messageCbResultPortTelegramsROS2, &lls_transform, std::placeholders::_1));
  rosSubscriber<sick_lidar_localization_msgs::LocalizationControllerResultMessage0502> result_telegram_subscriber = rosSubscriber<sick_lidar_localization_msgs::LocalizationControllerResultMessage0502>(subscriber);
# else
  rosSubscriber<sick_lidar_localization_msgs::LocalizationControllerResultMessage0502> result_telegram_subscriber = rosSubscribe<sick_lidar_localization_msgs::LocalizationControllerResultMessage0502>(nh, 
      result_telegrams_topic, &sick_lidar_localization::LLSTransformer::messageCbResultPortTelegramsROS2, &lls_transform);
#endif
#endif
 
  // Start pointcloud converter thread
  lls_transform.start();
  
  // Run ros event loop
  rosSpin(nh);
  
  std::cout << "lls_transform finished." << std::endl;
  ROS_INFO_STREAM("lls_transform finished.");
  lls_transform.stop();
  std::cout << "lls_transform exits." << std::endl;
  ROS_INFO_STREAM("lls_transform exits.");
  return 0;
}
