/*
 * @brief cola_service_node advertises and runs the following ros services for sick_lidar_localization:
 *
 * ROS service name               | Definition file                         | Cola request telegram                                | Description
 * "SickLocIsSystemReady"         | srv/SickLocIsSystemReadySrv.srv         | "sMN IsSystemReady"                                  | Check if the system is ready
 * "SickLocState"                 | srv/SickLocStateSrv.srv                 | "sRN LocState"                                       | Read localization state
 * "SickLocStartLocalizing"       | srv/SickLocStartLocalizingSrv.srv       | "sMN LocStartLocalizing"                             | Start localization
 * "SickLocStop"                  | srv/SickLocStopSrv.srv                  | "sMN LocStop"                                        | Stop localization or demo mapping
 * "SickLocStopAndSave"           | srv/SickLocStopAndSaveSrv.srv           | "sMN LocStopAndSave"                                 | Stop localization, save settings
 * "SickLocSetResultPort"         | srv/SickLocSetResultPortSrv.srv         | "sMN LocSetResultPort <port>"                        | Set TCP-port for result output (default: 2201)
 * "SickLocSetResultMode"         | srv/SickLocSetResultModeSrv.srv         | "sMN LocSetResultMode <mode>"                        | Set mode of result output (default: stream)
 * "SickLocSetResultPoseEnabled"  | srv/SickLocSetResultPoseEnabledSrv.srv  | "sMN LocSetResultPoseEnabled <enabled>"              | Disable/enable result output
 * "SickLocSetResultEndianness"   | srv/SickLocSetResultEndiannessSrv.srv   | "sMN LocSetResultEndianness <endianness>"            | Set endianness of result output
 * "SickLocSetResultPoseInterval" | srv/SickLocSetResultPoseIntervalSrv.srv | "sMN LocSetResultPoseInterval <interval>"            | Set interval of result output
 * "SickLocRequestResultData"     | srv/SickLocRequestResultDataSrv.srv     | "sMN LocRequestResultData"                           | If in poll mode, trigger sending the localization result of the next processed scan via TCP interface.
 * "SickLocSetPose"               | srv/SickLocSetPoseSrv.srv               | "sMN LocSetPose <posex> <posey> <yaw> <uncertainty>" | Initialize vehicle pose
 *
 * See Telegram-Listing-v1.1.0.241R.pdf for further details about Cola telegrams.
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

#include "sick_lidar_localization/cola_configuration.h"
#include "sick_lidar_localization/cola_services.h"

int main(int argc, char** argv)
{
  // Ros configuration and initialization
  ros::init(argc, argv, "cola_service_node");
  ros::NodeHandle nh;
  ROS_INFO_STREAM("cola_service_node started.");
  
  // Initialize cola services
  sick_lidar_localization::ColaServices cola_services(&nh);
  
  // Initial configuration from launch file
  sick_lidar_localization::ColaConfiguration cola_configuration(&nh);
  cola_configuration.start();
  
  // Run ros event loop
  ros::spin();
  
  // Cleanup and exit
  std::cout << "cola_service_node finished." << std::endl;
  ROS_INFO_STREAM("cola_service_node finished.");
  return EXIT_SUCCESS;
}
