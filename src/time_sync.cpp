/*
 * @brief time_sync advertises and runs ros services "SickLocRequestTimestamp" and "SickLocTimeSync"
 * for time synchronization.
 *
 * ROS service SickLocRequestTimestamp requests a timestamp from the localization controller
 * by sending cola command LocRequestTimestamp ("sMN LocRequestTimestamp").
 *
 * The service receives and decodes the current timestamp (uint32 timestamp in milliseconds)
 * and calculates the time offset with the following formular:
 *
 * delta_time_ms := mean_time_vehicle_ms - timestamp_lidar_ms
 * mean_time_vehicle_ms := (send_time_vehicle + receive_time_vehicle) / 2
 *                      := vehicles mean timestamp in milliseconds
 * send_time_vehicle    := vehicles timestamp when sending LocRequestTimestamp
 * receive_time_vehicle := vehicles timestamp when receiving the LocRequestTimestamp response
 * timestamp_lidar_ms   := lidar timestamp in milliseconds from LocRequestTimestamp response
 *
 * See Operation-Instruction-v1.1.0.241R.pdf for details about time synchronization and
 * time offset calculation. See Technical_information_Telegram_Listing_NAV_LOC_en_IM0076556.PDF
 * for further details about Cola telegram LocRequestTimestamp.
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

#include "sick_lidar_localization/time_sync_service.h"

int main(int argc, char** argv)
{
  // Ros configuration and initialization
  ros::init(argc, argv, "time_sync");
  ros::NodeHandle nh;
  ROS_INFO_STREAM("time_sync started.");
  
  // Initialize TimeSyncService
  sick_lidar_localization::TimeSyncService time_sync_service(&nh);
  
  // Start time synchronization thread to run the software pll
  if(!time_sync_service.start())
  {
    ROS_ERROR_STREAM("## ERROR time_sync: could not start synchronization thread, exiting");
    return EXIT_FAILURE;
  }
  
  // Run ros event loop
  ros::spin();
  
  // Cleanup and exit
  std::cout << "time_sync finished." << std::endl;
  ROS_INFO_STREAM("time_sync finished.");
  time_sync_service.stop();
  std::cout << "time_sync exits." << std::endl;
  ROS_INFO_STREAM("time_sync exits.");
  return EXIT_SUCCESS;
}
