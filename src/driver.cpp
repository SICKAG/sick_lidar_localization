/*
 * @brief sim_loc_driver implements a ros driver for sick localization.
 * sim_loc_driver establishes a tcp connection to the localization controller
 * (f.e. SIM1000FXA), receives and converts result port telegrams and
 * publishes all sim location data by ros messages of type SickLocResultPortTelegramMsg.
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

#include "sick_lidar_localization/driver_monitor.h"
#include "sick_lidar_localization/cola_configuration.h"
#include "sick_lidar_localization/cola_services.h"
#include "sick_lidar_localization/odom_converter.h"
#include "sick_lidar_localization/time_sync_service.h"

int main(int argc, char** argv)
{
  // Ros configuration and initialization
  ROS::init(argc, argv, "sim_loc_driver");
  ROS::NodePtr nh = ROS::createNode("sim_loc_driver");
  ROS_INFO_STREAM("sim_loc_driver started.");
  
  // Configuration and parameter for sim_loc_driver
  std::string server_adress("192.168.0.1"), server_default_adress("192.168.0.1"), system_server_adress("");  // IP adress of the localization controller, default: "192.168.0.1"

  // Default tcp ports: see Operation-Instruction-v1.1.0.241R.pdf, page 51, "IP port number and protocol":
  // For requests and to transmit settings to the localization controller:
  // * IP port number 2111 and 2112 to send telegrams and to request data.
  // * SOPAS CoLa-A or CoLa-B protocols
  // To transmit the localization results to the vehicle controller, the localization controller uses:
  // * IP port number 2201 to send localization results in a single direction to the external vehicle controller.
  // * Binary result port protocol TCP/IP
  int tcp_port_results = 2201; // Default: The localization controller uses IP port number 2201 to send localization results
  int tcp_port_cola = 2111;    // For requests and to transmit settings to the localization controller: IP port number 2111 and 2112 to send telegrams and to request data, SOPAS CoLa-A or CoLa-B protocols
  int udp_port_odom = 3000;    // Default udp port to send odom packages to the localization controller
  ROS::param<std::string>(nh, "/sim_loc_driver/localization_controller_ip_address" , server_adress, server_adress);
  ROS::param<std::string>(nh, "/sick_lidar_localization/driver/localization_controller_default_ip_address", server_default_adress, server_default_adress);
  ROS::param<int>(nh, "/sick_lidar_localization/driver/result_telegrams_tcp_port", tcp_port_results, tcp_port_results);
  ROS::param<int>(nh, "/sick_lidar_localization/driver/cola_telegrams_tcp_port", tcp_port_cola, tcp_port_cola);
  ROS::param<int>(nh, "/sick_lidar_localization/driver/odom_telegrams_udp_port", udp_port_odom, udp_port_odom);
  ROS::param<std::string>(nh, "/system/localization_controller_ip_address" , system_server_adress, system_server_adress); // optional system setting
  if(!system_server_adress.empty())
    server_adress = system_server_adress;
  server_adress = (server_adress.empty()) ? server_default_adress : server_adress;
  
  // Initialize driver threads to connect to localization controller and to monitor driver messages
  // DriverMonitor creates a worker thread, which
  // - connects to the localization controller (f.e. SIM1000FXA),
  // - receives binary result port telegrams,
  // - converts them to SickLocResultPortTelegramMsg
  // - publishes the sim location data
  sick_lidar_localization::DriverMonitor driver_monitor(nh, server_adress, tcp_port_results, tcp_port_cola);
  
  // Initialize cola services
  sick_lidar_localization::ColaServices cola_services(nh, &driver_monitor);

  // Initialize TimeSyncService
  sick_lidar_localization::TimeSyncService time_sync_service(nh, &driver_monitor);

  // Initialize odometry converter
  sick_lidar_localization::OdomConverter odom_converter(nh, server_adress, udp_port_odom, true);
  
  // sim_loc_driver messages and services
#if defined __ROS_VERSION && __ROS_VERSION == 1
  auto messageCbResultPortTelegrams =  &sick_lidar_localization::DriverMonitor::messageCbResultPortTelegrams;
  auto serviceCbColaTelegram = &sick_lidar_localization::DriverMonitor::serviceCbColaTelegram;
  auto messageCbOdometry =  &sick_lidar_localization::OdomConverter::messageCbOdometry;
#elif defined __ROS_VERSION && __ROS_VERSION == 2
  auto messageCbResultPortTelegrams =  &sick_lidar_localization::DriverMonitor::messageCbResultPortTelegramsROS2;
  auto serviceCbColaTelegram = &sick_lidar_localization::DriverMonitor::serviceCbColaTelegramROS2;
  auto messageCbOdometry =  &sick_lidar_localization::OdomConverter::messageCbOdometryROS2;
#endif

  // Subscribe to sim_loc_driver messages
  std::string result_telegrams_topic = "/sick_lidar_localization/driver/result_telegrams";      // default topic to publish result port telegram messages (type SickLocResultPortTelegramMsg)
  ROS::param<std::string>(nh, "/sick_lidar_localization/driver/result_telegrams_topic", result_telegrams_topic, result_telegrams_topic);
  sick_lidar_localization::SickLocResultPortTelegramMsgSubscriber result_telegram_subscriber = ROS_CREATE_SUBSCRIBER(nh, sick_lidar_localization::SickLocResultPortTelegramMsg, result_telegrams_topic, messageCbResultPortTelegrams, &driver_monitor);
  std::string odom_topic = "/odom"; // ros topic for odometry messages
  ROS::param<std::string>(nh, "/sick_lidar_localization/driver/odom_topic", odom_topic, odom_topic);
  sick_lidar_localization::OdomMsgSubscriber odom_subscriber = ROS_CREATE_SUBSCRIBER(nh, sick_lidar_localization::OdomMsg, odom_topic, messageCbOdometry, &odom_converter);

  // Advertise service "SickLocColaTelegram" to send and receive Cola-ASCII telegrams to resp. from the localization server (request and response)
  sick_lidar_localization::SickLocColaTelegramSrvServer srv_server = ROS_CREATE_SRV_SERVER(nh, sick_lidar_localization::SickLocColaTelegramSrv, "SickLocColaTelegram", serviceCbColaTelegram, &driver_monitor);
  ROS_INFO_STREAM("sim_loc_driver advertising service \"SickLocColaTelegram\" for Cola commands, message type SickLocColaTelegramSrv");
  
  // Start driver threads to connect to localization controller and to monitor driver messages
  if(!driver_monitor.start())
  {
    ROS_ERROR_STREAM("## ERROR sim_loc_driver: could not start driver monitor thread, exiting");
    return EXIT_FAILURE;
  }

  // Initial configuration from launch file
  sick_lidar_localization::ColaConfiguration cola_configuration(nh, &cola_services);
  cola_configuration.start();  

  // Start time synchronization thread to run the software pll
  if(!time_sync_service.start())
  {
    ROS_ERROR_STREAM("## ERROR time_sync: could not start synchronization thread, exiting");
    return EXIT_FAILURE;
  }
  
  // Run ros event loop
  ROS::spin(nh);
  
  // Cleanup and exit
  std::cout << "sim_loc_driver finished." << std::endl;
  ROS_INFO_STREAM("sim_loc_driver finished.");
  driver_monitor.stop();
  std::cout << "sim_loc_driver exits." << std::endl;
  ROS_INFO_STREAM("sim_loc_driver exits.");
  ROS::deleteNode(nh);
  return EXIT_SUCCESS;
}
