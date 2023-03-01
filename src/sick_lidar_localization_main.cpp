/*
 * @brief sick_lidar_localization implements the driver for sick localization.
 * It connects to the localization server (f.e. SIM1000FXA) and handles
 * all data communication incl. UDP telegrams and REST commands.
 *
 * Copyright (C) 2021 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2021 SICK AG, Waldkirch
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
 *  Copyright 2021 SICK AG
 *  Copyright 2021 Ing.-Buero Dr. Michael Lehning
 *
 */
#include <limits>
#include <thread>
#include "sick_lidar_localization/sick_lidar_localization.h"

// Examples to send UDP messages to the localization controller (native Linux or Windows, on ROS-1 and ROS-2 by corresponding ros-messages):
static void sendUDPMessageExamples(sick_lidar_localization::API& lidar_loc_api)
{
    sick_lidar_localization::UDPMessage::OdometryPayload0104 odometry0104;
    sick_lidar_localization::UDPMessage::OdometryPayload0105 odometry0105;
    sick_lidar_localization::UDPMessage::EncoderMeasurementPayload0202 encoder_measurement0202;
    sick_lidar_localization::UDPMessage::CodeMeasurementPayload0303 code_measurement0303;
    sick_lidar_localization::UDPMessage::CodeMeasurementPayload0701 code_measurement0701;
    sick_lidar_localization::UDPMessage::LineMeasurementPayload0403 line_measurement0403;
    sick_lidar_localization::UDPMessage::LineMeasurementPayload0404 line_measurement0404;
    odometry0104.telegram_count = 1000001;
    odometry0104.timestamp = 123456789;
    odometry0104.source_id = 1;
    odometry0104.x_velocity = -1234;
    odometry0104.y_velocity = -1234;
    odometry0104.angular_velocity = 1234;
    odometry0105.telegram_count = 1000002;
    odometry0105.timestamp = 123456780;
    odometry0105.source_id = 1;
    odometry0105.x_position = -1234;
    odometry0105.y_position = -1234;
    odometry0105.heading = 1234;
    encoder_measurement0202.telegram_count = 1000003;
    encoder_measurement0202.timestamp = 123456781;
    encoder_measurement0202.source_id = 1;
    encoder_measurement0202.encoder_value = 123456789;
    code_measurement0303.telegram_count = 1000004;
    code_measurement0303.timestamp = 123456782;
    code_measurement0303.source_id = 1;
    code_measurement0303.code = 1234;
    code_measurement0701.telegram_count = 1000004;
    code_measurement0701.timestamp = 123456782;
    code_measurement0701.source_id = 1;
    code_measurement0701.code = "1234";
    code_measurement0701.x_position = -1234;
    code_measurement0701.y_position = -1234;
    code_measurement0701.heading = 1234;
    line_measurement0403.telegram_count = 1000005;
    line_measurement0403.timestamp = 123456783;
    line_measurement0403.source_id = 1;
    line_measurement0403.num_lanes = 1;
    line_measurement0403.lanes.push_back(1234);
    line_measurement0404.telegram_count = 1000006;
    line_measurement0404.timestamp = 123456784;
    line_measurement0404.source_id = 1;
    line_measurement0404.lcp1 = 12;
    line_measurement0404.lcp2 = 34;
    line_measurement0404.lcp3 = 56;
    line_measurement0404.cnt_lpc = 78;
    line_measurement0404.reserved = 0;
    if (!lidar_loc_api.sendUDPMessage(odometry0104, false, false)
        || !lidar_loc_api.sendUDPMessage(odometry0105, false, false)
        || !lidar_loc_api.sendUDPMessage(encoder_measurement0202, false, false)
        || !lidar_loc_api.sendUDPMessage(code_measurement0303, false, false)
        || !lidar_loc_api.sendUDPMessage(code_measurement0701, false, false)
        || !lidar_loc_api.sendUDPMessage(line_measurement0403, false, false)
        || !lidar_loc_api.sendUDPMessage(line_measurement0404, false, false))
    {
        ROS_ERROR_STREAM("## ERROR sick_lidar_localization: UDPSender::sendUDPPayload() failed");
    }
}

int main(int argc, char** argv)
{
    /*
    ** sick_lidar_localization: Initialization
    */
#if __ROS_VERSION == 0
    rosNodePtr node = 0;
    bool launchfile_ok = sick_lidar_localization::API::parseLaunchfileSetParameter(node, argc, argv);
#elif __ROS_VERSION == 1
    ros::init(argc, argv, "sick_lidar_localization", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("~");
    rosNodePtr node = &nh;
    bool launchfile_ok = true;
#elif __ROS_VERSION ==  2
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    //node_options.automatically_declare_initial_parameters(true);
    rosNodePtr node = rclcpp::Node::make_shared("sick_lidar_localization", "", node_options);
    bool launchfile_ok = sick_lidar_localization::API::parseLaunchfileSetParameter(node, argc, argv);
#else
#error __ROS_VERSION not defined
#endif

    if (!launchfile_ok)
    {
        ROS_ERROR_STREAM("## ERROR sick_lidar_localization: parseLaunchfileSetParameter() failed, aborting... ");
        exit(EXIT_FAILURE);
    }

    /*
    ** sick_lidar_localization: Configuration
    */
    sick_lidar_localization::Config config; // default configuration
    if (!sick_lidar_localization::API::getParams(node, config)) // overwrites configuration with arguments from commandline resp. launchfile
    {
        ROS_ERROR_STREAM("## ERROR sick_lidar_localization::API::getParams() failed, check launchfile, aborting... ");
        exit(EXIT_FAILURE);
    }
    ROS_INFO_STREAM("sick_lidar_localization hostname:=" << config.hostname << " serverpath:=" << config.serverpath << " verbose:=" << config.verbose << " started.");

    /*
    ** sick_lidar_localization: Install services and threads, run startup sequence
    */

    sick_lidar_localization::API lidar_loc_api;
    if (!lidar_loc_api.init(node, config))
    {
        ROS_ERROR_STREAM("## ERROR sick_lidar_localization::API::init() failed, aborting... ");
        exit(EXIT_FAILURE);
    }

    // Examples to send UDP messages to and to receive UDP messages from localication controller
    // (for native Linux or Windows, on ROS-1 and ROS-2 preferably done by the
    // corresponding ros-services and messages)
    if (false)
    {
        sendUDPMessageExamples(lidar_loc_api);
        static sick_lidar_localization::UDPMessage::InfoListener udp_receiver_info_listener;
        lidar_loc_api.registerListener(&udp_receiver_info_listener);
    }

    // Run event loop
#if __ROS_VERSION > 0
    rosSpin(node);
#elif defined WIN32 || defined _MSC_VER
    ROS_INFO_STREAM("sick_lidar_localization running. Press ENTER to exit...");
    getchar();
#else
    std::this_thread::sleep_for(std::chrono::seconds(std::numeric_limits<int>::max()));
#endif    

    // Exit and cleanup
    ROS_INFO_STREAM("sick_lidar_localization finished.");
    lidar_loc_api.close();

    return EXIT_SUCCESS;
}
