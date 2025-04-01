/*
 * @brief sick_lidar_localization implements the interface for sick_lidar_localization.
 *        All functions for external usage are provided by class sick_lidar_localization::API.
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
#ifndef __SICK_LIDAR_LOCALIZATION_API_INCLUDED
#define __SICK_LIDAR_LOCALIZATION_API_INCLUDED

#include "sick_lidar_localization/sick_common.h"
#include "sick_lidar_localization/message_parser.h"

namespace sick_lidar_localization
{

    /*
    ** Container for configuration parameter and default values of sick_lidar_localization
    ** Default values can be overwritten by launchfile
    */
    class Config
    {
    public:
        std::string lls_device_ip = "192.168.0.1";  // IP address of the localization device
        int lls_device_udp_port = 5009;             // port of LLS device
        int lls_device_webserver_port = 80;         // Port of the rest api, i.e. url of rest requests is "http://<lls_device_ip>:<lls_device_webserver_port>/api/
        int verbose = 0;                            // If verbose>0: print informational messages, otherwise silent except for error messages
        std::string ros_machine_ip = "";            // IP address of local machine or "" for Broadcast
        int ros_machine_udp_port = 5010;            // Port of of local machine 
        int src_id_odometry_msg_type_1 = 1;         // Source_id of vehicle odometry messages
        std::string udp_output_logfile = "";        // Optional logfile for human readable output messages, default: "" (no outputlogfile)
        int software_pll_fifo_length = 7;           // Length of fifo in SoftwarePLL
        std::string odom_topic = "/odom";           // Topic of ros odom messages
        int ros_odom_to_udp_msg = 0;                    // Convert ros odom message to:
                                                        // 1 = map velocity to OdometryPayload0104 (Type 1, Version 4, LidarLoc 2),
                                                        // 2 = map position to OdometryPayload0105 (Type 1, Version 5, LidarLoc 2),
                                                        // 3 = map velocity to OdometryPayload0104 and position to OdometryPayload0105
    };

    class CurlWrapper;
    class SickServices;
    class UDPReceiverThread;
    class PublishMessagesListener;
    class UDPSender;

    /*
    ** @brief class sick_lidar_localization::API implements the interface for sick_lidar_localization
    **        and provides functions for external usage
    */
    class API
    {
    public:

        /*
        ** @brief Default constructor
        */
        API();

        /*
        ** @brief Default destructor
        */
        ~API();

        /*
        ** @brief Initialization, installs services and threads, runs startup sequence
        */
        bool init(rosNodePtr node, sick_lidar_localization::Config& config);

        /*
        ** @brief Stops all services and threads
        */
        void close();

        /*
        ** @brief Register a listener for messages. The callback functions of the listener will be called after receiving a new message.
        ** Overwrite the functions defined in sick_lidar_localization::Message::Listener with customized code to handle messages.
        */
        bool registerListener(sick_lidar_localization::Message::Listener* listener);

        /*
        ** @brief Unregister a listener. Removes the listener from notifications after receiving messages.
        */
        bool unregisterListener(sick_lidar_localization::Message::Listener* listener);

        /*
        ** @brief Send messages to the LLS device
        */
        bool sendMessageUDP(const sick_lidar_localization::Message::OdometryPayload0104& payload, bool encode_header_big_endian, bool encode_payload_big_endian);
        bool sendMessageUDP(const sick_lidar_localization::Message::OdometryPayload0105& payload, bool encode_header_big_endian, bool encode_payload_big_endian);
        bool sendMessageUDP(const sick_lidar_localization::Message::EncoderMeasurementPayload0202& payload, bool encode_header_big_endian, bool encode_payload_big_endian);
        bool sendMessageUDP(const sick_lidar_localization::Message::CodeMeasurementPayload0303& payload, bool encode_header_big_endian, bool encode_payload_big_endian);
        bool sendMessageUDP(const sick_lidar_localization::Message::CodeMeasurementPayload0701& payload, bool encode_header_big_endian, bool encode_payload_big_endian);
        bool sendMessageUDP(const sick_lidar_localization::Message::LineMeasurementPayload0403& payload, bool encode_header_big_endian, bool encode_payload_big_endian);
        bool sendMessageUDP(const sick_lidar_localization::Message::LineMeasurementPayload0404& payload, bool encode_header_big_endian, bool encode_payload_big_endian);

        /*
        ** @brief Parses commandline arguments, reads a launchfile and sets parameters for sick_lidar_localization.
        */
        static bool parseLaunchfileSetParameter(rosNodePtr node, int argc, char** argv);

        /*
        ** @brief Queries and returns the current configuration
        */
        static bool getParams(rosNodePtr node, sick_lidar_localization::Config& config);

    protected:

        rosNodePtr m_node;
        sick_lidar_localization::Config m_config;
        sick_lidar_localization::CurlWrapper* m_curl;
        sick_lidar_localization::SickServices* m_services;
        sick_lidar_localization::UDPReceiverThread* m_receiver_thread;
        sick_lidar_localization::Message::InfoListener* m_receiver_listener;
        sick_lidar_localization::PublishMessagesListener* m_message_publisher;
        sick_lidar_localization::UDPSender* m_sender;
    }; // class sick_lidar_localization::API

} // namespace sick_lidar_localization
#endif // __SICK_LIDAR_LOCALIZATION_API_INCLUDED
  