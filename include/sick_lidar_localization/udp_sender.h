/*
 * @brief udp_sender implements a UDP sender for UDP input messages.
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
#ifndef __SICK_LIDAR_LOCALIZATION_UDP_SENDER_H_INCLUDED
#define __SICK_LIDAR_LOCALIZATION_UDP_SENDER_H_INCLUDED

#include <string>
#include "sick_lidar_localization/udp_message_parser.h"

#if __ROS_VERSION == 1
#include "sick_lidar_localization/OdometryMessage0101.h"
#include "sick_lidar_localization/OdometryMessage0104.h"
#include "sick_lidar_localization/OdometryMessage0105.h"
#include "sick_lidar_localization/EncoderMeasurementMessage0202.h"
#include "sick_lidar_localization/CodeMeasurementMessage0303.h"
#include "sick_lidar_localization/LineMeasurementMessage0403.h"
#include "sick_lidar_localization/LineMeasurementMessage0404.h"
#elif __ROS_VERSION == 2
#include "sick_lidar_localization/msg/odometry_message0101.hpp"
#include "sick_lidar_localization/msg/odometry_message0104.hpp"
#include "sick_lidar_localization/msg/odometry_message0105.hpp"
#include "sick_lidar_localization/msg/encoder_measurement_message0202.hpp"
#include "sick_lidar_localization/msg/code_measurement_message0303.hpp"
#include "sick_lidar_localization/msg/line_measurement_message0403.hpp"
#include "sick_lidar_localization/msg/line_measurement_message0404.hpp"
namespace sick_lidar_localization { using namespace msg; }
#endif

namespace sick_lidar_localization
{
    /*
    ** @brief Forward declaration of udp sender implementation hiding the udp socket details
    */
    class UDPSenderImpl;

    /*
    ** @brief class UDPSender implements a UDP sender for UDP input messages.
    */
    class UDPSender
    {
    public:

        /*
        ** @brief Default constructor
        ** @param[in] nh ros node handle (always 0 for native linus or windows)
        ** @param[in] sim_ip_address UDP IP address, or "" for broadcast
        ** @param[in] udp_port_sim_input UDP port of UDP input messages, default: 5009
        ** @param[in] udp_sim_input_source_id source_id of UDP input messages, default: 1
        ** @param[in] verbose print informational messages if verbose > 0, otherwise silent mode (error messages only)
        ** @param[in] ros_odom_to_udp_msg  Convert ros odom message to upd: 0 = map velocity to OdometryPayload0101 (Type 1, Version 1, LidarLoc 1), 
        **                                 1 = map velocity to OdometryPayload0104 (Type 1, Version 4, LidarLoc 2),
        **                                 2 = map position to OdometryPayload0105 (Type 1, Version 5, LidarLoc 2),
        **                                 3 = map velocity to OdometryPayload0104 and position to OdometryPayload0105
        */
        UDPSender(rosNodePtr nh = 0, const std::string& sim_ip_address = "192.168.0.1", int udp_port_sim_input = 5009, int udp_sim_input_source_id = 1, int verbose = 0, 
            const std::string& odom_topic = "/odom", int ros_odom_to_udp_msg = 3);

        /*
        ** @brief Default destructor, exits running threads
        */
        ~UDPSender();

        /*
        ** @brief Sends a UDP input message. 
        **        payload can be OdometryPayload0104, OdometryPayload0105, EncoderMeasurementPayload0202, 
        **        CodeMeasurementPayload0303, LineMeasurementPayload0403 or LineMeasurementPayload0404
        ** @param[in] payload UDP message payload data
        ** @return true on success or false on error
        */
        template<typename T> bool sendUDPPayload(const T& payload, bool encode_header_big_endian, bool encode_payload_big_endian, uint16_t source_id)
        {
            std::vector<uint8_t> message = sick_lidar_localization::UDPMessage::encodeMessage(payload, encode_header_big_endian, encode_payload_big_endian, source_id);
            bool ok = sendData(message);
            if (!ok)
                ROS_ERROR_STREAM("## ERROR sick_lidar_localization::UDPSender::sendUDPPayload(" << m_sim_ip_address << ":" << m_udp_port_sim_input << ", payload=" << sick_lidar_localization::UDPMessage::printPayload(payload) << ", header_big_endian=" << encode_header_big_endian << ", payload_big_endian=" << encode_payload_big_endian << ", source_id=" << source_id << ") failed");
            else if (ok && m_verbose)
                ROS_INFO_STREAM("sick_lidar_localization::UDPSender: udp payload " << sick_lidar_localization::UDPMessage::printPayload(payload, false) << " sent to " << m_sim_ip_address << ":" << m_udp_port_sim_input);
            return ok;
        }

    protected:

        /*
        ** @brief (Re-)initializes the udp socket
        ** @return true on success or false on error
        */
        bool init(void);

        /*
        ** @brief Closes the udp socket
        */
        void close(void);

        /*
        ** @brief Send udp data
        ** @param[in] udp_data raw data bytes
        ** @return true on success or false on error
        */
        bool sendData(const std::vector<uint8_t>& udp_data);

        std::string m_sim_ip_address;        // UDP IP address, or "" for broadcast
        int m_udp_port_sim_input;            // UDP port of UDP input messages, default: 5009
        int m_source_id;                     // source_id of UDP input messages, default: 1
        int m_verbose;                       // Print informational messages if verbose > 0, otherwise silent mode (error messages only)
        int m_ros_odom_to_udp_msg;           // Convert ros odom message to upd
        UDPSenderImpl* m_udp_sender_impl;    // UDP sender implementation hiding the udp socket details

        #if __ROS_VERSION > 0
        /** subscriber callback functions for input udp messages */
        void messageCbOdomROS(const ros_nav_msgs::Odometry& msg);
        void messageCbOdomROS2(const std::shared_ptr<ros_nav_msgs::Odometry> msg) { messageCbOdomROS(*msg); }

        void messageCbOdometryMessage0101(const sick_lidar_localization::OdometryMessage0101 & msg);
        void messageCbOdometryMessage0101ROS2(const std::shared_ptr<sick_lidar_localization::OdometryMessage0101> msg) { messageCbOdometryMessage0101(*msg); }

        void messageCbOdometryMessage0104(const sick_lidar_localization::OdometryMessage0104 & msg);
        void messageCbOdometryMessage0104ROS2(const std::shared_ptr<sick_lidar_localization::OdometryMessage0104> msg) { messageCbOdometryMessage0104(*msg); }

        void messageCbOdometryMessage0105(const sick_lidar_localization::OdometryMessage0105 & msg);
        void messageCbOdometryMessage0105ROS2(const std::shared_ptr<sick_lidar_localization::OdometryMessage0105> msg) { messageCbOdometryMessage0105(*msg); }

        void messageCbEncoderMeasurementMessage0202(const sick_lidar_localization::EncoderMeasurementMessage0202 & msg);
        void messageCbEncoderMeasurementMessage0202ROS2(const std::shared_ptr<sick_lidar_localization::EncoderMeasurementMessage0202> msg) { messageCbEncoderMeasurementMessage0202(*msg); }

        void messageCbCodeMeasurementMessage0303(const sick_lidar_localization::CodeMeasurementMessage0303 & msg);
        void messageCbCodeMeasurementMessage0303ROS2(const std::shared_ptr<sick_lidar_localization::CodeMeasurementMessage0303> msg) { messageCbCodeMeasurementMessage0303(*msg); }

        void messageCbLineMeasurementMessage0403(const sick_lidar_localization::LineMeasurementMessage0403 & msg);
        void messageCbLineMeasurementMessage0403ROS2(const std::shared_ptr<sick_lidar_localization::LineMeasurementMessage0403> msg) { messageCbLineMeasurementMessage0403(*msg); }

        void messageCbLineMeasurementMessage0404(const sick_lidar_localization::LineMeasurementMessage0404 & msg);
        void messageCbLineMeasurementMessage0404ROS2(const std::shared_ptr<sick_lidar_localization::LineMeasurementMessage0404> msg) { messageCbLineMeasurementMessage0404(*msg); }

        /** subscriber for ros messages, converted to udp messages and send to localization server */
        rosSubscriber<ros_nav_msgs::Odometry> m_subOdomROS;
        rosSubscriber<sick_lidar_localization::OdometryMessage0101> m_subOdometryMessage0101;
        rosSubscriber<sick_lidar_localization::OdometryMessage0104> m_subOdometryMessage0104;
        rosSubscriber<sick_lidar_localization::OdometryMessage0105> m_subOdometryMessage0105;
        rosSubscriber<sick_lidar_localization::EncoderMeasurementMessage0202> m_subEncoderMeasurementMessage0202;
        rosSubscriber<sick_lidar_localization::CodeMeasurementMessage0303> m_subCodeMeasurementMessage0303;
        rosSubscriber<sick_lidar_localization::LineMeasurementMessage0403> m_subLineMeasurementMessage0403;
        rosSubscriber<sick_lidar_localization::LineMeasurementMessage0404> m_subLineMeasurementMessage0404;
        #endif

    }; // class UDPSender

} // namespace sick_lidar_localization
#endif // __SICK_LIDAR_LOCALIZATION_UDP_SENDER_H_INCLUDED
