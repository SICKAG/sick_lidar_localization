/*
 * @brief udp_message_publisher publishes output UDP messages on ROS-1 and ROS-2.
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
#ifndef __SICK_LIDAR_LOCALIZATION_UDP_MESSAGE_PUBLISHER_H_INCLUDED
#define __SICK_LIDAR_LOCALIZATION_UDP_MESSAGE_PUBLISHER_H_INCLUDED

#include "sick_lidar_localization/sick_common.h"
#include "sick_lidar_localization/udp_message_parser.h"

#if __ROS_VERSION == 1
#include "sick_lidar_localization/OdometryMessage0104.h"
#include "sick_lidar_localization/OdometryMessage0105.h"
#include "sick_lidar_localization/CodeMeasurementMessage0304.h"
#include "sick_lidar_localization/LineMeasurementMessage0403.h"
#include "sick_lidar_localization/LineMeasurementMessage0404.h"
#include "sick_lidar_localization/LocalizationControllerResultMessage0502.h"
#elif __ROS_VERSION == 2
#include "sick_lidar_localization/msg/odometry_message0104.hpp"
#include "sick_lidar_localization/msg/odometry_message0105.hpp"
#include "sick_lidar_localization/msg/code_measurement_message0304.hpp"
#include "sick_lidar_localization/msg/line_measurement_message0403.hpp"
#include "sick_lidar_localization/msg/line_measurement_message0404.hpp"
#include "sick_lidar_localization/msg/localization_controller_result_message0502.hpp"
namespace sick_lidar_localization { using namespace msg; }
#endif
namespace sick_lidar_localization
{
    /*
    ** @brief Implements a listener for udp messages and publishes udp messages.
    */
    class PublishUdpMessagesListener : public sick_lidar_localization::UDPMessage::Listener
    {
    public:
        /** default constructor, advertises ros messages for SIM udp messages */
        PublishUdpMessagesListener(rosNodePtr nh = 0, int verbose = 0) : m_verbose(verbose)
        {
            #if __ROS_VERSION > 0
            if(nh != 0)
            {
                m_pubOdometryMessage0104 = rosAdvertise<sick_lidar_localization::OdometryMessage0104>(nh, "/localizationcontroller/out/odometry_message_0104");
                m_pubOdometryMessage0105 = rosAdvertise<sick_lidar_localization::OdometryMessage0105>(nh, "/localizationcontroller/out/odometry_message_0105");
                m_pubCodeMeasurementMessage0304 = rosAdvertise<sick_lidar_localization::CodeMeasurementMessage0304>(nh, "/localizationcontroller/out/code_measurement_message_0304");
                m_pubLineMeasurementMessage0403 = rosAdvertise<sick_lidar_localization::LineMeasurementMessage0403>(nh, "/localizationcontroller/out/line_measurement_message_0403");
                m_pubLineMeasurementMessage0404 = rosAdvertise<sick_lidar_localization::LineMeasurementMessage0404>(nh, "/localizationcontroller/out/line_measurement_message_0404");
                m_pubLocalizationControllerResultMessage0502 = rosAdvertise<sick_lidar_localization::LocalizationControllerResultMessage0502>(nh, "/localizationcontroller/out/localizationcontroller_result_message_0502");
            }
            #endif // __ROS_VERSION > 0
        }

        /** default destructor */
        virtual ~PublishUdpMessagesListener(){}

        #if __ROS_VERSION > 0
        /** callback function for odometry messages type 1 version 4 */
        virtual void udpMessageReceived(const sick_lidar_localization::UDPMessage::OdometryPayload0104& message)
        { 
            printMessageReceived(message);
            sick_lidar_localization::OdometryMessage0104 ros_msg;
            ros_msg.header.stamp = rosTimeNow();
            ros_msg.telegram_count = message.telegram_count;
            ros_msg.timestamp = message.timestamp;
            ros_msg.x_velocity = message.x_velocity;
            ros_msg.y_velocity = message.y_velocity;
            ros_msg.angular_velocity = message.angular_velocity;
            ros_msg.sync_timestamp_sec = message.sync_timestamp_sec;
            ros_msg.sync_timestamp_nsec = message.sync_timestamp_nsec;
            ros_msg.sync_timestamp_valid = message.sync_timestamp_valid;
            rosPublish(m_pubOdometryMessage0104, ros_msg);
        };
        
        /** callback function for odometry messages type 1 version 5 */
        virtual void udpMessageReceived(const sick_lidar_localization::UDPMessage::OdometryPayload0105& message)
        { 
            printMessageReceived(message);
            sick_lidar_localization::OdometryMessage0105 ros_msg;
            ros_msg.header.stamp = rosTimeNow();
            ros_msg.telegram_count = message.telegram_count;
            ros_msg.timestamp = message.timestamp;
            ros_msg.x_position = message.x_position;
            ros_msg.y_position = message.y_position;
            ros_msg.heading = message.heading;
            ros_msg.sync_timestamp_sec = message.sync_timestamp_sec;
            ros_msg.sync_timestamp_nsec = message.sync_timestamp_nsec;
            ros_msg.sync_timestamp_valid = message.sync_timestamp_valid;
            rosPublish(m_pubOdometryMessage0105, ros_msg);
        };
        
        /** callback function for code measurement messages type 3 version 4 */
        virtual void udpMessageReceived(const sick_lidar_localization::UDPMessage::CodeMeasurementPayload0304& message)
        { 
            printMessageReceived(message);
            sick_lidar_localization::CodeMeasurementMessage0304 ros_msg;
            ros_msg.header.stamp = rosTimeNow();
            ros_msg.telegram_count = message.telegram_count;
            ros_msg.timestamp = message.timestamp;
            ros_msg.code = message.code;
            ros_msg.distance = message.distance;
            ros_msg.sync_timestamp_sec = message.sync_timestamp_sec;
            ros_msg.sync_timestamp_nsec = message.sync_timestamp_nsec;
            ros_msg.sync_timestamp_valid = message.sync_timestamp_valid;
            rosPublish(m_pubCodeMeasurementMessage0304, ros_msg);
        };
        
        /** callback function for line measurement messages type 4 version 3 */
        virtual void udpMessageReceived(const sick_lidar_localization::UDPMessage::LineMeasurementPayload0403& message)
        { 
            printMessageReceived(message);
            sick_lidar_localization::LineMeasurementMessage0403 ros_msg;
            ros_msg.header.stamp = rosTimeNow();
            ros_msg.telegram_count = message.telegram_count;
            ros_msg.timestamp = message.timestamp;
            ros_msg.num_lanes = message.num_lanes;
            ros_msg.lanes = message.lanes;
            ros_msg.sync_timestamp_sec = message.sync_timestamp_sec;
            ros_msg.sync_timestamp_nsec = message.sync_timestamp_nsec;
            ros_msg.sync_timestamp_valid = message.sync_timestamp_valid;
            rosPublish(m_pubLineMeasurementMessage0403, ros_msg);
        };
        
        /** callback function for line measurement messages type 4 version 4 */
        virtual void udpMessageReceived(const sick_lidar_localization::UDPMessage::LineMeasurementPayload0404& message)
        { 
            printMessageReceived(message);
            sick_lidar_localization::LineMeasurementMessage0404 ros_msg;
            ros_msg.header.stamp = rosTimeNow();
            ros_msg.telegram_count = message.telegram_count;
            ros_msg.timestamp = message.timestamp;
            ros_msg.lcp1 = message.lcp1;
            ros_msg.lcp2 = message.lcp2;
            ros_msg.lcp3 = message.lcp3;
            ros_msg.cnt_lpc = message.cnt_lpc;
            ros_msg.sync_timestamp_sec = message.sync_timestamp_sec;
            ros_msg.sync_timestamp_nsec = message.sync_timestamp_nsec;
            ros_msg.sync_timestamp_valid = message.sync_timestamp_valid;
            rosPublish(m_pubLineMeasurementMessage0404, ros_msg);
        };
    
        /** callback function for localizationcontroller result messages type 5 version 2 */
        virtual void udpMessageReceived(const sick_lidar_localization::UDPMessage::LocalizationControllerResultPayload0502& message)
        { 
            printMessageReceived(message);
            sick_lidar_localization::LocalizationControllerResultMessage0502 ros_msg;
            ros_msg.header.stamp = rosTimeNow();
            ros_msg.telegram_count = message.telegram_count;
            ros_msg.timestamp = message.timestamp;
            ros_msg.x = message.x;
            ros_msg.y = message.y;
            ros_msg.heading = message.heading;
            ros_msg.loc_status = message.loc_status;
            ros_msg.map_match_status = message.map_match_status;
            ros_msg.sync_timestamp_sec = message.sync_timestamp_sec;
            ros_msg.sync_timestamp_nsec = message.sync_timestamp_nsec;
            ros_msg.sync_timestamp_valid = message.sync_timestamp_valid;
            rosPublish(m_pubLocalizationControllerResultMessage0502, ros_msg);
        };
        #endif // __ROS_VERSION > 0

    protected:

        /** just prints a udp message in a human readable string */
        template<typename T> void printMessageReceived(const T& message)
        {
            if(m_verbose)
            {
                sick_lidar_localization::UDPMessage::Payload<T> payload(message);
                ROS_INFO_STREAM("PublishUdpMessagesListener::udpMessageReceived: " << payload.toString());
            }
        };

        #if __ROS_VERSION > 0
        rosPublisher<sick_lidar_localization::OdometryMessage0104> m_pubOdometryMessage0104;
        rosPublisher<sick_lidar_localization::OdometryMessage0105> m_pubOdometryMessage0105;
        rosPublisher<sick_lidar_localization::CodeMeasurementMessage0304> m_pubCodeMeasurementMessage0304;
        rosPublisher<sick_lidar_localization::LineMeasurementMessage0403> m_pubLineMeasurementMessage0403;
        rosPublisher<sick_lidar_localization::LineMeasurementMessage0404> m_pubLineMeasurementMessage0404;
        rosPublisher<sick_lidar_localization::LocalizationControllerResultMessage0502> m_pubLocalizationControllerResultMessage0502;
        #endif // __ROS_VERSION > 0
        int m_verbose;
    };

}   // namespace sick_lidar_localization
#endif // __SICK_LIDAR_LOCALIZATION_UDP_MESSAGE_PUBLISHER_H_INCLUDED
