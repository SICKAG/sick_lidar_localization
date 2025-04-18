/*
 * @brief sender implements a sender for input messages.
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
#include <string>
#if defined WIN32 || defined _MSC_VER
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <winsock2.h>
static std::string getErrorMessage(void)
{
    int error_num = WSAGetLastError();
    char error_message[1024] = { 0 };
    FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL, error_num, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), error_message, sizeof(error_message), NULL);
    return std::to_string(error_num) + " (" + std::string(error_message) + ")";
}
#else
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <sys/types.h>
#include <sys/socket.h>
typedef int SOCKET;
typedef struct sockaddr SOCKADDR;
#define INVALID_SOCKET (-1)
#define closesocket close
static std::string getErrorMessage(void) { return std::to_string(errno) + " (" + std::string(strerror(errno)) + ")"; }
#endif

#include "sick_lidar_localization/udp_sender.h"

namespace sick_lidar_localization
{
    /*
    ** @brief class SenderImpl implements an sender implementation hiding the socket details
    */
    class SenderImpl
    {
    public:

        /*
        ** @brief Default constructor
        */
        SenderImpl() : m_socket(INVALID_SOCKET), m_lls_ip_address(), m_port_lls_input(0)
        {
        }

        /*
        ** @brief Initializes the socket
        ** @param[in] lls_ip_address IP address of LLS device, or "" for broadcast
        ** @param[in] port_lls_input port of input messages, default: 5009
        */
        bool initSocket(const std::string & lls_ip_address = "192.168.0.1", int port_lls_input = 5009)
        {
            m_lls_ip_address = lls_ip_address;
            m_port_lls_input = port_lls_input;
            if ((m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == INVALID_SOCKET)
            {
                ROS_ERROR_STREAM("## ERROR SenderImpl::init(" << port_lls_input << "): can't create socket");
                return false;
            }
            #if defined WIN32 || defined _MSC_VER
            char broadcast_opt = 1;
            #else
            int broadcast_opt = 1;
            #endif
            if (setsockopt(m_socket, SOL_SOCKET, SO_BROADCAST, &broadcast_opt, sizeof(broadcast_opt)) < 0)
                ROS_ERROR_STREAM("## ERROR SenderImpl::init(" << port_lls_input << "): setsockopt(SO_BROADCAST) failed, error: " << getErrorMessage());
            return true;
        }

        /*
        ** @brief Closes the socket
        */
        void closeSocket(void)
        {
            if (m_socket != INVALID_SOCKET)
                closesocket(m_socket);
            m_socket = INVALID_SOCKET;
        }

        bool sendData(const uint8_t* buffer, int num_bytes)
        {
            if (m_socket != INVALID_SOCKET)
            {
                struct sockaddr_in lls_servaddr = { 0 };
                if(m_lls_ip_address.empty())
                {
                    lls_servaddr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
                }
                else
                {
                    #if defined WIN32 || defined _MSC_VER
                    lls_servaddr.sin_addr.s_addr = inet_addr(m_lls_ip_address.c_str());
                    #else
                    struct in_addr lls_in_addr;
                    if (inet_aton(m_lls_ip_address.c_str(), &lls_in_addr) != 0)
                    {
                        lls_servaddr.sin_addr.s_addr = lls_in_addr.s_addr;
                    }
                    else
                    {
                        ROS_ERROR_STREAM("## ERROR SenderImpl::sendData(): inet_aton(" << m_lls_ip_address << ") failed (invalid address)");
                        lls_servaddr.sin_addr.s_addr = inet_addr(m_lls_ip_address.c_str());
                    }
                    #endif
                }
                lls_servaddr.sin_family = AF_INET;
                lls_servaddr.sin_port = htons(m_port_lls_input);
                int bytes_send = sendto(m_socket, (const char*)buffer, num_bytes, 0, (SOCKADDR*)&lls_servaddr, sizeof(lls_servaddr));
                if (bytes_send != num_bytes)
                {
                    ROS_ERROR_STREAM("## ERROR SenderImpl::sendData(): " << bytes_send << " of " << num_bytes << " bytes send to " << m_lls_ip_address << ":" << m_port_lls_input << ", error: " << getErrorMessage());
                    return false;
                }
                return true;
            }
            return false;
        }

    protected:

        SOCKET m_socket; // OS socket descriptor
        std::string m_lls_ip_address; // IP address of LLS device, or "" for broadcast
        int m_port_lls_input; // port of input messages, default: 5009
    };
}

/*
** @brief Default constructor
** @param[in] nh ros node handle
** @param[in] lls_ip_address IP address of localization device, or "" for broadcast
** @param[in] port_lls_input port of input messages, default: 5009
** @param[in] source_id_vehicle source_id map of input messages, default source id: 1
** @param[in] verbose print informational messages if verbose > 0, otherwise silent mode (error messages only)
** @param[in] ros_odom_to_udp_msg  Convert ros odom message:
**                                 1 = map velocity to OdometryPayload0104 (Type 1, Version 4, LidarLoc 2),
**                                 2 = map position to OdometryPayload0105 (Type 1, Version 5, LidarLoc 2),
**                                 3 = map velocity to OdometryPayload0104 and position to OdometryPayload0105
*/
sick_lidar_localization::UDPSender::UDPSender(rosNodePtr nh, const std::string& lls_ip_address, int port_lls_input, const int& source_id_vehicle, int verbose, const std::string& odom_topic, int ros_odom_to_udp_msg) 
: m_lls_ip_address(lls_ip_address), m_port_lls_input(port_lls_input), m_source_id_vehicle(source_id_vehicle), m_verbose(verbose), m_ros_odom_to_udp_msg(ros_odom_to_udp_msg), m_sender_impl(0)
{
    #if __ROS_VERSION > 0
    if(nh != 0)
    {
        // Subscribe to odometry messages
        #if __ROS_VERSION == 1
        auto messageCbOdomROS = &sick_lidar_localization::UDPSender::messageCbOdomROS;
        auto messageCbOdometryMessage0104 = &sick_lidar_localization::UDPSender::messageCbOdometryMessage0104;
        auto messageCbOdometryMessage0105 = &sick_lidar_localization::UDPSender::messageCbOdometryMessage0105;
        auto messageCbEncoderMeasurementMessage0202 = &sick_lidar_localization::UDPSender::messageCbEncoderMeasurementMessage0202;
        auto messageCbCodeMeasurementMessage0303 = &sick_lidar_localization::UDPSender::messageCbCodeMeasurementMessage0303;
        auto messageCbCodeMeasurementMessage0701 = &sick_lidar_localization::UDPSender::messageCbCodeMeasurementMessage0701;
        auto messageCbLineMeasurementMessage0403 = &sick_lidar_localization::UDPSender::messageCbLineMeasurementMessage0403;
        auto messageCbLineMeasurementMessage0404 = &sick_lidar_localization::UDPSender::messageCbLineMeasurementMessage0404;
        #else
        auto messageCbOdomROS = &sick_lidar_localization::UDPSender::messageCbOdomROS2;
        auto messageCbOdometryMessage0104 = &sick_lidar_localization::UDPSender::messageCbOdometryMessage0104ROS2;
        auto messageCbOdometryMessage0105 = &sick_lidar_localization::UDPSender::messageCbOdometryMessage0105ROS2;
        auto messageCbEncoderMeasurementMessage0202 = &sick_lidar_localization::UDPSender::messageCbEncoderMeasurementMessage0202ROS2;
        auto messageCbCodeMeasurementMessage0303 = &sick_lidar_localization::UDPSender::messageCbCodeMeasurementMessage0303ROS2;
        auto messageCbCodeMeasurementMessage0701 = &sick_lidar_localization::UDPSender::messageCbCodeMeasurementMessage0701ROS2;
        auto messageCbLineMeasurementMessage0403 = &sick_lidar_localization::UDPSender::messageCbLineMeasurementMessage0403ROS2;
        auto messageCbLineMeasurementMessage0404 = &sick_lidar_localization::UDPSender::messageCbLineMeasurementMessage0404ROS2;
        #endif

# ifdef _MSC_VER
        m_subOdomROS = rosSubscriber<ros_nav_msgs::Odometry>(nh->create_subscription<ros_nav_msgs::Odometry>(odom_topic, 10, std::bind(&sick_lidar_localization::UDPSender::messageCbOdomROS2, this, std::placeholders::_1)));
        m_subOdometryMessage0104 = rosSubscriber<sick_lidar_localization_msgs::OdometryMessage0104>(nh->create_subscription<sick_lidar_localization_msgs::OdometryMessage0104>("/localizationcontroller/in/odometry_message_0104", 10, std::bind(&sick_lidar_localization::UDPSender::messageCbOdometryMessage0104ROS2, this, std::placeholders::_1)));
        m_subOdometryMessage0105 = rosSubscriber<sick_lidar_localization_msgs::OdometryMessage0105>(nh->create_subscription<sick_lidar_localization_msgs::OdometryMessage0105>("/localizationcontroller/in/odometry_message_0105", 10, std::bind(&sick_lidar_localization::UDPSender::messageCbOdometryMessage0105ROS2, this, std::placeholders::_1)));
        m_subEncoderMeasurementMessage0202 = rosSubscriber<sick_lidar_localization_msgs::EncoderMeasurementMessage0202>(nh->create_subscription<sick_lidar_localization_msgs::EncoderMeasurementMessage0202>("/localizationcontroller/in/encoder_measurement_message_0202", 10, std::bind(&sick_lidar_localization::UDPSender::messageCbEncoderMeasurementMessage0202ROS2, this, std::placeholders::_1)));
        m_subCodeMeasurementMessage0303 = rosSubscriber<sick_lidar_localization_msgs::CodeMeasurementMessage0303>(nh->create_subscription<sick_lidar_localization_msgs::CodeMeasurementMessage0303>("/localizationcontroller/in/code_measurement_message_0303", 10, std::bind(&sick_lidar_localization::UDPSender::messageCbCodeMeasurementMessage0303ROS2, this, std::placeholders::_1)));
        m_subCodeMeasurementMessage0701 = rosSubscriber<sick_lidar_localization_msgs::CodeMeasurementMessage0701>(nh->create_subscription<sick_lidar_localization_msgs::CodeMeasurementMessage0701>("/localizationcontroller/in/code_measurement_message_0701", 10, std::bind(&sick_lidar_localization::UDPSender::messageCbCodeMeasurementMessage0701ROS2, this, std::placeholders::_1)));
        m_subLineMeasurementMessage0403 = rosSubscriber<sick_lidar_localization_msgs::LineMeasurementMessage0403>(nh->create_subscription<sick_lidar_localization_msgs::LineMeasurementMessage0403>("/localizationcontroller/in/line_measurement_message_0403", 10, std::bind(&sick_lidar_localization::UDPSender::messageCbLineMeasurementMessage0403ROS2, this, std::placeholders::_1)));
        m_subLineMeasurementMessage0404 = rosSubscriber<sick_lidar_localization_msgs::LineMeasurementMessage0404>(nh->create_subscription<sick_lidar_localization_msgs::LineMeasurementMessage0404>("/localizationcontroller/in/line_measurement_message_0404", 10, std::bind(&sick_lidar_localization::UDPSender::messageCbLineMeasurementMessage0404ROS2, this, std::placeholders::_1)));
#else
        m_subOdomROS = rosSubscribe<ros_nav_msgs::Odometry>(nh, odom_topic, messageCbOdomROS, this);
        m_subOdometryMessage0104 = rosSubscribe<sick_lidar_localization_msgs::OdometryMessage0104>(nh, "/localizationcontroller/in/odometry_message_0104", messageCbOdometryMessage0104, this);
        m_subOdometryMessage0105 = rosSubscribe<sick_lidar_localization_msgs::OdometryMessage0105>(nh, "/localizationcontroller/in/odometry_message_0105", messageCbOdometryMessage0105, this);
        m_subEncoderMeasurementMessage0202 = rosSubscribe<sick_lidar_localization_msgs::EncoderMeasurementMessage0202>(nh, "/localizationcontroller/in/encoder_measurement_message_0202", messageCbEncoderMeasurementMessage0202, this);
        m_subCodeMeasurementMessage0303 = rosSubscribe<sick_lidar_localization_msgs::CodeMeasurementMessage0303>(nh, "/localizationcontroller/in/code_measurement_message_0303", messageCbCodeMeasurementMessage0303, this);
        m_subCodeMeasurementMessage0701 = rosSubscribe<sick_lidar_localization_msgs::CodeMeasurementMessage0701>(nh, "/localizationcontroller/in/code_measurement_message_0701", messageCbCodeMeasurementMessage0701, this);
        m_subLineMeasurementMessage0403 = rosSubscribe<sick_lidar_localization_msgs::LineMeasurementMessage0403>(nh, "/localizationcontroller/in/line_measurement_message_0403", messageCbLineMeasurementMessage0403, this);
        m_subLineMeasurementMessage0404 = rosSubscribe<sick_lidar_localization_msgs::LineMeasurementMessage0404>(nh, "/localizationcontroller/in/line_measurement_message_0404", messageCbLineMeasurementMessage0404, this);
#endif
    }
    #endif
    init();
}

/*
** @brief Default destructor, exits running threads
*/
sick_lidar_localization::UDPSender::~UDPSender()
{
    close();
}

/*
** @brief (Re-)initializes the socket
** @return true on success or false on error
*/
bool sick_lidar_localization::UDPSender::init(void)
{
    if (m_sender_impl)
    {
        close();
    }
    m_sender_impl = new sick_lidar_localization::SenderImpl();
    if (!m_sender_impl->initSocket(m_lls_ip_address, m_port_lls_input))
    {
        delete(m_sender_impl);
        m_sender_impl = 0;
        ROS_ERROR_STREAM("## ERROR UDPSender::init(" << m_lls_ip_address << ", " << m_port_lls_input << ") failed");
        return false;
    }
    return true;
}

/*
** @brief Closes the socket
*/
void sick_lidar_localization::UDPSender::close(void)
{
    if (m_sender_impl)
    {
        m_sender_impl->closeSocket();
        delete(m_sender_impl);
        m_sender_impl = 0;
    }
}

/*
** @brief Send data
** @param[in] data raw data bytes
** @return true on success or false on error
*/
bool sick_lidar_localization::UDPSender::sendData(const std::vector<uint8_t>& data)
{
    if (!m_sender_impl)
    {
        init();
    }
    if (!m_sender_impl)
    {
        ROS_ERROR_STREAM("## ERROR UDPSender: can't init socket, UDPSender(" << m_lls_ip_address << ", " << m_port_lls_input << ") failed");
        return false;
    }
    bool ok = m_sender_impl->sendData(data.data(), (int)data.size());
    if (!ok)
    {
        ROS_ERROR_STREAM("## ERROR UDPSender: send data failed, retrying...");
        init();
        if(m_sender_impl)
            ok = m_sender_impl->sendData(data.data(), (int)data.size());
        if(ok)
            ROS_INFO_STREAM("UDPSender: reinit and send successfull");
        else
            ROS_ERROR_STREAM("## ERROR UDPSender: reinit and send failed");
    }
    return ok;
}

/** Convert quaternion (w, x, y, z) to yaw angle (z-axis rotation) in radians */
static double quaternionToYawAngle(double w, double x, double y, double z)
{
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

#if __ROS_VERSION > 0

/** subscriber callback function for ros odom messages */
void sick_lidar_localization::UDPSender::messageCbOdomROS(const ros_nav_msgs::Odometry& msg)
{
    // Get timestamp, linear and angular velocity from odom message
    rosTime timestamp_msg = msg.header.stamp;
    uint64_t timestamp_sec = sec(timestamp_msg);
    uint64_t timestamp_nsec = nsec(timestamp_msg);
    int64_t timestamp_mircrosec = 1000000 * timestamp_sec + timestamp_nsec / 1000;// message timestamp in microseconds
    double vx_ms = msg.twist.twist.linear.x;     // vx in m/s
    double vy_ms = msg.twist.twist.linear.y;     // vy in m/s
    double omega_rs = msg.twist.twist.angular.z; // omega in rad/s
    int64_t x_pos_mm = (int64_t)std::round(1000.0 * msg.pose.pose.position.x);     // x position in mm
    int64_t y_pos_mm = (int64_t)std::round(1000.0 * msg.pose.pose.position.y);     // y position in mm
    double yaw = quaternionToYawAngle(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    int64_t heading_mdeg = (int64_t)std::round(1000.0 * 180.0 * yaw / M_PI);       // heading in mdeg
    int32_t vx_mms = (int32_t)std::round(1000.0 * vx_ms);                          // x-component of velocity in mm/s
    int32_t vy_mms = (int32_t)std::round(1000.0 * vy_ms);                          // y-component of velocity in mm/s
    int64_t omega_mdegs = (int64_t)std::round(1000.0 * 180.0 * omega_rs / M_PI);   // angular velocity in mdeg/s

    static uint64_t s_telegram_count = 0;
    s_telegram_count++;
    // Convert to OdometryPayload0104 message
    if(m_ros_odom_to_udp_msg == 1 || m_ros_odom_to_udp_msg == 3)
    {
        sick_lidar_localization::Message::OdometryPayload0104 odometry0104;
        odometry0104.telegram_count = s_telegram_count;
        odometry0104.timestamp = timestamp_mircrosec;
        odometry0104.x_velocity = vx_mms;
        odometry0104.y_velocity = vy_mms;
        odometry0104.angular_velocity = omega_mdegs;
        odometry0104.source_id = m_source_id_vehicle;
        // Send OdometryMessage0104
        if (m_verbose)
            ROS_INFO_STREAM("sick_lidar_localization::UDPSender: sending odom message (vx=" << vx_ms << ", vy=" << vy_ms << ", omega=" << omega_rs << ") converted to OdometryPayload0104");
        if (!sendPayload(odometry0104, HEADER_BIG_ENDIAN_DEFAULT, PAYLOAD_BIG_ENDIAN_DEFAULT))
            ROS_ERROR_STREAM("## ERROR sick_lidar_localization::UDPSender::messageCbOdometryMessage0104(): sendPayload(" << sick_lidar_localization::Message::printPayload(odometry0104) << ") failed");
    }
    // Convert to OdometryPayload0105 message
    if(m_ros_odom_to_udp_msg == 2 || m_ros_odom_to_udp_msg == 3)
    {
        sick_lidar_localization::Message::OdometryPayload0105 odometry0105;
        odometry0105.telegram_count = s_telegram_count;
        odometry0105.timestamp = timestamp_mircrosec;
        odometry0105.x_position = x_pos_mm;
        odometry0105.y_position = y_pos_mm;
        odometry0105.heading = heading_mdeg;
        odometry0105.source_id = m_source_id_vehicle;
        // Send OdometryMessage0105
        if (m_verbose)
            ROS_INFO_STREAM("sick_lidar_localization::UDPSender: sending odom message (x=" << x_pos_mm << ", y=" << y_pos_mm << ", heading=" << heading_mdeg << ") converted to OdometryPayload0105");
        if (!sendPayload(odometry0105, HEADER_BIG_ENDIAN_DEFAULT, PAYLOAD_BIG_ENDIAN_DEFAULT))
            ROS_ERROR_STREAM("## ERROR sick_lidar_localization::UDPSender::OdometryPayload0105(): sendPayload(" << sick_lidar_localization::Message::printPayload(odometry0105) << ") failed");
    }
    if(m_ros_odom_to_udp_msg < 0 || m_ros_odom_to_udp_msg > 3)
    {
        ROS_ERROR_STREAM("## ERROR sick_lidar_localization::UDPSender::messageCbOdomROS(): ros_odom_to_udp_msg = " << m_ros_odom_to_udp_msg << " not supported");
    }
}

/** subscriber callback function for input messages type OdometryMessage0104 */
void sick_lidar_localization::UDPSender::messageCbOdometryMessage0104(const sick_lidar_localization_msgs::OdometryMessage0104 & msg)
{
    sick_lidar_localization::Message::OdometryPayload0104 odometry0104;
    odometry0104.telegram_count = msg.telegram_count;
    odometry0104.timestamp = msg.timestamp;
    odometry0104.x_velocity = msg.x_velocity;
    odometry0104.y_velocity = msg.y_velocity;
    odometry0104.angular_velocity = msg.angular_velocity;
    odometry0104.source_id = msg.source_id;
    if (!sendPayload(odometry0104, HEADER_BIG_ENDIAN_DEFAULT, PAYLOAD_BIG_ENDIAN_DEFAULT))
        ROS_ERROR_STREAM("## ERROR sick_lidar_localization::UDPSender::messageCbOdometryMessage0104(): sendPayload(" << sick_lidar_localization::Message::printPayload(odometry0104) << ") failed");
} 

/** subscriber callback function for input messages type messageCbOdometryMessage0105 */
void sick_lidar_localization::UDPSender::messageCbOdometryMessage0105(const sick_lidar_localization_msgs::OdometryMessage0105 & msg)
{
    sick_lidar_localization::Message::OdometryPayload0105 odometry0105;
    odometry0105.telegram_count = msg.telegram_count;
    odometry0105.timestamp = msg.timestamp;
    odometry0105.x_position = msg.x_position;
    odometry0105.y_position = msg.y_position;
    odometry0105.heading = msg.heading;
    odometry0105.source_id = msg.source_id;
    if (!sendPayload(odometry0105, HEADER_BIG_ENDIAN_DEFAULT, PAYLOAD_BIG_ENDIAN_DEFAULT))
        ROS_ERROR_STREAM("## ERROR sick_lidar_localization::UDPSender::OdometryPayload0105(): sendPayload(" << sick_lidar_localization::Message::printPayload(odometry0105) << ") failed");
} 

/** subscriber callback function for input messages type messageCbEncoderMeasurementMessage0202 */
void sick_lidar_localization::UDPSender::messageCbEncoderMeasurementMessage0202(const sick_lidar_localization_msgs::EncoderMeasurementMessage0202 & msg)
{
    sick_lidar_localization::Message::EncoderMeasurementPayload0202 encoder_measurement0202;
    encoder_measurement0202.telegram_count = msg.telegram_count;
    encoder_measurement0202.timestamp = msg.timestamp;
    encoder_measurement0202.encoder_value = msg.encoder_value;
    encoder_measurement0202.source_id = msg.source_id;
    if (!sendPayload(encoder_measurement0202, HEADER_BIG_ENDIAN_DEFAULT, PAYLOAD_BIG_ENDIAN_DEFAULT))
        ROS_ERROR_STREAM("## ERROR sick_lidar_localization::UDPSender::messageCbEncoderMeasurementMessage0202(): sendPayload(" << sick_lidar_localization::Message::printPayload(encoder_measurement0202) << ") failed");
} 

/** subscriber callback function for input messages type messageCbCodeMeasurementMessage0303 */
void sick_lidar_localization::UDPSender::messageCbCodeMeasurementMessage0303(const sick_lidar_localization_msgs::CodeMeasurementMessage0303 & msg)
{
    sick_lidar_localization::Message::CodeMeasurementPayload0303 code_measurement0303;
    code_measurement0303.telegram_count = msg.telegram_count;
    code_measurement0303.timestamp = msg.timestamp;
    code_measurement0303.code = msg.code;
    code_measurement0303.source_id = msg.source_id;
    if (!sendPayload(code_measurement0303, HEADER_BIG_ENDIAN_DEFAULT, PAYLOAD_BIG_ENDIAN_DEFAULT))
        ROS_ERROR_STREAM("## ERROR sick_lidar_localization::UDPSender::messageCbCodeMeasurementMessage0303(): sendPayload(" << sick_lidar_localization::Message::printPayload(code_measurement0303) << ") failed");
} 

/** subscriber callback function for input messages type messageCbCodeMeasurementMessage0701 */
void sick_lidar_localization::UDPSender::messageCbCodeMeasurementMessage0701(const sick_lidar_localization_msgs::CodeMeasurementMessage0701 & msg)
{
    sick_lidar_localization::Message::CodeMeasurementPayload0701 code_measurement0701;
    code_measurement0701.telegram_count = msg.telegram_count;
    code_measurement0701.timestamp = msg.timestamp;
    code_measurement0701.code = msg.code;
    code_measurement0701.x_position = msg.x_position;
    code_measurement0701.y_position = msg.y_position;
    code_measurement0701.heading = msg.heading;
    code_measurement0701.source_id = msg.source_id;
    if (!sendPayload(code_measurement0701, HEADER_BIG_ENDIAN_DEFAULT, PAYLOAD_BIG_ENDIAN_DEFAULT))
        ROS_ERROR_STREAM("## ERROR sick_lidar_localization::UDPSender::messageCbCodeMeasurementMessage0701(): sendPayload(" << sick_lidar_localization::Message::printPayload(code_measurement0701) << ") failed");
} 

/** subscriber callback function for input messages type messageCbLineMeasurementMessage0403 */
void sick_lidar_localization::UDPSender::messageCbLineMeasurementMessage0403(const sick_lidar_localization_msgs::LineMeasurementMessage0403 & msg)
{
    sick_lidar_localization::Message::LineMeasurementPayload0403 line_measurement0403;
    line_measurement0403.telegram_count = msg.telegram_count;
    line_measurement0403.timestamp = msg.timestamp;
    line_measurement0403.num_lanes = msg.num_lanes;
    line_measurement0403.lanes = msg.lanes;
    line_measurement0403.source_id = msg.source_id;
    if (!sendPayload(line_measurement0403, HEADER_BIG_ENDIAN_DEFAULT, PAYLOAD_BIG_ENDIAN_DEFAULT))
        ROS_ERROR_STREAM("## ERROR sick_lidar_localization::UDPSender::messageCbLineMeasurementMessage0403(): sendPayload(" << sick_lidar_localization::Message::printPayload(line_measurement0403) << ") failed");
} 

/** subscriber callback function for input messages type messageCbLineMeasurementMessage0404 */
void sick_lidar_localization::UDPSender::messageCbLineMeasurementMessage0404(const sick_lidar_localization_msgs::LineMeasurementMessage0404 & msg)
{
    sick_lidar_localization::Message::LineMeasurementPayload0404 line_measurement0404;
    line_measurement0404.telegram_count = msg.telegram_count;
    line_measurement0404.timestamp = msg.timestamp;
    line_measurement0404.lcp1 = msg.lcp1;
    line_measurement0404.lcp2 = msg.lcp2;
    line_measurement0404.lcp3 = msg.lcp3;
    line_measurement0404.cnt_lpc = msg.cnt_lpc;
    line_measurement0404.reserved = 0;
    line_measurement0404.source_id = msg.source_id;
    if (!sendPayload(line_measurement0404, HEADER_BIG_ENDIAN_DEFAULT, PAYLOAD_BIG_ENDIAN_DEFAULT))
        ROS_ERROR_STREAM("## ERROR sick_lidar_localization::UDPSender::messageCbLineMeasurementMessage0404(): sendPayload(" << sick_lidar_localization::Message::printPayload(line_measurement0404) << ") failed");
} 
#endif // __ROS_VERSION > 0
