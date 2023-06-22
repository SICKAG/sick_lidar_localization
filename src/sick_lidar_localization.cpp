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
#include "sick_lidar_localization/sick_lidar_localization.h"
#include "launchparser/launchparser.h"
#include "sick_lidar_localization/sick_services.h"
#include "sick_lidar_localization/udp_message_publisher.h"
#include "sick_lidar_localization/udp_sender.h"
#include "sick_lidar_localization/udp_receiver_thread.h"

#define DELETE_PTR(p) if(p)delete(p);p=0

template <typename T> static bool getRequiredParam(rosNodePtr nh, const std::string& param_name, T& param_value)
{
    if (!rosGetParam(nh, param_name, param_value))
    {
        ROS_ERROR_STREAM("## ERROR sick_lidar_localization: getRequiredParam(" << param_name << ") failed, check launchfile (required parameter is missing), aborting... ");
        return false;
    }
    return true;
}

template <typename T> static bool getOptionalParam(rosNodePtr nh, const std::string& param_name, T& param_value, const T& default_value)
{
    bool ret_val = rosGetParam(nh, param_name, param_value);
    if (!ret_val)
        param_value = default_value;
    return ret_val;
}


/*
** @brief Parses commandline arguments, reads a launchfile and sets parameters for sick_lidar_localization
*/
bool sick_lidar_localization::API::parseLaunchfileSetParameter(rosNodePtr node, int argc, char** argv)
{
    if (!LaunchParser::parseLaunchfileSetParameter(node, argc, argv))
    {
        ROS_ERROR_STREAM("## ERROR sick_lidar_localization: parseLaunchfileSetParameter() failed, aborting... ");
        return false;
    }
    return true;
}

/*
** @brief Queries and returns the current configuration
*/
bool sick_lidar_localization::API::getParams(rosNodePtr node, sick_lidar_localization::Config& config)
{
    bool okay = true;
    okay = getRequiredParam(node, "hostname", config.hostname) && okay;
    okay = getRequiredParam(node, "serverpath", config.serverpath) && okay;
    okay = getRequiredParam(node, "verbose", config.verbose) && okay;
    okay = getRequiredParam(node, "udp_ip_lls_output", config.udp_ip_lls_output) && okay;
    okay = getRequiredParam(node, "udp_ip_lls_input", config.udp_ip_lls_input) && okay;
    okay = getRequiredParam(node, "udp_port_lls_input", config.udp_port_lls_input) && okay;
    okay = getRequiredParam(node, "udp_lls_input_source_id", config.udp_lls_input_source_id) && okay;
    okay = getRequiredParam(node, "udp_port_lls_output", config.udp_port_lls_output) && okay;
    okay = getRequiredParam(node, "udp_lls_output_logfile", config.udp_lls_output_logfile) && okay;
    okay = getRequiredParam(node, "software_pll_fifo_length", config.software_pll_fifo_length) && okay;
    okay = getRequiredParam(node, "odom_topic", config.odom_topic) && okay;
    okay = getRequiredParam(node, "ros_odom_to_udp_msg", config.ros_odom_to_udp_msg) && okay;
    if (!okay)
    {
        ROS_ERROR_STREAM("## ERROR sick_lidar_localization: getRequiredParam failed");
    }
    getOptionalParam(node, "udp_lls_input_source_id_1_1", config.msgtype_version_sourceid_map[1][1], config.udp_lls_input_source_id);
    getOptionalParam(node, "udp_lls_input_source_id_1_4", config.msgtype_version_sourceid_map[1][4], config.udp_lls_input_source_id);
    getOptionalParam(node, "udp_lls_input_source_id_1_5", config.msgtype_version_sourceid_map[1][5], config.udp_lls_input_source_id);
    getOptionalParam(node, "udp_lls_input_source_id_3_3", config.msgtype_version_sourceid_map[3][3], config.udp_lls_input_source_id);
    getOptionalParam(node, "udp_lls_input_source_id_4_3", config.msgtype_version_sourceid_map[4][3], config.udp_lls_input_source_id);
    getOptionalParam(node, "udp_lls_input_source_id_4_4", config.msgtype_version_sourceid_map[4][4], config.udp_lls_input_source_id);
    getOptionalParam(node, "udp_lls_input_source_id_7_1", config.msgtype_version_sourceid_map[7][1], config.udp_lls_input_source_id);
    ROS_INFO_STREAM("sick_lidar_localization config: hostname                           = " << config.hostname);
    ROS_INFO_STREAM("sick_lidar_localization config: serverpath                         = " << config.serverpath);
    ROS_INFO_STREAM("sick_lidar_localization config: verbose                            = " << config.verbose);
    ROS_INFO_STREAM("sick_lidar_localization config: udp_ip_lls_output                  = " << config.udp_ip_lls_output);
    ROS_INFO_STREAM("sick_lidar_localization config: udp_ip_lls_input                   = " << config.udp_ip_lls_input);
    ROS_INFO_STREAM("sick_lidar_localization config: udp_port_lls_input                 = " << config.udp_port_lls_input);
    ROS_INFO_STREAM("sick_lidar_localization config: udp_lls_input_source_id            = " << config.udp_lls_input_source_id);
    ROS_INFO_STREAM("sick_lidar_localization config: msgtype_version_sourceid_map[1][1] = " << config.msgtype_version_sourceid_map[1][1]);
    ROS_INFO_STREAM("sick_lidar_localization config: msgtype_version_sourceid_map[1][4] = " << config.msgtype_version_sourceid_map[1][4]);
    ROS_INFO_STREAM("sick_lidar_localization config: msgtype_version_sourceid_map[1][5] = " << config.msgtype_version_sourceid_map[1][5]);
    ROS_INFO_STREAM("sick_lidar_localization config: msgtype_version_sourceid_map[3][3] = " << config.msgtype_version_sourceid_map[3][3]);
    ROS_INFO_STREAM("sick_lidar_localization config: msgtype_version_sourceid_map[4][3] = " << config.msgtype_version_sourceid_map[4][3]);
    ROS_INFO_STREAM("sick_lidar_localization config: msgtype_version_sourceid_map[4][4] = " << config.msgtype_version_sourceid_map[4][4]);
    ROS_INFO_STREAM("sick_lidar_localization config: msgtype_version_sourceid_map[7][1] = " << config.msgtype_version_sourceid_map[7][1]);
    ROS_INFO_STREAM("sick_lidar_localization config: udp_port_lls_output                = " << config.udp_port_lls_output);
    ROS_INFO_STREAM("sick_lidar_localization config: udp_lls_output_logfile             = " << config.udp_lls_output_logfile);
    ROS_INFO_STREAM("sick_lidar_localization config: software_pll_fifo_length           = " << config.software_pll_fifo_length);
    ROS_INFO_STREAM("sick_lidar_localization config: odom_topic                         = " << config.odom_topic);
    ROS_INFO_STREAM("sick_lidar_localization config: ros_odom_to_udp_msg                = " << config.ros_odom_to_udp_msg);
    return okay;
}

/*
** @brief Default constructor
*/
sick_lidar_localization::API::API() : m_node(0)
{
}

/*
** @brief Default destructor
*/
sick_lidar_localization::API::~API()
{
    close();
}

/*
** @brief Initialization, installs services and threads, runs startup sequence
*/
bool sick_lidar_localization::API::init(rosNodePtr node, sick_lidar_localization::Config& config)
{
    m_node = node;
    m_config = config;

#ifdef _DEBUG
    sick_lidar_localization::JsonParser::unittest(m_config.verbose);
    sick_lidar_localization::UDPMessage::unittest();
#endif

    // Init REST-API
    m_curl = new sick_lidar_localization::CurlWrapper();
    m_curl->init(m_config.hostname, m_config.serverpath, m_config.verbose);

    // Install services
    m_services = new sick_lidar_localization::SickServices(node, m_config.hostname, m_config.serverpath, m_config.software_pll_fifo_length, m_config.verbose);

    // Start UDP receiver thread for UDP output messages
    m_udp_receiver_thread = new sick_lidar_localization::UDPReceiverThread(m_services, m_config.udp_ip_lls_output, m_config.udp_port_lls_output, m_config.udp_lls_output_logfile, m_config.verbose);
    m_udp_receiver_listener = new sick_lidar_localization::UDPMessage::InfoListener();
    if (m_config.verbose)
        m_udp_receiver_thread->registerListener(m_udp_receiver_listener);
    m_udp_message_publisher = new sick_lidar_localization::PublishUdpMessagesListener(node, m_config.verbose);
    m_udp_receiver_thread->registerListener(m_udp_message_publisher);
    if (!m_udp_receiver_thread->start())
    {
        ROS_ERROR_STREAM("## ERROR sick_lidar_localization: could not start UDP receiver thread, no UDP messages will be received or published");
    }

    // Start time sync thread to periodically request timestamps and to update Software PLL
    if (!m_services->startTimeSyncThread())
    {
        ROS_ERROR_STREAM("## ERROR sick_lidar_localization: could not start time synchronization thread, Software PLL will not be updated automatically");
    }

    // Initialize UDP sender for UDP input messages
    m_udp_sender = new sick_lidar_localization::UDPSender(node, m_config.udp_ip_lls_input, m_config.udp_port_lls_input, 
        sick_lidar_localization::UDPDefaultInputSourceId(m_config.udp_lls_input_source_id, m_config.msgtype_version_sourceid_map),
        m_config.verbose, m_config.odom_topic, m_config.ros_odom_to_udp_msg);

    return true;
}

/*
** @brief Stops all services and threads
*/
void sick_lidar_localization::API::close()
{
    if(m_udp_receiver_thread && m_udp_message_publisher)
        m_udp_receiver_thread->unregisterListener(m_udp_message_publisher);
    if (m_udp_receiver_thread && m_config.verbose)
        m_udp_receiver_thread->unregisterListener(m_udp_receiver_listener);
    if (m_services)
        m_services->stopTimeSyncThread();
    DELETE_PTR(m_services);
    if (m_udp_receiver_thread)
        m_udp_receiver_thread->stop();
    DELETE_PTR(m_udp_receiver_thread);
    DELETE_PTR(m_udp_receiver_listener);
    DELETE_PTR(m_udp_message_publisher);
    DELETE_PTR(m_udp_sender);
    DELETE_PTR(m_curl);
    m_node = 0;
}

/*
** @brief Register a listener for udp messages. The callback functions of the listener will be called after receiving a new udp message.
** Overwrite the functions defined in sick_lidar_localization::UDPMessage::Listener with customized code to handle udp messages.
*/
bool sick_lidar_localization::API::registerListener(sick_lidar_localization::UDPMessage::Listener* listener)
{
    if (m_udp_receiver_thread)
    {
        m_udp_receiver_thread->registerListener(listener);
        return true;
    }
    else
    {
        ROS_ERROR_STREAM("## ERROR sick_lidar_localization::API::registerListener(): sick_lidar_localization::API not initialized, use sick_lidar_localization::API::init() before calling this function.");

        return false;
    }
}

/*
** @brief Unregister a listener. Removes the listener from notifications after receiving udp messages.
*/
bool sick_lidar_localization::API::unregisterListener(sick_lidar_localization::UDPMessage::Listener* listener)
{
    if (m_udp_receiver_thread)
    {
        m_udp_receiver_thread->unregisterListener(listener);
        return true;
    }
    else
    {
        ROS_ERROR_STREAM("## ERROR sick_lidar_localization::API::unregisterListener(): sick_lidar_localization::API not initialized, use sick_lidar_localization::API::init() before calling this function.");
        return false;
    }
}

/*
** @brief Sends a UDP input message.
**        payload can be OdometryPayload0104, OdometryPayload0105, EncoderMeasurementPayload0202,
**        CodeMeasurementPayload0303, odeMeasurementPayload0701, LineMeasurementPayload0403 or LineMeasurementPayload0404
** @param[in] payload UDP message payload data
** @return true on success or false on error
*/
template<typename T> bool sendUDPPayload(sick_lidar_localization::UDPSender* udp_sender, const T& payload, bool encode_header_big_endian, bool encode_payload_big_endian)

{
    if (udp_sender)
    {
        return udp_sender->sendUDPPayload(payload, encode_header_big_endian, encode_payload_big_endian);
    }
    else
    {
        ROS_ERROR_STREAM("## ERROR sick_lidar_localization::API::sendUDPPayload(): sick_lidar_localization::API not initialized, use sick_lidar_localization::API::init() before calling this function.");
        return false;
    }
}

/*
** @brief Send udp messages to the localization controller
*/
bool sick_lidar_localization::API::sendUDPMessage(const sick_lidar_localization::UDPMessage::OdometryPayload0104& payload, bool encode_header_big_endian, bool encode_payload_big_endian)
{
    return sendUDPPayload(m_udp_sender, payload, encode_header_big_endian, encode_payload_big_endian);
}
bool sick_lidar_localization::API::sendUDPMessage(const sick_lidar_localization::UDPMessage::OdometryPayload0105& payload, bool encode_header_big_endian, bool encode_payload_big_endian)
{
    return sendUDPPayload(m_udp_sender, payload, encode_header_big_endian, encode_payload_big_endian);
}
bool sick_lidar_localization::API::sendUDPMessage(const sick_lidar_localization::UDPMessage::EncoderMeasurementPayload0202& payload, bool encode_header_big_endian, bool encode_payload_big_endian)
{
    return sendUDPPayload(m_udp_sender, payload, encode_header_big_endian, encode_payload_big_endian);
}
bool sick_lidar_localization::API::sendUDPMessage(const sick_lidar_localization::UDPMessage::CodeMeasurementPayload0303& payload, bool encode_header_big_endian, bool encode_payload_big_endian)
{
    return sendUDPPayload(m_udp_sender, payload, encode_header_big_endian, encode_payload_big_endian);
}
bool sick_lidar_localization::API::sendUDPMessage(const sick_lidar_localization::UDPMessage::CodeMeasurementPayload0701& payload, bool encode_header_big_endian, bool encode_payload_big_endian)
{
    return sendUDPPayload(m_udp_sender, payload, encode_header_big_endian, encode_payload_big_endian);
}
bool sick_lidar_localization::API::sendUDPMessage(const sick_lidar_localization::UDPMessage::LineMeasurementPayload0403& payload, bool encode_header_big_endian, bool encode_payload_big_endian)
{
    return sendUDPPayload(m_udp_sender, payload, encode_header_big_endian, encode_payload_big_endian);
}
bool sick_lidar_localization::API::sendUDPMessage(const sick_lidar_localization::UDPMessage::LineMeasurementPayload0404& payload, bool encode_header_big_endian, bool encode_payload_big_endian)
{
    return sendUDPPayload(m_udp_sender, payload, encode_header_big_endian, encode_payload_big_endian);
}
