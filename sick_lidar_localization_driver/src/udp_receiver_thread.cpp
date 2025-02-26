/*
 * @brief receiver_thread implements a receiver thread for output messages.
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
#define UNLINK _unlink
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
#define UNLINK unlink
#define closesocket close
static std::string getErrorMessage(void) { return std::to_string(errno) + " (" + std::string(strerror(errno)) + ")"; }
#endif

#include "sick_lidar_localization/sick_common.h"
#include "sick_lidar_localization/udp_receiver_thread.h"

/*
** @brief Default constructor
** @param[in] services time sync services
** @param[in] ip_lls_output IP address for output messages, or "" for broadcast (INADDR_ANY), default: "", use IP address of your local machine
** @param[in] port_lls_output port of output messages, default: 5010
** @param[in] udp_output_logfile Optional logfile for human readable output messages, default: "" (no outputlogfile)
*/
sick_lidar_localization::UDPReceiverThread::UDPReceiverThread(sick_lidar_localization::SickServices* services, const std::string& ip_lls_input, const std::string& ip_lls_output, int port_lls_output, const std::string& udp_output_logfile, int verbose) 
    : m_services(services), m_ip_lls_input(ip_lls_input), m_ip_lls_output(ip_lls_output), m_port_lls_output(port_lls_output), m_udp_output_logfile(udp_output_logfile), m_run_receiver_thread(false), m_receiver_thread(0), m_verbose(verbose)
{
}

/*
** @brief Default destructor, exits running threads
*/
sick_lidar_localization::UDPReceiverThread::~UDPReceiverThread()
{
    stop();
}

/*
** @brief Starts the receiver thread.
**        - receive data
**        - parse and convert output messages (odometry, line measurement, code measurement and LocalizationController result messages)
**        - publish converted messages
*/
bool sick_lidar_localization::UDPReceiverThread::start()
{
    m_run_receiver_thread = true;
    m_receiver_thread = new std::thread(&sick_lidar_localization::UDPReceiverThread::runReceiver, this);
    return m_run_receiver_thread;
}

/*
** @brief Stops and exits the receiver thread.
*/
void sick_lidar_localization::UDPReceiverThread::stop()
{
    m_run_receiver_thread = false;
    if (m_receiver_thread)
    {
        m_receiver_thread->join();
        delete(m_receiver_thread);
        m_receiver_thread = 0;
    }
}

/*
** @brief Register a listener for messages. The callback functions of the listener will be called after receiving a new message.
** Overwrite the functions defined in sick_lidar_localization::Message::Listener with customized code to handle messages.
*/
void sick_lidar_localization::UDPReceiverThread::registerListener(sick_lidar_localization::Message::Listener* listener)
{
    m_message_listener.push_back(listener);
}

/*
** @brief Unregister a listener. Removes the listener from notifications after receiving messages.
*/
void sick_lidar_localization::UDPReceiverThread::unregisterListener(sick_lidar_localization::Message::Listener* listener)
{
    m_message_listener.remove(listener);
}

/*
** @brief Just sleeps for a given amount of time
** @param[in] seconds time to sleep in seconds
*/
void sick_lidar_localization::UDPReceiverThread::sleep(double seconds)
{
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(seconds * 1000));
}

/*
** @brief Thread callback, runs the receiver thread:
**        - init udp_socket
**        - receive data
**        - parse and convert output messages (odometry, line measurement, code measurement and LocalizationController result messages)
**        - publish converted messages
*/
bool sick_lidar_localization::UDPReceiverThread::runReceiver(void)
{
    if (!m_udp_output_logfile.empty())
        UNLINK(m_udp_output_logfile.c_str());
    SOCKET udp_socket = INVALID_SOCKET;
    while (rosOk() && m_run_receiver_thread)
    {
        // Init udp_socket
        ROS_INFO_STREAM("sick_lidar_localization::UDPReceiverThread: receiver thread started, initializing udp_socket... ");
        while (rosOk() && m_run_receiver_thread && (udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == INVALID_SOCKET)
        {
            sleep(1.0); // Retry after 1 second
        }
        // int broadcast_opt = 1;
        // setsockopt(udp_socket, SOL_SOCKET, SO_BROADCAST, &broadcast_opt, sizeof(broadcast_opt));
        struct sockaddr_in lls_servaddr = { 0 };
        if(m_ip_lls_output.empty())
            lls_servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
        else
            lls_servaddr.sin_addr.s_addr = inet_addr(m_ip_lls_output.c_str()); 
        lls_servaddr.sin_family = AF_INET;
        lls_servaddr.sin_port = htons(m_port_lls_output);
        ROS_INFO_STREAM("sick_lidar_localization::UDPReceiverThread: udp_socket created, binding to port " << ntohs(lls_servaddr.sin_port) << " ... ");
        while (rosOk() && m_run_receiver_thread && udp_socket != INVALID_SOCKET && bind(udp_socket, (SOCKADDR*)&lls_servaddr, sizeof(lls_servaddr)) < 0)
        {
            ROS_INFO_STREAM("sick_lidar_localization::UDPReceiverThread: udp_socket bind failed, error " << getErrorMessage() << ", retrying...");
            sleep(1.0); // Retry after 1 second
        }
        
        // Receive and handle messages
        ROS_INFO_STREAM("sick_lidar_localization::UDPReceiverThread: udp_socket initialized, listening ...");
        int bytes_received = 0;
        uint8_t buffer[1024] = { 0 };
        while (rosOk() && m_run_receiver_thread && udp_socket != INVALID_SOCKET)
        {
            // Receive new message, at least 16 byte header required
            int bytes_required = sizeof(sick_lidar_localization::Message::HeaderData);
            bool msg_header_received = false;
            sick_lidar_localization::Message::HeaderData msg_header = { 0 };
            while (rosOk() && m_run_receiver_thread && udp_socket != INVALID_SOCKET && bytes_received < bytes_required)
            {
                struct sockaddr_in lls_controller_addr = { 0 };
                socklen_t lls_controller_addr_len = sizeof(lls_controller_addr);
                int n = recvfrom(
                    udp_socket, (char*)&buffer[bytes_received], (int)sizeof(buffer) - bytes_received, 0, (SOCKADDR*)&lls_controller_addr, &lls_controller_addr_len
                );

                bool ip_specific_recv = !m_ip_lls_input.empty() && lls_controller_addr.sin_addr.s_addr != inet_addr(m_ip_lls_input.c_str()); // If no specific input IP is set, no check of the sender IP is done
                if (n <= 0 || ip_specific_recv)
                {
                    if (n < 0)
                        ROS_INFO_STREAM("error: " << getErrorMessage());
                    sleep(0.001);
                    continue;
                }
                bytes_received += n;
                if (!msg_header_received && bytes_received >= sizeof(sick_lidar_localization::Message::HeaderData))
                {
                    // Decode message header
                    if (!sick_lidar_localization::Message::decodeHeader(&buffer[0], bytes_received, msg_header))
                    {
                        ROS_ERROR_STREAM("## ERROR sick_lidar_localization::UDPReceiverThread: invalid message header, decodeMessageHeader failed, message ignored");
                        bytes_received = 0;
                        break;
                    }
                    bytes_required += msg_header.payloadlength;
                    msg_header_received = true;
                }
            }
            if (bytes_received < bytes_required)
            {
                sleep(0.001);
                continue;
            }

            // Parse and decode output messages (odometry, line measurement, code measurement and LocalizationController result messages)
            bool payload_is_big_endian = (msg_header.payloadtype == 0x0642);
            uint8_t* payload_buffer = &buffer[0] + sizeof(sick_lidar_localization::Message::HeaderData);
            int payload_len = bytes_received - sizeof(sick_lidar_localization::Message::HeaderData);
            sick_lidar_localization::Message::PayloadBase* payload = 0;
            if (msg_header.msgtype == 1 && msg_header.msgtypeversion == 4) // odometry payload message type 1 version 4 (24 byte payload)
            {
                payload = new sick_lidar_localization::Message::Payload<sick_lidar_localization::Message::OdometryPayload0104>();
            }
            else if (msg_header.msgtype == 1 && msg_header.msgtypeversion == 5) // odometry payload message type 1 version 5 (40 byte payload)
            {
                payload = new sick_lidar_localization::Message::Payload<sick_lidar_localization::Message::OdometryPayload0105>();
            }
            else if (msg_header.msgtype == 3 && msg_header.msgtypeversion == 4) // code measurement payload message type 3 version 4 (24 byte payload)
            {
                payload = new sick_lidar_localization::Message::Payload<sick_lidar_localization::Message::CodeMeasurementPayload0304>();
            }
            else if (msg_header.msgtype == 4 && msg_header.msgtypeversion == 3) // line measurement payload message type 4 version 3 (variable length payload)
            {
                payload = new sick_lidar_localization::Message::Payload<sick_lidar_localization::Message::LineMeasurementPayload0403>();
            }
            else if (msg_header.msgtype == 4 && msg_header.msgtypeversion == 4) // line measurement payload message type 4 version 4 (24 byte payload)
            {
                payload = new sick_lidar_localization::Message::Payload<sick_lidar_localization::Message::LineMeasurementPayload0404>();
            }
            else if (msg_header.msgtype == 5 && msg_header.msgtypeversion == 2) // LocalizationController result payload message type 5 version 2 (40 byte payload)
            {
                payload = new sick_lidar_localization::Message::Payload<sick_lidar_localization::Message::LocalizationControllerResultPayload0502>();
            }
            else
            {
                ROS_ERROR_STREAM("## ERROR sick_lidar_localization::UDPReceiverThread: " << bytes_received << " byte message received, msgtype " << msg_header.msgtype << ", msgtypeversion " << msg_header.msgtypeversion << ", message not supported");
            }
            if (payload && payload->decodePayload(payload_buffer, payload_len, payload_is_big_endian))
            {
                payload->sourceID() = msg_header.sourceid;
                sick_lidar_localization::SyncTimeStamp sync_time_stamp = sick_lidar_localization::makeSyncTimeStamp(0, 0, false);
                if(m_services)
                {
                    uint64_t tics_u64 = payload->getTimestamp(); // SIM uses tics in microseconds in UDP-messages
                    sync_time_stamp = m_services->getSystemTimeFromTics(tics_u64, 0);
                    if(sync_time_stamp.valid != 0)
                    {
                        payload->setSyncTimestamp(sync_time_stamp.sec, sync_time_stamp.nsec);
                    }
                }
                if (m_verbose > 0) {
                    ROS_INFO_STREAM("sick_lidar_localization::UDPReceiverThread: message received: " << payload->toString(true));         
                }                
                // Notify listener (and publish output messages on ROS-1 or ROS-2)
                payload->notifyListener(m_message_listener);
                // Log output messages
                FILE* fp_udp_output_logfile = 0;
                if (!m_udp_output_logfile.empty() && (fp_udp_output_logfile = fopen(m_udp_output_logfile.c_str(), "a")) != 0)
                {
                    std::string payload_str = payload->toString(false);
                    fprintf(fp_udp_output_logfile, "%s\n", payload_str.c_str());
                    fclose(fp_udp_output_logfile);
                }
            }
            else
            {
                ROS_ERROR_STREAM("## ERROR sick_lidar_localization::UDPReceiverThread: " << bytes_received << " byte message received, msgtype " << msg_header.msgtype << ", msgtypeversion " << msg_header.msgtypeversion << ", " << msg_header.payloadlength << " byte payload, " << (payload_is_big_endian ? "big endian" : "little endian"));
                ROS_ERROR_STREAM("## ERROR sick_lidar_localization::UDPReceiverThread: decodePayload failed, message ignored");
            }
            if(payload)
            {
                delete(payload);
                payload = 0;
            }

            // Next message
            if (bytes_received > bytes_required) // buffer contains start of next message, move all consumed bytes from buffer[bytes_required] to buffer[0] (bytes_required := number of bytes for the complete message incl. header and payload)
            {
                ROS_INFO_STREAM("sick_lidar_localization::UDPReceiverThread: " << bytes_received << " bytes received, " << bytes_required << " bytes consumed");
                memmove(&buffer[0], &buffer[bytes_required], sizeof(buffer) - bytes_required);
                bytes_received -= bytes_required;
            }
            else // start a new message at buffer[0]
            {
                bytes_received = 0;
            }

        }
        if (udp_socket != INVALID_SOCKET)
            closesocket(udp_socket);
        udp_socket = INVALID_SOCKET;
    }
    ROS_INFO_STREAM("sick_lidar_localization::UDPReceiverThread: receiver thread exiting...");
    
    // Close udp_socket
    if (udp_socket != INVALID_SOCKET)
        closesocket(udp_socket);

    m_run_receiver_thread = false;
    return true;
}
