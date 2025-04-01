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
#ifndef __SICK_LIDAR_LOCALIZATION_RECEIVER_THREAD_H_INCLUDED
#define __SICK_LIDAR_LOCALIZATION_RECEIVER_THREAD_H_INCLUDED

#include <list>
#include <string>
#include <thread>
#include "sick_lidar_localization/sick_services.h"
#include "sick_lidar_localization/message_parser.h"

namespace sick_lidar_localization
{
    /*
    ** @brief class UDPReceiverThread implements a receiver thread for output messages.
    */
    class UDPReceiverThread
    {
    public:

        /*
        ** @brief Default constructor
        ** @param[in] services time sync services
        ** @param[in] ip_lls_input IP address for input messages, or "" for broadcast, default: "192.168.0.1", (IP adress of the LLS device)
        ** @param[in] ip_lls_output IP address for output messages, or "" for broadcast (INADDR_ANY), default: "", use IP address of your local machine
        ** @param[in] port_lls_output port of output messages, default: 5010
        ** @param[in] udp_output_logfile Optional logfile for human readable output messages, default: "" (no outputlogfile)
        */
        UDPReceiverThread(sick_lidar_localization::SickServices* services = 0, const std::string& ip_lls_input = "", const std::string& ip_lls_output = "", int port_lls_output = 5010, const std::string& udp_output_logfile = "", int verbose = 0);

        /*
        ** @brief Default destructor, exits running threads
        */
        ~UDPReceiverThread();

        /*
        ** @brief Starts the receiver thread.
        **        - receive data
        **        - parse and convert output messages (odometry, line measurement, code measurement and LocalizationController result messages)
        **        - publish converted messages
        */
        bool start();

        /*
        ** @brief Stops and exits the receiver thread.
        */
        void stop();

        /*
        ** @brief Register a listener for messages. The callback functions of the listener will be called after receiving a new message.
        ** Overwrite the functions defined in sick_lidar_localization::Message::Listener with customized code to handle messages.
        */
        void registerListener(sick_lidar_localization::Message::Listener* listener);

        /*
        ** @brief Unregister a listener. Removes the listener from notifications after receiving messages.
        */
        void unregisterListener(sick_lidar_localization::Message::Listener* listener);

    protected:

        /*
        ** @brief Just sleeps for a given amount of time
        ** @param[in] seconds time to sleep in seconds
        */
        void sleep(double seconds);

        /*
        ** @brief Thread callback, runs the receiver thread:
        **        - init socket
        **        - receive data
        **        - parse and convert output messages (odometry, line measurement, code measurement and LocalizationController result messages)
        **        - publish converted messages
        */
        bool runReceiver(void);

        sick_lidar_localization::SickServices* m_services; // time sync services
        std::string m_ip_lls_input;       // IP address for input messages, or "" for broadcast, default: "192.168.0.1", (IP adress of the LLS device)
        std::string m_ip_lls_output;      // IP address for output messages, or "" for broadcast (INADDR_ANY), default: "", use IP address of your local machine
        int m_port_lls_output;            // port of output messages, default: 5010
        std::string m_udp_output_logfile; // Optional logfile for human readable output messages, default: "" (no outputlogfile)
        bool m_run_receiver_thread;           // Flag to start and stop m_receiver_thread
        std::thread* m_receiver_thread;       // Background thread to receive data, parsing, conversion and publishing
        std::list<sick_lidar_localization::Message::Listener*> m_message_listener;
        int m_verbose;

    }; // class UDPReceiverThread

} // namespace sick_lidar_localization
#endif // __SICK_LIDAR_LOCALIZATION_RECEIVER_THREAD_H_INCLUDED
