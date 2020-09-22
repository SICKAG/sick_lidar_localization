/*
 * @brief odom_converter subscribes to odometry messages, converts (vx, vy, omega) to binary odometry telegrams
 * and sends the telegrams to the localization server by udp.
 * See Operating-Instructions-EN-LLS-1.2.0.300R.pdf, chapter 5.9 "About Odometry for support", page 44-48
 * and Odometry_LLS.py for further details.
 *
 * Copyright (C) 2020 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2020 SICK AG, Waldkirch
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
 *  Copyright 2020 SICK AG
 *  Copyright 2020 Ing.-Buero Dr. Michael Lehning
 *
 */
#ifndef __SIM_LOC_ODOM_CONVERTER_H_INCLUDED
#define __SIM_LOC_ODOM_CONVERTER_H_INCLUDED

#include <string>
#include <vector>
#include <boost/asio.hpp>

#include "sick_lidar_localization/ros_wrapper.h"
#include "sick_lidar_localization/utils.h"

namespace sick_lidar_localization
{
  class UdpSenderSocketImpl;      ///< forward declaration of udp sender implementation

  /*!
   * @brief class OdomConverter subscribes to odometry messages, converts (timestamp, vx, vy, omega) to binary odometry telegrams
   * and sends the telegrams to the localization server by udp.
   * See Operating-Instructions-EN-LLS-1.2.0.300R.pdf, chapter 5.9 "About Odometry for support", page 44-48
   * and Odometry_LLS.py for further details.
   */
  class OdomConverter
  {
  public:

    /*!
     * Constructor.
     * @param[in] nh ros node handle
     * @param[in] server_adress ip adress of the localization controller, default: 192.168.0.1
     * @param[in] udp_port_odom udp port for odometry telegrams, default: 3000
     * @param[in] run_unittest run a short unittest, default: true
     */
    OdomConverter(ROS::NodePtr nh = 0, const std::string & server_adress = "192.168.0.1", int udp_port_odom = 3000, bool run_unittest = true);

    /*!
     * Callback for ros odometry messages. Converts the message to a binary odom telegram and sends the telegram to the localization server by udp.
     * @param[in] msg odometry message
     */
    virtual void messageCbOdometry(const sick_lidar_localization::OdomMsg & msg);
      /*! ROS2 version of function messageCbOdometry */
    virtual void messageCbOdometryROS2(const std::shared_ptr<sick_lidar_localization::OdomMsg> msg) { messageCbOdometry(*msg); }

    /*!
     * @brief Runs a unit test by converting some odometry messages and checking the result.
     * @param[in] verbose true: print info messages, false: print error messages only
     * @return unit test passed (true) or failed (false)
     */
    static bool Unittest(bool verbose = false);

    /*!
     * @brief Converts (timestamp, vx, vy, omega) to binary odometry telegrams.
     * Telegram format:
     *  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
     * |                                  H E A D E R                                               |                   P A Y L O A D                         |
     *  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
     * | MagicWord         PayloadLength payloadType(endianess)  MsgType MsgTypeVersion  SourceID   |  telegramCounter   timestamp    Vx     Vy     Omega     |
     * |  534d4f50             000C          0642                 0000        0000         0000     |      00000001      00000000    0001   FFFF   00000001   |
     *  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
     * Note: The header always has Big Endian format
     * @param[in] bigendian send payload big endian (true) or little endian (false)
     * @param[in] SourceID: 1 (e.g. vehicle controller 1)
     * @param[in] TelegramCount telegram counter
     * @param[in] Timestamp timestamp in ms
     * @param[in] vx x-component of velocity in mm/s
     * @param[in] vy y-component of velocity in mm/s
     * @param[in] omega angular velocity in mdeg/s
     * @return binary odometry telegram
     */
    static std::vector<uint8_t> Encode(bool bigendian = true, uint16_t SourceID = 1, uint32_t TelegramCount = 0, uint32_t Timestamp = 0, int16_t vx = 0, int16_t vy = 0, int32_t omega = 0);

  protected:

    /*!
    * Converts and returns a byte vector to hex string
    */
    static std::string ToHexString(const std::vector<uint8_t> & binary_telegram);

    /*!
    * @brief Runs a single test by converting a odometry messages and checking the result.
    * @param[in] bigendian send payload big endian (true) or little endian (false)
    * @param[in] SourceID: 1 (e.g. vehicle controller 1)
    * @param[in] TelegramCount telegram counter
    * @param[in] Timestamp timestamp in ms
    * @param[in] vx x-component of velocity in mm/s
    * @param[in] vy y-component of velocity in mm/s
    * @param[in] omega angular velocity in mdeg/s
    * @param[in] hex_result expected result (binary odometry telegram as hex string)
    * @param[in] verbose true: print info messages, false: print error messages only
    * @return unit test passed (true) or failed (false)
    */
    static bool Unittest(bool bigendian, uint16_t SourceID, uint32_t TelegramCount, uint32_t Timestamp, int16_t vx, int16_t vy, int32_t omega, const std::string & hex_result, bool verbose = false);

    /*!
    * @brief Converts and pushes 2 byte to binary odometry telegram.
    * @param[out] binary_telegram byte vetor to append 2 bytes
    * @param[in] value: bytes to encode in big or little endian order
    * @param[in] bigendian send payload big endian (true) or little endian (false)
    */
    static void EncodeAndPush(std::vector<uint8_t> & binary_telegram, uint16_t value, bool bigendian);

    /*!
    * @brief Converts and pushes 4 byte to binary odometry telegram.
    * @param[out] binary_telegram byte vetor to append 4 bytes
    * @param[in] value: bytes to encode in big or little endian order
    * @param[in] bigendian send payload big endian (true) or little endian (false)
    */
    static void EncodeAndPush(std::vector<uint8_t> & binary_telegram, uint32_t value, bool bigendian);

    /*
     * member data
     */

    ROS::NodePtr m_nh;              ///< ros node handle
    std::string m_server_adress;    ///< ip adress of the localization controller, default: 192.168.0.1
    int m_udp_port_odom;            ///< udp port for odometry telegrams, default: 3000
    int m_odom_telegrams_bigendian; ///< Send udp odometry telegrams big endian (1) or little endian (0)
    int m_odom_telegrams_source_id; ///< SourceID of udp odometry telegrams, e.g. vehicle controller 1
    sick_lidar_localization::UdpSenderSocketImpl* m_udp_sender_impl; ///< udp sender for odometry telegrams
    static sick_lidar_localization::SetGet32 s_odom_telegram_count;  ///< static odometry telegram counter

  }; // class OdomConverter

} // namespace sick_lidar_localization
#endif // __SIM_LOC_ODOM_CONVERTER_H_INCLUDED
