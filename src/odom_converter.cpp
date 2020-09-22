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
#include <iomanip>
#include <sstream>

#include "sick_lidar_localization/odom_converter.h"

/** static odometry telegram counter */
sick_lidar_localization::SetGet32 sick_lidar_localization::OdomConverter::s_odom_telegram_count(0); 

namespace sick_lidar_localization
{
  /*!
   * @brief class UdpSenderSocketImpl implements the udp socket for sending udp packages, uses boost::asio::ip::udp
   */
  class UdpSenderSocketImpl
  {
  public:
    /*!
     * Constructor, opens an udp socket.
     * @param[in] server_adress ip adress of the localization controller, default: 192.168.0.1
     * @param[in] udp_port_odom udp port for odometry telegrams, default: 3000
     */
    UdpSenderSocketImpl(const std::string & server_adress = "192.168.0.1", int udp_port_odom = 3000)
    : m_io_service(), m_udp_socket(m_io_service), m_socket_opened(false)
    {
      try
      {
        m_udp_socket.open(boost::asio::ip::udp::v4());
        m_socket_opened = m_udp_socket.is_open();
        if (!m_socket_opened)
        {
          ROS_ERROR_STREAM("## ERROR UdpSenderSocketImpl(" << server_adress << ":" << udp_port_odom << "): can't open udp socket.");
        }
        // m_udp_socket.set_option(boost::asio::socket_base::reuse_address(true));
        m_udp_socket.set_option(boost::asio::socket_base::broadcast(true));
        m_udp_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(server_adress), udp_port_odom);
      }
      catch(const std::exception& e)
      {
        m_socket_opened = false;
        ROS_ERROR_STREAM("## ERROR UdpSenderSocketImpl(): socket initialization failed, exception " << e.what());
      }
    }

    /*!
     * Returns true if the udp socket is opened and ready to send, or false otherwise.
     */
    bool IsOpen(void) const { return m_socket_opened; }

    /*!
     * Sends a binary telegram.
     */
    bool Send(std::vector<uint8_t> & binary_telegram)
    {
      size_t bytes_sent = 0;
      if(m_socket_opened)
      {
        try
        {
          bytes_sent = m_udp_socket.send_to(boost::asio::buffer(binary_telegram.data(), binary_telegram.size()), m_udp_endpoint);
          if(bytes_sent != binary_telegram.size())
          {
            ROS_ERROR_STREAM("## ERROR UdpSenderSocketImpl()::Send(): " << bytes_sent << " of " << binary_telegram.size() << " bytes sent, boost::asio::ip::udp::socket::send_to() failed.");
          }
        }
        catch(const std::exception& e)
        {
          ROS_ERROR_STREAM("## ERROR UdpSenderSocketImpl()::Send(): boost::asio::ip::udp::socket::send_to() failed, exception " << e.what());
        }
      }
      else
      {
        ROS_ERROR_STREAM("## ERROR UdpSenderSocketImpl()::Send(): udp socket not initialized");
      }
      return (bytes_sent == binary_telegram.size());
    }

  protected:

    bool m_socket_opened; ///< true if the udp socket is opened and ready to send, or false otherwise
    boost::asio::io_service m_io_service; ///< boost io for udp socket
    boost::asio::ip::udp::socket m_udp_socket; ///< udp socket for binary odom telegrams
    boost::asio::ip::udp::endpoint m_udp_endpoint; ///< udp receiver (i.e. the localizaton server)
  };
} // namespace sick_lidar_localization

/*!
 * Constructor.
 * @param[in] nh ros node handle
 * @param[in] server_adress ip adress of the localization controller, default: 192.168.0.1
 * @param[in] udp_port_odom udp port for odometry telegrams, default: 3000
 * @param[in] run_unittest run a short unittest, default: true
 */
sick_lidar_localization::OdomConverter::OdomConverter(ROS::NodePtr nh, const std::string & server_adress, int udp_port_odom, bool run_unittest)
: m_nh(nh), m_server_adress(server_adress), m_udp_port_odom(udp_port_odom), m_odom_telegrams_bigendian(1), m_odom_telegrams_source_id(1), m_udp_sender_impl(0)
  
{
  if(nh)
  {
    ROS::param<int>(nh, "/sick_lidar_localization/driver/odom_telegrams_bigendian", m_odom_telegrams_bigendian, m_odom_telegrams_bigendian); // Send udp odometry telegrams big endian (1) or little endian (0)
    ROS::param<int>(nh, "/sick_lidar_localization/driver/odom_telegrams_source_id", m_odom_telegrams_source_id, m_odom_telegrams_source_id); // SourceID of udp odometry telegrams, e.g. vehicle controller 1
    // Initialize upd sender
    m_udp_sender_impl = new sick_lidar_localization::UdpSenderSocketImpl(m_server_adress, m_udp_port_odom);
    if(!m_udp_sender_impl->IsOpen())
    {
        ROS_ERROR_STREAM("## ERROR OdomConverter(" << server_adress << ":" << udp_port_odom << "): can't initialize udp socket.");
    }
  }
  // Run unittest
  if(run_unittest && !Unittest(false))
  {
    ROS_ERROR_STREAM("## ERROR OdomConverter::Unittest() failed.");
    Unittest(true); // repeat unittest with verbose=true
  }
}

/*!
 * Callback for ros odometry messages. Converts the message to a binary odom telegram and sends the telegram to the localization server by udp.
 * @param[in] msg odometry message
 */
void sick_lidar_localization::OdomConverter::messageCbOdometry(const sick_lidar_localization::OdomMsg & msg)
{
  // Get timestamp, linear and angular velocity from odom message
  ROS::Time timestamp_msg = ROS::timeFromHeader(&msg.header);
  double vx_ms = msg.twist.twist.linear.x;     // vx in m/s
  double vy_ms = msg.twist.twist.linear.y;     // vy in m/s
  double omega_rs = msg.twist.twist.angular.z; // omega in rad/s

  // Convert to binary telegram
  int16_t vx_mms = (int16_t)std::round(1000.0 * vx_ms);                          // x-component of velocity in mm/s
  int16_t vy_mms = (int16_t)std::round(1000.0 * vy_ms);                          // y-component of velocity in mm/s
  int32_t omega_mdegs = (int32_t)std::round(1000.0 * 180.0 * omega_rs / M_PI);   // angular velocity in mdeg/s
  uint32_t timestamp_msec = (uint32_t)ROS::timestampMilliseconds(timestamp_msg); // message timestamp in ms
  bool bigendian = (m_odom_telegrams_bigendian > 0);                             // encode telegram payload big or little endian
  uint16_t source_id = (uint16_t)(m_odom_telegrams_source_id & 0xFFFF);          // configurable source id (sender id)
  uint32_t telegram_count = s_odom_telegram_count.inc();                         // increasing odom telegram counter
  std::vector<uint8_t> binary_telegram = Encode(bigendian, source_id, telegram_count, timestamp_msec, vx_mms, vy_mms, omega_mdegs); // binary telegram

  // Send binary telegram
  if(m_udp_sender_impl == 0 || !m_udp_sender_impl->IsOpen()) // Reopen socket in case of network errors
  {
    delete(m_udp_sender_impl);
    m_udp_sender_impl = new sick_lidar_localization::UdpSenderSocketImpl(m_server_adress, m_udp_port_odom);
  }
  if(m_udp_sender_impl == 0 || !m_udp_sender_impl->IsOpen())
  {
    ROS_ERROR_STREAM("## ERROR OdomConverter(" << m_server_adress << ":" << m_udp_port_odom << "): can't initialize udp socket.");
  }
  else
  {
    if(!m_udp_sender_impl->Send(binary_telegram))
    {
      // upd send failed, reopen socket on next telegram
      ROS_ERROR_STREAM("OdomConverter::messageCbOdometry(vx=" << vx_ms << ",vy=" << vy_ms << ",vth_rs=" << omega_rs << ",timestamp_sec=" << ROS::secondsSinceStart(timestamp_msg) 
        << "): failed to send udp telegram " << ToHexString(binary_telegram));
      delete(m_udp_sender_impl);
      m_udp_sender_impl = 0;
    }
    else
    {
      ROS_INFO_STREAM("OdomConverter::messageCbOdometry(vx=" << vx_ms << ",vy=" << vy_ms << ",vth_rs=" << omega_rs << ",timestamp_sec=" << ROS::secondsSinceStart(timestamp_msg) << "): udp telegram " 
        << ToHexString(binary_telegram) << " sent (vx_mms=" << vx_mms << ",vy_mms=" << vy_mms << ",vth_mdegs=" << omega_mdegs << ",timestamp_msec=" << timestamp_msec << ")");
    }
  }
}

/*!
 * @brief Runs a unit test by converting some odometry messages and checking the result.
 * @param[in] verbose true: print info messages, false: print error messages only
 * @return unit test passed (true) or failed (false)
 */
bool sick_lidar_localization::OdomConverter::Unittest(bool verbose)
{
  return Unittest(true, 1, 1000, 50000, -500, 500, -1000, "534D4F5000100642000100010001000003E80000C350FE0C01F4FFFFFC18")
    && Unittest(true, 100, 126, (uint32_t)1599489146050U, 200, 0, 10000, "534D4F50001006420001000100640000007E68FB7CC200C8000000002710")
    && Unittest(true, 100, 481, (uint32_t)1599501152450U, -200, -300, -5000, "534D4F5000100642000100010064000001E169B2B0C2FF38FED4FFFFEC78")
    && Unittest(true, 100, 1365, (uint32_t)1599502220425U, 200, 300, 5000, "534D4F50001006420001000100640000055569C2FC8900C8012C00001388");
}

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
bool sick_lidar_localization::OdomConverter::Unittest(bool bigendian, uint16_t SourceID, uint32_t TelegramCount, uint32_t Timestamp, int16_t vx, int16_t vy, int32_t omega, const std::string & hex_result, bool verbose)
{
  std::vector<uint8_t> binary_telegram = Encode(bigendian, SourceID, TelegramCount, Timestamp, vx, vy, omega);
  std::string hex_encoded = ToHexString(binary_telegram);
  if(hex_result != hex_encoded)
  {
    ROS_ERROR_STREAM("## ERROR OdomConverter::Unittest(" << bigendian << ", " << SourceID << ", " << TelegramCount << ", " << Timestamp << ", " << vx << ", " << vy << ", " << omega
      << ") failed: encoded: \"" << hex_encoded << "\", expected: \"" << hex_result << "\"");
    return false;
  }
  if(verbose)
  {
    ROS_INFO_STREAM("OdomConverter::Unittest(" << bigendian << ", " << SourceID << ", " << TelegramCount << ", " << Timestamp << ", " << vx << ", " << vy << ", " << omega
      << ") passed: encoded: \"" << hex_encoded << "\", expected: \"" << hex_result << "\"");
  }
  else
  {
    ROS_DEBUG_STREAM("OdomConverter::Unittest(" << bigendian << ", " << SourceID << ", " << TelegramCount << ", " << Timestamp << ", " << vx << ", " << vy << ", " << omega
      << ") passed: encoded: \"" << hex_encoded << "\", expected: \"" << hex_result << "\"");
  }
  return true;
}

/*!
 * Converts and returns a byte vector to hex string
 */
std::string sick_lidar_localization::OdomConverter::ToHexString(const std::vector<uint8_t> & binary_telegram)
{
  std::stringstream str;
  for(size_t n = 0; n < binary_telegram.size(); n++)
  {
    str << std::uppercase << std::hex << std::setfill('0') << std::setw(2) << (uint32_t)binary_telegram[n];
  }
  return str.str();
}

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
std::vector<uint8_t> sick_lidar_localization::OdomConverter::Encode(bool bigendian, uint16_t SourceID, uint32_t TelegramCount, uint32_t Timestamp, int16_t vx, int16_t vy, int32_t omega)
{
  std::vector<uint8_t> binary_telegram;
  // MagicWord Char32 SMOP 4 byte (SMOP: Sick Mobile Platforms UDP telegram)
  binary_telegram.push_back('S');
  binary_telegram.push_back('M');
  binary_telegram.push_back('O');
  binary_telegram.push_back('P');
  // Header: 2 byte PayloadLength UInt16: Number of bytes per payload, here always 4+4+2+2+4 = 16
  EncodeAndPush(binary_telegram, (uint16_t)(16), true);
  // Header: 2 byte PayloadType (endianness) UInt16 (BE: 0x0642, LE: 0x06c2)
  EncodeAndPush(binary_telegram, (uint16_t)(bigendian ? 0x0642 : 0x06C2), true);
  // Header: 2 byte MsgType UInt16: For odometry messages, MsgType = 1 is used
  EncodeAndPush(binary_telegram, (uint16_t)(1), true);
  // Header: 2 byte MsgTypeVersion UInt16: Version of the MsgType payload (previously only MsgTypeVersion = 1)
  EncodeAndPush(binary_telegram, (uint16_t)(1), true);
  // Header: 2 byte SourceID UInt16: e.g. vehicle controller 1
  EncodeAndPush(binary_telegram, (uint16_t)(SourceID), true);
  // Payload: 4 byte TelegramCount UInt32
  EncodeAndPush(binary_telegram, (uint32_t)(TelegramCount), bigendian);
  // Payload: 4 byte Timestamp UInt32 in ms
  EncodeAndPush(binary_telegram, (uint32_t)(Timestamp), bigendian);
  // Payload: 2 byte vx Int16 in mm/s
  EncodeAndPush(binary_telegram, (uint16_t)(vx), bigendian);
  // Payload: 2 byte vy Int16 in mm/s
  EncodeAndPush(binary_telegram, (uint16_t)(vy), bigendian);
  // Payload: 4 byte vx Int16 in mm/s
  EncodeAndPush(binary_telegram, (uint32_t)(omega), bigendian);
  return binary_telegram;
}

/*!
 * @brief Converts and pushes 2 byte to binary odometry telegram.
 * @param[out] binary_telegram byte vetor to append 2 bytes
 * @param[in] value: bytes to encode in big or little endian order
 * @param[in] bigendian send payload big endian (true) or little endian (false)
 */
void sick_lidar_localization::OdomConverter::EncodeAndPush(std::vector<uint8_t> & binary_telegram, uint16_t value, bool bigendian)
{
  if(bigendian)
  {
    binary_telegram.push_back((value >> 8) & 0xFF);
    binary_telegram.push_back((value) & 0xFF);
  }
  else
  {
    binary_telegram.push_back((value) & 0xFF);
    binary_telegram.push_back((value >> 8) & 0xFF);
  }
}

/*!
 * @brief Converts and pushes 4 byte to binary odometry telegram.
 * @param[out] binary_telegram byte vetor to append 4 bytes
 * @param[in] value: bytes to encode in big or little endian order
 * @param[in] bigendian send payload big endian (true) or little endian (false)
 */
void sick_lidar_localization::OdomConverter::EncodeAndPush(std::vector<uint8_t> & binary_telegram, uint32_t value, bool bigendian)
{
  if(bigendian)
  {
    binary_telegram.push_back((value >> 24) & 0xFF);
    binary_telegram.push_back((value >> 16) & 0xFF);
    binary_telegram.push_back((value >>  8) & 0xFF);
    binary_telegram.push_back((value) & 0xFF);
  }
  else
  {
    binary_telegram.push_back((value) & 0xFF);
    binary_telegram.push_back((value >>  8) & 0xFF);
    binary_telegram.push_back((value >> 16) & 0xFF);
    binary_telegram.push_back((value >> 24) & 0xFF);
  }
}