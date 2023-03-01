/*
 * @brief udp_message_parser encodes and decodes UDP input and output messages.
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
#include <iomanip>
#include <string.h>
#include "sick_lidar_localization/sick_common.h"
#include "sick_lidar_localization/udp_message_parser.h"

/*
** @brief Encodes a 64 bit value in little or big endian
** @param[in] value value to be encoded
** @param[out] buffer output buffer
** @param[in] len number of bytes to write to buffer
** @param[in] encode_big_endian encode in big endian (true) or little endian (false) byte order
*/
static void encode(uint64_t value, uint8_t* buffer, int len, bool encode_big_endian)
{
    if (encode_big_endian)
    {
        for (int n = len - 1; n >= 0; n--)
        {
            buffer[n] = (value & 0xFF);
            value = (value >> 8);
        }
    }
    else
    {
        for (int n = 0; n < len; n++)
        {
            buffer[n] = (value & 0xFF);
            value = (value >> 8);
        }
    }
}

/*
** @brief Decodes the header of a udp message
** @param[in] udp_buffer raw bytes received via UDP
** @param[in] big_endian assume big endian (true) or little endian (false) encoding
** @param[out] msg_header decoded message header
*/
bool sick_lidar_localization::UDPMessage::decodeHeader(const uint8_t* udp_buffer, int bytes_received, sick_lidar_localization::UDPMessage::HeaderData& msg_header)
{
    if (bytes_received >= sizeof(sick_lidar_localization::UDPMessage::HeaderData))
    {
        uint8_t magic_spec_be[4] = { 0x4d, 0x4f, 0x50, 0x53 }; // expected magic field according to specification LocalizationController 2.0
        uint8_t magic_spec_le[4] = { 0x53, 0x50, 0x4f, 0x4d }; // expected magic field according to specification LocalizationControlleralizationController 2.0
        for (int n = 0; n < 4; n++)
        {
            msg_header.magic[n] = udp_buffer[n];
        }
        if (memcmp(&msg_header.magic[0], &magic_spec_be[0], 4) != 0 && memcmp(&msg_header.magic[0], &magic_spec_le[0], 4) != 0)
        {
            ROS_ERROR_STREAM("## ERROR decodeUDPMessageHeader: unexpected magic word");
            return false;
        }
        // Determine header endianness (LocalizationController 2.0 spec): 
        // PayloadType (endianness) uint16 (BE: 0x0642, LE: 0x06c2): The endianness of the payload. 
        // The endianness of the header can differ and is automatically detected based on this field.
        uint8_t payloadtype_raw[2];
        payloadtype_raw[0] = udp_buffer[8];
        payloadtype_raw[1] = udp_buffer[9];
        bool header_is_big_endian = true; // default: header encoded in big endian (payload is little or big endian)
        if (payloadtype_raw[0] == 0x06 && (payloadtype_raw[1] == 0x42 || payloadtype_raw[1] == 0xc2))
            header_is_big_endian = true;
        else if (payloadtype_raw[1] == 0x06 && (payloadtype_raw[0] == 0x42 || payloadtype_raw[0] == 0xc2))
            header_is_big_endian = false;
        else
            ROS_ERROR_STREAM("## ERROR UDPMessage::decodeHeader: payloadtype = { 0x" << std::hex << std::setfill('0') << std::setw(2) << (int)(payloadtype_raw[0]) << ", 0x" << (int)(payloadtype_raw[1]) << " }, "
                << ", expected \"0642\", \"06c2\", \"4206\" or \"c206\"");
        sick_lidar_localization::UDPMessage::decode(udp_buffer + 4, header_is_big_endian, msg_header.headerversion);
        if (msg_header.headerversion != 0x02)
        {
            ROS_ERROR_STREAM("## ERROR UDPMessage::decodeHeader: invalid headerversion " << msg_header.headerversion << ", expected 2");
            return false;
        }
        decode(udp_buffer + 6, header_is_big_endian, msg_header.payloadlength);
        decode(udp_buffer + 8, header_is_big_endian, msg_header.payloadtype);
        if (msg_header.payloadtype != 0x0642 && msg_header.payloadtype != 0x06c2)
        {
            ROS_ERROR_STREAM("## ERROR UDPMessage::decodeHeader: invalid payloadtype " << msg_header.headerversion << ", expected 0x0642 or 0x06c2");
            return false;
        }
        decode(udp_buffer + 10, header_is_big_endian, msg_header.msgtype);
        decode(udp_buffer + 12, header_is_big_endian, msg_header.msgtypeversion);
        decode(udp_buffer + 14, header_is_big_endian, msg_header.sourceid);
        return true;
    }
    return false;
}

/*
** @brief Encodes the header of a udp message
** @param[in] payload_size length of payload in byte
** @param[in] payload_big_endian payload encoded in big endian (true) or little endian (false)
** @param[out] encoded message header
*/
std::vector<uint8_t> sick_lidar_localization::UDPMessage::encodeHeader(size_t payload_size, bool header_big_endian, bool payload_big_endian, uint16_t msgtype, uint16_t msgtypeversion, uint16_t source_id, int lidar_loc_version)
{

    if(lidar_loc_version == 2)
    {
        // Encode Header according to lidar localization specification version 2 (default)
        std::vector<uint8_t> buffer(16); // header has always 16 byte
        uint8_t magic[4] = { 0x4d, 0x4f, 0x50, 0x53 }; // magic field according to specification LocalizationController 2.0
        for(int n = 0; n < 4; n++)
            buffer[n] = magic[n]; // 4 byte Magic, always { 0x4d, 0x4f, 0x50, 0x53 } ("MOPS")
        encode(0x02, &buffer[0] + 4, 2, header_big_endian); // 2 byte version, always 0x02
        encode(payload_size, &buffer[0] + 6, 2, header_big_endian); // 2 byte payload length
        uint16_t payload_endianness = (payload_big_endian ? 0x0642 : 0x06c2);  // 2 byte endianness, 0x0642 for big endian or 0x06c2 for little endian
        encode(payload_endianness, &buffer[0] + 8, 2, header_big_endian); // 2 byte payload length
        encode(msgtype, &buffer[0] + 10, 2, header_big_endian); // 2 byte message type
        encode(msgtypeversion, &buffer[0] + 12, 2, header_big_endian); // 2 byte message version
        encode(source_id, &buffer[0] + 14, 2, header_big_endian); // 2 byte source id
        return buffer;
    }
    else if(lidar_loc_version == 1)
    {
        // Encode Header according to lidar localization specification version 1 (for backward compatibility only)
        std::vector<uint8_t> buffer(14); // 14 byte header
        bool header_is_big_endian = true; // the header itself is always encoded in big endian (payload is little or big endian)
        // MagicWord Char32 SMOP 4 byte (SMOP: Sick Mobile Platforms UDP telegram)
        buffer[0] = 'S';
        buffer[1] = 'M';
        buffer[2] = 'O';
        buffer[3] = 'P';
        // Header: 2 byte PayloadLength UInt16: Number of bytes per payload
        encode(payload_size, &buffer[0] + 4, 2, header_is_big_endian);
        // Header: 2 byte PayloadType (endianness) UInt16 (BE: 0x0642, LE: 0x06c2)
        encode((payload_big_endian ? 0x0642 : 0x06C2), &buffer[0] + 6, 2, header_is_big_endian);
        // Header: 2 byte MsgType UInt16: For odometry messages, MsgType = 1 is used
        encode(msgtype, &buffer[0] + 8, 2, header_is_big_endian);
        // Header: 2 byte MsgTypeVersion UInt16: Version of the MsgType payload (previously only MsgTypeVersion = 1)
        encode(msgtypeversion, &buffer[0] + 10, 2, header_is_big_endian);
        // Header: 2 byte SourceID UInt16: e.g. vehicle controller 1
        encode(source_id, &buffer[0] + 12, 2, header_is_big_endian);
        return buffer;
    }
    else
    {
        ROS_ERROR_STREAM("## ERROR UDPMessage::encodeHeader(msgtype=" << msgtype << ", msgtypeversion=" << msgtypeversion << "): lidar_loc_version = " << lidar_loc_version << " not supported, expected version 2 (default) or version 1 (backward compatibility)");
        return std::vector<uint8_t>();
    }
}

/*
** @brief Decodes the payload of a udp message. Implementation for odometry payload message type 1 version 4 (24 byte payload)
** @param[in] udp_buffer payload bytes received by udp
** @param[in] udp_buffer payload buffer size in byte
** @param[in] big_endian true: assume big endian payload, otherwise assume little endian payload
** @return true on success or false in case of errors
*/
template<> bool sick_lidar_localization::UDPMessage::Payload<sick_lidar_localization::UDPMessage::OdometryPayload0104>::decodePayload(const uint8_t* udp_buffer, int udp_buffer_len, bool big_endian)
{
    if (udp_buffer_len + 24 < sizeof(m_payload_data)) // sizeof(udp payload) + 24 byte sync_time from Software-PLL
    {
        ROS_ERROR_STREAM("## ERROR UDPMessage::Payload<UDPMessage::OdometryPayload0104>::decodePayload(): udp_buffer_len = " << udp_buffer_len << " bytes, expected at least " << sizeof(m_payload_data) << " byte");
        return false;
    }
    clearSyncTimestamp(); // invalidate the synchronized timestamp calculated by software pll
    decode(udp_buffer + 0, big_endian, m_payload_data.telegram_count);
    decode(udp_buffer + 8, big_endian, m_payload_data.timestamp);
    decode(udp_buffer + 16, big_endian, m_payload_data.x_velocity);
    decode(udp_buffer + 18, big_endian, m_payload_data.y_velocity);
    decode(udp_buffer + 20, big_endian, m_payload_data.angular_velocity);
    return true;
}

#define OdometryPayload0104UdpInputMsgLidarLocVersion 2 // Note: odometry message type 1 version 4 (localization controller udp input message) in lidar loc version 2 message format (specifiation) or 1 (backward compatibility)

/** @brief Encodes the header of a OdometryPayload0104 message */
template<> std::vector<uint8_t> sick_lidar_localization::UDPMessage::encodeHeader(const sick_lidar_localization::UDPMessage::OdometryPayload0104& message_payload, size_t payload_size, bool header_big_endian, bool payload_big_endian, uint16_t source_id)
{
    int lidar_loc_version = OdometryPayload0104UdpInputMsgLidarLocVersion; // should be version 2 according to lidar loc specification
    return sick_lidar_localization::UDPMessage::encodeHeader(payload_size, header_big_endian, payload_big_endian, 1, 4, source_id, lidar_loc_version);
}

/*
** @brief Encodes the payload of a OdometryPayload0104 udp message (odometry payload message type 1 version 4 (24 byte payload)
** @param[in] message_payload payload
** @param[in] encode_big_endian encode in big or little endiang
** @param[out] encoded vector of bytes
*/
template<> std::vector<uint8_t> sick_lidar_localization::UDPMessage::encodePayload(const sick_lidar_localization::UDPMessage::OdometryPayload0104& message_payload, bool encode_big_endian)
{
    int lidar_loc_version = OdometryPayload0104UdpInputMsgLidarLocVersion; // should be version 2 according to lidar loc specification
    if(lidar_loc_version == 2)
    {
        // Encode payload according to lidar localization specification version 2 (default)
        std::vector<uint8_t> buffer(24);
        encode((uint64_t)message_payload.telegram_count,   &buffer[0] + 0, 8, encode_big_endian);
        encode((uint64_t)message_payload.timestamp,        &buffer[0] + 8, 8, encode_big_endian);
        encode((uint64_t)message_payload.x_velocity,       &buffer[0] + 16, 2, encode_big_endian);
        encode((uint64_t)message_payload.y_velocity,       &buffer[0] + 18, 2, encode_big_endian);
        encode((uint64_t)message_payload.angular_velocity, &buffer[0] + 20, 4, encode_big_endian);
        return buffer;
    }
    else if(lidar_loc_version == 1)
    {
        // Encode payload according to lidar localization specification version 1 (for backward compatibility only)
        std::vector<uint8_t> buffer(16);
        // Payload: 4 byte TelegramCount UInt32
        encode((uint64_t)message_payload.telegram_count,   &buffer[0] + 0, 4, encode_big_endian);
        // Payload: 4 byte Timestamp UInt32 in ms
        encode((uint64_t)message_payload.timestamp,        &buffer[0] + 4, 4, encode_big_endian);
        // Payload: 2 byte vx Int16 in mm/s
        encode((uint64_t)message_payload.x_velocity,       &buffer[0] + 8, 2, encode_big_endian);
        // Payload: 2 byte vy Int16 in mm/s
        encode((uint64_t)message_payload.y_velocity,       &buffer[0] + 10, 2, encode_big_endian);
        // Payload: 4 byte omega Int32 in mdeg/s
        encode((uint64_t)message_payload.angular_velocity, &buffer[0] + 12, 4, encode_big_endian);
        return buffer;
    }
    else
    {
        ROS_ERROR_STREAM("## ERROR UDPMessage::encodePayload(OdometryPayload0104): lidar_loc_version = " << lidar_loc_version << " not supported, expected version 2 (default) or version 1 (backward compatibility)");
        return std::vector<uint8_t>();
    }
}

/*
** @brief Print the payload data. Implementation for odometry payload message type 1 version 4 (24 byte payload)
** @return payload data as human readable string
*/
template<> std::string sick_lidar_localization::UDPMessage::printPayload(const sick_lidar_localization::UDPMessage::OdometryPayload0104& payload, bool print_sync_time)
{
    std::stringstream s;
    s << "OdometryPayload0104:{"
        << "telegram_count:" << payload.telegram_count
        << ", timestamp:" << payload.timestamp
        << ", source_id:" << payload.source_id
        << ", x_velocity:" << payload.x_velocity
        << ", y_velocity:" << payload.y_velocity
        << ", angular_velocity:" << payload.angular_velocity
        << printSyncTime(payload, print_sync_time)
        << "}";
    return s.str();
}

/*
** @brief Decodes the payload of a udp message. Implementation for odometry payload message type 1 version 5 (40 byte payload)
** @param[in] udp_buffer payload bytes received by udp
** @param[in] udp_buffer payload buffer size in byte
** @param[in] big_endian true: assume big endian payload, otherwise assume little endian payload
** @return true on success or false in case of errors
*/
template<> bool sick_lidar_localization::UDPMessage::Payload<sick_lidar_localization::UDPMessage::OdometryPayload0105>::decodePayload(const uint8_t* udp_buffer, int udp_buffer_len, bool big_endian)
{
    if (udp_buffer_len + 24 < sizeof(m_payload_data)) // sizeof(udp payload) + 24 byte sync_time from Software-PLL
    {
        ROS_ERROR_STREAM("## ERROR UDPMessage::Payload<UDPMessage::OdometryPayload0105>::decodePayload(): udp_buffer_len = " << udp_buffer_len << " bytes, expected at least " << sizeof(m_payload_data) << " byte");
        return false;
    }
    clearSyncTimestamp(); // invalidate the synchronized timestamp calculated by software pll
    decode(udp_buffer + 0, big_endian, m_payload_data.telegram_count);
    decode(udp_buffer + 8, big_endian, m_payload_data.timestamp);
    decode(udp_buffer + 16, big_endian, m_payload_data.x_position);
    decode(udp_buffer + 24, big_endian, m_payload_data.y_position);
    decode(udp_buffer + 32, big_endian, m_payload_data.heading);
    return true;
}

/** @brief Encodes the header of a OdometryPayload0105 message */
template<> std::vector<uint8_t> sick_lidar_localization::UDPMessage::encodeHeader(const sick_lidar_localization::UDPMessage::OdometryPayload0105& message_payload, size_t payload_size, bool header_big_endian, bool payload_big_endian, uint16_t source_id)
{
    return sick_lidar_localization::UDPMessage::encodeHeader(payload_size, header_big_endian, payload_big_endian, 1, 5, source_id);
}

/*
** @brief Encodes the payload of a OdometryPayload0105 udp message (odometry payload message type 1 version 5 (40 byte payload)
** @param[in] message_payload payload
** @param[in] encode_big_endian encode in big or little endiang
** @param[out] encoded vector of bytes
*/
template<> std::vector<uint8_t> sick_lidar_localization::UDPMessage::encodePayload(const sick_lidar_localization::UDPMessage::OdometryPayload0105& message_payload, bool encode_big_endian)
{
    std::vector<uint8_t> buffer(40);
    encode((uint64_t)message_payload.telegram_count, &buffer[0] +  0, 8, encode_big_endian);
    encode((uint64_t)message_payload.timestamp,      &buffer[0] +  8, 8, encode_big_endian);
    encode((uint64_t)message_payload.x_position,     &buffer[0] + 16, 8, encode_big_endian);
    encode((uint64_t)message_payload.y_position,     &buffer[0] + 24, 8, encode_big_endian);
    encode((uint64_t)message_payload.heading,        &buffer[0] + 32, 8, encode_big_endian);
    return buffer;
}

/*
** @brief Print the payload data. Implementation for odometry payload message type 1 version 5 (40 byte payload)
** @return payload data as human readable string
*/
template<> std::string sick_lidar_localization::UDPMessage::printPayload(const sick_lidar_localization::UDPMessage::OdometryPayload0105& payload, bool print_sync_time)
{
    std::stringstream s;
    s << "OdometryPayload0105:{"
        << "telegram_count:" << payload.telegram_count
        << ", timestamp:" << payload.timestamp
        << ", source_id:" << payload.source_id
        << ", x_position:" << payload.x_position
        << ", y_position:" << payload.y_position
        << ", heading:" << payload.heading
        << printSyncTime(payload, print_sync_time)
        << "}";
    return s.str();
}

/** @brief Encodes the header of a EncoderMeasurementPayload0202 message */
template<> std::vector<uint8_t> sick_lidar_localization::UDPMessage::encodeHeader(const sick_lidar_localization::UDPMessage::EncoderMeasurementPayload0202& message_payload, size_t payload_size, bool header_big_endian, bool payload_big_endian, uint16_t source_id)
{
    return sick_lidar_localization::UDPMessage::encodeHeader(payload_size, header_big_endian, payload_big_endian, 2, 2, source_id);
}

/*
** @brief Encodes the payload of a EncoderMeasurementPayload0202 udp message (encoder measurement payload message type 2 version 2 (24 byte payload)
** @param[in] message_payload payload
** @param[in] encode_big_endian encode in big or little endiang
** @param[out] encoded vector of bytes
*/
template<> std::vector<uint8_t> sick_lidar_localization::UDPMessage::encodePayload(const sick_lidar_localization::UDPMessage::EncoderMeasurementPayload0202& message_payload, bool encode_big_endian)
{
    std::vector<uint8_t> buffer(24);
    encode((uint64_t)message_payload.telegram_count, &buffer[0] +  0, 8, encode_big_endian);
    encode((uint64_t)message_payload.timestamp,      &buffer[0] +  8, 8, encode_big_endian);
    encode((uint64_t)message_payload.encoder_value,  &buffer[0] + 16, 8, encode_big_endian);
    return buffer;
}

/*
** @brief Print the payload data. Implementation for encoder measurement payload message type 2 version 2
** @return payload data as human readable string
*/
template<> std::string sick_lidar_localization::UDPMessage::printPayload(const sick_lidar_localization::UDPMessage::EncoderMeasurementPayload0202& payload, bool print_sync_time)
{
    std::stringstream s;
    s << "EncoderMeasurementPayload0202:{"
        << "telegram_count:" << payload.telegram_count
        << ", timestamp:" << payload.timestamp
        << ", source_id:" << payload.source_id
        << ", code:" << payload.encoder_value
        << "}";
    return s.str();
}

/** @brief Encodes the header of a CodeMeasurementPayload0303 message */
template<> std::vector<uint8_t> sick_lidar_localization::UDPMessage::encodeHeader(const sick_lidar_localization::UDPMessage::CodeMeasurementPayload0303& message_payload, size_t payload_size, bool header_big_endian, bool payload_big_endian, uint16_t source_id)
{
    return sick_lidar_localization::UDPMessage::encodeHeader(payload_size, header_big_endian, payload_big_endian, 3, 3, source_id);
}

/*
** @brief Encodes the payload of a CodeMeasurementPayload0303 udp message (code measurement payload message type 3 version 3 (24 byte payload)
** @param[in] message_payload payload
** @param[in] encode_big_endian encode in big or little endiang
** @param[out] encoded vector of bytes
*/
template<> std::vector<uint8_t> sick_lidar_localization::UDPMessage::encodePayload(const sick_lidar_localization::UDPMessage::CodeMeasurementPayload0303& message_payload, bool encode_big_endian)
{
    std::vector<uint8_t> buffer(24);
    encode((uint64_t)message_payload.telegram_count, &buffer[0] + 0, 8, encode_big_endian);
    encode((uint64_t)message_payload.timestamp,      &buffer[0] + 8, 8, encode_big_endian);
    encode((uint64_t)message_payload.code,           &buffer[0] + 16, 8, encode_big_endian);
    return buffer;
}

/*
** @brief Print the payload data. Implementation for code measurement payload message type 3 version 3
** @return payload data as human readable string
*/
template<> std::string sick_lidar_localization::UDPMessage::printPayload(const sick_lidar_localization::UDPMessage::CodeMeasurementPayload0303& payload, bool print_sync_time)
{
    std::stringstream s;
    s << "CodeMeasurementPayload0303:{"
        << "telegram_count:" << payload.telegram_count
        << ", timestamp:" << payload.timestamp
        << ", source_id:" << payload.source_id
        << ", code:" << payload.code
        << "}";
    return s.str();
}

/** @brief Encodes the header of a CodeMeasurementPayload0701 message */
template<> std::vector<uint8_t> sick_lidar_localization::UDPMessage::encodeHeader(const sick_lidar_localization::UDPMessage::CodeMeasurementPayload0701& message_payload, size_t payload_size, bool header_big_endian, bool payload_big_endian, uint16_t source_id)
{
    return sick_lidar_localization::UDPMessage::encodeHeader(payload_size, header_big_endian, payload_big_endian, 7, 1, source_id);
}

/*
** @brief Encodes the payload of a CodeMeasurementPayload0701 udp message (code measurement payload message type 7 version 1 (variable length payload)
** @param[in] message_payload payload
** @param[in] encode_big_endian encode in big or little endiang
** @param[out] encoded vector of bytes
*/
template<> std::vector<uint8_t> sick_lidar_localization::UDPMessage::encodePayload(const sick_lidar_localization::UDPMessage::CodeMeasurementPayload0701& message_payload, bool encode_big_endian)
{
    int buffer_length = (int)(sizeof(message_payload.telegram_count) + sizeof(message_payload.timestamp) + 2 + message_payload.code.length() 
        + sizeof(message_payload.x_position) + sizeof(message_payload.y_position) + sizeof(message_payload.heading));
    std::vector<uint8_t> buffer(buffer_length);

    uint8_t* p_data = &buffer[0];
    int len = sizeof(message_payload.telegram_count);
    encode((uint64_t)message_payload.telegram_count, p_data, len, encode_big_endian);
    p_data += len;

    len = sizeof(message_payload.timestamp);
    encode((uint64_t)message_payload.timestamp, p_data, len, encode_big_endian);
    p_data += len;

    uint16_t codelength = (uint16_t)message_payload.code.length();
    len = sizeof(codelength);
    encode((uint16_t)codelength, p_data, len, encode_big_endian);
    p_data += len;

    for(int n = 0; n < codelength; n++, p_data++)
    {
        *p_data = (uint8_t)message_payload.code[n];
    }

    len = sizeof(message_payload.x_position);
    encode((uint64_t)message_payload.x_position, p_data, len, encode_big_endian);
    p_data += len;

    len = sizeof(message_payload.y_position);
    encode((uint64_t)message_payload.y_position, p_data, len, encode_big_endian);
    p_data += len;

    len = sizeof(message_payload.heading);
    encode((uint64_t)message_payload.heading, p_data, len, encode_big_endian);
    p_data += len;

    return buffer;
}

/*
** @brief Print the payload data. Implementation for code measurement payload message type 7 version 1
** @return payload data as human readable string
*/
template<> std::string sick_lidar_localization::UDPMessage::printPayload(const sick_lidar_localization::UDPMessage::CodeMeasurementPayload0701& payload, bool print_sync_time)
{
    std::stringstream s;
    s << "CodeMeasurementPayload0701:{"
        << "telegram_count:" << payload.telegram_count
        << ", timestamp:" << payload.timestamp
        << ", source_id:" << payload.source_id
        << ", code:" << payload.code
        << ", x_position:" << payload.x_position
        << ", y_position:" << payload.y_position
        << ", heading:" << payload.heading
        << "}";
    return s.str();
}

/*
** @brief Decodes the payload of a udp message. Implementation for code measurement payload message type 3 version 4 (24 byte payload)
** @param[in] udp_buffer payload bytes received by udp
** @param[in] udp_buffer payload buffer size in byte
** @param[in] big_endian true: assume big endian payload, otherwise assume little endian payload
** @return true on success or false in case of errors
*/
template<> bool sick_lidar_localization::UDPMessage::Payload<sick_lidar_localization::UDPMessage::CodeMeasurementPayload0304>::decodePayload(const uint8_t* udp_buffer, int udp_buffer_len, bool big_endian)
{
    if (udp_buffer_len + 24 < sizeof(m_payload_data)) // sizeof(udp payload) + 24 byte sync_time from Software-PLL
    {
        ROS_ERROR_STREAM("## ERROR UDPMessage::Payload<UDPMessage::CodeMeasurementPayload0304>::decodePayload(): udp_buffer_len = " << udp_buffer_len << " bytes, expected at least " << sizeof(m_payload_data) << " byte");
        return false;
    }
    clearSyncTimestamp(); // invalidate the synchronized timestamp calculated by software pll
    decode(udp_buffer + 0, big_endian, m_payload_data.telegram_count);
    decode(udp_buffer + 8, big_endian, m_payload_data.timestamp);
    decode(udp_buffer + 16, big_endian, m_payload_data.code);
    decode(udp_buffer + 20, big_endian, m_payload_data.distance);
    return true;
}

/*
** @brief Print the payload data. Implementation for code measurement payload message type 3 version 4 (24 byte payload)
** @return payload data as human readable string
*/
template<> std::string sick_lidar_localization::UDPMessage::printPayload(const sick_lidar_localization::UDPMessage::CodeMeasurementPayload0304& payload, bool print_sync_time)
{
    std::stringstream s;
    s << "CodeMeasurementPayload0304:{"
        << "telegram_count:" << payload.telegram_count
        << ", timestamp:" << payload.timestamp
        << ", source_id:" << payload.source_id
        << ", code:" << payload.code
        << ", distance:" << payload.distance
        << printSyncTime(payload, print_sync_time)
        << "}";
    return s.str();
}

/*
** @brief Decodes the payload of a udp message. Implementation for line measurement payload message type 4 version 3 (variable length payload)
** @param[in] udp_buffer payload bytes received by udp
** @param[in] udp_buffer payload buffer size in byte
** @param[in] big_endian true: assume big endian payload, otherwise assume little endian payload
** @return true on success or false in case of errors
*/
template<> bool sick_lidar_localization::UDPMessage::Payload<sick_lidar_localization::UDPMessage::LineMeasurementPayload0403>::decodePayload(const uint8_t* udp_buffer, int udp_buffer_len, bool big_endian)
{
    if (udp_buffer_len < 17)
    {
        ROS_ERROR_STREAM("## ERROR UDPMessage::Payload<UDPMessage::LineMeasurementPayload0403>::decodePayload(): udp_buffer_len = " << udp_buffer_len << " bytes, expected at least 17 byte");
        return false;
    }
    clearSyncTimestamp(); // invalidate the synchronized timestamp calculated by software pll
    decode(udp_buffer + 0, big_endian, m_payload_data.telegram_count);
    decode(udp_buffer + 8, big_endian, m_payload_data.timestamp);
    decode(udp_buffer + 16, big_endian, m_payload_data.num_lanes);
    m_payload_data.lanes.reserve(m_payload_data.num_lanes);
    if (udp_buffer_len < 17 + 2 * m_payload_data.num_lanes)
    {
        ROS_ERROR_STREAM("## ERROR UDPMessage::Payload<UDPMessage::LineMeasurementPayload0403>::decodePayload(): udp_buffer_len = " << udp_buffer_len << " bytes, num_lanes = " << m_payload_data.num_lanes << ", expected at least " << (17 + 2 * m_payload_data.num_lanes) << " byte");
        return false;
    }
    for (int n = 0; n < m_payload_data.num_lanes; n++)
    {
        int16_t lane = 0;
        decode(udp_buffer + 17 + 2 * n, big_endian, lane);
        m_payload_data.lanes.push_back(lane);
    }
    return true;
}

/** @brief Encodes the header of a LineMeasurementPayload0403 message */
template<> std::vector<uint8_t> sick_lidar_localization::UDPMessage::encodeHeader(const sick_lidar_localization::UDPMessage::LineMeasurementPayload0403& message_payload, size_t payload_size, bool header_big_endian, bool payload_big_endian, uint16_t source_id)
{
    return sick_lidar_localization::UDPMessage::encodeHeader(payload_size, header_big_endian, payload_big_endian, 4, 3, source_id);
}

/*
** @brief Encodes the payload of a LineMeasurementPayload0403 udp message (line measurement payload message type 4 version 3 (variable length payload)
** @param[in] message_payload payload
** @param[in] encode_big_endian encode in big or little endiang
** @param[out] encoded vector of bytes
*/
template<> std::vector<uint8_t> sick_lidar_localization::UDPMessage::encodePayload(const sick_lidar_localization::UDPMessage::LineMeasurementPayload0403& message_payload, bool encode_big_endian)
{
    std::vector<uint8_t> buffer(17 + 2 * message_payload.num_lanes);
    encode((uint64_t)message_payload.telegram_count, &buffer[0] +  0, 8, encode_big_endian);
    encode((uint64_t)message_payload.timestamp,      &buffer[0] +  8, 8, encode_big_endian);
    encode((uint64_t)message_payload.num_lanes,      &buffer[0] + 16, 1, encode_big_endian);
    for (int n = 0; n < message_payload.num_lanes; n++)
    {
        encode(message_payload.lanes[n], &buffer[0] + 17 + 2 * n, 2, encode_big_endian);
    }
    return buffer;
}

/*
** @brief Print the payload data. Implementation for line measurement payload message type 4 version 3 (variable length payload)
** @return payload data as human readable string
*/
template<> std::string sick_lidar_localization::UDPMessage::printPayload(const sick_lidar_localization::UDPMessage::LineMeasurementPayload0403& payload, bool print_sync_time)
{
    std::stringstream s;
    s << "LineMeasurementPayload0403:{"
        << "telegram_count:" << payload.telegram_count
        << ", timestamp:" << payload.timestamp
        << ", source_id:" << payload.source_id
        << ", num_lanes:" << (int)payload.num_lanes
        << ", lanes:[";
    for (int n = 0; n < payload.num_lanes; n++)
    {
        if (n > 0)
            s << ",";
        s << (int)payload.lanes[n];
    }
    s << "]";
    s << printSyncTime(payload, print_sync_time);
    s << "}";
    return s.str();
}

/*
** @brief Decodes the payload of a udp message. Implementation for line measurement payload message type 4 version 4 (24 byte payload)
** @param[in] udp_buffer payload bytes received by udp
** @param[in] udp_buffer payload buffer size in byte
** @param[in] big_endian true: assume big endian payload, otherwise assume little endian payload
** @return true on success or false in case of errors
*/
template<> bool sick_lidar_localization::UDPMessage::Payload<sick_lidar_localization::UDPMessage::LineMeasurementPayload0404>::decodePayload(const uint8_t* udp_buffer, int udp_buffer_len, bool big_endian)
{
    if (udp_buffer_len + 24 < sizeof(m_payload_data)) // sizeof(udp payload) + 24 byte sync_time from Software-PLL
    {
        ROS_ERROR_STREAM("## ERROR UDPMessage::Payload<UDPMessage::LineMeasurementPayload0404>::decodePayload(): udp_buffer_len = " << udp_buffer_len << " bytes, expected at least " << sizeof(m_payload_data) << " byte");
        return false;
    }
    clearSyncTimestamp(); // invalidate the synchronized timestamp calculated by software pll
    decode(udp_buffer + 0, big_endian, m_payload_data.telegram_count);
    decode(udp_buffer + 8, big_endian, m_payload_data.timestamp);
    decode(udp_buffer + 16, big_endian, m_payload_data.lcp1);
    decode(udp_buffer + 18, big_endian, m_payload_data.lcp2);
    decode(udp_buffer + 20, big_endian, m_payload_data.lcp3);
    decode(udp_buffer + 22, big_endian, m_payload_data.cnt_lpc);
    m_payload_data.reserved = 0;
    return true;
}

/** @brief Encodes the header of a LineMeasurementPayload0404 message */
template<> std::vector<uint8_t> sick_lidar_localization::UDPMessage::encodeHeader(const sick_lidar_localization::UDPMessage::LineMeasurementPayload0404& message_payload, size_t payload_size, bool header_big_endian, bool payload_big_endian, uint16_t source_id)
{
    return sick_lidar_localization::UDPMessage::encodeHeader(payload_size, header_big_endian, payload_big_endian, 4, 4, source_id);
}

/*
** @brief Encodes the payload of a LineMeasurementPayload0404 udp message (line measurement payload message type 4 version 4 (24 byte payload)
** @param[in] message_payload payload
** @param[in] encode_big_endian encode in big or little endiang
** @param[out] encoded vector of bytes
*/
template<> std::vector<uint8_t> sick_lidar_localization::UDPMessage::encodePayload(const sick_lidar_localization::UDPMessage::LineMeasurementPayload0404& message_payload, bool encode_big_endian)
{
    std::vector<uint8_t> buffer(24);
    encode((uint64_t)message_payload.telegram_count, &buffer[0] + 0, 8, encode_big_endian);
    encode((uint64_t)message_payload.timestamp,      &buffer[0] + 8, 8, encode_big_endian);
    encode((uint64_t)message_payload.lcp1,           &buffer[0] + 16, 2, encode_big_endian);
    encode((uint64_t)message_payload.lcp2,           &buffer[0] + 18, 2, encode_big_endian);
    encode((uint64_t)message_payload.lcp3,           &buffer[0] + 20, 2, encode_big_endian);
    encode((uint64_t)message_payload.cnt_lpc,        &buffer[0] + 22, 1, encode_big_endian);
    buffer[23] = 0; // memory alignment
    return buffer;
}

/*
** @brief Print the payload data. Implementation for line measurement payload message type 4 version 4 (24 byte payload)
** @return payload data as human readable string
*/
template<> std::string sick_lidar_localization::UDPMessage::printPayload(const sick_lidar_localization::UDPMessage::LineMeasurementPayload0404& payload, bool print_sync_time)
{
    std::stringstream s;
    s << "LineMeasurementPayload0404:{"
        << "telegram_count:" << payload.telegram_count
        << ", timestamp:" << payload.timestamp
        << ", source_id:" << payload.source_id
        << ", lcp1:" << (int)payload.lcp1
        << ", lcp2:" << (int)payload.lcp2
        << ", lcp3:" << (int)payload.lcp3
        << ", cnt_lpc:" << (int)payload.cnt_lpc
        << printSyncTime(payload, print_sync_time)
        << "}";
    return s.str();
}

/*
** @brief Decodes the payload of a udp message. Implementation for LocalizationController result payload message type 5 version 2 (40 byte payload).
** @param[in] udp_buffer payload bytes received by udp
** @param[in] udp_buffer payload buffer size in byte
** @param[in] big_endian true: assume big endian payload, otherwise assume little endian payload
** @return true on success or false in case of errors
*/
template<> bool sick_lidar_localization::UDPMessage::Payload<sick_lidar_localization::UDPMessage::LocalizationControllerResultPayload0502>::decodePayload(const uint8_t* udp_buffer, int udp_buffer_len, bool big_endian)
{
    if (udp_buffer_len + 24 < sizeof(m_payload_data)) // sizeof(udp payload) + 24 byte sync_time from Software-PLL
    {
        ROS_ERROR_STREAM("## ERROR UDPMessage::Payload<UDPMessage::LocalizationControllerResultPayload0502>::decodePayload(): udp_buffer_len = " << udp_buffer_len << " bytes, expected at least " << sizeof(m_payload_data) << " byte");
        return false;
    }
    clearSyncTimestamp(); // invalidate the synchronized timestamp calculated by software pll
    decode(udp_buffer + 0, big_endian, m_payload_data.telegram_count);
    decode(udp_buffer + 8, big_endian, m_payload_data.timestamp);
    decode(udp_buffer + 16, big_endian, m_payload_data.x);
    decode(udp_buffer + 24, big_endian, m_payload_data.y);
    decode(udp_buffer + 32, big_endian, m_payload_data.heading);
    decode(udp_buffer + 36, big_endian, m_payload_data.loc_status);
    decode(udp_buffer + 37, big_endian, m_payload_data.map_match_status);
    m_payload_data.reserved = 0;
    return true;
}

/*
** @brief Print the payload data. Implementation for LocalizationController result payload message type 5 version 2 (40 byte payload).
** @return payload data as human readable string
*/
template<> std::string sick_lidar_localization::UDPMessage::printPayload(const sick_lidar_localization::UDPMessage::LocalizationControllerResultPayload0502& payload, bool print_sync_time)
{
    std::stringstream s;
    s << "LocalizationControllerResultPayload0502:{"
        << "telegram_count:" << payload.telegram_count
        << ", timestamp:" << payload.timestamp
        << ", source_id:" << payload.source_id
        << ", x:" << payload.x
        << ", y:" << payload.y
        << ", heading:" << payload.heading
        << ", loc_status:" << (int)payload.loc_status
        << ", map_match_status:" << (int)payload.map_match_status
        << printSyncTime(payload, print_sync_time)
        << "}";
    return s.str();
}

static std::string printHex(const std::vector<uint8_t>& data)
{
    std::stringstream s;
    for(int n = 0; n < data.size(); n++)
        s << std::hex << std::setfill('0') << std::setw(2) << (int)(data[n]);
    return s.str();
}

template<class T> static void decodeAndPrint(std::vector<uint8_t>& reference_bin)
{
    sick_lidar_localization::UDPMessage::HeaderData msg_header = { 0 };
    sick_lidar_localization::UDPMessage::decodeHeader(&reference_bin[0], (int)reference_bin.size(), msg_header);
    bool payload_is_big_endian = (msg_header.payloadtype == 0x0642);
    uint8_t* udp_payload_buffer = &reference_bin[0] + sizeof(sick_lidar_localization::UDPMessage::HeaderData);
    int udp_payload_len = (int)reference_bin.size() - (int)sizeof(sick_lidar_localization::UDPMessage::HeaderData);
    sick_lidar_localization::UDPMessage::Payload<T> reference_payload;
    reference_payload.decodePayload(udp_payload_buffer, udp_payload_len, payload_is_big_endian);
    ROS_INFO_STREAM("UDPMessage::unittest: payload_reference = " << reference_payload.toString(false) << ", payload_big_endian = " << payload_is_big_endian << ", source_id = " << msg_header.sourceid);
}

/*
** @brief Unittest for UDP message encoding
*/
bool sick_lidar_localization::UDPMessage::unittest(void)
{
    bool okay = true;
    uint16_t source_id = 1; // should be 31
    std::vector<uint8_t> message, reference_bin;

    // Unittest for odometry message type 1 version 4 (little endian)
    OdometryPayload0104 payload0104;
    payload0104.telegram_count = 1; // 8 byte TelegramCount uint64
    payload0104.timestamp = 1632381121881116; // 8 byte Timestamp uint64 [microseconds]
    payload0104.x_velocity = 130; // 2 byte X - velocity int16 [mm/s]
    payload0104.y_velocity = -120; // 2 byte Y - velocity int16 [mm/s]
    payload0104.angular_velocity = 300; // 4 byte angular velocity int32 [mdeg/s]
    reference_bin = { 0x4d, 0x4f, 0x50, 0x53, 0x02, 0x00, 0x18, 0x00, 0xc2, 0x06, 0x01, 0x00, 0x04, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x64, 0xe8, 0x58, 0xa4, 0xcc, 0x05, 0x00, 0x82, 0x00, 0x88, 0xff, 0x2c, 0x01, 0x00, 0x00 };
    message = sick_lidar_localization::UDPMessage::encodeMessage(payload0104, false, false, source_id);
    if (reference_bin.size() != message.size() || memcmp(&message[0], &reference_bin[0], message.size()) != 0)
    {
        ROS_ERROR_STREAM("## ERROR unittest OdometryPayload0104 little endian:" << "\n## encoded:  " << printHex(message) << "\n## expected: " << printHex(reference_bin));
        okay = false;
        decodeAndPrint<sick_lidar_localization::UDPMessage::OdometryPayload0104>(reference_bin);
    }
    else
    {
        ROS_INFO_STREAM("UDPMessage::unittest passed: OdometryPayload0104 little endian = " << printHex(message));
    }

    // Unittest for odometry message type 1 version 4 (big endian)
    payload0104.timestamp = 1632381121913034; // 8 byte Timestamp uint64 [microseconds]
    reference_bin = { 0x4d, 0x4f, 0x50, 0x53, 0x00, 0x02, 0x00, 0x18, 0x06, 0x42, 0x00, 0x01, 0x00, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x05, 0xcc, 0xa4, 0x58, 0xe8, 0xe0, 0xca, 0x00, 0x82, 0xff, 0x88, 0x00, 0x00, 0x01, 0x2c };
    message = sick_lidar_localization::UDPMessage::encodeMessage(payload0104, true, true, source_id);
    if (reference_bin.size() != message.size() || memcmp(&message[0], &reference_bin[0], message.size()) != 0)
    {
        ROS_ERROR_STREAM("## ERROR unittest OdometryPayload0104 big endian:" << "\n## encoded:  " << printHex(message) << "\n## expected: " << printHex(reference_bin));
        okay = false;
        decodeAndPrint<sick_lidar_localization::UDPMessage::OdometryPayload0104>(reference_bin);
    }
    else
    {
        ROS_INFO_STREAM("UDPMessage::unittest passed: OdometryPayload0104 big endian = " << printHex(message));
    }

    // Unittest for odometry message type 1 version 5 (little endian)
    OdometryPayload0105 payload0105;
    payload0105.telegram_count = 1; // 8 byte TelegramCount uint64
    payload0105.timestamp = 1632381121898072; // 8 byte Timestamp uint64 [microseconds]
    payload0105.x_position = -123; // 8 byte X - position [mm]
    payload0105.y_position = -432; // 8 byte Y - position [mm]
    payload0105.heading = 5432; // 8 byte heading [mdeg]
    reference_bin = { 0x4d, 0x4f, 0x50, 0x53, 0x02, 0x00, 0x28, 0x00, 0xc2, 0x06, 0x01, 0x00, 0x05, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0xa6, 0xe8, 0x58, 0xa4, 0xcc, 0x05, 0x00, 0x85, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x50, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x38, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    message = sick_lidar_localization::UDPMessage::encodeMessage(payload0105, false, false, source_id);
    if (reference_bin.size() != message.size() || memcmp(&message[0], &reference_bin[0], message.size()) != 0)
    {
        ROS_ERROR_STREAM("## ERROR unittest OdometryPayload0105 little endian:" << "\n## encoded:  " << printHex(message) << "\n## expected: " << printHex(reference_bin));
        okay = false;
        decodeAndPrint<sick_lidar_localization::UDPMessage::OdometryPayload0105>(reference_bin);
    }
    else
    {
        ROS_INFO_STREAM("UDPMessage::unittest passed: OdometryPayload0105 little endian = " << printHex(message));
    }

    // Unittest for odometry message type 1 version 5 (big endian)
    payload0105.timestamp = 1632381121927992; // 8 byte Timestamp uint64 [microseconds]
    reference_bin = { 0x4d, 0x4f, 0x50, 0x53, 0x00, 0x02, 0x00, 0x28, 0x06, 0x42, 0x00, 0x01, 0x00, 0x05, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x05, 0xcc, 0xa4, 0x58, 0xe9, 0x1b, 0x38, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x85, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x38 };
    message = sick_lidar_localization::UDPMessage::encodeMessage(payload0105, true, true, source_id);
    if (reference_bin.size() != message.size() || memcmp(&message[0], &reference_bin[0], message.size()) != 0)
    {
        ROS_ERROR_STREAM("## ERROR unittest OdometryPayload0105 big endian:" << "\n## encoded:  " << printHex(message) << "\n## expected: " << printHex(reference_bin));
        okay = false;
        decodeAndPrint<sick_lidar_localization::UDPMessage::OdometryPayload0105>(reference_bin);
    }
    else
    {
        ROS_INFO_STREAM("UDPMessage::unittest passed: OdometryPayload0105 big endian = " << printHex(message));
    }

    if(!okay)
    {
        ROS_ERROR_STREAM("## ERROR sick_lidar_localization::UDPMessage::unittest failed...");
        // exit(EXIT_FAILURE);
    }
    // exit(EXIT_SUCCESS);
    return okay;
}
