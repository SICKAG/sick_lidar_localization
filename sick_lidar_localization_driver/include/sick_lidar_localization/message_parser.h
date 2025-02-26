/*
 * @brief message_parser encodes and decodes input and output messages.
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
#ifndef __SICK_LIDAR_LOCALIZATION_MESSAGE_PARSER_H_INCLUDED
#define __SICK_LIDAR_LOCALIZATION_MESSAGE_PARSER_H_INCLUDED

#include <list>
#include <string>
#include "sick_lidar_localization/sick_common.h"

#define HEADER_BIG_ENDIAN_DEFAULT false
#define PAYLOAD_BIG_ENDIAN_DEFAULT false

namespace sick_lidar_localization
{
	class Message
	{
	public:

        /*
        ** @brief Container for the header data of all messages (input and output).
        */
        struct HeaderData
        {
            uint8_t magic[4];            // 4 byte Magic, always { 0x4d, 0x4f, 0x50, 0x53 } ("MOPS") resp. { 0x53, 0x50, 0x4f, 0x4d }
            uint16_t headerversion;      // 2 byte version, always 0x02
            uint16_t payloadlength;      // 2 byte payload length
            uint16_t payloadtype;        // 2 byte endianness, 0x0642 for big endian or 0x06c2 for little endian
            uint16_t msgtype;            // 2 byte message type
            uint16_t msgtypeversion;     // 2 byte message version
            uint16_t sourceid;           // 2 byte source id
        };

        /*
        ** @brief Container for the odometry payload message type 1 version 4 (24 byte payload, input and output)
        */
        struct OdometryPayload0104
        {
            uint64_t telegram_count;     // 8 byte TelegramCount uint64
            uint64_t timestamp;          // 8 byte Timestamp uint64 [microseconds]
            int32_t source_id;           // 4 byte source id
            int16_t x_velocity;          // 2 byte X - velocity int16 [mm/s]
            int16_t y_velocity;          // 2 byte Y - velocity int16 [mm/s]
            int32_t angular_velocity;    // 4 byte angular velocity int32 [mdeg/s]
            uint32_t sync_timestamp_sec;   // seconds part of synchronized timestamp in system time calculated by Software-PLL
            uint32_t sync_timestamp_nsec;  // nanoseconds part of synchronized timestamp in system time calculated by Software-PLL
            uint32_t sync_timestamp_valid; // 1: sync_timestamp successfully calculated by Software-PLL, 0: Software-PLL not yet synchronized
        };

        /*
        ** @brief Container for the odometry payload message type 1 version 5 (40 byte payload, input and output)
        */
        struct OdometryPayload0105
        {
            uint64_t telegram_count;     // 8 byte TelegramCount uint64
            uint64_t timestamp;          // 8 byte Timestamp uint64 [microseconds]
            int32_t source_id;           // 4 byte source id
            int64_t x_position;          // 8 byte X-position int64 [mm]
            int64_t y_position;          // 8 byte Y-position int64 [mm]
            int64_t heading;             // 8 byte heading int32 [mdeg]
            uint32_t sync_timestamp_sec;   // seconds part of synchronized timestamp in system time calculated by Software-PLL
            uint32_t sync_timestamp_nsec;  // nanoseconds part of synchronized timestamp in system time calculated by Software-PLL
            uint32_t sync_timestamp_valid; // 1: sync_timestamp successfully calculated by Software-PLL, 0: Software-PLL not yet synchronized
        };

        /*
        ** @brief Container for the encoder measurement payload message type 2 version 2 (24 byte payload, input message)
        */
        struct EncoderMeasurementPayload0202
        {
            uint64_t telegram_count;     // 8 byte TelegramCount uint64
            uint64_t timestamp;          // 8 byte Timestamp uint64 [microseconds]
            int32_t source_id;           // 4 byte source id
            int64_t encoder_value;       // 8 byte EncoderValue int64 in tics
        };

        /*
        ** @brief Container for the code measurement payload message type 3 version 3 (24 byte payload, input message)
        */
        struct CodeMeasurementPayload0303
        {
            uint64_t telegram_count;     // 8 byte TelegramCount uint64
            uint64_t timestamp;          // 8 byte Timestamp uint64 [microseconds]
            int32_t source_id;           // 4 byte source id
            int64_t code;                // 8 byte Code int32
        };

        /*
        ** @brief Container for the code measurement payload message type 7 version 1 (variable length payload, input message)
        */
        struct CodeMeasurementPayload0701
        {
            uint64_t telegram_count;     // 8 byte TelegramCount uint64
            uint64_t timestamp;          // 8 byte Timestamp uint64 [microseconds]
            int32_t source_id;           // 4 byte source id
            std::string code;            // variable length string => message: 2 byte codelength (uint16) + <codelength> byte string
            int32_t x_position;          // 4 byte relative pose along the x-axis of the sensor frame [mm]
            int32_t y_position;          // 4 byte relative pose along the y-axis of the sensor frame [mm]
            int32_t heading;             // 4 byte relative orientation of the code in the sensor frame [mdeg]
        };

        /*
        ** @brief Container for the code measurement payload message type 3 version 4 (24 byte payload, output message)
        */
        struct CodeMeasurementPayload0304
        {
            uint64_t telegram_count;     // 8 byte TelegramCount uint64
            uint64_t timestamp;          // 8 byte Timestamp uint64 [microseconds]
            int32_t source_id;           // 4 byte source id
            int32_t code;                // 4 byte Code int32
            int32_t distance;            // 4 byte Distance int32 [mm]
            uint32_t sync_timestamp_sec;   // seconds part of synchronized timestamp in system time calculated by Software-PLL
            uint32_t sync_timestamp_nsec;  // nanoseconds part of synchronized timestamp in system time calculated by Software-PLL
            uint32_t sync_timestamp_valid; // 1: sync_timestamp successfully calculated by Software-PLL, 0: Software-PLL not yet synchronized
        };

        /*
        ** @brief Container for the line measurement payload message type 4 version 3 (variable length payload, input and output)
        */
        struct LineMeasurementPayload0403
        {
            uint64_t telegram_count;     // 8 byte TelegramCount uint64
            uint64_t timestamp;          // 8 byte Timestamp uint64 [microseconds]
            int32_t source_id;           // 4 byte source id
            uint8_t num_lanes;           // 1 byte NumOfLanes uint8
            std::vector<int16_t> lanes;  // Lanes: num_lanes * 2 byte, each lane encoded by int16
            uint32_t sync_timestamp_sec;   // seconds part of synchronized timestamp in system time calculated by Software-PLL
            uint32_t sync_timestamp_nsec;  // nanoseconds part of synchronized timestamp in system time calculated by Software-PLL
            uint32_t sync_timestamp_valid; // 1: sync_timestamp successfully calculated by Software-PLL, 0: Software-PLL not yet synchronized
        };
        
        /*
        ** @brief Container for the line measurement payload message type 4 version 4 (24 byte payload, input and output)
        */
        struct LineMeasurementPayload0404
        {
            uint64_t telegram_count;     // 8 byte TelegramCount uint64
            uint64_t timestamp;          // 8 byte Timestamp uint64 [microseconds]
            int32_t source_id;           // 4 byte source id
            int16_t lcp1;                // 2 byte lcp1 int16 [mm]
            int16_t lcp2;                // 2 byte lcp2 int16 [mm]
            int16_t lcp3;                // 2 byte lcp3 int16 [mm]
            uint8_t cnt_lpc;             // 1 byte cnt_lpc uint8
            uint8_t reserved;            // 1 byte memory alignment
            uint32_t sync_timestamp_sec;   // seconds part of synchronized timestamp in system time calculated by Software-PLL
            uint32_t sync_timestamp_nsec;  // nanoseconds part of synchronized timestamp in system time calculated by Software-PLL
            uint32_t sync_timestamp_valid; // 1: sync_timestamp successfully calculated by Software-PLL, 0: Software-PLL not yet synchronized
        };

        /*
        ** @brief Container for the LocalizationController result payload message type 5 version 2 (40 byte payload, output message)
        */
        struct LocalizationControllerResultPayload0502
        {
            uint64_t telegram_count;     // 8 byte TelegramCount uint64
            uint64_t timestamp;          // 8 byte Timestamp uint64 [microseconds]
            int32_t source_id;           // 4 byte source id
            int64_t x;                   // 8 byte X int64 [mm]
            int64_t y;                   // 8 byte Y int64 [mm]
            int32_t heading;             // 4 byte Heading int32 [mdeg]
            uint8_t loc_status;          // 1 byte LocalizationStatus [0...100, 10: OK, 20: Warning, 30: Not localized, 40: System error]
            uint8_t map_match_status;    // 1 byte MapMatchingStatus [0...100, 90: Good, 60: Medium, 30: Low, 0; None]
            uint16_t reserved;           // 2 byte memory alignment
            uint32_t sync_timestamp_sec;   // seconds part of synchronized timestamp in system time calculated by Software-PLL
            uint32_t sync_timestamp_nsec;  // nanoseconds part of synchronized timestamp in system time calculated by Software-PLL
            uint32_t sync_timestamp_valid; // 1: sync_timestamp successfully calculated by Software-PLL, 0: Software-PLL not yet synchronized
        };

        /** Returns a udp payload in a human readable string */
        template<typename T> static std::string printPayload(const T& message, bool print_sync_time = true);

        /*
        ** @brief Listener interface for messages. The callback functions will be called after receiving a new message.
        ** Register for upd messages using UDPReceiverThread::registerListener() and overwrite these functions with customized code to handle messages.
        */
        class Listener
        {
        public:
            /** callback function for odometry messages type 1 version 4 */
            virtual void messageReceived(const sick_lidar_localization::Message::OdometryPayload0104& message) {};
            /** callback function for odometry messages type 1 version 5 */
            virtual void messageReceived(const sick_lidar_localization::Message::OdometryPayload0105& message) {};
            /** callback function for code measurement messages type 3 version 4 */
            virtual void messageReceived(const sick_lidar_localization::Message::CodeMeasurementPayload0304& message) {};
            /** callback function for line measurement messages type 4 version 3 */
            virtual void messageReceived(const sick_lidar_localization::Message::LineMeasurementPayload0403& message) {};
            /** callback function for line measurement messages type 4 version 4 */
            virtual void messageReceived(const sick_lidar_localization::Message::LineMeasurementPayload0404& message) {};
            /** callback function for localizationcontroller result messages type 5 version 2 */
            virtual void messageReceived(const sick_lidar_localization::Message::LocalizationControllerResultPayload0502& message) {};
        };

        /*
        ** @brief Implements a simple listener for messages, just prints messages.
        */
        class InfoListener : public sick_lidar_localization::Message::Listener
        {
        public:
            /** default destructor */
            virtual ~InfoListener(){}
            /** callback function for odometry messages type 1 version 4 */
            virtual void messageReceived(const sick_lidar_localization::Message::OdometryPayload0104& message) { printMessageReceived(message); };
            /** callback function for odometry messages type 1 version 5 */
            virtual void messageReceived(const sick_lidar_localization::Message::OdometryPayload0105& message) { printMessageReceived(message); };
            /** callback function for code measurement messages type 3 version 4 */
            virtual void messageReceived(const sick_lidar_localization::Message::CodeMeasurementPayload0304& message) { printMessageReceived(message); };
            /** callback function for line measurement messages type 4 version 3 */
            virtual void messageReceived(const sick_lidar_localization::Message::LineMeasurementPayload0403& message) { printMessageReceived(message); };
            /** callback function for line measurement messages type 4 version 4 */
            virtual void messageReceived(const sick_lidar_localization::Message::LineMeasurementPayload0404& message) { printMessageReceived(message); };
            /** callback function for localizationcontroller result messages type 5 version 2 */
            virtual void messageReceived(const sick_lidar_localization::Message::LocalizationControllerResultPayload0502& message) { printMessageReceived(message); };
        protected:
            /** just prints a message in a human readable string */
            template<typename T> void printMessageReceived(const T& message)
            {
                ROS_INFO_STREAM("Message::InfoListener: messageReceived: " << printPayload(message));
            };
        };

        /*
        ** @brief Abstract base class for all message payload types
        */
        class PayloadBase
        {
        public:
            /** Default constructor */
            PayloadBase(){}
            /** Default destructor */
            virtual ~PayloadBase(){}
            /*
            ** @brief Decodes the payload of a message. Overwrite to decode the payload data depending on the message type and version.
            ** @param[in] buffer payload bytes received
            ** @param[in] buffer_len payload buffer size in byte
            ** @param[in] big_endian true: assume big endian payload, otherwise assume little endian payload
            ** @return true on success or false in case of errors
            */
            virtual bool decodePayload(const uint8_t* buffer, int buffer_len, bool big_endian) = 0;
            /*
            ** @brief Print the payload data. Overwrite to print the payload depending on the message type and version.
            ** @return payload data as human readable string
            */
            virtual std::string toString(bool print_sync_time = true) = 0;
            /*
            ** @brief Notifies all listeners after receiving a new message
            */
            virtual void notifyListener(std::list<sick_lidar_localization::Message::Listener*>& listeners) = 0;
            /*
            ** @brief Returns the sensor timestamp in tics
            */
            virtual uint64_t getTimestamp(void) = 0;
            /*
            ** @brief Sets the synchronized timestamp calculated by software pll
            */
            virtual void setSyncTimestamp(uint32_t sec, uint32_t nsec) = 0;
            /*
            ** @brief Get/set the source id
            */
            virtual int32_t& sourceID(void) = 0;
        };

        /*
        ** @brief Template payload class, implements decoding and publishing depending on the message type and version
        */
        template<typename T> class Payload : public PayloadBase
        {
        public:
            /** Default constructor */
            Payload() {}
            /** Initializing constructor */
            Payload(const T& payload) { m_payload_data = payload; }
            /** Default destructor */
            virtual ~Payload(){}
            /*
            ** @brief Decodes the payload of a message. Implemented depending on the message type and version.
            ** @param[in] buffer payload bytes received
            ** @param[in] buffer_len payload buffer size in byte
            ** @param[in] big_endian true: assume big endian payload, otherwise assume little endian payload
            ** @return true on success or false in case of errors
            */
            virtual bool decodePayload(const uint8_t* buffer, int buffer_len, bool big_endian);
            /*
            ** @brief Print the payload data. Overwrite to print the payload depending on the message type and version.
            ** @return payload data as human readable string
            */
            virtual std::string toString(bool print_sync_time = true) { return sick_lidar_localization::Message::printPayload(m_payload_data, print_sync_time); }
            /*
            ** @brief Notifies all listeners after receiving a new message
            */
            virtual void notifyListener(std::list<sick_lidar_localization::Message::Listener*>& listeners)
            {
                for (auto iter_listener = listeners.begin(); iter_listener != listeners.end(); iter_listener++)
                {
                    (*iter_listener)->messageReceived(m_payload_data);
                }
            }
            /*
            ** @brief Returns the sensor timestamp in tics
            */
            virtual uint64_t getTimestamp(void)
            {
                return m_payload_data.timestamp;
            }
            /*
            ** @brief Sets the synchronized timestamp calculated by software pll
            */
            virtual void setSyncTimestamp(uint32_t sec, uint32_t nsec)
            {
                m_payload_data.sync_timestamp_sec = sec;
                m_payload_data.sync_timestamp_nsec = nsec;
                m_payload_data.sync_timestamp_valid = 1;
            }
            /*
            ** @brief Get/set the source id
            */
            virtual int32_t& sourceID(void)
            {
                return m_payload_data.source_id;
            }

        protected:
            /*
            ** @brief Invalidates the synchronized timestamp calculated by software pll
            */
            virtual void clearSyncTimestamp(void)
            {
                m_payload_data.sync_timestamp_sec = 0;
                m_payload_data.sync_timestamp_nsec = 0;
                m_payload_data.sync_timestamp_valid = 0;
            }

            T m_payload_data; // decoded payload data, depending on the message type and version
        };

        /*
        ** @brief Decodes the header of a message
        ** @param[in] buffer raw bytes received
        ** @param[in] big_endian assume big endian (true) or little endian (false) encoding
        ** @param[out] msg_header decoded message header
        */
        static bool decodeHeader(const uint8_t* buffer, int bytes_received, Message::HeaderData& msg_header);

        /*
        ** @brief Encodes the header of a message
        ** @param[in] payload_size length of payload in byte
        ** @param[in] payload_big_endian payload encoded in big endian (true) or little endian (false)
        ** @param[out] encoded message header
        */
        static std::vector<uint8_t> encodeHeader(size_t payload_size, bool header_big_endian, bool payload_big_endian, uint16_t msgtype, uint16_t msgtypeversion, uint16_t source_id, int lidar_loc_version = 2);

        /*
        ** @brief Encodes the header of a message
        ** @param[in] payload_size length of payload in byte
        ** @param[in] payload_big_endian payload encoded in big endian (true) or little endian (false)
        ** @param[out] encoded message header
        */
        template<typename T> static std::vector<uint8_t> encodeHeader(const T& message_payload, size_t payload_size, bool header_big_endian, bool payload_big_endian, uint16_t source_id);

        /*
        ** @brief Encodes the payload of a message
        ** @param[in] message_payload payload of a message, can be OdometryPayload0104, OdometryPayload0105, EncoderMeasurementPayload0202, 
        **            CodeMeasurementPayload0303, CodeMeasurementPayload0701, LineMeasurementPayload0403 or LineMeasurementPayload0404
        ** @param[in] encode_big_endian encode in big or little endiang
        ** @param[out] encoded vector of bytes
        */
        template<typename T> static std::vector<uint8_t> encodePayload(const T& message_payload, bool encode_big_endian = false);

        /*
        ** @brief Encodes an input message
        ** @param[in] message_payload payload of a message, can be OdometryPayload0104, OdometryPayload0105, EncoderMeasurementPayload0202,
        **            CodeMeasurementPayload0303, CodeMeasurementPayload0701, LineMeasurementPayload0403 or LineMeasurementPayload0404
        ** @param[in] encode_big_endian encode in big or little endiang
        ** @param[out] encoded vector of bytes
        */
        template<typename T> static std::vector<uint8_t> encodeMessage(const T& message_payload, bool encode_header_big_endian = true, bool encode_payload_big_endian = false, uint16_t source_id = 1)
        {
            std::vector<uint8_t> payload = encodePayload(message_payload, encode_payload_big_endian);
            std::vector<uint8_t> header = encodeHeader(message_payload, payload.size(), encode_header_big_endian, encode_payload_big_endian, source_id);
            std::vector<uint8_t> message;
            message.reserve(header.size() + payload.size());
            message.insert(message.end(), header.begin(), header.end());
            message.insert(message.end(), payload.begin(), payload.end());
            return message;
        }

        /*
        ** @brief Unittest for message encoding
        */
        static bool unittest(void);

    protected:

        /*
        ** @brief Decodes sizeof(value) bytes to <value> with conversion using big endian or little endian.
        ** @param[in] buffer raw bytes received
        ** @param[in] big_endian assume big endian (true) or little endian (false) encoding
        ** @param[out] value decoded value
        */
        template<typename T> inline static void decode(const uint8_t* buffer, bool big_endian, T& value)
        {
            value = 0;
            if (sizeof(value) == 1)
                value = (T)(*buffer);
            else if (big_endian) // decode in big endian order
            {
                for (int n = 0; n < sizeof(value); n++)
                {
                    value = (value << 8);
                    value |= (buffer[n] & 0xFF);
                }
            }
            else // decode in little endian order
            {
                for (int n = sizeof(value) - 1; n >= 0; n--)
                {
                    value = (value << 8);
                    value |= (buffer[n] & 0xFF);
                }
            }
        }
        
        /*
        ** @brief Prints and returns the sync_timestamp calculated by Software-PLL to human readable string 
        */
        template<typename T> inline static std::string printSyncTime(const T& payload, bool print_sync_time)
        {
            if (print_sync_time)
            {
                std::stringstream s;
                s << ", sync_timestamp_sec:" << payload.sync_timestamp_sec;
                s << ", sync_timestamp_nsec:" << payload.sync_timestamp_nsec;
                s << ", sync_timestamp_valid:" << payload.sync_timestamp_valid;
                return s.str();
            }
            return "";
        }

    }; // class Message

}   // namespace sick_lidar_localization
#endif // __SICK_LIDAR_LOCALIZATION_MESSAGE_PARSER_H_INCLUDED
