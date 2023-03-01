"""
    lls_udp_sender generates synthetical datagrams to emulate a local localization server
    (specified by LocalizationController 2.0 UDP Data Format) and sends the UDP packages on port 5010.

"""

import argparse
import os
import socket
import time

source_id = 0x0001

# Encodes an integer <value> to a bytearray of size <length> in little or big endian.
def encode(value, length, big_endian = False):
    payload = bytearray(length)
    if big_endian:
        for n in range(0,length):
            payload[n] = ((value >> 8*(length - n - 1)) & 0xFF)
    else:
         for n in range(0,length):
            payload[n] = ((value >> 8*(n)) & 0xFF)
    return payload

# Runs a short unittest for encoding integers in little and big endian
def unittestEncode():
    assert(encode(0x01020304,         4, False) == b"\x04\x03\x02\x01")
    assert(encode(0x01020304,         4, True)  == b"\x01\x02\x03\x04")
    assert(encode(0x010203,           4, False) == b"\x03\x02\x01\x00")
    assert(encode(0x010203,           4, True)  == b"\x00\x01\x02\x03")
    assert(encode(0x0102030405060708, 8, False) == b"\x08\x07\x06\x05\x04\x03\x02\x01")
    assert(encode(0x0102030405060708, 8, True)  == b"\x01\x02\x03\x04\x05\x06\x07\x08")
    assert(encode(0x0102030405,       8, False) == b"\x05\x04\x03\x02\x01\x00\x00\x00")
    assert(encode(0x0102030405,       8, True)  == b"\x00\x00\x00\x01\x02\x03\x04\x05")

# Creates and returns the header (16 byte) of a message.
# The message header is always encoded in big endian,
# the payload is encoded in little endian (default) or big endian.
def createHeader(msg_type, msg_type_version, payload_length, big_endian):
    payload = bytearray(16)                             #  header has always 16 byte
    payload[0:4] = b"\x4d\x4f\x50\x53"                  # 4 byte Magic 0x4d 0x4f 0x50 0x53 ("MOPS")
    payload[4:6] = encode(2, 2, True)                   # 2 byte HeaderVersion 0x0002
    payload[6:8] = encode(payload_length, 2, True)      # 2 byte PayloadLength
    if big_endian:
        payload[8:10] = encode(0x0642, 2, True)         # 2 byte PayloadType 0x0642 for big endian
    else:
        payload[8:10] = encode(0x06c2, 2, True)         # 2 byte PayloadType 0x06c2 for little endian
    payload[10:12] = encode(msg_type, 2, True)          # 2 byte MsgType
    payload[12:14] = encode(msg_type_version, 2, True)  # 2 byte MsgTypeVersion
    payload[14:16] = encode(source_id, 2, True)            # 2 byte SourceID 0x0001
    return payload

# Creates and returns an odometry message (type 1, version 4, 16 byte message header + 24 byte payload)
def createOdometryMessage0104(telegram_count, timestamp, x_velocity, y_velocity, angular_velocity, big_endian):
    message = bytearray(40)                                   # odometry messages (type 1, version 4) have always 40 byte
    message[0:16] = createHeader(1, 4, 24, big_endian)        # type 1, version 4, 24 byte payload
    message[16:24] = encode(telegram_count, 8, big_endian)    # 8 byte TelegramCount uint64
    message[24:32] = encode(timestamp, 8, big_endian)         # 8 byte Timestamp uint64 [microseconds]
    message[32:34] = encode(x_velocity, 2, big_endian)        # 2 byte X-velocity int16 [mm/s]
    message[34:36] = encode(y_velocity, 2, big_endian)        # 2 byte Y-velocity int16 [mm/s]
    message[36:40] = encode(angular_velocity, 4, big_endian)  # 4 byte angular velocity int32 [mdeg/s]
    message_str = "OdometryPayload0104:{" + "telegram_count:" + str(telegram_count) + ", timestamp:" + str(timestamp) + ", source_id:" + str(source_id) + ", x_velocity:" + str(x_velocity) + ", y_velocity:" + str(y_velocity) + ", angular_velocity:" + str(angular_velocity) + "}"
    return message, message_str

# Creates and returns an odometry message (type 1, version 5, 16 byte message header + 40 byte payload)
def createOdometryMessage0105(telegram_count, timestamp, x_position, y_position, heading, big_endian):
    message = bytearray(56)                                   # odometry messages (type 1, version 5) has always 56 byte
    message[0:16] = createHeader(1, 5, 40, big_endian)        # type 1, version 5, 40 byte payload
    message[16:24] = encode(telegram_count, 8, big_endian)    # 8 byte TelegramCount uint64
    message[24:32] = encode(timestamp, 8, big_endian)         # 8 byte Timestamp uint64 [microseconds]
    message[32:40] = encode(x_position, 8, big_endian)        # 8 byte X-position int64 [mm]
    message[40:48] = encode(y_position, 8, big_endian)        # 8 byte Y-position int64 [mm]
    message[48:56] = encode(heading, 8, big_endian)           # 8 byte heading int64 [mdeg]
    message_str = "OdometryPayload0105:{" + "telegram_count:" + str(telegram_count) + ", timestamp:" + str(timestamp) + ", source_id:" + str(source_id) + ", x_position:" + str(x_position) + ", y_position:" + str(y_position) + ", heading:" + str(heading) + "}"
    return message, message_str

# Creates and returns an code measurement message (type 3, version 4, 16 byte message header + 24 byte payload)
def createCodeMeasurementMessage0304(telegram_count, timestamp, code, distance, big_endian):
    message = bytearray(40)                                   # code measurement messages (type 3, version 4) have always 40 byte
    message[0:16] = createHeader(3, 4, 24, big_endian)        # type 3, version 4, 24 byte payload
    message[16:24] = encode(telegram_count, 8, big_endian)    # 8 byte TelegramCount uint64
    message[24:32] = encode(timestamp, 8, big_endian)         # 8 byte Timestamp uint64 [microseconds]
    message[32:36] = encode(code, 4, big_endian)              # 4 byte Code int32
    message[36:40] = encode(distance, 4, big_endian)          # 4 byte Distance int32 [mm]
    message_str = "CodeMeasurementPayload0304:{" + "telegram_count:" + str(telegram_count) + ", timestamp:" + str(timestamp) + ", source_id:" + str(source_id) + ", code:" + str(code) + ", distance:" + str(distance) + "}"
    return message, message_str

# Creates and returns a line measurement message (type 4, version 3, 16 byte message header + variable payload)
def createLineMeasurementMessage0403(telegram_count, timestamp, lanes, big_endian):
    payload_length = 16 + 1 + 2 * len(lanes)                        # 17 byte + 2*NumOfLanes byte
    message = bytearray(16 + payload_length)                        # 16 byte header + payload_length
    message[0:16] = createHeader(4, 3, payload_length, big_endian)  # type 4, version 3
    message[16:24] = encode(telegram_count, 8, big_endian)          # 8 byte TelegramCount uint64
    message[24:32] = encode(timestamp, 8, big_endian)               # 8 byte Timestamp uint64 [microseconds]
    message[32] = len(lanes)                                        # 1 byte NumOfLanes uint8
    for n, lane in enumerate(lanes):                                # variable number of lanes
        message[33+2*n+0:33+2*n+2] = encode(lane, 2, big_endian)    # 2 byte lane int16    
    message_str = "LineMeasurementPayload0403:{" + "telegram_count:" + str(telegram_count) + ", timestamp:" + str(timestamp) + ", source_id:" + str(source_id) + ", num_lanes:" + str(len(lanes)) + ", lanes:["
    for n, lane in enumerate(lanes):
        if n > 0:
            message_str = message_str + ","
        message_str = message_str + str(lane)
    message_str = message_str + "]}"
    return message, message_str

# Creates and returns a line measurement message (type 4, version 4, 16 byte message header + 24 byte payload)
def createLineMeasurementMessage0404(telegram_count, timestamp, lcp1, lcp2, lcp3, cnt_lpc, big_endian):
    message = bytearray(40)                                   # line measurement messages (type 4, version 4) have always 40 byte
    message[0:16] = createHeader(4, 4, 24, big_endian)        # type 4, version 4, 40 byte payload
    message[16:24] = encode(telegram_count, 8, big_endian)    # 8 byte TelegramCount uint64
    message[24:32] = encode(timestamp, 8, big_endian)         # 8 byte Timestamp uint64 [microseconds]
    message[32:34] = encode(lcp1, 2, big_endian)              # 2 byte lcp1 int16 [mm]
    message[34:36] = encode(lcp2, 2, big_endian)              # 2 byte lcp2 int16 [mm]
    message[36:38] = encode(lcp3, 2, big_endian)              # 2 byte lcp3 int16 [mm]
    message[38] = cnt_lpc                                     # 1 byte cnt_lpc uint8
    message[39] = 0                                           # 1 byte memory alignment
    message_str = "LineMeasurementPayload0404:{" + "telegram_count:" + str(telegram_count) + ", timestamp:" + str(timestamp) + ", source_id:" + str(source_id) + ", lcp1:" + str(lcp1) + ", lcp2:" + str(lcp2) + ", lcp3:" + str(lcp3) + ", cnt_lpc:" + str(cnt_lpc) + "}"
    return message, message_str

# Creates and returns a LocalizationController result message (type 5, version 2, 16 byte message header + 40 byte payload)
def createResultMessage0502(telegram_count, timestamp, x, y, heading, big_endian):
    message = bytearray(56)                                # LocalizationController result messages (type 5, version 2) have always 56 byte
    message[0:16] = createHeader(5, 2, 40, big_endian)     # type 5, version 2, 40 byte payload
    message[16:24] = encode(telegram_count, 8, big_endian) # 8 byte TelegramCount uint64
    message[24:32] = encode(timestamp, 8, big_endian)      # 8 byte Timestamp uint64 [microseconds]
    message[32:40] = encode(x, 8, big_endian)              # 8 byte X int64 [mm]
    message[40:48] = encode(y, 8, big_endian)              # 8 byte Y int64 [mm]
    message[48:52] = encode(heading, 4, big_endian)        # 4 byte Heading int32 [mdeg]
    message[52] = 10                                       # 1 byte LocalizationStatus [0...100, 10: OK]
    message[53] = 90                                       # 1 byte MapMatchingStatus [0...100, 90: Good]
    message[54] = 0                                        # 2 byte Reserved for memory alignment
    message[55] = 0                                        # 2 byte Reserved for memory alignment
    message_str = "LocalizationControllerResultPayload0502:{" + "telegram_count:" + str(telegram_count) + ", timestamp:" + str(timestamp) + ", source_id:" + str(source_id) + ", x:" + str(x) + ", y:" + str(y) + ", heading:" + str(heading) + ", loc_status:10, map_match_status:90}"
    return message, message_str

if __name__ == "__main__":

    # Configuration
    udp_port = 5010 # UDP port to send datagrams
    udp_send_rate = 30.0 # send rate in datagrams per second
    udp_output_logfile = "" # Optional logfile for human readable output messages, default: "" (no logfile)
    max_message_count = -1  # Max number of udp messages to send, or -1 for endless loop (default)
    
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("--udp_port", help="udp port", default=udp_port, type=int)
    arg_parser.add_argument("--udp_send_rate", help="udp send rate in hz", default=udp_send_rate, type=float)
    arg_parser.add_argument("--udp_output_logfile", help="optional logfile for human readable output messages", default=udp_output_logfile, type=str)
    arg_parser.add_argument("--max_message_count", help="Max number of udp messages to send, or -1 for endless loop (default)", default=max_message_count, type=int)
    cli_args = arg_parser.parse_args()
    udp_port = cli_args.udp_port
    udp_send_rate = cli_args.udp_send_rate
    udp_output_logfile = cli_args.udp_output_logfile
    max_message_count = cli_args.max_message_count
    print("lls_udp_sender --udp_port={} --udp_send_rate={} --udp_output_logfile={} --max_message_count={} initializing...".format(udp_port, udp_send_rate, udp_output_logfile, max_message_count))
    
    # Setup
    if udp_output_logfile != "" and os.path.exists(udp_output_logfile):
        os.remove(udp_output_logfile)
    unittestEncode()

    # Init udp sender
    udp_sender_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) # UDP socket
    udp_sender_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) # Enable broadcasting mode
    print("udp_sender: sending on udp port {}".format(udp_port))
    
    # Send udp messages
    telegram_count = 0
    telegram_idx = -1
    msg_type_idx = 0
    while max_message_count < 0 or telegram_count < max_message_count:
        telegram_count = telegram_count + 1
        telegram_idx = telegram_idx + 1
        message = None
        message_str = ""
        if msg_type_idx == 0:
            if telegram_idx == 0:
                message, message_str = createOdometryMessage0104(1000000000 + telegram_count, 12345678901234567 + telegram_count, +20000, +10000, +5000, False)
            elif telegram_idx == 1:
                 message, message_str = createOdometryMessage0104(1000000000 + telegram_count, 12345678901234567 + telegram_count, +20000, +10000, +5000, True)
            elif telegram_idx == 2:
                 message, message_str = createOdometryMessage0104(1000000000 + telegram_count, 12345678901234567 + telegram_count, -20000, -10000, -5000, False)
            elif telegram_idx == 3:
                 message, message_str = createOdometryMessage0104(1000000000 + telegram_count, 12345678901234567 + telegram_count, -20000, -10000, -5000, True)
        elif msg_type_idx == 1:
            if telegram_idx == 0:
                 message, message_str = createOdometryMessage0105(1000000000 + telegram_count, 12345678901234567 + telegram_count, +30000, +25000, +123123, False)
            elif telegram_idx == 1:
                 message, message_str = createOdometryMessage0105(1000000000 + telegram_count, 12345678901234567 + telegram_count, +30000, +25000, +123123, True)
            elif telegram_idx == 2:
                 message, message_str = createOdometryMessage0105(1000000000 + telegram_count, 12345678901234567 + telegram_count, -30000, -25000, -123123, False)
            elif telegram_idx == 3:
                 message, message_str = createOdometryMessage0105(1000000000 + telegram_count, 12345678901234567 + telegram_count, -30000, -25000, -123123, True)
        elif msg_type_idx == 2:
            if telegram_idx == 0:
                 message, message_str = createCodeMeasurementMessage0304(1000000000 + telegram_count, 12345678901234567 + telegram_count, +12345, +67890, False)
            if telegram_idx == 1:
                 message, message_str = createCodeMeasurementMessage0304(1000000000 + telegram_count, 12345678901234567 + telegram_count, +12345, +67890, True)
            if telegram_idx == 2:
                 message, message_str = createCodeMeasurementMessage0304(1000000000 + telegram_count, 12345678901234567 + telegram_count, -12345, -67890, False)
            if telegram_idx == 3:
                 message, message_str = createCodeMeasurementMessage0304(1000000000 + telegram_count, 12345678901234567 + telegram_count, -12345, -67890, True)
        elif msg_type_idx == 3:
            if telegram_idx == 0:
                 message, message_str = createLineMeasurementMessage0403(1000000000 + telegram_count, 12345678901234567 + telegram_count, [+300, +345], False)
            elif telegram_idx == 1:
                 message, message_str = createLineMeasurementMessage0403(1000000000 + telegram_count, 12345678901234567 + telegram_count, [+300, +345], True)
            elif telegram_idx == 2:
                 message, message_str = createLineMeasurementMessage0403(1000000000 + telegram_count, 12345678901234567 + telegram_count, [-300, -345, -6789], False)
            elif telegram_idx == 3:
                 message, message_str = createLineMeasurementMessage0403(1000000000 + telegram_count, 12345678901234567 + telegram_count, [-300, -345, -6789], True)
        elif msg_type_idx == 4:
            if telegram_idx == 0:
                 message, message_str = createLineMeasurementMessage0404(1000000000 + telegram_count, 12345678901234567 + telegram_count, +300, +345, -6789, 7, False)
            elif telegram_idx == 1:
                 message, message_str = createLineMeasurementMessage0404(1000000000 + telegram_count, 12345678901234567 + telegram_count, +300, +345, -6789, 7, True)
            elif telegram_idx == 2:
                 message, message_str = createLineMeasurementMessage0404(1000000000 + telegram_count, 12345678901234567 + telegram_count, -300, -345, +6789, 7, False)
            elif telegram_idx == 3:
                 message, message_str = createLineMeasurementMessage0404(1000000000 + telegram_count, 12345678901234567 + telegram_count, -300, -345, +6789, 7, True)
        elif msg_type_idx == 5:
            if telegram_idx == 0:
                 message, message_str = createResultMessage0502(1000000000 + telegram_count, 12345678901234567 + telegram_count, +1020304, -1020304, +123456, False)
            elif telegram_idx == 1:
                 message, message_str = createResultMessage0502(1000000000 + telegram_count, 12345678901234567 + telegram_count, +1020304, -1020304, +123456, True)
            elif telegram_idx == 2:
                 message, message_str = createResultMessage0502(1000000000 + telegram_count, 12345678901234567 + telegram_count, -1020304, +1020304, -123456, False)
            elif telegram_idx == 3:
                 message, message_str = createResultMessage0502(1000000000 + telegram_count, 12345678901234567 + telegram_count, -1020304, +1020304, -123456, True)
        if message is None:
            telegram_idx = -1
            msg_type_idx = msg_type_idx + 1
            if msg_type_idx > 7:
                msg_type_idx = 0
            continue;
        print("udp_sender: sending {} byte message = {}".format(len(message), message.hex()))
        udp_sender_socket.sendto(message, ('<broadcast>', udp_port))
        # udp_sender_socket.sendto(message, ('127.0.0.1', udp_port))
        if udp_output_logfile != "":
            with open(udp_output_logfile, "a") as logfile:
                logfile.write(message_str + "\n")
        time.sleep(1.0 / udp_send_rate)        
