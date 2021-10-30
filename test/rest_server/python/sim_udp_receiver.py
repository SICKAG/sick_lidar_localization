import argparse
import os
import socket
import time

# Encodes an integer <value> to a bytearray of size <length> in little or big endian.
def decode(payload, big_endian = True, signed_value = False):
    value = 0
    if big_endian:
        sign = (payload[0] & 0x80)
        for n in range(0,len(payload)):
            value = value << 8
            value = value | (payload[n] & 0xFF)
    else:
        sign = (payload[len(payload) - 1] & 0x80)
        for n in range(0,len(payload)):
            value = value << 8
            value = value | (payload[len(payload) - n - 1] & 0xFF)
    if signed_value and sign != 0:
        value = value - pow(2, 8 * len(payload))
    return value

if __name__ == "__main__":

    # Configuration
    udp_port = 5010 # UDP port to receive datagrams
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("--udp_port", help="udp port", default=udp_port, type=int)
    cli_args = arg_parser.parse_args()
    udp_port = cli_args.udp_port

    # Init upd receiver
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) # UDP socket
    udp_socket.bind(("", udp_port))
    
    # Receive udp datagrams
    print("sim_udp_receiver listening on port {} ...".format(udp_port))
    while True:
        data, addr = udp_socket.recvfrom(1024) # buffer size is 1024 bytes
        if len(data) >= 16 and data[0:4] == b"\x4d\x4f\x50\x53": # 4 byte Magic 0x4d 0x4f 0x50 0x53 ("MOPS")

            payloadtypeBE = decode(data[8:10], True)
            payloadtypeLE = decode(data[8:10], False)
            header_is_big_endian = False
            payload_is_big_endian = False
            if payloadtypeBE == 0x06c2:
                header_is_big_endian = True
                payload_is_big_endian = False
            elif payloadtypeBE == 0x0642:
                header_is_big_endian = True
                payload_is_big_endian = True
            elif payloadtypeLE == 0x06c2:
                header_is_big_endian = False
                payload_is_big_endian = False
            elif payloadtypeLE == 0x0642:
                header_is_big_endian = False
                payload_is_big_endian = True
            else:
                print("## ERROR sim_udp_receiver: unknown message header encoding")
                continue

            payloadlen = decode(data[6:8], header_is_big_endian)
            msgtype = decode(data[10:11], header_is_big_endian) # int(data[11])
            msgtypeversion = decode(data[12:13], header_is_big_endian) # int(data[13])
            print("{} byte udp message received, msgtype {}, msgtypeversion {}, {} byte payload".format(len(data), msgtype, msgtypeversion, payloadlen))
            
            if msgtype == 1 and msgtypeversion == 1 and payloadlen == 32:
                telegram_count = decode(data[16:24], payload_is_big_endian)
                timestamp = decode(data[24:32], payload_is_big_endian)
                x_velocity = decode(data[32:36], payload_is_big_endian, True)
                y_velocity = decode(data[36:40], payload_is_big_endian, True)
                angular_velocity = decode(data[40:48], payload_is_big_endian, True)
                print("OdometryMessage0101:{" + "telegram_count:" + str(telegram_count) + ", timestamp:" + str(timestamp) + ", x_velocity:" + str(x_velocity) + ", y_velocity:" + str(y_velocity) + ", angular_velocity:" + str(angular_velocity) + "}")
                
            elif msgtype == 1 and msgtypeversion == 4 and payloadlen == 24:
                telegram_count = decode(data[16:24], payload_is_big_endian)
                timestamp = decode(data[24:32], payload_is_big_endian)
                x_velocity = decode(data[32:34], payload_is_big_endian, True)
                y_velocity = decode(data[34:36], payload_is_big_endian, True)
                angular_velocity = decode(data[36:40], payload_is_big_endian, True)
                print("OdometryMessage0104:{" + "telegram_count:" + str(telegram_count) + ", timestamp:" + str(timestamp) + ", x_velocity:" + str(x_velocity) + ", y_velocity:" + str(y_velocity) + ", angular_velocity:" + str(angular_velocity) + "}")
                
            elif msgtype == 1 and msgtypeversion == 5 and payloadlen == 40:
                telegram_count = decode(data[16:24], payload_is_big_endian)
                timestamp = decode(data[24:32], payload_is_big_endian)
                x_position = decode(data[32:40], payload_is_big_endian, True)
                y_position = decode(data[40:48], payload_is_big_endian, True)
                heading = decode(data[48:56], payload_is_big_endian, True)
                print("OdometryMessage0105:{" + "telegram_count:" + str(telegram_count) + ", timestamp:" + str(timestamp) + ", x_position:" + str(x_position) + ", y_position:" + str(y_position) + ", heading:" + str(heading) + "}")
           
            elif msgtype == 2 and msgtypeversion == 2 and payloadlen == 24:
                telegram_count = decode(data[16:24], payload_is_big_endian)
                timestamp = decode(data[24:32], payload_is_big_endian)
                encoder_value = decode(data[32:40], payload_is_big_endian, True)
                print("EncoderMeasurementMessage0202:{" + "telegram_count:" + str(telegram_count) + ", timestamp:" + str(timestamp) + ", encoder_value:" + str(encoder_value) + "}")
            
            elif msgtype == 3 and msgtypeversion == 3 and payloadlen == 24:
                telegram_count = decode(data[16:24], payload_is_big_endian)
                timestamp = decode(data[24:32], payload_is_big_endian)
                code = decode(data[32:40], payload_is_big_endian, True)
                print("CodeMeasurementMessage0303:{" + "telegram_count:" + str(telegram_count) + ", timestamp:" + str(timestamp) + ", code:" + str(code) + "}")
            
            elif msgtype == 4 and msgtypeversion == 3 and payloadlen >= 17:
                telegram_count = decode(data[16:24], payload_is_big_endian)
                timestamp = decode(data[24:32], payload_is_big_endian)
                num_lanes = int(data[32])
                lanes = ""
                for n in range(0,num_lanes):
                    lane = decode(data[(33 + 2 * n): (35 + 2 * n)], payload_is_big_endian)
                    if n > 0:
                        lanes = lanes + ","
                    lanes = lanes + str(lane)
                print("LineMeasurementMessage0403:{" + "telegram_count:" + str(telegram_count) + ", timestamp:" + str(timestamp) + ", num_lanes:" + str(num_lanes) + ", lanes:[" + lanes + "]}")
                
            elif msgtype == 4 and msgtypeversion == 4 and payloadlen == 24:
                telegram_count = decode(data[16:24], payload_is_big_endian)
                timestamp = decode(data[24:32], payload_is_big_endian)
                lcp1 = decode(data[32:34], payload_is_big_endian)
                lcp2 = decode(data[34:36], payload_is_big_endian)
                lcp3 = decode(data[36:38], payload_is_big_endian)
                cnt_lpc = int(data[38]) # decode(data[38:39], payload_is_big_endian)
                print("LineMeasurementMessage0404:{" + "telegram_count:" + str(telegram_count) + ", timestamp:" + str(timestamp) + ", lcp1:" + str(lcp1) + ", lcp2:" + str(lcp2) + ", lcp3:" + str(lcp3) + ", cnt_lpc:" + str(cnt_lpc) + "}")
                          
            else:
                print("## ERROR sim_udp_receiver: unknown message type or wrong payload length")
            

        