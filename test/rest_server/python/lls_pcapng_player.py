"""
    pcapng player to parse pcapng files recorded from a localization server
    and replay the UDP packages to emulate a local localization controller.

    The UDP Sender sends packets via UDP over localhost, Port: 5010

    Usage:

    pip install scapy
    pip install pypcapfile
    pip install python-pcapng

"""

import argparse
import socket
import time

from pcapng import FileScanner
from pcapng.blocks import EnhancedPacket #, InterfaceDescription, SectionHeader

import scapy.all
import scapy.packet
from scapy.layers.l2 import Ether

if __name__ == "__main__":

    pcap_filename = "../../data/wireshark/20210816_lidarloc2_2.0.0.14R_nonmoving.pcapng"
    udp_port = 5010 # UDP port to send datagrams
    udp_send_rate = 0.0 # send rate in datagrams per second, or 0 to send corresponding to pcap-timestamps

    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("--pcap_filename", help="pcapng filepath", default=pcap_filename, type=str)
    arg_parser.add_argument("--udp_port", help="udp port", default=udp_port, type=int)
    cli_args = arg_parser.parse_args()
    pcap_filename = cli_args.pcap_filename
    udp_port = cli_args.udp_port
    print("pcapng_player {} initializing, udp port {} ...".format(pcap_filename, udp_port))
    
    # Init udp sender
    udp_sender_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) # UDP socket
    udp_sender_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) # Enable broadcasting mode
    print("pcapng_player: sending on udp port {}".format(udp_port))
   
    # Read and parse pcap file, extract udp raw data
    send_timestamp = 0
    with open(pcap_filename, 'rb') as pcap_file:
        pcap_scanner = FileScanner(pcap_file)
        for block_cnt, block in enumerate(pcap_scanner):
            # print("im_pcapng_player block {}: {}".format(block_cnt, block))
            if isinstance(block, EnhancedPacket):
                # Decode a single pcap block
                # print("pcapng_player block {}: packet_len={}, captured_len={}".format(block_cnt, block.packet_len, block.captured_len))
                if block.captured_len != block.packet_len:
                    print("## pcapng_player block {}: {} byte block truncated to {} bytes".format(block_cnt, block.packet_len, block.captured_len))
                block_data = Ether(block.packet_data)
                block_decoded = block_data
                for n in range(0,10):
                  if isinstance(block_decoded.payload, scapy.packet.Raw):
                      break                  
                  elif isinstance(block_decoded.payload, scapy.packet.Packet):
                      block_decoded = block_decoded.payload
                  else:
                      break
                    
                # Send payload
                if isinstance(block_decoded.payload, scapy.packet.Raw) and len(block_decoded.payload) > 0:                
                    payload = bytes(block_decoded.payload)
                    #print("pcapng_player block {}: {} byte udp payload".format(block_cnt, len(payload)))
                    if payload[0:4] == b"\x4d\x4f\x50\x53" or payload[0:4] == b"\x53\x50\x4f\x4d": # LocalizationController 2.0 UDP payload always start with \x4d\x4f\x50\x53 (magic string "MOPS")
                        print("pcapng_player block {}: sending {} byte LocalizationController 2.0 UDP payload: {}".format(block_cnt, len(payload), payload.hex()))
                        udp_sender_socket.sendto(payload, ('<broadcast>', udp_port))
                        # udp_sender_socket.sendto(payload, ('127.0.0.1', udp_port))
                
                    # Send next block with delay from pcap timestamps or with configured rate
                    if send_timestamp > 0 and block.timestamp > send_timestamp:
                        if udp_send_rate <= 0: #  delay from pcap timestamps
                            delay = block.timestamp - send_timestamp
                        else: # delay from configured rate
                            delay = 1.0 / udp_send_rate
                        time.sleep(delay)
                    send_timestamp = block.timestamp
                    # if block_cnt > 3:
                    #     break

    print("pcapng_player finished.")
