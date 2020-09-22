# udp receiver for odometry telegrams from sim_loc_driver
# python3 odom_udp_receiver.py

import socket

# configuration
exit_after = 100 # exit after 100 odometry messages

# upd receiver to verify udp telegrams from sim_loc_driver
udp_receive_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
udp_receive_socket.setblocking(True)
# udp_receive_socket.settimeout(1.0)
udp_receive_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
udp_receive_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
udp_receive_socket.bind(("0.0.0.0", 3000))
# udp_receive_socket.bind(("127.0.0.1", 3000))

telegramcounter = 0

while telegramcounter < exit_after:

      # receive udp telegram from sim_loc_driver with timeout
      #if select.select([udp_receive_socket], [], [], 0.1)[0]:
      udp_received = udp_receive_socket.recvfrom(1024)[0]
      udp_hex = udp_received.hex()
      telegramcounter = telegramcounter + 1
      print("odom_msg_sender: received {}. udp telegram {}".format(telegramcounter, udp_hex))

