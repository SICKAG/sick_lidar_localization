#!/bin/bash

# 
# Run sick_lidar_localization simu on native linux
# 
printf "\033c"

# 
# Run unittest for services and udp messages
# 

./run_linux_unittest_gen_sick_caller.bash
sleep 3
./run_linux_unittest_gen_sick_caller_2.bash
sleep 3
./run_linux_unittest_gen_sick_caller_3.bash
sleep 3
./run_linux_unittest_udp_receiver.bash
sleep 3
./run_linux_unittest_time_sync.bash
sleep 3

# 
# Start rest server
# 

echo -e "Starting sudo sick_rest_server.py ...\n" 
sudo pkill -f sick_rest_server.py
sudo python3 ../rest_server/python/sick_rest_server.py --time_sync=1 &
sleep 3

# 
# Start sick_lidar_localization
# 

pushd ../../build
./sick_lidar_localization ../launch/sick_lidar_localization.launch hostname:=localhost udp_ip_lls_input:=127.0.0.1 udp_ip_lls_output:=localhost verbose:=1 &
sleep 5
popd 

# 
# Start pcapng player sending recorded UDP messages
# 

python3 ../rest_server/python/lls_pcapng_player.py  --pcap_filename ../data/wireshark/20210816_lidarloc2_2.0.0.14R_nonmoving.pcapng
sleep 5

# 
# Close rest server and sick_lidar_localization
# 

# echo -e "\nRunning sick_lidar_localization simu...\n" 
# sleep 30
echo -e "\nShutdown sick_lidar_localization simu...\n" 
killall sick_lidar_localization
pkill -f lls_pcapng_player.py
sudo pkill -f sick_rest_server.py
pkill -f roslaunch
echo -e "\nsick_lidar_localization simu finished.\n" 


