#!/bin/bash

# 
# Run sick_lidar_localization unittest for udp messages on native linux
# 
killall sick_lidar_localization

# 
# Start rest server
# 

echo -e "Starting sudo sick_rest_server.py ...\n" 
sudo pkill -f sick_rest_server.py
sudo python3 ../rest_server/python/sick_rest_server.py &
sleep 3

# 
# Start sick_lidar_localization
# 

pushd ../../build
./sick_lidar_localization ../launch/sick_lidar_localization.launch hostname:=localhost udp_ip_lls_input:=127.0.0.1 udp_ip_lls_output:=localhost verbose:=1 udp_lls_output_logfile:=udp_lls_output.log &
sleep 3
popd 

# 
# Start udp sender
# 

python3 ../rest_server/python/lls_udp_sender.py --udp_port=5010 --udp_send_rate=30.0 --udp_output_logfile=../../build/udp_sender.log --max_message_count=300

# 
# Shutdown rest server and sick_lidar_localization
# 

echo -e "\nShutdown sick_lidar_localization ...\n" 
killall sick_lidar_localization
sudo pkill -f sick_rest_server.py

# 
# Compare received udp_lls_output.log with udp_sender.log, should be identical
# 

echo -e "\nrun_linux_unittest_udp_receiver finished.\n" 
cat ../../build/udp_lls_output.log
diff ../../build/udp_lls_output.log ../../build/udp_sender.log
if [ $? != 0 ] ; then
  echo -e "\n## ERROR unittest for udp messages: files ../../build/udp_lls_output.log and ../../build/udp_sender.log different, should be identical\n" 
  echo -e "## ERROR: unittest for udp messagesr failed. Press any key to continue..."
  read -n1 -s key 
else
  echo -e "\nuunittest for udp messages passed, no errors.\n" 
fi
