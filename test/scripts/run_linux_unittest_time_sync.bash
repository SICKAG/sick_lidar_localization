#!/bin/bash
# 
# Unittest for sick_lidar_localization time sync on native Linux
# 
killall sick_lidar_localization

# 
# Start rest server for LocTimestampRequests
# 

echo -e "Starting sudo sick_rest_server.py ...\n" 
sudo pkill -f sick_rest_server.py
sudo python3 ../rest_server/python/sick_rest_server.py --time_sync=1 &
sleep 3

# 
# Run sick_lidar_localization
# 

pushd ../../build
./sick_lidar_localization ../launch/sick_lidar_localization.launch hostname:=localhost udp_ip_sim_input:=127.0.0.1 udp_ip_sim_output:=localhost verbose:=1 | tee sick_lidar_localization.log &
sleep 3
popd 

# 
# Wait 7 seconds until Software-PLL is initialized and start udp sender
# 

sleep 7
python3 ../rest_server/python/sim_udp_sender.py --udp_port=5010 --udp_send_rate=30.0 --udp_output_logfile=../../build/udp_sender.log --max_message_count=1000
sleep 15
grep "WARN" ../../build/sick_lidar_localization.log > ../../build/sick_lidar_localization_error.log
grep "ERR" ../../build/sick_lidar_localization.log >> ../../build/sick_lidar_localization_error.log

# 
# Shutdown rest server and sick_lidar_localization
# 

echo -e "\nShutdown sick_lidar_localization ...\n" 
killall sick_lidar_localization
sudo pkill -f sick_rest_server.py

# 
# Check the timestamps calculated from sensor tics by Software-PLL:
# - Software-PLL initialized after 7 LocTimestampRequests
# - Calculated timestamps in output messages close to current time
# 

NUM_ERRORS=$(cat ../../build/sick_lidar_localization_error.log | wc -l)
if [[ "$NUM_ERRORS" -eq 0 ]]; then
    echo -e "\nunittest for time sync passed, no errors detected\n"
else
    echo -e "\n## ERROR: unittest for time sync failed, $NUM_ERRORS errors detected:\n"
    cat ../../build/sick_lidar_localization_error.log
fi
