#!/bin/bash

#
# Run cola examples (offline-test for cola telegrams and ros services provided by the sick_lidar_localization driver)
# against local test server simulating a localization controller
#

#
# Environment
#

pushd ../../../..
printf "\033c"
source /opt/ros/eloquent/setup.bash
source ./install/setup.bash

#
# Cleanup
#

if [ -d ./src/sick_lidar_localization ]         ; then ./src/sick_lidar_localization/test/ros2_scripts/killall.bash         ; fi
if [ -d ./src/sick_lidar_localization_pretest ] ; then ./src/sick_lidar_localization_pretest/test/ros2_scripts/killall.bash ; fi
rm -rf ~/.ros/*
if [ ! -d ~/.ros/log/cola_examples ] ; then mkdir -p ~/.ros/log/cola_examples ; fi 

#
# Global configuration for offline simulation
#

export localization_controller_ip_address=127.0.0.1
export sim_loc_test_server_demo_circles=0
export sim_loc_test_server_error_simulation=0 

#
# Run standalone unittests for sim_loc_parser
#

ros2 launch sick_lidar_localization unittest_sim_loc_parser.launch.py &
sleep 10
if [ -d ./src/sick_lidar_localization ]         ; then ./src/sick_lidar_localization/test/ros2_scripts/killall.bash         ; fi
if [ -d ./src/sick_lidar_localization_pretest ] ; then ./src/sick_lidar_localization_pretest/test/ros2_scripts/killall.bash ; fi
sleep 1

#
# Run test server, simulate localization controller for offline tests
#

ros2 launch sick_lidar_localization sim_loc_test_server.launch.py 2>&1 >> ~/.ros/log/cola_examples/sim_loc_test_server.log &
sleep 3 # make sure ros core and sim_loc_test_server are up and running 

#
# Run ros driver, connect to localization controller and receive, convert and publish telegrams
#

ros2 launch sick_lidar_localization sim_loc_driver.launch.py 2>&1 >> ~/.ros/log/cola_examples/sim_loc_driver.log &
sleep 1 # make sure sim_loc_test_server and sim_loc_driver are up and running 

#
# Send cola examples using ros services provided by the sick_lidar_localization driver
# Example:
# ros2 service list -t
# ros2 service call /SickLocIsSystemReady sick_lidar_localization/srv/SickLocIsSystemReadySrv "{}"
# ros2 service call /SickLocRequestTimestamp sick_lidar_localization/srv/SickLocRequestTimestampSrv "{}"
# ros2 service call /SickLocColaTelegram sick_lidar_localization/srv/SickLocColaTelegramSrv "{cola_ascii_request: 'sMN IsSystemReady', wait_response_timeout: 1}"

popd
./send_cola_advanced.bash 2>&1 | tee -a ~/.ros/log/cola_examples/send_cola_advanced.log
./send_cola_examples.bash 2>&1 | tee -a ~/.ros/log/cola_examples/send_cola_examples.log

#
# Cleanup and exit
#

./killall.bash
sleep 1 ; killall roslaunch ; sleep 1
cat ~/.ros/log/cola_examples/send_cola_examples.log
cat ~/.ros/log/cola_examples/send_cola_advanced.log
# sleep 20
mkdir -p ./log/cola_examples
cp -rf ~/.ros/log/cola_examples/*.log ./log/cola_examples
echo -e "\nsick_lidar_localization finished. Warnings and errors:"
grep "WARN" ./log/cola_examples/*.log
grep "ERR" ./log/cola_examples/*.log
echo -e "\nsim_loc_driver check summary:"
grep -i "check messages thread summary" ./log/cola_examples/*.log 
grep -i "verification summary" ./log/cola_examples/*.log 
