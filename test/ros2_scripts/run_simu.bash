#!/bin/bash

#
# Run simulation (offline-test) of sick_lidar_localization driver against local test server simulating a localization controller
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
if [ ! -d ~/.ros/log ] ; then mkdir -p ~/.ros/log ; fi

#
# Global configuration for offline simulation
#

export localization_controller_ip_address=127.0.0.1
export sim_loc_test_server_demo_circles=0
export sim_loc_test_server_error_simulation=0 

#
# Run test server, simulate localization controller for offline tests
#

ros2 launch sick_lidar_localization sim_loc_test_server.launch.py 2>&1 | tee -a ~/.ros/log/sim_loc_test_server.log &
sleep 3 # make sure ros core and sim_loc_test_server are up and running 

#
# Log diagnostic messages
#

ros2 topic echo "/sick_lidar_localization/driver/diagnostic" | tee -a ~/.ros/log/sim_loc_driver_diagnostic_messages.log &

#
# Run ros driver, connect to localization controller and receive, convert and publish telegrams
#

ros2 launch sick_lidar_localization sim_loc_driver.launch.py 2>&1 | tee -a ~/.ros/log/sim_loc_driver.log &
sleep 1 # make sure sim_loc_test_server and sim_loc_driver are up and running 

#
# Run verification and check driver messages against testcases from localization controller
# ros2 topic echo "/sick_lidar_localization/driver/result_telegrams"
# ros2 topic echo "/sick_lidar_localization/test_server/result_testcases"
#

ros2 launch sick_lidar_localization verify_sim_loc_driver.launch.py 2>&1 | tee -a ~/.ros/log/verify_sim_loc_driver.log &

#
# Run test server and driver for some time and verify results
#

sleep 180
if [ -d ./src/sick_lidar_localization ]         ; then ./src/sick_lidar_localization/test/ros2_scripts/killall.bash         ; fi
if [ -d ./src/sick_lidar_localization_pretest ] ; then ./src/sick_lidar_localization_pretest/test/ros2_scripts/killall.bash ; fi
sleep 3

#
# Run standalone unittests for sim_loc_parser
#

ros2 launch sick_lidar_localization unittest_sim_loc_parser.launch.py 2>&1 | tee -a ~/.ros/log/unittest_sim_loc_parser.log &
sleep 10
if [ -d ./src/sick_lidar_localization ]         ; then ./src/sick_lidar_localization/test/ros2_scripts/killall.bash         ; fi
if [ -d ./src/sick_lidar_localization_pretest ] ; then ./src/sick_lidar_localization_pretest/test/ros2_scripts/killall.bash ; fi
sleep 3
cat ~/.ros/log/sim_loc_driver_diagnostic_messages.log
grep "WARN" ~/.ros/log/*.log ; grep "ERR" ~/.ros/log/*.log
sleep 20

#
# Cleanup and exit
#

popd
rm -rf ./log
mkdir  ./log
cp -rf ~/.ros/log/unittest_sim_loc_parser.log ~/.ros/log/sim_loc_driver.log ~/.ros/log/verify_sim_loc_driver.log ./log
echo -e "\nsick_lidar_localization finished. Warnings and errors:"
grep "WARN" ~/.ros/log/*.log
grep "ERR" ~/.ros/log/*.log
echo -e "\n\nsick_lidar_localization check and verification summary:"
grep -i "unittest_sim_loc_parser finished" ./log/unittest_sim_loc_parser.log
grep -i "check messages thread summary"    ./log/sim_loc_driver.log 
grep -i "verification thread summary"      ./log/verify_sim_loc_driver.log

