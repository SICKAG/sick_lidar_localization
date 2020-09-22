#!/bin/bash

#
# Run simulation (offline-test) of sick_lidar_localization driver against local test server simulating a localization controller
#

#
# Environment
#

pushd ../../../..
printf "\033c"
source /opt/ros/melodic/setup.bash
source ./devel/setup.bash
# source ./install/setup.bash

#
# Cleanup
#

if [ -d ./src/sick_lidar_localization ]         ; then ./src/sick_lidar_localization/test/ros1_scripts/killall.bash         ; fi
if [ -d ./src/sick_lidar_localization_pretest ] ; then ./src/sick_lidar_localization_pretest/test/ros1_scripts/killall.bash ; fi
rm -rf ~/.ros/*
rosclean purge -y
if [ ! -d ~/.ros/log ] ; then mkdir -p ~/.ros/log ; fi

#
# Run test server, simulate localization controller for offline tests
#

roslaunch sick_lidar_localization sim_loc_test_server.launch 2>&1 | tee -a ~/.ros/log/sim_loc_test_server.log &
sleep 3 # make sure ros core and sim_loc_test_server are up and running 

#
# Log diagnostic messages
#

rostopic echo -p "/sick_lidar_localization/driver/diagnostic" | tee -a ~/.ros/log/sim_loc_driver_diagnostic_messages.log &

#
# Run ros driver, connect to localization controller and receive, convert and publish telegrams
#

roslaunch sick_lidar_localization sim_loc_driver.launch localization_controller_ip_address:=127.0.0.1 2>&1 | tee -a ~/.ros/log/sim_loc_driver.log &
sleep 1 # make sure sim_loc_test_server and sim_loc_driver are up and running 

#
# Run verification and check driver messages against testcases from localization controller
# rostopic echo "/sick_lidar_localization/driver/result_telegrams"
# rostopic echo "/sick_lidar_localization/test_server/result_testcases"
#

roslaunch sick_lidar_localization verify_sim_loc_driver.launch 2>&1 | tee -a ~/.ros/log/verify_sim_loc_driver.log &

#
# Run test server and driver for some time and verify results
#

sleep 180
if [ -d ./src/sick_lidar_localization ]         ; then ./src/sick_lidar_localization/test/ros1_scripts/killall.bash         ; fi
if [ -d ./src/sick_lidar_localization_pretest ] ; then ./src/sick_lidar_localization_pretest/test/ros1_scripts/killall.bash ; fi
sleep 1 ; killall roslaunch ; sleep 1

#
# Run standalone unittests for sim_loc_parser
#

roslaunch sick_lidar_localization unittest_sim_loc_parser.launch 2>&1 | tee -a ~/.ros/log/unittest_sim_loc_parser.log &
sleep 10
if [ -d ./src/sick_lidar_localization ]         ; then ./src/sick_lidar_localization/test/ros1_scripts/killall.bash         ; fi
if [ -d ./src/sick_lidar_localization_pretest ] ; then ./src/sick_lidar_localization_pretest/test/ros1_scripts/killall.bash ; fi
sleep 1 ; killall roslaunch
cat ~/.ros/log/sim_loc_driver_diagnostic_messages.log
grep "WARN" ~/.ros/log/*.log ; grep "ERR" ~/.ros/log/*.log
sleep 20

#
# Cleanup and exit
#

popd
rm -rf ./log
mkdir  ./log
cp -rf ~/.ros/log/*.log ./log
rm -f ./log/rostopic_*.log
rm -f ./log/rosservice_*.log
echo -e "\nsick_lidar_localization finished. Warnings and errors:"
grep "WARN" ./log/*.log
grep "ERR" ./log/*.log
echo -e "\nsim_loc_driver check and verification summary:"
grep -i "unittest_sim_loc_parser finished" ./log/*.log
grep -i "check messages thread summary"    ./log/*.log 
grep -i "verification thread summary"      ./log/*.log 
echo -e "\nsick_lidar_localization summary:"
grep -R "summary" ./log/*

