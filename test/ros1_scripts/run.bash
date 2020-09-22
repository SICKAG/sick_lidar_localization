#!/bin/bash

#
# Run test of sick_lidar_localization driver against localization controller SIM1000FXA
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

# upload demo_map:
# => file "/Z5....smap

#
# Run sim_loc_driver_check for a plausibility check of messages and telegrams published by the sick_lidar_localization ros driver
#

roslaunch sick_lidar_localization sim_loc_driver_check.launch sim_loc_driver_check_cfg:=message_check_demo.yaml    2>&1 | tee -a ~/.ros/log/sim_loc_driver_check_demo.log    &
roslaunch sick_lidar_localization sim_loc_driver_check.launch sim_loc_driver_check_cfg:=message_check_default.yaml 2>&1 | tee -a ~/.ros/log/sim_loc_driver_check_default.log &

#
# Log diagnostic messages
#

rostopic echo -p "/sick_lidar_localization/driver/diagnostic" | tee -a ~/.ros/log/sim_loc_driver_diagnostic_messages.log &

#
# Run ros driver, connect to localization controller and receive, convert and publish telegrams
#

roslaunch sick_lidar_localization sim_loc_driver.launch localization_controller_ip_address:=192.168.0.1 2>&1 | tee -a ~/.ros/log/sim_loc_driver.log &

#
# Run online test with localization controller and ros driver for some time, press 'q' to quit test
#
# sleep 3600
while true ; do  
  echo -e "sick_lidar_localization running. Press 'q' to exit..." ; read -t 1.0 -n1 -s key
  if [[ $key = "q" ]] || [[ $key = "Q" ]]; then break ; fi
done

#
# Cleanup and exit
#

popd
./killall.bash
killall roslaunch
sleep 1
rm -rf ./log
mkdir  ./log
cp -rf ~/.ros/log/*.log ./log
echo -e "\nsick_lidar_localization finished. Warnings and errors:"
grep "WARN" ./log/*.log
grep "ERR" ./log/*.log
echo -e "sim_loc_driver check summary:"
grep -i "check messages thread summary" ./log/*.log 

