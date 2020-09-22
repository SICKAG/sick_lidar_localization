#!/bin/bash

#
# Run odometry simulation (offline-test) of sick_lidar_localization driver
# Odometry simulation runs the sim_loc_driver, a odometry message sender
# (odem_msg_sender.py) and checks the odom udp packages sent by sim_loc_driver.
#

#
# Environment
#

pushd ../../../..
printf "\033c"
source /opt/ros/melodic/setup.bash
source ./devel/setup.bash
export ROS_VERSION=1

#
# Cleanup
#

if [ -d ./src/sick_lidar_localization ]         ; then ./src/sick_lidar_localization/test/ros1_scripts/killall.bash         ; fi
if [ -d ./src/sick_lidar_localization_pretest ] ; then ./src/sick_lidar_localization_pretest/test/ros1_scripts/killall.bash ; fi
rm -rf ~/.ros/*
if [ ! -d ~/.ros/log ] ; then mkdir -p ~/.ros/log ; fi


#
# Start test server and sim_loc_driver
#

roslaunch sick_lidar_localization sim_loc_test_server.launch > ~/.ros/log/sim_loc_test_server.log &
sleep 3 # make sure ros core and sim_loc_test_server are up and running 
roslaunch sick_lidar_localization sim_loc_driver.launch localization_controller_ip_address:=127.0.0.1 > ~/.ros/log/sim_loc_driver.log &
sleep 3 # make sure sim_loc_driver is up and running before sending and verifying odometry telegrams

#
# Visualize odom messages by rviz:
#

rosrun rviz rviz -d ./src/sick_lidar_localization_pretest/test/config/rviz_odom_msg_sender.rviz &

#
# Start odometry message sender, send 100 odometry messages and verify udp telegrams from sim_loc_driver
#

roslaunch sick_lidar_localization odom_msg_sender.launch

popd
./killall.bash
echo -e "\nrun_odom_simu for sick_lidar_localization finished."

