#!/bin/bash

#
# Run demo simulation (offline-test) of sick_lidar_localization driver against local test server simulating a localization controller.
# Demo simulation shows a sensor moving in circles.
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
# Run test server, simulate localization controller for offline tests. Run in demo mode, create telegrams of a sensor moving in circles.
#

roslaunch sick_lidar_localization sim_loc_test_server.launch demo_circles:=true &
sleep 3 # make sure ros core and sim_loc_test_server are up and running 

#
# Run ros driver, connect to localization controller and receive, convert and publish telegrams
#

roslaunch sick_lidar_localization sim_loc_driver.launch localization_controller_ip_address:=127.0.0.1 &

#
# Visualize PointCloud2 messages by rviz:
#
# rviz -> Add by topic /cloud/PointCloud2
#      -> Style: Points
#      -> Size (Pixels): 5
#
# Visualize TF messages by rviz:
#
# rviz -> Global options: Fixed Frame -> tf_demo_map
#      -> Add by display type TF
#

rosrun tf static_transform_publisher 0 0 0 0 0 0 map pointcloud_sick_lidar_localization 10 &
rosrun rviz rviz -d ./src/sick_lidar_localization_pretest/test/config/rviz_sick_lidar_localization_demo_pointcloud.rviz &
rosrun rviz rviz -d ./src/sick_lidar_localization_pretest/test/config/rviz_sick_lidar_localization_demo_tf.rviz &

#
# Run test server and driver for some time and exit
#

sleep 600
if [ -d ./src/sick_lidar_localization ]         ; then ./src/sick_lidar_localization/test/ros1_scripts/killall.bash         ; fi
if [ -d ./src/sick_lidar_localization_pretest ] ; then ./src/sick_lidar_localization_pretest/test/ros1_scripts/killall.bash ; fi
killall roslaunch
killall rviz
killall static_transform_publisher
echo -e "\nrun_demo_simu for sick_lidar_localization finished."

