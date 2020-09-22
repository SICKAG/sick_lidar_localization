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
export sim_loc_test_server_demo_circles=1
export sim_loc_test_server_error_simulation=0

#
# Run test server, simulate localization controller for offline tests. Run in demo mode, create telegrams of a sensor moving in circles.
#

ros2 launch sick_lidar_localization sim_loc_test_server.launch.py &
# ros2 run sick_lidar_localization sim_loc_test_server &
sleep 3 # make sure ros core and sim_loc_test_server are up and running 

#
# Run ros driver, connect to localization controller and receive, convert and publish telegrams
#

ros2 launch sick_lidar_localization sim_loc_driver.launch.py &
# ros2 param dump sim_loc_driver --print 
# ros2 run sick_lidar_localization sim_loc_driver &
# ros2 run sick_lidar_localization cola_service_node &
# ros2 run sick_lidar_localization sim_loc_time_sync &
# ros2 run sick_lidar_localization pointcloud_converter &

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

# ros2 topic echo /cloud
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map pointcloud_sick_lidar_localization &
ros2 run rviz2 rviz2 -d ./src/sick_lidar_localization_pretest/test/config/rviz2_sick_lidar_localization_demo_pointcloud.rviz &
ros2 run rviz2 rviz2 -d ./src/sick_lidar_localization_pretest/test/config/rviz2_sick_lidar_localization_demo_tf.rviz &

#
# Run test server and driver for some time and exit
#

sleep 600
popd
./killall.bash
killall rviz2
killall static_transform_publisher
echo -e "\nrun_demo_simu for sick_lidar_localization finished."

