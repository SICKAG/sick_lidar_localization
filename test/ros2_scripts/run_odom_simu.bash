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
export ROS_VERSION=2

#
# Start test server and sim_loc_driver
#

ros2 launch sick_lidar_localization sim_loc_test_server.launch.py > ~/.ros/log/sim_loc_test_server.log &
sleep 3 # make sure ros core and sim_loc_test_server are up and running 
ros2 launch sick_lidar_localization sim_loc_driver.launch.py > ~/.ros/log/sim_loc_driver.log &
sleep 3 # make sure sim_loc_driver is up and running before sending and verifying odometry telegrams

#
# Visualize odom messages by rviz:
#

ros2 run rviz2 rviz2 -d ./src/sick_lidar_localization_pretest/test/config/rviz2_odom_msg_sender.rviz &

#
# Disable firewall on udp port 3000
#
# sudo iptables -A INPUT -p udp -m udp --dport 3000 -j ACCEPT
# sudo iptables -A OUTPUT -p udp -m udp --sport 3000 -j ACCEPT
# sudo iptables-save
# sudo ufw allow from any to any port 3000 proto udp 
# sudo ufw status

#
# Start odometry message sender, send 100 odometry messages and verify udp telegrams from sim_loc_driver
#

# ros2 service call SickLocSetOdometryActive sick_lidar_localization/srv/SickLocSetOdometryActiveSrv "{active: 1}"
# ros2 service call SickLocSetOdometryPort sick_lidar_localization/srv/SickLocSetOdometryPortSrv "{port: 3000}"
# ros2 service call SickLocSetOdometryRestrictYMotion sick_lidar_localization/srv/SickLocSetOdometryRestrictYMotionSrv "{active: 0}"
python3 ./src/sick_lidar_localization_pretest/test/python/odom_msg_sender.py

popd
./killall.bash
pkill -f 'python3 ./src/sick_lidar_localization_pretest/test/python/odom_msg_sender.py'
echo -e "\nrun_odom_simu for sick_lidar_localization finished."

