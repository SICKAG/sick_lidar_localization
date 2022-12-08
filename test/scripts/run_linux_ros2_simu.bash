#!/bin/bash

#
# Function call_service "<servicename>" "<servicetype>" "<parameter>" "<expected answer>" just calls a ros service
# Example: call_service LocIsSystemReady sick_lidar_localization/srv/LocIsSystemReadySrv "{}" "success: True"
#
function call_service()
{
  echo -e "ros2 service call $1 $2 \"$3\""
  answer=$(ros2 service call $1 $2 "$3") # call rosservice
  answer=$(echo $answer|tr -d '\n') # remove line feeds
  echo -e "expected answer: \"$4\""
  echo -e "received answer: \"$answer\""
  if [[ "$answer" == *"$4"* ]]; then
    echo -e "OK: service responded with expected answer.\n"
  else
    echo -e "## ERROR: service did NOT respond with expected answer. Press any key to continue..." ; read -n1 -s key 
  fi
} 

#
# Function unittest_services just runs a unittest with ros service examples
#
function unittest_services()
{
  call_service LocGetErrorLevel sick_lidar_localization/srv/LocGetErrorLevelSrv "{}" "level=0, description='No error', success=True"
  call_service LocIsSystemReady sick_lidar_localization/srv/LocIsSystemReadySrv "{}" "success=True"
  call_service LocAutoStartSavePose sick_lidar_localization/srv/LocAutoStartSavePoseSrv "{}" "success=True"
  call_service LocClearMapCache sick_lidar_localization/srv/LocClearMapCacheSrv "{}" "success=True"
  call_service LocGetMap sick_lidar_localization/srv/LocGetMapSrv "{}" "mappath='test.vmap', success=True"
  call_service LocGetSystemState sick_lidar_localization/srv/LocGetSystemStateSrv "{}" "systemstate='LOCALIZING', success=True"
  call_service LocInitializeAtPose sick_lidar_localization/srv/LocInitializeAtPoseSrv "{x: 1000, y: 1000, yaw: 1000, searchradius: 1000}" "success=True"
  call_service LocLoadMapToCache sick_lidar_localization/srv/LocLoadMapToCacheSrv "{mappath: \"test.vmap\"}" "success=True"
  call_service LocRequestTimestamp sick_lidar_localization/srv/LocRequestTimestampSrv "{}" "timestamp_lidar_microsec="
  call_service LocResumeAtPose sick_lidar_localization/srv/LocResumeAtPoseSrv "{x: 1000, y: 1000, yaw: 1000}" "success=True"
  call_service LocSaveRingBufferRecording sick_lidar_localization/srv/LocSaveRingBufferRecordingSrv "{reason: \"YYYY-MM-DD_HH-MM-SS pose quality low\"}" "success=True"
  call_service LocSetKinematicVehicleModelActive sick_lidar_localization/srv/LocSetKinematicVehicleModelActiveSrv "{active: 1}" "success=True"
  call_service LocSetLinesForSupportActive sick_lidar_localization/srv/LocSetLinesForSupportActiveSrv "{active: 1}" "success=True"
  call_service LocSetMap sick_lidar_localization/srv/LocSetMapSrv "{mappath: \"test.vmap\"}" "success=True"
  call_service LocSetMappingActive sick_lidar_localization/srv/LocSetMappingActiveSrv "{active: 1}" "success=True"
  call_service LocSetOdometryActive sick_lidar_localization/srv/LocSetOdometryActiveSrv "{active: 1}" "success=True"
  call_service LocSetRecordingActive sick_lidar_localization/srv/LocSetRecordingActiveSrv "{active: 1}" "success=True"
  call_service LocSetRingBufferRecordingActive sick_lidar_localization/srv/LocSetRingBufferRecordingActiveSrv "{active: 1}" "success=True"
  call_service LocStartLocalizing sick_lidar_localization/srv/LocStartLocalizingSrv "{}" "success=True"
  call_service LocStop sick_lidar_localization/srv/LocStopSrv "{}" "success=True"
  call_service LocSwitchMap sick_lidar_localization/srv/LocSwitchMapSrv "{submapname: \"test.vmap\"}" "success=True"
  call_service LocGetLocalizationStatus sick_lidar_localization/srv/LocGetLocalizationStatusSrv "{}" "locstatus=1"
  call_service LocGetSoftwareVersion sick_lidar_localization/srv/LocGetSoftwareVersionSrv "{}" "version='LLS 2.0.0.14R'"
  call_service LocLoadPersistentConfig sick_lidar_localization/srv/LocLoadPersistentConfigSrv "{}" "success=True"
}

# 
# Run sick_lidar_localization simu on native linux
# 
killall sick_lidar_localization
printf "\033c"

# 
# Start rest server
# 

echo -e "(Re-)starting sudo sick_rest_server.py ...\n" 
pkill -f roslaunch > /dev/null 2>&1
pkill -f rosmaster > /dev/null 2>&1
sudo pkill -f sick_rest_server.py > /dev/null 2>&1
sudo pkill -9 -f sick_rest_server.py > /dev/null 2>&1
sudo python3 ../rest_server/python/sick_rest_server.py --time_sync=1 &
sleep 3

# 
# Start sick_lidar_localization
# 

pushd ../../../..
if [ -f /opt/ros/humble/setup.bash ] ; then 
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/foxy/setup.bash ] ; then 
    source /opt/ros/foxy/setup.bash
fi
source ./install/setup.bash 
SICK_LIDAR_LOCALIZATION_ROOT=./src/sick_lidar_localization
if [ -d ./src/sick_lidar_localization2_pretest ] ; then SICK_LIDAR_LOCALIZATION_ROOT=./src/sick_lidar_localization2_pretest ; fi
ros2 run sick_lidar_localization sick_lidar_localization $SICK_LIDAR_LOCALIZATION_ROOT/launch/sick_lidar_localization.launch --ros-args -p hostname:=localhost -p udp_ip_sim_input:=127.0.0.1 -p udp_ip_sim_output:=localhost -p verbose:=1 &
sleep 3

# 
# Run services
# 

ros2 service list
ros2 service call LocIsSystemReady sick_lidar_localization/srv/LocIsSystemReadySrv "{}"
unittest_services
sleep 3

# 
# Start udp sender with synthetical data
# 

ros2 topic echo /localizationcontroller/out/odometry_message_0104 &
ros2 topic echo /localizationcontroller/out/odometry_message_0105 &
ros2 topic echo /localizationcontroller/out/code_measurement_message_0304 &
ros2 topic echo /localizationcontroller/out/line_measurement_message_0403 &
ros2 topic echo /localizationcontroller/out/line_measurement_message_0404 &
ros2 topic echo /localizationcontroller/out/localizationcontroller_result_message_0502 &
python3 $SICK_LIDAR_LOCALIZATION_ROOT/test/rest_server/python/sim_udp_sender.py --udp_port=5010 --udp_send_rate=30.0 --max_message_count=300
sleep 3

#
# Run pointcloud_converter and visualize PointCloud2 messages by rviz:
# rviz -> Add by topic /cloud/PointCloud2
# rviz -> Add by display type TF
#

ros2 run sick_lidar_localization pointcloud_converter $SICK_LIDAR_LOCALIZATION_ROOT/launch/pointcloud_converter.launch &
sleep 3
rviz2 -d $SICK_LIDAR_LOCALIZATION_ROOT/test/config/rviz2_sick_lidar_localization_pointcloud.rviz2 & 
sleep 3

# 
# Start pcapng player sending recorded UDP messages
# 

python3 $SICK_LIDAR_LOCALIZATION_ROOT/test/rest_server/python/sim_pcapng_player.py  --pcap_filename $SICK_LIDAR_LOCALIZATION_ROOT/test/data/wireshark/20210816_lidarloc2_2.0.0.14R_moving.pcapng # 20210816_lidarloc2_2.0.0.14R_nonmoving.pcapng
sleep 3

# 
# Start udp receiver and send UDP input message examples
# 

python3 $SICK_LIDAR_LOCALIZATION_ROOT/test/rest_server/python/sim_udp_receiver.py --udp_port=5009 &
ros2 topic pub --once /localizationcontroller/in/odometry_message_0101            sick_lidar_localization/OdometryMessage0101           '{telegram_count: 1000001, timestamp: 123456789, x_velocity: -1234, y_velocity: -1234, angular_velocity: 1234}'
ros2 topic pub --once /localizationcontroller/in/odometry_message_0104            sick_lidar_localization/OdometryMessage0104           '{telegram_count: 1000001, timestamp: 123456789, x_velocity: -1234, y_velocity: -1234, angular_velocity: 1234}'
ros2 topic pub --once /localizationcontroller/in/odometry_message_0105            sick_lidar_localization/OdometryMessage0105           '{telegram_count: 1000002, timestamp: 123456780, x_position: -1234, y_position: -1234, heading: 1234}'
ros2 topic pub --once /localizationcontroller/in/encoder_measurement_message_0202 sick_lidar_localization/EncoderMeasurementMessage0202 '{telegram_count: 1000003, timestamp: 123456781, encoder_value: 123456789}'
ros2 topic pub --once /localizationcontroller/in/code_measurement_message_0303    sick_lidar_localization/CodeMeasurementMessage0303    '{telegram_count: 1000004, timestamp: 123456782, code: 1234}'
ros2 topic pub --once /localizationcontroller/in/line_measurement_message_0403    sick_lidar_localization/LineMeasurementMessage0403    '{telegram_count: 1000005, timestamp: 123456783, num_lanes: 1, lanes: [1234]}'
ros2 topic pub --once /localizationcontroller/in/line_measurement_message_0404    sick_lidar_localization/LineMeasurementMessage0404    '{telegram_count: 1000006, timestamp: 123456784, lcp1: 12, lcp2: 34, lcp3: 56, cnt_lpc: 78}'
python3 $SICK_LIDAR_LOCALIZATION_ROOT/test/rest_server/python/send_ros2_odom_messages.py

# 
# Finish simulation
# 

sleep 3
popd
killall sick_lidar_localization
killall pointcloud_converter
pkill -f sim_udp_receiver.py
sudo pkill -f sick_rest_server.py
sudo pkill -9 -f sick_rest_server.py
pkill -f "ros2 topic echo"
echo -e "\nsick_lidar_localization simu finished.\n" 
