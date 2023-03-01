#!/bin/bash

#
# Function call_service "<service>" "<parameter>" "<expected answer>" just calls a ros service
# Example: call_service LocIsSystemReady "{}" "success: True"
#
function call_service() 
{
  echo -e "rosservice call $1 \"$2\""
  answer=$(rosservice call $1 "$2") # call rosservice
  answer=$(echo $answer|tr -d '\n') # remove line feeds
  echo -e "expected answer: \"$3\""
  echo -e "received answer: \"$answer\""
  if [[ "$answer" == *"$3"* ]]; then
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
  call_service LocGetErrorLevel "{}" "level: 0 description: \"No error\" success: True"
  call_service LocIsSystemReady "{}" "success: True"
  call_service LocAutoStartSavePose "{}" "success: True"
  call_service LocClearMapCache "{}" "success: True"
  call_service LocGetMap "{}" "mappath: \"test.vmap\" success: True"
  call_service LocGetSystemState "{}" "systemstate: \"LOCALIZING\" success: True"
  call_service LocInitializeAtPose "{x: 1000, y: 1000, yaw: 1000, searchradius: 1000}" "success: True"
  call_service LocLoadMapToCache "{mappath: \"test.vmap\"}" "success: True"
  call_service LocRequestTimestamp "{}" "timestamp_lidar_microsec:"
  call_service LocResumeAtPose "{x: 1000, y: 1000, yaw: 1000}" "success: True"
  call_service LocSaveRingBufferRecording "{reason: \"YYYY-MM-DD_HH-MM-SS pose quality low\"}" "success: True"
  call_service LocSetKinematicVehicleModelActive "{active: 1}" "success: True"
  call_service LocSetLinesForSupportActive "{active: 1}" "success: True"
  call_service LocSetMap "{mappath: \"test.vmap\"}" "success: True"
  call_service LocSetMappingActive "{active: 1}" "success: True"
  call_service LocSetOdometryActive "{active: 1}" "success: True"
  call_service LocSetRecordingActive "{active: 1}" "success: True"
  call_service LocSetRingBufferRecordingActive "{active: 1}" "success: True"
  call_service LocStart "{}" "success: True"
  call_service LocStop "{}" "success: True"
  call_service LocSwitchMap "{submapname: \"test.vmap\"}" "success: True"
  call_service LocGetLocalizationStatus "{}" "locstatus: 1"
  call_service LocGetSoftwareVersion "{}" "version: \"LLS 2.0.0.14R\""
  call_service LocLoadPersistentConfig "{}" "success: True"
} 

# 
# Run sick_lidar_localization simu on Linux ROS-1 
# 
printf "\033c"

# 
# Start rest server
# 

echo -e "Starting sudo sick_rest_server.py ...\n" 
sudo pkill -f sick_rest_server.py
sudo python3 ../rest_server/python/sick_rest_server.py --time_sync=1 &
sleep 3

# 
# Start sick_lidar_localization
# 

pushd ../../../..
source /opt/ros/noetic/setup.bash
source ./install/setup.bash 
SICK_LIDAR_LOCALIZATION_ROOT=./src/sick_lidar_localization
if [ -d ./src/sick_lidar_localization2_pretest ] ; then SICK_LIDAR_LOCALIZATION_ROOT=./src/sick_lidar_localization2_pretest ; fi
roslaunch sick_lidar_localization sick_lidar_localization.launch hostname:=localhost udp_ip_lls_input:=127.0.0.1 udp_ip_lls_output:=localhost verbose:=1 &
sleep 3

# 
# Run services
# 

rosservice list
rosservice call LocIsSystemReady "{}"
unittest_services
sleep 3

# 
# Start udp sender with synthetical data
# 

rostopic echo /localizationcontroller/out/odometry_message_0104 &
rostopic echo /localizationcontroller/out/odometry_message_0105 &
rostopic echo /localizationcontroller/out/code_measurement_message_0304 &
rostopic echo /localizationcontroller/out/line_measurement_message_0403 &
rostopic echo /localizationcontroller/out/line_measurement_message_0404 &
rostopic echo /localizationcontroller/out/localizationcontroller_result_message_0502 &
python3 $SICK_LIDAR_LOCALIZATION_ROOT/test/rest_server/python/lls_udp_sender.py --udp_port=5010 --udp_send_rate=30.0 --max_message_count=300
sleep 3

#
# Run lls_transform and visualize TF messages by rviz:
# rviz -> Add by display type TF
#

roslaunch sick_lidar_localization lls_transform.launch &
sleep 3
rosrun rviz rviz -d $SICK_LIDAR_LOCALIZATION_ROOT/test/config/rviz_sick_lidar_localization_pointcloud.rviz & 
sleep 3

# 
# Start pcapng player sending recorded UDP messages
# 

python3 $SICK_LIDAR_LOCALIZATION_ROOT/test/rest_server/python/lls_pcapng_player.py  --pcap_filename $SICK_LIDAR_LOCALIZATION_ROOT/test/data/wireshark/20210816_lidarloc2_2.0.0.14R_moving.pcapng # 20210816_lidarloc2_2.0.0.14R_nonmoving.pcapng
sleep 3

# 
# Start udp receiver and send UDP input message examples
# 

python3 $SICK_LIDAR_LOCALIZATION_ROOT/test/rest_server/python/lls_udp_receiver.py --udp_port=5009 &
rostopic pub --once /localizationcontroller/in/odometry_message_0104            sick_lidar_localization/OdometryMessage0104           '{telegram_count: 1000001, timestamp: 123456789, source_id: 0, x_velocity: -1234, y_velocity: -1234, angular_velocity: 1234}'
rostopic pub --once /localizationcontroller/in/odometry_message_0104            sick_lidar_localization/OdometryMessage0104           '{telegram_count: 1000001, timestamp: 123456789, source_id: 2, x_velocity: -1234, y_velocity: -1234, angular_velocity: 1234}'
rostopic pub --once /localizationcontroller/in/odometry_message_0105            sick_lidar_localization/OdometryMessage0105           '{telegram_count: 1000002, timestamp: 123456780, source_id: 0, x_position: -1234, y_position: -1234, heading: 1234}'
rostopic pub --once /localizationcontroller/in/odometry_message_0105            sick_lidar_localization/OdometryMessage0105           '{telegram_count: 1000002, timestamp: 123456780, source_id: 3, x_position: -1234, y_position: -1234, heading: 1234}'
rostopic pub --once /localizationcontroller/in/encoder_measurement_message_0202 sick_lidar_localization/EncoderMeasurementMessage0202 '{telegram_count: 1000003, timestamp: 123456781, source_id: 0, encoder_value: 123456789}'
rostopic pub --once /localizationcontroller/in/encoder_measurement_message_0202 sick_lidar_localization/EncoderMeasurementMessage0202 '{telegram_count: 1000003, timestamp: 123456781, source_id: 4, encoder_value: 123456789}'
rostopic pub --once /localizationcontroller/in/code_measurement_message_0303    sick_lidar_localization/CodeMeasurementMessage0303    '{telegram_count: 1000004, timestamp: 123456782, source_id: 0, code: 1234}'
rostopic pub --once /localizationcontroller/in/code_measurement_message_0303    sick_lidar_localization/CodeMeasurementMessage0303    '{telegram_count: 1000004, timestamp: 123456782, source_id: 5, code: 1234}'
rostopic pub --once /localizationcontroller/in/code_measurement_message_0701    sick_lidar_localization/CodeMeasurementMessage0701    '{telegram_count: 1000004, timestamp: 123456782, source_id: 0, code: "1234", x_position: -1234, y_position: -2345, heading: -3456}'
rostopic pub --once /localizationcontroller/in/code_measurement_message_0701    sick_lidar_localization/CodeMeasurementMessage0701    '{telegram_count: 1000004, timestamp: 123456782, source_id: 6, code: "1234", x_position: -1234, y_position: -2345, heading: -3456}'
rostopic pub --once /localizationcontroller/in/line_measurement_message_0403    sick_lidar_localization/LineMeasurementMessage0403    '{telegram_count: 1000005, timestamp: 123456783, source_id: 0, num_lanes: 1, lanes: [1234]}'
rostopic pub --once /localizationcontroller/in/line_measurement_message_0403    sick_lidar_localization/LineMeasurementMessage0403    '{telegram_count: 1000005, timestamp: 123456783, source_id: 7, num_lanes: 1, lanes: [1234]}'
rostopic pub --once /localizationcontroller/in/line_measurement_message_0404    sick_lidar_localization/LineMeasurementMessage0404    '{telegram_count: 1000006, timestamp: 123456784, source_id: 0, lcp1: 12, lcp2: 34, lcp3: 56, cnt_lpc: 78}'
rostopic pub --once /localizationcontroller/in/line_measurement_message_0404    sick_lidar_localization/LineMeasurementMessage0404    '{telegram_count: 1000006, timestamp: 123456784, source_id: 8, lcp1: 12, lcp2: 34, lcp3: 56, cnt_lpc: 78}'
python3 $SICK_LIDAR_LOCALIZATION_ROOT/test/rest_server/python/send_ros_odom_messages.py
python3 $SICK_LIDAR_LOCALIZATION_ROOT/test/rest_server/python/send_odometry_message_0104.py
python3 $SICK_LIDAR_LOCALIZATION_ROOT/test/rest_server/python/send_odometry_message_0105.py

# 
# Finish simulation
# 

echo -e "\nsick_lidar_localization simu finished, cleaning up\n" 
sleep 3
popd
rosnode kill -a
pkill -f lls_udp_receiver.py
sudo pkill -f sick_rest_server.py
killall sick_lidar_localization
killall lls_transform

echo -e "\nsick_lidar_localization simu finished.\n" 
