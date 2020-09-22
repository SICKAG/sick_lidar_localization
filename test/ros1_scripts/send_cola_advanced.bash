#!/bin/bash

#
# Run examples for Cola-ASCII telegrams and sick_lidar_localization services provided by sim_loc_driver.
# Testset for the new sick_lidar_localization services implemented september 2020.
#
# Note: ROS sim_loc_driver and test server should be up and running, start by
# roslaunch sick_lidar_localization sim_loc_test_server.launch &
# roslaunch sick_lidar_localization sim_loc_driver.launch localization_controller_ip_adress:=127.0.0.1 &
#

#
# Function call_service "<service>" "<parameter>" "<expected answer>" just calls a ros service
# Example: call_service "SickLocIsSystemReady" "{}" "success: True" # Check if the system is ready
#
function call_service() {
  sleep 1
  echo -e "rosservice call $1 \"$2\""
  answer=$(rosservice call $1 "$2") # call rosservice
  answer=$(echo $answer|tr -d '\n') # remove line feeds
  echo -e "expected answer: \"$3\""
  echo -e "received answer: \"$answer\""
  if [[ "$answer" == *"$3"* ]]; then
    echo -e "OK: service responded with expected answer.\n"
  else
    testcase_error_counter=$((testcase_error_counter+1))
    echo -e "## ERROR: service did NOT respond with expected answer. Press any key to continue..." ; read -n1 -s key 
  fi
  testcase_counter=$((testcase_counter+1))
}

#
# Environment
#

pushd ../../../..
source ./devel/setup.bash
testcase_counter=0
testcase_error_counter=0

#
# Call services and check answer
#

for ((n=0;n<1;n++)) ; do
  # service    servicename        request  expected response
  call_service SickLocIsSystemReady "{}" "success: True"
  call_service SickLocStartLocalizing "{}" "success: True"
  call_service SickDevSetLidarConfig "{index: 0, minrange: 100, maxrange: 200000, minangle: -15000, maxangle: 15000, x: 1000, y: -1000, yaw: 2000, upsidedown: true, ip: 192.168.1.30, port: 2111, interfacetype: 0, maplayer: 0, active: true}" "set: True executed: True"
  call_service SickDevGetLidarConfig "{scannerindex: 0}" "minrange: 100 maxrange: 200000 minangle: -15000 maxangle: 15000 x: 1000 y: -1000 yaw: 2000 upsidedown: True ip: \"192.168.1.30\" port: 2111 interfacetype: 0 maplayer: 0 active: True"
  call_service SickLocSetMap "{mapfilename: test.map}" "set: True executed: True"
  call_service SickLocMap "{}" "map: \"test.map\" success: True"
  call_service SickLocMapState "{}" "mapstate: True success: True"
  call_service SickLocInitializePose "{x: 100, y: -100, yaw: 2000, sigmatranslation: 1000}" "success: True"
  call_service SickLocInitialPose "{}" "x: 100 y: -100 yaw: 2000 sigmatranslation: 1000 success: True"
  call_service SickLocSetReflectorsForSupportActive "{active: 1}" "success: True"
  call_service SickLocReflectorsForSupportActive "{}" "active: True success: True"
  call_service SickLocSetOdometryActive "{active: 1}" "set: True executed: True"
  call_service SickLocOdometryActive "{}" "active: True success: True"
  call_service SickLocSetOdometryPort "{port: 3000}" "set: True executed: True"
  call_service SickLocOdometryPort "{}" "port: 3000 success: True"
  call_service SickLocSetOdometryRestrictYMotion "{active: 1}" "success: True"
  call_service SickLocOdometryRestrictYMotion "{}" "active: True success: True"
  call_service SickLocSetAutoStartActive "{active: 1}" "success: True"
  call_service SickLocAutoStartActive "{}" "active: True success: True"
  call_service SickLocSetAutoStartSavePoseInterval "{interval: 5}" "success: True"
  call_service SickLocAutoStartSavePoseInterval "{}" "interval: 5 success: True"
  call_service SickLocSetRingBufferRecordingActive "{active: 1}" "success: True"
  call_service SickLocRingBufferRecordingActive "{}" "active: True success: True"
  call_service SickDevGetLidarIdent "{index: 0}" "scannerident: \"TestcaseGenerator0\" success: True"
  call_service SickDevGetLidarState "{index: 0}" "devicestatus: 2 deviceconnected: 2 receivingdata: 2 success: True"
  call_service SickGetSoftwareVersion "{}" "version: \"1.0\" success: True"
  call_service SickLocAutoStartSavePose "{}" "success: True"
  call_service SickLocForceUpdate "{}" "success: True"
  call_service SickLocSaveRingBufferRecording "{reason: test}" "success: True"
  call_service SickLocStop "{}" "success: True"
  call_service SickLocStartDemoMapping "{}" "success: True"
  call_service SickLocStop "{}" "success: True"
  call_service SickLocStartLocalizing "{}" "success: True"  
  call_service SickReportUserMessage "{usermessage: test_message}" "success: True"
  call_service SickSavePermanent "{}" "success: True"
  call_service SickLocResultPort "{}" "port: 2201 success: True"
  call_service SickLocResultMode "{}" "mode: 0 success: True"
  call_service SickLocResultEndianness "{}" "endianness: 0 success: True"
  call_service SickLocResultState "{}" "state: 1 success: True"
  call_service SickLocResultPoseInterval "{}" "interval: 1 success: True"
  call_service SickDevSetIMUActive "{active: 1}" "success: True"
  call_service SickDevIMUActive "{}" "active: True success: True"
done

echo -e "send_cola_advanced.bash finished.\n"
echo -e "Advanced services and cola telegram verification summary: $testcase_counter testcases, $testcase_error_counter failures.\n"
