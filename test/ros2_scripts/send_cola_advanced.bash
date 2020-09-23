#!/bin/bash

#
# Run examples for advanced Cola-ASCII telegrams and sick_lidar_localization services provided by sim_loc_driver.
# Testset for the new sick_lidar_localization services implemented september 2020.
#
# Note: ROS sim_loc_driver and test server should be up and running, start by
# ros2 launch sick_lidar_localization sim_loc_test_server.launch &
# ros2 launch sick_lidar_localization sim_loc_driver.launch &
#

#
# Function call_service "<servicename>" "<servicetype>"  "<parameter>" "<expected answer>" just calls a ros service
# Example: call_service SickLocIsSystemReady sick_lidar_localization/srv/SickLocIsSystemReadySrv "{}" "success: True" # Check if the system is ready
#
function call_service() {
  sleep 1
  echo -e "ros2 service call $1 $2 \"$3\""
  answer=$(ros2 service call $1 $2 "$3") # call rosservice
  answer=$(echo $answer|tr -d '\n') # remove line feeds
  echo -e "expected answer: \"$4\""
  echo -e "received answer: \"$answer\""
  if [[ "$answer" == *"$4"* ]]; then
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
source /opt/ros/eloquent/setup.bash
source ./install/setup.bash
testcase_counter=0
testcase_error_counter=0

#
# Call services and check answer
#

for ((n=0;n<1;n++)) ; do
  # service    servicename          servicetype                                       request  expected response
  call_service SickLocIsSystemReady sick_lidar_localization/srv/SickLocIsSystemReadySrv "{}" "success=True"
  call_service SickLocStop sick_lidar_localization/srv/SickLocStopSrv "{}" "success=True"
  call_service SickLocStartLocalizing sick_lidar_localization/srv/SickLocStartLocalizingSrv "{}" "success=True"
  # ros2 service call SickLocColaTelegram sick_lidar_localization/srv/SickLocColaTelegramSrv "{cola_ascii_request: 'sMN LocStartLocalizing', wait_response_timeout: 1}"
  call_service SickDevSetLidarConfig sick_lidar_localization/srv/SickDevSetLidarConfigSrv "{index: 0, minrange: 100, maxrange: 12400, minangle: 0, maxangle: 180000, x: 1000, y: -1000, yaw: 0, upsidedown: false, ip: 192.168.1.30, port: 2111, interfacetype: 0, maplayer: 0, active: true}" "set=True, executed=True"
  call_service SickDevGetLidarConfig sick_lidar_localization/srv/SickDevGetLidarConfigSrv "{scannerindex: 0}" "minrange=100, maxrange=12400, minangle=0, maxangle=180000, x=1000, y=-1000, yaw=0, upsidedown=False, ip='192.168.1.30', port=2111, interfacetype=0, maplayer=0, active=True"
  # ros2 service call SickLocColaTelegram sick_lidar_localization/srv/SickLocColaTelegramSrv "{cola_ascii_request: 'sMN DevSetLidarConfig +0 +300 +100000 -180000 +180000 +1000 -300 -45000 0 +12 192.168.1.30 +2111 0 0 1', wait_response_timeout: 1}"
  # ros2 service call SickLocColaTelegram sick_lidar_localization/srv/SickLocColaTelegramSrv "{cola_ascii_request: 'sMN DevGetLidarConfig 0', wait_response_timeout: 1}"
  call_service SickLocSetMap sick_lidar_localization/srv/SickLocSetMapSrv "{mapfilename: demo_map.smap}" "set=True, executed=True"
  call_service SickLocMap sick_lidar_localization/srv/SickLocMapSrv "{}" "map='demo_map.smap', success=True"
  call_service SickLocMapState sick_lidar_localization/srv/SickLocMapStateSrv "{}" "mapstate=True, success=True"
  call_service SickLocInitializePose sick_lidar_localization/srv/SickLocInitializePoseSrv "{x: 100, y: -100, yaw: 10, sigmatranslation: 1000}" "success=True"
  # ros2 service call SickLocColaTelegram sick_lidar_localization/srv/SickLocColaTelegramSrv "{cola_ascii_request: 'sMN LocInitializePose +10300 -5200 +30000 +1000', wait_response_timeout: 1}"
  call_service SickLocInitialPose sick_lidar_localization/srv/SickLocInitialPoseSrv "{}" "x=100, y=-100, yaw=10, sigmatranslation=1000, success=True"
  call_service SickLocSetReflectorsForSupportActive sick_lidar_localization/srv/SickLocSetReflectorsForSupportActiveSrv "{active: 1}" "success=True"
  call_service SickLocReflectorsForSupportActive sick_lidar_localization/srv/SickLocReflectorsForSupportActiveSrv "{}" "active=True, success=True"
  call_service SickLocSetOdometryActive sick_lidar_localization/srv/SickLocSetOdometryActiveSrv "{active: 1}" "set=True, executed=True"
  call_service SickLocOdometryActive sick_lidar_localization/srv/SickLocOdometryActiveSrv "{}" "active=True, success=True"
  call_service SickLocSetOdometryPort sick_lidar_localization/srv/SickLocSetOdometryPortSrv "{port: 3000}" "set=True, executed=True"
  call_service SickLocOdometryPort sick_lidar_localization/srv/SickLocOdometryPortSrv "{}" "port=3000, success=True"
  call_service SickLocSetOdometryRestrictYMotion sick_lidar_localization/srv/SickLocSetOdometryRestrictYMotionSrv "{active: 1}" "success=True"
  call_service SickLocOdometryRestrictYMotion sick_lidar_localization/srv/SickLocOdometryRestrictYMotionSrv "{}" "active=True, success=True"
  call_service SickLocSetAutoStartActive sick_lidar_localization/srv/SickLocSetAutoStartActiveSrv "{active: 1}" "success=True"
  call_service SickLocAutoStartActive sick_lidar_localization/srv/SickLocAutoStartActiveSrv "{}" "active=True, success=True"
  call_service SickLocSetAutoStartSavePoseInterval sick_lidar_localization/srv/SickLocSetAutoStartSavePoseIntervalSrv "{interval: 5}" "success=True"
  call_service SickLocAutoStartSavePoseInterval sick_lidar_localization/srv/SickLocAutoStartSavePoseIntervalSrv "{}" "interval=5, success=True"
  call_service SickLocSetRingBufferRecordingActive sick_lidar_localization/srv/SickLocSetRingBufferRecordingActiveSrv "{active: 1}" "success=True"
  call_service SickLocRingBufferRecordingActive sick_lidar_localization/srv/SickLocRingBufferRecordingActiveSrv "{}" "active=True, success=True"
  call_service SickDevGetLidarIdent sick_lidar_localization/srv/SickDevGetLidarIdentSrv "{index: 0}" "scannerident='TestcaseGenerator0', success=True"
  call_service SickDevGetLidarState sick_lidar_localization/srv/SickDevGetLidarStateSrv "{index: 0}" "devicestatus=2, deviceconnected=2, receivingdata=2, success=True"
  call_service SickGetSoftwareVersion sick_lidar_localization/srv/SickGetSoftwareVersionSrv "{}" "version='1.0', success=True"
  call_service SickLocAutoStartSavePose sick_lidar_localization/srv/SickLocAutoStartSavePoseSrv "{}" "success=True"
  call_service SickLocForceUpdate sick_lidar_localization/srv/SickLocForceUpdateSrv "{}" "success=True"
  call_service SickLocSaveRingBufferRecording sick_lidar_localization/srv/SickLocSaveRingBufferRecordingSrv "{reason: test}"
  # ros2 service call SickLocColaTelegram sick_lidar_localization/srv/SickLocColaTelegramSrv "{cola_ascii_request: 'sMN LocSaveRingBufferRecording +36 YYYY-MM-DD_HH-MM-SS pose quality low', wait_response_timeout: 1}"
  call_service SickLocStop sick_lidar_localization/srv/SickLocStopSrv "{}" "success=True"
  call_service SickLocStartDemoMapping sick_lidar_localization/srv/SickLocStartDemoMappingSrv "{}"
  # ros2 service call SickLocColaTelegram sick_lidar_localization/srv/SickLocColaTelegramSrv "{cola_ascii_request: 'sMN LocStartDemoMapping', wait_response_timeout: 1}"
  call_service SickLocStop sick_lidar_localization/srv/SickLocStopSrv "{}" "success=True"
  call_service SickLocStartLocalizing sick_lidar_localization/srv/SickLocStartLocalizingSrv "{}" "success=True"
  call_service SickReportUserMessage sick_lidar_localization/srv/SickReportUserMessageSrv "{usermessage: test_message}" "success=True"
  call_service SickSavePermanent sick_lidar_localization/srv/SickSavePermanentSrv "{}" "success=True"
  call_service SickLocResultPort sick_lidar_localization/srv/SickLocResultPortSrv "{}" "port=2201, success=True"
  call_service SickLocResultMode sick_lidar_localization/srv/SickLocResultModeSrv "{}" "mode=0, success=True"
  call_service SickLocResultEndianness sick_lidar_localization/srv/SickLocResultEndiannessSrv "{}" "endianness=0, success=True"
  call_service SickLocResultState sick_lidar_localization/srv/SickLocResultStateSrv "{}" "state=1, success=True"
  call_service SickLocResultPoseInterval sick_lidar_localization/srv/SickLocResultPoseIntervalSrv "{}" "interval=1, success=True"
  call_service SickDevSetIMUActive sick_lidar_localization/srv/SickDevSetIMUActiveSrv "{active: 1}" "success=True"
  call_service SickDevIMUActive sick_lidar_localization/srv/SickDevIMUActiveSrv "{}" "active=True, success=True"
done

echo -e "send_cola_advanced.bash finished.\n"
echo -e "Advanced services and cola telegram verification summary: $testcase_counter testcases, $testcase_error_counter failures.\n"

