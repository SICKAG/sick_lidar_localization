#!/bin/bash

#
# Run examples for Cola-ASCII telegrams and sick_lidar_localization services provided by sim_loc_driver
#
# Note: ROS sim_loc_driver and test server should be up and running, start by
# roslaunch sick_lidar_localization sim_loc_test_server.launch &
# roslaunch sick_lidar_localization sim_loc_driver.launch localization_controller_ip_adress:=127.0.0.1 &
#

#
# Function send_cola_telegram "<command>" "<expected answer>" sends and echos a cola telegram using ros service SickLocColaTelegram
# Example: send_cola_telegram "{cola_ascii_request: 'sMN IsSystemReady', wait_response_timeout: 1}" "sAN IsSystemReady 1"
#
function send_cola_telegram() {
  sleep 1
  echo -e "rosservice call SickLocColaTelegram \"$1\""
  echo -e "expected answer:     \"$2\""
  answer=$(rosservice call SickLocColaTelegram "$1")
  echo -e "$answer"
  if [[ "$answer" == *"$2"* ]]; then
    echo -e "OK: service responded with expected answer.\n"
  else
    testcase_error_counter=$((testcase_error_counter+1))
    echo -e "## ERROR: service did NOT respond with expected answer. Press any key to continue..." ; read -n1 -s key 
  fi
  testcase_counter=$((testcase_counter+1))
}

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
# Toggle localization on/off and result telegrams on/off
#

for ((n=0;n<1;n++)) ; do
  call_service SickLocIsSystemReady "{}"    "success: True"                 # Check if the system is ready
  call_service SickLocState "{}"            "state: 2 success: True"        # Read localization state: 0:BOOTING, 1:IDLE, 2:LOCALIZING, 3:DEMO_MAPPING
  call_service SickLocStop "{}"             "success: True"                 # Stop localization or demo mapping
  call_service SickLocState "{}"            "state: 1 success: True"        # Read localization state: 0:BOOTING, 1:IDLE, 2:LOCALIZING, 3:DEMO_MAPPING
  call_service SickLocState "{}"            "state: 1 success: True"        # Read localization state: 0:BOOTING, 1:IDLE, 2:LOCALIZING, 3:DEMO_MAPPING
  call_service SickLocStartLocalizing  "{}" "success: True"                 # Start localization
  call_service SickLocState "{}"            "state: 2 success: True"        # Read localization state: 0:BOOTING, 1:IDLE, 2:LOCALIZING, 3:DEMO_MAPPING
  call_service SickLocState "{}"            "state: 2 success: True"        # Read localization state: 0:BOOTING, 1:IDLE, 2:LOCALIZING, 3:DEMO_MAPPING
  call_service SickLocSetResultPoseEnabled "{enabled: 0}" "success: True"                                          # Disable/enable result output
  send_cola_telegram "{cola_ascii_request: 'sRN LocResultState', wait_response_timeout: 1}" "sRA LocResultState 0" # 0:disabled, 1:enabled, MSB: error flag # Read state of the result output
  call_service SickLocState "{}"            "state: 2 success: True"                                               # Read localization state: 0:BOOTING, 1:IDLE, 2:LOCALIZING, 3:DEMO_MAPPING
  call_service SickLocSetResultPoseEnabled "{enabled: 1}" "success: True"                                          # Disable/enable result output
  send_cola_telegram "{cola_ascii_request: 'sRN LocResultState', wait_response_timeout: 1}" "sRA LocResultState 1" # 0:disabled, 1:enabled, MSB: error flag # Read state of the result output
  call_service SickLocState "{}"            "state: 2 success: True"                                               # Read localization state: 0:BOOTING, 1:IDLE, 2:LOCALIZING, 3:DEMO_MAPPING
done

#
# Configure SIM by both Cola telegrams and ros services
#

for ((n=0;n<1;n++)) ; do

  # Cola-ASCII States Telegrams
  send_cola_telegram "{cola_ascii_request: 'sMN IsSystemReady', wait_response_timeout: 1}"      "sAN IsSystemReady 1"      # 0:true, 1:false # Check if the system is ready
  send_cola_telegram "{cola_ascii_request: 'sRN LocState', wait_response_timeout: 1}"           "sRA LocState 2"           # 0:BOOTING, 1:IDLE, 2:LOCALIZING, 3:DEMO_MAPPING # Read localization state
  send_cola_telegram "{cola_ascii_request: 'sMN LocStop', wait_response_timeout: 1}"            "sAN LocStop 1"            # 0:failed, 1:success # Stop localization or demo mapping
  # send_cola_telegram "{cola_ascii_request: 'sMN LocStopAndSave', wait_response_timeout: 1}"   "sAN LocStopAndSave 1"     # Deprecated 0:failed, 1:success # Stop localization, save settings
  send_cola_telegram "{cola_ascii_request: 'sMN LocStartLocalizing', wait_response_timeout: 1}" "sAN LocStartLocalizing 1" # 0:failed, 1:success # Start localization
  # Cola-ASCII Result Output Configuration Telegrams
  send_cola_telegram "{cola_ascii_request: 'sMN LocSetResultPort +2201', wait_response_timeout: 1}"     "sAN LocSetResultPort 1 1"       # 0:failed, 1:success, <port>: uint16 (default: 2201) # Set TCP-port for result output
  send_cola_telegram "{cola_ascii_request: 'sMN LocSetResultMode 0', wait_response_timeout: 1}"         "sAN LocSetResultMode 1"         # 0:failed, 1:success, <mode>: uint8 (0:stream, 1:poll, default: stream) #  Set mode of result output (stream or poll)
  send_cola_telegram "{cola_ascii_request: 'sMN LocSetResultPoseEnabled 1', wait_response_timeout: 1}"  "sAN LocSetResultPoseEnabled 1"  # 0:failed, 1:success, <enabled>: uint8 (0: disabled, 1: enabled, default: enabled) # Disable/enable result output
  send_cola_telegram "{cola_ascii_request: 'sMN LocSetResultEndianness 0', wait_response_timeout: 1}"   "sAN LocSetResultEndianness 1"   # 0:failed, 1:success, <endianness>: uint8 (0: big endian, 1: little endian, default: big endian) #  Set endianness of result output
  send_cola_telegram "{cola_ascii_request: 'sMN LocSetResultPoseInterval 1', wait_response_timeout: 1}" "sAN LocSetResultPoseInterval 1" # 0:failed, 1:success, <interval>: uint8 (0-255, interval in number of scans, 1: result with each processed scan, default: 1) #  Set interval of result output
  send_cola_telegram "{cola_ascii_request: 'sMN LocRequestResultData', wait_response_timeout: 1}"       "sAN LocRequestResultData 1"     # 0:failed, 1:success # If in poll mode, trigger sending the localization result of the next processed scan via TCP interface.
  # Cola-ASCII SetPose Telegrams
  send_cola_telegram "{cola_ascii_request: 'sMN LocSetPose +10300 -5200 +30000 +1000', wait_response_timeout: 1}" "sAN LocSetPose 1"   # 0:failed, 1:success, <posex>: int32 (x coordinate in mm), <posey>: int32 (y coordinate in mm), <yaw>: int32 (yaw angle in millidegree, -180000 to +180000), <uncertainty>: uint16 (translation uncertainty in mm) # Initialize vehicle pose
  # Cola-ASCII Result Output Settings (queries, ros service SickLocColaTelegram only)
  send_cola_telegram "{cola_ascii_request: 'sRN LocResultState', wait_response_timeout: 1}"      "sRA LocResultState 1"                # 0:disabled, 1:enabled, MSB: error flag # Read state of the result output
  send_cola_telegram "{cola_ascii_request: 'sRN LocResultMode', wait_response_timeout: 1}"       "sRA LocResultMode 0"                 # 0:stream, 1: poll # Read result mode
  send_cola_telegram "{cola_ascii_request: 'sRN LocResultEndianness', wait_response_timeout: 1}" "sRA LocResultEndianness 0"           # 0: big endian, 1: little endian # Read endianness of result output
  # Cola-ASCII Timestamp Telegrams
  send_cola_telegram "{cola_ascii_request: 'sMN LocRequestTimestamp', wait_response_timeout: 1}" "sAN LocRequestTimestamp"             # "sAN LocRequestTimestamp <hexvalue>" with 4 byte timestamp (ticks in milliseconds) # Query timestamp, see "Time synchronization"
  
  # Cola-ASCII Start/stop localization and query settings
  send_cola_telegram "{cola_ascii_request: 'sMN LocStop', wait_response_timeout: 1}"                   "sAN LocStop 1"                 # 0:failed, 1:success # Stop localization or demo mapping
  send_cola_telegram "{cola_ascii_request: 'sRN LocState', wait_response_timeout: 1}"                  "sRA LocState 1"                # 0:BOOTING, 1:IDLE, 2:LOCALIZING, 3:DEMO_MAPPING # Read localization state
  send_cola_telegram "{cola_ascii_request: 'sMN LocStartLocalizing', wait_response_timeout: 1}"        "sAN LocStartLocalizing 1"      # 0:failed, 1:success # Start localization
  send_cola_telegram "{cola_ascii_request: 'sRN LocState', wait_response_timeout: 1}"                  "sRA LocState 2"                # 0:BOOTING, 1:IDLE, 2:LOCALIZING, 3:DEMO_MAPPING # Read localization state
  # Cola-ASCII Set tcp port for result telegram and query settings
  send_cola_telegram "{cola_ascii_request: 'sMN LocSetResultPort +2345', wait_response_timeout: 1}"    "sAN LocSetResultPort 1 1"      # 0:failed, 1:success, <port>: uint16 (default: 2201) # Set TCP-port for result output
  send_cola_telegram "{cola_ascii_request: 'sMN LocSetResultPort +2201', wait_response_timeout: 1}"    "sAN LocSetResultPort 1 1"      # 0:failed, 1:success, <port>: uint16 (default: 2201) # Set TCP-port for result output
  # Cola-ASCII Set result mode and query settings
  send_cola_telegram "{cola_ascii_request: 'sMN LocSetResultMode 1', wait_response_timeout: 1}"        "sAN LocSetResultMode 1"        # 0:failed, 1:success, <mode>: uint8 (0:stream, 1:poll, default: stream) #  Set mode of result output (stream or poll)
  send_cola_telegram "{cola_ascii_request: 'sRN LocResultMode', wait_response_timeout: 1}"             "sRA LocResultMode 1"           # 0:stream, 1: poll # Read result mode
  send_cola_telegram "{cola_ascii_request: 'sMN LocSetResultMode 0', wait_response_timeout: 1}"        "sAN LocSetResultMode 1"        # 0:failed, 1:success, <mode>: uint8 (0:stream, 1:poll, default: stream) #  Set mode of result output (stream or poll)
  send_cola_telegram "{cola_ascii_request: 'sRN LocResultMode', wait_response_timeout: 1}"             "sRA LocResultMode 0"           # 0:stream, 1: poll # Read result mode
  # Cola-ASCII Set result pose enabled/disabled and query settings
  send_cola_telegram "{cola_ascii_request: 'sMN LocSetResultPoseEnabled 0', wait_response_timeout: 1}" "sAN LocSetResultPoseEnabled 1" # 0:failed, 1:success, <enabled>: uint8 (0: disabled, 1: enabled, default: enabled) # Disable/enable result output
  send_cola_telegram "{cola_ascii_request: 'sRN LocResultState', wait_response_timeout: 1}"            "sRA LocResultState 0"          # 0:disabled, 1:enabled, MSB: error flag # Read state of the result output
  send_cola_telegram "{cola_ascii_request: 'sMN LocSetResultPoseEnabled 1', wait_response_timeout: 1}" "sAN LocSetResultPoseEnabled 1" # 0:failed, 1:success, <enabled>: uint8 (0: disabled, 1: enabled, default: enabled) # Disable/enable result output
  send_cola_telegram "{cola_ascii_request: 'sRN LocResultState', wait_response_timeout: 1}"            "sRA LocResultState 1"          # 0:disabled, 1:enabled, MSB: error flag # Read state of the result output
  # Cola-ASCII Set result endianness and query settings
  send_cola_telegram "{cola_ascii_request: 'sMN LocSetResultEndianness 1', wait_response_timeout: 1}"  "sAN LocSetResultEndianness 1"  # 0:failed, 1:success, <endianness>: uint8 (0: big endian, 1: little endian, default: big endian) #  Set endianness of result output
  send_cola_telegram "{cola_ascii_request: 'sRN LocResultEndianness', wait_response_timeout: 1}"       "sRA LocResultEndianness 1"     # 0:big endian, 1:little endian # Read endianness of result output
  send_cola_telegram "{cola_ascii_request: 'sMN LocSetResultEndianness 0', wait_response_timeout: 1}"  "sAN LocSetResultEndianness 1"  # 0:failed, 1:success, <endianness>: uint8 (0: big endian, 1: little endian, default: big endian) #  Set endianness of result output
  send_cola_telegram "{cola_ascii_request: 'sRN LocResultEndianness', wait_response_timeout: 1}"       "sRA LocResultEndianness 0"     # 0:big endian, 1:little endian # Read endianness of result output

  # ROS services for States Telegrams
  call_service SickLocIsSystemReady "{}"    "success: True"                 # Check if the system is ready
  call_service SickLocState "{}"            "state: 2 success: True"        # Read localization state: 0:BOOTING, 1:IDLE, 2:LOCALIZING, 3:DEMO_MAPPING
  call_service SickLocStop "{}"             "success: True"                 # Stop localization or demo mapping
  # call_service SickLocStopAndSave "{}"    "success: True"                 # Deprecated, Stop localization, save settings
  call_service SickLocStartLocalizing  "{}" "success: True"                 # Start localization
  # ROS services for Result Output Configuration Telegrams
  call_service SickLocSetResultPort "{port: 2201}"          "success: True" # Set TCP-port for result output
  call_service SickLocSetResultMode "{mode: 0}"             "success: True" # Set mode of result output (stream or poll)
  call_service SickLocSetResultPoseEnabled "{enabled: 1}"   "success: True" # Disable/enable result output
  call_service SickLocSetResultEndianness "{endianness: 0}" "success: True" # Set endianness of result output
  call_service SickLocSetResultPoseInterval "{interval: 1}" "success: True" # Set interval of result output
  call_service SickLocRequestResultData "{}"                "success: True" # If in poll mode, trigger sending the localization result of the next processed scan via TCP interface.
  # ROS services for SetPose Telegrams
  call_service SickLocSetPose "{posex: 10300, posey: -5200, yaw: 30000, uncertainty: 1000}" "success: True" # Initialize vehicle pose
  # ROS services for Timestamp Telegrams
  call_service SickLocRequestTimestamp "{}" "timestamp_lidar_ms" # "timestamp_lidar_ms: <uint32> mean_time_vehicle_ms: <uint64> delta_time_ms: <uint64> ..." # Query timestamp, see "Time synchronization"

  # Change settings with ROS services for States Telegrams
  call_service SickLocIsSystemReady "{}"    "success: True"                 # Check if the system is ready
  call_service SickLocState "{}"            "state: 2 success: True"        # Read localization state: 0:BOOTING, 1:IDLE, 2:LOCALIZING, 3:DEMO_MAPPING
  call_service SickLocStop "{}"             "success: True"                 # Stop localization or demo mapping
  call_service SickLocState "{}"            "state: 1 success: True"        # Read localization state: 0:BOOTING, 1:IDLE, 2:LOCALIZING, 3:DEMO_MAPPING
  call_service SickLocStartLocalizing  "{}" "success: True"                 # Start localization
  call_service SickLocState "{}"            "state: 2 success: True"        # Read localization state: 0:BOOTING, 1:IDLE, 2:LOCALIZING, 3:DEMO_MAPPING
  # Change settings with ROS services for Result Output Configuration Telegrams
  call_service SickLocSetResultPort "{port: 2345}" "success: True"                                                           # Set TCP-port for result output
  call_service SickLocSetResultPort "{port: 2201}" "success: True"                                                           # Set TCP-port for result output
  call_service SickLocSetResultMode "{mode: 1}" "success: True"                                                              # Set mode of result output (stream or poll)
  send_cola_telegram "{cola_ascii_request: 'sRN LocResultMode', wait_response_timeout: 1}" "sRA LocResultMode 1"             # 0:stream, 1:poll # Read result mode
  call_service SickLocSetResultMode "{mode: 0}" "success: True"                                                              # Set mode of result output (stream or poll)
  send_cola_telegram "{cola_ascii_request: 'sRN LocResultMode', wait_response_timeout: 1}" "sRA LocResultMode 0"             # 0:stream, 1:poll # Read result mode
  call_service SickLocSetResultPoseEnabled "{enabled: 0}" "success: True"                                                    # Disable/enable result output
  send_cola_telegram "{cola_ascii_request: 'sRN LocResultState', wait_response_timeout: 1}" "sRA LocResultState 0"           # 0:disabled, 1:enabled, MSB: error flag # Read state of the result output
  call_service SickLocSetResultPoseEnabled "{enabled: 1}" "success: True"                                                    # Disable/enable result output
  send_cola_telegram "{cola_ascii_request: 'sRN LocResultState', wait_response_timeout: 1}" "sRA LocResultState 1"           # 0:disabled, 1:enabled, MSB: error flag # Read state of the result output
  call_service SickLocSetResultEndianness "{endianness: 1}" "success: True"                                                  # Set endianness of result output
  send_cola_telegram "{cola_ascii_request: 'sRN LocResultEndianness', wait_response_timeout: 1}" "sRA LocResultEndianness 1" # 0:big endian, 1:little endian # Read endianness of result output
  call_service SickLocSetResultEndianness "{endianness: 0}" "success: True"                                                  # Set endianness of result output
  send_cola_telegram "{cola_ascii_request: 'sRN LocResultEndianness', wait_response_timeout: 1}" "sRA LocResultEndianness 0" # 0:big endian, 1:little endian # Read endianness of result output

done
echo -e "send_cola_examples.bash finished.\n"
echo -e "Services and cola telegram verification summary: $testcase_counter testcases, $testcase_error_counter failures.\n"

# 
# Debug stuff
#
# rossrv show SickLocColaTelegramSrv
# rosservice info SickLocColaTelegram
# rossrv show SickLocRequestTimestampSrv
# rosservice info SickLocRequestTimestamp
# for ((n=0;n<=10;n++)) ; do 
#   sleep 1 ; rosservice call SickLocColaTelegram "{cola_ascii_request: 'sRN LocMapState', wait_response_timeout: 1}"          # expected reponse: cola_ascii_response="sRA LocMapState 1"
#   sleep 1 ; rosservice call SickLocColaTelegram "{cola_ascii_request: 'sMN LocRequestTimestamp', wait_response_timeout: 1}"  # expected reponse: cola_ascii_response="sAN LocRequestTimestamp <uint32>"
#   sleep 1 ; rosservice call SickLocRequestTimestamp "{}" # expected reponse: "timestamp_lidar_ms: <uint32>, mean_time_vehicle_ms: <uint64>, delta_time_ms: <uint64>, ..."
#   sleep 1 ; rosservice call SickLocTimeSync "{timestamp_lidar_ms: 123456}" # expected reponse: "vehicle_time_valid: True vehicle_time_sec: 1573119167 vehicle_time_nsec: 380565047"
# done
# sleep 1 ; 
# if [ -d ./src/sick_lidar_localization ]         ; then ./src/sick_lidar_localization/test/ros1_scripts/killall.bash         ; fi
# if [ -d ./src/sick_lidar_localization_pretest ] ; then ./src/sick_lidar_localization_pretest/test/ros1_scripts/killall.bash ; fi
# sleep 1 ; killall roslaunch ; sleep 20
# ps -elf

