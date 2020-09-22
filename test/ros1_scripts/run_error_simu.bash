#!/bin/bash

#
# Run error simulation of sick_lidar_localization driver (offline-test error handling) against local test server simulating error testcases.
#

#
# Function wait_telegram_message() waits until at least one message on topic "/sick_lidar_localization/driver/result_telegrams" has been published
#
function wait_telegram_message() {
  local telegram_messages_topic="/sick_lidar_localization/driver/result_telegrams"
  local telegram_message_length=0
  while [ $telegram_message_length -le 1060 ] ; do
    echo -e "sick_lidar_localization error simulation: waiting for telegram messages from sim_loc_driver on topic $telegram_messages_topic ..." 2>&1 | tee -a ~/.ros/log/error_simu/error_simu.log
    telegram_message_length=`rostopic echo -n 10 $telegram_messages_topic | wc -c`
  done
  echo -e "sick_lidar_localization error simulation: $telegram_message_length byte telegram messages received on topic $telegram_messages_topic" 2>&1 | tee -a ~/.ros/log/error_simu/error_simu.log
  rostopic echo -n 1 -p $telegram_messages_topic 2>&1 | tee -a ~/.ros/log/error_simu/error_simu.log
}

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
if [ ! -d ~/.ros/log/error_simu ] ; then mkdir -p ~/.ros/log/error_simu ; fi

#
# Start sick_lidar_localization driver, log diagnostic messages
#

echo -e "sick_lidar_localization error simulation: starting sick_lidar_localization sim_loc_driver.launch ..." 2>&1 | tee -a ~/.ros/log/error_simu/error_simu.log
roslaunch sick_lidar_localization sim_loc_driver.launch localization_controller_ip_address:=127.0.0.1 sim_loc_driver_check_cfg:=message_check_error_simu.yaml 2>&1 | tee -a ~/.ros/log/error_simu/sim_loc_driver_error_simu.log &
echo -e "sick_lidar_localization error simulation: sick_lidar_localization sim_loc_driver.launch started." 2>&1 | tee -a ~/.ros/log/error_simu/error_simu.log
sleep 10

#
# Log diagnostic messages
#

rostopic echo -p "/sick_lidar_localization/driver/diagnostic" | tee -a ~/.ros/log/error_simu/sim_loc_driver_diagnostic_messages.log &

#
# Start test server, run some time, kill and restart. Check reconnect of ros driver after connection lost
#

if true ; then
  echo -e "sick_lidar_localization error simulation: Starting test server ..." 2>&1 | tee -a ~/.ros/log/error_simu/error_simu.log
  roslaunch sick_lidar_localization sim_loc_test_server.launch & # 2>&1 | unbuffer -p tee -a ~/.ros/log/error_simu/sim_loc_test_server_error_simu.log &
  sleep 10
  for testcnt in {1..3} ; do
    echo -e "sick_lidar_localization error simulation: running $testcnt. testcase ..." 2>&1 | tee -a ~/.ros/log/error_simu/error_simu.log
    echo -e "sick_lidar_localization error simulation: killing test server ..." 2>&1 | tee -a ~/.ros/log/error_simu/error_simu.log
    if [ $testcnt -eq 1 ] ; then
      rosnode kill /sim_loc_test_server ; sleep 10 ; killall -9 sim_loc_test_server
    elif [ $testcnt -eq 2 ] ; then
      killall -9 sim_loc_test_server
    else
      rosnode kill /sim_loc_test_server ; sleep 1 ; killall -9 sim_loc_test_server
    fi
    sleep 10
    echo -e "sick_lidar_localization error simulation: restarting test server ..." 2>&1 | tee -a ~/.ros/log/error_simu/error_simu.log
    roslaunch sick_lidar_localization sim_loc_test_server.launch & # 2>&1 | unbuffer -p tee -a ~/.ros/log/error_simu/sim_loc_test_server_error_simu.log &
    sleep 10
    wait_telegram_message
    echo -e "sick_lidar_localization error simulation: finished $testcnt. testcase.\n" 2>&1 | tee -a ~/.ros/log/error_simu/error_simu.log
  done
  echo -e "sick_lidar_localization error simulation: Tests 1 to $testcnt finished, stopping test server ..." 2>&1 | tee -a ~/.ros/log/error_simu/error_simu.log
  rosnode kill /sim_loc_test_server ; sleep 1 ; killall -9 sim_loc_test_server ; sleep 15
  echo -e "sick_lidar_localization error simulation summary: finished $testcnt testcases, reconnect after connection lost okay.\n" 2>&1 | tee -a ~/.ros/log/error_simu/error_simu.log
fi

#
# Run test server in error simulation mode (disconnect and reconnect, send no or invalid telegrams). 
# sick_lidar_localization driver handles errors with diagnostic messages and reconnects if required.
#

if true ; then
  roslaunch sick_lidar_localization sim_loc_test_server.launch error_simulation:=true 2>&1 | tee -a ~/.ros/log/error_simu/error_simu.log &
  sleep 180
  rosnode kill /sim_loc_test_server ; sleep 1 ; killall -9 sim_loc_test_server
fi

#
# Cleanup and exit
#

if [ -d ./src/sick_lidar_localization ]         ; then ./src/sick_lidar_localization/test/ros1_scripts/killall.bash         ; fi
if [ -d ./src/sick_lidar_localization_pretest ] ; then ./src/sick_lidar_localization_pretest/test/ros1_scripts/killall.bash ; fi
killall roslaunch
grep "WARN" ~/.ros/log/error_simu/*.log ; grep "ERR" ~/.ros/log/error_simu/*.log
sleep 15
echo -e "\nrun_error_simu.bash: sick_lidar_localization error simulation finished.\n"
cat ~/.ros/log/error_simu/sim_loc_driver_diagnostic_messages.log
cat ~/.ros/log/error_simu/error_simu.log
echo -e "\nrun_error_simu.bash: sick_lidar_localization error simulation summary:\n"
grep "summary" ~/.ros/log/error_simu/*.log
popd
rm -rf ./log/error_simu
mkdir  ./log/error_simu
cp -rf ~/.ros/log/error_simu/*.log ./log/error_simu
echo -e "\nsick_lidar_localization summary:"
grep -R "summary" ./log/*

