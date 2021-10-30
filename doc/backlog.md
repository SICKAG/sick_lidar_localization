# sick_lidar_localization2 backlog

Project abbreviation: pa0410_sick_lidar_localization2

Brief description: Device driver for ROS, ROS2 and as a generic library for RESTAPI-based SICK Localization Server

## Specification, requirements

These specifications are given from the client's side:
- C++14 (specified by ROS)
- Driver core as standalone library
- No boost dependencies
- Minimize library dependencies

## Work packages sick_lidar_localization2

* Implement Simulator etc.
* curl library might be used (f.e. for REST-API)
* Work packages 13.7.2021: [arbeitspakete_20210713.png](specifications/arbeitspakete_20210713.png)
* Siehe Aufgabenbeschreibung im Backlog: https://github.com/michael1309/sick_lidar_localization2/blob/main/doc/backlog.md
* Input messages: The data that is send to the LocalizationController system.
    * Odometry
    * Encoder Measurement
    * Code measurement
    * Line measurement
* Generic: 
     * REST-API:
         * Test mit Curl-Commandline einschl. Beispiel in der Dokumentation.
         * genServiceCall implementiert es in gleicher Art und Weise. �ber eine Dump-Option im genServiceCall erh�lt man die Ausgabe als Curl-Call. Man sollte dann f�r den Test eine M�glichkeit schaffe, dass man mit dem Dump direkte ein Script-Datei o.�. erzeugt, mit der man dann den Curl-Call testen kann.
         * F�r die Dokumentation kann man dann auch einfach die Dump-Option ausw�hlen, die KEINE direkt REST-Api-Aufrufe startet, um somit alles f�r libcurl vorzubereiten.
         * Bsp. -d=1 : Dump auf Bildschirm und ausf�hren des Calls intern.                          -d=2 : Dump auf Bildschirm und KEINE Ausf�hrung des Calls.
     * UDP-Empfang generisch: Dump in Json-Datei als Ergebnis der UDP-Nachricht. Registrierung �ber Callback und Beispiel als Json-Output.
     * UDP-Versand generisch: Via Fkt.-Aufruf versenden von Json-Daten und Umwandlung in Json-Calls.

## REST-API

* [cola_interface.html](specifications/2021-07-13-cola_interface.html)
* [localizationcontroller_UDP_input_output.pdf](specifications/2021-07-13-LLS2_UDP_input_output.pdf)
* REST commands priority 1: LocStartLocalizing, LocStop, LocSetMappingActive, LocGetSystemState, LocInitializeAtPose, LocResumeAtPose, LocRequestTimestamp, IsSystemReady, GetErrorLevel
* Rest commands priority 2: LocSetMap, LocSwitchMap, LocLoadMapToCache, LocClearMapCache, LocGetMap, LocSetLinesForSupportActive, LocSetKinematicVehicleModelActive, LocSetOdometryActive, LocAutoStartSavePose, LocSetRingBufferRecordingActive, LocSaveRingBufferRecording, LocSetRecordingActive

Umzusetzende REST-API-Befehle (Test 28.7.2021, sick_lidar_localization2_pretest ROS-1 gegen SIM, Firmware 2.0.0.8C):

LocGetSystemState: OK
```
./gen_service_call LocGetSystemState POST "{}" --verbose=1 --dump=1
gen_service_call request: LocGetSystemState, response: {"header":{"status":0,"message":"Ok"},"data":{"systemState":"Localization"}}
rosservice call /sick_lidar_localization/LocGetSystemState "{}"
systemstate: "Localization" success: True
```

LocInitializeAtPose: OK
```
# Note: The pose should be the correct pose to enable localization. This example uses x = -8.206 m, y = 4.580 m and yaw = 85.2 deg shown in the SOPAS dashboard. 
./gen_service_call LocInitializeAtPose POST "{\"data\":{\"pose\":{\"x\":-8206,\"y\":4580,\"yaw\":85200},\"searchRadius\":1000}}" --verbose=1 --dump=1
gen_service_call request: LocInitializeAtPose, response: {"header":{"status":0,"message":"Ok"},"data":{"success":true}}
rosservice call /sick_lidar_localization/LocInitializeAtPose "{\"x\":-8206,\"y\":4580,\"yaw\":85200,\"searchradius\":1000}"
success: True
```

LocResumeAtPose: OK
```
./gen_service_call LocResumeAtPose POST "{\"data\":{\"pose\":{\"x\":1000,\"y\":1000,\"yaw\":1000}}}" --verbose=1 --dump=1
gen_service_call request: LocResumeAtPose, response: {"header":{"status":0,"message":"Ok"},"data":{"success":true}}
rosservice call /sick_lidar_localization/LocResumeAtPose "{\"x\":-8206,\"y\":4580,\"yaw\":85200}"
success: True
```

LocRequestTimestamp: OK
```
./gen_service_call LocRequestTimestamp POST "{}" --verbose=1 --dump=1
gen_service_call request: LocRequestTimestamp, response: {"header":{"status":0,"message":"Ok"},"data":{"timestamp":4673341}}
rosservice call /sick_lidar_localization/LocRequestTimestamp "{}"
timestamp_lidar_ms: 7628343
```

IsSystemReady: OK
```
./gen_service_call IsSystemReady POST "{}" --verbose=1 --dump=1
gen_service_call request: IsSystemReady, response: {"header":{"status":0,"message":"Ok"},"data":{"success":true}}
rosservice call /sick_lidar_localization/IsSystemReady "{}"
success: True
```

GetErrorLevel: OK
```
./gen_service_call GetErrorLevel POST "{}" --verbose=1 --dump=1
gen_service_call request: GetErrorLevel, response: {"header":{"status":0,"message":"Ok"},"data":{"level":0,"description":""}}
rosservice call /sick_lidar_localization/GetErrorLevel "{}"
level: 0 description: '' success: True
```

LocGetMap, LocSetMap: OK
```
./gen_service_call LocGetMap POST "{}"
gen_service_call request: LocGetMap, response: {"header":{"status":0,"message":"Ok"},"data":{"mapPath":"2020-12-21_CNord_40m_v0.vmap"}}
./gen_service_call LocSetMap POST "{\"data\":{\"mapPath\":\"2020-12-21_CNord_40m_v0.vmap\"}}" --verbose=1 --dump=1
gen_service_call request: LocSetMap, response: {"header":{"status":0,"message":"Ok"},"data":{"success":true}}
rosservice call /sick_lidar_localization/LocGetMap "{}"
mappath: "2020-12-21_CNord_40m_v0.vmap" success: True
rosservice call /sick_lidar_localization/LocSetMap "{\"mappath\":\"2020-12-21_CNord_40m_v0.vmap\"}"
success: True
```

LocSaveRingBufferRecording: OK
```
# Note: RingBuffer must be activated for command LocSaveRingBufferRecording, activate using SOPAS dashboard if not yet done
./gen_service_call LocSaveRingBufferRecording POST "{\"data\":{\"reason\":\"2021-07-28 test\"}}" --verbose=1 --dump=1
gen_service_call request: LocSaveRingBufferRecording, response: {"header":{"status":0,"message":"Ok"},"data":{"success":true}}
rosservice call /sick_lidar_localization/LocSaveRingBufferRecording "{\"reason\":\"2021-07-28 test\"}"
success: True
```

Weitere REST-API-Befehle (Prio 1 und 2, Test 28.7.2021, sick_lidar_localization2_pretest ROS-1 gegen SIM, Firmware 2.0.0.8C):

LocClearMapCache: OK
```
rosservice call /sick_lidar_localization/LocClearMapCache "{}"
success: True
```

LocLoadMapToCache: OK
```
rosservice call /sick_lidar_localization/LocLoadMapToCache "{\"mappath\":\"2020-12-21_CNord_40m_v0.vmap\"}"
success: True
```

LocSetKinematicVehicleModelActive: OK
```
rosservice call /sick_lidar_localization/LocSetKinematicVehicleModelActive "{\"active\":1}"
success: True
```

LocSetLinesForSupportActive: OK
```
rosservice call /sick_lidar_localization/LocSetLinesForSupportActive "{\"active\":1}"
success: True
```

LocSetMappingActive: OK
```
rosservice call /sick_lidar_localization/LocSetMappingActive "{\"active\":1}"
success: True
```

LocSetOdometryActive OK
rosservice call /sick_lidar_localization/LocSetOdometryActive "{\"active\":1}"

LocSetRecordingActive: OK
```
rosservice call /sick_lidar_localization/LocSetRecordingActive "{\"active\":1}"
success: True
```

LocSetRingBufferRecordingActive: OK
```
rosservice call /sick_lidar_localization/LocSetRingBufferRecordingActive "{\"active\":1}"
success: True
```

LocStartLocalizing: OK
```
rosservice call /sick_lidar_localization/LocStartLocalizing "{}"
success: True
```

LocStop: OK
```
rosservice call /sick_lidar_localization/LocStop "{}"
success: True
```

LocSwitchMap: OK
```
rosservice call /sick_lidar_localization/LocSwitchMap "{\"submapname\":\"2020-12-21_CNord_40m_v0.vmap\"}"
success: True
```

## ROS-1 usage example

```
printf "\033c"
rosclean purge
source ./install/setup.bash
# Start sick_lidar_localization2, replace 192.168.0.100 with your local ip address
roslaunch sick_lidar_localization2 sick_lidar_localization2.launch hostname:=192.168.0.1 udp_ip_sim_input:=192.168.0.1 udp_ip_sim_output:=192.168.0.100 # verbose:=1 # Use option verbose:=1 for informational messages
rosservice call /sick_lidar_localization/IsSystemReady "{}"        # expected response: success: True
rosservice call /sick_lidar_localization/LocGetSystemState "{}"    # expected response: systemstate: "Localization" success: True
roslaunch sick_lidar_localization2 pointcloud_converter.launch     # convert poses from LocalizationController output to pointcloud messages
rosrun rviz rviz -d ./src/sick_lidar_localization2/test/config/rviz_sick_lidar_localization_pointcloud.rviz # show poses converted from LocalizationController output

rosservice call /sick_lidar_localization/LocSetOdometryActive "{\"active\":1}"
rostopic pub --once /localizationcontroller/in/odometry_message_0105 sick_lidar_localization2/OdometryMessage0105 '{telegram_count: 1000002, timestamp: 123456780, x_position: -1234, y_position: -1234, heading: 1234}'
rostopic pub --once /localizationcontroller/in/odometry_message_0104 sick_lidar_localization2/OdometryMessage0104 '{telegram_count: 1000001, timestamp: 123456789, x_velocity: -1234, y_velocity: -1234, angular_velocity: 1234}'
```

## ROS-2 usage example

```
printf "\033c"
source ./install/setup.bash
# Start sick_lidar_localization2, replace 192.168.0.100 with your local ip address
ros2 run sick_lidar_localization2 sick_lidar_localization ./src/sick_lidar_localization2/launch/sick_lidar_localization2.launch hostname:=192.168.0.1 udp_ip_sim_input:=192.168.0.1 udp_ip_sim_output:=192.168.0.100 # verbose:=1 # Use option verbose:=1 for informational messages

ros2 service call /LocSetOdometryActive sick_lidar_localization2/srv/SickLocSetOdometryActiveSrv "{\"active\":1}"
ros2 topic pub --once /localizationcontroller/in/odometry_message_0105 sick_lidar_localization2/OdometryMessage0105 '{telegram_count: 1000002, timestamp: 123456780, x_position: -1234, y_position: -1234, heading: 1234}'
ros2 topic pub --once /localizationcontroller/in/odometry_message_0104 sick_lidar_localization2/OdometryMessage0104 '{telegram_count: 1000001, timestamp: 123456789, x_velocity: -1234, y_velocity: -1234, angular_velocity: 1234}'
```

## Verification SIM input message examples

SIM input message examples:
```
rostopic pub --once /localizationcontroller/in/odometry_message_0105            sick_lidar_localization2/OdometryMessage0105           '{telegram_count: 1000002, timestamp: 123456780, x_position: -1234, y_position: -1234, heading: 1234}'
rostopic pub --once /localizationcontroller/in/odometry_message_0104            sick_lidar_localization2/OdometryMessage0104           '{telegram_count: 1000001, timestamp: 123456789, x_velocity: -1234, y_velocity: -1234, angular_velocity: 1234}'
rostopic pub --once /localizationcontroller/in/encoder_measurement_message_0202 sick_lidar_localization2/EncoderMeasurementMessage0202 '{telegram_count: 1000003, timestamp: 123456781, encoder_value: 123456789}'
rostopic pub --once /localizationcontroller/in/code_measurement_message_0303    sick_lidar_localization2/CodeMeasurementMessage0303    '{telegram_count: 1000004, timestamp: 123456782, code: 1234}'
rostopic pub --once /localizationcontroller/in/line_measurement_message_0403    sick_lidar_localization2/LineMeasurementMessage0403    '{telegram_count: 1000005, timestamp: 123456783, num_lanes: 1, lanes: [1234]}'
rostopic pub --once /localizationcontroller/in/line_measurement_message_0404    sick_lidar_localization2/LineMeasurementMessage0404    '{telegram_count: 1000006, timestamp: 123456784, lcp1: 12, lcp2: 34, lcp3: 56, cnt_lpc: 78}' 
```
UDP messages recorded using wireshark and verified:
```
python ../rest_server/python/sim_udp_receiver.py --udp_port=5009
python ../rest_server/python/sim_pcapng_player.py --pcap_filename ../data/wireshark/20210804_UDP_Port_5009_RecvData.pcapng --udp_port=5009
OdometryMessage0105:{telegram_count:1000002, timestamp:123456780, x_position:-1234, y_position:-1234, heading:1234}
OdometryMessage0104:{telegram_count:1000001, timestamp:123456789, x_velocity:-1234, y_velocity:-1234, angular_velocity:1234}
EncoderMeasurementMessage0202:{telegram_count:1000003, timestamp:123456781, encoder_value:123456789}
CodeMeasurementMessage0303:{telegram_count:1000004, timestamp:123456782, code:1234}
LineMeasurementMessage0403:{telegram_count:1000005, timestamp:123456783, num_lanes:1, lanes:[1234]}
LineMeasurementMessage0404:{telegram_count:1000006, timestamp:123456784, lcp1:12, lcp2:34, lcp3:56, cnt_lpc:78}
```

## Time synchronization

* Time sync, tics, Software-PLL:
    * LocRequestTimestamp returns uint32 tics in milliseconds
    * UDP-output messages: uint64 timestamp in microseconds
    ```
    Beispiellog SIM:
    [1628057611.391312618]: LocRequestTimestamp: timestamp:8481058
    [1628057611.412485756]: OdometryPayload0105: timestamp:8481039054
    [1628057612.401770896]: LocRequestTimestamp: timestamp:8482067
    [1628057612.413992076]: OdometryPayload0105: timestamp:8482038947
    => LocRequestTimestamp: 32 bit timestamp in Millisekunden, UDP-Nachrichten: 64 bit timestamp in Mikrosekunden
    ```

## Test Release 1.1.0 with SIM firmware 2.0.0.14R:

Run Linux native OK:
```
./sick_lidar_localization ../launch/sick_lidar_localization2.launch verbose:=1
```

Run Linux ROS-1 OK:
```
source ./install/setup.bash
roslaunch sick_lidar_localization2 sick_lidar_localization2.launch verbose:=1 &
roslaunch sick_lidar_localization2 pointcloud_converter.launch &
rosrun rviz rviz &
```

Run Linux ROS-2 OK:
```
source ./install/setup.bash
ros2 run sick_lidar_localization2 sick_lidar_localization ./src/sick_lidar_localization2_pretest/launch/sick_lidar_localization2.launch & 
ros2 run sick_lidar_localization2 pointcloud_converter ./src/sick_lidar_localization2_pretest/launch/pointcloud_converter.launch &
rviz2 &  
```

Test REST-API Linux native OK:
```
./build/gen_service_call SickLocIsSystemReady POST "{}" -d=2
./build/gen_service_call SickLocGetMap POST "{}" -d=2
./build/gen_service_call SickLocGetSystemState POST "{}" -d=2
./build/gen_service_call SickLocSetMappingActive POST "{\"data\":{\"active\":true}}" -d=2
./build/gen_service_call SickLocSetOdometryActive POST "{\"data\":{\"active\":true}}" -d=2
./build/gen_service_call SickLocStop POST "{}" -d=2
./build/gen_service_call SickLocStartLocalizing POST "{}" -d=2
./build/gen_service_call SickLocInitializeAtPose POST "{\"data\":{\"pose\":{\"x\":-8206,\"y\":4580,\"yaw\":85200},\"searchRadius\":1000}}" --verbose=1 --dump=1
./build/gen_service_call SickLocResumeAtPose POST "{\"data\":{\"pose\":{\"x\":1000,\"y\":1000,\"yaw\":1000}}}" --verbose=1 --dump=1
./build/gen_service_call SickLocRequestTimestamp POST "{}" --verbose=1 --dump=1
./build/gen_service_call SickLocGetErrorLevel POST "{}" --verbose=1 --dump=1
./build/gen_service_call SickLocClearMapCache POST "{}" --verbose=1 --dump=1
./build/gen_service_call SickLocSetLinesForSupportActive POST "{\"data\":{\"active\":true}}" --verbose=1 --dump=1
./build/gen_service_call SickLocSetOdometryActive POST "{\"data\":{\"active\":true}}" --verbose=1 --dump=1
./build/gen_service_call SickLocSetRecordingActive POST "{\"data\":{\"active\":true}}" --verbose=1 --dump=1
./build/gen_service_call SickLocSetRingBufferRecordingActive POST "{\"data\":{\"active\":true}}" --verbose=1 --dump=1
./build/gen_service_call SickLocStartLocalizing POST "{}" --verbose=1 --dump=1
./build/gen_service_call SickLocStop POST "{}" --verbose=1 --dump=1
./build/gen_service_call SickLocSaveRingBufferRecording POST "{\"data\":{\"reason\":\"2021-08-16 test\"}}" --verbose=1 --dump=1
./build/gen_service_call SickLocStartLocalizing POST "{}" --verbose=1 --dump=1
./build/gen_service_call SickLocSetMappingActive POST "{\"data\":{\"active\":true}}" --verbose=1 --dump=1
./build/gen_service_call SickLocSwitchMap POST "{\"data\":{\"subMapName\":\"2020-12-21_CNord_40m_v0.vmap\"}}" --verbose=1 --dump=1
```

Test REST-API Linux native NOT OK:
```
./build/gen_service_call SickLocLoadMapToCache POST "{\"data\":{\"mapPath\":\"2020-12-21_CNord_40m_v0.vmap\"}}" --verbose=1 --dump=1
curl -i -H "Content-Type: application/json" -X POST -d "{\"data\":{\"mapPath\":\"2020-12-21_CNord_40m_v0.vmap\"}}" http://192.168.0.1/api/LocLoadMapToCache
HTTP/1.1 200 OK
Pragma: no-cache
Cache-Control: no-cache
Content-Type: application/json
Content-Length: 000000000048
{"header":{"status":2,"message":"Invalid Data"}}

./build/gen_service_call SickLocSetKinematicVehicleModelActive POST "{\"data\":{\"active\":true}}" --verbose=1 --dump=1
curl -i -H "Content-Type: application/json" -X POST -d "{\"data\":{\"active\":true}}" http://192.168.0.1/api/LocSetKinematicVehicleModelActive
HTTP/1.1 200 OK
Pragma: no-cache
Cache-Control: no-cache
Content-Type: application/json
Content-Length: 000000000045
{"header":{"status":5,"message":"Not Found"}}

./build/gen_service_call SickLocSavePermanent POST "{}" --verbose=1 --dump=1
```

Test REST-API Linux ROS-1 OK:
```
rosservice call SickLocIsSystemReady "{}"
rosservice call SickLocGetMap "{}"
rosservice call SickLocGetSystemState "{}"
rosservice call SickLocSetMappingActive "{\"active\":1}"
rosservice call SickLocSetOdometryActive "{\"active\":1}"
rosservice call SickLocStop "{}"
rosservice call SickLocStartLocalizing "{}"
rosservice call SickLocInitializeAtPose "{\"x\":-8206,\"y\":4580,\"yaw\":85200,\"searchradius\":1000}"
rosservice call SickLocResumeAtPose "{\"x\":1000,\"y\":1000,\"yaw\":1000}"
rosservice call SickLocRequestTimestamp "{}"
rosservice call SickLocGetErrorLevel "{}"
rosservice call SickLocClearMapCache "{}"
rosservice call SickLocSetLinesForSupportActive "{\"active\":1}"
rosservice call SickLocSetOdometryActive "{\"active\":1}"
rosservice call SickLocSetRecordingActive "{\"active\":1}"
rosservice call SickLocSetRingBufferRecordingActive "{\"active\":1}"
rosservice call SickLocSaveRingBufferRecording "{\"reason\":\"2021-08-16 test\"}"
rosservice call SickLocSetMappingActive "{\"active\":1}"
rosservice call SickLocSwitchMap "{\"submapname\":\"2020-12-21_CNord_40m_v0.vmap\"}"
```

Test REST-API Linux ROS-1 NOT OK:
```
rosservice call SickLocLoadMapToCache "{\"mappath\":\"2020-12-21_CNord_40m_v0.vmap\"}"
rosservice call SickLocSetKinematicVehicleModelActive "{\"active\":1}"
rosservice call SickLocSavePermanent "{}"
```

Test REST-API Linux ROS-2 OK:
```
ros2 service call SickLocIsSystemReady sick_lidar_localization2/srv/SickLocIsSystemReadySrv "{}"
ros2 service call SickLocGetMap sick_lidar_localization2/srv/SickLocGetMapSrv "{}"
ros2 service call SickLocGetSystemState sick_lidar_localization2/srv/SickLocGetSystemStateSrv "{}"
ros2 service call SickLocSetMappingActive sick_lidar_localization2/srv/SickLocSetMappingActiveSrv "{\"active\":1}"
ros2 service call SickLocSetOdometryActive sick_lidar_localization2/srv/SickLocSetOdometryActiveSrv "{\"active\":1}"
ros2 service call SickLocStop sick_lidar_localization2/srv/SickLocStopSrv "{}"
ros2 service call SickLocStartLocalizing sick_lidar_localization2/srv/SickLocStartLocalizingSrv "{}"
ros2 service call SickLocInitializeAtPose sick_lidar_localization2/srv/SickLocInitializeAtPoseSrv "{\"x\":-8206,\"y\":4580,\"yaw\":85200,\"searchradius\":1000}"
ros2 service call SickLocResumeAtPose sick_lidar_localization2/srv/SickLocResumeAtPoseSrv "{\"x\":1000,\"y\":1000,\"yaw\":1000}"
ros2 service call SickLocRequestTimestamp sick_lidar_localization2/srv/SickLocRequestTimestampSrv "{}"
ros2 service call SickGetErrorLevel sick_lidar_localization2/srv/SickLocGetErrorLevelSrv "{}"
ros2 service call SickLocClearMapCache sick_lidar_localization2/srv/SickLocClearMapCacheSrv "{}"
ros2 service call SickLocClearMapCache sick_lidar_localization2/srv/SickLocClearMapCacheSrv "{}"
ros2 service call SickLocSetOdometryActive sick_lidar_localization2/srv/SickLocSetOdometryActiveSrv "{\"active\":1}"
ros2 service call SickLocSetRecordingActive sick_lidar_localization2/srv/SickLocSetRecordingActiveSrv "{\"active\":1}"
ros2 service call SickLocSetRingBufferRecordingActive sick_lidar_localization2/srv/SickLocSetRingBufferRecordingActiveSrv "{\"active\":1}"
ros2 service call SickLocSaveRingBufferRecording sick_lidar_localization2/srv/SickLocSaveRingBufferRecordingSrv "{\"reason\":\"2021-08-16 test\"}"
ros2 service call SickLocSetMappingActive sick_lidar_localization2/srv/SickLocSetMappingActiveSrv "{\"active\":1}"
ros2 service call SickLocSwitchMap sick_lidar_localization2/srv/SickLocSwitchMapSrv "{\"submapname\":\"2020-12-21_CNord_40m_v0.vmap\"}"
```

Test REST-API Linux ROS-1 NOT OK:
```
ros2 service call SickLocLoadMapToCache sick_lidar_localization2/srv/SickLocLoadMapToCacheSrv "{\"mappath\":\"2020-12-21_CNord_40m_v0.vmap\"}"
ros2 service call SickLocSetKinematicVehicleModelActive sick_lidar_localization2/srv/SickLocSetKinematicVehicleModelActiveSrv "{\"active\":1}"
ros2 service call SickLocSavePermanent sick_lidar_localization2/srv/SickLocSavePermanentSrv "{}"
```

## Test notes 21.9.2021, ROS-1, Linux

Run OK:
```
source ./install/setup.bash 
roslaunch sick_lidar_localization2 sick_lidar_localization2.launch verbose:=1 udp_ip_sim_output:=192.168.0.100
roslaunch sick_lidar_localization2 pointcloud_converter.launch
rosrun rviz rviz -d ./src/sick_lidar_localization2/test/config/rviz_sick_lidar_localization_pointcloud.rviz
rostopic echo /localizationcontroller/out/odometry_message_0105
rostopic echo /localizationcontroller/out/localizationcontroller_result_message_0502
```

Services OK:
```
rosservice call SickLocIsSystemReady "{}"
rosservice call SickLocGetMap "{}"
rosservice call SickLocGetSystemState "{}"
rosservice call SickLocSetMappingActive "{active: 1}"
rosservice call SickLocSetOdometryActive "{active: 1}"
rosservice call SickLocStop "{}"
rosservice call SickLocStartLocalizing "{}"
rosservice call SickLocInitializeAtPose "{x: -8206, y: 4580, yaw: 85200, searchradius: 1000}"
rosservice call SickLocResumeAtPose "{x: 1000, y: 1000, yaw: 1000}"
rosservice call SickLocRequestTimestamp "{}"
rosservice call SickLocGetErrorLevel "{}"
rosservice call SickLocSetLinesForSupportActive "{active: 1}"
rosservice call SickLocSetOdometryActive "{active: 1}"
rosservice call SickLocSetRecordingActive "{active: 1}"
rosservice call SickLocSetRingBufferRecordingActive "{active: 1}"
rosservice call SickLocSetMappingActive "{active: 1}"
rosservice call SickLocSwitchMap "{submapname: \"2020-12-21_CNord_40m_v0.vmap\"}"
rosservice call SickLocSaveRingBufferRecording "{reason: \"2021-08-16 test\"}"
rosservice call SickLocSetKinematicVehicleModelActive "{active: 1}"
rosservice call SickLocClearMapCache "{}"
rosservice call SickLocLoadMapToCache "{mappath: \"2020-12-21_CNord_40m_v0.vmap\"}"
```

Odometry:
* scripts_LLS-OdometryUDPSender.py (sick): läuft, verwendet aber MsgType 1, MsgTypeVersion 1 -> nach Spezifikation MsgType 1, MsgTypeVersion 5 (oder MsgTypeVersion 4)

## Test notes 21.9.2021, ROS-2, Linux

Run OK:
```
source ./install/setup.bash
ros2 run sick_lidar_localization2 sick_lidar_localization ./src/sick_lidar_localization2/launch/sick_lidar_localization2.launch udp_ip_sim_output:=192.168.0.100 verbose:=1
ros2 run sick_lidar_localization2 pointcloud_converter ./src/sick_lidar_localization2/launch/pointcloud_converter.launch
rviz2 -d ./src/sick_lidar_localization2/test/config/rviz2_sick_lidar_localization_pointcloud.rviz2
ros2 topic echo /localizationcontroller/out/odometry_message_0105
ros2 topic echo /localizationcontroller/out/localizationcontroller_result_message_0502
```

Services OK:
```
ros2 service call SickLocIsSystemReady sick_lidar_localization2/srv/SickLocIsSystemReadySrv "{}"
ros2 service call SickLocGetMap sick_lidar_localization2/srv/SickLocGetMapSrv "{}"
ros2 service call SickLocGetSystemState sick_lidar_localization2/srv/SickLocGetSystemStateSrv "{}"
ros2 service call SickLocSetMappingActive sick_lidar_localization2/srv/SickLocSetMappingActiveSrv "{active: 1}"
ros2 service call SickLocSetOdometryActive sick_lidar_localization2/srv/SickLocSetOdometryActiveSrv "{active: 1}"
ros2 service call SickLocStop sick_lidar_localization2/srv/SickLocStopSrv "{}"
ros2 service call SickLocStartLocalizing sick_lidar_localization2/srv/SickLocStartLocalizingSrv "{}"
ros2 service call SickLocInitializeAtPose sick_lidar_localization2/srv/SickLocInitializeAtPoseSrv "{x: -8206, y: 4580, yaw: 85200, searchradius: 1000}"
ros2 service call SickLocResumeAtPose sick_lidar_localization2/srv/SickLocResumeAtPoseSrv "{x: 1000, y: 1000, yaw: 1000}"
ros2 service call SickLocRequestTimestamp sick_lidar_localization2/srv/SickLocRequestTimestampSrv "{}"
ros2 service call SickLocGetErrorLevel sick_lidar_localization2/srv/SickLocGetErrorLevelSrv "{}"
ros2 service call SickLocSetOdometryActive sick_lidar_localization2/srv/SickLocSetOdometryActiveSrv "{active: 1}"
ros2 service call SickLocSetRecordingActive sick_lidar_localization2/srv/SickLocSetRecordingActiveSrv "{active: 1}"
ros2 service call SickLocSetRingBufferRecordingActive sick_lidar_localization2/srv/SickLocSetRingBufferRecordingActiveSrv "{active: 1}"
ros2 service call SickLocSaveRingBufferRecording sick_lidar_localization2/srv/SickLocSaveRingBufferRecordingSrv "{reason: \"2021-08-16 test\"}"
ros2 service call SickLocSetMappingActive sick_lidar_localization2/srv/SickLocSetMappingActiveSrv "{active: 1}"
ros2 service call SickLocSwitchMap sick_lidar_localization2/srv/SickLocSwitchMapSrv "{submapname: \"2020-12-21_CNord_40m_v0.vmap\"}"
ros2 service call SickLocClearMapCache sick_lidar_localization2/srv/SickLocClearMapCacheSrv "{}"
ros2 service call SickLocLoadMapToCache sick_lidar_localization2/srv/SickLocLoadMapToCacheSrv "{mappath: \"2020-12-21_CNord_40m_v0.vmap\"}"
ros2 service call SickLocSetKinematicVehicleModelActive sick_lidar_localization2/srv/SickLocSetKinematicVehicleModelActiveSrv "{active: 1}"
```

## Test notes 21.9.2021, Linux nativ

Run OK:
```
mkdir ./build
pushd ./build
cmake -DROS_VERSION=0 -G "Unix Makefiles" ..
make -j4
./sick_lidar_localization ../launch/sick_lidar_localization2.launch verbose:=1
pushd
```

Services OK:
```
./build/gen_service_call SickLocIsSystemReady POST "{}"
./build/gen_service_call SickLocGetMap POST "{}"
./build/gen_service_call SickLocGetSystemState POST "{}"
./build/gen_service_call SickLocSetMappingActive POST "{\"data\":{\"active\":true}}"
./build/gen_service_call SickLocSetOdometryActive POST "{\"data\":{\"active\":true}}"
./build/gen_service_call SickLocStop POST "{}"
./build/gen_service_call SickLocStartLocalizing POST "{}"
./build/gen_service_call SickLocInitializeAtPose POST "{\"data\":{\"pose\":{\"x\":-8206,\"y\":4580,\"yaw\":85200},\"searchRadius\":1000}}"
./build/gen_service_call SickLocResumeAtPose POST "{\"data\":{\"pose\":{\"x\":1000,\"y\":1000,\"yaw\":1000}}}"
./build/gen_service_call SickLocRequestTimestamp POST "{}"
./build/gen_service_call SickLocGetErrorLevel POST "{}"
./build/gen_service_call SickLocClearMapCache POST "{}"
./build/gen_service_call SickLocSetLinesForSupportActive POST "{\"data\":{\"active\":true}}"
./build/gen_service_call SickLocSetOdometryActive POST "{\"data\":{\"active\":true}}"
./build/gen_service_call SickLocSetRecordingActive POST "{\"data\":{\"active\":true}}"
./build/gen_service_call SickLocSetRingBufferRecordingActive POST "{\"data\":{\"active\":true}}"
./build/gen_service_call SickLocSaveRingBufferRecording POST "{\"data\":{\"reason\":\"2021-08-16 test\"}}"
./build/gen_service_call SickLocSetMappingActive POST "{\"data\":{\"active\":true}}"
./build/gen_service_call SickLocSwitchMap POST "{\"data\":{\"subMapName\":\"2020-12-21_CNord_40m_v0.vmap\"}}"
./build/gen_service_call SickLocLoadMapToCache POST "{\"data\":{\"mapPath\":\"2020-12-21_CNord_40m_v0.vmap\"}}"
./build/gen_service_call SickLocSetKinematicVehicleModelActive POST "{\"data\":{\"active\":true}}"
```

## Odometrie

Test 5.10.2021, ROS-1, Linux, SIM 2.0.0.14R, OK:

```
source ./install/setup.bash 
roslaunch sick_lidar_localization sick_lidar_localization.launch verbose:=1 udp_ip_sim_output:=192.168.0.100
roslaunch sick_lidar_localization pointcloud_converter.launch
rosrun rviz rviz -d ./src/sick_lidar_localization2/test/config/rviz_sick_lidar_localization_pointcloud.rviz
# rosservice call SickLocStop "{}"
# rosservice call SickLocStartLocalizing "{}"
# rosservice call SickLocSetMappingActive "{active: 1}"
# rosservice call SickLocSetOdometryActive "{active: 1}"
# rosservice call SickLocSetKinematicVehicleModelActive "{active: 1}"
python3 ./src/sick_lidar_localization2_pretest/test/rest_server/python/send_ros_odom_messages.py
```

Test 5.10.2021, ROS-2, Linux, SIM 2.0.0.14R, OK:

```
source ./install/setup.bash 
ros2 run sick_lidar_localization sick_lidar_localization ./src/sick_lidar_localization2_pretest/launch/sick_lidar_localization.launch udp_ip_sim_output:=192.168.0.100 verbose:=1
ros2 run sick_lidar_localization pointcloud_converter ./src/sick_lidar_localization2_pretest/launch/pointcloud_converter.launch
rviz2 -d ./src/sick_lidar_localization2_pretest/test/config/rviz2_sick_lidar_localization_pointcloud.rviz2
python3 ./src/sick_lidar_localization2_pretest/test/rest_server/python/send_ros2_odom_messages.py
```

Test 5.10.2021, Linux native, SIM 2.0.0.14R, OK:

```
mkdir ./build
pushd ./build
cmake -DROS_VERSION=0 -G "Unix Makefiles" ..
make -j4
./sick_lidar_localization ../launch/sick_lidar_localization.launch verbose:=1
pushd
```

Nachtrag SOPAS-Befehle/services:
* LocGetLocalizationStatus: SickLocGetLocalizationStatusSrv.srv
* GetSoftwareVersion: SickLocGetSoftwareVersionSrv.srv
* LoadPersistentConfig: SickLocLoadPersistentConfigSrv.srv
```
Native:
  ./gen_service_call SickLocGetLocalizationStatus POST "{}"
  ./gen_service_call SickLocGetSoftwareVersion POST "{}"
  ./gen_service_call SickLocLoadPersistentConfig POST "{}"
ROS1:
  rosservice call SickLocGetLocalizationStatus "{}"
  rosservice call SickLocGetSoftwareVersion "{}"
  rosservice call SickLocLoadPersistentConfig "{}"
ROS2:
  ros2 service call SickLocGetLocalizationStatus sick_lidar_localization/srv/SickLocGetLocalizationStatusSrv "{}"
  ros2 service call SickLocGetSoftwareVersion sick_lidar_localization/srv/SickLocGetSoftwareVersionSrv "{}"
  ros2 service call SickLocLoadPersistentConfig sick_lidar_localization/srv/SickLocLoadPersistentConfigSrv "{}"
```

Test 13.10.2021, ROS-1, Linux, SIM 2.0.0.14R, OK:

```
source ./install/setup.bash 
roslaunch sick_lidar_localization sick_lidar_localization.launch verbose:=1 udp_ip_sim_output:=192.168.0.100
rosservice call SickLocGetLocalizationStatus "{}"
rosservice call SickLocGetSoftwareVersion "{}"
rosservice call SickLocLoadPersistentConfig "{}"
rosservice call SickLocIsSystemReady "{}"
rosservice call SickLocGetMap "{}"
rosservice call SickLocGetSystemState "{}"
rosservice call SickLocSetMappingActive "{active: true}"
rosservice call SickLocSetOdometryActive "{active: true}"
rosservice call SickLocSetLinesForSupportActive "{active: true}"
rosservice call SickLocSetOdometryActive "{active: true}"
rosservice call SickLocSetRecordingActive "{active: true}"
rosservice call SickLocSetRingBufferRecordingActive "{active: true}"
rosservice call SickLocSetKinematicVehicleModelActive "{active: true}"
```

Test 13.10.2021, ROS-2, Linux, SIM 2.0.0.14R, OK:

```
source ./install/setup.bash 
ros2 run sick_lidar_localization sick_lidar_localization ./src/sick_lidar_localization2_pretest/launch/sick_lidar_localization.launch udp_ip_sim_output:=192.168.0.100 verbose:=1
ros2 service call SickLocGetLocalizationStatus sick_lidar_localization/srv/SickLocGetLocalizationStatusSrv "{}"
ros2 service call SickLocGetSoftwareVersion sick_lidar_localization/srv/SickLocGetSoftwareVersionSrv "{}"
ros2 service call SickLocLoadPersistentConfig sick_lidar_localization/srv/SickLocLoadPersistentConfigSrv "{}"
```

Renaming of services (Release 5.3.0, 19.10.2021):

| **Old services**    |  **Services release 5.3.0**   | **Rest-API commando** |
|---|---|---|
| "SickLocAutoStartSavePose"              | "LocAutoStartSavePose"               | "LocAutoStartSavePose"               |
| "SickLocClearMapCache"                  | "LocClearMapCache"                   | "LocClearMapCache"                   |
| "SickLocGetErrorLevel"                  | "LocGetErrorLevel"                   | "GetErrorLevel"                      |
| "SickLocGetMap"                         | "LocGetMap"                          | "LocGetMap"                          |
| "SickLocGetSystemState"                 | "LocGetSystemState"                  | "LocGetSystemState"                  |
| "SickLocInitializeAtPose"               | "LocInitializeAtPose"                | "LocInitializeAtPose"                |
| "SickLocIsSystemReady"                  | "LocIsSystemReady"                   | "IsSystemReady"                      |
| "SickLocLoadMapToCache"                 | "LocLoadMapToCache"                  | "LocLoadMapToCache"                  |
| "SickLocResumeAtPose"                   | "LocResumeAtPose"                    | "LocResumeAtPose"                    |
| "SickLocSaveRingBufferRecording"        | "LocSaveRingBufferRecording"         | "LocSaveRingBufferRecording"         |
| "SickLocSetKinematicVehicleModelActive" | "LocSetKinematicVehicleModelActive"  | "LocSetKinematicVehicleModelActive"  |
| "SickLocSetLinesForSupportActive"       | "LocSetLinesForSupportActive"        | "LocSetLinesForSupportActive"        |
| "SickLocSetMappingActive"               | "LocSetMappingActive"                | "LocSetMappingActive"                |
| "SickLocSetMap"                         | "LocSetMap"                          | "LocSetMap"                          |
| "SickLocSetOdometryActive"              | "LocSetOdometryActive"               | "LocSetOdometryActive"               |
| "SickLocSetRecordingActive"             | "LocSetRecordingActive"              | "LocSetRecordingActive"              |
| "SickLocSetRingBufferRecordingActive"   | "LocSetRingBufferRecordingActive"    | "LocSetRingBufferRecordingActive"    |
| "SickLocStartLocalizing"                | "LocStartLocalizing"                 | "LocStartLocalizing"                 |
| "SickLocStop"                           | "LocStop"                            | "LocStop"                            |
| "SickLocSwitchMap"                      | "LocSwitchMap"                       | "LocSwitchMap"                       |
| "SickLocGetLocalizationStatus"          | "LocGetLocalizationStatus"           | "LocGetLocalizationStatus"           |
| "SickLocGetSoftwareVersion"             | "LocGetSoftwareVersion"              | "GetSoftwareVersion"                 |
| "SickLocLoadPersistentConfig"           | "LocLoadPersistentConfig"            | "LoadPersistentConfig"               |
| "SickLocRequestTimestamp"               | "LocRequestTimestamp"                | "LocRequestTimestamp"                |

