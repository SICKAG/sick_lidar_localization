# sick_lidar_localization

sick_lidar_localization is an open-source project to support the LiDAR-LOC software of the company SICK 
using the ROS-framework

## Install and build

To install and build ROS support for SICK LiDAR Localization, run the following commands:

```console
cd ~/catkin_ws/src
# sudo apt-get install expect                                        # install unbuffer for logging console output, development only
# git clone https://github.com/madler/crcany.git                     # get crc implementation, development only
git clone https://github.com/michael1309/sick_lidar_localization.git # get ros driver sources for sick localization
cd ..
source /opt/ros/melodic/setup.bash
catkin_make
catkin_make install
source ./devel/setup.bash
```

## Run

To run SICK LiDAR Localization under ROS, install the SICK localization controller and run the ros driver:

1. Install and run the SICK localization controller. See [doc/Quickstart-Setup-SOPASair.md](doc/Quickstart-Setup-SOPASair.md)
for a quickstart. Find detailed information in the operation manuals published on 
https://www.sick.com/de/en/search?text=NAV-LOC.

2. Start the sick_lidar_localization driver:

```console
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch sick_lidar_localization sim_loc_driver.launch localization_controller_ip_adress:=192.168.0.1
```

Note: The IP address of the SICK localization controller is 192.168.0.1 by default. Depending on your network configuration,
different IP addresses can be configured by commandline argument `localization_controller_ip_adress:=<ip-address>` or 
by changing the value `localization_controller_default_ip_adress: "<ip-address>"` in the drivers configuration file 
[yaml/sim_loc_driver.yaml](yaml/sim_loc_driver.yaml).

The sick_lidar_localization driver connects to the localization controller, receives result port telegrams and publishes
them on ros topic "/sick_lidar_localization/driver/result_telegrams". After successful installation, you can view the telegram
messages with

```console
rostopic echo "/sick_lidar_localization/driver/result_telegrams"
```

## Result port telegrams

Result port telegrams are continously received and published to inform about a sensors pose (position and orientation).
ROS result telegram messages have the datatype [msg/SickLocResultPortTelegramMsg.msg](msg/SickLocResultPortTelegramMsg.msg),
containing 4 sub-elements:

Type | Definition | Name | Description
--- | --- | --- | ---
Header | http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html | header | ROS Header with sequence id, timestamp and frame id
SickLocResultPortHeaderMsg | [msg/SickLocResultPortHeaderMsg.msg](msg/SickLocResultPortHeaderMsg.msg) | telegram_header | 52 byte header of a result port telegram
SickLocResultPortPayloadMsg | [msg/SickLocResultPortPayloadMsg.msg](msg/SickLocResultPortPayloadMsg.msg) | telegram_payload | 52 byte payload of a result port telegram
SickLocResultPortCrcMsg | [msg/SickLocResultPortCrcMsg.msg](msg/SickLocResultPortCrcMsg.msg) | telegram_trailer | 2 byte CRC trailer of a result port telegram

The following table shows a complete list over all elements of the result telegram message:

Type | Name | Description
--- | --- | ---
uint32 | header.seq | ROS sequence identifier (consecutively increasing id)
time | header.stamp | ROS timestamp in seconds and nanoseconds
string | header.frame_id | ROS frame identifier
uint32 | telegram_header.MagicWord | Magic word SICK (0x53 0x49 0x43 0x4B)
uint32 | telegram_header.Length | Length of telegram incl. header, payload, and trailer
uint16 | telegram_header.PayloadType | Payload type: 0x06c2 = Little Endian, 0x0642 = Big Endian
uint16 | telegram_header.PayloadVersion | Version of PayloadType structure
uint32 | telegram_header.OrderNumber | Order number of the localization controller
uint32 | telegram_header.SerialNumber | Serial number of the localization controller
uint8[] | telegram_header.FW_Version | Software version of the localization controller, 20 byte
uint32 | telegram_header.TelegramCounter | Telegram counter since last start-up
uint64 | telegram_header.SystemTime | Not used
uint16 | telegram_payload.ErrorCode | ErrorCode 0: OK, ErrorCode 1: UNKNOWNERROR
uint32 | telegram_payload.ScanCounter | Counter of related scan data
uint32 | telegram_payload.Timestamp | Time stamp of the pose [ms], indicating the time at which the pose is calculated
int32 | telegram_payload.PoseX | Position X of the vehicle on the map in cartesian global coordinates [mm]
int32 | telegram_payload.PoseY | Position Y of the vehicle on the map in cartesian global coordinates [mm]
int32 | telegram_payload.PoseYaw | Orientation (yaw) of the vehicle on the map [mdeg]
uint32 | telegram_payload.Reserved1 | Reserved
int32 | telegram_payload.Reserved2 | Reserved
uint8 | telegram_payload.Quality | Quality of pose [1 … 100], 1 = bad pose quality, 100 = good pose quality
uint8 | telegram_payload.OutliersRatio | Ratio of beams that cannot be assigned to the current reference map [%]
int32 | telegram_payload.CovarianceX | Covariance c1 of the pose X [mm^2]
int32 | telegram_payload.CovarianceY | Covariance c5 of the pose Y [mm^2]
int32 | telegram_payload.CovarianceYaw | Covariance c9 of the pose Yaw [mdeg^2]
uint64 | telegram_payload.Reserved3 | Reserved
uint16 | telegram_trailer.Checksum | CRC16-CCITT over length of header (52 bytes) and payload (52 bytes) without 2 bytes of the trailer

Example output of a result telegram (ros message of type [msg/SickLocResultPortTelegramMsg.msg](msg/SickLocResultPortTelegramMsg.msg)):

```
header: 
  seq: 0
  stamp: 
    secs: 1571732539
    nsecs: 914320492
  frame_id: "sick_lidar_localization"
telegram_header: 
  MagicWord: 1397310283
  Length: 106
  PayloadType: 1602
  PayloadVersion: 1
  OrderNumber: 1097816
  SerialNumber: 19047026
  FW_Version: [0, 0, 0, 0, 0, 0, 0, 76, 76, 83, 32, 86, 48, 46, 49, 46, 57, 46, 120, 66]
  TelegramCounter: 621
  SystemTime: 9487549550560573440
telegram_payload: 
  ErrorCode: 0
  ScanCounter: 623
  Timestamp: 3468531
  PoseX: 93
  PoseY: 33
  PoseYaw: 17895
  Reserved1: 0
  Reserved2: 0
  Quality: 55
  OutliersRatio: 0
  CovarianceX: 32905
  CovarianceY: 39315
  CovarianceYaw: 1210527
  Reserved3: 0
telegram_trailer: 
  Checksum: 25105
```

Note: Result telegrams always have the same MagicWord: `1397310283` (`0x5349434B` hex resp. `SICK` in ascii/ansi) and a
length of 106 bytes (`Length: 106`).

## Diagnostics

The sick_lidar_localization driver publishes diagnostic messages on ros topic "/sick_lidar_localization/driver/diagnostic", 
which can be examined by command `rostopic echo "/sick_lidar_localization/driver/diagnostic"`. 
Example diagnostic messages  (ros message of type [msg/SickLocDiagnosticMsg.msg](msg/SickLocDiagnosticMsg.msg)):

```
header: 
  seq: 3
  stamp: 
    secs: 1571735362
    nsecs: 630462359
  frame_id: "sick_lidar_localization"
error_code: 0
message: "sim_loc_driver: tcp connection established to localization controller 192.168.0.1:2201"
---
header: 
  seq: 4
  stamp: 
    secs: 1571735362
    nsecs: 631530587
  frame_id: "sick_lidar_localization"
error_code: 0
message: "sim_loc_driver: status okay, receiving and publishing result telegrams"
```

Note: In case of errors (f.e. connection lost, parse errors or invalid telegrams), diagnostic messages with an error code
are published. Error codes defined in [include/sick_lidar_localization/sim_loc_driver_thread.h](include/sick_lidar_localization/sim_loc_driver_thread.h) are:

Error code | Value | Description
--- | --- | ---
NO_ERROR | 0 | No error, driver works as expected
NO_TCP_CONNECTION | 1 |  Tcp connection to localization controller could not be established
PARSE_ERROR | 2 | Parse error, telegram could not be decoded
CONFIGURATION_ERROR | 3 | Invalid driver configuration
INTERNAL_ERROR | 4 | Internal error (should never happen)

## Configuration

The sick_lidar_localization driver is configured by file [yaml/sim_loc_driver.yaml](yaml/sim_loc_driver.yaml):

Parametername | Defaultvalue | Description
--- | --- | ---
localization_controller_default_ip_adress | "192.168.0.1" | Default IP adress "192.168.0.1" of the localization controller (if not otherwise set by parameter "localization_controller_ip_adress")
result_telegrams_tcp_port | 2201 | TCP port number of the localization controller sending localization results
tcp_connection_retry_delay | 1.0 | Delay in seconds to retry to connect to the localization controller, default 1 second
result_telegrams_topic | "/sick_lidar_localization/driver/result_telegrams" | ros topic to publish result port telegram messages (type SickLocResultPortTelegramMsg)
result_telegrams_frame_id | "sick_lidar_localization" | ros frame id of result port telegram messages (type SickLocResultPortTelegramMsg)
diagnostic_topic | "/sick_lidar_localization/driver/diagnostic" | ros topic to publish diagnostic messages (type SickLocDiagnosticMsg)
diagnostic_frame_id | "sick_lidar_localization" | ros frame id of diagnostic messages (type SickLocDiagnosticMsg)
monitoring_rate | 1.0 | frequency to monitor driver messages, once per second by default
monitoring_message_timeout | 1.0 | timeout for driver messages, shutdown tcp-sockets and reconnect after message timeout, 1 second by default

Note: The IP address of the SICK localization controller (192.168.0.1 by default) can be set by commandline argument 
`localization_controller_ip_adress:=<ip-address>` when starting the driver with 
`roslaunch sick_lidar_localization sim_loc_driver.launch localization_controller_ip_adress:=<ip-address>`.

## Testing

To test the sick_lidar_localization ros driver, just connect your ros system with the SICK localization controller,
start the driver and observe the result port telegrams and diagnostic messages:

```console
cd ~/catkin_ws
source ./devel/setup.bash
rostopic echo "/sick_lidar_localization/driver/diagnostic" &
rostopic echo "/sick_lidar_localization/driver/result_telegrams" &
roslaunch sick_lidar_localization sim_loc_driver.launch localization_controller_ip_adress:=192.168.0.1
```

Warnings and error messages will be printed in case of failures like unreachable controller, connection losts or 
invalid telegrams.

For automated tests over long time, the values of result telegram messages can be automatically checked against minimal
and maximal limits. These limits - i.e. min. and max. allowed values for each element in a result telegram message - can
be configured by a yaml-file and can be automatically checked by sim_loc_driver_check:

```console
roslaunch sick_lidar_localization sim_loc_driver_check.launch sim_loc_driver_check_cfg:=message_check_demo.yaml
```

Each result telegram message which violates the limits defined in file [yaml/message_check_demo.yaml](yaml/message_check_demo.yaml) 
will result in warnings and error messages. By providing limits adapted to a specific scenario, result telegram messages 
can be checked automatically over long time.

Configuration file [yaml/message_check_demo.yaml](yaml/message_check_demo.yaml) defines lower bounds in section result_telegram_min_values 
and upper bounds in section result_telegram_max_values for all values of a result telegram. By default, these limits are 
configured to fit all scenarios. Feel free to provide a configuration file with narrower limits; this might help to track
occasionally or otherwise hard to find problems.

## Simulation and offline testing

Offline simulation without hardware or a dedicated localization controller enables a wider range of automated tests and
scenarios. Simulation can verify the handling of errors like invalid telegrams, broken networks or other errors hard to create 
when using hardware controllers.

sim_loc_test_server simulates a localization controller and generates random based result port telegrams. To run an offline
simulation, start the test server and the sick_lidar_localization ros driver on you local system:

```console
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch sick_lidar_localization sim_loc_test_server.launch & # start test server to generate result port telegrams
sleep 3 # make sure ros core and sim_loc_test_server are up and running 
roslaunch sick_lidar_localization sim_loc_driver.launch localization_controller_ip_adress:=127.0.0.1
```

The sick_lidar_localization ros driver will connect to the local test server, receive random based result port telegrams
and publish them on ros topic "/sick_lidar_localization/driver/result_telegrams". Telegram messages can be viewed with

```console
rostopic echo "/sick_lidar_localization/driver/diagnostic" &
rostopic echo "/sick_lidar_localization/driver/result_telegrams" &
```

Since the driver input (i.e. the binary result port telegrams) and the expected output (i.e. the generated telegrams) are
known, the driver output can be verified. The test server sim_loc_test_server publishes testcases with both the 
generated telegrams (expected driver output) and their binaries (driver input) on ros topic "/sick_lidar_localization/test_server/result_testcases".
Comparing the telegram messages (driver output) with the testcases (expected driver output), the driver can be verified.
This is done by verify_sim_loc_driver, which just subscribes to both topics, compares the telegram messages from the driver 
to the expected telegram from the test server, and counts and prints warnings if both messages are not identical.

To run the verification, start driver and test server as described above, and launch verify_sim_loc_driver:

```console
roslaunch sick_lidar_localization verify_sim_loc_driver.launch
```

After stopping the test, a summary note is printed. Example output by verify_sim_loc_driver:

```
VerifierThread: verification thread summary: 588 messages checked, 0 failures. 
```

Use `run_simu.bash` in folder src/sick_lidar_localization/test/scripts to run an automated offline test:

```console
cd ~/catkin_ws/src/sick_lidar_localization/test/scripts
./run_simu.bash
```

In case of a successful test, the following summary will be displayed (example output):

```
MessageCheckThread: check messages thread summary: 599 messages checked, 0 failures.
VerifierThread: verification thread summary: 585 messages checked, 0 failures.
```

## Error simulation and error handling

The sick_lidar_localization ros driver monitors the telegram messages. In case of errors (network errors like unreachable
hosts or connection lost, or communication errors like invalid telegrams or false checksums), the tcp connection to the
localization controller is automatically closed and re-established. A diagnostic message will be logged and published, 
f.e. with error code 1 (NO_TCP_CONNECTION) after connect lost:

```
sick_lidar_localization,1,sim_loc_driver: no tcp connection to localization controller 192.168.0.1:2201
```

Errors can be simulated and tested using sim_loc_test_server with commandline argument `error_simulation:=true`:

```console
roslaunch sick_lidar_localization sim_loc_test_server.launch error_simulation:=true & # run test server in error simulation mode
```

In error simulation mode, sim_loc_test_server will toggle between errors and correct execution each 10 seconds. 
After switching to normal mode, sim_loc_test_server checks for telegram messages from the driver and displays an
error, if the driver isn't reconnecting or isn't publishing telegrams. 

Currently, the following errors are simulated and tested by sim_loc_test_server (enumerated in 
[include/sick_lidar_localization/sim_loc_test_server_thread.h](include/sick_lidar_localization/sim_loc_test_server_thread.h):

Error testcase | Description
--- | ---
DONT_LISTEN | Testserver does not open a listening port
DONT_ACCECPT | Testserver does not accecpt tcp clients
DONT_SEND | Testserver does not send any data
SEND_RANDOM_TCP | Testserver sends invalid random tcp packets
SEND_INVALID_TELEGRAMS | Testserver sends invalid telegrams (invalid data, false checksums, etc.)

sim_loc_test_server runs each of these error modes for 10 seconds and switches then back to normal mode.
After switching to normal mode, the driver has to reconnect and the test server expects
new telegram messages. In case of missing telegram messages from the driver, the test will fail.

Use `run_error_simu.bash` in folder src/sick_lidar_localization/test/scripts to run an automated error simulation and error test:

```console
cd ~/catkin_ws/src/sick_lidar_localization/test/scripts
./run_error_simu.bash
```

Note: This test intentionally creates lots of error messages. In case of successful error simulation and test, the following
summary will be displayed:

```
sick_lidar_localization error simulation summary: finished 3 testcases, reconnect after connection lost okay.
TestServerThread: error simulation summary: 6 of 6 testcases passed, 0 failures.
MessageCheckThread: check messages thread summary: 1153 messages checked, 0 failures.
```
 
## Usage example: pointcloud_convert

pointcloud_convert in file [src/pointcloud_converter.cpp](src/pointcloud_converter.cpp) implements a subscriber to 
sim_loc_driver messages and converts them to PointCloud2 messages, which are published on topic "/cloud".
They can be viewed by rviz:

```console
rosrun tf static_transform_publisher 0 0 0 0 0 0 map sick_lidar_localization 10 &
rosrun rviz rviz &
```

Example output:

![doc/screenshot-rviz-simu1.png](doc/screenshot-rviz-simu1.png)

pointcloud_convert is an usage example for sick_lidar_localization, too, and shows how to subscribe and use the
sick_lidar_localization messages published by the driver. Feel free to use this example as a starting point for
customization. pointcloud_convert in file [src/pointcloud_converter.cpp](src/pointcloud_converter.cpp) 
is just the main entry point. Conversion and message handling is implemented in class 
sick_lidar_localization::PointCloudConverter in file [src/sim_loc_pointcloud_converter.cpp](src/sim_loc_pointcloud_converter.cpp). 

## Source code, doxygen

The main entry point of the ros driver is implemented in file [src/sim_loc_driver.cpp](src/sim_loc_driver.cpp).
It creates an instance of class sick_lidar_localization::DriverMonitor implemented in [src/sim_loc_driver_monitor.cpp](src/sim_loc_driver_monitor.cpp).

sick_lidar_localization::DriverMonitor creates and monitors an instance of class sick_lidar_localization::DriverThread implemented in
[src/sim_loc_driver_thread.cpp](src/sim_loc_driver_thread.cpp), which runs all driver functions, including the
telegram parser implemented by class sick_lidar_localization::ResultPortParser. 

After successful initialization, the driver runs 3 threads:

- The receiver thread implemented by sick_lidar_localization::DriverThread::runReceiverThreadCb in file 
[src/sim_loc_driver_thread.cpp](src/sim_loc_driver_thread.cpp). The receiver thread connects to the localization 
controller, receives binary result telegram and buffers them in a fifo (first-in, first-out)

- The converter thread implemented by sick_lidar_localization::DriverThread::runConverterThreadCb in file 
[src/sim_loc_driver_thread.cpp](src/sim_loc_driver_thread.cpp). The converter thread pops binary telegrams from the
fifo, decodes and parses result port telegrams and publishes telegram messages on ros topic 
"/sick_lidar_localization/driver/result_telegrams". Telegram decoding is implemented by sick_lidar_localization::ResultPortParser::decode
in file [src/sim_loc_result_port_parser.cpp](src/sim_loc_result_port_parser.cpp).

- The monitoring thread implemented by sick_lidar_localization::DriverMonitor::runMonitorThreadCb in file 
[src/sim_loc_driver_monitor.cpp](src/sim_loc_driver_monitor.cpp). It subscribes and monitors the telegram messages from
sick_lidar_localization::DriverThread. In case of errors or missing telegram messages, the tcp connection to the 
localization controller is closed and re-established.

Doxygen source code documentation is supported. To build the doxygen source code documentation, run the following commands:

```console
cd ~/catkin_ws
catkin_make RunDoxygen
```

Doxygen will create source code documentation in folder `~/catkin_ws/build/sick_lidar_localization/doxygen`. Run
```console
firefox ~/catkin_ws/build/sick_lidar_localization/doxygen/html/index.html &
```
to view the doxygen generated html documentation.

Note: Doxygen needs to be installed. If package doxygen wasn't found during build, run
```console
sudo apt-get install doxygen
```
to install doxygen.

## FAQ, troubleshooting and further documentation

FAQ, troubleshooting:

* FAQ: [doc/faq.md](doc/faq.md)

Quickstart, tutorials and manuals:

* Quickstart Setup LiDAR-LOC: [doc/Quickstart-Setup-SOPASair.md](doc/Quickstart-Setup-SOPASair.md)

* Operation manuals: https://www.sick.com/de/en/search?text=NAV-LOC
