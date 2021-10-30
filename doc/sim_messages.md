# UDP stream messages

LiDAR-LOC receives and sends messages from resp. to the localization controller using UDP. [UDP output messages](#sim_output_messages) are UDP messages sent from localization server to the local PC. [UDP input messages](#sim_input_messages) are UDP messages sent from the local PC to the localization server. On ROS-1 and ROS-2, these UDP-messages are converted from resp. to ROS messages. On native Linux and Windows systems, these UDP-messages can be processed using the [C++ API](../README.md#cpp_api).

## <a name="sim_output_messages"></a> UDP output messages

UDP output messages are:

* [Odometry messages type 1 version 4](../msg/OdometryMessage0104.msg)
* [Odometry messages type 1 version 5](../msg/OdometryMessage0105.msg)
* [Code measurement messages type 3 version 4](../msg/CodeMeasurementMessage0304.msg)
* [Line measurement messages type 4 version 3](../msg/LineMeasurementMessage0403.msg)
* [Line measurement messages type 4 version 4](../msg/LineMeasurementMessage0404.msg)
* [Localization result messages type 5 version 2](../msg/LocalizationControllerResultMessage0502.msg)

A detailed description of these messages is available in the operation manuals published on https://supportportal.sick.com/. 

ROS-1 examples to receive UDP output messages (odometry, code measurement, line measurement and localization result message):

```
source ./install/setup.bash 
roslaunch sick_lidar_localization sick_lidar_localization.launch verbose:=1 &
rostopic echo /localizationcontroller/out/odometry_message_0104 &
rostopic echo /localizationcontroller/out/odometry_message_0105 &
rostopic echo /localizationcontroller/out/code_measurement_message_0304 &
rostopic echo /localizationcontroller/out/line_measurement_message_0403 &
rostopic echo /localizationcontroller/out/line_measurement_message_0404 &
rostopic echo /localizationcontroller/out/localizationcontroller_result_message_0502 &
```

Example output from `sick_lidar_localization` with option `verbose:=1` and `rostopic` on ROS-1 (odometry, code measurement, line measurement and localization result message):

```
[ INFO] [1628157236.969082207]: sick_lidar_localization::UDPReceiverThread: udp message received: OdometryPayload0104:{telegram_count:1000000064, timestamp:12345678901234631, x_velocity:-20000, y_velocity:-10000, angular_velocity:-5000, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
[ INFO] [1628157236.969182035]: UDPMessage::InfoListener: udpMessageReceived: OdometryPayload0104:{telegram_count:1000000064, timestamp:12345678901234631, x_velocity:-20000, y_velocity:-10000, angular_velocity:-5000, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
[ INFO] [1628157236.969221612]: PublishUdpMessagesListener::udpMessageReceived: OdometryPayload0104:{telegram_count:1000000064, timestamp:12345678901234631, x_velocity:-20000, y_velocity:-10000, angular_velocity:-5000, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
header: seq: 51 stamp: secs: 1628157236 nsecs: 969233201 frame_id: ''
telegram_count: 1000000064
timestamp: 12345678901234631
x_velocity: -20000
y_velocity: -10000
angular_velocity: -5000
sync_timestamp_sec: 1630099057
sync_timestamp_nsec: 333530902
sync_timestamp_valid: 1

[ INFO] [1628157237.004634426]: sick_lidar_localization::UDPReceiverThread: udp message received: OdometryPayload0105:{telegram_count:1000000066, timestamp:12345678901234633, x_position:30000, y_position:25000, heading:123123, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
[ INFO] [1628157237.004702891]: UDPMessage::InfoListener: udpMessageReceived: OdometryPayload0105:{telegram_count:1000000066, timestamp:12345678901234633, x_position:30000, y_position:25000, heading:123123, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
[ INFO] [1628157237.004739923]: PublishUdpMessagesListener::udpMessageReceived: OdometryPayload0105:{telegram_count:1000000066, timestamp:12345678901234633, x_position:30000, y_position:25000, heading:123123, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
header: seq: 48 stamp: secs: 1628157237 nsecs:   4751669 frame_id: ''
telegram_count: 1000000066
timestamp: 12345678901234633
x_position: 30000
y_position: 25000
heading: 123123
sync_timestamp_sec: 1630099057
sync_timestamp_nsec: 333530902
sync_timestamp_valid: 1

[ INFO] [1628157237.140145475]: sick_lidar_localization::UDPReceiverThread: udp message received: CodeMeasurementPayload0304:{telegram_count:1000000071, timestamp:12345678901234638, code:12345, distance:67890, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
[ INFO] [1628157237.140227342]: UDPMessage::InfoListener: udpMessageReceived: CodeMeasurementPayload0304:{telegram_count:1000000071, timestamp:12345678901234638, code:12345, distance:67890, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
[ INFO] [1628157237.140264435]: PublishUdpMessagesListener::udpMessageReceived: CodeMeasurementPayload0304:{telegram_count:1000000071, timestamp:12345678901234638, code:12345, distance:67890, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
header: seq: 48 stamp: secs: 1628157237 nsecs: 140293307 frame_id: ''
telegram_count: 1000000071
timestamp: 12345678901234638
code: 12345
distance: 67890
sync_timestamp_sec: 1630099057
sync_timestamp_nsec: 333530902
sync_timestamp_valid: 1

[ INFO] [1628157237.275969209]: sick_lidar_localization::UDPReceiverThread: udp message received: LineMeasurementPayload0403:{telegram_count:1000000076, timestamp:12345678901234643, num_lanes:2, lanes:[300,345], sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
[ INFO] [1628157237.276035522]: UDPMessage::InfoListener: udpMessageReceived: LineMeasurementPayload0403:{telegram_count:1000000076, timestamp:12345678901234643, num_lanes:2, lanes:[300,345], sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
[ INFO] [1628157237.276079245]: PublishUdpMessagesListener::udpMessageReceived: LineMeasurementPayload0403:{telegram_count:1000000076, timestamp:12345678901234643, num_lanes:2, lanes:[300,345], sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
header: seq: 48 stamp: secs: 1628157237 nsecs: 276147870 frame_id: ''
telegram_count: 1000000076
timestamp: 12345678901234643
num_lanes: 2
lanes: [300, 345]
sync_timestamp_sec: 1630099057
sync_timestamp_nsec: 333530902
sync_timestamp_valid: 1

[ INFO] [1628157237.411670286]: sick_lidar_localization::UDPReceiverThread: udp message received: LineMeasurementPayload0404:{telegram_count:1000000081, timestamp:12345678901234648, lcp1:300, lcp2:345, lcp3:-6789, cnt_lpc:7, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
[ INFO] [1628157237.411735674]: UDPMessage::InfoListener: udpMessageReceived: LineMeasurementPayload0404:{telegram_count:1000000081, timestamp:12345678901234648, lcp1:300, lcp2:345, lcp3:-6789, cnt_lpc:7, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
[ INFO] [1628157237.411755766]: PublishUdpMessagesListener::udpMessageReceived: LineMeasurementPayload0404:{telegram_count:1000000081, timestamp:12345678901234648, lcp1:300, lcp2:345, lcp3:-6789, cnt_lpc:7, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
header: seq: 48 stamp: secs: 1628157237 nsecs: 411781304 frame_id: ''
telegram_count: 1000000081
timestamp: 12345678901234648
lcp1: 300
lcp2: 345
lcp3: -6789
cnt_lpc: 7
sync_timestamp_sec: 1630099057
sync_timestamp_nsec: 333530902
sync_timestamp_valid: 1

[ INFO] [1628157237.547426524]: sick_lidar_localization::UDPReceiverThread: udp message received: LocalizationControllerResultPayload0502:{telegram_count:1000000086, timestamp:12345678901234653, x:1020304, y:-1020304, heading:123456, loc_status:10, map_match_status:90, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
[ INFO] [1628157237.547522015]: UDPMessage::InfoListener: udpMessageReceived: LocalizationControllerResultPayload0502:{telegram_count:1000000086, timestamp:12345678901234653, x:1020304, y:-1020304, heading:123456, loc_status:10, map_match_status:90, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
[ INFO] [1628157237.547622457]: PublishUdpMessagesListener::udpMessageReceived: LocalizationControllerResultPayload0502:{telegram_count:1000000086, timestamp:12345678901234653, x:1020304, y:-1020304, heading:123456, loc_status:10, map_match_status:90, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
header: seq: 48 stamp: secs: 1628157237 nsecs: 547683518 frame_id: ''
telegram_count: 1000000086
timestamp: 12345678901234653
x: 1020304
y: -1020304
heading: 123456
loc_status: 10
map_match_status: 90
sync_timestamp_sec: 1630099057
sync_timestamp_nsec: 333530902
sync_timestamp_valid: 1
```

Use `ros2 topic echo` on ROS-2. Examples to receive UDP output messages (odometry, code measurement, line measurement and localization result message):

```
ros2 topic echo /localizationcontroller/out/odometry_message_0104 &
ros2 topic echo /localizationcontroller/out/odometry_message_0105 &
ros2 topic echo /localizationcontroller/out/code_measurement_message_0304 &
ros2 topic echo /localizationcontroller/out/line_measurement_message_0403 &
ros2 topic echo /localizationcontroller/out/line_measurement_message_0404 &
ros2 topic echo /localizationcontroller/out/localizationcontroller_result_message_0502 &
```

### Configuration of UDP output messages

Parameters can be configured in the launch file [sick_lidar_localization.launch](../launch/sick_lidar_localization.launch). The following parameters are used for UDP output messages:

| **parameter name** | **default value** | **parameter type** | **description** |
|--------------------|-------------------|--------------------|-----------------|
| udp_ip_sim_output | "" | string | IP address of your local machine (i.e. the receiver of UDP stream messages) |
| udp_port_sim_output | 5010 | int | UDP port of output messages |
| udp_sim_output_logfile | "" | string | Optional logfile for human readable UDP output messages, or "" to disable logfile |

## <a name="sim_input_messages"></a> UDP input messages

UDP input messages are:

* [Odometry messages type 1 version 1](../msg/OdometryMessage0101.msg)
* [Odometry messages type 1 version 4](../msg/OdometryMessage0104.msg)
* [Odometry messages type 1 version 5](../msg/OdometryMessage0105.msg)
* [Encoder measurement messages type 2 version 2](../msg/EncoderMeasurementMessage0202.msg)
* [Code measurement messages type 3 version 3](../msg/CodeMeasurementMessage0303.msg)
* [Line measurement messages type 4 version 3](../msg/LineMeasurementMessage0403.msg)
* [Line measurement messages type 4 version 4](../msg/LineMeasurementMessage0404.msg)
* [ROS odometry messages](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)

A detailed description of these messages is available in the operation manuals published on https://supportportal.sick.com/. 

ROS-1 examples to send UDP input messages (odometry, encoder measurement, code measurement and line measurement message):

```
source ./install/setup.bash 
roslaunch sick_lidar_localization sick_lidar_localization.launch verbose:=1 &
rostopic pub --once /localizationcontroller/in/odometry_message_0104            sick_lidar_localization/OdometryMessage0104           '{telegram_count: 1000001, timestamp: 123456789, x_velocity: -1234, y_velocity: -1234, angular_velocity: 1234}'
rostopic pub --once /localizationcontroller/in/odometry_message_0105            sick_lidar_localization/OdometryMessage0105           '{telegram_count: 1000002, timestamp: 123456780, x_position: -1234, y_position: -1234, heading: 1234}'
rostopic pub --once /localizationcontroller/in/encoder_measurement_message_0202 sick_lidar_localization/EncoderMeasurementMessage0202 '{telegram_count: 1000003, timestamp: 123456781, encoder_value: 123456789}'
rostopic pub --once /localizationcontroller/in/code_measurement_message_0303    sick_lidar_localization/CodeMeasurementMessage0303    '{telegram_count: 1000004, timestamp: 123456782, code: 1234}'
rostopic pub --once /localizationcontroller/in/line_measurement_message_0403    sick_lidar_localization/LineMeasurementMessage0403    '{telegram_count: 1000005, timestamp: 123456783, num_lanes: 1, lanes: [1234]}'
rostopic pub --once /localizationcontroller/in/line_measurement_message_0404    sick_lidar_localization/LineMeasurementMessage0404    '{telegram_count: 1000006, timestamp: 123456784, lcp1: 12, lcp2: 34, lcp3: 56, cnt_lpc: 78}'
```

Example output from `sick_lidar_localization`:

```
[ INFO] [1628159024.678353489]: sick_lidar_localization::UDPSender: udp payload OdometryPayload0104:{telegram_count:1000001, timestamp:123456789, x_velocity:-1234, y_velocity:-1234, angular_velocity:1234} sent to 192.168.0.1:5009
[ INFO] [1628159027.488724322]: sick_lidar_localization::UDPSender: udp payload OdometryPayload0105:{telegram_count:1000002, timestamp:123456780, x_position:-1234, y_position:-1234, heading:1234} sent to 192.168.0.1:5009
[ INFO] [1628159031.805828483]: sick_lidar_localization::UDPSender: udp payload EncoderMeasurementPayload0202:{telegram_count:1000003, timestamp:123456781, code:123456789} sent to 192.168.0.1:5009
[ INFO] [1628159034.731210064]: sick_lidar_localization::UDPSender: udp payload CodeMeasurementPayload0303:{telegram_count:1000004, timestamp:123456782, code:1234} sent to 192.168.0.1:5009
[ INFO] [1628159039.052258683]: sick_lidar_localization::UDPSender: udp payload LineMeasurementPayload0403:{telegram_count:1000005, timestamp:123456783, num_lanes:1, lanes:[1234]} sent to 192.168.0.1:5009
[ INFO] [1628159041.771734216]: sick_lidar_localization::UDPSender: udp payload LineMeasurementPayload0404:{telegram_count:1000006, timestamp:123456784, lcp1:12, lcp2:34, lcp3:56, cnt_lpc:78} sent to 192.168.0.1:5009
```

Use `ros2 topic pub` on ROS-2. Examples to to send UDP input messages (odometry, encoder measurement, code measurement and line measurement message):

```
ros2 topic pub --once /localizationcontroller/in/odometry_message_0104            sick_lidar_localization/OdometryMessage0104           '{telegram_count: 1000001, timestamp: 123456789, x_velocity: -1234, y_velocity: -1234, angular_velocity: 1234}'
ros2 topic pub --once /localizationcontroller/in/odometry_message_0105            sick_lidar_localization/OdometryMessage0105           '{telegram_count: 1000002, timestamp: 123456780, x_position: -1234, y_position: -1234, heading: 1234}'
ros2 topic pub --once /localizationcontroller/in/encoder_measurement_message_0202 sick_lidar_localization/EncoderMeasurementMessage0202 '{telegram_count: 1000003, timestamp: 123456781, encoder_value: 123456789}'
ros2 topic pub --once /localizationcontroller/in/code_measurement_message_0303    sick_lidar_localization/CodeMeasurementMessage0303    '{telegram_count: 1000004, timestamp: 123456782, code: 1234}'
ros2 topic pub --once /localizationcontroller/in/line_measurement_message_0403    sick_lidar_localization/LineMeasurementMessage0403    '{telegram_count: 1000005, timestamp: 123456783, num_lanes: 1, lanes: [1234]}'
ros2 topic pub --once /localizationcontroller/in/line_measurement_message_0404    sick_lidar_localization/LineMeasurementMessage0404    '{telegram_count: 1000006, timestamp: 123456784, lcp1: 12, lcp2: 34, lcp3: 56, cnt_lpc: 78}'
```

### Configuration of UDP input messages

Parameters can be configured in the launch file [sick_lidar_localization.launch](../launch/sick_lidar_localization.launch). The following parameters are used for UDP input messages:

| **parameter name** | **default value** | **parameter type** | **description** |
|--------------------|-------------------|--------------------|-----------------|
| udp_ip_sim_input | "192.168.0.1" | string | IP address for input UDP messages, i.e. IP adress of the localization controller or "" for UDP broadcasts |
| udp_port_sim_input | 5009 | int | UDP port of input messages |
| odom_topic | "/odom" | string | Topic of ros odom messages or "" to deactivate |
| udp_sim_input_source_id | 21 | int | Source_id of UDP input messages, see notes below |
| ros_odom_to_udp_msg | 3 | int | Convert ros odom message to upd, see notes below |

Parameter `udp_sim_input_source_id` is an identifier of the odometry sender. This ID has to match the ID in the SIM configuration file. You can get SIM configuration file using Sopas Air: Log in as user service, download the configuration file and get parameter odometer/external/interface/sourceId. Use this sourceId for the parameter `udp_sim_input_source_id`.

Parameter `ros_odom_to_udp_msg` converts ros odom message to upd:
* ros_odom_to_udp_msg = 0: map velocity to OdometryPayload0101 (Type 1, Version 1, LidarLoc 1), 
* ros_odom_to_udp_msg = 1: map velocity to OdometryPayload0104 (Type 1, Version 4, LidarLoc 2),
* ros_odom_to_udp_msg = 2: map position to OdometryPayload0105 (Type 1, Version 5, LidarLoc 2),
* ros_odom_to_udp_msg = 3: map velocity to OdometryPayload0104 and position to OdometryPayload0105
