# UDP stream messages

LiDAR-LOC receives and sends messages from resp. to the LLS device using UDP. [output messages](#lls_output_messages) are messages sent from localization device to the local PC. [input messages](#lls_input_messages) are messages sent from the local PC to the localization device. On ROS-1 and ROS-2, these messages are converted from resp. to ROS messages. On native Linux and Windows systems, these messages can be processed using the [C++ API](../README.md#cpp_api).

## <a name="lls_output_messages"></a> output messages

output messages are:

* [Odometry messages type 1 version 4](../msg/OdometryMessage0104.msg)
* [Odometry messages type 1 version 5](../msg/OdometryMessage0105.msg)
* [Code measurement messages type 3 version 4](../msg/CodeMeasurementMessage0304.msg)
* [Line measurement messages type 4 version 3](../msg/LineMeasurementMessage0403.msg)
* [Line measurement messages type 4 version 4](../msg/LineMeasurementMessage0404.msg)
* [Localization result messages type 5 version 2](../msg/LocalizationControllerResultMessage0502.msg)

A detailed description of these messages is available in the operation manuals published on https://supportportal.sick.com/. 

ROS-1 examples to receive output messages (odometry, code measurement, line measurement and localization result message):

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
[ INFO] [1628157236.969082207]: sick_lidar_localization::UDPReceiverThread: message received: OdometryPayload0104:{telegram_count:1000000064, timestamp:12345678901234631, x_velocity:-20000, y_velocity:-10000, angular_velocity:-5000, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
[ INFO] [1628157236.969182035]: Message::InfoListener: MessageReceived: OdometryPayload0104:{telegram_count:1000000064, timestamp:12345678901234631, x_velocity:-20000, y_velocity:-10000, angular_velocity:-5000, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
[ INFO] [1628157236.969221612]: PublishMessagesListener::MessageReceived: OdometryPayload0104:{telegram_count:1000000064, timestamp:12345678901234631, x_velocity:-20000, y_velocity:-10000, angular_velocity:-5000, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
header: seq: 51 stamp: secs: 1628157236 nsecs: 969233201 frame_id: ''
telegram_count: 1000000064
timestamp: 12345678901234631
x_velocity: -20000
y_velocity: -10000
angular_velocity: -5000
sync_timestamp_sec: 1630099057
sync_timestamp_nsec: 333530902
sync_timestamp_valid: 1

[ INFO] [1628157237.004634426]: sick_lidar_localization::UDPReceiverThread: message received: OdometryPayload0105:{telegram_count:1000000066, timestamp:12345678901234633, x_position:30000, y_position:25000, heading:123123, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
[ INFO] [1628157237.004702891]: Message::InfoListener: MessageReceived: OdometryPayload0105:{telegram_count:1000000066, timestamp:12345678901234633, x_position:30000, y_position:25000, heading:123123, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
[ INFO] [1628157237.004739923]: PublishMessagesListener::MessageReceived: OdometryPayload0105:{telegram_count:1000000066, timestamp:12345678901234633, x_position:30000, y_position:25000, heading:123123, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
header: seq: 48 stamp: secs: 1628157237 nsecs:   4751669 frame_id: ''
telegram_count: 1000000066
timestamp: 12345678901234633
x_position: 30000
y_position: 25000
heading: 123123
sync_timestamp_sec: 1630099057
sync_timestamp_nsec: 333530902
sync_timestamp_valid: 1

[ INFO] [1628157237.140145475]: sick_lidar_localization::UDPReceiverThread: message received: CodeMeasurementPayload0304:{telegram_count:1000000071, timestamp:12345678901234638, code:12345, distance:67890, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
[ INFO] [1628157237.140227342]: Message::InfoListener: MessageReceived: CodeMeasurementPayload0304:{telegram_count:1000000071, timestamp:12345678901234638, code:12345, distance:67890, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
[ INFO] [1628157237.140264435]: PublishMessagesListener::MessageReceived: CodeMeasurementPayload0304:{telegram_count:1000000071, timestamp:12345678901234638, code:12345, distance:67890, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
header: seq: 48 stamp: secs: 1628157237 nsecs: 140293307 frame_id: ''
telegram_count: 1000000071
timestamp: 12345678901234638
code: 12345
distance: 67890
sync_timestamp_sec: 1630099057
sync_timestamp_nsec: 333530902
sync_timestamp_valid: 1

[ INFO] [1628157237.275969209]: sick_lidar_localization::UDPReceiverThread: message received: LineMeasurementPayload0403:{telegram_count:1000000076, timestamp:12345678901234643, num_lanes:2, lanes:[300,345], sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
[ INFO] [1628157237.276035522]: Message::InfoListener: MessageReceived: LineMeasurementPayload0403:{telegram_count:1000000076, timestamp:12345678901234643, num_lanes:2, lanes:[300,345], sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
[ INFO] [1628157237.276079245]: PublishMessagesListener::MessageReceived: LineMeasurementPayload0403:{telegram_count:1000000076, timestamp:12345678901234643, num_lanes:2, lanes:[300,345], sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
header: seq: 48 stamp: secs: 1628157237 nsecs: 276147870 frame_id: ''
telegram_count: 1000000076
timestamp: 12345678901234643
num_lanes: 2
lanes: [300, 345]
sync_timestamp_sec: 1630099057
sync_timestamp_nsec: 333530902
sync_timestamp_valid: 1

[ INFO] [1628157237.411670286]: sick_lidar_localization::UDPReceiverThread: message received: LineMeasurementPayload0404:{telegram_count:1000000081, timestamp:12345678901234648, lcp1:300, lcp2:345, lcp3:-6789, cnt_lpc:7, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
[ INFO] [1628157237.411735674]: Message::InfoListener: MessageReceived: LineMeasurementPayload0404:{telegram_count:1000000081, timestamp:12345678901234648, lcp1:300, lcp2:345, lcp3:-6789, cnt_lpc:7, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
[ INFO] [1628157237.411755766]: PublishMessagesListener::MessageReceived: LineMeasurementPayload0404:{telegram_count:1000000081, timestamp:12345678901234648, lcp1:300, lcp2:345, lcp3:-6789, cnt_lpc:7, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
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

[ INFO] [1628157237.547426524]: sick_lidar_localization::UDPReceiverThread: message received: LocalizationControllerResultPayload0502:{telegram_count:1000000086, timestamp:12345678901234653, x:1020304, y:-1020304, heading:123456, loc_status:10, map_match_status:90, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
[ INFO] [1628157237.547522015]: Message::InfoListener: MessageReceived: LocalizationControllerResultPayload0502:{telegram_count:1000000086, timestamp:12345678901234653, x:1020304, y:-1020304, heading:123456, loc_status:10, map_match_status:90, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
[ INFO] [1628157237.547622457]: PublishMessagesListener::MessageReceived: LocalizationControllerResultPayload0502:{telegram_count:1000000086, timestamp:12345678901234653, x:1020304, y:-1020304, heading:123456, loc_status:10, map_match_status:90, sync_timestamp_sec:1630099057, sync_timestamp_nsec:333530902, sync_timestamp_valid:1}
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

Use `ros2 topic echo` on ROS-2. Examples to receive output messages (odometry, code measurement, line measurement and localization result message):

```
ros2 topic echo /localizationcontroller/out/odometry_message_0104 &
ros2 topic echo /localizationcontroller/out/odometry_message_0105 &
ros2 topic echo /localizationcontroller/out/code_measurement_message_0304 &
ros2 topic echo /localizationcontroller/out/line_measurement_message_0403 &
ros2 topic echo /localizationcontroller/out/line_measurement_message_0404 &
ros2 topic echo /localizationcontroller/out/localizationcontroller_result_message_0502 &
```

### Configuration of output messages

Parameters can be configured in the launch file [sick_lidar_localization.launch](../launch/sick_lidar_localization.launch). The following parameters are used for output messages:

| **parameter name** | **default value** | **parameter type** | **description** |
|--------------------|-------------------|--------------------|-----------------|
| lls_output_ip | "" | string | IP address of your local machine (i.e. the receiver of stream messages) |
| lls_output_port | 5010 | int | port of output messages |
| udp_output_logfile | "" | string | Optional logfile for human readable output messages, or "" to disable logfile |

## <a name="lls_input_messages"></a> input messages

input messages are:

* [Odometry messages type 1 version 4](../msg/OdometryMessage0104.msg)
* [Odometry messages type 1 version 5](../msg/OdometryMessage0105.msg)
* [Encoder measurement messages type 2 version 2](../msg/EncoderMeasurementMessage0202.msg)
* [Code measurement messages type 3 version 3](../msg/CodeMeasurementMessage0303.msg)
* [Code measurement messages type 7 version 1](../msg/CodeMeasurementMessage0701.msg)
* [Line measurement messages type 4 version 3](../msg/LineMeasurementMessage0403.msg)
* [Line measurement messages type 4 version 4](../msg/LineMeasurementMessage0404.msg)
* [ROS odometry messages](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)

A detailed description of these messages is available in the operation manuals published on https://supportportal.sick.com/. 

ROS-1 examples to send input messages (odometry, encoder measurement, code measurement and line measurement message):

```
source ./install/setup.bash 
roslaunch sick_lidar_localization sick_lidar_localization.launch verbose:=1 &
rostopic pub --once /localizationcontroller/in/odometry_message_0104            sick_lidar_localization/OdometryMessage0104           '{telegram_count: 1000001, timestamp: 123456789, source_id: 0, x_velocity: -1234, y_velocity: -1234, angular_velocity: 1234}'
rostopic pub --once /localizationcontroller/in/odometry_message_0105            sick_lidar_localization/OdometryMessage0105           '{telegram_count: 1000002, timestamp: 123456780, source_id: 0, x_position: -1234, y_position: -1234, heading: 1234}'
rostopic pub --once /localizationcontroller/in/encoder_measurement_message_0202 sick_lidar_localization/EncoderMeasurementMessage0202 '{telegram_count: 1000003, timestamp: 123456781, source_id: 0, encoder_value: 123456789}'
rostopic pub --once /localizationcontroller/in/code_measurement_message_0303    sick_lidar_localization/CodeMeasurementMessage0303    '{telegram_count: 1000004, timestamp: 123456782, source_id: 0, code: 1234}'
rostopic pub --once /localizationcontroller/in/code_measurement_message_0701    sick_lidar_localization/CodeMeasurementMessage0701    '{telegram_count: 1000004, timestamp: 123456782, source_id: 0, code: "1234", x_position: -1234, y_position: -2345, heading: -3456}'
rostopic pub --once /localizationcontroller/in/line_measurement_message_0403    sick_lidar_localization/LineMeasurementMessage0403    '{telegram_count: 1000005, timestamp: 123456783, source_id: 0, num_lanes: 1, lanes: [1234]}'
rostopic pub --once /localizationcontroller/in/line_measurement_message_0404    sick_lidar_localization/LineMeasurementMessage0404    '{telegram_count: 1000006, timestamp: 123456784, source_id: 0, lcp1: 12, lcp2: 34, lcp3: 56, cnt_lpc: 78}'
```

Example output from `sick_lidar_localization`:

```
[ INFO] [1628159024.678353489]: sick_lidar_localization::UDPSender: payload OdometryPayload0104:{telegram_count:1000001, timestamp:123456789, x_velocity:-1234, y_velocity:-1234, angular_velocity:1234} sent to 192.168.0.1:5009
[ INFO] [1628159027.488724322]: sick_lidar_localization::UDPSender: payload OdometryPayload0105:{telegram_count:1000002, timestamp:123456780, x_position:-1234, y_position:-1234, heading:1234} sent to 192.168.0.1:5009
[ INFO] [1628159031.805828483]: sick_lidar_localization::UDPSender: payload EncoderMeasurementPayload0202:{telegram_count:1000003, timestamp:123456781, code:123456789} sent to 192.168.0.1:5009
[ INFO] [1628159034.731210064]: sick_lidar_localization::UDPSender: payload CodeMeasurementPayload0303:{telegram_count:1000004, timestamp:123456782, code:1234} sent to 192.168.0.1:5009
[ INFO] [1628159039.052258683]: sick_lidar_localization::UDPSender: payload LineMeasurementPayload0403:{telegram_count:1000005, timestamp:123456783, num_lanes:1, lanes:[1234]} sent to 192.168.0.1:5009
[ INFO] [1628159041.771734216]: sick_lidar_localization::UDPSender: payload LineMeasurementPayload0404:{telegram_count:1000006, timestamp:123456784, lcp1:12, lcp2:34, lcp3:56, cnt_lpc:78} sent to 192.168.0.1:5009
```

Use `ros2 topic pub` on ROS-2. Examples to to send input messages (odometry, encoder measurement, code measurement and line measurement message):

```
ros2 topic pub --once /localizationcontroller/in/odometry_message_0104            sick_lidar_localization/OdometryMessage0104           '{telegram_count: 1000001, timestamp: 123456789, source_id: 0, x_velocity: -1234, y_velocity: -1234, angular_velocity: 1234}'
ros2 topic pub --once /localizationcontroller/in/odometry_message_0105            sick_lidar_localization/OdometryMessage0105           '{telegram_count: 1000002, timestamp: 123456780, source_id: 0, x_position: -1234, y_position: -1234, heading: 1234}'
ros2 topic pub --once /localizationcontroller/in/encoder_measurement_message_0202 sick_lidar_localization/EncoderMeasurementMessage0202 '{telegram_count: 1000003, timestamp: 123456781, source_id: 0, encoder_value: 123456789}'
ros2 topic pub --once /localizationcontroller/in/code_measurement_message_0303    sick_lidar_localization/CodeMeasurementMessage0303    '{telegram_count: 1000004, timestamp: 123456782, source_id: 0, code: 1234}'
ros2 topic pub --once /localizationcontroller/in/code_measurement_message_0701    sick_lidar_localization/CodeMeasurementMessage0701    '{telegram_count: 1000004, timestamp: 123456782, source_id: 0, code: "1234", x_position: -1234, y_position: -2345, heading: -3456}'
ros2 topic pub --once /localizationcontroller/in/line_measurement_message_0403    sick_lidar_localization/LineMeasurementMessage0403    '{telegram_count: 1000005, timestamp: 123456783, source_id: 0, num_lanes: 1, lanes: [1234]}'
ros2 topic pub --once /localizationcontroller/in/line_measurement_message_0404    sick_lidar_localization/LineMeasurementMessage0404    '{telegram_count: 1000006, timestamp: 123456784, source_id: 0, lcp1: 12, lcp2: 34, lcp3: 56, cnt_lpc: 78}'
```

### Configuration of input messages

Parameters can be configured in the launch file [sick_lidar_localization.launch](../launch/sick_lidar_localization.launch). The following parameters are used for input messages:

| **parameter name** | **default value** | **parameter type** | **description** |
|--------------------|-------------------|--------------------|-----------------|
| lls_input_ip | "192.168.0.1" | string | IP address for input messages, i.e. IP address of the LLS device or "" for broadcasts |
| lls_input_port | 5009 | int | port of input messages |
| odom_topic | "/odom" | string | Topic of ros odom messages or "" to deactivate |
| lls_input_source_id | 21 | int | Source_id of input messages, see notes below |
| ros_odom_to_udp_msg | 3 | int | Convert ros odom message, see notes below |

Parameter `lls_input_source_id` is a default identifier of all input messages if source_id is not otherwise specified in the UDPSender. This ID has to match the ID in the LLS configuration file (lidarloc_config.yml). You can get configuration file using SOPASair: Log in as user service, download the configuration file and get parameter odometer/external/interface/sourceId. Use this sourceId for the parameter `lls_input_source_id`.
Note: In sick_lidar_localization version 5.5 or newer, the ID can be set in 3 different ways:
* The ID can be specified in each input message by its parameter `source_id`. If the source_id in the message is greater 0, it will be send to the localizer server. We recommend this setting.  
* If the source_id in the message is equal 0, a default ID from configuration file (launch-file) is used.
* This default source_id can be configured for different message types and message versions in [sick_lidar_localization.launch](../launch/sick_lidar_localization.launch) by 
   ```
   <param name="lls_input_source_id_<msgtype>_<msgversion>" type="int" value="21"/>   <!-- Source_id of input messages type <msgtype> version <msgversion> --> 
   ```
   e.g.
   ```
   <param name="lls_input_source_id_7_1" type="int" value="21"/>   <!-- Source_id of input messages type 7 version 1 --> 
   ```
* If the parameter `lls_input_source_id_<msgtype>_<msgversion>` is not configured, the default source id is given by parameter `lls_input_source_id`.

Parameter `ros_odom_to_udp_msg` converts ros odom message:
* ros_odom_to_udp_msg = 1: map velocity to OdometryPayload0104 (Type 1, Version 4, LidarLoc 2),
* ros_odom_to_udp_msg = 2: map position to OdometryPayload0105 (Type 1, Version 5, LidarLoc 2),
* ros_odom_to_udp_msg = 3: map velocity to OdometryPayload0104 and position to OdometryPayload0105
