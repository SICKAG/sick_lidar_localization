# System setup and source Ids

This chapter describes the system setup, messages and source ids.

## Background: System setup, messages and source ids

System example: A unit (e.g. robot) has multiple sensors (e.g. a lidar, a code reader and 2 wheel encoders). The LLS device, for example a SIM1000FX, is used for localization. The sensors communicate with the localization device using sick_lidar_localization driver (e.g. ROS1 on Linux). To identify which sensor sent the received data the sourceIds are used.

Communication:
* The sensor ROS nodes communicate with ROS messages and services with sick_lidar_localization node
* sick_lidar_localization node communicates with SIM1000FX using UDP out of ROS
* UDP messages are sent from sick_lidar_localization to SIM1000FX and vice versa
* sick_lidar_localization node converts UDP into ROS messages and vice versa

Source Ids:
* Each sensor has to get a unique source_id assigned within the node (e.g. left wheel encoder: sourceId=150, right wheel encoder: sourceId=151). 
* All ROS messages of the topics and UDP messages of the sick_lidar_localization pkg shall contain those sourceIds correspondingly.
* All sourceIds must be configured in the LLS device Web Gui (SOPASair) accordingly.

Example:
![source_id_01.png](source_id_01.png)

## Configuration of source ids

The source ID 
* has to be set individually unique for each sensor in a corresponding node
* of external odometry (subscribing topic on ```/odom```) has to be set by the [launch file](../sick_lidar_localization_driver/launch/sick_lidar_localization.launch) parameter ``src_id_vehicle`` (see example 3).

### Examples

Example 1 (ROS1):
* Source ID of left wheel encoder: 150 (configured by customer)
* Source ID of right wheel encoder: 151 (configured by customer)
* Source ID is set individually for each wheel encoder in the ROS message
* LiDAR data is received by the LLS device (here SIM1000FXA)
* LLS sends an UDP telegram containing the calculated localization result
* The sick_lidar_localization node converts the received UDP message into a ROS message and calculates a transform for the TF tree. Both is sent to ROS master.
* Encoder data are received by the application node
* The application node converts the data into ROS messages and sends them to sick_lidar_localization node
* The sick_lidar_localization node converts the ROS messages into a UDP stream and sends it to the LLS device, which is calculatin the odometry out of it.

![sequenceDiagramROSUDPMessages.png](sequenceDiagramROSUDPMessages.png)

Example 2: Customized source ID configuration for 2 wheel encoders:
* Source ID of left wheel encoder: 150 (configured by customer)
* Source ID of right wheel encoder: 151 (configured by customer)
* Source ID is set individually for each wheel encoder in the ROS message
* ROS message EncoderMeasurementMessage0202 of left wheel encoder: source_id=150
   * sick_lidar_localization generates UDP input message type 2 version 2 with source_id=150
* ROS message EncoderMeasurementMessage0202 of right wheel encoder: source_id=151
   * sick_lidar_localization generates UDP input message type 2 version 2 with source_id=151

![sequenceDiagramSourceIdExample2.png](sequenceDiagramSourceIdExample2.png)

Example 3: A robot uses standard ROS odometry messages of type [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) to publish odometry data. Standard ROS odometry messages of type [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) do not contain a source id. In this case, the source ID must be configured in the launchfile:
* Source ID of odometry device: 1 (configured by launchfile), e.g.:
   `<arg name="src_id_vehicle" default="1"/>`
* A robot publishes its velocity using [ROS odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) messages
* sick_lidar_localization receives the [ROS odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) messages and generates UDP input message type 1 version 4 or/and 5 with source_id=1
* Standard ROS odometry messages of type [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) do not contain a source id. ROS [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) messages are supported by sick_lidar_localization; their source id is configured in the launchfile. Depending on launchfile parameter `ros_odom_to_udp_msg`, ROS [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) messages are converted to UDP input message type 1 version 4 and/or 5:

   * ros_odom_to_udp_msg = 1: [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) messages are converted to UDP input message type 1 version 4 (source id by launchfile, linear and angular velocity by ROS message [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))
   * ros_odom_to_udp_msg = 2: [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) messages are converted to UDP input message type 1 version 5 (source id by launchfile, position and heading by ROS message [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))
   * ros_odom_to_udp_msg = 3: [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) messages are converted to both UDP input message type 1 version 4 and 5
   * Default value for ros_odom_to_udp_msg is 3 (convert to both udp message type 1, version 4 and 5):
      ```
      <arg name="ros_odom_to_udp_msg" default="3"/>          
      <!-- Convert ros odom message to udp: -->
      <!-- 1 = map velocity to OdometryPayload0104 and send udp message type 1, version 4 -->
      <!-- 2 = map position to OdometryPayload0105 and send udp message type 1, version 5 -->
      <!-- 3 = map velocity to OdometryPayload0104 and position to OdometryPayload0105 and send both udp message type 1, version 4 and 5 -->
      ```
   * A customer does not need to know details about source ids - except: "The source id used in the SIM1000FX configuration and in the launchfile must be identical"

![sequenceDiagramSourceIdExample3.png](sequenceDiagramSourceIdExample3.png)

   