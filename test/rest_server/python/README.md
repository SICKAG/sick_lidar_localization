# Test scripts for sick_lidar_localization

Folder `test/rest_server/python` contains the following python test scripts for sick_lidar_localization:

* [lls_pcapng_player.py](lls_pcapng_player.py): plays a pcapng-file recorded by Wireshark and broadcasts udp messages to simulate a localization controller.<br/>
   Usage example: 
   ```
   python3 test/rest_server/python/lls_pcapng_player.py  --pcap_filename test/data/wireshark/20210816_lidarloc2_2.0.0.14R_moving.pcapng 
   ```

* [lls_udp_receiver.py](lls_udp_receiver.py): receives udp messages from sick_lidar_localization, parses the messages and prints received localization data.<br/>
   Usage example: 
   ```
   python3 test/rest_server/python/lls_udp_receiver.py --udp_port=5009
   ```

* [lls_udp_sender.py](lls_udp_sender.py): generates synthetical datagrams to emulate a local localization controller and sends corresponding UDP messages.<br/>
   Usage example: 
   ```
   python3 test/rest_server/python/lls_udp_sender.py --udp_port=5010 --udp_send_rate=30.0 --max_message_count=300
   ```

* [send_odometry_message_0104.py](send_odometry_message_0104.py): creates and publishes odometry messages type 1 version 4 (ROS-1)<br/>
   Usage example: 
   ```
   python test/rest_server/python/send_odometry_message_0104.py
   ```
   Alternatively or on ROS-2, odometry messages can be published e.g. by
   ```
   ros2 topic pub --once /localizationcontroller/in/odometry_message_0104 sick_lidar_localization/OdometryMessage0104 '{telegram_count: 1000001, timestamp: 123456789, source_id: 0, x_velocity: -1234, y_velocity: -1234, angular_velocity: 1234}'
   ```

* [send_odometry_message_0105.py](send_odometry_message_0105.py): creates and publishes odometry messages type 1 version 5 (ROS-1)<br/>
   Usage example: 
   ```
   python test/rest_server/python/send_odometry_message_0105.py
   ```
   Alternatively or on ROS-2, odometry messages can be published e.g. by
   ```
   ros2 topic pub --once /localizationcontroller/in/odometry_message_0105 sick_lidar_localization/OdometryMessage0105 '{telegram_count: 1000002, timestamp: 123456780, source_id: 0, x_position: -1234, y_position: -1234, heading: 1234}'
   ```

* [send_ros_odom_messages.py](send_ros_odom_messages.py): creates and publishes ROS messages of type `nav_msgs.msg.Odometry` (ROS-1)<br/>
   Usage example: 
   ```
   python test/rest_server/python/send_ros_odom_messages.py
   ```

* [send_ros2_odom_messages.py](send_ros2_odom_messages.py): creates and publishes ROS messages of type `nav_msgs.msg.Odometry` (ROS-2)<br/>
   Usage example: 
   ```
   python3 test/rest_server/python/send_ros2_odom_messages.py
   ```

* [sick_rest_server.py](sick_rest_server.py): implements a tiny rest server, responds to http get- and post-request for local offline tests.<br/>
   Usage example: 
   ```
   sudo python3 ../rest_server/python/sick_rest_server.py
   ```

See the bash- and cmd-scripts in folder `test/scripts` for complete usage examples.
