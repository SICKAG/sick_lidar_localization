import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()

    # sick_lidar_localization configuration
    os_localization_controller_ip_address = os.getenv('localization_controller_ip_address', '')
    sick_lidar_localization_parameters=[
            # Driver configuration. See Operation-Instruction-v1.1.0.241R.pdf, page 51, "IP port number and protocol" for default tcp ports.
            {'/sick_lidar_localization/driver/localization_controller_default_ip_address': "192.168.0.1"},                  # Default IP address "192.168.0.1" of the localization controller (if not otherwise set by parameter "localization_controller_ip_address")
            {'/sim_loc_driver/localization_controller_ip_address': "192.168.0.1"},                                          # IP address "192.168.0.1" of the localization controller (if not otherwise set by parameter "localization_controller_ip_address")
            {'/system/localization_controller_ip_address': "{}".format(os_localization_controller_ip_address)},             # IP address can optionally be set by system environment
            {'/sick_lidar_localization/driver/result_telegrams_tcp_port': 2201},                                            # TCP port number of the localization controller sending localization results. To transmit the localization results to the vehicle controller, the localization controller uses IP port number 2201 to send localization results in a single direction to the external vehicle controller.
            {'/sick_lidar_localization/driver/cola_telegrams_tcp_port':   2111},                                            # For requests and to transmit settings to the localization controller: IP port number 2111 and 2112 to send telegrams and to request data, SOPAS CoLa-A or CoLa-B protocols
            {'/sick_lidar_localization/driver/cola_binary': 0},                                                             # 0: send Cola-ASCII (default), 1: send Cola-Binary, 2: toggle between Cola-ASCII and Cola-Binary (test and development only!)
            {'/sick_lidar_localization/driver/tcp_connection_retry_delay': 1.0},                                            # Delay in seconds to retry to connect to the localization controller, default 1 second
            {'/sick_lidar_localization/driver/result_telegrams_topic': "/sick_lidar_localization/driver/result_telegrams"}, # ros topic to publish result port telegram messages (type SickLocResultPortTelegramMsg)
            {'/sick_lidar_localization/driver/result_telegrams_frame_id': "sick_lidar_localization"},                       # ros frame id of result port telegram messages (type SickLocResultPortTelegramMsg)
            {'/sick_lidar_localization/driver/diagnostic_topic': "/sick_lidar_localization/driver/diagnostic"},             # ros topic to publish diagnostic messages (type SickLocDiagnosticMsg)
            {'/sick_lidar_localization/driver/diagnostic_frame_id': "sick_lidar_localization"},                             # ros frame id of diagnostic messages (type SickLocDiagnosticMsg)
            {'/sick_lidar_localization/driver/monitoring_rate': 1.0},                                                       # frequency to monitor driver messages, once per second by default
            {'/sick_lidar_localization/driver/monitoring_message_timeout': 1.0},                                            # timeout for driver messages, shutdown tcp-sockets and reconnect after message timeout, 1 second by default
            # Configuration for time sync service
            {'/sick_lidar_localization/time_sync/cola_response_timeout':     10.0},                                         # Timeout in seconds for cola responses from localization controller
            {'/sick_lidar_localization/time_sync/software_pll_fifo_length':   7},                                           # Length of software pll fifo, default: 7
            {'/sick_lidar_localization/time_sync/time_sync_rate':             0.1},                                         # Frequency to request timestamps from localization controller using ros service "SickLocRequestTimestamp" and to update software pll, default: 0.1
            {'/sick_lidar_localization/time_sync/time_sync_initial_rate':     1.0},                                         # Frequency to request timestamps and to update software pll during initialization phase, default: 1.0 (LocRequestTimestamp queries every second)
            {'/sick_lidar_localization/time_sync/time_sync_initial_length':  10},                                           # Length of initialization phase with LocRequestTimestamps every second, default: 10 (i.e. 10 LocRequestTimestamp queries every second after start, otherwise LocRequestTimestamp queries every 10 seconds)
            # Pointcloud configuration
            {'/sick_lidar_localization/driver/point_cloud_topic': "/cloud"},                                                # ros topic to publish PointCloud2 data
            {'/sick_lidar_localization/driver/point_cloud_frame_id': "pointcloud_sick_lidar_localization"},                 # ros frame id of PointCloud2 messages
            {'/sick_lidar_localization/driver/tf_parent_frame_id': "tf_demo_map"},                                          # parent frame of tf messages of of vehicles pose (typically frame of the loaded map)
            {'/sick_lidar_localization/driver/tf_child_frame_id': "tf_sick_lidar_localization"},                            # child frame of tf messages of of vehicles pose
            # Configuration for cola service
            {'/sick_lidar_localization/time_sync/cola_response_timeout':     10.0},                                         # Timeout in seconds for cola responses from localization controller
            {'/sick_lidar_localization/driver/tcp_connection_retry_delay':    1.0},                                         # Delay in seconds to retry to connect to the localization controller, default 1 second
            # Odometry configuration
            {'/sick_lidar_localization/driver/odom_telegrams_udp_port':   3000},                                            # Udp port to send odom packages to the localization controller
            {'/sick_lidar_localization/driver/odom_topic':             "/odom"},                                            # ROS topic for odometry messages
            {'/sick_lidar_localization/driver/odom_telegrams_bigendian':   1},                                              # Send udp odometry telegrams big endian (true) or little endian (false)
            {'/sick_lidar_localization/driver/odom_telegrams_source_id': 100},                                              # SourceID of udp odometry telegrams, e.g. vehicle controller 1
            # sim_loc_driver_check configuration
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegrams_topic':  "/sick_lidar_localization/driver/result_telegrams"},          # ros topic to publish result port telegram messages (type SickLocResultPortTelegramMsg)
            {'/sick_lidar_localization/sim_loc_driver_check/message_check_frequency': 100.0},                                                       # frequency to check sim_loc_driver messages
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/header/seq':              0.0},                              # sequence ID, consecutively increasing ID, uint32, size:= 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/header/stamp':            0.0},                              # time stamp
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/header/frame_id':         "sick_lidar_localization"},        # frame id of ros header
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/telegram_header/MagicWord':        0x5349434B},              # Magic word SICK (0x53 0x49 0x43 0x4B). uint32, size:= 4 � UInt8 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/telegram_header/Length':           106},                     # Length of telegram incl. header, payload, and trailer. uint32, size:= UInt32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/telegram_header/PayloadType':      0x0642},                  # Payload type, 0x06c2 = Little Endian, 0x0642 = Big Endian. uint16, size:= UInt16 = 2 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/telegram_header/PayloadVersion':   1},                       # Version of PayloadType structure. uint16, size:= UInt16 = 2 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/telegram_header/OrderNumber':      0.0},                     # Order number of the localization controller. uint32, size:= UInt32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/telegram_header/SerialNumber':     0.0},                     # Serial number of the localization controller. uint32, size:= UInt32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/telegram_header/FW_Version':       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, # Software version of the localization controller. uint8[], size:= 20 � UInt8 = 20 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/telegram_header/TelegramCounter':  0.0},                     # Telegram counter since last start-up. uint32, size:= UInt32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/telegram_header/SystemTime':       0.0},                     # Not used. uint64, size:= NTP = 8 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/telegram_payload/ErrorCode':        0},                      # ErrorCode 0:= OK, ErrorCode 1:= UNKNOWNERROR. uint16, size:= UInt16 = 2 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/telegram_payload/ScanCounter':      0.0},                    # Counter of related scan data. uint32, size:= UInt32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/telegram_payload/Timestamp':        0.0},                    # Time stamp of the pose [ms]. The time stamp indicates the time at which the pose is calculated. uint32, size:= UInt32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/telegram_payload/PoseX':            -2147483648},            # Position X of the vehicle on the map in cartesian global coordinates [mm]. int32, size:= Int32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/telegram_payload/PoseY':            -2147483648},            # Position Y of the vehicle on the map in cartesian global coordinates [mm]. int32, size:= Int32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/telegram_payload/PoseYaw':          -360000},                # Orientation (yaw) of the vehicle on the map [mdeg]. int32, size:= Int32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/telegram_payload/Reserved1':        0.0},                    # Reserved. uint32, size:= UInt32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/telegram_payload/Reserved2':        -2147483648},            # Reserved. int32, size:= Int32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/telegram_payload/Quality':          0},                      # Quality of pose [0 � 100], 1 = bad pose quality, 100 = good pose quality. uint8, size:= UInt8 = 1 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/telegram_payload/OutliersRatio':    0},                      # Ratio of beams that cannot be assigned to the current reference map [%]. uint8, size:= UInt8 = 1 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/telegram_payload/CovarianceX':      0},                      # Covariance c1 of the pose X [mm^2]. int32, size:= Int32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/telegram_payload/CovarianceY':      0},                      # Covariance c5 of the pose Y [mm^2]. int32, size:= Int32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/telegram_payload/CovarianceYaw':    0},                      # Covariance c9 of the pose Yaw [mdeg^2]. int32, size:= Int32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/telegram_payload/Reserved3':        0.0},                    # Reserved. uint64, size:= UInt64 = 8 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/telegram_trailer/Checksum':         0},                      # CRC16-CCITT checksum. uint16, size:= UInt16 = 2 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/vehicle_time_delta': -1.0e308},                              # min. allowed time diff in seconds between vehicle time (system time from ticks by software pll) and ros::Time::now()
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_min_values/check_vehicle_time': False},                                 # we can't check the vehicle time in case of network errors
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/header/seq':              4294967296.0},                     # 4294967296 = 0xFFFFFFFF # sequence ID, consecutively increasing ID, uint32, size:= 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/header/stamp':            2147483648.2147483648},            # time stamp (max. 2 int values, each 2^31 = 2147483648)
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/header/frame_id':         "sick_lidar_localization"},        # frame id of ros header
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/telegram_header/MagicWord':        0x5349434B},              # Magic word SICK (0x53 0x49 0x43 0x4B). uint32, size:= 4 � UInt8 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/telegram_header/Length':           106},                     # Length of telegram incl. header, payload, and trailer. uint32, size:= UInt32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/telegram_header/PayloadType':      0x06c2},                  # Payload type, 0x06c2 = Little Endian, 0x0642 = Big Endian. uint16, size:= UInt16 = 2 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/telegram_header/PayloadVersion':   0xFFFF},                  # Version of PayloadType structure. uint16, size:= UInt16 = 2 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/telegram_header/OrderNumber':      4294967296.0},            # 4294967296 = 0xFFFFFFFF # Order number of the localization controller. uint32, size:= UInt32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/telegram_header/SerialNumber':     4294967296.0},            # 4294967296 = 0xFFFFFFFF # Serial number of the localization controller. uint32, size:= UInt32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/telegram_header/FW_Version':       [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]}, # Software version of the localization controller. uint8[], size:= 20 � UInt8 = 20 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/telegram_header/TelegramCounter':  4294967296.0},            # 4294967296 = 0xFFFFFFFF # Telegram counter since last start-up. uint32, size:= UInt32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/telegram_header/SystemTime':       18446744073709551616.0},  # 18446744073709551616 = 0xFFFFFFFFFFFFFFFF # Not used. uint64, size:= NTP = 8 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/telegram_payload/ErrorCode':        0xFFFF},                 # ErrorCode 0:= OK, ErrorCode 1:= UNKNOWNERROR. uint16, size:= UInt16 = 2 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/telegram_payload/ScanCounter':      4294967296.0},           # 4294967296 = 0xFFFFFFFF # Counter of related scan data. uint32, size:= UInt32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/telegram_payload/Timestamp':        4294967296.0},           # 4294967296 = 0xFFFFFFFF # Time stamp of the pose [ms]. The time stamp indicates the time at which the pose is calculated. uint32, size:= UInt32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/telegram_payload/PoseX':            2147483647},             # Position X of the vehicle on the map in cartesian global coordinates [mm]. int32, size:= Int32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/telegram_payload/PoseY':            2147483647},             # Position Y of the vehicle on the map in cartesian global coordinates [mm]. int32, size:= Int32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/telegram_payload/PoseYaw':          360000},                 # Orientation (yaw) of the vehicle on the map [mdeg]. int32, size:= Int32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/telegram_payload/Reserved1':        4294967296.0},           # 4294967296 = 0xFFFFFFFF          # Reserved. uint32, size:= UInt32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/telegram_payload/Reserved2':        2147483647},             # Reserved. int32, size:= Int32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/telegram_payload/Quality':          100},                    # Quality of pose [1 � 100], 1 = bad pose quality, 100 = good pose quality. uint8, size:= UInt8 = 1 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/telegram_payload/OutliersRatio':    100},                    # Ratio of beams that cannot be assigned to the current reference map [%]. uint8, size:= UInt8 = 1 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/telegram_payload/CovarianceX':      2147483647},             # Covariance c1 of the pose X [mm^2]. int32, size:= Int32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/telegram_payload/CovarianceY':      2147483647},             # Covariance c5 of the pose Y [mm^2]. int32, size:= Int32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/telegram_payload/CovarianceYaw':    2147483647},             # Covariance c9 of the pose Yaw [mdeg^2]. int32, size:= Int32 = 4 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/telegram_payload/Reserved3':        18446744073709551616.0}, # 18446744073709551616 = 0xFFFFFFFFFFFFFFFF # Reserved. uint64, size:= UInt64 = 8 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/telegram_trailer/Checksum':         0xFFFF},                 # CRC16-CCITT checksum. uint16, size:= UInt16 = 2 byte
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/vehicle_time_delta': 1.0e308},                               # max. allowed time diff in seconds between vehicle time (system time from ticks by software pll) and ros::Time::now()
            {'/sick_lidar_localization/sim_loc_driver_check/result_telegram_max_values/check_vehicle_time': False}                                  # we can't check the vehicle time in case of network errors
            # Initial result output configuration. Unless configuration is activated here, default values are used. -->
            # Uncomment and set result output configuration if required, otherwise SIM default configuration applies. -->
            # {'SickLocSetResultPort':         "2201"}, # LocSetResultPort: Set TCP-port for result output (default: 2201) -->
            # {'SickLocSetResultMode':         "0",     # LocSetResultMode: Set mode of result output (0=stream or 1=poll, default:0) -->
            # {'SickLocSetResultPoseEnabled':  "1",     # LocSetResultPoseEnabled: Disable (0) or enable (1) result output (default: 1, enabled) -->
            # {'SickLocSetResultEndianness':   "0",     # LocSetResultEndianness: Set endianness of result output (0 = big endian, 1 = little endian, default: 0) -->
            # {'SickLocSetResultPoseInterval': "1",     # LocSetResultPoseInterval: Set interval of result output (0-255, interval in number of scans, 1: result with each processed scan, default: 1) -->
            # {'SickLocRequestResultData':     "0",     # LocRequestResultData: If in poll mode, trigger sending the localization result of the next processed scan via TCP interface (default: 0) -->
    ]

    # Launch sim_loc_driver
    sim_loc_driver_node = Node(
        package='sick_lidar_localization',
        name = 'sim_loc_driver',
        node_executable='sim_loc_driver',
        output='screen',
        parameters=sick_lidar_localization_parameters
    )

    # Launch sim_loc_driver_check to monitor sim_loc_driver
    sim_loc_driver_check_node = Node(
        package='sick_lidar_localization',
        name = 'sim_loc_driver_check',
        node_executable='sim_loc_driver_check',
        output='screen',
        parameters=sick_lidar_localization_parameters
    )
    
    # Launch pointcloud_converter to convert and publish PointCloud2 messages
    pointcloud_converter_node = Node(
        package='sick_lidar_localization',
        name = 'pointcloud_converter',
        node_executable='pointcloud_converter',
        output='screen',
        parameters=sick_lidar_localization_parameters
    )
    
    ld.add_action(sim_loc_driver_node)
    ld.add_action(sim_loc_driver_check_node)
    ld.add_action(pointcloud_converter_node)
    return ld
