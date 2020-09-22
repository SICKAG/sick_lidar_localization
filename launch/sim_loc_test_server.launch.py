import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()

    # sim_loc_test_server configuration. See Operation-Instruction-v1.1.0.241R.pdf, page 51, "IP port number and protocol" for default tcp ports. 
    os_demo_circles = os.getenv('sim_loc_test_server_demo_circles', '')
    os_error_simulation = os.getenv('sim_loc_test_server_error_simulation', '')
    sim_loc_test_server_parameters=[
            {'/sim_loc_test_server/demo_circles': False},                               # True: simulate a sensor moving in circles, False (default): create random based result port telegrams
            {'//sim_loc_test_server/error_simulation': False},                          # If true, test server simulates errors and communication failures
            {'/system/test_server/demo_circles': "{}".format(os_demo_circles)},         # 1: simulate a sensor moving in circles, 0 (default): create random based result port telegrams
            {'/system/test_server/error_simulation': "{}".format(os_error_simulation)}, # 0: false (no error simulation), if true, test server simulates errors and communication failures
            {'/sick_lidar_localization/test_server/result_telegrams_tcp_port': 2201},   # Default tcp port for sim_loc_test_server is 2201 (ip port number of the localization controller sending localization results)
            {'/sick_lidar_localization/test_server/cola_telegrams_tcp_port':   2111},   # For requests and to transmit settings to the localization controller: IP port number 2111 and 2112 to send telegrams and to request data, SOPAS CoLa-A or CoLa-B protocols
            {'/sick_lidar_localization/test_server/result_telegrams_rate':       10.0}, # Rate to generate and send result port telegrams
            {'/sick_lidar_localization/test_server/result_testcases_topic':    "/sick_lidar_localization/test_server/result_testcases"}, # ROS topic to publish testcases with result port telegrams (type SickLocResultPortTestcaseMsg)
            {'/sick_lidar_localization/test_server/result_testcases_frame_id': "result_testcases"} # ROS frame id of testcase messages (type SickLocResultPortTestcaseMsg)
    ]

    #  Launch sim_loc_test_server
    sim_loc_test_server_node = Node(
        package='sick_lidar_localization',
        name = 'sim_loc_test_server',
        node_executable='sim_loc_test_server',
        output='screen',
        parameters=sim_loc_test_server_parameters
    )
    
    ld.add_action(sim_loc_test_server_node)
    return ld
