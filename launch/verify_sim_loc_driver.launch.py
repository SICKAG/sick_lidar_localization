import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()

    # sim_loc_test_server configuration. See Operation-Instruction-v1.1.0.241R.pdf, page 51, "IP port number and protocol" for default tcp ports. 
    verify_sim_loc_driver_parameters=[
            {'/sick_lidar_localization/driver/result_telegrams_topic': "/sick_lidar_localization/driver/result_telegrams"}, # default topic to publish result port telegram messages (type SickLocResultPortTelegramMsg)
            {'/sick_lidar_localization/test_server/result_testcases_topic':    "/sick_lidar_localization/test_server/result_testcases"} # ROS topic to publish testcases with result port telegrams (type SickLocResultPortTestcaseMsg)
    ]

    #  Launch verify_sim_loc_driver
    verify_sim_loc_driver_node = Node(
        package='sick_lidar_localization',
        name = 'verify_sim_loc_driver',
        node_executable='verify_sim_loc_driver',
        output='screen',
        parameters=verify_sim_loc_driver_parameters
    )
    
    ld.add_action(verify_sim_loc_driver_node)
    return ld
