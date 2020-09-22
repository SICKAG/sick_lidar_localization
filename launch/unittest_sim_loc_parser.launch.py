import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()

    # unittest_sim_loc_parser configuration
    unittest_sim_loc_parser_parameters=[
            {'/unittest_sim_loc_parser/number_result_port_testcases': 100} # number of random based result port testcases, default: 100 testcases
    ]

    #  Launch verify_sim_loc_driver
    unittest_sim_loc_parser_node = Node(
        package='sick_lidar_localization',
        name = 'unittest_sim_loc_parser',
        node_executable='unittest_sim_loc_parser',
        output='screen',
        parameters=unittest_sim_loc_parser_parameters
    )
    
    ld.add_action(unittest_sim_loc_parser_node)
    return ld
