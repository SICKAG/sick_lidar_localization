cmake_minimum_required(VERSION 2.8.3)
project(sick_lidar_localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -g -Wall -Wno-reorder -Wno-sign-compare -Wno-unused-local-typedefs -Wno-unused-parameter)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        # cmake_modules # Note: find_package(catkin ... "cmake_modules" ...) causes cmake error "Target ... links to target "UUID::UUID", caused by some dependency issue with gazebo
        # see https://bitbucket.org/ignitionrobotics/ign-cmake/issues/40/target-uuid-uuid-does-not-exist for further details
        message_generation
        message_runtime
        roscpp
        rospy
        geometry_msgs
        sensor_msgs
        std_msgs
        tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Doxygen)
# find_package(OpenCV 3 REQUIRED)
# find_package(cmake_modules REQUIRED) # find_package(cmake_modules) required for TinyXML. find_package(catkin ... "cmake_modules" ...) causes issues by gazebo dependencies (see note above).
# find_package(TinyXML REQUIRED)
# find_package(cv_bridge REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
        FILES
        SickLocColaTelegramMsg.msg
        SickLocDiagnosticMsg.msg
        SickLocResultPortHeaderMsg.msg
        SickLocResultPortPayloadMsg.msg
        SickLocResultPortCrcMsg.msg
        SickLocResultPortTelegramMsg.msg
        SickLocResultPortTestcaseMsg.msg
)

## Generate services in the 'srv' folder
add_service_files(
        FILES
        SickLocColaTelegramSrv.srv
        SickLocRequestTimestampSrv.srv
        SickLocSetResultModeSrv.srv
        SickLocSetResultPoseIntervalSrv.srv
        SickLocStopAndSaveSrv.srv
        SickLocIsSystemReadySrv.srv
        SickLocSetPoseSrv.srv
        SickLocSetResultPortSrv.srv
        SickLocStartLocalizingSrv.srv
        SickLocStopSrv.srv
        SickLocRequestResultDataSrv.srv
        SickLocSetResultEndiannessSrv.srv
        SickLocSetResultPoseEnabledSrv.srv
        SickLocStateSrv.srv
        SickLocTimeSyncSrv.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        geometry_msgs
        sensor_msgs
        std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES sick_localization_lib
        CATKIN_DEPENDS message_runtime roscpp rospy geometry_msgs sensor_msgs std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        test/include
        ${catkin_INCLUDE_DIRS}
        ${BOOST_INCLUDE_DIRS}
        # ${TinyXML_INCLUDE_DIRS}
        # ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(sick_localization_lib
        src/client_socket.cpp
        src/cola_configuration.cpp
        src/cola_converter.cpp
        src/cola_parser.cpp
        src/cola_services.cpp
        src/cola_transmitter.cpp
        src/driver_check_thread.cpp
        src/driver_monitor.cpp
        src/driver_thread.cpp
        src/pointcloud_converter_thread.cpp
        src/random_generator.cpp
        src/result_port_parser.cpp
        src/SoftwarePLL.cpp
        src/testcase_generator.cpp
        src/time_sync_service.cpp
        src/utils.cpp
        src/crc/crc16ccitt_false.cpp
        )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
##
## Executables for ros driver sick_lidar_localization
add_executable(sim_loc_driver src/driver.cpp)
add_executable(sim_loc_driver_check src/driver_check.cpp)
add_executable(sim_loc_time_sync src/time_sync.cpp)
add_executable(pointcloud_converter src/pointcloud_converter.cpp)
add_executable(cola_service_node src/cola_service_node.cpp)

## Executables for test purposes
add_executable(sim_loc_test_server test/src/test_server.cpp test/src/test_server_thread.cpp)
add_executable(unittest_sim_loc_parser test/src/unittest_sim_loc_parser.cpp)
add_executable(verify_sim_loc_driver test/src/verify_sim_loc_driver.cpp test/src/verifier_thread.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(sick_localization_lib
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
add_dependencies(sim_loc_driver
        sick_localization_lib
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
add_dependencies(sim_loc_driver_check
        sick_localization_lib
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
add_dependencies(sim_loc_test_server
        sick_localization_lib
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
add_dependencies(sim_loc_time_sync
        sick_localization_lib
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
add_dependencies(pointcloud_converter
        sick_localization_lib
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
add_dependencies(cola_service_node
        sick_localization_lib
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
add_dependencies(unittest_sim_loc_parser
        sick_localization_lib
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
add_dependencies(verify_sim_loc_driver
        sick_localization_lib
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

## Specify libraries to link a library or executable target against
target_link_libraries(sick_localization_lib
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        )
target_link_libraries(sim_loc_driver
        sick_localization_lib
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        )
target_link_libraries(sim_loc_driver_check
        sick_localization_lib
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        )
target_link_libraries(sim_loc_test_server
        sick_localization_lib
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        )
target_link_libraries(sim_loc_time_sync
        sick_localization_lib
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        )
target_link_libraries(pointcloud_converter
        sick_localization_lib
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        )
target_link_libraries(cola_service_node
        sick_localization_lib
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        )
target_link_libraries(unittest_sim_loc_parser
        sick_localization_lib
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        )
target_link_libraries(verify_sim_loc_driver
        sick_localization_lib
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS sick_localization_lib sim_loc_driver sim_loc_driver_check sim_loc_test_server sim_loc_time_sync pointcloud_converter cola_service_node unittest_sim_loc_parser verify_sim_loc_driver
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
        launch
        yaml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

#############
## Doxygen ##
#############

if (DOXYGEN_FOUND)
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    add_custom_target(RunDoxygen
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            VERBATIM )
    message(STATUS "Doxygen found, doxygen documentation will be generated in folder ${DOXYGEN_OUT}")
else ()
    message(STATUS "Doxygen not found, doxygen documentation will not be generated.")
endif ()

