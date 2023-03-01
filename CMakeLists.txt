cmake_minimum_required(VERSION 3.5)
project(sick_lidar_localization)

#####################
## Compile options ##
#####################

if(ROS_VERSION EQUAL 2)
    set(ROS2_HUMBLE 1)
    if(WIN32 OR EXISTS "/opt/ros/eloquent" OR EXISTS "/opt/ros/foxy" OR EXISTS "/opt/ros/galactic") # rosidl_typesupport for ROS2 eloquent, foxy, galaxy # use $ENV{ROS_DISTRO} instead
        set(ROS2_HUMBLE 0)
    endif()
endif()

## Compile as C++14
set(CMAKE_CXX_STANDARD 14)
if(WIN32)
    add_compile_options(-D_CRT_SECURE_NO_WARNINGS)
    set(WIN_LIBRARIES Ws2_32)
else()
    if(ROS2_HUMBLE) # ROS2 humble or later require C++17
        add_compile_options(-std=c++17)
    else()
        add_compile_options(-std=c++14)
    endif()
    add_compile_options(-g -Wall -Wno-reorder -Wno-sign-compare -Wno-unused-local-typedefs -Wno-unused-parameter -Wno-unused-function -Wno-unused-result)
    # add_compile_options(-Wshadow) # Note: compiler option -Wshadow generates a lot of warnings in ros header files. Therefore it's deactivated by default, but can be usefull for development and testing.
    set(LINUX_LIBRARIES pthread)    # gcc maps std::thread to pthread, using std::thread requires linking with pthread
endif()

if("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    add_compile_options(-O3)
endif()
if(DEFINED ROS_VERSION)
    add_compile_options(-D__ROS_VERSION=${ROS_VERSION})
else()
    add_compile_options(-D__ROS_VERSION=0)
endif()

#######################
## Required packages ##
#######################

## ROS 1 resp. ROS2 packages
if(ROS_VERSION EQUAL 1)
    find_package(catkin REQUIRED COMPONENTS message_generation message_runtime roscpp rospy geometry_msgs nav_msgs sensor_msgs std_msgs tf)
elseif(ROS_VERSION EQUAL 2)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(tf2 REQUIRED)  
    find_package(tf2_ros REQUIRED)
    find_package(visualization_msgs REQUIRED)
endif() 

## packages required by sick_lidar_localization
find_package(CURL REQUIRED)    # install libcurl by running "sudo apt-get install libcurl-dev" (Linux) resp. "vcpkg install curl[tool]:x64-windows" (Windows)
find_package(jsoncpp REQUIRED) # install libjsoncpp by running "sudo apt-get install libjsoncpp-dev" (Linux) resp. "vcpkg install jsoncpp:x64-windows" (Windows)

###############################
## ROS messages and services ##
###############################

if(ROS_VERSION EQUAL 1) 
    file(GLOB MSG_FILES LIST_DIRECTORIES false RELATIVE ${PROJECT_SOURCE_DIR}/msg msg/*.msg)
    add_message_files(FILES ${MSG_FILES})
    file(GLOB SRV_FILES LIST_DIRECTORIES false RELATIVE ${PROJECT_SOURCE_DIR}/srv srv/*.srv)
    add_service_files(FILES ${SRV_FILES})
    generate_messages(DEPENDENCIES geometry_msgs nav_msgs sensor_msgs std_msgs)    
    catkin_package(INCLUDE_DIRS include LIBRARIES sick_localization_lib CATKIN_DEPENDS message_runtime roscpp rospy geometry_msgs nav_msgs sensor_msgs std_msgs)
elseif(ROS_VERSION EQUAL 2) 
    file(GLOB MSG_FILES LIST_DIRECTORIES false RELATIVE ${PROJECT_SOURCE_DIR} msg/*.msg)
    file(GLOB SRV_FILES LIST_DIRECTORIES false RELATIVE ${PROJECT_SOURCE_DIR} srv/*.srv)
    rosidl_generate_interfaces(${PROJECT_NAME} ${MSG_FILES} ${SRV_FILES} DEPENDENCIES builtin_interfaces std_msgs)
endif() 

###########
## Build ##
###########

## include path
include_directories(include include/tinyxml ${catkin_INCLUDE_DIRS} ${CURL_INCLUDE_DIRS})

## sick_lidar_localization library
if(ROS_VERSION EQUAL 2) # AND WIN32)
    set(LIB_TYPE STATIC)
endif()
add_library(sick_localization_lib ${LIB_TYPE}
    src/curl_wrapper.cpp
    src/json_parser.cpp
    src/launchparser/launchparser.cpp
    src/sick_lidar_localization.cpp
    src/sick_ros_wrapper.cpp
    src/sick_services.cpp
    src/SoftwarePLL.cpp
    src/udp_message_parser.cpp
    src/udp_receiver_thread.cpp
    src/udp_sender.cpp
    src/tinyxml/tinystr.cpp
    src/tinyxml/tinyxml.cpp
    src/tinyxml/tinyxmlerror.cpp
    src/tinyxml/tinyxmlparser.cpp    
)
target_link_libraries(sick_localization_lib ${CURL_LIBRARIES} jsoncpp_lib ${catkin_LIBRARIES} ${LINUX_LIBRARIES} ${WIN_LIBRARIES})

## Executables
add_executable(sick_lidar_localization_main src/sick_lidar_localization_main.cpp)
set_target_properties(sick_lidar_localization_main PROPERTIES OUTPUT_NAME sick_lidar_localization)
target_link_libraries(sick_lidar_localization_main sick_localization_lib ${CURL_LIBRARIES} jsoncpp_lib ${catkin_LIBRARIES})

add_executable(gen_service_call src/gen_service_call.cpp)
target_link_libraries(gen_service_call sick_localization_lib ${CURL_LIBRARIES} jsoncpp_lib ${catkin_LIBRARIES})

if(ROS_VERSION GREATER 0)
    add_executable(lls_transform src/lls_transform.cpp src/lls_transform_thread.cpp)
    target_link_libraries(lls_transform sick_localization_lib ${CURL_LIBRARIES} jsoncpp_lib ${catkin_LIBRARIES})
endif()     

if(ROS_VERSION EQUAL 1)
    add_dependencies(sick_localization_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    add_dependencies(sick_lidar_localization_main sick_localization_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    add_dependencies(lls_transform sick_localization_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
elseif(ROS_VERSION EQUAL 2)
    ament_target_dependencies(sick_localization_lib rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2_ros visualization_msgs)
    ament_target_dependencies(lls_transform rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2_ros visualization_msgs)
    if(ROS2_HUMBLE) # rosidl_typesupport for ROS2 humble or later
        rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
        target_link_libraries(sick_localization_lib "${cpp_typesupport_target}")
        target_link_libraries(lls_transform "${cpp_typesupport_target}")
    else()
        rosidl_target_interfaces(sick_localization_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")
        rosidl_target_interfaces(lls_transform ${PROJECT_NAME} "rosidl_typesupport_cpp")
    endif()
else()
    add_dependencies(sick_lidar_localization_main sick_localization_lib)
endif()     

#############
## Install ##
############# 

if(ROS_VERSION EQUAL 1)
    install(TARGETS sick_localization_lib sick_lidar_localization_main gen_service_call lls_transform ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h*")
    install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
elseif(ROS_VERSION EQUAL 2)
    ament_export_dependencies(sick_localization_lib rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2_ros)
    ament_export_include_directories(include/${PROJECT_NAME}/)
    ament_export_libraries(sick_localization_lib sick_lidar_localization_main)
    ament_package()
    # install(TARGETS sick_localization_lib sick_lidar_localization_main gen_service_call lls_transform DESTINATION lib)
    install(TARGETS sick_localization_lib sick_lidar_localization_main gen_service_call lls_transform DESTINATION lib/${PROJECT_NAME})
    install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
else() 
    install(DIRECTORY launch DESTINATION ${CMAKE_BINARY_DIR})
endif() 
