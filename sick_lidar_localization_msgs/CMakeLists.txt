cmake_minimum_required(VERSION 3.5)
project(sick_lidar_localization_msgs)

# Set ROS_VERSION from environment variable if not defined
if(NOT DEFINED ROS_VERSION AND DEFINED ENV{ROS_VERSION})
    set(ROS_VERSION $ENV{ROS_VERSION})
endif()

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
    find_package(catkin REQUIRED COMPONENTS message_generation message_runtime std_msgs)
elseif(ROS_VERSION EQUAL 2)
    find_package(ament_cmake REQUIRED)
    find_package(builtin_interfaces REQUIRED)
    find_package(rosidl_default_generators REQUIRED)

    find_package(std_msgs REQUIRED)
endif() 


###############################
## ROS messages and services ##
###############################

if(ROS_VERSION EQUAL 1) 
    file(GLOB MSG_FILES LIST_DIRECTORIES false RELATIVE ${PROJECT_SOURCE_DIR}/msg msg/*.msg)
    add_message_files(FILES ${MSG_FILES})
    file(GLOB SRV_FILES LIST_DIRECTORIES false RELATIVE ${PROJECT_SOURCE_DIR}/srv srv/*.srv)
    add_service_files(FILES ${SRV_FILES})
    generate_messages(DEPENDENCIES std_msgs)    
    catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
elseif(ROS_VERSION EQUAL 2) 
    file(GLOB MSG_FILES LIST_DIRECTORIES false RELATIVE ${PROJECT_SOURCE_DIR} msg/*.msg)
    file(GLOB SRV_FILES LIST_DIRECTORIES false RELATIVE ${PROJECT_SOURCE_DIR} srv/*.srv)
    rosidl_generate_interfaces(${PROJECT_NAME} ${MSG_FILES} ${SRV_FILES} DEPENDENCIES builtin_interfaces std_msgs)
endif()

###########
## Build ##
###########

if(ROS_VERSION EQUAL 2)
    if(ROS2_HUMBLE) # rosidl_typesupport for ROS2 humble or later
        rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    endif()
endif()

#############
## Install ##
#############

if(ROS_VERSION EQUAL 1)
    install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h*") 
elseif(ROS_VERSION EQUAL 2)
    ament_export_include_directories(include/${PROJECT_NAME}/)
    ament_export_dependencies(rosidl_default_runtime)
    ament_package()
endif() 