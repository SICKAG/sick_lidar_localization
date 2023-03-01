/*
 * @brief Wrapper for systemdependent API to Windows/Linux native, ROS-1 and ROS-2
 *
 * Use
 *   #include <sick_lidar_localization/sick_ros_wrapper.h>
 * for
 *   #include <ros/ros.h>
 *   #include <rclcpp/rclcpp.hpp>
 *
 * Copyright (C) 2021, Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2021, SICK AG, Waldkirch
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: 16.07.2021
 *
 *      Authors:
 *         Michael Lehning <michael.lehning@lehning.de>
 *
 */

#ifndef __SICK_ROS_WRAPPER_H_INCLUDED
#define __SICK_ROS_WRAPPER_H_INCLUDED

#define _USE_MATH_DEFINES
#include <chrono>
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <sstream>
#include <vector>

#if !defined __ROS_VERSION
#define __ROS_VERSION 0 // default: native Linux or Windows
#endif

#ifdef _MSC_VER
#  ifdef min
#    undef min
#  endif
#  ifdef max
#    undef max
#    endif
#endif
#define MAX std::max
#define MIN std::min

#ifdef __linux__
#endif

template <typename T> std::string paramToString(const std::vector<T>& param_value)
{
    std::stringstream s;
    s << param_value.size();
    return s.str();
}

template <typename T> std::string paramToString(const T& param_value)
{
    std::stringstream s;
    s << param_value;
    return s.str();
}

#if __ROS_VERSION == 0 // native Linux or Windows

typedef void* rosNodePtr; // always 0 since we have not ROS

template <typename T> void rosDeclareParam(rosNodePtr nh, const std::string& param_name, const T& param_value);
template <typename T> bool rosGetParam(rosNodePtr nh, const std::string& param_name, T& param_value);
template <typename T> void rosSetParam(rosNodePtr nh, const std::string& param_name, const T& param_value);

#define ROS_FATAL(...)  do{ fprintf(stderr,"FATAL: "); fprintf (stderr, __VA_ARGS__); fprintf (stderr,"\n"); }while(0)
#define ROS_ERROR(...)  do{ fprintf(stderr,"ERROR: "); fprintf (stderr, __VA_ARGS__); fprintf (stderr,"\n"); }while(0)
#define ROS_WARN(...)   do{ fprintf(stderr,"WARN : "); fprintf (stderr, __VA_ARGS__); fprintf (stderr,"\n"); }while(0)
#define ROS_INFO(...)   do{ fprintf(stdout,"INFO : "); fprintf (stderr, __VA_ARGS__); fprintf (stderr,"\n"); }while(0)
#define ROS_DEBUG(...)  do{ fprintf(stdout,"DEBUG: "); fprintf (stderr, __VA_ARGS__); fprintf (stderr,"\n"); }while(0)

#define ROS_FATAL_STREAM(msg) do{ std::cerr <<"FATAL: " << msg << std::endl; }while(0)
#define ROS_ERROR_STREAM(msg) do{ std::cerr <<"ERROR: " << msg << std::endl; }while(0)
#define ROS_WARN_STREAM(msg)  do{ std::cerr <<"WARN : " << msg << std::endl; }while(0)
#define ROS_INFO_STREAM(msg)  do{ std::cout <<"INFO : " << msg << std::endl; }while(0)
#define ROS_DEBUG_STREAM(msg) do{ std::cout <<"DEBUG: " << msg << std::endl; }while(0)

typedef std::chrono::time_point<std::chrono::system_clock> rosTime;
inline rosTime rosTimeNow(void) { return std::chrono::system_clock::now(); }
inline int64_t nanoseconds_since_epoch(const rosTime& time) { return std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count(); } // returns nanoseconds since epoch
inline uint32_t sec(const rosTime& time) { return (uint32_t)(nanoseconds_since_epoch(time) / 1000000000); }              // return seconds part of epoch time
inline uint32_t nsec(const rosTime& time) { return (uint32_t)(nanoseconds_since_epoch(time) - 1000000000 * sec(time)); } // return nanoseconds part of epoch time

inline bool rosOk(void) { return true; }

#elif __ROS_VERSION == 1 // ROS-1 (Linux only)

#include <ros/ros.h>
#include <geometry_msgs/Point.h> 
#include <geometry_msgs/TransformStamped.h> 
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h> 
#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>

typedef ros::NodeHandle* rosNodePtr;

#define ros_sensor_msgs sensor_msgs
#define ros_std_msgs std_msgs
#define ros_geometry_msgs geometry_msgs
#define ros_nav_msgs nav_msgs
#define ros_visualization_msgs visualization_msgs

template <typename T> void rosDeclareParam(rosNodePtr nh, const std::string& param_name, const T& param_value) { }
template <typename T> bool rosGetParam(rosNodePtr nh, const std::string& param_name, T& param_value) { return nh->getParam(param_name, param_value); }
template <typename T> void rosSetParam(rosNodePtr nh, const std::string& param_name, const T& param_value) { nh->setParam(param_name, param_value); }

typedef ros::Duration rosDuration;
typedef ros::Time rosTime;
inline rosTime rosTimeNow(void) { return ros::Time::now(); }
inline uint32_t sec(const rosTime& time) { return time.sec; }   // return seconds part of ros::Time
inline uint32_t nsec(const rosTime& time) { return time.nsec; } // return nanoseconds part of ros::Time
#define NSEC nsec

template <class T> class rosPublisher : public ros::Publisher
{
public:
    rosPublisher() : ros::Publisher() {}
    rosPublisher(ros::Publisher& _publisher) : ros::Publisher(_publisher) {}
};
template <typename T> rosPublisher<T> rosAdvertise(rosNodePtr nh, const std::string& topic, uint32_t queue_size = 10, int qos = 10)
{
    std::string topic2;
    if(topic.empty() || topic[0] != '/')
      topic2 = std::string("/") + topic;
    else
      topic2 = topic;
    ROS_INFO_STREAM("Publishing on topic \"" << topic2 << "\"");
    ros::Publisher publisher = nh->advertise<T>(topic2, queue_size);
    return rosPublisher<T>(publisher);
}
template <typename T> void rosPublish(rosPublisher<T>& publisher, const T& msg) { publisher.publish(msg); }
template <typename T> std::string rosTopicName(rosPublisher<T>& publisher) { return publisher.getTopic(); }

template <class T> class rosSubscriber : public ros::Subscriber
{
public:
    rosSubscriber() : ros::Subscriber() {}
    rosSubscriber(ros::Subscriber& _subscriber) : ros::Subscriber(_subscriber) {}
};
template <typename T, class M, class U> rosSubscriber<T> rosSubscribe(rosNodePtr nh, const std::string& topic, void(U::*cbfunction)(M), U* cbobject)
{
    ROS_INFO_STREAM("Subscribing to topic \"" << topic << "\"");
    ros::Subscriber subscriber = nh->subscribe(topic,1,cbfunction,cbobject);
    return rosSubscriber<T>(subscriber);
}

inline bool rosOk(void) { return ros::ok(); }
inline void rosSpin(rosNodePtr nh) { ros::spin(); }
inline void rosSpinOnce(rosNodePtr nh) { ros::spinOnce(); }
inline void rosShutdown(void) { ros::shutdown(); }
inline void rosSleep(double seconds) { ros::Duration(seconds).sleep(); }
inline rosDuration rosDurationFromSec(double seconds) { return rosDuration(seconds); }

template <class T> class rosServiceClient : public ros::ServiceClient
{
public:
    rosServiceClient() : ros::ServiceClient() {}
    template <class U> rosServiceClient(U& _client) : ros::ServiceClient(_client) {}
};
template <class T> class rosServiceServer : public ros::ServiceServer
{
public:
    rosServiceServer() : ros::ServiceServer() {}
    template <class U> rosServiceServer(U& _server) : ros::ServiceServer(_server) {}
};
#define ROS_CREATE_SRV_CLIENT(nh,srv,name) nh->serviceClient<srv>(name)
#define ROS_CREATE_SRV_SERVER(nh,srv,name,cbfunction,cbobject) nh->advertiseService(name,cbfunction,cbobject)

#elif __ROS_VERSION == 2 // ROS-2 (Linux or Windows)

#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time_source.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp> 
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/header.hpp> 
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h> 
// #include <visualization_msgs/msg/marker.hpp>
// #include <visualization_msgs/msg/marker_array.hpp>

typedef rclcpp::Node::SharedPtr rosNodePtr;

#define ros_sensor_msgs sensor_msgs::msg
#define ros_std_msgs std_msgs::msg
#define ros_geometry_msgs geometry_msgs::msg
#define ros_nav_msgs nav_msgs::msg
#define ros_visualization_msgs visualization_msgs::msg

#define RCLCPP_LOGGER         rclcpp::get_logger("sick_lidar_localization")
#define ROS_FATAL(msg)        RCLCPP_FATAL(RCLCPP_LOGGER,msg)
#define ROS_ERROR(msg)        RCLCPP_ERROR(RCLCPP_LOGGER,msg)
#define ROS_WARN(msg)         RCLCPP_WARN(RCLCPP_LOGGER,msg)
#define ROS_INFO(msg)         RCLCPP_INFO(RCLCPP_LOGGER,msg)
#define ROS_DEBUG(msg)        RCLCPP_DEBUG(RCLCPP_LOGGER,msg)
#define ROS_FATAL_STREAM(msg) RCLCPP_FATAL_STREAM(RCLCPP_LOGGER,msg)
#define ROS_ERROR_STREAM(msg) RCLCPP_ERROR_STREAM(RCLCPP_LOGGER,msg)
#define ROS_WARN_STREAM(msg)  RCLCPP_WARN_STREAM(RCLCPP_LOGGER,msg)
#define ROS_INFO_STREAM(msg)  RCLCPP_INFO_STREAM(RCLCPP_LOGGER,msg)
#define ROS_DEBUG_STREAM(msg) RCLCPP_DEBUG_STREAM(RCLCPP_LOGGER,msg)

template <typename T> void rosDeclareParam(rosNodePtr nh, const std::string& param_name, const T& param_value) { if(!nh->has_parameter(param_name)) nh->declare_parameter<T>(param_name, param_value); }
template <typename T> bool rosGetParam(rosNodePtr nh, const std::string& param_name, T& param_value)
{
    try
    {
        bool bRet = nh->get_parameter(param_name, param_value);
        ROS_DEBUG_STREAM("rosGetParam(" << param_name << "): " << paramToString(param_value) << ", " << typeid(param_value).name());
        return bRet;
    }
    catch(const std::exception& exc)
    {
        ROS_WARN_STREAM("## ERROR rosGetParam(" << param_name << ", " << paramToString(param_value) << ", " << typeid(param_value).name() << ") failed, exception " << exc.what());
    }
    return false;
}
template <typename T> void rosSetParam(rosNodePtr nh, const std::string& param_name, const T& param_value)
{
    try
    {
        ROS_DEBUG_STREAM("rosSetParam(" << param_name << "," << paramToString(param_value) << ", " << typeid(param_value).name() << ")");
        nh->set_parameter(rclcpp::Parameter(param_name, param_value));
    }
    catch(const std::exception& exc)
    {
        ROS_WARN_STREAM("## ERROR rosSetParam(" << param_name << ", " << paramToString(param_value) << ", " << typeid(param_value).name() << ") failed, exception " << exc.what());
    }
}

typedef rclcpp::Duration rosDuration;
typedef rclcpp::Time rosTime; // typedef builtin_interfaces::msg::Time rosTime;
inline rosTime rosTimeNow(void) { return rclcpp::Clock().now(); }
inline uint32_t sec(const rosTime& time) { return (uint32_t)(time.nanoseconds() / 1000000000); }              // return seconds part of rclcpp::Time
inline uint32_t nsec(const rosTime& time) { return (uint32_t)(time.nanoseconds() - 1000000000 * sec(time)); } // return nanoseconds part of rclcpp::Time
#define NSEC nanosec

template <class T> class rosPublisher : public rclcpp::Publisher<T>::SharedPtr
{
public:
    rosPublisher() : rclcpp::Publisher<T>::SharedPtr(0) {}
    template <class U> rosPublisher(U& _publisher) : rclcpp::Publisher<T>::SharedPtr(_publisher) {}
};
template <class T> rosPublisher<T> rosAdvertise(rosNodePtr nh, const std::string& topic, uint32_t queue_size = 10, rclcpp::QoS qos = rclcpp::SystemDefaultsQoS())
{
    ROS_INFO_STREAM("Publishing on topic \"" << topic << "\"");
    auto publisher = nh->create_publisher<T>(topic, qos);
    return rosPublisher<T>(publisher);
}
template <typename T> void rosPublish(rosPublisher<T>& publisher, const T& msg) { publisher->publish(msg); }
template <typename T> std::string rosTopicName(rosPublisher<T>& publisher) { return publisher->get_topic_name(); }

template <class T> class rosSubscriber : public rclcpp::Subscription<T>::SharedPtr
{
public:
    rosSubscriber() : rclcpp::Subscription<T>::SharedPtr(0) {}
    template <class U> rosSubscriber(U& _subscriber) : rclcpp::Subscription<T>::SharedPtr(_subscriber) {}
};
template <typename T, class M, class U> rosSubscriber<T> rosSubscribe(rosNodePtr nh, const std::string& topic, void(U::*cbfunction)(M), U* cbobject)
{
    ROS_INFO_STREAM("Subscribing to topic \"" << topic << "\"");
    auto subscriber = nh->create_subscription<T>(topic,10,std::bind(cbfunction,cbobject,std::placeholders::_1));
    return rosSubscriber<T>(subscriber);
}

inline bool rosOk(void) { return rclcpp::ok(); }
inline void rosSpin(rosNodePtr nh) { rclcpp::spin(nh); }
inline void rosSpinOnce(rosNodePtr nh) { rclcpp::spin_some(nh); }
inline void rosShutdown(void) { rclcpp::shutdown(); }
inline void rosSleep(double seconds) { rclcpp::sleep_for(std::chrono::nanoseconds((int64_t)(seconds * 1.0e9))); }
inline rosDuration rosDurationFromSec(double seconds) { return rosDuration(std::chrono::nanoseconds((int64_t)(seconds * 1.0e9))); }

template <class T> class rosServiceClient : public rclcpp::Client<T>::SharedPtr
{
public:
    rosServiceClient() : rclcpp::Client<T>::SharedPtr(0) {}
    template <class U> rosServiceClient(U& _client) : rclcpp::Client<T>::SharedPtr(_client) {}
};
template <class T> class rosServiceServer : public rclcpp::Service<T>::SharedPtr
{
public:
    rosServiceServer() : rclcpp::Service<T>::SharedPtr(0) {}
    template <class U> rosServiceServer(U& _server) : rclcpp::Service<T>::SharedPtr(_server) {}
};
#define ROS_CREATE_SRV_CLIENT(nh,srv,name) nh->create_client<srv>(name)
#define ROS_CREATE_SRV_SERVER(nh,srv,name,cbfunction,cbobject) nh->create_service<srv>(name,std::bind(cbfunction,cbobject,std::placeholders::_1,std::placeholders::_2))

#else

#error __ROS_VERSION undefined or unsupported, build with __ROS_VERSION 0, 1 or 2

#endif //__ROS_VERSION

#endif // __SICK_ROS_WRAPPER_H_INCLUDED
