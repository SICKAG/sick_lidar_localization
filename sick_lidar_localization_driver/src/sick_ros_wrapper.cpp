/*
 * @brief Wrapper for systemdependent API to Windows/Linux native, ROS-1 and ROS-2.
 *        Implements a global parameter server for targets not using ROS.
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
#include "sick_lidar_localization/sick_ros_wrapper.h"

#if __ROS_VERSION == 2 // ROS-2: Global logger for publishing to /rosout

static rclcpp::Logger g_sick_lidar_localization_logger = rclcpp::get_logger("sick_lidar_localization");

const rclcpp::Logger& sick_lidar_localization_logger()
{
    return g_sick_lidar_localization_logger;
}

void sick_lidar_localization_set_logger(const rclcpp::Logger& logger)
{
    g_sick_lidar_localization_logger = logger;
}

#elif __ROS_VERSION == 0 // native Linux or Windows

template <typename T> class ParamServer
{
public:
	template <typename U> void declareParam(const std::string& param_name, const U& param_value)
	{
		if (param_map.find(param_name) == param_map.end())
		{
			param_map[param_name] = param_value;
		}
	}
	template <typename U> bool getParam(const std::string& param_name, U& param_value)
	{
		if (param_map.find(param_name) != param_map.end())
		{
			param_value = param_map[param_name];
			return true;
		}
		return false;
	}
	template <typename U> void setParam(const std::string& param_name, U& param_value)
	{
		param_map[param_name] = param_value;
	}
protected:
	std::map<std::string, T> param_map;
};

class ParamServerList
{
public:
	static ParamServer<bool>* getParamServer(const bool& param_value) { return &s_bool_params; }
	static ParamServer<int>* getParamServer(const int& param_value) { return &s_int_params; }
	static ParamServer<float>* getParamServer(const float& param_value) { return &s_float_params; }
	static ParamServer<double>* getParamServer(const double& param_value) { return &s_double_params; }
	static ParamServer<std::string>* getParamServer(const std::string& param_value) { return &s_string_params; }

protected:
	static ParamServer<bool> s_bool_params;
	static ParamServer<int> s_int_params;
	static ParamServer<float> s_float_params;
	static ParamServer<double> s_double_params;
	static ParamServer<std::string> s_string_params;
};

ParamServer<bool> ParamServerList::s_bool_params;
ParamServer<int> ParamServerList::s_int_params;
ParamServer<float> ParamServerList::s_float_params;
ParamServer<double> ParamServerList::s_double_params;
ParamServer<std::string> ParamServerList::s_string_params;

template <> void rosDeclareParam(rosNodePtr nh, const std::string& param_name, const bool& param_value)
{
	ParamServerList::getParamServer(param_value)->declareParam(param_name, param_value);
}

template <> void rosDeclareParam(rosNodePtr nh, const std::string& param_name, const int& param_value)
{
	ParamServerList::getParamServer(param_value)->declareParam(param_name, param_value);
}

template <> void rosDeclareParam(rosNodePtr nh, const std::string& param_name, const float& param_value)
{
	ParamServerList::getParamServer(param_value)->declareParam(param_name, param_value);
}

template <> void rosDeclareParam(rosNodePtr nh, const std::string& param_name, const double& param_value)
{
	ParamServerList::getParamServer(param_value)->declareParam(param_name, param_value);
}

template <> void rosDeclareParam(rosNodePtr nh, const std::string& param_name, const std::string& param_value)
{
	ParamServerList::getParamServer(param_value)->declareParam(param_name, param_value);
}

template <> bool rosGetParam(rosNodePtr nh, const std::string& param_name, bool& param_value)
{
	return ParamServerList::getParamServer(param_value)->getParam(param_name, param_value);
}

template <> bool rosGetParam(rosNodePtr nh, const std::string& param_name, int& param_value)
{
	return ParamServerList::getParamServer(param_value)->getParam(param_name, param_value);
}

template <> bool rosGetParam(rosNodePtr nh, const std::string& param_name, float& param_value)
{
	return ParamServerList::getParamServer(param_value)->getParam(param_name, param_value);
}

template <> bool rosGetParam(rosNodePtr nh, const std::string& param_name, double& param_value)
{
	return ParamServerList::getParamServer(param_value)->getParam(param_name, param_value);
}

template <> bool rosGetParam(rosNodePtr nh, const std::string& param_name, std::string& param_value)
{
	return ParamServerList::getParamServer(param_value)->getParam(param_name, param_value);
}

template <> void rosSetParam(rosNodePtr nh, const std::string& param_name, const bool& param_value)
{
	ParamServerList::getParamServer(param_value)->setParam(param_name, param_value);
}

template <> void rosSetParam(rosNodePtr nh, const std::string& param_name, const int& param_value)
{
	ParamServerList::getParamServer(param_value)->setParam(param_name, param_value);
}

template <> void rosSetParam(rosNodePtr nh, const std::string& param_name, const float& param_value)
{
	ParamServerList::getParamServer(param_value)->setParam(param_name, param_value);
}

template <> void rosSetParam(rosNodePtr nh, const std::string& param_name, const double& param_value)
{
	ParamServerList::getParamServer(param_value)->setParam(param_name, param_value);
}

template <> void rosSetParam(rosNodePtr nh, const std::string& param_name, const std::string& param_value)
{
	ParamServerList::getParamServer(param_value)->setParam(param_name, param_value);
}

#endif // __ROS_VERSION == 0 // native Linux or Windows
