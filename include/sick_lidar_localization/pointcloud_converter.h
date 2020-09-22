/*
 * @brief sim_loc_pointcloud_converts sim_loc_driver messages (type sick_lidar_localization::SickLocResultPortTelegramMsg),
 * to PointCloud2 messages and publishes PointCloud2 messages on topic "/cloud".
 *
 * The vehicle poses (PoseX, PoseY, PoseYaw of result port telegrams) are transformed and published
 * by tf-messages with configurable parent and child frame id.
 *
 * It also serves as an usage example for sick_lidar_localization and shows how to use sick_lidar_localization
 * in a custumized application.
 *
 * Copyright (C) 2019 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2019 SICK AG, Waldkirch
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
 *     * Neither the name of SICK AG nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission
 *     * Neither the name of Ing.-Buero Dr. Michael Lehning nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission
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
 *      Authors:
 *         Michael Lehning <michael.lehning@lehning.de>
 *
 *  Copyright 2019 SICK AG
 *  Copyright 2019 Ing.-Buero Dr. Michael Lehning
 *
 */
#ifndef __SIM_LOC_POINT_CLOUD_CONVERTER_H_INCLUDED
#define __SIM_LOC_POINT_CLOUD_CONVERTER_H_INCLUDED

#include <list>
#include <string>

#include "sick_lidar_localization/ros_wrapper.h"
#include "sick_lidar_localization/fifo_buffer.h"
#include "sick_lidar_localization/result_port_parser.h"

namespace sick_lidar_localization
{
  /*!
   * class PointCloudConverter implements a thread to converts sim_loc_driver messages
   * (type sick_lidar_localization::SickLocResultPortTelegramMsg) to PointCloud2 messages
   * and publishes them on topic "/cloud".
   */
  class PointCloudConverter
  {
  public:
    
    /*!
     * Constructor
     * @param[in] nh ros node handle
     */
    PointCloudConverter(ROS::NodePtr nh = 0);
    
    /*!
     * Destructor
     */
    virtual ~PointCloudConverter();
    
    /*!
     * Starts the converter thread, converts telegrams and publishes PointCloud2 messages.
     * @return true on success, false on failure.
     */
    virtual bool start(void);
    
    /*!
     * Stops the converter thread.
     * @return true on success, false on failure.
     */
    virtual bool stop(void);
  
    /*!
     * Callback for result telegram messages (SickLocResultPortTelegramMsg) from sim_loc_driver.
     * The received message is buffered by fifo m_result_port_telegram_fifo.
     * Message handling and evaluation is done in the converter thread.
     * @param[in] msg result telegram message (SickLocResultPortTelegramMsg)
     */
    virtual void messageCbResultPortTelegrams(const sick_lidar_localization::SickLocResultPortTelegramMsg & msg);
    /*! ROS2 version of function messageCbResultPortTelegrams */
    virtual void messageCbResultPortTelegramsROS2(const std::shared_ptr<sick_lidar_localization::SickLocResultPortTelegramMsg> msg) { messageCbResultPortTelegrams(*msg); }
  
  protected:

    /*!
     * Creates and returns a list of 4 points from position and orientaion of a sensor:
     * the center point (PoseX,PoseY) and 3 corner points of a triangle showing the orientation (PoseYaw)
     * Note: This is just an example to show the position and orientation of a sensor by a list of points.
     * @param[in] posx x-position of sensor in meter
     * @param[in] posy y-position of sensor in meter
     * @param[in] yaw orientation (yaw) of sensor in radians
     * @param[in] triangle_height heigt of the virtual triangle in meter
     * @return list of 3D points
     */
    std::vector<geometry_msgs::Point> poseToDemoPoints(double posx, double posy, double yaw, double triangle_height);
    
    /*!
     * Converts the vehicle position from a result port telegram to PointCloud2 message with 4 points
     * (centerpoint plus 3 demo corner points showing a triangle).
     * @param[in] msg result telegram message (SickLocResultPortTelegramMsg)
     * @return PointCloud2 message
     */
    sensor_msgs::PointCloud2 convertToPointCloud(const sick_lidar_localization::SickLocResultPortTelegramMsg & msg);

    /*!
     * Converts the vehicle pose from a result port telegram to a tf transform.
     * @param[in] msg result telegram message (SickLocResultPortTelegramMsg)
     * @return tf transform
     */
    geometry_msgs::TransformStamped convertToTransform(const sick_lidar_localization::SickLocResultPortTelegramMsg & msg);
    
    /*!
     * Thread callback, pops received telegrams from the fifo buffer m_result_port_telegram_fifo,
     * converts the telegrams from type SickLocResultPortTelegramMsg to PointCloud2 and publishes
     * PointCloud2 messages. The vehicle pose is converted to a tf transform and broadcasted.
     */
    virtual void runPointCloudConverterThreadCb(void);

    /*
     * member data
     */

    ROS::NodePtr m_nh; ///< ROS node handle
    sick_lidar_localization::FifoBuffer<sick_lidar_localization::SickLocResultPortTelegramMsg, boost::mutex> m_result_port_telegram_fifo; ///< fifo buffer for result port telegrams from sim_loc_driver
    std::string m_point_cloud_frame_id;      ///< ros frame id of PointCloud2 messages, default: "sick_lidar_localization"
    std::string m_tf_parent_frame_id;        ///< parent frame of tf messages of of vehicles pose (typically frame of the loaded map)
    std::string m_tf_child_frame_id;         ///< child frame of tf messages of of vehicles pose
    sick_lidar_localization::PointCloud2MsgPublisher m_point_cloud_publisher;  ///< ros publisher for PointCloud2 messages
    bool m_converter_thread_running;         ///< true: m_verification_thread is running, otherwise false
    boost::thread* m_converter_thread;       ///< thread to verify sim_loc_driver
    
  }; // class PointCloudConverter
  
} // namespace sick_lidar_localization
#endif // __SIM_LOC_POINT_CLOUD_CONVERTER_H_INCLUDED
