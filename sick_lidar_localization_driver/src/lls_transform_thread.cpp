/*
 * @brief lls_loc_pointcloud_converts lls_loc_driver messages (type sick_lidar_localization::LocResultPortTelegramMsg),
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
#include <math.h>

#include "sick_lidar_localization/sick_common.h"
#include "sick_lidar_localization/lls_transform.h"

/*!
 * Constructor
 * @param[in] nh ros node handle
 */
sick_lidar_localization::LLSTransformer::LLSTransformer(rosNodePtr nh)
: m_nh(nh), m_converter_thread_running(false), m_converter_thread(0)
{
  if(nh)
  {
    m_tf_parent_frame_id = "map";
    m_tf_child_frame_id = "lls";
    rosGetParam(nh, "tf_parent_frame_id", m_tf_parent_frame_id);
    rosGetParam(nh, "tf_child_frame_id", m_tf_child_frame_id);
    // m_marker_frame_id = "map";
    // std::string marker_topic = "lls/marker";
    // rosGetParam(nh, "marker_topic", marker_topic);
    // rosGetParam(nh, "marker_frame_id", m_marker_frame_id);
    // m_marker_arrow_length = 1.0;
    // m_marker_arrow_width = 0.1;
    // m_marker_arrow_height = 0.1;
    // m_marker_color_r = 1.0;
    // m_marker_color_g = 1.0;
    // m_marker_color_b = 1.0;
    // m_marker_color_a = 0.5;
    // rosGetParam(nh, "marker_arrow_length", m_marker_arrow_length);
    // rosGetParam(nh, "marker_arrow_width", m_marker_arrow_width);
    // rosGetParam(nh, "marker_arrow_height", m_marker_arrow_height);
    // rosGetParam(nh, "marker_color_r", m_marker_color_r);
    // rosGetParam(nh, "marker_color_g", m_marker_color_g);
    // rosGetParam(nh, "marker_color_b", m_marker_color_b);
    // rosGetParam(nh, "marker_color_a", m_marker_color_a);
    // m_marker_publisher = rosAdvertise<ros_visualization_msgs::MarkerArray>(nh, marker_topic);
    // m_point_cloud_publisher = rosAdvertise<ros_sensor_msgs::PointCloud2>(nh, "/map");
  }
}

/*!
 * Destructor
 */
sick_lidar_localization::LLSTransformer::~LLSTransformer()
{
  stop();
}

/*!
 * Starts the converter thread, converts telegrams and publishes PointCloud2 messages.
 * @return true on success, false on failure.
 */
bool sick_lidar_localization::LLSTransformer::start(void)
{
  m_converter_thread_running = true;
  m_converter_thread = new std::thread(&sick_lidar_localization::LLSTransformer::runLLSTransformerThreadCb, this);
  return true;
}

/*!
 * Stops the converter thread.
 * @return true on success, false on failure.
 */
bool sick_lidar_localization::LLSTransformer::stop(void)
{
  m_converter_thread_running = false;
  if(m_converter_thread)
  {
    sick_lidar_localization_msgs::LocalizationControllerResultMessage0502 empty_msg;
    m_result_port_telegram_fifo.push(empty_msg); // push empty telegram to interrupt converter thread waiting for notification
    m_converter_thread->join();
    delete(m_converter_thread);
    m_converter_thread = 0;
  }
  return true;
}

/*!
 * Callback for result telegram messages (LocResultPortTelegramMsg) from lls_loc_driver.
 * The received message is buffered by fifo m_result_port_telegram_fifo.
 * Message handling and evaluation is done in the converter thread.
 * @param[in] msg result telegram message (LocResultPortTelegramMsg)
 */
void sick_lidar_localization::LLSTransformer::messageCbResultPortTelegrams(const sick_lidar_localization_msgs::LocalizationControllerResultMessage0502 & msg)
{
  m_result_port_telegram_fifo.push(msg);
}

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
// std::vector<ros_geometry_msgs::Point> sick_lidar_localization::LLSTransformer::poseToDemoPoints(double posx, double posy, double yaw, double triangle_height)
// {
//   // Center point at (posx, posy)
//   std::vector<ros_geometry_msgs::Point> points;
//   ros_geometry_msgs::Point centerpoint;
//   centerpoint.x = posx;
//   centerpoint.y = posy;
//   centerpoint.z = 0;
//   points.push_back(centerpoint);
//   ros_geometry_msgs::Point cornerpoint = centerpoint;
//   // Add triangle corner points at (posx + dx, posy + dy)
//   cornerpoint.x = centerpoint.x + triangle_height * std::cos(yaw);
//   cornerpoint.y = centerpoint.y + triangle_height * std::sin(yaw);
//   points.push_back(cornerpoint);
//   cornerpoint.x = centerpoint.x + 0.5 * triangle_height * std::cos(yaw + M_PI_2);
//   cornerpoint.y = centerpoint.y + 0.5 * triangle_height * std::sin(yaw + M_PI_2);
//   points.push_back(cornerpoint);
//   cornerpoint.x = centerpoint.x + 0.5 * triangle_height * std::cos(yaw - M_PI_2);
//   cornerpoint.y = centerpoint.y + 0.5 * triangle_height * std::sin(yaw - M_PI_2);
//   points.push_back(cornerpoint);
//   return points;
// }

/*!
 * Converts the vehicle position from a result port telegram to PointCloud2 message with 4 points
 * (centerpoint plus 3 demo corner points showing a triangle).
 * @param[in] msg result telegram message (LocResultPortTelegramMsg)
 * @return PointCloud2 message
 */
// ros_sensor_msgs::PointCloud2 sick_lidar_localization::LLSTransformer::convertToPointCloud(const sick_lidar_localization::LocalizationControllerResultMessage0502 & msg)
// {
//   ros_sensor_msgs::PointCloud2 pointcloud_msg;
//   // Create center point (PoseX,PoseY) and 3 corner points of a triangle showing the orientation (PoseYaw)
//   // Note: This is just an example to demonstrate use of sick_lidar_localization and PointCloud2 messages!
//   std::vector<ros_geometry_msgs::Point> points = poseToDemoPoints(
//     1.0e-3 * msg.x, // x-position in meter
//     1.0e-3 * msg.y, // y-position in meter
//     1.0e-3 * msg.heading * M_PI / 180.0, // yaw angle in radians
//     0.6); // triangle_height in meter (demo only)
//   // set pointcloud header
//   pointcloud_msg.header.stamp = msg.header.stamp; // telegram timestamp
//   if(msg.sync_timestamp_valid) // software pll initialized: use system time of vehicle pose calculated from lidar ticks by software pll
//   {
//     pointcloud_msg.header.stamp.sec = msg.sync_timestamp_sec;
//     pointcloud_msg.header.stamp.NSEC = msg.sync_timestamp_nsec;
//   }
//   pointcloud_msg.header.frame_id = "map";
//   // pointcloud_msg.header.seq = 0; // no seq field in ROS2
//   // clear cloud data
//   pointcloud_msg.height = 0;
//   pointcloud_msg.width = 0;
//   pointcloud_msg.data.clear();
//   // set pointcloud field properties
//   int numChannels = 3; // "x", "y", "z"
//   std::string channelId[] = { "x", "y", "z" };
//   pointcloud_msg.height = 1;
//   pointcloud_msg.width = points.size(); // normally we have 4 points (center point and 3 corner points)
//   pointcloud_msg.is_bigendian = false;
//   pointcloud_msg.is_dense = true;
//   pointcloud_msg.point_step = numChannels * sizeof(float);
//   pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_msg.width;
//   pointcloud_msg.fields.resize(numChannels);
//   for (int i = 0; i < numChannels; i++)
//   {
//     pointcloud_msg.fields[i].name = channelId[i];
//     pointcloud_msg.fields[i].offset = i * sizeof(float);
//     pointcloud_msg.fields[i].count = 1;
//     pointcloud_msg.fields[i].datatype = ros_sensor_msgs::PointField::FLOAT32;
//   }
//   // set pointcloud data values
//   pointcloud_msg.data.resize(pointcloud_msg.row_step * pointcloud_msg.height);
//   float* pfdata = reinterpret_cast<float*>(&pointcloud_msg.data[0]);
//   for(size_t data_cnt = 0, point_cnt = 0; point_cnt < points.size(); point_cnt++)
//   {
//     pfdata[data_cnt++] = static_cast<float>(points[point_cnt].x);
//     pfdata[data_cnt++] = static_cast<float>(points[point_cnt].y);
//     pfdata[data_cnt++] = static_cast<float>(points[point_cnt].z);
//   }
//   return pointcloud_msg;
// }

/*!
 * Converts the vehicle pose from a result port telegram to a marker array.
 * @param[in] msg result telegram message (LocResultPortTelegramMsg)
 * @return marker array
 */
// ros_visualization_msgs::MarkerArray sick_lidar_localization::LLSTransformer::convertToMarker(const sick_lidar_localization::LocalizationControllerResultMessage0502 & msg)
// {
//   double posx = 1.0e-3 * msg.x; // x-position in meter
//   double posy = 1.0e-3 * msg.y; // y-position in meter
//   double yaw  = 1.0e-3 * msg.heading * M_PI / 180.0; // yaw angle in radians
//   ros_visualization_msgs::Marker marker_msg;
//   marker_msg.header.stamp = msg.header.stamp; // telegram timestamp
//   if(msg.sync_timestamp_valid) // software pll initialized: use system time of vehicle pose calculated from lidar ticks by software pll
//   {
//     marker_msg.header.stamp.sec = msg.sync_timestamp_sec;
//     marker_msg.header.stamp.NSEC = msg.sync_timestamp_nsec;
//   }
//   marker_msg.header.frame_id = m_marker_frame_id;
//   marker_msg.ns = "lls";
//   marker_msg.id = 1;
//   marker_msg.type = ros_visualization_msgs::Marker::ARROW;
//   marker_msg.scale.x = m_marker_arrow_length; // arrow length
//   marker_msg.scale.y = m_marker_arrow_width;  // arrow width
//   marker_msg.scale.z = m_marker_arrow_height; // arrow height
//   marker_msg.pose.position.x = posx;
//   marker_msg.pose.position.y = posy;
//   marker_msg.pose.position.z = 0.0;
//   tf2::Quaternion q;
//   q.setRPY(0, 0, yaw);
//   marker_msg.pose.orientation.x = q.x();
//   marker_msg.pose.orientation.y = q.y();
//   marker_msg.pose.orientation.z = q.z();
//   marker_msg.pose.orientation.w = q.w();
//   marker_msg.action = ros_visualization_msgs::Marker::ADD; // note: ADD == MODIFY
//   marker_msg.color.r = m_marker_color_r;
//   marker_msg.color.g = m_marker_color_g;
//   marker_msg.color.b = m_marker_color_b;
//   marker_msg.color.a = m_marker_color_a;
//   marker_msg.lifetime = rosDurationFromSec(0); // lifetime 0 indicates forever
//   ros_visualization_msgs::MarkerArray marker_array;
//   marker_array.markers.push_back(marker_msg);
//   return marker_array;
// }

/*!
 * Converts the vehicle pose from a result port telegram to a tf transform.
 * @param[in] msg result telegram message (LocResultPortTelegramMsg)
 * @return tf transform
 */
ros_geometry_msgs::TransformStamped sick_lidar_localization::LLSTransformer::convertToTransform(const sick_lidar_localization_msgs::LocalizationControllerResultMessage0502 & msg)
{
  double posx = 1.0e-3 * msg.x; // x-position in meter
  double posy = 1.0e-3 * msg.y; // y-position in meter
  double yaw  = 1.0e-3 * msg.heading * M_PI / 180.0; // yaw angle in radians
  ros_geometry_msgs::TransformStamped vehicle_transform;
  vehicle_transform.header.stamp = msg.header.stamp; // telegram timestamp
  if(msg.sync_timestamp_valid) // software pll initialized: use system time of vehicle pose calculated from lidar ticks by software pll
  {
    vehicle_transform.header.stamp.sec = msg.sync_timestamp_sec;
    vehicle_transform.header.stamp.NSEC = msg.sync_timestamp_nsec;
  }
  vehicle_transform.header.frame_id = m_tf_parent_frame_id;
  vehicle_transform.child_frame_id = m_tf_child_frame_id;
  vehicle_transform.transform.translation.x = posx;
  vehicle_transform.transform.translation.y = posy;
  vehicle_transform.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  vehicle_transform.transform.rotation.x = q.x();
  vehicle_transform.transform.rotation.y = q.y();
  vehicle_transform.transform.rotation.z = q.z();
  vehicle_transform.transform.rotation.w = q.w();
  return vehicle_transform;
}

/*!
 * Thread callback, pops received telegrams from the fifo buffer m_result_port_telegram_fifo,
 * converts the telegrams from type LocResultPortTelegramMsg to PointCloud2 and publishes
 * PointCloud2 messages. The vehicle pose is converted to a tf transform and broadcasted.
 */
void sick_lidar_localization::LLSTransformer::runLLSTransformerThreadCb(void)
{
  ROS_INFO_STREAM("LLSTransformer: converter thread for lls_loc_driver messages started");
#if defined __ROS_VERSION && __ROS_VERSION == 1
  tf2_ros::TransformBroadcaster tf_broadcaster;
#elif defined __ROS_VERSION && __ROS_VERSION == 2
  tf2_ros::TransformBroadcaster tf_broadcaster(m_nh);
#endif

  while(rosOk() && m_converter_thread_running)
  {
    // Wait for next telegram
    while(rosOk() && m_converter_thread_running && m_result_port_telegram_fifo.empty())
    {
      rosSleep(0.0001);
      m_result_port_telegram_fifo.waitForElement();
    }
    if(rosOk() && m_converter_thread_running && !m_result_port_telegram_fifo.empty())
    {
      sick_lidar_localization_msgs::LocalizationControllerResultMessage0502 telegram = m_result_port_telegram_fifo.pop();
      // Convert vehicle position from result telegram to PointCloud2
      // ros_sensor_msgs::PointCloud2 pointcloud_msg = convertToPointCloud(telegram);
      // rosPublish(m_point_cloud_publisher, pointcloud_msg);
      // Convert vehicle position from result telegram to marker
      // ros_visualization_msgs::MarkerArray marker_msg = convertToMarker(telegram);
      // rosPublish(m_marker_publisher, marker_msg);
      // Convert vehicle pose from result telegram to tf transform
      ros_geometry_msgs::TransformStamped tf2_vehicle_transform = convertToTransform(telegram);
      tf_broadcaster.sendTransform(tf2_vehicle_transform);
    }
  }
  ROS_INFO_STREAM("LLSTransformer: converter thread for lls_loc_driver messages finished");
}
