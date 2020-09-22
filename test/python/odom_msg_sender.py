#!/usr/bin/env python

# odometry message sender simulating a vehicle moving in circles.
# Source: https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf
# ROS1 usage:
# source /opt/ros/melodic/setup.bash
# source ./devel/setup.bash
# roslaunch sick_lidar_localization odom_msg_sender.launch
# ROS2 usage:
# source /opt/ros/eloquent/setup.bash
# pip3 install transformations
# python3 odom_msg_sender.py
# ros2 run rviz2 rviz2 -d ../config/rviz2_odom_msg_sender.rviz

import math
from math import sin, cos, pi
import os
import socket
import select
import sys
import nav_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped, Twist, Vector3
from Odometry_LLS import encodeOdometryTelegram

# Determine ROS1 or ROS2. Unfortunately their APIs are different.
if "ROS_VERSION" in os.environ:
  ROS_VERSION = int(os.environ["ROS_VERSION"])
else:
  ROS_VERSION = 2
  print("odom_msg_sender: environment ROS_VERSION not set, assuming ROS_VERSION=2")
print("odom_msg_sender: ROS_VERSION={}".format(ROS_VERSION))

if ROS_VERSION > 1:
  import rclpy as rospy
  import tf2_ros as tf2
  import transformations
else:
  import rospy
  import tf as tf2

def ros_ok():
  if ROS_VERSION > 1:
    return rospy.ok()
  else:
    return not rospy.is_shutdown()

def spin_once(node):
  if ROS_VERSION > 1:
    rospy.spin_once(node)
  else:
    pass

def now(node):
  if ROS_VERSION > 1:
    return node.get_clock().now()
  else:
    return rospy.Time.now()

def to_sec(dt):
  if ROS_VERSION > 1:
    return 1.0e-9 * dt.nanoseconds
  else:
    return dt.to_sec()

def to_millisec(timestamp):
  if ROS_VERSION > 1:
    return int(current_time.nanoseconds / 1000000)
  else:
    return int(1000 * current_time.secs + current_time.nsecs / 1000000)

def timestamp(current_time):
  if ROS_VERSION > 1:
    return current_time.to_msg()
  else:
    return current_time

def to_hex(data):
  if ROS_VERSION > 1:
    return data.hex()
  else:
    return ''.join(hex(ord(c))[2:].zfill(2) for c in data)

def quaternion_from_euler(roll,pitch,yaw):
  if ROS_VERSION > 1:
    # ROS2 does not include package tf2.transformations, therefore transformations from https://pypi.org/project/transformations/ are used.
    # Install with "pip3 install transformations". Note from https://pypi.org/project/transformations/:
    # Transformations.py is no longer actively developed and has a few known issues and numerical instabilities.
    # Quaternions w+ix+jy+kz are represented as [w, x, y, z]
    [w, x, y, z] = transformations.quaternion_from_euler(roll,pitch,yaw, "sxyz")
    return [x, y, z, w]
  else:
    return tf2.transformations.quaternion_from_euler(roll,pitch,yaw)
 
def sendTransform(odom_broadcaster, x, y, z, odom_quat, current_time, base_id, child_frame_id):
  if ROS_VERSION > 1:
    t = TransformStamped()
    t.header.stamp = timestamp(current_time)
    t.header.frame_id = base_id
    t.child_frame_id = child_frame_id
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = 0.0
    t.transform.rotation.x = odom_quat[0]
    t.transform.rotation.y = odom_quat[1]
    t.transform.rotation.z = odom_quat[2]
    t.transform.rotation.w = odom_quat[3]
    odom_broadcaster.sendTransform(t)
  else:
    odom_broadcaster.sendTransform((x, y, 0.), odom_quat, current_time, base_id, child_frame_id)


# main entry point of this script, runs odom_msg_sender
if __name__ == "__main__": 
  
  if ROS_VERSION > 1:
    rospy.init(args=sys.argv)
    node = rospy.create_node('odom_msg_sender')
    odom_pub = node.create_publisher(Odometry, 'odom', 10)
    odom_broadcaster = tf2.TransformBroadcaster(node)
    per_millisecond_rate = node.create_rate(1000)
  else:
    rospy.init_node('odom_msg_sender')
    node = None
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf2.TransformBroadcaster()
    per_millisecond_rate = rospy.Rate(1000.0)
  
  # upd receiver to verify udp telegrams from sim_loc_driver
  udp_receive_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
  udp_receive_socket.setblocking(False)
  udp_receive_socket.settimeout(1.0)
  udp_receive_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
  udp_receive_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  udp_receive_socket.bind(("127.0.0.1", 3000))
  # udp_receive_socket.bind(("0.0.0.0", 3000))
  
  # position and velocity of odom messages
  x = 0
  y = 0.0
  th = 0.0
  vx = 0.1
  vy = -0.1
  vth = 0.1
  
  # configuration
  send_rate = 20.0 # send at least 20 odometry messages per second
  exit_after = 500 # exit after 500 odometry messages
  
  telegramcounter = 0
  udp_verified_counter = 0
  current_time = now(node)
  last_time = now(node)
  
  while ros_ok() and telegramcounter < exit_after:
      current_time = now(node)
  
      # compute odometry in a typical way given the velocities of the robot
      dt = to_sec(current_time - last_time)
      delta_x = (vx * cos(th) - vy * sin(th)) * dt
      delta_y = (vx * sin(th) + vy * cos(th)) * dt
      delta_th = vth * dt
  
      x += delta_x
      y += delta_y
      th += delta_th
    
      # since all odometry is 6DOF we'll need a quaternion created from yaw
      odom_quat = quaternion_from_euler(0.0, 0.0, th)
    
      # first, we'll publish the transform over tf
      sendTransform(odom_broadcaster, x, y, 0.0, odom_quat, current_time, "base_link", "odom")
    
      # next, we'll publish the odometry message over ROS
      odom = Odometry()
      odom.header.stamp = timestamp(current_time)
      odom.header.frame_id = "odom"
    
      # set the position and orientation
      # odom.pose.pose = Pose((Point(x, y, 0.), Quaternion(*odom_quat))
      odom.pose.pose.position.x = x
      odom.pose.pose.position.y = y
      odom.pose.pose.position.z = 0.0
      odom.pose.pose.orientation.x = odom_quat[0]
      odom.pose.pose.orientation.y = odom_quat[1]
      odom.pose.pose.orientation.z = odom_quat[2]
      odom.pose.pose.orientation.w = odom_quat[3]
      
      # set the velocity
      odom.child_frame_id = "base_link"
      # odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
      odom.twist.twist.linear.x = vx
      odom.twist.twist.linear.y = vy
      odom.twist.twist.linear.z = 0.0
      odom.twist.twist.angular.x = 0.0
      odom.twist.twist.angular.y = 0.0
      odom.twist.twist.angular.z = vth
      
      # publish the message
      udp_telegram_received = False
      odom_pub.publish(odom)
      
      last_time = current_time
      print("odom_msg_sender: x={}, y={}, th={}, vx={}, vy={}, vth={}".format(x, y, th, vx, vy, vth))
      spin_once(node)
      
      # receive upd telegram from sim_loc_driver and check against odom message sent
      # (verify with encodeOdometryTelegram by Odometry_LLS.py)
      while(to_sec(now(node) - last_time) < 1.0/send_rate):
        spin_once(node)
        # receive udp telegram from sim_loc_driver with timeout
        if not udp_telegram_received and select.select([udp_receive_socket], [], [], 0.5/send_rate)[0]:
            udp_received = udp_receive_socket.recvfrom(1024)[0]
            udp_hex = to_hex(udp_received)
            udp_telegram_received = True
            # verify udp telegram
            telegramcounter = telegramcounter + 1
            ts = to_millisec(current_time)
            udp_expected = encodeOdometryTelegram(1, telegramcounter, int(round(vx * 1000)), int(round(vy * 1000)), int(round(vth * 1000 * 180 / pi)), ts)
            print("odom_msg_sender: received udp telegram {}".format(udp_hex))
            print("odom_msg_sender: expected udp telegram {}".format(udp_expected))
            if udp_hex == udp_expected:
              udp_verified_counter = udp_verified_counter + 1
              print("odom_msg_sender: ok, received and expected udp telegram are identical.")
            else:
              print("## ERROR odom_msg_sender: received and expected udp telegram NOT identical.")
        per_millisecond_rate.sleep()
      if not udp_telegram_received:
        print("## ERROR odom_msg_sender: udp telegram NOT received.")
      if telegramcounter == udp_verified_counter:
        print("odom_msg_sender: {} odom messages sent, {} udp telegrams received and verified, {} errors".format(telegramcounter, udp_verified_counter, telegramcounter - udp_verified_counter))
      else:
        print("## ERROR odom_msg_sender: {} odom messages sent, {} udp telegrams received and verified, {} errors".format(telegramcounter, udp_verified_counter, telegramcounter - udp_verified_counter))

