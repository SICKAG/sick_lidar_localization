import math 
import os 
import rclpy
from rclpy.node import Node
import time
starttime=time.time()

import nav_msgs.msg

global telegramCount
telegramCount = 0

# convert yaw, pitch, roll to quaternion
def toQuaternion(yaw, pitch, roll): # yaw (Z), pitch (Y), roll (X)
    # Abbreviations for the various angular functions
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    # Quaternion q;
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return w, x, y, z

#------------------------------------------------------------------------------------------

def sendOdometryTelegram(node, pub, vx, vy, omega, x, y, heading):

    #Telegram counter
    global telegramCount

    odom_msg = nav_msgs.msg.Odometry()
    odom_msg.header.stamp = node.get_clock().now().to_msg()
    odom_msg.twist.twist.linear.x = vx / 1000
    odom_msg.twist.twist.linear.y = vy / 1000
    odom_msg.twist.twist.angular.z = math.pi * (omega / 1000.0) / 180.0
    odom_msg.pose.pose.position.x = x / 1000
    odom_msg.pose.pose.position.y = y / 1000
    odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z = toQuaternion(math.pi * (heading / 1000.0) / 180.0, 0, 0)
    pub.publish(odom_msg)

    # Increment telegram counter
    telegramCount = telegramCount + 1

def startSendingTelegrams(node, pub):

    # Send a few telegram
    ts = 0
    prevts = 0
    angVel = -10000
    heading = 0
    delta_heading = -10000 * 0.025 # 10 degree per second
    while telegramCount < 300:
        if (telegramCount % 200) == 0:
            angVel = -angVel
            delta_heading = -delta_heading
        ts = int(time.time()*1000)
        sendOdometryTelegram(node, pub, 200, 0, angVel, 2000, 1000, heading)
        heading = heading + delta_heading
        # Delta between telegram timestamps
        print("Delta ts: ", ts  - prevts, " ms")
        # Update previous telegram timestamps
        prevts = ts
        # Sleep in order to reach 25ms odom telegram cycle time
        time.sleep(0.025 - (time.time() % 0.025))

def initialize():

    rclpy.init()
    node = Node("sendodom")
    pub = node.create_publisher(nav_msgs.msg.Odometry, "/odom", 10)

    # Start sending odometry messages
    startSendingTelegrams(node, pub)

if __name__ == '__main__':
    initialize()
    