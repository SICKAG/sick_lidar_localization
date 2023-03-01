import math 
import os 
import rospy
import time
starttime=time.time()

from sick_lidar_localization.msg import OdometryMessage0105

global telegramCount
telegramCount = 0

#------------------------------------------------------------------------------------------

def sendOdometryTelegram(pub, x, y, heading, ts):

    #Telegram counter
    global telegramCount

    odom_msg = OdometryMessage0105()
    odom_msg.telegram_count = telegramCount
    odom_msg.timestamp = ts
    odom_msg.x_position = x
    odom_msg.y_position = y
    odom_msg.heading = heading
    pub.publish(odom_msg)

    # Increment telegram counter
    telegramCount = telegramCount + 1

def startSendingTelegrams(pub):

    # Send a few telegram
    ts = 0
    prevts = 0
    heading = 0
    delta_heading = 10000 * 0.025 # 10 degree per second
    while telegramCount < 300:
        if (telegramCount % 200) == 0:
            delta_heading = -delta_heading
        ts = int(time.time()*1000000)
        sendOdometryTelegram(pub, 2000, 1000, int(heading), ts)
        heading = heading + delta_heading
        # Delta between telegram timestamps
        print("Delta ts: ", ts  - prevts, " microsec")
        # Update previous telegram timestamps
        prevts = ts
        # Sleep in order to reach 25ms odom telegram cycle time
        time.sleep(0.025 - (time.time() % 0.025))

def initialize():

    rospy.init_node("send_odometry_message_0105", anonymous=True)
    pub = rospy.Publisher("/localizationcontroller/in/odometry_message_0105", OdometryMessage0105, queue_size=10)

    # Start sending odometry messages
    startSendingTelegrams(pub)

if __name__ == '__main__':
    initialize()
    