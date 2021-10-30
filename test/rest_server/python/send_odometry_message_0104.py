import math 
import os 
import rospy
import time
starttime=time.time()

from sick_lidar_localization.msg import OdometryMessage0104

global telegramCount
telegramCount = 0

#------------------------------------------------------------------------------------------

def sendOdometryTelegram(pub, vx, vy, omega, ts):

    #Telegram counter
    global telegramCount

    odom_msg = OdometryMessage0104()
    odom_msg.telegram_count = telegramCount
    odom_msg.timestamp = ts
    odom_msg.x_velocity = vx
    odom_msg.y_velocity = vy
    odom_msg.angular_velocity = omega
    pub.publish(odom_msg)

    # Increment telegram counter
    telegramCount = telegramCount + 1

def startSendingTelegrams(pub):

    # Send a few telegram
    ts = 0
    prevts = 0
    angVel = -10000
    while telegramCount < 300:
        if (telegramCount % 200) == 0:
            angVel = -angVel
        ts = int(time.time()*1000000)
        sendOdometryTelegram(pub, 200, 0, angVel, ts)
        # Delta between telegram timestamps
        print("Delta ts: ", ts  - prevts, " microsec")
        # Update previous telegram timestamps
        prevts = ts
        # Sleep in order to reach 25ms odom telegram cycle time
        time.sleep(0.025 - (time.time() % 0.025))

def initialize():

    rospy.init_node("send_odometry_message_0104", anonymous=True)
    pub = rospy.Publisher("/localizationcontroller/in/odometry_message_0104", OdometryMessage0104, queue_size=10)

    # Start sending odometry messages
    startSendingTelegrams(pub)

if __name__ == '__main__':
    initialize()
    