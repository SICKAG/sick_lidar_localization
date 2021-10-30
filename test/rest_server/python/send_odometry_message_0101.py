import math 
import os 
import rospy
import time
starttime=time.time()

from sick_lidar_localization.msg import OdometryMessage0101

global telegramCount
telegramCount = 0

#------------------------------------------------------------------------------------------

def sendOdometryTelegram(pub, vx, vy, omega, ts):

    #Telegram counter
    global telegramCount

    #odometry_msg_str = "{{telegram_count: {}, timestamp: {}, x_velocity: {}, y_velocity: {}, angular_velocity: {}}}".format(telegramCount, ts, vx, vy, omega)
    #command = "rostopic pub --once /localizationcontroller/in/odometry_message_0101 sick_lidar_localization/OdometryMessage0101 \"{}\"".format(odometry_msg_str)
    #print(command)    
    #os.system(command)
    
    odom_msg = OdometryMessage0101()
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
        ts = int(time.time()*1000)
        sendOdometryTelegram(pub, 200, 0, angVel, ts)
        #if angVel == 0:
        #    angVel=25000
        #else:
        #    angVel=0
        # Delta between telegram timestamps
        print("Delta ts: ", ts  - prevts, " ms")
        # Update previous telegram timestamps
        prevts = ts
        # Sleep in order to reach 25ms odom telegram cycle time
        time.sleep(0.025 - (time.time() % 0.025))

def initialize():

    rospy.init_node("send_odometry_message_0101", anonymous=True)
    pub = rospy.Publisher("/localizationcontroller/in/odometry_message_0101", OdometryMessage0101, queue_size=10)

    # Start sending odometry messages
    startSendingTelegrams(pub)

if __name__ == '__main__':
    initialize()