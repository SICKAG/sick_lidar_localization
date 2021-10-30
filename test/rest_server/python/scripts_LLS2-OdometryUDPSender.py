import socket
from struct import *
import time

# IP of localization controller
LLS_DEVICE_IP = "192.168.0.1" # "192.168.0.70"

# Input port specified in lidarloc_config.yml (communication/smopUdp/input/port)
CONFIG_UDP_PORT = 5009
# Source ID for odom messages specified in lidarloc_config.yml (vehicle/odometer/external/interface/sourceId)
CONFIG_ODOM_SOURCE_ID = 21 # 100

# Network Socket
global sock
# Telegram Count
global telegramCount
telegramCount = 0

#------------------------------------------------------------------------------------------

def sendOdometryTelegram(vx, vy, omega, ts):

    # Import socket
    global sock
    #Telegram counter
    global telegramCount
    
    # Odom Telegram Header Data
    magicWord = "MOPS"
    headerVersion = 2
    payloadLength = 24
    endianess = 1602
    msgType = 1
    msgTypeVersion = 4

    # Odom Telegram (Header + Payload)
    odom_telegram = pack(">4sHHHHHHQQhhi",
                    magicWord.encode("ASCII"),
                    headerVersion,
                    payloadLength,
                    endianess,
                    msgType,
                    msgTypeVersion,
                    CONFIG_ODOM_SOURCE_ID,
                    telegramCount,
                    ts,
                    vx,
                    vy,
                    omega)
    

    print('########### DATA ###################')
    print("MsgTypeVersion :", msgTypeVersion)
    print("SourceID :", CONFIG_ODOM_SOURCE_ID)
    print("Vx :", vx)
    print("Vy :", vy)
    print("Vomega: ", omega)

    # Send datagram
    sock.sendto(odom_telegram, (LLS_DEVICE_IP, CONFIG_UDP_PORT))

    # Increment telegram counter
    telegramCount = telegramCount + 1


def startSendingTelegrams():
    # Current and previous timestamp
    ts = 0
    prevts = 0
    # Velocity
    velX = 0
    velY = 0
    velAng = 0
    # Odom telegram frequency [in seconds]
    odom_freq = 0.025
    # Odom message loop
    while True:
        # Generate timestamp in [us]
        ts = int(time.time()*1000000)
        # Send odom telegram
        sendOdometryTelegram(velX,velY,velAng, ts)
        # Delta between telegram timestamps
        print("Delta ts: ", (ts  - prevts)/1000, " ms")
        # Update previous telegram timestamps
        prevts = ts
        # Sleep in order to reach 25ms odom telegram cycle time
        time.sleep(odom_freq - (time.time() % odom_freq))



def initilize():
    global sock

    print ("UDP target IP:", LLS_DEVICE_IP)
    print ("UDP target port:", CONFIG_UDP_PORT)

    # UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Start sending odometry messages
    startSendingTelegrams()


if __name__ == '__main__':
    initilize()

















