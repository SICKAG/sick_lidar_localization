import socket
import math 

import time
starttime=time.time()

# Network Socket
global sock

# IP and Port for UDP Telegrams (IP = IP of SIM1000FX/SIM1012)
UDP_IP = "192.168.0.1"
UDP_PORT = 3000

# Telegram counter
global telegramCount
telegramCount = 0

def to_hex(data): # replaces data.hex() in python2
    return ''.join(hex(ord(c))[2:] for c in data) # return data.hex(

# Function to ensure that number "decINT" is represented by "digits" bytes 
def toHexPad(decINT, digits):
    # Number of bits given the number of digits the hex string has (e.g. 4 Hex digits = 2 Byte = 16 Bit)
    numBits = digits*4
    # Convert to hex (considers also negative numbers) + remove "\x" prefix
    decINTHex = hex((decINT + (1 << numBits)) % (1 << numBits))[2:]
    # Ensure correct length of hex string
    return decINTHex.zfill(digits)

#------------------------------------------------------------------------------------------

def encodeOdometryTelegram(sourceid, telegramcounter, vx, vy, omega, ts):

    # ------------------------------------------------------------------------------------------------------------------------------------------------------
    #|                                      HEADER                                               |                      PAYLOAD                            |
    # ------------------------------------------------------------------------------------------------------------------------------------------------------
    # MagicWord         PayloadLength payloadType(endianess)  MsgType MsgTypeVersion  SourceID   |  telegramCounter   timestamp    Vx     Vy     Omega     |
    #  534d4f50             000C          0642                 0000        0000         0000     |      00000001      00000000    0001   FFFF   00000001   | 
    # ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    
    #---------------- HEADER ---------------------------
    # Magic Word
    magicWordInByte = 'SMOP' # .encode('utf-8')
    message = to_hex(magicWordInByte) # magicWordInByte.hex()
    # Payload length (2Byte) in hex (length = 12 byte here)
    message = message + toHexPad(16, 4)
    # Using big endian
    message = message +  toHexPad(1602, 4)
    # MsgType
    message = message +  toHexPad(1, 4)
    # MsgType Version 
    message = message +  toHexPad(1, 4)
    # SourceID 
    message = message +  toHexPad(sourceid, 4)

    #---------------- PAYLOAD ---------------------------

    # Add Telegram count
    message = message + toHexPad(telegramcounter, 8)
    # Timestamp [in ms]
    message = message + toHexPad(ts, 8)
    # Vx [in mm/s]
    message = message + toHexPad(vx, 4)
    # Vy [in mm/s] 
    message = message + toHexPad(vy, 4)
    # Omega [in mdeg/s]
    message = message + toHexPad(omega, 8)

    return message

def sendOdometryTelegram(vx, vy, omega, ts):

    # Import socket
    global sock
    #Telegram counter
    global telegramCount

    message = encodeOdometryTelegram(100, telegramCount, vx, vy, omega, ts)

    print('########### DECIMAL ###################')
    print("TelegramCount :",telegramCount)
    print("Timestamp :", ts)
    print("Vx :", vx)
    print("Vy :", vy)
    print("Vomega :", omega)

    print('########### HEX ###################')
    print(toHexPad(telegramCount, 8))
    print(toHexPad(ts, 8))
    print(toHexPad(vx, 4))
    print(toHexPad(vy, 4))
    print(toHexPad(omega, 8))

    # Convert to bytes object
    MESSAGEHEX = bytes.fromhex(message)
    print ("message (str):", MESSAGEHEX)
    print ("message (hex):", MESSAGEHEX.hex())

    # Send datagram
    sock.sendto(MESSAGEHEX, (UDP_IP, UDP_PORT))

    # Increment telegram counter
    telegramCount = telegramCount + 1


def startSendingTelegrams():

    # Timestamp of current telegram and previous telegram
    ts = 0
    prevts = 0
    # Linear and angular velocity from odometry sensor (all integer numbers !!!)
    vel_x = 200       # [mm/s]
    vel_y = 0       # [mm/s]
    vel_omega = 10000 # [mdeg/s]
    # Odometry measurements frequency
    freq = 0.025 # send every 25ms
    while True:
        ts = int(time.time()*1000) # [ms]
        sendOdometryTelegram(vel_x,vel_y,vel_omega, ts)
        # Delta between telegram timestamps
        #print("Delta ts: ", ts  - prevts, " ms")
        # Update previous telegram timestamps
        #prevts = ts
        # Sleep in order to reach odom telegram cycle time specified by "freq"
        time.sleep(freq - (time.time() % freq))



def initilize():
    global sock

    # Target IP and Port for Odometry UDP telegrams 
    print ("UDP target IP:", UDP_IP)
    print ("UDP target port:", UDP_PORT)

    # UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Start sending odometry messages
    startSendingTelegrams()


if __name__ == '__main__':
    initilize()