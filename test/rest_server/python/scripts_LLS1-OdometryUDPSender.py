import socket
import math 

import time
starttime=time.time()

# Network Socket
global sock

# IP and Port for UDP Telegrams
UDP_IP = "192.168.0.1" # "192.168.1.1"
UDP_PORT = 5009

# Source ID from odom sender -> has to match the ID in the configuration file
SourceID = 21 # 31 

global telegramCount
telegramCount = 0

#Found on https://stackoverflow.com/questions/25239423/crc-ccitt-16-bit-python-manual-calculation
#Using preset 0xffff instead of 0 (like in edp)
class CRC16:
    def __init__(self):
        self.poly = 0x1021 #CCITT polynom of CRC16
        self.preset = 0xffff
        self._table = [ self._initial(i) for i in range(256)]

    def _initial(self, c):
        crc = 0
        c = c << 8
        for _ in range(8):
            if (crc ^ c) & 0x8000:
                crc = (crc << 1) ^ self.poly
            else:
                crc = crc << 1
            c = c << 1
        return crc
    
    def _update_crc(self, crc, c):
        cc = 0xff & c
        tmp = (crc >> 8) ^ cc
        crc = (crc << 8) ^ self._table[tmp & 0xff]
        crc = crc & 0xffff
        return crc

    def compute(self, msg):
        crc = self.preset
        for c in msg:
            crc = self._update_crc(crc, c)
        return crc



def toHexPad(decINT, digits):
    # Number of bits given the number of digits the hex string has (e.g. 4 Hex digits = 2 Byte = 16 Bit)
    numBits = digits*4
    # Convert to hex (considers also negative numbers) + remove "\x" prefix
    decINTHex = hex((decINT + (1 << numBits)) % (1 << numBits))[2:]
    # Ensure correct length of hex string
    return decINTHex.zfill(digits)

#------------------------------------------------------------------------------------------

def sendOdometryTelegram(vx, vy, omega, ts):

    # Import socket
    global sock
    #Telegram counter
    global telegramCount

    # ------------------------------------------------------------------------------------------------------------------------------------------------------
    #|                                      HEADER                                               |                      PAYLOAD                            |
    # ------------------------------------------------------------------------------------------------------------------------------------------------------
    # MagicWord         PayloadLength payloadType(endianess)  MsgType MsgTypeVersion  SourceID   |  telegramCounter   timestamp    Vx     Vy     Omega     |
    #  534d4f50             000C          0642                 0000        0000         0000     |      00000001      00000000    0001   FFFF   00000001   | 
    # ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    
    #---------------- HEADER ---------------------------
    # Magic Word
    magicWordInByte = 'SMOP'.encode('utf-8')
    message = magicWordInByte.hex()
    # Payload length (2Byte) in hex (length = 12 byte here)
    message = message + toHexPad(16, 4)
    # Using big endian
    message = message +  toHexPad(1602, 4)
    # MsgType
    message = message +  toHexPad(1, 4)
    # MsgType Version 
    message = message +  toHexPad(1, 4)
    # SourceID 
    message = message +  toHexPad(SourceID, 4)

    #---------------- PAYLOAD ---------------------------

    # Add Telegram count
    message = message + toHexPad(telegramCount, 8)
    # Timestamp [in ms]
    message = message + toHexPad(ts, 8)
    # Vx [in mm/s]
    message = message + toHexPad(vx, 4)
    # Vy [in mm/s] 
    message = message + toHexPad(vy, 4)
    # Omega [in mdeg/s]
    message = message + toHexPad(omega, 8)


    #print('########### DECIMAL ###################')
    #print(telegramCount)
    #print ("Timestamp :", ts)
    #print ("Vx :", vx)
    #print ("Vy :", vy)
    #print ("Vomega :", omega)

    #print('########### HEX ###################')
    #print(toHexPad(telegramCount, 8))
    #print(toHexPad(toHexPad(ts, 8))
    #print(toHexPad(vx, 4))
    #print(toHexPad(vy, 4))
    #print(toHexPad(omega, 8))
    

    # Compute CRC
    CRC = CRC16()
    crcIN = CRC.compute(bytes.fromhex(message))
    msgCRC16 = toHexPad(crcIN,4)
    #print(msgCRC16)

    #Append crc16 to message
    #message = message + msgCRC16
    #print ("message:", message)

    # Convert to bytes object
    MESSAGEHEX = bytes.fromhex(message)
    #print ("message:", MESSAGEHEX)

    # Send datagram
    sock.sendto(MESSAGEHEX, (UDP_IP, UDP_PORT))

    # Increment telegram counter
    telegramCount = telegramCount + 1


def startSendingTelegrams():

    # Send a few telegram
    #for i in range(0,5000,25):
    #   sendOdometryTelegram(1,0,-1, i)
    ts = 0
    prevts = 0
    angVel = 10000
    while True:
        ts = int(time.time()*1000)
        sendOdometryTelegram(200,0,angVel, ts)
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



def initilize():
    global sock

    print ("UDP target IP:", UDP_IP)
    print ("UDP target port:", UDP_PORT)

    # UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Start sending odometry messages
    startSendingTelegrams()


if __name__ == '__main__':
    initilize()