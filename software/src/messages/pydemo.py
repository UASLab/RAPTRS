#!/usr/bin/python3

import time
import os

import fmu_messages


#%% Message Framing
# Every message should be framed (start and end) with 0x7E (b'~')
# The escape byte is 0x7D
# The invert byte for escape is 0x20
frameEdge = 0x7E; frameEdgeByte = bytes.fromhex('7E');
frameEscape = 0x7D; frameEscapeByte = bytes.fromhex('7D');
frameInvert = 0x20; frameInvertByte = bytes.fromhex('20');

def UnescapeMessage(msg):    
    msgUnEsc = b''
    escPrevFlag = False
    for char in msg:
        if not escPrevFlag:
            if char == frameEscape:
                escPrevFlag = True
            else:
                msgUnEsc += char.to_bytes(1, 'little')
        else:
            msgUnEsc += (char ^ frameInvert).to_bytes(1, 'little')
            escPrevFlag = False

    if escPrevFlag:
        raise ValueError('Invalid message, ends in escape byte')

    return msgUnEsc


def EscapeMessage(msg):    
    msgEsc = b''
    for char in msg:
        if char in (frameEdgeByte, frameEscapeByte):
            msgEsc += frameEscapeByte + (char ^ frameInvert).to_bytes(1, 'little')
        else:
            msgEsc += char.to_bytes(1, 'little')
            
    return msgEsc


#%% CRC
def crc16(data, crc=0):
    
    table = [
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
        0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
        0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
        0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
        0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
        0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
        0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
        0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
        0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
        0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
        0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
        0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
        0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
        0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
        0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
        0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
        0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
        0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
        0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
        0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
        0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
        0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
        0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
        ]
    
    for byte in data:
        crc = ((crc << 8) & 0xff00) ^ table[((crc >> 8) & 0xff) ^ byte]
    
    return crc & 0xffff


#%% SerialLink
import serial
    

class SerialLink():
    
    ser = serial.Serial()
    ser.timeout = 0
    readBufList = []
    
    def __init__ (self, port):
        self.ser.port = port
    
    def begin(self):
        # Open flush, probably overboard.
        if self.ser.isOpen():
            self.ser.flush()
            self.ser.close()
        self.ser.open()
        self.ser.flush()

    def write(self, msgID, msgAddress, msgPayload):
        
        # Join msgData as: Address + ID + Payload
        msgData = b''
        msgData += msgID.to_bytes(1, byteorder = 'little')
        msgData += msgAddress.to_bytes(1, byteorder = 'little')
        msgData += msgPayload
        
        # Compute CRC
        crcBytes = crc16(msgData).to_bytes(2, byteorder = 'little')
        
        # Message: Framed, Escaped, with CRC
        msgBytes = frameEdgeByte + EscapeMessage(msgData + crcBytes) + frameEdgeByte
        
        # Write to serial
        numBytes = self.ser.write(msgBytes)
        
        return numBytes
        
    
    def checkReceived(self):
        # Read everything out of the serial
        if (self.ser.in_waiting > 0):
            msgRecv = self.ser.read(self.ser.in_waiting)
        
            # multiple messages could have been read, Split on edge framing
            msgLines = msgRecv.split(frameEdgeByte)
            
             # Remove empty, append to readBufList
            [self.readBufList.append(msg) for msg in msgLines if b'' is not msg]
        
        return len(self.readBufList)
    
    
    def available(self):
        return len(self.readBufList)
    

    def read(self):
        
         # Pop the oldest message off front
        msgBytes = self.readBufList.pop(0)
        
        print('Raw Recv: ' + str(msgBytes))
        
        # Un-Escape the Message
        msgBytes = UnescapeMessage(msgBytes)
                
        # Check CRC
        msgData = msgBytes[:-2]
        msgCRC = int.from_bytes(msgBytes[-2:], byteorder = 'little') # Last 2 bytes as integer
    
        if (crc16(msgData) == msgCRC):
            # Split msgData into: ID + Address + Payload
            msgID = int.from_bytes(msgData[1:2], byteorder = 'little') # Message ID
            msgAddress = int.from_bytes(msgData[2:3], byteorder = 'little') # Address - FIXIT
            msgPayload = msgData[2:]
        else:
            print('\tCRC Fail: ' + str(msgBytes))
            msgID = -1
            msgAddress = -1
            msgPayload = b''
        
        return msgID, msgAddress, msgPayload
   

#%%
class AircraftSocComms():
    def __init__ (self, port):
        
        self.serialLink = SerialLink(port)
        
    def Begin(self):
        print("Initializing communication with SOC...")
        self.serialLink.begin()
        print("done!")
        
        return True
        
    def SendMessage(self, message, Payload):
        address = 0
        
        print('Message Send - ID: ' + str(message) + '\tAddress: ' + str(address) + '\tPayload: ' + str(Payload))
        
        self.serialLink.write(message, address, Payload);
        
        return True
        
    def CheckMessage(self):
        self.serialLink.checkReceived()
        return self.serialLink.available()
        
    def ReceiveMessage(self):
        message, address, Payload = self.serialLink.read()
        
        print('Message Recv - ID: ' + str(message) + '\tAddress: ' + str(address) + '\tPayload: ' + str(Payload))
        
        return message, address, Payload
        
    def SendAck(self, msgID, msgSubID = 0):
        
        configMsgAck = fmu_messages.config_ack()
        configMsgAck.ack_id = msgID
        configMsgAck.ack_subid = msgSubID
        
        self.SendMessage(configMsgAck.id, configMsgAck.pack())

        return True


#%%
# SOC connects to ptySimSoc : 

#socat -vx -d udp4-datagram:127.0.0.1:6223 PTY,link=ptySimSoc,raw,nonblock,echo=0,b1500000,crnl | 
#socat -vx -d PTY,link=ptySimSoc,raw,nonblock,echo=0,b1500000,crnl udp4-datagram:127.0.0.1:6222

# Start a Virtual link for a psuedo-tty, SIL
# socat -vx -d -d PTY,link=ptySimFmu,rawer PTY,link=ptySimSoc,rawer; stty sane;

SocComms = AircraftSocComms(port = 'ptySimFmu')
fmuMode = 'Config'


# Config Messages
cfgMsgBasic = fmu_messages.config_basic()
cfgMsgMpu9250 = fmu_messages.config_mpu9250()
cfgMsgBme280 = fmu_messages.config_bme280()
cfgMsgUblox = fmu_messages.config_ublox()
cfgMsgAms5915 = fmu_messages.config_ams5915()
cfgMsgSwift = fmu_messages.config_swift()
cfgMsgAnalog = fmu_messages.config_analog()
cfgMsgEffector = fmu_messages.config_effector()
cfgMsgMission = fmu_messages.config_mission()
cfgMsgControlGain = fmu_messages.config_control_gain()

# Data Messages
dataMsgCommand = fmu_messages.command_effectors()

dataMsgBifrost = fmu_messages.data_bifrost()
dataMsgMpu9250 = fmu_messages.data_mpu9250()
dataMsgBme280 = fmu_messages.data_bme280()
dataMsgUblox = fmu_messages.data_ublox()
dataMsgSwift = fmu_messages.data_swift()
dataMsgSbus = fmu_messages.data_sbus()


SocComms.Begin()

while (True):
    # Run Stuff
    # Send Bifrost Data...
    # FIXIT
    
    # Update Mission run mode
    # FIXIT
    
    #
    if (fmuMode is 'Run'):
        # Read all data from Sim
        # FIXIT
        
        # Send Data Messages to SOC
        # Loop through all items that have been configured
        # FIXIT
        
        SocComms.SendMessage(dataMsgMpu9250.id, dataMsgMpu9250.pack())
        SocComms.SendMessage(dataMsgBme280.id, dataMsgBme280.pack())
        SocComms.SendMessage(dataMsgUblox.id, dataMsgUblox.pack())
        SocComms.SendMessage(dataMsgSwift.id, dataMsgSwift.pack())
        SocComms.SendMessage(dataMsgSbus.id, dataMsgSbus.pack())
    
    # Check and Recieve Messages

    while(SocComms.CheckMessage()):
        msgID, msgAddress, msgPayload = SocComms.ReceiveMessage()
        
        if msgID == fmu_messages.command_mode_id: # request mode
                    
            fmuModeMsg = fmu_messages.command_mode()
            fmuModeMsg.unpack(msg = msgPayload[-1].to_bytes(1, byteorder = 'little'))
            
            if fmuModeMsg.mode == 0:
                fmuMode = 'Config'
            elif fmuModeMsg.mode == 1:
                fmuMode = 'Run'
                    
            print ('Set FMU Mode: ' + fmuMode)
                
        elif (fmuMode is 'Run'):
                        
            # Receive Command Effectors
            if msgID == dataMsgCommand.id:
                dataMsgCommand.unpack(msg = msgPayload.to_bytes(1, byteorder = 'little'))

                print(dataMsgCommand.command[:2])
            elif msgID == dataMsgBifrost.id:
                dataMsgBifrost.unpack(msg = msgPayload.to_bytes(len(msgPayload), byteorder = 'little'))
            
            
        elif (fmuMode is 'Config'):
            
            if (msgID in [cfgMsgBasic.id, cfgMsgMpu9250.id, cfgMsgBme280.id, cfgMsgUblox.id, cfgMsgSwift.id]) : 
                
                # FIXIT - do something with the config messages!!
                
                print("\tConfig " + str(msgPayload))
                
                SocComms.SendAck(msgID, 0);
            else:
                print("Unhandled message while in Configuration mode, id: " + str(msgID))
            
        
#SocComms.Close()

