#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
Copyright (c) 2016 - 2020 Regents of the University of Minnesota.
MIT License; See LICENSE.md for complete details
Author: Chris Regan
'''

import time
import os

import fmu_messages
from SerialLink import SerialLink

# Act as the FMU side of the FMU-SOC comms.
# SOC will connect to ptySimSoc : 

# Start a Virtual link for a psuedo-tty, SIL
#On Host: socat -d -d PTY,link=ptySimSoc,rawer PTY,link=ptySimFmu,rawer; stty sane;

# Start a Virtual link for a psuedo-tty through BBB USB, HIL
#On BBB: socat -d -d tcp-listen:8000,reuseaddr,fork PTY,link=ptySimSoc,rawer; stty sane;
#On Host: socat -d -d PTY,link=ptySimFmu,rawer tcp:192.168.7.2:8000; stty sane;


#socat -vx -d udp4-datagram:127.0.0.1:6223 PTY,link=ptySimSoc,rawer,nonblock,crnl | 
#socat -vx -d PTY,link=ptySimSoc,rawer,nonblock,crnl udp4-datagram:127.0.0.1:6222


# SOC will request the FMU to Config Mode, then Run Mode

#Config:
#1) Read config messages
#
#Run:
#1) Read Sensors (JSBSim source, or otherwise faked)
#2) Send Sensor messages to SOC
#3) Read effector messages from SOC
#4) Send effector commands (JSBSim)
#5) Step Simulation (X times, for 20ms worth)
#6) wait until 20ms (from Sensor read)

#%%
class AircraftSocComms():
    def __init__ (self, port):
        
        self.serialLink = SerialLink(port)
        
    def Begin(self):
        print("Initializing communication with SOC...")
        self.serialLink.begin()
        print("done!")
        
        return True
        
    def SendMessage(self, msgID, msgAddress, msgPayload, ackReq = True):
        # msgType is used by SerialLink to determine if a SerialLink-level Ack is required
        if ackReq:
            msgType = 3
        else:
            msgType = 2
            
        # print('Message Send: ' + '\tID: ' + str(msgID) + '\tAddress: ' + str(msgAddress) + '\tPayload: ' + str(msgPayload))
        
        # Join msgData as: ID + Address + Payload
        msgData = b''
        msgData += msgID.to_bytes(1, byteorder = 'little')
        msgData += msgAddress.to_bytes(1, byteorder = 'little')
        msgData += msgPayload
        
        self.serialLink.write(msgType, msgData);
        
        return True
        
    def CheckMessage(self):
        self.serialLink.checkReceived()
        return self.serialLink.available()
        
    def ReceiveMessage(self):
        msgID = None
        msgAddress = None
        msgPayload = None
        
        msgType, msgData = self.serialLink.read()
        
        if msgType >= 2: # command type message
            # Data = ID + Address + Payload
            msgID = int.from_bytes(msgData[0:1], byteorder = 'little') # Message ID
            msgAddress = int.from_bytes(msgData[1:2], byteorder = 'little') # Address
            msgPayload = msgData[2:]
            
            # Send an SerialLink Ack - FIXIT - move to SerialLink
#            self.serialLink.sendStatus(True)

        # print('Message Recv: ' + '\tID: ' + str(msgID) + '\tAddress: ' + str(msgAddress) + '\tPayload: ' + str(msgPayload))
        
        return msgID, msgAddress, msgPayload
        
    def SendAck(self, msgID, msgSubID = 0): # This is a Config Message Ack (not a SerialLink Ack!)

        configMsgAck = fmu_messages.config_ack()
        configMsgAck.ack_id = msgID
        configMsgAck.ack_subid = msgSubID
        
        self.SendMessage(configMsgAck.id, 0, configMsgAck.pack(), ackReq = False)

        return True


#%%

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


tRate_s = 1/50 # Desired Run rate
SocComms.Begin()
while (True):
    tStart_s = time.time()
        
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

#        SocComms.SendMessage(dataMsgMpu9250.id, 0, dataMsgMpu9250.pack())
#        SocComms.SendMessage(dataMsgBme280.id, 0, dataMsgBme280.pack())
#        SocComms.SendMessage(dataMsgUblox.id, 0, dataMsgUblox.pack())
#        SocComms.SendMessage(dataMsgSwift.id, 0, dataMsgSwift.pack())
#        SocComms.SendMessage(dataMsgSbus.id, 0, dataMsgSbus.pack())
        pass
    
    # Check and Recieve Messages
    while(SocComms.CheckMessage()):
        msgID, msgAddress, msgPayload = SocComms.ReceiveMessage()
        
        # Message did not have an ID, likely a SerialLink Ack/Nack message
        if msgID is None:
            continue
        
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
            
            
            cfgIdList = [cfgMsgBasic.id, cfgMsgMpu9250.id, cfgMsgBme280.id, cfgMsgUblox.id, 
                         cfgMsgAms5915.id, cfgMsgSwift.id, cfgMsgAnalog.id, cfgMsgEffector.id, 
                         cfgMsgMission.id, cfgMsgControlGain.id]
            
            if msgID in cfgIdList: 
            
                # FIXIT - do something with the config messages!!
                
                print("Config Set: " + str(msgPayload))
                
                SocComms.SendAck(msgID);
            else:
                print("Unhandled message while in Configuration mode, id: " + str(msgID))

        # if 
    #
    
    # Timer - attempt...
    tHold_s = tRate_s - (time.time() - tStart_s)
    if tHold_s > 0:
        time.sleep(tHold_s)
        
#    print('Time (ms): ' + str((time.time() - tStart_s) * 1000))
        
# end while(True)
        
#SocComms.Close()


