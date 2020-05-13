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
dataMsgCompound = fmu_messages.data_compound()
dataMsgBifrost = fmu_messages.data_bifrost()

AcquireTimeData = False
AcquireInternalMpu9250Data = False
AcquireInternalBme280Data = False


def add_msg(msgID, msgIndx, msgPayload):
    msgLen = len(msgPayload)
    
    msgData = b''
    msgData += msgID.to_bytes(1, byteorder = 'little')
    msgData += msgIndx.to_bytes(1, byteorder = 'little')
    msgData += msgLen.to_bytes(1, byteorder = 'little')
    msgData += msgPayload
    return msgData


#
tFrameRate_s = 1/50 # Desired Run rate
SocComms.Begin()
cfgMsgList = []
sensorList = []
effList = []

tStart_s = time.time()

while (True):
    tFrameStart_s = time.time()
        
    # Run Stuff
    # Send Bifrost Data...
    # FIXIT
    
    # Update Mission run mode
    # FIXIT
    
    #
    if (fmuMode is 'Run'):
        # Read all data from Sim, populate into the message
        # FIXIT
        
        
        # Send Data Messages to SOC
        # Loop through all items that have been configured
        dataMsg = b''

        if AcquireTimeData:
            msg = next((s for s in sensorList if s.id is fmu_messages.data_time().id), None)
            msg.time_us = int((tFrameStart_s - tStart_s) * 1e6)
            dataMsg += add_msg(msg.id, 0, msg.pack())
            
        if AcquireInternalMpu9250Data:
            msg = next((s for s in sensorList if s.id is fmu_messages.data_mpu9250().id), None)
            dataMsg += add_msg(msg.id, 0, msg.pack())
            
        if AcquireInternalBme280Data:
            msg = next((s for s in sensorList if s.id is fmu_messages.data_bme280().id), None)
            dataMsg += add_msg(msg.id, 0, msg.pack())
            
        for indx, msg in enumerate([s for s in sensorList if s.id is fmu_messages.data_mpu9250_short().id]):
            dataMsg += add_msg(msg.id, indx, msg.pack())
            
        for indx, msg in  enumerate([s for s in sensorList if s.id is fmu_messages.data_bme280().id]):
            if not (AcquireInternalBme280Data and indx == 1):
                dataMsg += add_msg(msg.id, indx, msg.pack())
            
        for indx, msg in enumerate([s for s in sensorList if s.id is fmu_messages.data_ublox().id]):
            msg.Fix = True
            dataMsg += add_msg(msg.id, indx, msg.pack())
            
        for indx, msg in enumerate([s for s in sensorList if s.id is fmu_messages.data_swift().id]):
            dataMsg += add_msg(msg.id, indx, msg.pack())
            
        for indx, msg in enumerate([s for s in sensorList if s.id is fmu_messages.data_sbus().id]):
            dataMsg += add_msg(msg.id, indx, msg.pack())
            
        for indx, msg in enumerate([s for s in sensorList if s.id is fmu_messages.data_ams5915().id]):
            dataMsg += add_msg(msg.id, indx, msg.pack())
            
        for indx, msg in enumerate([s for s in sensorList if s.id is fmu_messages.data_analog().id]):
            dataMsg += add_msg(msg.id, indx, msg.pack())
        
        # Send the Compound Sensor Message
        SocComms.SendMessage(dataMsgCompound.id, 0, dataMsg)
        
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
                dataMsgCommand.unpack(msg = msgPayload)

                print(dataMsgCommand.command)
            elif msgID == dataMsgBifrost.id:
                pass
                # dataMsgBifrost.unpack(msg = msgPayload)
            
            
        elif (fmuMode is 'Config'):
            print("Config Set: " + str(msgPayload))
            
            if msgID == cfgMsgBasic.id:
                cfgMsgBasic.unpack(msgPayload)
                
                sensorType = cfgMsgBasic.sensor
                cfgMsgList.append((msgID, sensorType))
                
                if sensorType == fmu_messages.sensor_type_time:
                    sensorList.append(fmu_messages.data_time())
                    AcquireTimeData = True
                    
                elif sensorType in [fmu_messages.sensor_type_input_voltage, fmu_messages.sensor_type_regulated_voltage, fmu_messages.sensor_type_pwm_voltage, fmu_messages.sensor_type_sbus_voltage]:
                    sensorList.append(fmu_messages.data_analog())
                    
                elif sensorType == fmu_messages.sensor_type_internal_bme280:
                    sensorList.append(fmu_messages.data_bme280())
                    AcquireInternalBme280Data = True
                    
                elif sensorType == fmu_messages.sensor_type_sbus:
                    sensorList.append(fmu_messages.data_sbus())
                    
                SocComms.SendAck(msgID)
                    
            elif msgID == cfgMsgMpu9250.id:
                cfgMsgList.append((msgID, 0))
                
                cfgMsgMpu9250.unpack(msgPayload)
                
                if cfgMsgMpu9250.internal == True:
                    AcquireInternalMpu9250Data = True
                    sensorList.append(fmu_messages.data_mpu9250())
                else:
                    sensorList.append(fmu_messages.data_mpu9250_short())
                
                SocComms.SendAck(msgID)
                
            elif msgID == cfgMsgBme280.id:
                cfgMsgList.append((msgID, 0))
                sensorList.append(fmu_messages.data_bme280())
                
                cfgMsgBme280.unpack(msgPayload)
                
                SocComms.SendAck(msgID)
                
            elif msgID == cfgMsgUblox.id:
                cfgMsgList.append((msgID, 0))
                sensorList.append(fmu_messages.data_ublox())
                
                cfgMsgUblox.unpack(msgPayload)
                
                SocComms.SendAck(msgID)
                
            elif msgID == cfgMsgAms5915.id:
                cfgMsgList.append((msgID, 0))
                sensorList.append(fmu_messages.data_ams5915())
                
                cfgMsgAms5915.unpack(msgPayload)
                cfgMsgAms5915.output
                
                SocComms.SendAck(msgID)
                
            elif msgID == cfgMsgSwift.id:
                cfgMsgList.append((msgID, 0))
                sensorList.append(fmu_messages.data_swift())
                
                cfgMsgSwift.unpack(msgPayload)
                cfgMsgSwift.output
                
                SocComms.SendAck(msgID)
                
            elif msgID == cfgMsgAnalog.id:
                cfgMsgList.append((msgID, 0))
                sensorList.append(fmu_messages.data_analog())
                
                cfgMsgAnalog.unpack(msgPayload)
                
                SocComms.SendAck(msgID)
                
            elif msgID == cfgMsgEffector.id:
                cfgMsgList.append((msgID, 0))
                
                cfgMsgEffector.unpack(msgPayload)
                effList.append(cfgMsgEffector.input)
                
                SocComms.SendAck(msgID)
                
            elif msgID == cfgMsgMission.id:
                cfgMsgList.append((msgID, 0))
                
                cfgMsgMission.unpack(msgPayload)
                
                SocComms.SendAck(msgID)
                
            elif msgID == cfgMsgControlGain.id:
                cfgMsgList.append((msgID, 0))
                
                cfgMsgControlGain.unpack(msgPayload)
                
                SocComms.SendAck(msgID)
                
            else:
                print("Unhandled message while in Configuration mode, id: " + str(msgID))

        # if 
    #
    
    # Timer
    tHold_s = tFrameRate_s - (time.time() - tFrameStart_s)
    if tHold_s > 0:
        time.sleep(tHold_s)
    
# end while(True)
        
#SocComms.Close()


