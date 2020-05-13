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

dataMsgTime = fmu_messages.data_time()
dataMsgMpu9250Short = fmu_messages.data_mpu9250_short()
dataMsgMpu9250 = fmu_messages.data_mpu9250()
dataMsgBme280 = fmu_messages.data_bme280()
dataMsgUblox = fmu_messages.data_ublox()
dataMsgAms5915 = fmu_messages.data_ams5915()
dataMsgSwift = fmu_messages.data_swift()
dataMsgSbus = fmu_messages.data_sbus()
dataMsgAnalog = fmu_messages.data_analog()
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
        
        dataMsgTime.time_us = int((tFrameStart_s - tStart_s) * 1e6)
        
        dataMsgUblox.Fix = True
        
        # Send Data Messages to SOC
        # Loop through all items that have been configured
        dataMsg = b''

        if AcquireTimeData:
            dataMsg += add_msg(dataMsgTime.id, 0, dataMsgTime.pack())
            
        if AcquireInternalMpu9250Data:
            dataMsg += add_msg(dataMsgMpu9250.id, 0, dataMsgMpu9250.pack())
            
        if AcquireInternalBme280Data:
            dataMsg += add_msg(dataMsgBme280.id, 0, dataMsgBme280.pack())
            
        for indx in range(sensorList.count(dataMsgMpu9250Short.id)):
            dataMsg += add_msg(dataMsgMpu9250Short.id, indx, dataMsgMpu9250Short.pack())
            
        for indx in range(sensorList.count(dataMsgBme280.id) - 1):
            dataMsg += add_msg(dataMsgBme280.id, indx, dataMsgBme280.pack())
            
        for indx in range(sensorList.count(dataMsgUblox.id)):
            dataMsg += add_msg(dataMsgUblox.id, indx, dataMsgUblox.pack())
            
        for indx in range(sensorList.count(dataMsgSwift.id)):
            dataMsg += add_msg(dataMsgSwift.id, indx, dataMsgSwift.pack())
            
        for indx in range(sensorList.count(dataMsgSbus.id)):
            dataMsg += add_msg(dataMsgSbus.id, indx, dataMsgSbus.pack())
            
        for indx in range(sensorList.count(dataMsgAms5915.id)):
            dataMsg += add_msg(dataMsgAms5915.id, indx, dataMsgAms5915.pack())
            
        for indx in range(sensorList.count(dataMsgAnalog.id)):
            dataMsg += add_msg(dataMsgAnalog.id, indx, dataMsgAnalog.pack())
        
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

                print(dataMsgCommand.command[:6])
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
                    sensorList.append(fmu_messages.data_time_id)
                    AcquireTimeData = True
                    
                elif sensorType in [fmu_messages.sensor_type_input_voltage, fmu_messages.sensor_type_regulated_voltage, fmu_messages.sensor_type_pwm_voltage, fmu_messages.sensor_type_sbus_voltage]:
                    sensorList.append(fmu_messages.data_analog_id)
                    
                elif sensorType == fmu_messages.sensor_type_internal_bme280:
                    sensorList.append(fmu_messages.data_bme280_id)
                    AcquireInternalBme280Data = True
                    
                elif sensorType == fmu_messages.sensor_type_sbus:
                    sensorList.append(fmu_messages.data_sbus_id)
                    
                SocComms.SendAck(msgID)
                    
            elif msgID == cfgMsgMpu9250.id:
                cfgMsgList.append((msgID, 0))
                sensorList.append(fmu_messages.data_mpu9250_id)
                
                cfgMsgMpu9250.unpack(msgPayload)
                
                if cfgMsgMpu9250.internal == True:
                    AcquireInternalMpu9250Data = True
                
                SocComms.SendAck(msgID)
                
            elif msgID == cfgMsgBme280.id:
                cfgMsgList.append((msgID, 0))
                sensorList.append(fmu_messages.data_bme280_id)
                
                cfgMsgBme280.unpack(msgPayload)
                
                SocComms.SendAck(msgID)
                
            elif msgID == cfgMsgUblox.id:
                cfgMsgList.append((msgID, 0))
                sensorList.append(fmu_messages.data_ublox_id)
                
                cfgMsgUblox.unpack(msgPayload)
                
                SocComms.SendAck(msgID)
                
            elif msgID == cfgMsgAms5915.id:
                cfgMsgList.append((msgID, 0))
                sensorList.append(fmu_messages.data_ams5915_id)
                
                cfgMsgAms5915.unpack(msgPayload)
                cfgMsgAms5915.output
                
                SocComms.SendAck(msgID)
                
            elif msgID == cfgMsgSwift.id:
                cfgMsgList.append((msgID, 0))
                sensorList.append(fmu_messages.data_swift_id)
                
                cfgMsgSwift.unpack(msgPayload)
                cfgMsgSwift.output
                
                SocComms.SendAck(msgID)
                
            elif msgID == cfgMsgAnalog.id:
                cfgMsgList.append((msgID, 0))
                sensorList.append(fmu_messages.data_analog_id)
                
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


