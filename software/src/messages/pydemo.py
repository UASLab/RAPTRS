#!/usr/bin/python3

import socket
import time

import fmu_messages

UDP_IP = "127.0.0.1"
RECV_PORT = 6222
SEND_PORT = 6223

recv_sock = socket.socket(socket.AF_INET, # Internet
                          socket.SOCK_DGRAM) # UDP
recv_sock.bind(('', RECV_PORT))
recv_sock.setblocking(False)

send_sock = socket.socket(socket.AF_INET, # Internet
                          socket.SOCK_DGRAM) # UDP
send_sock.setblocking(False)

imu = fmu_messages.data_mpu9250()
eff = fmu_messages.command_effectors()

while True:
    # look for incoming message
    while True:
        try:
            msg, addr = recv_sock.recvfrom(1024) # buffer size is 1024 bytes
        except:
            break
        print("received:", len(msg), "bytes")
        imu.unpack(msg)
        print(" IMU:")
        print("  Accels: %.2f %.2f %.2f" % (imu.AccelX_mss, imu.AccelY_mss, imu.AccelZ_mss))
        print("  Gyros: %.2f %.2f %.2f" % (imu.GyroX_rads, imu.GyroY_rads, imu.GyroZ_rads))
        print("  Mags: %.2f %.2f %.2f" % (imu.MagX_uT, imu.MagY_uT, imu.MagZ_uT))
        print("  Temp: %.2f" % imu.Temperature_C)
        print()

    # send a message back
    for i in range(len(eff.command)):
        eff.command[i] = i / 20.0
    msg = eff.pack()
    print("sending:", len(msg), "bytes")
    send_sock.sendto(msg, (UDP_IP, SEND_PORT))

    time.sleep(1)
