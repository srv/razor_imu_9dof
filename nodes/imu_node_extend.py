#!/usr/bin/env python

# Copyright (c) 2012, Tang Tiong Yew
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import serial
import string
import math
import sys
import struct
import io

#from time import time
from sensor_msgs.msg import Imu, MagneticField
from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from razor_imu_9dof.cfg import imuConfig
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

degrees2rad = math.pi/180.0
imu_yaw_calibration = 0.0

# Callback for dynamic reconfigure requests
def reconfig_callback(config, level):
    global imu_yaw_calibration
    rospy.loginfo("""Reconfigure request for yaw_calibration: %d""" %(config['yaw_calibration']))
    #if imu_yaw_calibration != config('yaw_calibration'):
    imu_yaw_calibration = config['yaw_calibration']
    rospy.loginfo("Set imu_yaw_calibration to %d" % (imu_yaw_calibration))
    return config

rospy.init_node("razor_node")
#We only care about the most recent measurement, i.e. queue_size=1
imuMsgRaw_pub = rospy.Publisher('imu_raw', Imu, queue_size=1)
imuMsgCal_pub = rospy.Publisher('imu_cal', Imu, queue_size=1)
magMsgRaw_pub = rospy.Publisher('mag_raw', MagneticField, queue_size=1)
magMsgCal_pub = rospy.Publisher('mag_cal', MagneticField, queue_size=1)
srv = Server(imuConfig, reconfig_callback)  # define dynamic_reconfigure callback
diag_pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=1)
diag_pub_time = rospy.get_time();

imuMsgRaw = Imu()
imuMsgCal = Imu()
magMsgRaw = MagneticField()
magMsgCal = MagneticField()
seq = 1

# Orientation covariance estimation:
# Observed orientation noise: 0.3 degrees in x, y, 0.6 degrees in z
# Magnetometer linearity: 0.1% of full scale (+/- 2 gauss) => 4 milligauss
# Earth's magnetic field strength is ~0.5 gauss, so magnetometer nonlinearity could
# cause ~0.8% yaw error (4mgauss/0.5 gauss = 0.008) => 2.8 degrees, or 0.050 radians
# i.e. variance in yaw: 0.0025
# Accelerometer non-linearity: 0.2% of 4G => 0.008G. This could cause
# static roll/pitch error of 0.8%, owing to gravity orientation sensing
# error => 2.8 degrees, or 0.05 radians. i.e. variance in roll/pitch: 0.0025
# so set all covariances the same.
# imuMsg.orientation_covariance = [
# 0.0025 , 0 , 0,
# 0, 0.0025, 0,
# 0, 0, 0.0025
# ]

# Angular velocity covariance estimation:
# Observed gyro noise: 4 counts => 0.28 degrees/sec
# nonlinearity spec: 0.2% of full scale => 8 degrees/sec = 0.14 rad/sec
# Choosing the larger (0.14) as std dev, variance = 0.14^2 ~= 0.02
vel_cov = [
0.02, 0 , 0,
0 , 0.02, 0,
0 , 0 , 0.02
]
imuMsgRaw.angular_velocity_covariance = vel_cov
imuMsgCal.angular_velocity_covariance = vel_cov

# linear acceleration covariance estimation:
# observed acceleration noise: 5 counts => 20milli-G's ~= 0.2m/s^2
# nonliniarity spec: 0.5% of full scale => 0.2m/s^2
# Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
acc_cov = [
0.04 , 0 , 0,
0 , 0.04, 0,
0 , 0 , 0.04
]
imuMsgRaw.linear_acceleration_covariance = acc_cov
imuMsgCal.linear_acceleration_covariance = acc_cov

default_port='/dev/ttyUSB0'
port = rospy.get_param('~port', default_port)
# frame_id = rospy.get_param('~frame_id', 'base_imu_link')
frame_id = rospy.get_param('~frame_id', '/imu')

#accelerometer
accel_x_min = rospy.get_param('~accel_x_min', -250.0)
accel_x_max = rospy.get_param('~accel_x_max', 250.0)
accel_y_min = rospy.get_param('~accel_y_min', -250.0)
accel_y_max = rospy.get_param('~accel_y_max', 250.0)
accel_z_min = rospy.get_param('~accel_z_min', -250.0)
accel_z_max = rospy.get_param('~accel_z_max', 250.0)

# magnetometer
magn_x_min = rospy.get_param('~magn_x_min', -600.0)
magn_x_max = rospy.get_param('~magn_x_max', 600.0)
magn_y_min = rospy.get_param('~magn_y_min', -600.0)
magn_y_max = rospy.get_param('~magn_y_max', 600.0)
magn_z_min = rospy.get_param('~magn_z_min', -600.0)
magn_z_max = rospy.get_param('~magn_z_max', 600.0)
calibration_magn_use_extended = rospy.get_param('~calibration_magn_use_extended', True)
magn_ellipsoid_center = rospy.get_param('~magn_ellipsoid_center', [0, 0, 0])
magn_ellipsoid_transform = rospy.get_param('~magn_ellipsoid_transform', [[0, 0, 0], [0, 0, 0], [0, 0, 0]])
imu_yaw_calibration = rospy.get_param('~imu_yaw_calibration', 0.0)

# gyroscope
gyro_average_offset_x = rospy.get_param('~gyro_average_offset_x', 0.0)
gyro_average_offset_y = rospy.get_param('~gyro_average_offset_y', 0.0)
gyro_average_offset_z = rospy.get_param('~gyro_average_offset_z', 0.0)

#rospy.loginfo("%f %f %f %f %f %f", accel_x_min, accel_x_max, accel_y_min, accel_y_max, accel_z_min, accel_z_max)
#rospy.loginfo("%f %f %f %f %f %f", magn_x_min, magn_x_max, magn_y_min, magn_y_max, magn_z_min, magn_z_max)
#rospy.loginfo("%s %s %s", str(calibration_magn_use_extended), str(magn_ellipsoid_center), str(magn_ellipsoid_transform[0][0]))
#rospy.loginfo("%f %f %f", gyro_average_offset_x, gyro_average_offset_y, gyro_average_offset_z)

# Check your COM port and baud rate
rospy.loginfo("Opening %s...", port)
try:
    ser = serial.Serial(port=port, baudrate=57600, timeout=1, bytesize=serial.EIGHTBITS)
    ser_io = io.TextIOWrapper(io.BufferedRWPair(ser, ser, 1),  
                               newline = '\r\n',
                               line_buffering = True)

except serial.serialutil.SerialException:
    rospy.logerr("IMU not found at port "+port + ". Did you specify the correct port in the launch file?")
    #exit
    sys.exit(0)

roll=0
pitch=0
yaw=0
seq=0
accel_factor = 9.806 / 256.0    # sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.
rospy.loginfo("Giving the razor IMU board 5 seconds to boot...")
rospy.sleep(5) # Sleep for 5 seconds to wait for the board to boot

### configure board ###
#stop datastream
ser.write('#o0' + chr(13))

#discard old input
#automatic flush - NOT WORKING
#ser.flushInput()  #discard old input, still in invalid format
#flush manually, as above command is not working
discard = ser.readlines() 


rospy.loginfo("Writing calibration values to razor IMU board...")
#set calibration values
ser.write('#caxm' + str(accel_x_min) + chr(13))
ser.write('#caxM' + str(accel_x_max) + chr(13))
ser.write('#caym' + str(accel_y_min) + chr(13))
ser.write('#cayM' + str(accel_y_max) + chr(13))
ser.write('#cazm' + str(accel_z_min) + chr(13))
ser.write('#cazM' + str(accel_z_max) + chr(13))

if (not calibration_magn_use_extended):
    ser.write('#cmxm' + str(magn_x_min) + chr(13))
    ser.write('#cmxM' + str(magn_x_max) + chr(13))
    ser.write('#cmym' + str(magn_y_min) + chr(13))
    ser.write('#cmyM' + str(magn_y_max) + chr(13))
    ser.write('#cmzm' + str(magn_z_min) + chr(13))
    ser.write('#cmzM' + str(magn_z_max) + chr(13))
else:
    ser.write('#ccx' + str(magn_ellipsoid_center[0]) + chr(13))
    ser.write('#ccy' + str(magn_ellipsoid_center[1]) + chr(13))
    ser.write('#ccz' + str(magn_ellipsoid_center[2]) + chr(13))
    ser.write('#ctxX' + str(magn_ellipsoid_transform[0][0]) + chr(13))
    ser.write('#ctxY' + str(magn_ellipsoid_transform[0][1]) + chr(13))
    ser.write('#ctxZ' + str(magn_ellipsoid_transform[0][2]) + chr(13))
    ser.write('#ctyX' + str(magn_ellipsoid_transform[1][0]) + chr(13))
    ser.write('#ctyY' + str(magn_ellipsoid_transform[1][1]) + chr(13))
    ser.write('#ctyZ' + str(magn_ellipsoid_transform[1][2]) + chr(13))
    ser.write('#ctzX' + str(magn_ellipsoid_transform[2][0]) + chr(13))
    ser.write('#ctzY' + str(magn_ellipsoid_transform[2][1]) + chr(13))
    ser.write('#ctzZ' + str(magn_ellipsoid_transform[2][2]) + chr(13))

ser.write('#cgx' + str(gyro_average_offset_x) + chr(13))
ser.write('#cgy' + str(gyro_average_offset_y) + chr(13))
ser.write('#cgz' + str(gyro_average_offset_z) + chr(13))

rospy.loginfo("Calibration values transfered to razor IMU board")

#print calibration values for verification by user
ser.flushInput()
ser.write('#p' + chr(13))
calib_data = ser.readlines()
calib_data_print = "Printing set calibration values:\r\n"
for line in calib_data:
    calib_data_print += line
rospy.loginfo(calib_data_print)

#set output mode
rospy.loginfo("Set configuration of the razor IMU board")
ser.write('#osbt' + chr(13)) # To start display angle and sensor reading in text
ser.write('#s12' + chr(13)) # Token sync

#start datastream
rospy.loginfo("Start data stream")
ser.write('#o1' + chr(13))

#automatic flush - NOT WORKING
ser.flushInput()  #discard old input, still in invalid format
#flush manually, as above command is not working - it breaks the serial connection
# rospy.loginfo("Flushing first 200 IMU entries...")
# for x in range(0, 80):
#     line = ser.readline()
rospy.loginfo("Publishing IMU data...")
#f = open("raw_imu_data.log", 'w')

while not rospy.is_shutdown():
    #ser.flushInput()
    line = ser.readline()
    if line[:4]=='#A-R':
        acc = line 
        line = ser.readline()
        if line[:4]=='#M-R': 
            mag = line 
            line = ser.readline()
            if line[:4]=='#G-R': 
                gyr = line 
                acc_v = string.split(acc.replace("#A-R=",""),",")
                mag_v = string.split(mag.replace("#M-R=",""),",")
                gyr_v = string.split(gyr.replace("#G-R=",""),",")
                

                # Publish message
                stamp = rospy.Time.now()

                # Publish measures in ENU
                imuMsgRaw.linear_acceleration.x = -float(acc_v[0]) * accel_factor
                imuMsgRaw.linear_acceleration.y = float(acc_v[1]) * accel_factor
                imuMsgRaw.linear_acceleration.z = float(acc_v[2]) * accel_factor

                imuMsgRaw.angular_velocity.x = float(gyr_v[0]) 
                imuMsgRaw.angular_velocity.y = -float(gyr_v[1])
                imuMsgRaw.angular_velocity.z = -float(gyr_v[2])

                magMsgRaw.magnetic_field.x = float(mag_v[0]) 
                magMsgRaw.magnetic_field.y = -float(mag_v[1]) 
                magMsgRaw.magnetic_field.z = -float(mag_v[2])
               
                imuMsgRaw.header.stamp = stamp
                imuMsgRaw.header.frame_id = frame_id
                imuMsgRaw.header.seq = seq
                magMsgRaw.header.stamp = stamp
                magMsgRaw.header.frame_id = frame_id
                magMsgRaw.header.seq = seq

                seq = seq + 1
                imuMsgRaw_pub.publish(imuMsgRaw)
                magMsgRaw_pub.publish(magMsgRaw)

    if line[:4]=='#A-C':
        acc = line 
        line = ser.readline()
        if line[:4]=='#M-C': 
            mag = line 
            line = ser.readline()
            if line[:4]=='#G-C': 
                gyr = line 
                acc_v = string.split(acc.replace("#A-C=",""),",")
                mag_v = string.split(mag.replace("#M-C=",""),",")
                gyr_v = string.split(gyr.replace("#G-C=",""),",")

                # Publish message
                stamp = rospy.Time.now()

                # Publish measures in ENU
                imuMsgCal.linear_acceleration.x = -float(acc_v[0]) * accel_factor
                imuMsgCal.linear_acceleration.y = float(acc_v[1]) * accel_factor
                imuMsgCal.linear_acceleration.z = float(acc_v[2]) * accel_factor

                imuMsgCal.angular_velocity.x = float(gyr_v[0]) 
                imuMsgCal.angular_velocity.y = -float(gyr_v[1])
                imuMsgCal.angular_velocity.z = -float(gyr_v[2])

                magMsgCal.magnetic_field.x = float(mag_v[0]) 
                magMsgCal.magnetic_field.y = -float(mag_v[1]) 
                magMsgCal.magnetic_field.z = -float(mag_v[2])

                imuMsgCal.header.stamp = stamp
                imuMsgCal.header.frame_id = frame_id
                imuMsgCal.header.seq = seq
                magMsgCal.header.stamp = stamp
                magMsgCal.header.frame_id = frame_id
                magMsgCal.header.seq = seq

                seq = seq + 1

                imuMsgCal_pub.publish(imuMsgCal)
                magMsgCal_pub.publish(magMsgCal)

ser.close
#f.close
