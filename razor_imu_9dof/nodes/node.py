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

import roslib; roslib.load_manifest('razor_imu_9dof')
import rospy

import serial
import string
import math

from time import time
from sensor_msgs.msg import Imu
from razor_imu_9dof.msg import RazorImu
import tf

grad2rad = 3.141592/180.0

rospy.init_node("node")
pub = rospy.Publisher('imu', Imu)
pubRaw = rospy.Publisher('imuRaw', RazorImu)

imuMsg = Imu()
imuRawMsg = RazorImu()
imuMsg.orientation_covariance = [999999 , 0 , 0,
0, 9999999, 0,
0, 0, 999999]
imuMsg.angular_velocity_covariance = [9999, 0 , 0,
0 , 99999, 0,
0 , 0 , 0.02]
imuMsg.linear_acceleration_covariance = [0.2 , 0 , 0,
0 , 0.2, 0,
0 , 0 , 0.2]

default_port='/dev/ttyUSB1'
port = rospy.get_param('~device', default_port)
# Check your COM port and baud rate
ser = serial.Serial(port=port,baudrate=57600, timeout=1)

#f = open("Serial"+str(time())+".txt", 'w')

roll=0
pitch=0
yaw=0
rospy.loginfo("Giving the board 5 seconds to boot...")
rospy.sleep(5) # Sleep for 5 seconds to wait for the board to boot then only write command.
ser.write('#ox' + chr(13)) # To start display angle and sensor reading in text
rospy.sleep(0.5) #give serial port some time to process above command
ser.flushInput()  #discard old input, still in invalid format
rospy.loginfo("Publishing...")
while 1:
    line = ser.readline()
    line = line.replace("#YPRAMG=","")   # Delete "#YPR="
    #f.write(line)                     # Write to the output log file
    words = string.split(line,",")    # Fields split
    if len(words) > 2:
        try:
            yaw = float(words[0])*grad2rad
            pitch = -float(words[1])*grad2rad
            roll = -float(words[2])*grad2rad
            
            # Publish message
            imuMsg.linear_acceleration.x = float(words[3])
            imuMsg.linear_acceleration.y = float(words[4])
            imuMsg.linear_acceleration.z = float(words[5])
            
            imuMsg.angular_velocity.x = float(words[9])
            imuMsg.angular_velocity.y = float(words[10])
            imuMsg.angular_velocity.z = float(words[11])
        except Exception as e:
            print e
            
        q = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
        imuMsg.orientation.x = q[0]
        imuMsg.orientation.y = q[1]
        imuMsg.orientation.z = q[2]
        imuMsg.orientation.w = q[3]
        imuMsg.header.stamp= rospy.Time.now()
        imuMsg.header.frame_id = 'base_link'
        pub.publish(imuMsg)
            
        # Publish Raw message from Razor board
        imuRawMsg.yaw = yaw
        imuRawMsg.pitch = pitch
        imuRawMsg.roll = roll
        pubRaw.publish(imuRawMsg)
        
ser.close
#f.close
