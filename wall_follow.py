#!/usr/bin/env python
# Carolyn Wang, Eddie Jeffs
# ECE 350-012: Race 1
#
# Implements Left Wall Following. Node subscribes to /scan topic
# getting the LaserScan message and publishes to /nav topic with
# the AckermannDriveStamped message.
##

from __future__ import print_function
import sys
import math
import numpy as np
from numpy import diff
import time


#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


def cal_time(func):
    def wrapper(*args, **kargs):
        start = time.time()
        f = func(*args, **kargs)
        end = time.time()
        print("{:15}{:10.3e}".format(func.__name__, end - start))
        return f
    return wrapper


#PID CONTROL PARAMS
kp = .7
ki = .2
kd = 2

servo_offset = 0.0
prev_error = 0.0
error = 0.0
integral = 0.0
GAMMA = .4  #discounted factor for integral control
last = 0  #storing previous error for derivative control


#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.7
VELOCITY = 2.0 # meters per second
TIME_INCREMENT = 0.3
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)

        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
    @cal_time
    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle min
        # Outputs length in meters to object with angle in lidar scan field of view
        # make sure to take care of nans etc.

        return data.ranges[int(len(data.ranges)/2 + angle/data.angle_increment)]
    @cal_time
    def pid_control(self, error, velocity):
        global integral
        global last
        global prev_error
        global kp
        global ki
        global kd
        global GAMMA
        
        integral = error + GAMMA * integral # calculates discounted integral value
        derivative = (error - last)/2 # calculates derivative
        last = error

        angle = kp * error + ki * integral + kd * derivative # calculates steering angle

        velocity = 0.6 * math.exp(-(.3*abs(angle)-1.75)) # calculates velocity of car based on steering angle

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)
    @cal_time
    def followLeft(self, data, leftDist):
        # Follow left wall as per the algorithm

        phia = 50* math.pi / 180
        theta = 40 * math.pi / 180 # they add up to be 90
        dista = self.getRange(data, phia) # gives the lidar scan range of specific angle
        
        distb = self.getRange(data, phia + theta)
        
        alpha = math.atan((dista* math.cos(theta) - distb)/(dista * math.sin(theta))) # calculates angle from wall
        
        L = VELOCITY * TIME_INCREMENT
        
        Dt1 = distb * math.cos( alpha) + L * math.sin( alpha) # calculates future distance from wall
        
        piderror = leftDist - Dt1 # calculates error from desired distance
        return  -1 * piderror # -1* is for left wall

    @cal_time
    def lidar_callback(self, data):

        error = self.followLeft(data, DESIRED_DISTANCE_LEFT) # calculates error from target distance

        self.pid_control(error, VELOCITY) # sends error to PID control

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    rate = rospy.Rate(100) # 100Hz
    wf = WallFollow()
    # rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
    main(sys.argv)

