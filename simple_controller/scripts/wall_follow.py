#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 3.6#TODO
kd = 0.01#TODO
ki = 0.001#TODO
theta = np.pi/2.5
servo_offset = 0.0
prev_error = 0.0 
error = 0.0

#WALL FOLLOW PARAMS
DESIRED_DISTANCE_RIGHT = 0.7 # meters
DESIRED_DISTANCE_LEFT = 0.7
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        odom_topic = '/odom'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)#TODO: Subscribe to LIDAR
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)#TODO: Publish to drive
        
        self.speed = 0.0
        self.integral = 0.0

    #def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        #TODO: implement
        
        #return 0.0

    def pid_control(self, error):
        global prev_error
        global kp
        global ki
        global kd
        #angle = 0.0
        #TODO: Use kp, ki & kd to implement a PID controller
        
        self.integral += error
        
        u_t = kp*error + ki*self.integral + kd*(error - prev_error)
        prev_error = error
        
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = -u_t# pq left, not right #angle
        print(u_t)
        drive_msg.drive.speed = 1.5 if (u_t < np.pi/18) else ( 1.0 if (u_t < np.pi/9) else 0.5)# velocity after update
        #print(drive_msg.drive.speed)
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data):
        global theta
        #Follow left wall as per the algorithm, returnes e(t) = 1-D(t) 
        #TODO:implement
        b_angle = data.angle_min + 0.5*np.pi
        a_angle = b_angle - theta
        
        a = data.ranges[int(a_angle//data.angle_increment)]#it seems that it is our getrange
        b = data.ranges[int(b_angle//data.angle_increment)]
        print ("a = %f, b = %f", a, b)
        
        alpha = math.atan((a*math.cos(theta)-b)/(a*math.sin(theta)))
        
        #here I could have fucked up
        D_t = b * math.cos(alpha)
        error = DESIRED_DISTANCE_LEFT - D_t - 1.5*math.sin(alpha) #d - D_t - L sin alpha, probably i need to kill L
        
        return error 

    def lidar_callback(self, data):
        
        error = self.followLeft(data) #TODO: replace with error returned by followLeft
        
        #send error to pid_control
        self.pid_control(error)
        
    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = (odom_msg.twist.twist.linear.x)
        #print("self speed %f", self.speed)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
