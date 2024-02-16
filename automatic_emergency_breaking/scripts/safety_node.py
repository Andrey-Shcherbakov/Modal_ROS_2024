#!/usr/bin/env python
import rospy
import math
import numpy as np

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

# TODO: import ROS msg types and libraries

class Safety(object):
    """
    The class that handles emergency braking.
    """
    
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        self.threshold = 0.5
        
        # TODO: create ROS subscribers and publishers.
        self.pub_brake = rospy.Publisher('brake', AckermannDriveStamped, queue_size=10)
        self.pub_brake_bool = rospy.Publisher('brake_bool', Bool, queue_size=10)
        
        rospy.Subscriber('scan', LaserScan, self.scan_callback)
        rospy.Subscriber('odom', Odometry, self.odom_callback)
        
    
    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        N = int((scan_msg.angle_max - scan_msg.angle_min) // scan_msg.angle_increment)
        ttc_i = lambda i: scan_msg.ranges[i]/max(-self.speed*math.cos(scan_msg.angle_min+scan_msg.angle_increment*i+np.pi), 0.00001)
        ttc = min(ttc_i(i) for i in range(N))
        #print(scan_msg.angle_max)

        # TODO: publish brake message and publish controller bool
        if ttc < self.threshold:
            _brake = AckermannDriveStamped()
            #print("breaking!!!!")
            
            self.pub_brake_bool.publish(True)           
            self.pub_brake.publish(_brake)

def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()
