#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

command = rospy.Publisher('cmd_vel', Twist)


def callback(data):
    cmd_data = Twist()

    i = 0
    border = np.pi/12
    while (data.angle_min + i*data.angle_increment <= data.angle_max):
        #rospy.loginfo("range %s at %s", data.ranges[i], i*data.angle_increment)
        if(data.angle_min + i*data.angle_increment <= border
           and data.ranges[i] < 0.5):
                rospy.loginfo("Turning")
                cmd_data.angular.z = 0.1
                command.publish(cmd_data)
                return
        
        if(data.angle_min + i*data.angle_increment >= 2*np.pi - border
           and data.ranges[i] < 0.5):
                rospy.loginfo("Turning")
                cmd_data.angular.z = 0.1
                command.publish(cmd_data)
                return
        i+=1
    cmd_data.linear.x = 0.2
    rospy.loginfo("nothing to worry Moving forward")
    command.publish(cmd_data)


def navig_node():
    rospy.init_node('navig_node', anonymous=True)

    rospy.Subscriber('scan', LaserScan, callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        navig_node()
    except rospy.ROSInterruptException:
        pass
