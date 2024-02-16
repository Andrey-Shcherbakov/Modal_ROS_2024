#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)#TODO: Subscribe to         
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)#TODO: Publish to drive
        
        #params of preprocess_lidar
        self.preproc_conv_size = 3
        self.preproc_max_dist = 10
        
        #params of find_best_point
        self.best_point_conv_size = 20
        self.prev_closest_point = 0.
        self.fur_point_weight = 0.45 #between 0 and 1 - position between furthest point and gap center
        
        #params of lidar_callback
        self.bubble_radius = 0.13#meters
        self.forward_speed = 18.0
        self.min_speed = 0.5
        self.tuning_angle = 6
        self.tuning_dist = 5
        
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m - high)
        """
        # we take the convolution to "smoothen" the obstacle form, I took this idea from this code
        # https://github.com/gonultasbu/f1tenth_bullet/blob/main/gap_follower.py
        
        proc_ranges = np.array(ranges)
        proc_ranges = np.convolve(proc_ranges, np.ones(self.preproc_conv_size), 'same') / self.preproc_conv_size #convolution = mean over a window of size
        proc_ranges = np.clip(proc_ranges, 0, self.preproc_max_dist)

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        # mask the bubble 
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        slices = np.ma.notmasked_contiguous(masked) #gaps
        
        max_len = slices[0].stop - slices[0].start
        max_gap = slices[0]
        # simple max gap search
        for sl in slices[1:]:
            sl_len = sl.stop - sl.start
            if sl_len > max_len:
                max_len = sl_len
                max_gap = sl
        return max_gap.start, max_gap.stop
    
    def find_best_point(self, start_i, end_i, ranges):
        """ Start_i & end_i are start and end indicies of max-gap range, respectively
            Return index of best point in ranges
            Naive: Choose the furthest point within ranges and go there
        """
        #I'll try to take the best of 2 solutions - i will make a convolution over a small window, take the furthest point
        # and take a middle point between the center of the gap (avoiding corners) and this point(planning to go next)
       
        mean_max_gap = np.convolve(ranges[start_i:end_i], self.best_point_conv_size,'same') / self.best_point_conv_size
        best_point = start_i + mean_max_gap.argmax()*self.fur_point_weight + (int)((end_i - start_i)/2)*(1 - self.fur_point_weight)
        return best_point
    
    def get_steering_angle(self, range_index, angle_increment):
        """ Lidar range index into angle
        """
        
        return range_index * angle_increment - np.pi/3
    
    def get_index(self, lidar_angle, range_len, angle_increment):
        """ Lidar angle to index
        """

        return (int)((lidar_angle + np.pi) // angle_increment)

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        right_ind = self.get_index(-np.pi/3, len(data.ranges), data.angle_increment)
        left_ind = self.get_index(+np.pi/3, len(data.ranges), data.angle_increment)
        #print(right_ind, left_ind)
        ranges = data.ranges[right_ind : left_ind]
        proc_ranges = self.preprocess_lidar(ranges)

        #Find closest point to LiDAR
        closest_point = proc_ranges.argmin()
        closest_point_angle = self.get_steering_angle(proc_ranges[closest_point], data.angle_increment)
        
        bubble_ind_rad = (int)(np.arctan(self.bubble_radius/proc_ranges[closest_point])/data.angle_increment)

        # Eliminate all points inside 'bubble' (set them to zero)
        min_index = max(closest_point - bubble_ind_rad, 0)
        max_index = min(closest_point + bubble_ind_rad, len(proc_ranges) - 1)
        
        proc_ranges[min_index:max_index] = 0 #bubble

        # Find max length gap
        gap_start, gap_end = self.find_max_gap(proc_ranges)
        
        self.fur_point_weight = 0.5 if proc_ranges[closest_point] > self.bubble_radius/10 else 0.4
        self.fur_point_weight += min(np.sign(proc_ranges[closest_point] - self.prev_closest_point)*0.1, 0.05)

        # Find the best point in the gap
        best_point = self.find_best_point(gap_start, gap_end, proc_ranges)
        
        # Publish Drive message
        steering_angle = self.get_steering_angle(best_point, data.angle_increment) * (1.2 if proc_ranges[closest_point] < self.bubble_radius/4 and proc_ranges[closest_point] < self.prev_closest_point else 1)
        self.prev_closest_point = proc_ranges[closest_point]
        
        
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = self.forward_speed*np.exp(-(abs(steering_angle))*self.tuning_angle)*math.tanh(abs(proc_ranges[closest_point] - self.bubble_radius)*(self.tuning_dist)) + self.min_speed # velocity
        #print('Steering angle in degrees: {}'.format((steering_angle / (np.pi / 2)) * 90))
        #print('Forward speed: {}'.format(drive_msg.drive.speed))
        self.drive_pub.publish(drive_msg)

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
