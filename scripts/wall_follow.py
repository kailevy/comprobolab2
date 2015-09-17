#!/usr/bin/env python
"""Ros node to make a robot follow a wall parallel"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
import math

class WallFollow(object):
    """Object to move the robot parallel to wall"""
    def __init__(self, should_init):
        if should_init:
            rospy.init_node('wall_follow')
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_distances = {45:1,90:1,135:1}
        self.threshold = 0.01
        self.root_three = math.sqrt(3)
        self.half_root_three = math.sqrt(3)/2

    def process_scan(self, msg):
        """Stores scans at 3 angles in object"""
        self.scan_distances[45] = msg.ranges[44]
        self.scan_distances[90] = msg.ranges[89]
        self.scan_distances[135] = msg.ranges[134]

    def is_parallel(self):
        """Checks the ratios of the scans to determine if it's parallel"""
        if (self.scan_distances[45] == 0 or
            self.scan_distances[90] == 0 or
            self.scan_distances[135] == 0):
            return False
        root_three = self.scan_distances[45]/self.scan_distances[135]
        half_root_three = self.scan_distances[90]/self.scan_distances[135]
        return ((self.root_three-root_three < self.threshold or
                root_three-self.root_three < self.threshold) and
                (self.half_root_three-half_root_three < self.threshold or
                half_root_three-self.half_root_three < self.threshold))

    def run(self):
        """Moves forward if parallel; turns if not"""
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.is_parallel():
                self.pub.publish(Twist(linear=Vector3(x=0.5),angular=Vector3(z=0)))
            else:
                self.pub.publish(Twist(linear=Vector3(x=0),angular=Vector3(z=0.2)))
            r.sleep()

    def run2(self, on):
        """Same run program, without while loop"""
        if on:
            if self.is_parallel():
                self.pub.publish(Twist(linear=Vector3(x=0.5),angular=Vector3(z=0)))
            else:
                self.pub.publish(Twist(linear=Vector3(x=0),angular=Vector3(z=0.2)))


if __name__ == '__main__':
    wall_follower = WallFollow(True)
    wall_follower.run()
