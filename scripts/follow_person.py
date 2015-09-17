#!/usr/bin/env python
"""Ros node to make a robot follow person in front of it"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
import math

class FollowPerson(object):
    """Object to move the robot parallel to wall"""
    def __init__(self, should_init):
        if should_init:
            rospy.init_node('follow_person')
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.threshold = 0.05
        self.following_distance = 1
        self.speed_constant = 0.5
        self.scan_distances = []
        self.angles = range(-15,16)
        self.coords = [None] * len(self.angles)
        self.com = [0,1]

    def process_scan(self, msg):
        """Stores scans of those in front of it in list"""
        self.scan_distances = msg.ranges[-15:] + msg.ranges[:16]

    def calculate_coords(self):
        """Converts scans from radial readings to x,y from robots view"""
        for i in range(len(self.scan_distances)):
            if self.scan_distances[i] != 0:
                self.coords[i] = (self.scan_distances[i]*math.sin(math.radians(self.angles[i])),
                    self.scan_distances[i]*math.cos(math.radians(self.angles[i])))
                print self.scan_distances[i]
                print self.coords[i]
                print math.sin(math.radians(self.angles[i]))
            else:
                self.coords[i] = None

    def calculate_com(self):
        """Calculates center of mass of x,y readings"""
        coord_length = len([coord for coord in self.coords if coord is not None])
        com_x = 0
        com_y = 0
        for coord in self.coords:
            if coord != None:
                com_x += coord[0]
                com_y += coord[1]
        if coord_length != 0:
            self.com = (com_x/coord_length,com_y/coord_length)


    def should_turn(self):
        """Determines whether the robot is centered on target"""
        if self.com[0] > self.threshold:
            return self.com[0] * self.speed_constant
        elif self.com[0] < -self.threshold:
            return self.com[0] * self.speed_constant
        return 0

    def get_velocity(self):
        """Determines whether the robot is close enough"""
        off_distance = (self.com[1]-self.following_distance)
        return self.speed_constant * off_distance


    def run(self):
        """Runs the following person"""
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.calculate_coords()
            self.calculate_com()
            should_turn = self.should_turn()
            self.pub.publish(Twist(linear=Vector3(x=self.get_velocity()),
                                    angular=Vector3(z=self.should_turn())))
            r.sleep()

    def run2(self, on):
        """Same run program, without while loop"""
        if on:
            self.calculate_coords()
            self.calculate_com()
            self.pub.publish(Twist(linear=Vector3(x=self.get_velocity()),
                                    angular=Vector3(z=self.should_turn())))


if __name__ == '__main__':
    follower = FollowPerson(True)
    follower.run()
