#!/usr/bin/env python
"""Ros node to make a robot move while avoiding obstacles. Questionable reliability,
    especially when it has already moved and thus has prestored odom"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3, Quaternion, Point
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion

def convert_pose_to_xy_and_theta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return pose.position.x, pose.position.y, angles[2]

class AvoidObstacle(object):
    """Class to avoid obstacles"""
    def __init__(self, distance):
        rospy.init_node('avoid_obstacles')
        rospy.Subscriber('/scan', LaserScan, self.process_scan, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.process_odom, queue_size=1)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.distance = distance
        self.threshold = 0.02
        self.scans = {-1:[],0:[],1:[]}
        self.turning = False
        self.turned = 0
        self.init_angle = 0
        self.angle = None

    def process_scan(self, msg):
        """Processes the scan based on what state the robot is in"""
        if self.turned == 0:
            self.scans[0] = msg.ranges[-10:] + msg.ranges[:11]
            self.scans[-1] = msg.ranges[250:281]
            self.scans[1] = msg.ranges[80:101]
        elif self.turned == 1:
            self.scans[0] = msg.ranges[260:281]
            self.scans[-1] = msg.ranges[170:191]
            self.scans[1] = msg.ranges[-10:] + msg.ranges[:11]
        elif self.turned == -1:
            self.scans[0] = msg.ranges[80:101]
            self.scans[-1] = msg.ranges[-10:] + msg.ranges[:11]
            self.scans[1] = msg.ranges[170:191]

    def process_odom(self, msg):
        """Processes the odom"""
        self.angle = convert_pose_to_xy_and_theta(msg.pose.pose)[2]
        if self.init_angle == None:
            self.init_angle = self.angle
        # print self.angle


    def is_clear(self,direction):
        """Checks if a direction is clear"""
        clear = [scan == 0 or scan > self.distance for scan in self.scans[direction]]
        print clear
        for bool in clear:
            if bool == False:
                return False
        return True

    def turn_ninety(self, direction):
        """Turns ninety degrees"""
        self.turning = True
        if direction == 1:
            if self.angle < self.init_angle + math.radians(90):
                self.pub.publish(Twist(angular=Vector3(z=0.2)))
            else:
                self.turning = False
                self.turned = direction
        elif direction == -1:
            if self.angle > self.init_angle - math.radians(90):
                self.pub.publish(Twist(angular=Vector3(z=-0.2)))
            else:
                self.turning = False
                self.turned = direction

    def turn_to_pref(self):
        """Turns back to initial angle"""
        self.turning = True
        if abs(self.angle) - abs(self.init_angle) > self.threshold:
            self.pub.publish(Twist(angular=Vector3(z=self.turned * -0.2)))
        else:
            self.turning = False
            self.turned = 0

    def move_forward(self):
        """Moves forward"""
        self.pub.publish(Twist(linear=Vector3(x=0.5)))

    def run(self):
        """Runs loop, complicated conditional statements...."""
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            print self.turned
            print self.turning
            print self.scans
            if self.turned == 0 and self.is_clear(0) and not self.turning:
                self.move_forward()
            elif self.turned == 0:
                if self.is_clear(1):
                    print 'turn 1'
                    self.turn_ninety(1)
                elif self.is_clear(-1):
                    print 'turn -1'
                    self.turn_ninety(-1)
            else:
                if self.is_clear(0):
                    print 'back'
                    self.turn_to_pref()
                elif self.turning:
                    self.turning = False
                else:
                    self.move_forward()
            r.sleep()

if __name__ == '__main__':
    obstacle_avoider = AvoidObstacle(0.8)
    obstacle_avoider.run()
