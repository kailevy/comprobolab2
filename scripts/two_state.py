#!/usr/bin/env python
"""Ros node to switch between wall following and person following"""

import rospy
from sensor_msgs.msg import LaserScan
from follow_person import FollowPerson
from wall_follow import WallFollow

class TwoState(object):
    """Switches between wall following and person following if there's a person in front of it"""
    def __init__(self):
        rospy.init_node('follow_person')
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.scan_front = []
        self.person_follower = FollowPerson(False)
        self.wall_follower = WallFollow(False)

    def process_scan(self, msg):
        """Processes scan data in front of it"""
        self.scan_front = msg.ranges[-10:] + msg.ranges[:11]

    def person_in_front(self):
        """Decides whether there is a person in front of it that it cant follow"""
        not_clear = [scan != 0 and scan < 3 for scan in self.scan_front]
        for bool in not_clear:
            if bool == True:
                return True
        return False

    def run(self):
        """Calls the other object's runs to switch states"""
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.wall_follower.run2(True)
            if self.person_in_front():
                self.wall_follower.run2(False)
                self.person_follower.run2(True)
            else:
                self.person_follower.run2(False)
                self.wall_follower.run2(True)
            r.sleep()


if __name__ == '__main__':
    two_state = TwoState()
    two_state.run()
