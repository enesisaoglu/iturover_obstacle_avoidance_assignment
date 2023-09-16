#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ReactiveRobot:
    def __init__(self):
        rospy.init_node("reactive_robot_node")
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(10)  # 10 Hz

    def scan_callback(self, scan_data):
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = 1.0
        self.cmd_vel.angular.z = 0.0
        
        ranges = scan_data.ranges
        center_min = min(ranges[0:45] + ranges[316:360])


        MaxRanges = [ 
        (max(ranges[0:45] + ranges[316:360])),
        ( max(ranges[270:315]) ),
        ( max(ranges[46:90]) ),
        ]
        print("--------------------------")
        print(center_min)
        if center_min < 1:
            max_angle = ranges[0]
            for i in range(1, len(ranges)):
                if ranges[i] > max_angle:
                    max_angle = ranges[i]

            self.cmd_vel.angular.z = -(max_angle - 90) * 0.1
            self.cmd_vel.linear.x = 0.5
        if center_min >= 1:
            self.cmd_vel.linear.x = 1

        self.pub.publish(self.cmd_vel)


if __name__ == "__main__":
    try:
        robot = ReactiveRobot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass