#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ReactiveRobot:
    def __init__(self):
        # init node...
        rospy.init_node("reactive_robot_node")
        # subscribe to the LaserScan datas to get messages and manipulate...
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        # publish the linear and angular velocity depends on the LaserScan datas...
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(10)  # 10 Hz

    def scan_callback(self, scan_data):
        self.cmd_vel = Twist()
        # initial speeds...
        self.cmd_vel.linear.x = 1.0
        self.cmd_vel.angular.z = 0
        
        ranges = scan_data.ranges
        # Min distance to obstacle in center view...
        center_min = min(ranges[0:45] + ranges[316:360])

        # Max distances to obstacle in center, right and left views...
        MaxRanges = [ (max(ranges[0:45] + ranges[316:360])),
        ( max(ranges[270:315]) ),
        ( max(ranges[46:90]) ),
        ]
        # find max_angle in ranges to rotate and go in that way...
        max_angle = ranges[0]
        for i in range(1, len(ranges)):
            if ranges[i] > max_angle:
                max_angle = ranges[i]
        
        print("-----------------------")
        print(center_min)
        if center_min < 1:
            # rotate to the max angle...
            self.cmd_vel.angular.z = -(max_angle - 90) * 0.02
            self.cmd_vel.linear.x = 0.5 #keep going while rotating...
        else:
            self.cmd_vel.linear.x = 1.0
            self.cmd_vel.angular.z = 0

        self.pub.publish(self.cmd_vel)


if __name__ == "__main__":
    try:
        robot = ReactiveRobot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass