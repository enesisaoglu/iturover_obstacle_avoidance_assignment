#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan

# Current Positions Of The Robot...
x = 0.0
y = 0.0
theta = 0.0  # Orientation Of The Robot In The Odometry Frame...

class ReactiveRobot:
    def __init__(self):
        # init node...
        rospy.init_node("reactive_robot_node")
        # subscribe to the LaserScan datas to get messages and manipulate...
        rospy.Subscriber("/odom", Odometry, self.newOdom)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        # publish the linear and angular velocity depends on the LaserScan datas...
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

        self.cmd_vel = Twist()
        self.goal = Point()
        self.max_angle = 0.0


    def newOdom(self, odomMsg):
        global x
        global y
        global theta
        
        # setting current positions and orientation of turtlebot in gazebo, a setting that possible in any time robot change its position...
        x = odomMsg.pose.pose.position.x
        y = odomMsg.pose.pose.position.y
        rot_q = odomMsg.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


    def scan_callback(self, scan_data):
        self.cmd_vel = Twist()
        # initial speeds...
        self.cmd_vel.linear.x = 0.5
        self.cmd_vel.angular.z = 0
        
        # Distances that laser beam hit for each angle...
        ranges = scan_data.ranges

        # Find the maximum distance to an obstacle in the front of the robot.
        measuredAngle = self.FindAngle(ranges)
        # If the robot is close to the goal, find the maximum distance to an obstacle
        # again and rotate towards that direction.
        if self.goal.y + 0.5 != y:
            self.SetSpeed(measuredAngle)
        elif self.goal.y + 0.5 == y:
            measuredAngle = self.FindAngle(ranges)
            self.SetSpeed(measuredAngle)
        else:
            self.SetSpeed(None)
    
        self.pub.publish(self.cmd_vel)
    
    # FindAngle is a method that finding the angle of lidar sending from robot to the max distance -
    # to make turtlebot rotate to that measured angle from current theta which is Z-axis, while it is going forward... 
    def FindAngle(self, ranges):

        # Min distance to obstacle in center view...
        center_min = min(ranges[0:45] + ranges[316:360])

        threshold = 0.8# Enough distance to obstacle...

        # if the robot in danger zone that means close enough to obstacle...
        if center_min <= threshold:
            # find max_angle in front ranges of the robot that we represent to rotate and go in that way...
            self.max_angle = None
            valueToCompare = ranges[270]
            for i in range(len(ranges)):
                if not (90 < i < 270):
                    if ranges[i] > valueToCompare:
                        valueToCompare = ranges[i]
                        self.max_angle = i
    
    
            # Find the obstacle coordinate in the max distance from robot... 
            max_distance = ranges[self.max_angle]
    
            if 0 <= self.max_angle <= 45: 
                self.goal.x = x + max_distance * math.cos(self.max_angle) 
                self.goal.y = y + max_distance * math.sin(self.max_angle)
            else:
                self.goal.x = x + max_distance * math.cos(self.max_angle - 270) 
                self.goal.y = y + max_distance * math.sin(self.max_angle - 270)
            
            if self.max_angle is not None:
                print("Max Angle: " + str(self.max_angle))
                print("Max Distance: " + str(valueToCompare))
                print("Obstacle Coordinate X-Y: " + str(self.goal.x) + "/" + str(self.goal.y))
                print("Robot Coordinate:" + str(x) + "/" + str(y) )
            else:
                print("No valid max angle found.")
            
            inc_x = self.goal.x - x
            inc_y = self.goal.y - y
            angle_to_goal = math.atan2(inc_x, inc_y)
            
            return angle_to_goal
            
    # SetSpeed is a method that to manipulate the movement of the turtlebot depends on the angle that is being measured...   
    def SetSpeed(self, angle):
        if angle == None:
           self.cmd_vel.linear.x = 0.5
           self.cmd_vel.angular.y = 0.0
        else:
           if abs(angle - theta) > 0.1:
                print("Rotating...")
                print("angletogoal:", angle)
                self.cmd_vel.linear.x = 0.1
                self.cmd_vel.angular.z = 0.5
           else:
                self.cmd_vel.linear.x = 0.5
                self.cmd_vel.angular.y = 0.0

        self.pub.publish(self.cmd_vel)
          



if __name__ == "__main__":
    robot = ReactiveRobot()
    rospy.spin()


