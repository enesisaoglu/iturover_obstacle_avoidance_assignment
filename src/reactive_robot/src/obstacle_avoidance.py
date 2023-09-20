#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import DeleteLight, DeleteLightRequest

class ReactiveRobot:
    def __init__(self):
        # init node...
        rospy.init_node("reactive_robot_node")
        # subscribe to the LaserScan datas to get messages and manipulate...
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        # publish the linear and angular velocity depends on the LaserScan datas...
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # A service client for deleting the Gazebo light which is called sun...
        self.light_delete_client = rospy.ServiceProxy("/gazebo/delete_light", DeleteLight)
        self.rate = rospy.Rate(10)  # 10 Hz
        
        self.cmd_vel = Twist()
        # when this node has been worked, record the time...
        self.start_time = rospy.get_rostime()  
        # End time...
        self.end_time = self.start_time + rospy.Duration(60) 
    
    def scan_callback(self, scan_data):
        self.cmd_vel = Twist()
        # initial speeds...
        self.cmd_vel.linear.x = 0.5
        self.cmd_vel.angular.z = 0.0
        
        # Distances that laser beam hit for each angle...
        ranges = scan_data.ranges

        # Distances that is being hit by each laser beam that is being send from right, left and center of the robot with determined angles...
        front_center_obstacle_distance = min(ranges[0:10] + ranges[350:360])
        front_obstacle_distance = np.mean( min(ranges[0:30] + ranges[330:360]) )
        right_obstacle_distance = np.mean(ranges[270:360])
        left_obstacle_distance = np.mean(ranges[0:90])
        
        # Current time...
        current_time = rospy.get_rostime()
        threshold = 2.3
        print("Current Time:", current_time)
        if current_time < self.end_time:
            if front_center_obstacle_distance < threshold:
                if right_obstacle_distance > left_obstacle_distance:
                    print("There is an obstacle, Robot is rotating to the right...")
                    self.cmd_vel.linear.x =  0.2
                    self.cmd_vel.angular.z = -0.4
                else:
                    print("There is an obstacle, Robot is rotating to the left...")
                    self.cmd_vel.linear.x =  0.2
                    self.cmd_vel.angular.z = 0.4
            else:
                print("No obstacle, keep moving forward...")
                self.cmd_vel.linear.x = 0.7
                self.cmd_vel.angular.z = 0.0
        else:
            # Stop robot motion
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            rospy.loginfo("Time limit reached. Stopping...")
        
            # Delete the Gazebo light
            light_delete_req = DeleteLightRequest()
            light_delete_req.light_name = "sun"  # Specify the light name to delete
            try:
                self.light_delete_client(light_delete_req)
                rospy.loginfo("Light in Gazebo environment is removed.")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call to delete_light failed: {e}")

            
            self.scan_sub.unregister()
            rospy.loginfo("/scan topic is unsubscribed.")


        # publish the velocity of the robot on the '/cmd_vel' topic as Twist...
        self.pub.publish(self.cmd_vel)
    

if __name__ == "__main__":
    robot = ReactiveRobot()
    rospy.spin()


