#!/usr/bin/env python3
from re import T
import rclpy
import threading
import time
from rclpy.node import Node

#from numpy import rate
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt





class TurtleGoToGoal(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.velocity_publisher= self.create_publisher(Twist,'/turtle1/cmd_vel',10)
        self.pose_subscriber = self.create_subscription(Pose,'/turtle1/pose',self.update_pose,10)
        
        #self.rate = Node.create_rate(10,self.move2goal())
        #self.rate = rclpy.create_node("node").create_rate(10)
        #node = rclpy.create_node('simple_node')
        #self.rate = node.create_rate(10)
        #thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
        #thread.start()
        
        #timer_period = 0.5
        #self.timer = rclpy.timer.Rate(10,  context = None)

        print('after timer')

        t = threading.Thread(target=rclpy.spin)

        self.pose = Pose()

    def update_pose(self,data):

        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        #self.get_logger().info(data)

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)
        
    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
        
    def angular_vel(self, goal_pose, constant=6):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)
        

    def move2goal(self):
        print("in mov2gol")
        goal_pose = Pose()
        goal_pose.x = float(input("Set your x goal: "))
        goal_pose.y = float(input("Set your y goal: "))

        distance_tolerance = float(input("Set your tolerance: "))

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            print("in while")
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            
            self.velocity_publisher.publish(vel_msg)
            print("before sleep")
            #self.timer.sleep()
            #time.sleep(10)
            print("after sleep")
            
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)



def main(args = None):
    rclpy.init(args = args)
    print('in node')
    node = TurtleGoToGoal()
    node.move2goal()

    #node.move2goal()

    rclpy.spin(node)

    
    node.destroy_node()
    rclpy.shutdown()
    #thread.join()

if __name__ == "__main__":
    print('in main')
    main()