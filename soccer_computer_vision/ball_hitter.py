from tkinter.ttk import setup_master
import rclpy
import time
import numpy as np
import math
import tty
import select
import sys
import termios
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker



class BallHitterNode(Node):
    #states:
    # 0: looking for person
    # 1: approaching person
    # 2: 
    def __init__(self):
        super().__init__('person_follower')
        self.create_timer(0.1, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, 'marker', 10)
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.ball_x = 0.0
        self.ball_y = 0.0
        self.finished_setup = False
        self.slope = 0.0
        self.theta_g = 0.0
        self.theta_1 = 0.0
        self.p2 = [0.0, 0.0]
        self.p1 = [0.0, 0.0]
        self.d_theta_1 = 0.0
        self.d_theta_2 = 0.0
        self.d_1 = 0.0
        self.n = 0.5 #m
        self.ball_marker = Marker()
        self.goal_marker = Marker()
        self.dest_marker = Marker()


    def find_objects(self):
        self.goal_x = 3.0
        self.goal_y = 2.0
        self.ball_x = 2.0
        self.ball_y = 1.0
    
    def do_setup(self):
        #identify the locations of ball and goal
        self.find_objects()
        print(self.goal_x)
        print(self.ball_x)

        #Find the slope of the line between the goal and the ball in current Neato frame
        self.slope = (self.goal_y - self.ball_y) / (self.goal_x - self.ball_x)

        #find the angle that defines that slope, the final heading necessary to kick the ball
        self.theta_g = math.atan2(self.goal_y - self.ball_y, self.goal_x - self.ball_x)
        
        #the xy location of the ball
        self.p2 = [self.ball_x, self.ball_y]
        self.pb = self.p2
        print(self.p1)
        print(self.p2)

        #Calculate the kicking point behind the ball
        self.p1[0] = self.p2[0] + self.n * (-1 * math.cos(self.theta_g)) 
        self.p1[1] = self.p2[1] + self.n * (-1 * math.sin(self.theta_g))
        
        self.theta_1 = math.atan2(self.p1[1], self.p1[0])
        self.d_theta_1 = math.pi / 2 - self.theta_1
        self.d1 = math.sqrt(self.p1[0] ** 2 + self.p1[1] ** 2)
        self.d_theta_2 = self.theta_g - self.theta_1
        self.setup_markers()
        self.finished_setup = True



    def setup_markers(self):
        self.setup_marker(self.ball_marker, self.ball_x, self.ball_y, 0)
        self.setup_marker(self.goal_marker, self.goal_x, self.goal_y, 1)
        self.setup_marker(self.dest_marker, self.p1[0], self.p1[1], 2)

    
    def setup_marker(self, marker, x, y, num):
        marker.header.frame_id="base_link"
        marker.ns = "basic_shapes"
        marker.id = num
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.g = 1.0
        marker.color.r = 0.5
        marker.pose.position.z = 0.0
        marker.pose.position.y = y
        marker.pose.position.x = x
        self.marker_pub.publish(marker)

    def turn(self, angle):
        msg = Twist()
        speed = 0.5
        angle_sign = angle / abs(angle)
        time_to_go = abs(angle) / speed
        msg.angular.z = angle_sign * speed
        self.vel_pub.publish(msg)
        time.sleep(time_to_go)
        msg.angular.z = 0.0
        self.vel_pub.publish(msg)
    
    def drive(self, dist):
        msg = Twist()
        speed = 0.2
        time_to_go = dist / speed
        msg.linear.x = speed
        self.vel_pub.publish(msg)
        time.sleep(time_to_go)
        msg.linear.x = 0.0
        self.vel_pub.publish(msg)



    def run_loop(self):
        if not self.finished_setup:
            self.do_setup()
        print(f"d_theta_1: {self.d_theta_1}, d1: {self.d1}, d_theta_2: {self.d_theta_2}, n: {self.n}")
        self.turn(self.d_theta_1)
        self.drive(self.d1)
        self.turn(self.d_theta_2)
        self.drive(self.n)
        self.drive(0.01)
        time.sleep(100)

        # some sort of reference frame issue - is x and y the same in our model and atan2?

        

    def process_scan(self, msg):
        self.scan = msg.ranges
    

def main(args=None):
    rclpy.init(args=args)
    node = BallHitterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
