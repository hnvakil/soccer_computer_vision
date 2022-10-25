from tkinter.ttk import setup_master
import rclpy
import time
import numpy as np
import math
import tty
import select
import sys
import termios
import cv2
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image
from cv_bridge import CvBridge




class BallHitterNode(Node):
    #states:
    # 0: looking for person
    # 1: approaching person
    # 2: 
    def __init__(self, image_topic):
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
        self.hit_ball = False
        self.robot_pose = None
        self.create_subscription(Pose, 'currentpose', self.read_pose, 10)

        # open cv stuff:
        self.cv_image = None                        # the latest image from the camera
        self.hsv_image = None
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        self.image_center = 0                       # the x coordinate of the middle of the image

        self.create_subscription(Image, image_topic, self.process_image, 10)
        self.h_lower_bound = 0
        self.s_lower_bound = 195
        self.v_lower_bound = 50
        self.h_upper_bound = 10
        self.s_upper_bound = 255
        self.v_upper_bound = 255

    def read_pose(self,msg):
        self.robot_pose = msg

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

    def assign_objects(self):
        self.goal_x = 3.0
        self.goal_y = 2.0
        self.ball_x = 2.0
        self.ball_y = 1.0

    def find_objects(self):
        ball_seen = 0
        goal_seen = 0

        #start turning to look for ball
        self.search()
        while not ball_seen:
            self.detect_ball()
            if self.image_x == self.image_center: 
                ball_seen = 1
        self.stop()

        #make note of current heading
        ball_heading = self.heading_from_pose(self.robot_pose)

        #estimate distance
        ball_distance = self.find_ball_distance() 

        #spin until goal seen
        self.search()
        while not goal_seen:
            pass
        self.stop()

        #make note of angle change
        goal_heading = self.heading_from_pose(self.robot_pose) 
        angle_to_ball = goal_heading - ball_heading

        #estimate distance
        goal_distance = self.find_goal_distance()

        #convert from polar to cartesian
        [self.ball_x, self.ball_y] = self.polar_to_cartesian(ball_distance, angle_to_ball) 
        [self.goal_x, self.goal_y] = self.polar_to_cartesian(goal_distance, 0) 

    def detect_ball(self):
        #I ran this function by itself in the run loop and it worked -Liv
        cv2.namedWindow('video_window')
        cv2.namedWindow('masked_window')

        if not self.cv_image is None:
            self.binary_hsv = cv2.inRange(self.hsv_image, (self.h_lower_bound, self.s_lower_bound, self.v_lower_bound), (self.h_upper_bound, self.s_upper_bound, self.v_upper_bound))
            print(type(self.binary_hsv))
            print(self.binary_hsv)
            contours, _ = cv2.findContours(self.binary_hsv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            blob = max(contours, key=lambda el: cv2.contourArea(el))
            M = cv2.moments(blob)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            self.image_x = int(M["m10"] / M["m00"])
            
            canvas = self.binary_hsv.copy()
            cv2.circle(canvas, center, 20, (50,50,50), -1)
            cv2.imshow('masked_window', canvas)
            cv2.imshow('video_window', self.cv_image)
        if hasattr(self, 'image_info_window'):
            cv2.imshow('image_info', self.image_info_window)
        cv2.waitKey(5)
    
    def polar_to_cartesian(self,d,theta):
        x = d * math.cos(theta)
        y = d * math.sin(theta)
        return [x,y]

    def heading_from_pose(self, pose):
        """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
        orientation_tuple = (pose.orientation.x,
                             pose.orientation.y,
                             pose.orientation.z,
                             pose.orientation.w)
        angles = self.euler_from_quaternion(*orientation_tuple)
        return angles[2]
    
    def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
    
    def search(self):
        msg = Twist()
        speed = 0.5
        msg.angular.z = speed
        self.vel_pub.publish(msg)

    def stop(self):
        msg = Twist()
        speed = 0.0
        msg.angular.z = speed
        self.vel_pub.publish(msg)

    def do_setup(self):
        #identify the locations of ball and goal
        self.assign_objects() #change this once find_objects is actually made
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
        
        #Calculate the angle towards kicking point
        self.theta_1 = math.atan2(self.p1[1], self.p1[0])

        #Calculate the amount to turn to reach theta1
        self.d_theta_1 = self.theta_1
        
        #Calculate distance to kicking point
        self.d1 = math.sqrt(self.p1[0] ** 2 + self.p1[1] ** 2)
        
        #Calculate the amount that we need to turn to reach theta_g
        self.d_theta_2 = self.theta_g - self.theta_1
        
        #setup
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
        marker.color.b = 0.5
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
    
    def check_lined_up_for_shot(self):
        return True



    def run_loop(self):

        """
        while not self.hit_ball:
            if not self.finished_setup:
                self.do_setup()
            print(f"d_theta_1: {self.d_theta_1}, d1: {self.d1}, d_theta_2: {self.d_theta_2}, n: {self.n}, theta1: {self.theta_1}")
            self.turn(self.d_theta_1)
            self.drive(self.d1)
            self.turn(self.d_theta_2)
            if self.check_lined_up_for_shot():
                self.drive(self.n)
                self.hit_ball = True
                self.drive(0.01)
                time.sleep(100)
            else: 
                self.finished_setup = False
        """
        
        

    def process_scan(self, msg):
        self.scan = msg.ranges
    

def main(args=None):
    rclpy.init(args=args)
    node = BallHitterNode("camera/image_raw")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
