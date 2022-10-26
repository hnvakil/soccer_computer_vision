import rclpy
from threading import Thread
from rclpy.node import Node
import time
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3
import math

class BallTracker(Node):
    """ The BallTracker is a Python object that encompasses a ROS node 
        that can process images from the camera and search for a ball within.
        The node will issue motor commands to move forward while keeping
        the ball in the center of the camera's field of view. """

    def __init__(self, image_topic):
        """ Initialize the ball tracker """
        super().__init__('ball_tracker')
        self.cv_image = None                        # the latest image from the camera
        self.hsv_image = None
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        self.create_subscription(Image, image_topic, self.process_image, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.red_lower_bound = 150
        self.green_lower_bound = 40
        self.blue_lower_bound = 0
        self.red_upper_bound = 255
        self.green_upper_bound = 100
        self.blue_upper_bound = 50
        self.h_lower_bound = 0
        self.s_lower_bound = 176
        self.v_lower_bound = 0
        self.h_upper_bound = 12
        self.s_upper_bound = 255
        self.v_upper_bound = 255
        self.turn_speed = 0.0
        self.drive_speed = 0.0
        self.center_x = 0
        self.center_y = 0
        thread = Thread(target=self.loop_wrapper)
        thread.start()

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
    

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        cv2.namedWindow('video_window')
        cv2.namedWindow('binary_window')
        cv2.namedWindow('image_info')
        self.red_lower_bound = 0
        cv2.createTrackbar('red lower bound', 'binary_window', self.red_lower_bound, 255, self.set_red_lower_bound)
        cv2.createTrackbar('red upper bound', 'binary_window', self.red_upper_bound, 255, self.set_red_upper_bound)
        cv2.createTrackbar('green lower bound', 'binary_window', self.green_lower_bound, 255, self.set_green_lower_bound)
        cv2.createTrackbar('green upper bound', 'binary_window', self.green_upper_bound, 255, self.set_green_upper_bound)
        cv2.createTrackbar('blue lower bound', 'binary_window', self.blue_lower_bound, 255, self.set_blue_lower_bound)
        cv2.createTrackbar('blue upper bound', 'binary_window', self.blue_upper_bound, 255, self.set_blue_upper_bound)
        cv2.setMouseCallback('video_window', self.process_mouse_event)
        cv2.namedWindow('hsv_window')
        cv2.createTrackbar('h lower bound', 'hsv_window', self.h_lower_bound, 180, self.set_h_lower_bound)
        cv2.createTrackbar('h upper bound', 'hsv_window', self.h_upper_bound, 180, self.set_h_upper_bound)
        cv2.createTrackbar('s lower bound', 'hsv_window', self.s_lower_bound, 255, self.set_s_lower_bound)
        cv2.createTrackbar('s upper bound', 'hsv_window', self.s_upper_bound, 255, self.set_s_upper_bound)
        cv2.createTrackbar('v lower bound', 'hsv_window', self.v_lower_bound, 255, self.set_v_lower_bound)
        cv2.createTrackbar('v upper bound', 'hsv_window', self.v_upper_bound, 255, self.set_v_upper_bound)
        
        while True:
            self.run_loop()
            time.sleep(0.1)

    def set_red_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.red_lower_bound = val

    def set_red_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red upper bound """
        self.red_upper_bound = val

    def set_green_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the green lower bound """
        self.green_lower_bound = val

    def set_green_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the green upper bound """
        self.green_upper_bound = val

    def set_blue_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the blue lower bound """
        self.blue_lower_bound = val

    def set_blue_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the blue upper bound """
        self.blue_upper_bound = val

    def set_h_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.h_lower_bound = val

    def set_h_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.h_upper_bound = val
    
    def set_s_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.s_lower_bound = val
    
    def set_s_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.s_upper_bound = val
    
    def set_v_upper_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.v_upper_bound = val
    
    def set_v_lower_bound(self, val):
        """ A callback function to handle the OpenCV slider to select the red lower bound """
        self.v_lower_bound = val
    
    

    def process_mouse_event(self, event, x,y,flags,param):
        """ Process mouse events so that you can see the color values
            associated with a particular pixel in the camera images """
        self.image_info_window = 255*np.ones((500,500,3))
        cv2.putText(self.image_info_window,
                    'Color (b=%d,g=%d,r=%d)' % (self.cv_image[y,x,0], self.cv_image[y,x,1], self.cv_image[y,x,2]),
                    (5,50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0,0,0))
        cv2.putText(self.image_info_window,
                    'Color (h=%d,s=%d,v=%d)' % (self.hsv_image[y,x,0], self.hsv_image[y,x,1], self.hsv_image[y,x,2]),
                    (20, 200),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0,0,0))    

    def run_loop(self):
        msg = Twist()
        # NOTE: only do cv2.imshow and cv2.waitKey in this function 
        if not self.cv_image is None:
            self.binary_image = cv2.inRange(self.cv_image, (self.blue_lower_bound,self.green_lower_bound,self.red_lower_bound), (self.blue_upper_bound,self.green_upper_bound,self.red_upper_bound))
            self.binary_hsv = cv2.inRange(self.hsv_image, (self.h_lower_bound, self.s_lower_bound, self.v_lower_bound), (self.h_upper_bound, self.s_upper_bound, self.v_upper_bound))
            moments = cv2.moments(self.binary_hsv)
            if moments['m00'] != 0:
                self.center_x, self.center_y = moments['m10']/moments['m00'], moments['m01']/moments['m00']
            print(self.center_x)
            print(self.center_y)
            self.turn_speed = (self.center_x - 500) / -1000 * 3
            msg.angular.z = self.turn_speed
            if abs(self.center_x - 500) < 100:
                msg.linear.x = 0.0
            else:
                msg.linear.x = 0.0
            #self.pub.publish(msg)
            print(self.cv_image.shape)
            self.bin_with_circle = cv2.circle(self.binary_hsv, (math.floor(self.center_x), math.floor(self.center_y)), radius = 20, color = (0,100,50), thickness = 5)
            cv2.imshow('video_window', self.cv_image)
            #cv2.imshow('binary_window', self.binary_image)
            cv2.imshow('bin_with_circle', self.bin_with_circle)
            cv2.imshow('hsv_window', self.binary_hsv)
            if hasattr(self, 'image_info_window'):
                cv2.imshow('image_info', self.image_info_window)
            cv2.waitKey(5)

if __name__ == '__main__':
    node = BallTracker("/camera/image_raw")
    node.run()


def main(args=None):
    rclpy.init()
    n = BallTracker("camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()