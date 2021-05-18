#!/usr/bin/env python

#import rospy
import rclpy
import cv2
import sys
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class ROSVideoPublisher(object):
    def __init__(self, resource=0, topic='raw_image', visualize=True):
        self.visualize = False
        rclpy.init()
        self.node = rclpy.create_node("publisher")
        self.pub = self.node.create_publisher(Image, topic, 100)
        #self.pub = rospy.Publisher(topic, Image, queue_size=100)
        #rospy.init_node('image_publisher', anonymous=True)
        # rate = rospy.Rate(10) # not sure if needed
        self.bridge = CvBridge()
        resource = 0
        self.resource_name = "/dev/video" + str(resource)
        resource = int(resource)
        print("Here: ", resource)

        print("Trying to open resource: " + self.resource_name)
        print("Resource: ", resource)
        self.cap = cv2.VideoCapture(resource)

        if not self.cap.isOpened():
            print("Error opening resource: " + str(resource))
            exit(0)

    def publish(self):
        print("Correctly opened resource, starting to show feed.")
        rval, frame = self.cap.read()
        while rval:
            if self.visualize:
                cv2.imshow("Stream: " + self.resource_name, frame)
            rval, frame = self.cap.read()

            # ROS image conversion
            if frame is not None:
                frame = np.uint8(frame)
                print(frame)
                image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                print("Publishing another frame...")
                self.pub.publish(image_message)
            else:
                print("FRAME EMPTY")

            if self.visualize:
                key = cv2.waitKey(1)
                # print "key pressed: " + str(key)
                if key == 27 or key == 1048603:
                    break
        cv2.destroyWindow("preview")


if __name__ == '__main__':

    try:
        rp = ROSVideoPublisher(visualize=True)
        rp.publish()
    except:
        pass
