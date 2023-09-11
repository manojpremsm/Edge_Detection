#!/usr/bin/env python3

import rospy
from edge_service.srv import EdgeDetection, EdgeDetectionRequest
from sensor_msgs.msg import CompressedImage
import cv2
import argparse
from cv_bridge import CvBridge,CvBridgeError

def edge_detection_client(input_image_msg):
    rospy.wait_for_service('edge_detection')
    try:
        edge_detection = rospy.ServiceProxy('edge_detection', EdgeDetection)
        request = EdgeDetectionRequest()
        # Set the input image message in the request
        request.input_image = input_image_msg

        response = edge_detection(request)
        return response
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
   
        return None

def load_image_and_encode(image_path, cv_bridge):
    try:
        # Load the image using OpenCV
        image = cv2.imread(image_path)

        # Encode the image as JPEG
        encoded_image = cv_bridge.cv2_to_compressed_imgmsg(image, dst_format='png')

        return encoded_image
    except Exception as e:
        rospy.logerr("Error loading and encoding image: %s" % str(e))
        return None

if __name__ == '__main__':
    rospy.init_node('edge_detection_client')

    # Initialize the CvBridge to convert from cv2 to ros image format
    cv_bridge = CvBridge()

    # Load and encode the input image
    input_image_msg = load_image_and_encode("/home/ubuntu/catkin_ws/src/edge_detection_ros/edge_detection/data/Image_1.png", cv_bridge)

    if input_image_msg:
        result = edge_detection_client(input_image_msg)
        if result:
            rospy.loginfo("Edge detection service response: %s" % result.message)

        else:
            rospy.logerr("Edge detection service call failed")
    else:
        rospy.logerr("Image loading and encoding failed")




