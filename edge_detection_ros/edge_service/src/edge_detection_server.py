#!/usr/bin/env python3

import rospy
from edge_service.srv import EdgeDetection, EdgeDetectionResponse
import cv2
from sensor_msgs.msg import CompressedImage
import numpy as np

class EdgeDetectionServer:
    def __init__(self):
        rospy.init_node('edge_detection_server')
        self.service = rospy.Service('edge_detection', EdgeDetection, self.handle_edge_detection)
        self.image_sub = rospy.Subscriber('input_image', CompressedImage, self.image_callback)
        self.output_image_pub = rospy.Publisher('output_image', CompressedImage, queue_size=10)
        self.cv_image = None

    def image_callback(self, data):
        # Process the incoming image
       
        self.cv_image = cv2.imdecode(np.fromstring(data.data, np.uint8), cv2.IMREAD_COLOR)

    def handle_edge_detection(self, req):
        # Perform edge detection on self.cv_image
        edges = cv2.Canny(self.cv_image, 100, 200)
        # Convert the result back to CompressedImage format
        output_image_msg = CompressedImage(data=np.array(cv2.imencode('.jpg', edges)[1]).tostring())
        print("i am running")
        response = EdgeDetectionResponse()
        response.success = True
        response.message = "Edge detection  is finished successfully"
        response.output_image = output_image_msg

        return response

if __name__ == '__main__':
    EdgeDetectionServer()
    rospy.spin()
