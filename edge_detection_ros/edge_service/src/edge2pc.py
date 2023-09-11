#!/usr/bin/env python3


import cv2
import numpy as np
import rospy
import roslib
from sensor_msgs.msg import Image,CameraInfo,PointCloud
from geometry_msgs.msg import Point32
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from cv_bridge import CvBridge,CvBridgeError



class image_test:

    
    cv_rgbimage = None
    cv_depthimage =None
    binary_mask = None
    fx = 0
    fy = 0
    cx = 0
    cy = 0
    height = 0
    width = 0

    def __init__(self):

        
        self.img_pub = rospy.Publisher("ouput_image",Image)
        self.point_pub = rospy.Publisher("edge_points",PointCloud)
        self.img_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback_image,queue_size=1)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw",Image,self.callback_depth,queue_size=1)
        self_camerainfo = rospy.Subscriber("/camera/color/camera_info",CameraInfo,self.callback_caminfo,queue_size=1)
        self.bridge = CvBridge()
       

    def callback_image(self,data):
        
        
        try:
            
            image_test.cv_rgbimage= data
            cv_rgbimage = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
        gray = cv2.cvtColor(cv_rgbimage,cv2.COLOR_BGR2GRAY)
        #detecting edges using canny filter
        edge = cv2.Canny(gray, 230, 250)
        edge_rgb = cv2.cvtColor(edge,cv2.COLOR_GRAY2BGR)
        image_test.binary_mask = np.zeros_like(edge)
        image_test.binary_mask[edge > 0] = 255

        edge_rgb = edge_rgb * np.array((0,1,0),np.uint8)
        #cv2.namedWindow("outpu_Image_window",cv2.WINDOW_NORMAL)

        #cv2.imshow("outpu_Image_window", edge_rgb)
        
        #cv2.waitKey(3)

        try:
            #publishing the detected edges in green to the topic ouput_image
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(edge))
        except CvBridgeError as e:
            print(e)
    
    def callback_depth(self,data):
       #call back function for accessing depth image from the depth image topic 
        try:
            
            image_test.cv_depthimage_str = data
            image_test.cv_depthimage = self.bridge.imgmsg_to_cv2(image_test.cv_depthimage_str)
        except CvBridgeError as e:
            print(e)
        
        
    def callback_caminfo(self,caminfo):
       #camera intrinsics taken from topic camera_info based on the plumb_bob distortion model .
        

        image_test.fx = caminfo.K[0]
        image_test.fy = caminfo.K[4]
        image_test.cx = caminfo.K[2]
        image_test.cy = caminfo.K[5]
        
        image_test.height = caminfo.height
        image_test.width = caminfo.width
        



    def main_func(self):
        point_cloud = []
        point_cloud1 = []
        
        for y in range(image_test.height):
            for x in range(image_test.width):
                if image_test.binary_mask[y, x] == 255: # Check if it's an edge pixel
                    # Convert pixel coordinates to camera coordinates i.e., 2d to 3d 
                    
                    x_camera = (x - image_test.cx) * image_test.cv_depthimage[y, x] / image_test.fx
                    y_camera = (y - image_test.cy) * image_test.cv_depthimage[y, x] / image_test.fy
                    z_camera = image_test.cv_depthimage[y, x]
                    point_cloud.append(Point32(x_camera, y_camera, z_camera))
                    point_cloud1.append([x_camera, y_camera, z_camera])
        

        point_cloud1 = np.array(point_cloud1)
        #rospy.loginfo(f"this is the value of point cloud {point_cloud1}")
        num_points = point_cloud1.shape[0]
        #print("number of point cloud :", num_points)
        if image_test.height !=0:

            fig = plt.figure(figsize=(12, 6))
            
            # Display the RGB image
            ax1 = fig.add_subplot(121)
            try:
                rgb_image_np = self.bridge.imgmsg_to_cv2(image_test.cv_rgbimage, "bgr8")
                ax1.imshow(cv2.cvtColor(rgb_image_np, cv2.COLOR_BGR2RGB))
            except CvBridgeError as e:
                print(e)
            ax1.set_title('RGB Image')
            
            # Display the 3D point cloud
            ax2 = fig.add_subplot(122, projection="3d")
            ax2.scatter(point_cloud1[:, 0], point_cloud1[:, 1], point_cloud1[:, 2], s=1)
            ax2.set_xlabel("x")
            ax2.set_ylabel("y")
            ax2.set_zlabel("z")
            ax2.set_title('3D Point Cloud')

            plt.show()


            
            
            
        #writing the points into the pointcloud message along with the header containing time stamp and frame id.
        pc_mgs = PointCloud()
        pc_mgs.header.stamp = rospy.Time.now()
        pc_mgs.header.frame_id = "camera_color_optical_frame"
        pc_mgs.points = point_cloud
        print("pc_msg is of type", type(pc_mgs))
        self.point_pub.publish(pc_mgs)
        
        print("depth")

    def run_func(self):
        rate = rospy.Rate(1)
        try:
            while not rospy.is_shutdown():

                self.main_func()
                print("inside the run function")
                rate.sleep()
                #rospy.spin()
        except rospy.ROSInterruptException:
            pass

       


if __name__ == '__main__':
    #main(sys.argv)
    rospy.init_node('image_test', anonymous=True)
    ic = image_test()
    ic.run_func()

    
    
    
