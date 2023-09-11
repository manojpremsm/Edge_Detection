#!/usr/bin/env python3


import cv2
import argparse
import numpy as np

class EdgeDetector:

    def __init__(self,input_image,superimposed_image,edge_image,threshold1,threshold2) -> None:
        self.input_image = input_image
        self.output_image = superimposed_image
        self.edge_image = edge_image
        self.threshold1 = threshold1
        self.threshold2 = threshold2

    def edge_detection(self):

        img = cv2.imread(self.input_image)
        #convert the color image into gray scale for edge detection
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
        #gaussian filter to filter out the noises
        filtered_img = cv2.GaussianBlur(gray, (3, 3), 0)
        #canny edge detection
        edges = cv2.Canny(filtered_img,self.threshold1,self.threshold2)
        #saving the detected edges in green
        edge_rgb = cv2.cvtColor(edges,cv2.COLOR_GRAY2BGR)
        edge_rgb = edge_rgb * np.array((0,1,0),np.uint8)
        cv2.imwrite(self.edge_image,edge_rgb)

        # Superimpose edges on the input image
        result = img
        result[np.where(edges != 0)] = [0, np.clip(int(255*10),0,255), 0]
       
        cv2.imwrite(self.output_image,result)
        print(f"Edge image saved in {self.output_image}")
        cv2.imshow("reslut",result)
        cv2.waitKey(0)
        
        
if __name__ =='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('input_image',help="please add the input file path ")
    parser.add_argument('superimposed_image_output',help="please add the path where the superimposed image should be saved ")
    parser.add_argument('edge_image',help="please add the path where the edge image should be saved ")
    parser.add_argument('threshold1',type=int,default=200,help="lower threshold for detecting edges ")
    parser.add_argument('threshold2',type=int,default=250,help="higher threshold for detecting edges")
    args = parser.parse_args()
    edgedetector_obj = EdgeDetector(args.input_image,args.superimposed_image_output,args.edge_image,args.threshold1,args.threshold2)
    edgedetector_obj.edge_detection()
