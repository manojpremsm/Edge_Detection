# Edge_Detection

Libraries installed:

opencv library is used  and it can installed as below.

open a terminal and type the following command.

    pip install opencv-python



1. Basic

 

The steps in the algorithm are as follows:

 

·       The given input image is transformed into gray scale and passed through a noise reduction gaussian filter.

·       Then the smoothened image is used to detect edges

·       The canny edge detector filter works by identifying areas in the image where there is change in intensity significantly by computing the gradients of the images using derivative filters.

·       The detector has a lower threshold value and an upper threshold value , any edge pixel with an intensity gradient below the lower threshold is discarded and anything above the upper threshold value is considered an edge and in-   between this range is considered to be a weak edge.

Steps to run the program.

The file edge_detection.py is placed in ‘edge_detection_ros/Basic’ . The file consists of five arguments input image path, superimposed output image path, edgedetection output image path, threshold 1 , threshold 2 .

The following is the example template to run the file.

     Python3 path/to/file/edge_detection.py ‘path/to/input/image ‘ ‘path/to/superimposed/outputimage ‘ ‘path/to/edge/output/image’ 200 250.


2. Vision Ros

 

This part of the challenge consists of creating a service for edge detection. To run the service we have to navigate to the src folder of Edge_Service catkin package and run the service and client file in the order below. We can write a launch file to launch multiple nodes by running a single file but in my case the computation power of the pc was not that great so I choose to do in separate terminal. All the commands mentioned below should be run in a different terminal

 

To initialize the master

    roscore

 

To start the server

     rosrun /catkin_package name/ server python file name

Ex : rosrun Edge_Service  edge_detection_server.py

To run the client

    rosrun /catkin_package name/ client python file name

    rosrun Edge_Service  edge_detection_client.py

 

The second part of this challenge consist of converting the detected edges from 2d points to 3d point cloud.

The overview of the code is as follows.

·       We create subscribers to read images from the give .bag files from the topic ‘camera/color/image_raw’.

·       The images are in ros compressed image format which we will convert into opencv format using a library called cv_bridge.

·       Then we perform edge detection using canny filters and that is published as topic by creating a publisher and in the code it is published as “output_image”.

·       We create another subscriber to read depth images from the give .bag files from the topic ‘/camera/depth/image_rect_raw’.

·       We create another subscriber to read the camera information to obtain the intrincis needed to convert from image to 3d point cloud.

·       The depth values are extracted for every edge pixel corresponding to the image and we calculate the respection 3d point(x,y,z) of the 2d pixel(u.v).

·       The calculated point cloud is published into a topic called “edge_points” as message type sensor_msgs /point cloud. This contains header with details of the time that it was published and the frame id.

·       The same point cloud can be visualized in rviz by adding the corresponding topic.

 

To run this program, follow these steps. All the commands mentioned below should in run in a different terminals

 

To initialize the master

    roscore

 

This is to run the ros bag so that we can use the recorded data from the bag.

    rosbag play –clock -l path/to/rosbag

 

This run the ros node ‘image_test’

    rosrun /catkin_package name/ server python file name

    rosrun Edge_Service  edge2pc.py

 

To start the rviz visualizer.

    rviz

 

The running topics and nodes can be checked using the following commands.

    rostopic list
    rosnode list

 

3. Robot Ros

In this part of the challenge, we create markers. Markers are very important tool for vizualizing , monitoring and debugging the robotic system. We can use them to display position and orientation of the robot or the locations of the objects it detects. In our task we use the points from the published topic and fed it into the markers. We first initialize the markers and provide the pose orientation for them.

We can subscribe to any topic that is publishing the point cloud and feed them into our marker. We have used the point clouds from the provided  .bag file and created the markers. We can also change the topic from this to the one that we created earlier in task 2 and feed them into the markers. The only change in this case will be to change the topic name in the marker.py file. The markers are then published into the topic “visualization_marker” of type markers.

 

To run this program, follow these steps. All the commands mentioned below should be run in a different terminal.

 

To initialize the master
          
    roscore
 

This is to run the ros bag so that we can use the recorded data from the bag.

    rosbag play –clock -l path/to/rosbag

 

This run the ros node ‘PointCloud_Marker_Publisher’

    rosrun /catkin_package name/ python file.py

    rosrun Edge_Service  marker.py

 

To start the rviz visualizer.

    rviz
