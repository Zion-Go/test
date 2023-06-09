import json
import time
from datetime import datetime
import cv2
import sys
import os
import numpy as np
sys.path.append("../../tis_repos/Linux-tiscamera-Programming-Samples/python/python-common")
import TIS
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

with open("hardware_cameras_config.json") as jsonFile:
    cameraconfig = json.load(jsonFile)
    jsonFile.close()

def on_new_image(camera, userdata):
    if userdata.busy:
        return
    if camera.busy:
        return
    
    if camera.imageprefix == "left":
        userdata.imageleft = camera.get_image()
    else:
        userdata.imageright = camera.get_image()

def get_msg_header(self, frame_id=None, timestamp=None):
        """
        Get a filled ROS message header
        :return: ROS message header
        :rtype: std_msgs.msg.Header
        """
        header = Header()
        if frame_id:
            header.frame_id = frame_id
        else:
            header.frame_id = self.get_prefix()
        if timestamp:
            t = self.get_clock().now()
            header.stamp = t.to_msg()
        else:
            header.stamp = self.communication.get_current_ros_time()
        return header 

class CAMERA(TIS.TIS):
    
    def __init__(self, properties, imageprefix):
        '''
        Constructor of the CAMERA class
        :param properties: JSON object, that contains the list of to set properites
        :param triggerproperty: JSON object, which contains the trigger property name and the enable and disable values
        :param imageprefix: Used to create the file names of the images to be saved.
        '''
        super().__init__()
        self.properties = properties
        self.imageprefix = imageprefix
        self.busy = False
        self.imageCounter = 0

class CustomData:
    ''' Example class for user data passed to the on new image callback function
    '''
    def __init__(self):
        self.imageleft = None
        self.imageright = None
        self.dummy = None
        self.busy = False
        self.imageCounter = 0

    def saveImages(self):
        '''
        Save the last received image
        '''
        # Avoid being called, while the callback is busy
        if self.busy is True:
            return

        self.busy = True
        self.imageCounter += 1

        leftimagefilename = "{0}_{1:04d}.jpg".format("left", self.imageCounter)
        rightimagefilename = "{0}_{1:04d}.jpg".format("right", self.imageCounter)
        print(self.imageCounter)
        path_left = '/home/docker_tis_driver/recording_repos/datasets/%s/left' %((date_time))
        path_right = '/home/docker_tis_driver/recording_repos/datasets/%s/right' %((date_time))
        cv2.imwrite(os.path.join(path_left, leftimagefilename), self.imageleft)
        cv2.imwrite(os.path.join(path_right, rightimagefilename), self.imageright)
        self.busy = False

CD = CustomData()
camera = CAMERA(cameraconfig['properties'], cameraconfig['imageprefix'])
camera.set_image_callback(on_new_image, CD)

class Publisher(Node):
    def __init__(self):
        super().__init__('Camera_Publisher')
        self.publisher_ = self.create_publisher(Image, 'Image', 10)
        time_period = 0.1
        self.timer = self.crete_timer(time_period, self.ros_callback)
        self.i = 0
        self.im_list = []
        self.bridge = CvBridge()
        self.cv_image = on_new_image(camera, CD)
    
    def ros_callback(self):
        self.publisher_.publish(self.bridge.cv2_to_imgmsg(np.array(self.cv_image), "bgrx"))
        img_msg = Image(on_new_image(camera, CD))
        header_msg = get_msg_header()
        self.publisher_.publish(img_msg)
        self.publisher_.publish(header_msg)
        self.get_logger().info('Received image')

def main(args=None):
    rclpy.init(args=args)
    ros_Publisher = Publisher()
    rclpy.spin(ros_Publisher)
    ros_Publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
