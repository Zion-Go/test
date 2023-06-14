import json
import yaml
import time
from datetime import datetime
import cv2
import sys
import os
import numpy as np
sys.path.append("../../tis_repos/Linux-tiscamera-Programming-Samples/python/python-common")
sys.path.append("../../theimagingsource_ros/config")
sys.path.append("/../../theimagingsource_ros/src/stereocam_publisher/stereocam_publisher")
from stereocam_publisher import TIS
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from ament_index_python.packages import get_package_share_directory

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
        

    def set_property(self, property_name, value):
        '''
        Pass a new value to a camera property. If something fails an
        exception is thrown.
        :param property_name: Name of the property to set
        :param value: Property value. Can be of type int, float, string and boolean
        '''
        try:
            baseproperty = self.source.get_tcam_property(property_name)
            baseproperty.set_value(value)
        except Exception as error:
            raise RuntimeError(f"Failed to set property '{property_name}'") from error
        
    def enableTriggerMode(self, onoff):
        '''
        Enable or disable the trigger mode
        :param bool onoff: "On" or "Off"
        '''       
        try:
            self.set_property("TriggerMode", onoff)
        except Exception as error:
            print(error)

    def applyProperties(self):
        '''
        Apply the properties in self.properties to the used camera
        The properties are applied in the sequence they are saved
        int the json file.
        Therefore, in order to set properties, that have automatiation
        the automation must be disabeld first, then the value can be set.
        '''
        for prop in self.properties:
            try:
                self.set_property(prop['property'],prop['value'])
            except Exception as error:
                print(error)       

class Publisher(Node):
    def __init__(self):
        super().__init__("stereocamera_publisher")
        
        
        self.bridge = CvBridge()
        self.left_img = self.create_publisher(Image, 'left_image', 10)
        self.right_img = self.create_publisher(Image, 'right_image', 10)
        # self.left_camerainfo = self.create_publisher(CameraInfo, 'left_camerainfo', 10)
        # self.right_camerainfo = self.create_publisher(CameraInfo, 'right_camerainfo', 10)
        self.i = 0

        # get imgs from cams
        self.left_image, self.right_image = [], []
        # get the imgs source with callback
        with open("/home/theimagingsource_ros/src/stereocam_publisher/config/nodes_config.yaml") as yamlFile:
            cameraconfig = yaml.safe_load(yamlFile)
    
            self.cameraleft = cameraconfig['leftcamera_publisher']['ros__parameters']['imageprefix']
            self.cameraright = cameraconfig['rightcamera_publisher']['ros__parameters']['imageprefix']
            
    
        # left 
        self.lcamera = CAMERA(cameraconfig['%scamera_publisher' % self.cameraleft]['ros__parameters']['properties'], cameraconfig['%scamera_publisher' % self.cameraleft]['ros__parameters']['imageprefix'])
        pformat = cameraconfig['%scamera_publisher' % self.cameraleft]['ros__parameters']["pixelformat"]
        print(self.cameraleft)
        self.lcamera.open_device(cameraconfig['%scamera_publisher' % self.cameraleft]['ros__parameters']['serial'],
                                    cameraconfig['%scamera_publisher' % self.cameraleft]['ros__parameters']['width'],
                                    cameraconfig['%scamera_publisher' % self.cameraleft]['ros__parameters']['height'],
                                    cameraconfig['%scamera_publisher' % self.cameraleft]['ros__parameters']['framerate'],
                                    TIS.SinkFormats[pformat], False)
        print('Left Camera opened')
        self.lcamera.set_image_callback(self.ros_callback)
        
        self.lcamera.enableTriggerMode("Off")
        self.lcamera.busy = True
        self.lcamera.start_pipeline()
        print('Left Pipeline started')
        self.lcamera.applyProperties()
        print('Left Properties applied')
        self.lcamera.enableTriggerMode("On")
        print('Left Trigger: on')
        # self.leftimage = self.lcamera.get_image()
        # print(self.leftimage)
        
        # right    
        self.rcamera = CAMERA(cameraconfig['%scamera_publisher' % self.cameraright]['ros__parameters']['properties'], cameraconfig['%scamera_publisher' % self.cameraright]['ros__parameters']['imageprefix'])
        pformat = cameraconfig['%scamera_publisher' % self.cameraright]['ros__parameters']["pixelformat"]
        print(self.cameraright)
        self.rcamera.open_device(cameraconfig['%scamera_publisher' % self.cameraright]['ros__parameters']['serial'],
                                    cameraconfig['%scamera_publisher' % self.cameraright]['ros__parameters']['width'],
                                    cameraconfig['%scamera_publisher' % self.cameraright]['ros__parameters']['height'],
                                    cameraconfig['%scamera_publisher' % self.cameraright]['ros__parameters']['framerate'],
                                    TIS.SinkFormats[pformat], False)
        print('Right Camera opened')
        self.rcamera.set_image_callback(self.ros_callback)
        
        self.rcamera.enableTriggerMode("Off")
        self.rcamera.busy = True
        self.rcamera.start_pipeline()
        print('Right Pipeline started')
        self.rcamera.applyProperties()
        print('Right Properties applied')
        self.rcamera.enableTriggerMode("On")
        print('Right Trigger: on')
        # self.rightimage = self.rcamera.get_image()
        # print(self.rightimage)  
        
        # left_image.append(self.leftimage)
        # right_image.append(self.rightimage)
        self.camera_list = [self.lcamera, self.rcamera]
        # fill the msgs
        # limg_msg = Image()
        # limg_msg.data = self.bridge.cv2_to_imgmsg(np.array(self.leftimage[self.i][:,:,:3]), "bgr8")
        # limg_msg.header.frame_id = "%s_img" % self.cameraleft
        # limg_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        # limg_msg.height = np.shape(self.leftimage)[0]
        # limg_msg.width = np.shape(self.leftimage)[1]
        # limg_msg.encoding = "bgr8"
        # # self.camerainfo = CameraInfo()

        # rimg_msg = Image()
        # rimg_msg.data = self.bridge.cv2_to_imgmsg(np.array(self.rightimage[self.i][:,:,:3]), "bgr8")
        # rimg_msg.header.frame_id = "%s_img" % self.cameraright
        # rimg_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        # rimg_msg.height = np.shape(self.rightimage)[0]
        # rimg_msg.width = np.shape(self.rightimage)[1]
        # rimg_msg.encoding = "bgr8"

        # self.left_img.publish(limg_msg)
        # self.right_img.publish(rimg_msg)
        # self.left_camerainfo.publish(self.camerainfo)
        # self.right_camerainfo.publish(self.camerainfo)
    
    def ros_callback(self):
        self.leftimage = self.lcamera.get_image()
        self.rightimage = self.rcamera.get_image()
        self.left_image.append(self.leftimage)
        self.right_image.append(self.rightimage)

        limg_msg = Image()
        limg_msg.data = self.bridge.cv2_to_imgmsg(np.array(self.leftimage[self.i][:,:,:3]), "bgr8")
        limg_msg.header.frame_id = "%s_img" % self.cameraleft
        limg_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        limg_msg.height = np.shape(self.leftimage)[0]
        limg_msg.width = np.shape(self.leftimage)[1]
        limg_msg.encoding = "bgr8"
        # self.camerainfo = CameraInfo()

        rimg_msg = Image()
        rimg_msg.data = self.bridge.cv2_to_imgmsg(np.array(self.rightimage[self.i][:,:,:3]), "bgr8")
        rimg_msg.header.frame_id = "%s_img" % self.cameraright
        rimg_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        rimg_msg.height = np.shape(self.rightimage)[0]
        rimg_msg.width = np.shape(self.rightimage)[1]
        rimg_msg.encoding = "bgr8"

        self.left_img.publish(limg_msg)
        self.right_img.publish(rimg_msg)
        self.get_logger().info('Received_%s_image: %d' % (self.cameraleft, self.i))
        self.get_logger().info('Received_%s_image: %d' % (self.cameraright, self.i))
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    stereocam_Publisher = Publisher()
    print('1')
    rclpy.spin(stereocam_Publisher)
    
    stereocam_Publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

