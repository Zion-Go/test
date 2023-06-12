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
from sensor_msgs.msg import Image


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
        super().__init__("leftcamera_publisher")
        
        with open("/home/a/theimagingsource_ros/src/stereocam_publisher/config/nodes_config.yaml") as yamlFile:
            cameraconfig = yaml.safe_load(yamlFile)
        # print(cameraconfig)

        self.cameraleft = cameraconfig['leftcamera_publisher']['ros__parameters']['imageprefix']
        self.cameraright = cameraconfig['rightcamera_publisher']['ros__parameters']['imageprefix']
        camera_list = [self.cameraleft, self.cameraright]
        self.bridge = CvBridge()

        for camerawhich in camera_list:
            self.camerawhich = camerawhich
            self.camera = CAMERA(cameraconfig['%scamera_publisher' % self.camerawhich]['ros__parameters']['properties'], cameraconfig['%scamera_publisher' % self.camerawhich]['ros__parameters']['imageprefix'])
            pformat = cameraconfig['%scamera_publisher' % self.camerawhich]['ros__parameters']["pixelformat"]
        
            self.camera.open_device(cameraconfig['%scamera_publisher' % self.camerawhich]['ros__parameters']['serial'],
                                    cameraconfig['%scamera_publisher' % self.camerawhich]['ros__parameters']['width'],
                                    cameraconfig['%scamera_publisher' % self.camerawhich]['ros__parameters']['height'],
                                    cameraconfig['%scamera_publisher' % self.camerawhich]['ros__parameters']['framerate'],
                                    TIS.SinkFormats[pformat], False)
    
            self.camera.set_image_callback(self.ros_callback)
    
            self.camera.enableTriggerMode("Off")
            self.camera.busy = True
            self.camera.start_pipeline()
            self.camera.applyProperties()
            self.camera.enableTriggerMode("On")
        
            self.publisher_ = self.create_publisher(Image, '%s_Image' % self.camerawhich, 10)
            self.i = 0
            # self.im_list = []
        
    
    def ros_callback(self, camera):
        cv_image = camera.get_image()
        img_msg = self.bridge.cv2_to_imgmsg(np.array(cv_image[:,:,:3]), "bgr8")
        img_msg.header.frame_id = "%s_img" % self.camerawhich
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(img_msg)
        self.get_logger().info('Received_%s_image: %d' % (self.camerawhich, self.i))
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    cam_Publisher = Publisher()
    rclpy.spin(cam_Publisher)
    cam_Publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
