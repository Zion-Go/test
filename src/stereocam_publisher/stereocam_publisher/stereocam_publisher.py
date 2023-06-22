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

def parameters_parsing(self):
        
    cam_serial = self.declare_parameter("serial", '')
    self.serial = cam_serial.get_parameter_value().string_value

    cam_prefix = self.declare_parameter("imageprefix", '')
    self.prefix = cam_prefix.get_parameter_value().string_value

    config_path = self.declare_parameter("config_path", '')
    self.config_path = config_path.get_parameter_value().string_value

    with open(self.config_path) as jsonFile:
        self.prop = json.load(jsonFile)

    self.format = self.prop["format"]
    self.pformat = self.prop["pixelformat"]
    self.width = self.prop["width"]
    self.height = self.prop["height"]
    self.framerate = self.prop["framerate"]
    
    
    print("node config:", self.prefix, self.pformat, self.serial, self.width, self.height, self.framerate)

class Publisher(Node):
    def __init__(self):
        super().__init__("stereocamera_publisher")

        parameters_parsing(self)
            
        self.get_logger().info('\n%s camera: \n-serial num: %s, \n-format: %s, \n-pixel format: %s, \n-pixel width: %d, \n-pixel height: %d, \n-default frame rate: %s ' % (self.prefix, self.serial, self.format, self.pformat, self.width, self.height, self.framerate))

        self.bridge = CvBridge()
        self.i = 0

        self.img_publish = self.create_publisher(Image, '%s_image' % self.prefix, 20)

        self.camera = CAMERA(self.prop["properties"], self.prefix)
        self.camera.open_device(self.serial,
                                self.width,
                                self.height,
                                self.framerate,
                                TIS.SinkFormats[self.pformat], False)

        self.camera.set_image_callback(self.ros_callback)
        
        self.camera.enableTriggerMode("Off")
        self.camera.applyProperties()
        self.camera.start_pipeline()
        self.camera.enableTriggerMode("On")
    
    def ros_callback(self,camera):
        self.image = camera.get_image()

        img_msg = self.bridge.cv2_to_imgmsg(np.array(self.image[:,:,:3]), encoding="bgr8")
        img_msg.header.frame_id = "%s_camera" %(self.prefix)
        img_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        
        self.img_publish.publish(img_msg)

        self.get_logger().info('Received_%s_image: %d' % (self.prefix, self.i))
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    stereocam_Publisher = Publisher()
    rclpy.spin(stereocam_Publisher)

    stereocam_Publisher.camera.stop_pipeline()
    stereocam_Publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

