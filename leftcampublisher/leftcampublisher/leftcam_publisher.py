import json
import time
from datetime import datetime
import cv2
import sys
import os
import numpy as np
sys.path.append("../../tis_repos/Linux-tiscamera-Programming-Samples/python/python-common")
sys.path.append("../../theimagingsource_ros/config")
import TIS
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
        super().__init__('Left_Camera_Publisher')
        
        with open("leftcamera_publish_config.json") as jsonFile:
            cameraconfig = json.load(jsonFile)
            # print(cameraconfig)
            jsonFile.close()

        self.bridge = CvBridge()
        self.camera = CAMERA(cameraconfig['properties'], cameraconfig['imageprefix'])
        pformat = cameraconfig["pixelformat"]
        self.camera.open_device(cameraconfig['serial'],
                    cameraconfig['width'],
                    cameraconfig['height'],
                    cameraconfig['framerate'],
                    TIS.SinkFormats[pformat], False)
    
        self.camera.set_image_callback(self.ros_callback)
    
        self.camera.enableTriggerMode("Off")
        self.camera.busy = True
        self.camera.start_pipeline()
        self.camera.applyProperties()
        self.camera.enableTriggerMode("On")
        
        self.publisher_ = self.create_publisher(Image, 'Left_Image', 10)
        self.i = 0
        self.im_list = []
        
    
    def ros_callback(self, camera):
        cv_image = camera.get_image()
        img_msg = self.bridge.cv2_to_imgmsg(np.array(cv_image[:,:,:3]), "bgr8")
        img_msg.header.frame_id = "left_img"
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(img_msg)
        self.get_logger().info('Received left image')

def main(args=None):
    rclpy.init(args=args)
    ros_Publisher = Publisher()
    rclpy.spin(ros_Publisher)
    ros_Publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
