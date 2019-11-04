#!/usr/bin/env python

import cv2
import numpy as np
import os
import rospy
import yaml

from duckietown import DTROS
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
from cv_bridge import CvBridge
from augmented_reality import Augmented


class augmentedNode(DTROS, Augmented):
    """AugmentedNode Behaviour

    This node implements Braitenberg vehicle behavior on a Duckiebot.

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node
            that ROS will use

    Configuration:
        ~gain (:obj:`float`): scaling factor applied to the desired
            velocity, taken from the robot-specific kinematics
            calibration
        ~trim (:obj:`float`): trimming factor that is typically used
            to offset differences in the behaviour of the left and
            right motors, it is recommended to use a value that results
            in the robot moving in a straight line when forward command
            is given, taken from the robot-specific kinematics calibration
        ~baseline (:obj:`float`): the distance between the two wheels
            of the robot, taken from the robot-specific kinematics
            calibration
        ~radius (:obj:`float`): radius of the wheel, taken from the
            robot-specific kinematics calibration
        ~k (:obj:`float`): motor constant, assumed equal for both
            motors, taken from the robot-specific kinematics calibration
        ~limit (:obj:`float`): limits the final commands sent to the
            motors, taken from the robot-specific kinematics calibration

    Subscriber:
        ~image/compressed (:obj:`CompressedImage`): The acquired camera
            images

    Publisher:
        ~wheels_cmd (:obj:`duckietown_msgs.msg.WheelsCmdStamped`): The
            wheel commands that the motors will execute

    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(augmentedNode, self).__init__(node_name=node_name)
        self.veh_name = os.environ['VEHICLE_NAME']
        self.map_name = os.environ['MAP_NAME']

        print('initialized augmented')
        # Use the kinematics calibration for the gain and trim
        # self.parameters['~gain'] = None
        # self.parameters['~trim'] = None
        # self.parameters['~baseline'] = None
        # self.parameters['~radius'] = None
        # self.parameters['~k'] = None
        # self.parameters['~limit'] = None

        # Set parameters using a robot-specific yaml file if such exists
        # self.readParamFromFile()
        # rospy.set_param("~gain", 1)
        # self.updateParameters()
        # rospy.set_param("~gain", 1)

        # Wait for the automatic gain control
        # of the camera to settle, before we stop it


        #rospy.set_param("".join(['/',self.veh_name,'/camera_node/exposure_mode']), 'off')
                # change resolution camera
        Augmented(node_name = node_name)
        rospy.set_param('/' + 'symphony' + '/camera_node/res_w', 640) # 640
        rospy.set_param('/' + 'symphony' + '/camera_node/res_h', 480) # 480
        topic = '/' + 'symphony' + '/camera_node/image/compressed'

        self.subscriber(topic, CompressedImage, self.callback, queue_size=1)

        imageName = self.map_name.split('.')[0]
        topicPub = '/' + self.veh_name + '/augmented_reality_node/' + str(imageName) + '/image/compressed'

        self.pub_wheels_cmd = self.publisher(topicPub, CompressedImage, queue_size=1)

    def callback(self, data):
        returnedImage = Augmented.process_image(self, data)
        self.pub_wheels_cmd.publish(returnedImage)



if __name__ == '__main__':
    # Initialize the node
    camera_node = augmentedNode(node_name='augmented_reality_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
