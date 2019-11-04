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



class augmentedNode(DTROS):
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
        super(BraitenbergNode, self).__init__(node_name=node_name)
        print(rospy.get_namespace())
        self.veh_name = rospy.get_namespace().strip("/")
        # Use the kinematics calibration for the gain and trim
        self.parameters['~gain'] = None
        self.parameters['~trim'] = None
        self.parameters['~baseline'] = None
        self.parameters['~radius'] = None
        self.parameters['~k'] = None
        self.parameters['~limit'] = None

        # Set parameters using a robot-specific yaml file if such exists
        self.readParamFromFile()
        rospy.set_param("~gain", 1)
        self.updateParameters()
        rospy.set_param("~gain", 1)

        # Wait for the automatic gain control
        # of the camera to settle, before we stop it


        rospy.set_param("".join(['/',self.veh_name,'/camera_node/exposure_mode']), 'off')
                # change resolution camera
        topic = '/' + self.veh_name + '/camera_node/image/compressed'


        self.subscriber(topic, CompressedImage, self.drawLines)
        imageName = 'hoi'

        topicPub = '/' + self.veh_name + '/augmentedNode/' + str(imageName) + 'image/compressed'

        rate = rospy.Rate(4) # 4Hz

        self.pub_wheels_cmd = self.publisher(topicPub, CompressedImage, queue_size=1)
        self.log("Initialized")



    def drawLines(self, data):
        cv_image = CvBridge().compressed_imgmsg_to_cv2(data, desired_encoding="passthrough")
        finalImage = cv_image.copy()
        imageHeight, imageWidth, channels = finalImage.shape


        with open("hud.yaml", 'r') as stream:
            try:
                text = yaml.safe_load(stream)
                referenceFrame = text['points']['TL'][0]

                TL = text['points']['TL'][1]
                TR = text['points']['TR'][1]
                BR = text['points']['BR'][1]
                BL = text['points']['BL'][1]

                for entry in text['segments']:

                    point1X = text['points'][entry['points'][0]][1][0]
                    point1Y = text['points'][entry['points'][0]][1][1]
                    point2X = text['points'][entry['points'][1]][1][0]
                    point2Y = text['points'][entry['points'][1]][1][1]

                    point1RatioX = point1X / 1
                    point1RatioY = point1Y / 1
                    point2RatioX = point2X / 1
                    point2RatioY = point2Y / 1

                    point1CameraFrameX = point1RatioX * point1X
                    point1CameraFrameY = point1RatioY * point1Y
                    point2CameraFrameX = point2RatioX * point2X
                    point2CameraFrameY = point2RatioY * point2Y

                    point1 = [point1CameraFrameX, point1CameraFrameY]
                    point2 = [point2CameraFrameX, point2CameraFrameY]

                    color = entry['color']

                    image = self.draw_segment(data, point1, point2, color)
                    image_message = CvBridge().cv2_to_compressed_imgmsg(output)
                    self.pub_wheels_cmd.publish(image_message)

            except yaml.YAMLError as exc:
                print(exc)

    def draw_segment(self, image, pt_x, pt_y, color):
    defined_colors = {
        'red': ['rgb', [1, 0, 0]],
        'green': ['rgb', [0, 1, 0]],
        'blue': ['rgb', [0, 0, 1]],
        'yellow': ['rgb', [1, 1, 0]],
        'magenta': ['rgb', [1, 0 , 1]],
        'cyan': ['rgb', [0, 1, 1]],
        'white': ['rgb', [1, 1, 1]],
        'black': ['rgb', [0, 0, 0]]}
    _color_type, [r, g, b] = defined_colors[color]
    cv2.line(image, (pt_x[0], pt_y[0]), (pt_x[1], pt_y[1]), (b * 255, g * 255, r * 255), 5)
    return image





if __name__ == '__main__':
    # Initialize the node
    camera_node = augmentedNode(node_name='augmented')
    # Keep it spinning to keep the node alive
    rospy.spin()
