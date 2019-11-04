#!/usr/bin/env python

import cv2
import numpy as np
import os
import rospy
import yaml

from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel



class Augmented:

    def __init__(self, node_name):

        print('initialized')
        # Initialize the DTROS parent class
        self.veh_name = os.environ['VEHICLE_NAME']
        self.map_name = os.environ['MAP_NAME']
        # Use the kinematics calibration for the gain and trim

        # self.parameters['~homography'] = None
        # self.parameters['~camera_matrix'] = None
        # self.parameters['~distortion_coefficients'] = None
        # self.parameters['~projection_matrix'] = None
        # self.parameters['~rectification_matrix'] = None

        # Set parameters using a robot-specific yaml file if such exists
        self.readParamFromFile()
        print('initialized augmented')

        self.pcm_ = PinholeCameraModel()
        # Wait for the automatic gain control
        # of the camera to settle, before we stop it


        #rospy.set_param("".join(['/',self.veh_name,'/camera_node/exposure_mode']), 'off')
                # change resolution camera

    def ground2pixel(self, image, pointX, pointY):
        imageHeight, imageWidth, channels = image.shape

        homographyMatrix = self.parameters['~homography']

        pointMatrix = np.transpose(np.array([pointX, pointY, 1]))
        Hinv = np.linalg.inv(homographyMatrix)


        transformedMatrix = np.linalg.solve(homographyMatrix, pointMatrix)
        print(transformedMatrix)
        uL = transformedMatrix[0]
        vL = transformedMatrix[1]
        wL = transformedMatrix[2]
        u = int(round(uL/wL,0))
        v = int(round(vL/wL,0))
        print(uL, vL, wL, u, v)

        return u, v


    def drawLines(self, imageToDraw):

        imageHeight, imageWidth, channels = imageToDraw.shape
        mapName = self.map_name
        filePath = '/code/catkin_ws/src/augmented-reality/packages/augmentedreality/map/' + mapName
        with open(filePath, 'r') as stream:
            try:
                text = yaml.safe_load(stream)
                referenceFrame = text['points']['TL'][0]

                TL = text['points']['TL'][1]
                TR = text['points']['TR'][1]
                BR = text['points']['BR'][1]
                BL = text['points']['BL'][1]

                for entry in text['segments']:

                    point1X = text['points'][entry['points'][0]][1][1]
                    point1Y = text['points'][entry['points'][0]][1][0]
                    point2X = text['points'][entry['points'][1]][1][1]
                    point2Y = text['points'][entry['points'][1]][1][0]


                    point1RatioX = point1X / 1
                    point1RatioY = point1Y / 1
                    point2RatioX = point2X / 1
                    point2RatioY = point2Y / 1

                    point1CameraFrameX = int(round(point1RatioX * imageWidth,0))
                    point1CameraFrameY = int(round(point1RatioY * imageHeight,0))
                    point2CameraFrameX = int(round(point2RatioX * imageWidth,0))
                    point2CameraFrameY = int(round(point2RatioY * imageHeight,0))


                    pointX = [point1CameraFrameX, point2CameraFrameX]
                    pointY = [point1CameraFrameY, point2CameraFrameY]

                    color = entry['color']


                    if self.map_name == 'calibration_pattern.yaml':
                        point1X, point1Y = self.ground2pixel(imageToDraw, point1X, point1Y)
                        point2X, point2Y = self.ground2pixel(imageToDraw, point2X, point2Y)
                        pointX = [point1X, point2X]
                        pointY = [point1Y, point2Y]
                        print(pointX, pointY)
                    imageToDraw = self.draw_segment(imageToDraw, pointX, pointY, color)

                image_message = CvBridge().cv2_to_compressed_imgmsg(imageToDraw)
                return image_message

            except yaml.YAMLError as exc:
                print(exc)



    def process_image(self, data):
        cv_image = CvBridge().compressed_imgmsg_to_cv2(data, desired_encoding="passthrough")
        finalImage = cv_image.copy()

        self.readParamFromFile()

        self.parameters['~homography'] = np.reshape(np.array(rospy.get_param('~homography')), (3,3))
        self.parameters['~camera_matrix'] = np.array(rospy.get_param('~camera_matrix'))
        self.parameters['~distortion_coefficients'] = np.array(rospy.get_param('~distortion_coefficients'))
        self.parameters['~projection_matrix'] = np.array(rospy.get_param('~projection_matrix'))
        self.parameters['~rectification_matrix'] = np.array(rospy.get_param('~rectification_matrix'))

        self.parameters['~camera_matrix'] = np.reshape(self.parameters['~camera_matrix'], (3, 3))
        self.parameters['~distortion_coefficients'] = np.reshape(self.parameters['~distortion_coefficients'], (1, 5))
        self.parameters['~projection_matrix'] = np.reshape(self.parameters['~projection_matrix'], (3, 4))
        self.parameters['~rectification_matrix'] = np.reshape(self.parameters['~rectification_matrix'], (3, 3))


        imageHeight, imageWidth, channels = finalImage.shape

        # Used for rectification
        self.mapx = np.ndarray(shape=(imageHeight, imageWidth, 1), dtype='float32')
        self.mapy = np.ndarray(shape=(imageHeight, imageWidth, 1), dtype='float32')


        self.mapx, self.mapy = cv2.initUndistortRectifyMap(self.parameters['~camera_matrix'],
            self.parameters['~distortion_coefficients'], self.parameters['~rectification_matrix'],
            self.parameters['~projection_matrix'], (imageWidth, imageHeight), cv2.CV_32FC1)

        undistortedImage = cv2.remap(finalImage, self.mapx, self.mapy, cv2.INTER_CUBIC)
        returnImage = self.drawLines(undistortedImage)

        return returnImage


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



    def readParamFromFile(self):
        """
        Reads the saved parameters from
        `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml` or
        uses the default values if the file doesn't exist. Adjsuts
        the ROS paramaters for the node with the new values.

        """
        print('in paramreader intrinsic')
        # Check file existence
        fname = self.getFilePath(self.veh_name, 'extrinsic')
        # Use the default values from the config folder if a
        # robot-specific file does not exist.
        if not os.path.isfile(fname):
            self.log("Kinematics calibration file %s does not "
                     "exist! Using the default file." % fname, type='warn')
            fname = self.getFilePath('default')

        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return

        # Set parameters using value in yaml file
        if yaml_dict is None:
            # Empty yaml file
            return
        for param_name in ["homography"]:
            param_value = yaml_dict.get(param_name)
            if param_name is not None:
                rospy.set_param("~"+param_name, param_value)
            else:
                # Skip if not defined, use default value instead.
                pass

        fnameExt = self.getFilePath(self.veh_name, 'intrinsic')

        # Use the default values from the config folder if a
        # robot-specific file does not exist.
        if not os.path.isfile(fnameExt):
            self.log("Kinematics calibration file %s does not "
                     "exist! Using the default file." % fnameExt, type='warn')
            fnameExt = self.getFilePath('default')

        with open(fnameExt, 'r') as in_file:
            try:
                yaml_dict2 = yaml.load(in_file)
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         %(fnameExt, exc), type='fatal')
                rospy.signal_shutdown()
                return

        # Set parameters using value in yaml file
        if yaml_dict2 is None:
            # Empty yaml file
            return
        for param_name2 in ["camera_matrix", "distortion_coefficients", "projection_matrix", "rectification_matrix"]:
            param_value2 = yaml_dict2.get(param_name2)['data']

            # "camera_matrix", "distortion_coefficients", "projection_matrix", "rectification_matrix"
            if param_name2 is not None:
                rospy.set_param("~"+param_name2, param_value2)
            else:
                # Skip if not defined, use default value instead.
                pass

    def getFilePath(self, name, cameraCalibration):
        """
        Returns the path to the robot-specific configuration file,
        i.e. `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml`.

        Args:
            name (:obj:`str`): the Duckiebot name

        Returns:
            :obj:`str`: the full path to the robot-specific
                calibration file

        """
        if cameraCalibration == 'extrinsic':
            cali_file_folder = '/data/config/calibrations/camera_extrinsic/'

        if cameraCalibration == 'intrinsic':
            cali_file_folder = '/data/config/calibrations/camera_intrinsic/'

        cali_file = cali_file_folder + name + ".yaml"
        return cali_file
