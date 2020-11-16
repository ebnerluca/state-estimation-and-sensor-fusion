#!/usr/bin/env python3
import numpy as np
# import os
import math
import cv2


import rospy
import yaml
# import rospkg
# import sys
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from at_localization import AtLocalization


class AtLocalizationNode(DTROS):
    def __init__(self, node_name):

        super(AtLocalizationNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.veh = rospy.get_namespace().strip("/")  # robot name
        self.bridge = CvBridge()  # converts images (ros <--> cv2)

        # initialize camera subscriber and load calibration data
        self.sub_camera_img = rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.cb_camera,
                                               queue_size=1)
        calibration_data = self.read_yaml_file(f"/data/config/calibrations/camera_intrinsic/{self.veh}.yaml")
        camera_info = self.camera_info_from_yaml(calibration_data)

        # Initialize image rectification and apriltag detection
        self.at_loc = AtLocalization(camera_info, tag_size=0.065)

        # debugging
        self.debug = True
        self.pub_at_detection = rospy.Publisher("~image/compressed", CompressedImage, queue_size=1)

    def broadcast_frames(self):
        pass

    def cb_camera(self, msg):
        img = self.read_image(msg)

        # detect april tag and extract its reference frame
        img_rect = self.at_loc.rectify(img)
        tags = self.at_loc.detect(img_rect)

        if self.debug:
            self.visualize_at_detection(img_rect, tags)
            img_out = self.bridge.cv2_to_compressed_imgmsg(img_rect)
            img_out.header = msg.header
            img_out.format = msg.format
            self.pub_at_detection.publish(img_out)

    def read_image(self, msg_image):
        """
            Convert images to OpenCV images
            Args:
                msg_image (:obj:`CompressedImage`) the image from the camera node
            Returns:
                OpenCV image
        """
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg_image)
            return cv_image
        except CvBridgeError as e:
            self.log(e)
            return []

    def read_yaml_file(self, fname):
        """
            Reads the 'fname' yaml file and returns a dictionary with its input.

            You will find the calibration files you need in:
            `/data/config/calibrations/`
        """
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
                return yaml_dict
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         % (fname, exc), type='fatal')
                rospy.signal_shutdown()
                return

    @staticmethod
    def camera_info_from_yaml(calib_data):
        """
        Express calibration data (intrinsics) as a CameraInfo instance.

        :param calib_data: dict, loaded from yaml file
        :return: intrinsics as CameraInfo instance
        """
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info

    @staticmethod
    def visualize_at_detection(img, tags):
        """
        Visualize detected april tags for debugging.
        """
        for tag in tags:
            for idx in range(len(tag.corners)):
                cv2.line(img, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)),
                         (0, 255, 0))

            cv2.putText(img, str(tag.tag_id),
                        org=(tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=0.8,
                        color=(0, 0, 255))


if __name__ == "__main__":
    at_localization = AtLocalizationNode("at_localization_node")
    rospy.spin()
