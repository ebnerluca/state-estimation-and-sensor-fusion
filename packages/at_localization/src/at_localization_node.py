#!/usr/bin/env python3
import numpy as np
# import os
import cv2
import rospy
import yaml
# import rospkg
# import sys

from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
import tf2_ros
from tf import transformations
from cv_bridge import CvBridge, CvBridgeError

from duckietown.dtros import DTROS, NodeType
from at_localization import AtLocalization


class AtLocalizationNode(DTROS):
    """
    Detect map landmark (using april tag) and broadcast the robot's pose.
    """
    def __init__(self, node_name):

        super(AtLocalizationNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.veh = rospy.get_namespace().strip("/")  # robot name
        self.bridge = CvBridge()  # converts images (ros <--> cv2)

        # traffic sign parameters
        at_size = 0.065  # apriltag size (side length)
        at_height = 0.089  # height of apriltag center point above ground

        calibration_data = self.read_yaml_file(f"/data/config/calibrations/camera_intrinsic/{self.veh}.yaml")
        camera_info = self.camera_info_from_yaml(calibration_data)

        # initialize image rectification and apriltag detection
        self.at_loc = AtLocalization(camera_info, tag_size=at_size)

        # initialize tf broadcasters, broadcast the static ones
        self.timestamp = rospy.Time.now()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # construct map to apriltag static transform
        self.ts_map_apriltag = TransformStamped()
        self.ts_map_apriltag.header.frame_id = "map"
        self.ts_map_apriltag.child_frame_id = "apriltag"
        tf_map_apriltag = transformations.translation_matrix([0, 0, at_height])  # 4x4 numpy array
        self.ts_map_apriltag.transform = self.tf_to_msg(tf_map_apriltag)
        self.ts_map_apriltag.header.stamp = self.timestamp

        # construct apriltag to camera transform
        self.bc_apriltag_camera = tf2_ros.TransformBroadcaster()
        self.ts_apriltag_camera = TransformStamped()
        self.ts_apriltag_camera.header.frame_id = "apriltag"
        self.ts_apriltag_camera.child_frame_id = "camera"
        self.tf_apriltag_camera = None
        self.has_new_transform = False

        # construct camera to baselink static transform
        self.ts_camera_baselink = TransformStamped()
        self.ts_camera_baselink.header.frame_id = "camera"
        self.ts_camera_baselink.child_frame_id = "at_baselink"
        self.ts_camera_baselink.transform = self.get_tf_msg_camera_baselink()
        self.ts_camera_baselink.header.stamp = self.timestamp

        # publish static transforms
        self.static_broadcaster.sendTransform([self.ts_map_apriltag, self.ts_camera_baselink])

        # initialize camera subscriber and load calibration data
        self.sub_camera_img = rospy.Subscriber("camera_node/image/compressed",
        CompressedImage, self.cb_camera, queue_size=1)

        # debugging
        self.debug = False # no debugging
        self.pub_at_detection = rospy.Publisher("~image/compressed", CompressedImage, queue_size=1)

        self.log("Letsgoooo boizz")


    def run(self):
        """
        Broadcast the required transforms: map-apriltag, apiltag-camera, camera-at_baselink
        """
        r = rospy.Rate(30)  # 30Hz
        while not rospy.is_shutdown():
            #if( (self.tf_apriltag_camera is not None) && (self.has_new_transform) ): #only publish when there is a new transform
            if(self.has_new_transform): #only publish when there is a new transform
                self.ts_apriltag_camera.header.stamp = self.timestamp
                self.ts_apriltag_camera.transform = self.tf_to_msg(self.tf_apriltag_camera)
                self.bc_apriltag_camera.sendTransform(self.ts_apriltag_camera)  # broadcast apriltag-camera
                self.has_new_transform = False # reset flag
            r.sleep()


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
            
        if len(tags) > 1:
            rospy.logwarn_throttle(2.0, "[ATLocalizationNode]: More than one april tag detected in the image. Please make sure only one is visible.")
            return
        if len(tags) == 0:
            rospy.loginfo_throttle(2.0, "[ATLocalizationNode]: No apriltag detected.")
            return

        # update tf and timestamp
        self.set_tf_apriltag_camera(tags[0].pose_R, tags[0].pose_t)
        self.timestamp = msg.header.stamp
        self.has_new_transform = True # set flag if new transform was computed


    @staticmethod
    def tf_to_msg(transform):
        """
        Convert a transform expressed as a 4x4 numpy array to geometry_msgs/Transform.
        """
        translation = transform[0:3, 3]
        rotation = transformations.quaternion_from_matrix(transform)

        msg = Transform()
        msg.translation = Vector3(*translation)
        msg.rotation = Quaternion(*rotation)
        return msg


    def get_tf_msg_camera_baselink(self):
        """
        Returns the tf from camera to at_baselink as geometry_msgs/Transform.
        """
        # transform baselink-camera (easier to set by hand than the other way around)
        translation = [0.0582, 0, 0.1072]
        beta = np.pi * 15 / 180  # 15° rotation along y axis
        T_R = transformations.rotation_matrix(beta, (0, 1, 0))  # 4x4 numpy array
        T_t = transformations.translation_matrix(translation)  # 4x4 numpy array
        tf_baselink_camera = T_R @ T_t  # @ is matrix multiplication (with numpy arrays)
        tf_camera_baselink = np.linalg.inv(tf_baselink_camera)
        return self.tf_to_msg(tf_camera_baselink)


    def set_tf_apriltag_camera(self, R, t):
        # detected tf: camera-apriltag, camera frame has wrong orientation
        tf_detected = np.zeros((4, 4), dtype=np.float64)
        tf_detected[0:3, 0:3] = R
        tf_detected[0:3, 3] = t[:, 0]
        tf_detected[3, 3] = 1

        # correct camera frame and apriltag orientation to conform with specifications
        camera_rot = transformations.euler_matrix(np.pi/2, -np.pi/2, 0, axes="ryzx")  # rotate C to C'
        apriltag_rot = transformations.euler_matrix(np.pi/2, -np.pi/2, 0, axes="rxzy")  # rotate A' to A
        tf_camera_apriltag = camera_rot @ tf_detected @ apriltag_rot  # Tcc' * Tc'a' * Ta'a = Tca

        self.tf_apriltag_camera = np.linalg.inv(tf_camera_apriltag)  # Tac


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
                yaml_dict = yaml.load(in_file, Loader=yaml.Loader)
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
    at_localization.run()
