#!/usr/bin/env python3
import numpy as np
import cv2

from dt_apriltags import Detector

from image_geometry import PinholeCameraModel


class AtLocalization:
    """
    Handles image rectification and april tag detection.
    """
    def __init__(self, camera_info, tag_size):

        # init image rectification
        self.pcm = PinholeCameraModel()
        self.pcm.fromCameraInfo(camera_info)
        self.K_rect, self.mapx, self.mapy = self._init_rectification()

        # init apriltag detector
        self.at_detector = Detector(searchpath=['apriltags'], families='tag36h11', nthreads=1, quad_decimate=1.0,
                                    quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25, debug=0)
        self.tag_size = tag_size
        self.camera_params = (self.K_rect[0, 0], self.K_rect[1, 1], self.K_rect[0, 2], self.K_rect[1, 2])

    def _init_rectification(self):
        """
        Establish rectification mapping.
        """
        w = self.pcm.width
        h = self.pcm.height
        K_rect, roi = cv2.getOptimalNewCameraMatrix(self.pcm.K, self.pcm.D, (w, h), 1.0)
        mapx = np.ndarray(shape=(h, w, 1), dtype='float32')
        mapy = np.ndarray(shape=(h, w, 1), dtype='float32')
        mapx, mapy = cv2.initUndistortRectifyMap(self.pcm.K, self.pcm.D, self.pcm.R,
                                                 self.pcm.P, (w, h),
                                                 cv2.CV_32FC1, mapx, mapy)
        return K_rect, mapx, mapy

    def rectify(self, img_raw, interpolation=cv2.INTER_NEAREST):
        """
        Rectify image.
        """
        return cv2.remap(img_raw, self.mapx, self.mapy, interpolation)

    def detect(self, img):
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        tags = self.at_detector.detect(img_gray, estimate_tag_pose=True,
                                       camera_params=self.camera_params, tag_size=self.tag_size)
        return tags
