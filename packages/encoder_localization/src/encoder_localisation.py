#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String, Float32
from geometry_msgs.msg import TransformStamped
from duckietown_msgs.msg import WheelEncoderStamped
import yaml
import tf2_ros
import numpy as np


class EncoderLocalisation(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(EncoderLocalisation, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        # get vehicle name
        self.veh_name = rospy.get_namespace().strip("/")
        rospy.loginfo(f"Using vehicle name {self.veh_name}")

        # get baseline and radius
        self.baseline, self.radius = self.get_calib_params()
        rospy.loginfo(f'baseline: {self.baseline}, radius: {self.radius}')

        # set encoder received flag
        self.encoder_received = False

        # set initial transformation from map frame to baselink frame
        self.map_to_baselink = np.asarray(
            [[-1, 0, 0, -1],
            [0, -1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]]
        )

        # construct broadcaster
        self.broadcaster = tf2_ros.TransformBroadcaster()

        # construct publisher 
        pub_topic = f'/{self.veh_name}/wheel_distance_left'
        self.pub_transform = rospy.Publisher(
            pub_topic, TransformStamped, queue_size=10)
        
        # subscribe to wheel encoders
        left_encoder_topic = f'/{self.veh_name}/left_wheel_encoder_node/tick'
        right_encoder_topic = f'/{self.veh_name}/right_wheel_encoder_node/tick'

        self.sub_encoder_ticks_left = rospy.Subscriber(
            left_encoder_topic,
            WheelEncoderStamped, self.cb_encoder_to_transform, callback_args="left"
            )
        self.sub_encoder_ticks_right = rospy.Subscriber(
            right_encoder_topic,
            WheelEncoderStamped, self.cb_encoder_to_transform, callback_args="right"
            )


    def get_calib_params(self):
        """ Load baseline and radius from calibration folder on duckiebot
        """
        cali_folder = '/data/config/calibrations/kinematics'
        cali_file = f'{cali_folder}/{self.veh_name}.yaml'

        # Locate calibration yaml file or use the default otherwise
        rospy.loginfo(f'Looking for calibration {cali_file}')
        if not os.path.isfile(cali_file):
            rospy.logwarn(f'Calibration not found: {cali_file}\n Using default instead.')
            cali_file = (f'{cali_folder}/default.yaml')

        # Shutdown if no calibration file not found
        if not os.path.isfile(cali_file):
            rospy.signal_shutdown("Found no calibration file ... aborting")

        # Load the calibration file
        with open(cali_file, 'r') as yaml_file:
            yaml_dict = yaml.load(yaml_file, Loader=yaml.Loader)

        baseline = yaml_dict['baseline']
        radius = yaml_dict['radius']

        rospy.loginfo(f'Using calibration file: {cali_file}')

        return (baseline, radius)


    def cb_encoder_to_transform(self, msg, wheel):
        """ TODO process the wheel encoder data into a transform
        """

        self.encoder_received = True
        self.tick_received_time = rospy.Time.now() # record time for publishing
        rospy.loginfo(f"Recieved message from {wheel} wheel\n{msg}")

        # # copied below is code for finding the wheel distance for the estimator
        # if (wheel == "left"):
        #     # use flag for if ticks have not been recorded yet
        #     if (not self.initialised_ticks_left):
        #         self.total_ticks_left = msg.data
        #         self.initial_ticks_left = msg.data
        #         self.initialised_ticks_left = True

        #     self.dN_ticks_left = msg.data - self.total_ticks_left
        #     self.wheel_distance_left += 2 * np.pi * self._radius \
        #         * self.dN_ticks_left / self.N_total

        #     self.total_ticks_left = msg.data

        # elif (wheel == "right"):
        #     # use flag for if ticks have not been recorded yet
        #     if (not self.initialised_ticks_right):
        #         self.total_ticks_right = msg.data
        #         self.initial_ticks_right = msg.data
        #         self.initialised_ticks_right = True

        #     self.dN_ticks_right = msg.data - self.total_ticks_right
        #     self.wheel_distance_right += 2 * np.pi * self._radius \
        #         * self.dN_ticks_right / self.N_total

        #     self.total_ticks_right = msg.data

        # else:
        #     raise NameError("wheel name not found")

        # make transform message (published in self.run)
        self.transform_msg = TransformStamped()
        # time needs to be from the last wheel encoder tick
        self.transform_msg.header.stamp = self.tick_received_time
        self.transform_msg.header.frame_id = "map" # name of parent frame
        self.transform_msg.child_frame_id = "encoder_baselink"

        # self.transform_msg.transform.translation.x = 
        # self.transform_msg.transform.translation.y = 
        # self.transform_msg.transform.translation.z = 
        # self.transform_msg.transform.rotation.x = 
        # self.transform_msg.transform.rotation.y = 
        # self.transform_msg.transform.rotation.z = 
        # self.transform_msg.transform.rotation.w = 


    def run(self):
        """ Publish and broadcast the transform_msg calculated in the callback 
        """
        rate = rospy.Rate(30) # publish at 30Hz

        while not rospy.is_shutdown():
            # don't publish until first encoder message received ?good idea?
            if self.encoder_received:
                rospy.loginfo(f'Publishing transform ...')
                self.pub_transform.publish(self.transform_msg)
                # also broadcast transform so it can be viewed in RViz
                self.broadcaster.sendTransform(self.transform_msg)

            rate.sleep() # main thread waits here between publishes

if __name__ == '__main__':
    # create the node
    node = EncoderLocalisation(node_name='encoder_localisation')
    # run node
    node.run()
