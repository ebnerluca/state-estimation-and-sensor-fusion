#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String, Float32
from geometry_msgs.msg import TransformStamped
from duckietown_msgs.msg import WheelEncoderStamped
from std_srvs.srv import Trigger, TriggerResponse
import yaml
import tf2_ros
from squaternion import Quaternion
import numpy as np


class EncoderLocalization(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(EncoderLocalization, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self.veh_name = rospy.get_namespace().strip("/")
        self.log(f"Using vehicle name {self.veh_name}")

        # get baseline and radius
        self.baseline, self.radius = self.get_calib_params()
        self.log(f'baseline: {self.baseline}, radius: {self.radius}')

        # initialize encoder variables
        self.encoder_received = False
        self.encoder_resolution = 135

        self.initialised_ticks_left = False
        self.total_ticks_left = 0
        self.initial_ticks_left = 0
        self.wheel_distance_left = 0.0

        self.initialised_ticks_right = False
        self.total_ticks_right = 0
        self.initial_ticks_right = 0
        self.wheel_distance_right = 0.0


        # set initial transformation from map frame to baselink frame
        self.map_to_baselink = np.asarray(
            [[-1, 0, 0, -1],
            [0, -1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]]
        )

        # initialize estimation variables in map frame
        self.theta = np.pi
        self.x = 1.0
        self.y = 0.0

        # initialize transform msg
        self.transform_msg = TransformStamped() #published in self.run()
        self.transform_msg.header.frame_id = "map" #parent frame
        self.transform_msg.child_frame_id = "encoder_baselink"

        # initialize broadcaster
        self.broadcaster = tf2_ros.TransformBroadcaster()

        # initialize publishers
        pub_topic = f'/{self.veh_name}/encoder_localization/transform'
        self.pub_transform = rospy.Publisher(
            pub_topic, TransformStamped, queue_size=10)
        
        # initialize subscribers
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
        # initialize services
        self.serv_reset = rospy.Service(f'/{self.veh_name}/encoder_localization/reset', Trigger, self.reset)


    def get_calib_params(self):
        """ Load baseline and radius from calibration folder on duckiebot
        """
        cali_folder = '/data/config/calibrations/kinematics'
        cali_file = f'{cali_folder}/{self.veh_name}.yaml'

        # Locate calibration yaml file or use the default otherwise
        self.log(f'Looking for calibration {cali_file}')
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

        self.log(f'Using calibration file: {cali_file}')

        return (baseline, radius)


    def cb_encoder_to_transform(self, msg, wheel):
        """ TODO process the wheel encoder data into a transform
        """

        # distance increments
        d_left = 0.0
        d_right = 0.0

        if (wheel == "left"):

            if (not self.initialised_ticks_left):
                self.total_ticks_left = msg.data
                self.initial_ticks_left = msg.data
                self.initialised_ticks_left = True

            dN_ticks_left = msg.data - self.total_ticks_left
            #self.wheel_distance_left += 2 * np.pi * self.radius * dN_ticks_left / self.encoder_resolution
            d_left = 2.0 * np.pi * self.radius * dN_ticks_left / self.encoder_resolution
            self.wheel_distance_left += d_left

            self.total_ticks_left = msg.data

        elif (wheel == "right"):

            if (not self.initialised_ticks_right):
                self.total_ticks_right = msg.data
                self.initial_ticks_right = msg.data
                self.initialised_ticks_right = True

            dN_ticks_right = msg.data - self.total_ticks_right
            #self.wheel_distance_right += 2 * np.pi * self.radius * dN_ticks_right / self.encoder_resolution
            d_right = 2.0 * np.pi * self.radius * dN_ticks_right / self.encoder_resolution
            self.wheel_distance_right += d_right

            self.total_ticks_right = msg.data

        else:
            raise NameError("wheel name not found")


        # Estimate x,y
        d = (d_left + d_right) / 2.0
        direction = [np.cos(self.theta), np.sin(self.theta)] # assuming x(t + dt) = x(t) + d*cos(theta(t))
        self.x += d * direction[0]
        self.y += d * direction[1]
        rospy.loginfo_throttle(1.0, f"[Debug]: x,y = {self.x}, {self.y}")

        # Estimate theta
        self.theta = np.mod( np.pi + (self.wheel_distance_right - self.wheel_distance_left) / self.baseline, 2.0 * np.pi) #makes sure theta stays between [0, 2pi]
        rospy.loginfo_throttle(1.0, f"[Debug]: theta = {self.theta * 360.0 / (2.0*np.pi)} degrees")

        # Generate Message
        #self.transform_msg.header.stamp = msg.header.stamp # TODO: tick message stamps apparently are zero
        self.transform_msg.header.stamp = rospy.Time.now()
        self.transform_msg.transform.translation.x = self.x
        self.transform_msg.transform.translation.y = self.y
        self.transform_msg.transform.translation.z = 0.0

        q = Quaternion.from_euler(0.0, 0.0, self.theta)
        self.transform_msg.transform.rotation.x = q[1]
        self.transform_msg.transform.rotation.y = q[2]
        self.transform_msg.transform.rotation.z = q[3]
        self.transform_msg.transform.rotation.w = q[0]

        self.encoder_received = True



    def run(self):
        """ Publish and broadcast the transform_msg calculated in the callback 
        """
        rate = rospy.Rate(30) # publish at 30Hz

        while not rospy.is_shutdown():
            
            if self.encoder_received: #don't publish if no new encoder message was received
                
                self.pub_transform.publish(self.transform_msg) #for debug
                self.broadcaster.sendTransform(self.transform_msg) #for tf tree
                self.encoder_received = False

            rate.sleep() # main thread waits here between publishes

    def reset(self, request):

        self.initialised_ticks_left = False
        self.wheel_distance_left = 0.0

        self.initialised_ticks_right = False
        self.wheel_distance_right = 0.0

        #TODO maybe needs change? (compare to self.map_to_baselink)
        self.theta = np.pi
        self.x = 1.0
        self.y = 0.0

        return TriggerResponse(
        success=True,
        message="Reset! (initialized_ticks_left/right = False, wheel_distance_left/right = 0.0, theta,x,y = 0.0)"
        )

        self.log("Reset!")

if __name__ == '__main__':
    # create the node
    node = EncoderLocalization(node_name='encoder_localization')
    # run node
    node.run()

