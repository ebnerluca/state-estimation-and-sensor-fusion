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

        # Initialize DTROS Parent Class
        super(EncoderLocalization, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self.veh_name = rospy.get_namespace().strip("/")
        self.log(f"Using vehicle name {self.veh_name}")

        # Get Baseline and Radius
        self.baseline, self.radius = self.get_calib_params()
        self.log(f'Parameters: baseline = {self.baseline}, radius = {self.radius}')

        # Initialize Encoder Variables
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

        # Initialize Estimation Variables (in map reference frame)
        self.theta = np.pi
        self.x = 1.0
        self.y = 0.0

        # Initialize Transform Message
        self.transform_msg = TransformStamped() #published in self.run()
        self.transform_msg.header.frame_id = "map" #parent frame
        self.transform_msg.child_frame_id = "encoder_baselink"
        self.transform_msg.transform.translation.z = 0.0

        # Initialize TF Broadcaster
        self.broadcaster = tf2_ros.TransformBroadcaster()

        # Initialize Publishers
        pub_topic = f'/{self.veh_name}/encoder_localization/transform'
        self.pub_transform = rospy.Publisher(
            pub_topic, TransformStamped, queue_size=10)
        
        # Initialize Subscribers
        left_encoder_topic = f'/{self.veh_name}/left_wheel_encoder_node/tick'
        right_encoder_topic = f'/{self.veh_name}/right_wheel_encoder_node/tick'
        self.sub_encoder_ticks_left = rospy.Subscriber(left_encoder_topic, WheelEncoderStamped, self.cb_encoder_to_transform, callback_args="left")
        self.sub_encoder_ticks_right = rospy.Subscriber(right_encoder_topic, WheelEncoderStamped, self.cb_encoder_to_transform, callback_args="right")

        # Initialize Services
        self.serv_reset = rospy.Service(f'/{self.veh_name}/encoder_localization/reset', Trigger, self.reset)

        self.log("Initialized.")

    def get_calib_params(self):
        """ Load baseline and radius parameters from calibration folder on duckiebot.
        """
        cali_folder = '/data/config/calibrations/kinematics'
        cali_file = f'{cali_folder}/{self.veh_name}.yaml'

        # Locate Calibration .yaml File
        self.log(f'Looking for calibration {cali_file}')
        if not os.path.isfile(cali_file):
            rospy.logwarn(f'Calibration not found: {cali_file}\n Using default instead.')
            cali_file = (f'{cali_folder}/default.yaml')

        # Shutdown if calibration file not found
        if not os.path.isfile(cali_file):
            rospy.signal_shutdown("Found no calibration file ... aborting")

        # Load File
        with open(cali_file, 'r') as yaml_file:
            yaml_dict = yaml.load(yaml_file, Loader=yaml.Loader)

        baseline = yaml_dict['baseline']
        radius = yaml_dict['radius']

        self.log(f'Using calibration file: {cali_file}')

        return (baseline, radius)


    def cb_encoder_to_transform(self, msg, wheel):
        """ Processes the wheel encoder data and estimates the robot 2D pose.
        """

        # Initialize Distance Increments
        d_left = 0.0
        d_right = 0.0


        # Distinguish between Left and Right
        if (wheel == "left"):

            if (not self.initialised_ticks_left):
                self.total_ticks_left = msg.data
                self.initial_ticks_left = msg.data
                self.initialised_ticks_left = True

            dN_ticks_left = msg.data - self.total_ticks_left
            d_left = 2.0 * np.pi * self.radius * dN_ticks_left / self.encoder_resolution
            self.wheel_distance_left += d_left

            self.total_ticks_left = msg.data

        elif (wheel == "right"):

            if (not self.initialised_ticks_right):
                self.total_ticks_right = msg.data
                self.initial_ticks_right = msg.data
                self.initialised_ticks_right = True

            dN_ticks_right = msg.data - self.total_ticks_right
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
        rospy.loginfo_throttle(1.0, f"[Debug]: x,y = {self.x}, {self.y} meters")


        # Estimate theta
        self.theta = np.mod( np.pi + (self.wheel_distance_right - self.wheel_distance_left) / self.baseline, 2.0 * np.pi) #makes sure theta stays between [0, 2pi]
        rospy.loginfo_throttle(1.0, f"[Debug]: theta = {self.theta * 360.0 / (2.0*np.pi)} degrees")


        # Update Transform
        self.update_transform()


        # Set Flag
        self.encoder_received = True



    def update_transform(self):
        """ updates the transform from map frame to encoder_baselink frame according to the current estimation
        """

        #self.transform_msg.header.stamp = msg.header.stamp # TODO: tick message stamps apparently are zero
        self.transform_msg.header.stamp = rospy.Time.now()
        self.transform_msg.transform.translation.x = self.x
        self.transform_msg.transform.translation.y = self.y

        q = Quaternion.from_euler(0.0, 0.0, self.theta) #inputs: roll,pitch,yaw
        self.transform_msg.transform.rotation.x = q[1]
        self.transform_msg.transform.rotation.y = q[2]
        self.transform_msg.transform.rotation.z = q[3]
        self.transform_msg.transform.rotation.w = q[0]


    def run(self):
        """ Publish and broadcast the transform_msg calculated in the callback 
        """
        rate = rospy.Rate(30) # publish at 30Hz

        while not rospy.is_shutdown():
            
            if self.encoder_received: #don't publish if no new encoder message was received
                
                self.pub_transform.publish(self.transform_msg) #for debug
                self.broadcaster.sendTransform(self.transform_msg) #for tf tree
                self.encoder_received = False #prevent publishing old messages

            rate.sleep() # main thread waits here between publishes

    def reset(self, request):

        self.initialised_ticks_left = False
        self.wheel_distance_left = 0.0

        self.initialised_ticks_right = False
        self.wheel_distance_right = 0.0

        self.theta = np.pi
        self.x = 1.0
        self.y = 0.0

        self.update_transform()
        self.encoder_received = True 

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

