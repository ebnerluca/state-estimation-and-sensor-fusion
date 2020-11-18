#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String, Float32
from geometry_msgs.msg import TransformStamped
#from duckietown_msgs.msg import WheelEncoderStamped
from std_srvs.srv import Trigger, TriggerResponse
#import yaml
import tf2_ros
#from squaternion import Quaternion
import numpy as np


class FusedLocalization(DTROS):

    def __init__(self, node_name):

        # Initialize DTROS Parent Class
        super(FusedLocalization, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self.veh_name = rospy.get_namespace().strip("/")
        self.log(f"Using vehicle name {self.veh_name}")

        # Initialize Transform Message
        self.transform_msg = TransformStamped() #published in self.run()
        self.transform_msg.header.frame_id = "map" #parent frame
        self.transform_msg.child_frame_id = "fused_baselink"
        self.transform_msg.transform.translation.z = 0.0

        # Initialize TF Broadcaster
        # TODO

        # Initialize TF Listener
        # TODO

        # Initialize Publishers
        # TODO

        # Initialize Subscribers
        # TODO
        
        # Initialize Services
        # TODO

        self.log("Initialized.")


    def run(self):
        """ Publish and broadcast the transform_msg calculated in the callback 
        """
        rate = rospy.Rate(30) # publish at 30Hz

        while not rospy.is_shutdown():

            rate.sleep() # main thread waits here between publishes


if __name__ == '__main__':
    # create the node
    node = FusedLocalization(node_name='fused_localization')
    # run node
    node.run()

