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

        self.debug = True

        # Initialize DTROS Parent Class
        super(FusedLocalization, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self.veh_name = rospy.get_namespace().strip("/")
        self.log(f"Using vehicle name {self.veh_name}")

        # Initialize Transform Message
        self.transform_msg = TransformStamped() #published in self.run()

        # Initialize TF Broadcaster
        self.broadcaster = tf2_ros.TransformBroadcaster()

        # Initialize TF Listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.previous_apriltag_stamp = rospy.Time()

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

            # Look up Transforms
            try:
                tf_map_to_encoder_baselink = self.tfBuffer.lookup_transform('map', 'encoder_baselink', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn_throttle(1.0, "Exception caught while looking up transform map -> encoder_baselink. Retrying ...")
                rate.sleep()
                continue #jumps back to beginning of while loop

            try:
                tf_map_to_apriltag_baselink = self.tfBuffer.lookup_transform('map', 'at_baselink', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn_throttle(1.0, "Exception caught while looking up transform map -> at_baselink. Retrying ...")
                rate.sleep()
                continue #jumps back to beginning of while loop

            
            # Select relevant Transform
            if((tf_map_to_apriltag_baselink.header.stamp - self.previous_apriltag_stamp).to_sec() > 0.0): # Check if apriltag transforms is newer than last one

                if (self.debug):
                    rospy.loginfo_throttle(1.0, "[Debug] Using apriltag_localization.")
                
                self.transform_msg = tf_map_to_apriltag_baselink
                # TODO: update map frame in encoder_localization
                self.previous_apriltag_stamp = self.transform_msg.header.stamp

            else:
                
                if (self.debug):
                    rospy.loginfo_throttle(1.0, "[Debug] Using encoder_localization.")
                
                self.transform_msg = tf_map_to_encoder_baselink

            # Edit Message
            self.transform_msg.header.frame_id = "map"
            self.transform_msg.child_frame_id = "fused_baselink"
            self.transform_msg.header.stamp = rospy.Time.now()
            #self.transform_msg.transform.translation.z = 0.0 # project to ground plane

            # Broadcast Transform Message
            self.broadcaster.sendTransform(self.transform_msg)

            rate.sleep()


if __name__ == '__main__':
    # create the node
    node = FusedLocalization(node_name='fused_localization')
    # run node
    node.run()

