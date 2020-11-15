#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String, Float32
from geometry_msgs.msg import TransformStamped
from duckietown_msgs.msg import WheelEncoderStamped


class EncoderLocalisation(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(EncoderLocalisation, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        # get vehicle name
        self.veh_name = rospy.get_namespace().strip("/")
        rospy.loginfo(f"Using vehicle name {self.veh_name}")

        # construct publisher 
        pub_topic = f'/{self.veh_name}/wheel_distance_left'
        self.pub_transform = rospy.Publisher(
            pub_topic, TransformStamped, queue_size=10)
        
        # subscribe to wheel encoders
        left_encoder_topic = f'/{self.veh_name}/left_wheel_encoder_node/tick'
        right_encoder_topic = f'/{self.veh_name}/right_wheel_encoder_node/tick'

        self.sub_encoder_ticks_left = rospy.Subscriber(
            left_encoder_topic,
            WheelEncoderStamped, self.cb_encoder_data, callback_args="left"
            )
        self.sub_encoder_ticks_right = rospy.Subscriber(
            right_encoder_topic,
            WheelEncoderStamped, self.cb_encoder_data, callback_args="right"
            )


    def cb_encoder_data(self, msg, wheel):
        """ TODO process the wheel encoder data into a transform
        """

        rospy.loginfo(f"Recieved message from {wheel} wheel\n{msg}")

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


    def run(self):
        #TODO publish transform
        rate = rospy.Rate(30) # publish at 30Hz
        while not rospy.is_shutdown():
            rospy.loginfo(f'Publishing transform ...')
            # self.pub_transform.publish()
            rate.sleep() # main thread waits here between publishes

if __name__ == '__main__':
    # create the node
    node = EncoderLocalisation(node_name='encoder_localisation')
    # run node
    node.run()
