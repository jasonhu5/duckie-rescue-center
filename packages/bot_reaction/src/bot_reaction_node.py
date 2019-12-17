#!/usr/bin/env python

import os
import rospy
from duckietown import DTROS
from duckietown_msgs.msg import BoolStamped
import socket
import sys


class BotReactionNode(DTROS):

    def __init__(self, node_name):

        # initialize DTROS parent class
        super(BotReactionNode, self).__init__(node_name=node_name)

        # Subscribe to topics
        self.veh_list = [25, 26, 27, 28]
        self.sub_list = {}


        for veh in self.veh_list:
            self.sub_list[veh] = rospy.Subscriber(
                "/autobot{}/recovery_mode".format(veh),
                BoolStamped,
                self.callback,
                veh
            )

    def callback(self, msg, veh):
        message = "{},{}".format(veh, msg.data)
        # Create a UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_address = ('localhost', 10000)
        try:
            # Send data
            sent = sock.sendto(message, server_address)
        finally:
            sock.close()


if __name__ == '__main__':
    # create the node
    node = BotReactionNode(node_name='bot_reaction_node')
    # # run node
    # node.run()
    # keep spinning
    rospy.spin()