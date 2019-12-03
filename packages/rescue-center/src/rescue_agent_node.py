#!/usr/bin/env python
import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String


class RescueAgentNode(DTROS):

    def __init__(self, node_name):

        # initialize DTROS parent class
        super(RescueAgentNode, self).__init__(node_name=node_name)

        # get robot name
        self.veh_num = rospy.get_namespace().strip("/").split("autobot")[-1]

        # publish to topics
        self.pub_tst = rospy.Publisher(
            "/rescue_agents/test",
            String,
            queue_size=1,
        )


    def run(self):

        # publish rate
        rate = rospy.Rate(0.3)

        while not rospy.is_shutdown():
            self.pub_tst.publish("Hello from autobot{}".format(self.veh_num)) 
            rate.sleep()


if __name__ == '__main__':
    # create the node
    node = RescueAgentNode(node_name='rescue_agent_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
