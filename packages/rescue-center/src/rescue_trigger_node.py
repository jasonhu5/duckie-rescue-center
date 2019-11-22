#!/usr/bin/env python
import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
from duckietown_msgs.msg import BoolStamped


class RescueTriggerNode(DTROS):

    def __init__(self, node_name):
        # initialize DTROS parent class
        super(RescueTriggerNode, self).__init__(node_name=node_name)
        # get robot name
        # self.veh_name = rospy.get_namespace().strip("/")
        self.veh_name = os.environ["AUTOBOT_NAME"]
        # topics
        self.pub_on = rospy.Publisher(
            "/{}/recovery_mode".format(self.veh_name), BoolStamped, queue_size=1)
        self.parameters['~trigger_rescue'] = None
        self.updateParameters()
        # current trigger status
        self.trigger = False


    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(10) # 10Hz

        while not rospy.is_shutdown():
            trigger = self.parameters["~trigger_rescue"]
            if trigger != self.trigger:
                self.trigger = trigger
                msg = BoolStamped()
                msg.data = trigger 
                self.log("Sending %s to recovery_mode" % msg.data)
                self.pub_on.publish(msg)
            rate.sleep()


if __name__ == '__main__':
    # create the node
    node = RescueTriggerNode(node_name='rescue_trigger_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
