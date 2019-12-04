#!/usr/bin/env python

import os
import rospy
import math
import subprocess
from duckietown import DTROS
from std_msgs.msg import String
from duckietown_msgs.msg import BoolStamped, Twist2DStamped, FSMState
from visualization_msgs.msg import Marker, MarkerArray


class RescueTriggerNode(DTROS):

    def __init__(self, node_name):

        # initialize DTROS parent class
        super(RescueTriggerNode, self).__init__(node_name=node_name)

        # List of current bots
        self.veh_list = list()
        # Dictionary with up to date information on all bots
        self.id_dict = dict()

        ## parameters
        # threshold distance to consider bot not moving
        self.parameters['~dist_thres'] = None
        # number of windows to use for filtering
        self.parameters['~avg_window'] = None
        self.updateParameters()

        # Subscribe to topics online localization
        self.sub_markers = rospy.Subscriber(
            "/cslam_markers", MarkerArray, self.cbLocalization, queue_size=30)

        # Prepare for bot-wise topics
        self.pub_trigger = dict()
        self.pub_rescue_classfication = dict()
        self.sub_fsm_states = dict()


    def updateSubscriberPublisher(self, veh_id):
        '''
        Updates Subscriber and Publisher dictionaries, if new duckiebot is added
        Input: veh_id (int)
        '''

        # 1. Publisher: Trigger
        self.pub_trigger[veh_id] = rospy.Publisher(
            "/autobot{}/recovery_mode".format(veh_id),
            BoolStamped,
            queue_size=1
        )
        # 2. Publisher: Distress classification
        self.pub_rescue_classfication[veh_id] = rospy.Publisher(
            "~/autobot{}/distress_classification".format(veh_id),
            String,
            queue_size=1
        )
        # 3. Subscriber: FSM state
        self.sub_fsm_states[veh_id] = rospy.Subscriber(
            "/autobot{}/fsm_mode".format(veh_id),
            FSMState,
            self.cbFSM,
            callback_args=veh_id
        )
    

    def cbLocalization(self, msg):
        """
        Callback when receiving online localization results
        """

        for m in msg.markers:
            # care only about duckiebots
            if m.ns != "duckiebots":
                continue

            idx = m.id
            # Append new bots to list if they show up
            if not idx in self.veh_list:
                self.veh_list.append(idx)
                self.id_dict[idx] = AutobotInfo()
                # create relevant topic handlers for this bot
                self.updateSubscriberPublisher(idx) 
                # create rescue agent for this bot
                subprocess.Popen([
                    "roslaunch", "rescue-center",
                    "rescue_agent.launch", "botID:={}".format(idx)
                ])

            # Store position from localization system
            self.id_dict[idx].position = (m.pose.position.x, m.pose.position.y)

            # Filter position and update last_moved time stamp
            self.id_dict[idx].update_filtered(
                m.header.stamp,
                self.parameters['~dist_thres'],
                self.parameters['~avg_window'],
            )

            if self.id_dict[idx].in_rescue:
                continue
            # If duckiebot is not currently in rescue, check classifier
            time_diff = 0.0
            rescue_class = self.id_dict[idx].classifier(time_diff)
            if rescue_class != 0:
                # publish rescue_class
                self.log(
                    "Duckiebot [{}] in distress <{}>".format(idx, rescue_class))
                msg = BoolStamped()
                msg.data = True
                # turn duckiebot into rescue mode
                self.pub_trigger[idx].publish(msg)
                # notify rescue agent of the classification result
                self.pub_rescue_classfication[idx].publish(str(rescue_class))
                # note down so can skip the next time
                self.id_dict[idx].in_rescue = True


    # Callback for fsm state
    def cbFSM(self, msg, veh_id):
        """
        callback when the fsm state of a bot changed
        """
        if veh_id in self.id_dict:
            self.log("Duckiebot [{}] FSM state <{}>".format(veh_id, msg.state))
            self.id_dict[veh_id].fsm_state = msg.state


    # def run(self):
    #     # publish rate
    #     rate = rospy.Rate(4) # 10Hz
    #     while not rospy.is_shutdown():
    #         rate.sleep()


class AutobotInfo():
    def __init__(self):
        self.fsm_state = None
        self.position = (None, None)
        self.filtered = (float('inf'), float('inf'))
        self.last_moved = None
        self.heading = None
        self.in_rescue = False

    def update_filtered(self, timestamp, threshold, window):
        pass

    def classifier(self, time_diff):
        return 1 if rospy.get_param('~trigger_rescue') else 0


if __name__ == '__main__':
    # create the node
    node = RescueTriggerNode(node_name='rescue_trigger_node')
    # # run node
    # node.run()
    # keep spinning
    rospy.spin()