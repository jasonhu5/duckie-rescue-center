#!/usr/bin/env python

import os
import rospy
import math
import subprocess
from duckietown import DTROS
from std_msgs.msg import String
from duckietown_msgs.msg import BoolStamped, Twist2DStamped, FSMState
# from geometry_msgs.msg import TransformStamped, Transform
from visualization_msgs.msg import Marker, MarkerArray


class RescueTriggerNode(DTROS):

    def __init__(self, node_name):

        # initialize DTROS parent class
        super(RescueTriggerNode, self).__init__(node_name=node_name)

        # List of current bots
        self.veh_list = list()
        # Dictionary with up to date information on all bots
        self.id_dict = dict()
        # Threshold for averaging filter
        self.dist_thres = 0.5
        # Window for averaging filter
        self.avg_window = 10

        # Subscribe to topics
        # 1. Online localization
        self.sub_markers = rospy.Subscriber(
            "/cslam_markers", MarkerArray, self.cb_localization, queue_size=30)
        # 2. FSM states
        self.sub_fsmStates = dict()
        for veh_name in self.veh_list:
            topic_name = "/{}/fsm_node/mode".format("autobot"+veh_name)
            self.sub_fsmStates[veh_name] = rospy.Subscriber("%s"%(topic_name), FSMState, self.cb_fsm, callback_args=veh_name)
    

        # Publish to topics
        # 1. Trigger for each duckiebot
        self.pub_trigger = dict()
        for veh_name in self.veh_list:
            self.pub_trigger[veh_name] = rospy.Publisher(
                "/{}/recovery_mode".format("autobot"+veh_name),
                BoolStamped,
                queue_size=1
            )
        # 2. Rescue Classification for rescue_agent
        self.pub_rescueClassfication = dict()
        for veh_name in self.veh_list:
            self.pub_rescueClassfication[veh_name] = rospy.Publisher(
                "~/{}/distress_classification".format("autobot"+veh_name),
                String,
                queue_size=1
            )


    # Callback for online localization
    def cb_localization(self, msg):
        # print("Received cslam message")
        markers = msg.markers
        for m in markers:
            if m.ns == "duckiebots":
                idx = str(m.id)
                # Append new bots to list if they show up
                if not idx in self.veh_list:
                    self.veh_list.append(idx)
                    self.id_dict[idx] = AutobotInfo()
                    self.updateSubscriberPublisher(idx) 
                    subprocess.Popen([
                        "roslaunch", "rescue-center",
                        "rescue_agent.launch", "botID:={}".format(idx)
                    ])
                # Store position from localization system
                self.id_dict[idx].position = (m.pose.position.x, m.pose.position.y)
                # print("Deteced bot {} at {}".format(idx, self.id_dict[idx].position))
                # Filter position and update last_moved time stamp
                self.id_dict[idx].update_filtered(m.header.stamp, self.dist_thres, self.avg_window)
                # print("Filtered bot {} at {}".format(idx, self.id_dict[idx].filtered))
                # If duckiebot is not currently in rescue, call classifier
                if not self.id_dict[idx].in_rescue:
                    rescue_class = self.classifier()
                    self.id_dict[idx].rescue_class = rescue_class
                    if rescue_class != 0:
                        # publish rescue_class
                        self.log("Duckiebot {} is in distress: {}".format(idx, rescue_class))
                        msg = BoolStamped()
                        msg.data = True
                        self.pub_trigger[idx].publish(msg)
                        self.pub_rescueClassfication[idx].publish(str(rescue_class))
                        self.id_dict[idx].in_rescue = True

                # self.log("{}".format("Should shop" if y > 0.6 else "Fine"))

    def updateSubscriberPublisher(self, veh_id):
        '''Updates Subscriber and Publisher dictionaries, if new duckiebot is added
        Input: veh_id (str)'''
        # 1. Publisher: Trigger
        self.pub_trigger[veh_id] = rospy.Publisher(
                        "/{}/recovery_mode".format("autobot"+veh_id),
                        BoolStamped,
                        queue_size=1
                    )
        # 2. Publisher: Distress classification
        self.pub_rescueClassfication[veh_id] = rospy.Publisher(
                "~/{}/distress_classification".format("autobot"+veh_id),
                String,
                queue_size=1
        )
        # 3. Subscriber: FSM state
        topic_name = "/{}/fsm_node/mode".format("autobot"+veh_id)
        self.sub_fsmStates[veh_id] = rospy.Subscriber("%s"%(topic_name), FSMState, self.cb_fsm, callback_args=veh_id)
    
    def classifier(self):
        rescue_class = 0
        trigger_rescue = rospy.get_param('~trigger_rescue')
        # print(trigger_rescue)
        if trigger_rescue:
            print("Class change triggered")
            rescue_class = 1
        return rescue_class

    # Callback for fsm state
    def cb_fsm(self, msg, veh_name):
        if veh_name in self.id_dict:
            print("Received FSM state from: {}".format(veh_name))
            self.id_dict[veh_name].fsm_state = msg.state

    def run(self):

        # publish rate
        rate = rospy.Rate(4) # 10Hz

        while not rospy.is_shutdown():
            # print("Rescue_node running...")

            rate.sleep()

class AutobotInfo():
    def __init__(self):
        self.fsm_state = None
        self.position = (None, None)
        self.filtered = (float('inf'), float('inf'))
        self.last_moved = None
        self.heading = None
        self.rescue_class = 0
        self.in_rescue = False

    def distance(self, positionA, positionB):
        xA, yA = positionA
        xB, yB = positionB
        return math.sqrt((xA-xB)**2 + (yA-yB)**2)

    def avg_filter(self, positionNew, positionAvg, a):
        xNew, yNew = positionNew
        xAvg, yAvg = positionAvg
        return (a*xNew + (1-a)*xAvg, a*yNew + (1-a)*yAvg)

    def update_filtered(self, timestamp, threshold, window):
        if self.distance(self.position, self.filtered) > threshold:
            self.filtered = self.position
            self.last_moved = timestamp
        else:
            self.filtered = self.avg_filter(self.position, self.filtered, 1/window)

if __name__ == '__main__':
    # create the node
    node = RescueTriggerNode(node_name='rescue_trigger_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
