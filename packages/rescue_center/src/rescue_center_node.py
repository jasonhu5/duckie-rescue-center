#!/usr/bin/env python

import os
import rospy
import math
import subprocess
from duckietown import DTROS
from std_msgs.msg import String, Float64MultiArray
from duckietown_msgs.msg import BoolStamped, Twist2DStamped, FSMState
from visualization_msgs.msg import Marker, MarkerArray
from simple_map import SimpleMap
from autobot_info import AutobotInfo, Distress
from nav_msgs.msg import Path
from rescue_center.msg import AutobotInfoMsg
# 11 


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
        self.parameters['~dist_thres'] = 0.01 # TODO: tune parameter
        # number of windows to use for filtering
        self.parameters['~avg_window'] = 10 # TODO. tune parameter
        self.updateParameters()

        # Subscribe to topics online localization
        self.sub_markers = rospy.Subscriber(
            "/cslam_markers", MarkerArray, self.cbLocalization, queue_size=30)
        # for simple-localization
        self.sub_simpleLoc = rospy.Subscriber(
            "/simple_loc_bots", Float64MultiArray, self.cbSimpleLoc, queue_size=30)

        # Prepare for bot-wise topics
        self.pub_trigger = dict()
        self.pub_rescue_classfication = dict()
        self.sub_fsm_states = dict()
        self.sub_everythingOk = dict()
        self.sub_path = dict()
        self.pub_autobot_info = dict()

        # build simpleMap
        # TODO: pull a duckietownworld fork in container
        map_file_path = os.path.join(
            "/code/catkin_ws/src",
            "duckie-rescue-center/packages/rescue_center/src",
            "test_map.yaml"
        )
        self.map = SimpleMap(map_file_path)
        self.map.display_raw_debug()

        #save current time:
        self.currentTime = None


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
        # 4. Subscriber: EverythingOK from Rescue_agend
        self.sub_everythingOk[veh_id] = rospy.Subscriber(
            "/rescue/rescue_agents/autobot{}/rescueDone".format(veh_id),
            BoolStamped,
            self.cbEverythingOk,
            callback_args=veh_id
        )
        #5. Subscriber: Path
        self.sub_path[veh_id] = rospy.Subscriber(
            "/movable_path_autobot{}".format(veh_id), Path, self.cbPath, callback_args=veh_id, queue_size=30)

        # test
        self.pub_autobot_info[veh_id] = rospy.Publisher(
            "/autobot{}/autobot_info".format(veh_id),
            AutobotInfoMsg,
            queue_size=5,
        )


    def cbSimpleLoc(self, msg):
        # TODO: Jason will change the simple localization output from tag id to autobot id
        if (msg.data[0] == 426):
            veh_id = 27
        elif (msg.data[0] == 425):
            veh_id = 26
        else:
            veh_id = msg.data[0]
        
        # veh_id = int(msg.data[0])

        self.id_dict[veh_id].positionSimple = (msg.data[1], msg.data[2])
        self.id_dict[veh_id].headingSimple = msg.data[3] 
        self.pub_autobot_info[veh_id].publish(self.autobotInfo2Msg(self.id_dict[veh_id]))
        # print("[{}] Received simple localization: ({}, {}, {})".format(veh_id, msg.data[1], msg.data[2], msg.data[3]))
        print("[{}] Received simple localization: ({}, {})".format(veh_id, self.id_dict[veh_id].positionSimple, self.id_dict[veh_id].headingSimple))

    def cbPath(self, msg, veh_id):
        self.id_dict[veh_id].updatePath(msg)
    
    def cbEverythingOk(self, msg, veh_id):
        if msg.data == True:
            self.id_dict[veh_id].in_rescue = False
            self.id_dict[veh_id].last_moved = self.currentTime
            msg = BoolStamped()
            msg.data = False
            self.pub_trigger[veh_id].publish(msg)


    def autobotInfo2Msg(self, info):
        msg = AutobotInfoMsg()
        if info.timestamp:
            msg.timestamp = info.timestamp
        if info.fsm_state:
            msg.fsm_state = info.fsm_state
        if info.position[0]:
            msg.position = [info.position[0], info.position[1]]
        if info.filtered:
            msg.filtered = [info.filtered[0], info.filtered[1]]
        # msg.heading = 
        if info.last_moved:
            msg.last_moved = info.last_moved
        if info.in_rescue:
            msg.in_rescue = info.in_rescue
        if info.onRoad:
            msg.onRoad = info.onRoad
        if info.rescue_class:
            msg.rescue_class = info.rescue_class.value 
        if info.heading:
            msg.heading = info.heading
        # for simpleLoc:
        if info.positionSimple[0]:
            msg.positionSimple = [info.positionSimple[0], info.positionSimple[1]]
        if info.headingSimple:
            msg.headingSimple = info.headingSimple
        # msg.path =  
        return msg


    def cbLocalization(self, msg):
        """
        Callback when receiving online localization results
        """

        for m in msg.markers:
            # care only about duckiebots
            if m.ns != "duckiebots":
                continue

            idx = m.id
            # Append new bots to list if they show up the first time
            if not idx in self.veh_list:
                self.veh_list.append(idx)
                self.id_dict[idx] = AutobotInfo()
                # create relevant topic handlers for this bot
                self.updateSubscriberPublisher(idx) 
                # create rescue agent for this bot
                subprocess.Popen([
                    "roslaunch", "rescue_center",
                    "rescue_agent.launch", "botID:={}".format(idx)
                ])

            # Store position from localization system in AutoboInfo()
            self.id_dict[idx].update_from_marker(m)
            # print("[{}] heading: {}".format(idx, self.id_dict[idx].heading))
            # Filter position and update last_moved time stamp
            self.id_dict[idx].update_filtered(
                m.header.stamp,
                self.parameters['~dist_thres'],
                self.parameters['~avg_window'],
            )
             # publish autobot_info to rescue_agent
            self.pub_autobot_info[idx].publish(self.autobotInfo2Msg(self.id_dict[idx]))

            
            if self.id_dict[idx].in_rescue:
                continue
            # If duckiebot is not in rescue, check classifier
            self.currentTime = m.header.stamp
            time_diff = self.currentTime.to_sec()-self.id_dict[idx].last_moved.to_sec()
            rescue_class = self.id_dict[idx].classifier(time_diff, self.map)
            self.id_dict[idx].rescue_class = rescue_class
            # For debugging
            if idx == 27:
                print("[{}]: time diff[s]: {}".format(idx, time_diff))
                print("[{}] ({}) onRoad {}".format(idx, self.id_dict[idx].position, self.id_dict[idx].onRoad))

            if rescue_class.value != 0 and idx == 27:
                # publish rescue_class
                self.log(
                    "Duckiebot [{}] in distress <{}>".format(idx, rescue_class))
                msg = BoolStamped()
                msg.data = True
                # turn duckiebot into rescue mode
                self.pub_trigger[idx].publish(msg)
                # notify rescue agent of the classification result
                self.pub_rescue_classfication[idx].publish(str(rescue_class.value))
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


if __name__ == '__main__':
    # create the node
    node = RescueTriggerNode(node_name='rescue_trigger_node')
    # # run node
    # node.run()
    # keep spinning
    rospy.spin()