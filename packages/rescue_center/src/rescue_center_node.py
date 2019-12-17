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
from time import sleep


class RescueCenterNode(DTROS):
    """Rescue Center Node

    - Monitors all bots currenty visible on the map and classifies if they are in distress or not
    - Responsible for launching and alarming Rescue Agents 

    Args:
        node_name (): a unique, descriptive name for the node that ROS will use

    Attributes:
        - veh_list (list): A list of all vehicles visible on the map
        - veh_dict (dist): A dict of all vehicles visible on the map (key=id, value=AutobotInfo)
        - parameters: Used to pass certain parameters to the node via rosparam
        - map: A map of type SimpleMap(), necessary for classification
        - The following dictionaries to store the bot_wise subcribers and publishers 
        for communication between rescue center and rescue agents:
            - pub_trigger
            - pub_rescue_classfication
            - sub_fsm_states
            - sub_everythingOk
            - pub_autobot_info

    Subscriber:
        - sub_markers: /cslam_markers (:obj:`MarkerArray`): 
            Rviz markers from the online localization systems
        - sub_simpleLoc: /simple_loc_bots (:obj: `Float64MultiArray`): 
            Position and heading of the bots from the SimpleLoc system
        - sub_fsm_states[veh_id]: /autobot{}/fsm_mode (:obj:`FSMstate`)
            FSM state from every duckiebot
        - sub_everythingOk[veh_id]: /rescue/rescue_agents/autobot{}/rescueDone (:obj:`BoolStamped`)
            Used to get the signal from the rescue agent, that a rescue operation is done

    Publisher:
        - pub_trigger[veh_id]: /autobot{}/recovery_mode (:obj:`BoolStamped`): 
            Triggers the recovery mode for a certain bot and activates its rescue agent 
        - pub_rescue_classfication[veh_id]: ~/autobot{}/distress_classification (:obj: `String`):
            Gives the result of the latest classification to each rescue agent
        - pub_autobot_info[veh_id]: /autobot{}/autobot_info (:obj: `AutobotInfoMsg`):
            Passes all collected information about a duckiebot to the rescue agent
    """

    def __init__(self, node_name):

        # initialize DTROS parent class
        super(RescueCenterNode, self).__init__(node_name=node_name)

        # List of current bots
        self.veh_list = list()
        # Dictionary with up to date information on all bots
        self.id_dict = dict()

        ## parameters
        # threshold distance to consider bot not moving
        self.parameters['~dist_thres'] = 0.01 # TODO: tune parameter
        # number of windows to use for filtering, currently not used
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
        self.pub_autobot_info = dict()
        self.pub_rescue_stopped = dict()

        # build simpleMap
        # TODO: pull a duckietownworld fork in container
        map_file_path = os.path.join(
            "/code/catkin_ws/src",
            "duckie-rescue-center/packages/rescue_center/src",
            "test_map.yaml"
        )
        self.map = SimpleMap(map_file_path)
        self.map.display_raw_debug()


    def updateSubscriberPublisher(self, veh_id):
        """Updates Subscriber and Publisher dictionaries, if new duckiebot is added
        
        Args: 
            veh_id: ID of the new duckiebot spotted by the localization system (int)
        """

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
        # 5. Publisher: AutobotInfo Msg
        self.pub_autobot_info[veh_id] = rospy.Publisher(
            "/autobot{}/autobot_info".format(veh_id),
            AutobotInfoMsg,
            queue_size=5,
        )

        #6. Subscriber: Rescue stopped (not finished)
        self.pub_rescue_stopped[veh_id] = rospy.Subscriber(
            "/rescue/rescue_agents/autobot{}/rescueStopped/".format(veh_id), BoolStamped, self.cbRescueStopped,
            callback_args=veh_id)

    def cbRescueStopped(self, msg, veh_id):
        """Callback if rescue is stopped, but not finished"""
        print("Received rescue stopped!")
        if msg.data == True:
            self.id_dict[veh_id].in_rescue = False
            self.id_dict[veh_id].last_moved = self.id_dict[veh_id].timestamp
            

    def cbSimpleLoc(self, msg):
        """Callback of SimpleLoc, reads message, stores position and heading in
        AutobotInfo and publishes it
        
        Args: 
            msg: ROS message (Float64MultiArray)
        """

        veh_id = int(msg.data[0])
        if veh_id in self.id_dict:
            self.id_dict[veh_id].positionSimple = (msg.data[1], msg.data[2])
            self.id_dict[veh_id].headingSimple = msg.data[3]
        # if self.id_dict[veh_id].lastPosititionSimple[0]:
        #     distance = math.sqrt((self.id_dict[veh_id].positionSimple[0]-self.id_dict[veh_id].lastPosititionSimple[0])**2
        #                         (self.id_dict[veh_id].positionSimple[1]-self.id_dict[veh_id].lastPosititionSimple[1])**2)
        #     if distance > 0.02: 
        #         self.id_dict[veh_id].last_movedSimple = msg.data[4] # in s
        #         self.id_dict[veh_id].lastPosititionSimple = self.id_dict[veh_id].positionSimple
        self.id_dict[veh_id].last_movedSimple = msg.data[4] # in s
        self.pub_autobot_info[veh_id].publish(self.id_dict[veh_id].autobotInfo2Msg())
    
    def cbEverythingOk(self, msg, veh_id):
        """Callback of RescueAgent, which signals that the rescue operation is over
        
        Args: 
            msg: ROS message (Float64Array)
            veh_id: ID of duckiebot
        """

        if msg.data == True:
            self.id_dict[veh_id].in_rescue = False
            self.id_dict[veh_id].last_moved = self.id_dict[veh_id].timestamp
            msg = BoolStamped()
            msg.data = False
            self.pub_trigger[veh_id].publish(msg)


    def cbLocalization(self, msg):
        """ Callback when receiving online localization results
        Checks if there are any new bots on the map
        Reads the results from the online localization
        Calls the Classifier for each bot that is currently not in rescue operation

        Args: 
            msg: ROS visualization message (MarkerArray)
        """

        for m in msg.markers:
            # care only about duckiebots
            if m.ns != "duckiebots":
                continue

            idx = m.id
            # Append new bots to list if they show up the first time
            if not idx in self.veh_list:
                print("Detected new duckiebot [{}]. Launching new rescue agent ...".format(idx))
                self.veh_list.append(idx)
                self.id_dict[idx] = AutobotInfo(idx)
                # create relevant topic handlers for this bot
                self.updateSubscriberPublisher(idx) 
                # create rescue agent for this bot
                subprocess.Popen([
                    "roslaunch", "rescue_center",
                    "rescue_agent.launch", "botID:={}".format(idx)
                ])
                sleep(3)
                print("Finished launching")

            # Store position from localization system in AutoboInfo()
            self.id_dict[idx].update_from_marker(m)
            # Filter position and update last_moved time stamp
            self.id_dict[idx].update_filtered(
                self.parameters['~dist_thres'],
                self.parameters['~avg_window'],
            )
            # publish autobot_info to rescue_agent
            self.pub_autobot_info[idx].publish(self.id_dict[idx].autobotInfo2Msg())

            # add duckiebots to being monitored:
            change_monitored_duckiebots = rospy.get_param('~change_monitored_duckiebots') 
            if change_monitored_duckiebots:
                print("Please specify, which duckiebot to monitor")
                add_duckiebot = rospy.get_param('~add_duckiebot')
                remove_duckiebot = rospy.get_param('~remove_duckiebot')
                if add_duckiebot in self.id_dict and self.id_dict[add_duckiebot].monitoringActivated == False:
                    self.id_dict[add_duckiebot].monitoringActivated = True
                    # to prevent time_diff being triggered
                    self.id_dict[add_duckiebot].last_moved = m.header.stamp.to_sec()
                    self.id_dict[add_duckiebot].last_movedSimple = m.header.stamp.to_sec()
                    print("[{}] is now being monitored for rescue.".format(add_duckiebot))
                    rospy.set_param('~add_duckiebot', -1)
                    rospy.set_param('~change_monitored_duckiebots', False)
                if remove_duckiebot in self.id_dict:
                    self.id_dict[remove_duckiebot].monitoringActivated = False
                    print("[{}] is not being monitored for rescue.".format(remove_duckiebot))
                    rospy.set_param('~remove_duckiebot', -1) 
                    rospy.set_param('~change_monitored_duckiebots', False)

            # check, if we want to monitor that duckiebot
            if self.id_dict[idx].monitoringActivated == False:
                continue

            if self.id_dict[idx].in_rescue:
                continue
            # If duckiebot is not in rescue, check classifier
            rescue_class = self.id_dict[idx].classifier(self.map)
            self.id_dict[idx].rescue_class = rescue_class

            # TODO: make more general not just for duckiebot 27
            # For debugging
            if idx == 27:
                print("[{}] with time_diff {}".format(idx, self.id_dict[idx].time_diff))
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


    def cbFSM(self, msg, veh_id):
        """Callback when the fsm state of a bot changed
        
        Args: 
            msg: ROS message (Float64Array)
            veh_id: ID of duckiebot
        """

        if veh_id in self.id_dict:
            self.log("Duckiebot [{}] FSM state <{}>".format(veh_id, msg.state))
            self.id_dict[veh_id].fsm_state = msg.state


if __name__ == '__main__':
    # create the node
    node = RescueCenterNode(node_name='rescue_trigger_node')
    # # run node
    # node.run()
    # keep spinning
    rospy.spin()