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
            - sub_rescueDone
            - pub_autobot_info
            - sub_rescue_stopped

    Subscriber:
        - sub_markers: /cslam_markers (:obj:`MarkerArray`): 
            Rviz markers from the online localization systems
        - sub_simpleLoc: /simple_loc_bots (:obj: `Float64MultiArray`): 
            Position and heading of the bots from the SimpleLoc system
        - sub_fsm_states[veh_id]: /autobot{}/fsm_mode (:obj:`FSMstate`)
            FSM state from every duckiebot
        - sub_rescueDone[veh_id]: /rescue/rescue_agents/autobot{}/rescueDone (:obj:`BoolStamped`)
            Used to get the signal from the rescue agent, that a rescue operation is done
        - sub_rescueStopped[veh_id]: /rescue/rescue_agents/autobot{}/rescueStopped (:obj:`BoolStamped`)
            Used to get the signal from the rescue agent, that a rescue operation has been stopped, without being finished
            (if monitoring is deactivated during rescue operation)

    Publisher:
        - pub_trigger[veh_id]: /autobot{}/recovery_mode (:obj:`BoolStamped`): 
            Triggers the recovery mode for a certain bot (FSM state change) and activates its rescue agent 
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
        # threshold distance to consider bot not moving ()
        self.parameters['~dist_thres'] = 0.01
        # number of windows to use for filtering, currently not used
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
        self.sub_rescueDone = dict()
        self.pub_autobot_info = dict()
        self.sub_rescue_stopped = dict()

        # build simpleMap
        # TODO: @Jason pull a duckietownworld fork in container
        map_file_path = os.path.join(
            "/code/catkin_ws/src",
            "duckie-rescue-center/packages/rescue_center/src",
            "test_map.yaml"
        )
        self.map = SimpleMap(map_file_path)
        self.map.display_raw_debug()

        print{"Rescue Center is initialized!"}


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
        self.sub_rescueDone[veh_id] = rospy.Subscriber(
            "/rescue/rescue_agents/autobot{}/rescueDone".format(veh_id),
            BoolStamped,
            self.cbRescueDone,
            callback_args=veh_id
        )
        # 5. Publisher: AutobotInfo Msg
        self.pub_autobot_info[veh_id] = rospy.Publisher(
            "/autobot{}/autobot_info".format(veh_id),
            AutobotInfoMsg,
            queue_size=5,
        )

        # 6. Subscriber: Rescue stopped (not finished)
        self.sub_rescue_stopped[veh_id] = rospy.Subscriber(
            "/rescue/rescue_agents/autobot{}/rescueStopped/".format(veh_id), BoolStamped, self.cbRescueStopped,
            callback_args=veh_id)

    def cbRescueStopped(self, msg, veh_id):
        """Callback of RescueStopped:
            - gets triggered, if rescue_agent has stopped rescue, but rescue is not finished yet. 
            (happens, if autobot is stopped being monitored during a rescue operation)
    
        Args: 
            msg: ROS message (BoolStamped)
            veh_id: autobot ID
        """
        if msg.data == True:
            # Note: this should always be the case
            print("[{}] Received rescue stopped! rescue Center will take back from rescue agent ".format(veh_id))
            self.id_dict[veh_id].in_rescue = False
            self.id_dict[veh_id].last_moved = self.id_dict[veh_id].timestamp
            
    def cbFSM(self, msg, veh_id):
            """Callback when the fsm state of a bot changed
            
            Args: 
                msg: ROS message (Float64Array)
                veh_id: ID of duckiebot
            """

            if veh_id in self.id_dict:
                self.log("Duckiebot [{}] FSM state changed from <{}> to <{}>".format(veh_id, 
                        self.id_dict[veh_id].fsm_state, msg.state))
                self.id_dict[veh_id].fsm_state = msg.state

    def cbSimpleLoc(self, msg):
        """Callback of simpleLoc: 
                - stores position and heading from simple Localization into the corresponding AutobotInfo() dictionary
                (Note: simpleLoc publishes for one duckiebot at a time)
                - determines msg timestamp as the time, where the duckiebot has moved last 
                (Note: simpleLoc only publishes, if duckiebot moved)
                - publishes autobotInfo to corresponding duckiebot
        
        Args: 
            msg: ROS message (Float64MultiArray)
        """

        veh_id = int(msg.data[0])
        if veh_id in self.id_dict:
            # make sure duckiebot has already been spotted by global localization 
            # (need it, in case simpleLoc msg comes in before the first /cslam msg)
            self.id_dict[veh_id].positionSimple = (msg.data[1], msg.data[2])
            self.id_dict[veh_id].headingSimple = msg.data[3]
        self.id_dict[veh_id].last_movedSimple = msg.data[4] # in s
        self.pub_autobot_info[veh_id].publish(self.id_dict[veh_id].autobotInfo2Msg())
    
    def cbRescueDone(self, msg, veh_id):
        """Callback of RescueAgent, which signals that the rescue operation is over
         - rescue center will take over from rescue_agent
         - trigger state change from "Recovery" to "Lane Following" 
        
        Args: 
            msg: ROS message (Float64Array)
            veh_id: ID of duckiebot
        """

        if msg.data == True:
            self.id_dict[veh_id].in_rescue = False
            self.id_dict[veh_id].last_moved = self.id_dict[veh_id].timestamp
            msg = BoolStamped()
            msg.data = False
            self.pub_trigger[veh_id].publish(msg) # turns on lane following


    def cbLocalization(self, msg):
        """ Callback when receiving online localization results:
            Checks if there are any new bots on the map
            saves position/heading/lastMoved into corresponding autobotInfo
            calls the Classifier for each bot that is currently not in rescue operation and for which rescue is activated

        Args: 
            msg: ROS visualization message (MarkerArray)
        """

        for m in msg.markers:
            
            # care only about duckiebots
            if m.ns != "duckiebots":
                continue

            idx = m.id
            # Append new bots to veh_list if they show up the first time and launch a rescue_agent
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
                self.parameters['~dist_thres']
            )
            # publish autobot_info to rescue_agent
            self.pub_autobot_info[idx].publish(self.id_dict[idx].autobotInfo2Msg())

            # Check, if duckiebot should be classified:
            change_classified_duckiebots = rospy.get_param('~change_classified_duckiebots') 
            if change_classified_duckiebots:
                print("[Classification]: Please specify, which duckiebot to add or remove")
                add_duckiebot = rospy.get_param('~add_duckiebot')
                remove_duckiebot = rospy.get_param('~remove_duckiebot')
                if add_duckiebot in self.id_dict and self.id_dict[add_duckiebot].classificationActivated == False:
                    self.id_dict[add_duckiebot].classificationActivated = True
                    # to prevent time_diff being triggered
                    self.id_dict[add_duckiebot].last_moved = m.header.stamp.to_sec()
                    self.id_dict[add_duckiebot].last_movedSimple = m.header.stamp.to_sec()
                    print("[{}] is now being classified for rescue.".format(add_duckiebot))
                    rospy.set_param('~add_duckiebot', -1)
                    rospy.set_param('~change_classified_duckiebots', False)
                if remove_duckiebot in self.id_dict:
                    self.id_dict[remove_duckiebot].classificationActivated = False
                    print("[{}] is not being classified for rescue.".format(remove_duckiebot))
                    rospy.set_param('~remove_duckiebot', -1) 
                    rospy.set_param('~change_classified_duckiebots', False)

            # continue, if we don't want to classify that duckiebot
            if self.id_dict[idx].classificationActivated == False:
                continue

            # continue, if duckiebot is in rescue 
            if self.id_dict[idx].in_rescue:
                continue

            # Classify duckiebot
            rescue_class = self.id_dict[idx].classifier(self.map)
            self.id_dict[idx].rescue_class = rescue_class

            print("[{}] not moved for: {}, onRoad: {}, delta_phi: {}, tile_type: {}".format(idx,
                 self.id_dict[idx].time_diff, self.id_dict[idx].onRoad, self.id_dict[idx].delta_phi, self.id_dict[idx].tile_type))

            if rescue_class.value != 0:
                # publish rescue_class
                print("[{}] in distress <{}>".format(idx, rescue_class))
                msg = BoolStamped()
                msg.data = True
                # turn duckiebot into rescue mode
                self.pub_trigger[idx].publish(msg)
                # notify rescue agent of the classification result
                self.pub_rescue_classfication[idx].publish(str(rescue_class.value))
                # note down so can skip the next time
                self.id_dict[idx].in_rescue = True


if __name__ == '__main__':
    # create the node
    node = RescueCenterNode(node_name='rescue_trigger_node')
    # # run node
    # node.run()
    # keep spinning
    rospy.spin()