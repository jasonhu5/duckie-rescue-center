#!/usr/bin/env python
import os
import math
import rospy
import numpy as np
from enum import Enum
from visualization_msgs.msg import Marker, MarkerArray
from rescue_center.msg import AutobotInfoMsg


TIME_DIFF_THRESHOLD = 10
ANGLE_TRHESHOLD = 70

class AutobotInfo():
    """ 
    This is a class for saving all relevant information of a autobot. 
    It further provides a function for classification.
      
    Attributes: 
        - timestamp (float): When did it get updated the last time (in sec)
        - fsm_state (str): current FSM state of the duckiebot: e.g. "LANE_FOLLOWING"
        - position ((float, float)): position measurement from cslam localization
        - positionSimple ((float, float)): position measurement from simple localization
        - current_pos ((float, float)): position measurement, which is taken for further calculations/classification
        - filtered ((float, float)): filteredPosition for checking last movement of duckiebot
        - heading (float): heading measurement from cslam localization in DEG
        - headingSimple (float): heading measurement from simple localization
        - currentHeading (float): heading measurement, which is taken for further calculations/classification
        - last_moved (float): time of last detected movement from cslam localization
        - last_movedSimple (float): time of last detected movement from simple localization
        - time_diff (float): time passed from last movement (take the last detected movement from cslam or simple localization)
        - in_rescue (boolean): is autobot in rescue mode?
        - onRoad (boolean): is autobot on road?
        - rescueClass (:obj: Distress): how is the autobot distressed? e.g. Distress.OUT_OF_LANE
        - veh_id (int): autobot ID
        - classificationActivated (bool): is classification for that autobot activated? 
            True: autobot is classified and rescued
            False: only monitored
        - delta_phi (float): error in current heading and desired heading
        - tile_type (str): on what kind of tile is the duckiebot? e.g. "asphalt", "curve", etc.
        
    """

    def __init__(self, veh_id):
        """Initilialize AutobotInfo object to default values:

            Args: 
                - veh_id (int): id of duckiebot

        """
        self.timestamp = None
        self.fsm_state = None
        self.position = (None, None)
        self.positionSimple = (None, None)
        self.current_pos = (None, None)
        self.filtered = (float('inf'), float('inf'))
        self.heading = None
        self.headingSimple = None
        self.current_heading = None

        self.last_moved = None
        self.last_movedSimple = - float('inf')
        self.time_diff = None
        self.in_rescue = False
        self.onRoad = True
        self.rescue_class = Distress.NORMAL_OPERATION

        self.veh_id = veh_id
        self.classificationActivated  = False
        self.delta_phi = None
        self.tile_type = None
        
    ''' --- UPDATE FUNCTION --- '''

    def updatePositionAndHeading(self):
        """ Updates current position and heading, depending on simple localization is on or off

            Args: None

            Output: None

        """
        if (self.positionSimple[0] == 0 and self.positionSimple[1] == 0) or self.positionSimple[0] == None:
            # simple localization has not been triggered yet (duckiebot did not move) --> take normal localization output
            self.current_pos = self.position  # (x, y)
            self.current_heading = self.heading  # degree
        else:
            # use simple localization
            self.current_pos = self.positionSimple  # (x, y)
            self.current_heading = self.headingSimple  # degree

    def update_from_marker(self, m):
        """Method to update the AutobotInfo object from its ROS message

        Note:
            Careful with uninitialized variables, ROS message cannot store None
            It will use 0.0f when None which might cause confusion

        Args:
            m: AutoBotInfo ROS message

        Output: None
    
        """
        self.timestamp = m.header.stamp.to_sec()
        self.position = (m.pose.position.x, m.pose.position.y)
        self.heading = self.quat2angle(m.pose.orientation)
        self.updatePositionAndHeading()

    def update_filtered(self, threshold):
        """Updates position value and timestamp only when movement detectd

        Args:
            threshold: movement in [cm] before bot is seen as "has moved"

        Output: None
        """
        # timestamp of type genpy.rostime.Time
        if self.distance(self.position, self.filtered) > threshold:
            self.filtered = self.position
            self.last_moved = self.timestamp

    ''' --- ASSEMBLE ROS MESSAGE --- '''

    def autobotInfo2Msg(self):
        """Wraps instance of class into a ROS message

        Returns: 
            A ROS message as defined in rescue_center.msgs

        """
        # TODO: Make nicer and add reverse function
        msg = AutobotInfoMsg()
        if self.timestamp:
            msg.timestamp = self.timestamp
        if self.fsm_state:
            msg.fsm_state = self.fsm_state
        # if self.position[0]:
        #     msg.position = [self.position[0], self.position[1]]
        if self.filtered:
            msg.filtered = [self.filtered[0], self.filtered[1]]
        if self.last_moved:
            msg.last_moved = self.last_moved
        if self.last_movedSimple:
            msg.last_movedSimple = self.last_movedSimple
        if self.in_rescue:
            msg.in_rescue = self.in_rescue
        if self.onRoad:
            msg.onRoad = self.onRoad
        if self.rescue_class:
            msg.rescue_class = self.rescue_class.value 
        if self.current_heading:
            msg.current_heading = self.current_heading
        if self.current_pos[0]:
            msg.current_pos = [self.current_pos[0], self.current_pos[1]]
        
        msg.classificationActivated = self.classificationActivated
        return msg    

    ''' --- DISTRESS CLASSIFIER --- '''


    def classifier(self, map):
        """The Classifier decides whether a bot is in a distress situation
        and tells which kind of distress situation it is 

        Note:
            The classification is used as input for the Rescue Agent to plan 
            a suitable rescue strategy

        Args:
            map: An instance of the map

        Returns:
            An instance of the Distress Class, normally NORMAL_OPERATION

        """
        # update position and heading:
        # self.updatePositionAndHeading()       
        self.tile_type = map.pos_to_semantic(self.current_pos)

        # 0. debug mode: change through ros parameter
        debug_param = rospy.get_param('~trigger_rescue') # TODO: one for each autobot
        if debug_param:
            return Distress.DEBUG
        # 1. Out of lane 
        if not self.positionOnRoad(map):
            return Distress.OUT_OF_LANE
        # 2. Wrong heading: only for lanes right now
        if self.wrongHeading(map):
            return Distress.WRONG_HEADING
        # 3. stuck 
        if self.last_movedSimple:
            self.time_diff = self.timestamp-max(self.last_moved, self.last_movedSimple)
        else:
            self.time_diff = self.timestamp-self.last_moved

        if self.time_diff > TIME_DIFF_THRESHOLD: 
            print("[{}] stuck, FSM State: {}".format(self.veh_id, type(self.fsm_state)))
            # stuck at intersection 
            if self.fsm_state == "INTERSECTION_CONTROL" or self.fsm_state == "INTERSECTION_COORDINATION":
                # duckiebot at intersection
                print("[{}] at intersection.".format(self.veh_id))
                if self.time_diff > TIME_DIFF_THRESHOLD * 3:                      # TODO: parametrize
                    return Distress.STUCK_AT_INTERSECTION
                else: 
                    return Distress.NORMAL_OPERATION
            # stuck in intersection
            if self.tile_type == '4way' or self.tile_type == '3way':
                return  Distress.STUCK_IN_INTERSECTION
            # stuck general
            return Distress.STUCK_GENERAL
        # 4 everything ok
        self.rescue_class = Distress.NORMAL_OPERATION
        return Distress.NORMAL_OPERATION    
    
    ''' --- HELPER FUNCTIONS --- '''

    def positionOnRoad(self, map):
        self.onRoad = map.position_on_map(self.current_pos, subtile=True)
        return self.onRoad
    
    def wrongHeading(self, map):
        desired_heading = map.pos_to_ideal_heading(self.current_pos)
        if desired_heading == None:
            return False
        delta_phi = abs(self.current_heading-desired_heading)
        self.delta_phi = min(delta_phi, 360-delta_phi)      
        if  self.delta_phi > ANGLE_TRHESHOLD and self.tile_type == 'straight':
            return True
        return False

    def quat2angle(self, q):
        siny_cosp = 2 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2 * (q.y*q.y + q.z*q.z)
        return round((180.0/math.pi * math.atan2(siny_cosp, cosy_cosp)))

    def distance(self, positionA, positionB):
        xA, yA = positionA
        xB, yB = positionB
        return math.sqrt((xA-xB)**2 + (yA-yB)**2)

    def avg_filter(self, positionNew, positionAvg, a):
        xNew, yNew = positionNew
        xAvg, yAvg = positionAvg
        return (a*xNew + (1-a)*xAvg, a*yNew + (1-a)*yAvg)


class Distress(Enum):
    NORMAL_OPERATION = 0
    OUT_OF_LANE = 1
    WRONG_HEADING = 2
    STUCK_AT_INTERSECTION = 3
    STUCK_IN_INTERSECTION = 4
    STUCK_GENERAL = 5
    DEBUG = 6
