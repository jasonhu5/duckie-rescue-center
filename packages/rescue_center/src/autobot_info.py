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

    ''' --- INITIALIZE --- '''

    def __init__(self, veh_id):
        """Initilialize AutobotInfo object to default values
        no arguments required

        """
        self.timestamp = None
        self.fsm_state = None
        self.position = (None, None)
        self.positionSimple = (None, None)
        self.filtered = (float('inf'), float('inf'))
        self.heading = None
        self.headingSimple = None
        self.last_moved = None
        self.last_movedSimple = None
        self.time_diff = None
        self.in_rescue = False
        self.onRoad = True
        self.rescue_class = Distress.NORMAL_OPERATION

        self.veh_id = veh_id
        
    ''' --- UPDATE FUNCTION --- '''

    def update_from_marker(self, m):
        """Method to update the AutobotInfo object from its ROS message

        Note:
            Careful with uninitialized variables, ROS message cannot store None
            It will use 0.0f when None which might cause confusion

        Args:
            m: AutoBotInfo ROS message
    
        """
        self.timestamp = m.header.stamp.to_sec()
        self.position = (m.pose.position.x, m.pose.position.y)
        self.heading = self.quat2angle(m.pose.orientation)

    def update_filtered(self, threshold, window):
        """Updates position value and timestamp only when movement detectd

        Args:
            threshold: movement in [cm] before bot is seen as "has moved"

        """
        # timestamp of type genpy.rostime.Time
        if self.distance(self.position, self.filtered) > threshold:
            self.filtered = self.position
            self.last_moved = self.timestamp
            print("moved")
        # Filter currently not used, probably unnecessary
        # else:
        #     self.filtered = self.avg_filter(self.position, self.filtered, 1/window)


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
        if self.position[0]:
            msg.position = [self.position[0], self.position[1]]
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
        if self.heading:
            msg.heading = self.heading
        # for simpleLoc:
        if self.positionSimple[0]:
            msg.positionSimple = [self.positionSimple[0], self.positionSimple[1]]
        if self.headingSimple:
            msg.headingSimple = self.headingSimple
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
        if self.positionSimple[0] is None :
            # simple localization has not been triggered yet (duckiebot did not move) --> take normal localization output
            current_pos = self.position  # (x, y)
            current_heading = self.heading  # degree
            # print("Position from online loc", current_pos)
        else:
            # use simple localization
            current_pos = self.positionSimple  # (x, y)
            current_heading = self.headingSimple  # degree
            # print("Position from simple loc", current_pos)
        # calculate onRoad
        self.onRoad = map.position_on_map(current_pos, subtile=True)
        # calculate time difference:
        if self.last_movedSimple:
            self.time_diff = self.timestamp-max(self.last_moved, self.last_movedSimple)
        else:
            self.time_diff = self.timestamp-self.last_moved
        # calculate desired heading:
        desired_heading = map.pos_to_ideal_heading(current_pos)

        # 0. debug mode: change through ros parameter
        debug_param = rospy.get_param('~trigger_rescue') # TODO: one for each autobot
        if debug_param:
            return Distress.DEBUG
        # 1. Out of lane 
        if not self.onRoad:
            self.rescue_class =  Distress.OUT_OF_LANE
            return self.rescue_class
        # 2. Wrong heading
        delta_phi = abs(current_heading-desired_heading)
        delta_phi = min(delta_phi, 360-delta_phi)      
        if delta_phi > ANGLE_TRHESHOLD:
            self.rescue_class = Distress.WRONG_HEADING
            return self.rescue_class
        # 3. stuck 
        if self.time_diff > TIME_DIFF_THRESHOLD: 
            print("[{}] stuck, FSM State: {}".format(self.veh_id, type(self.fsm_state)))
            # stuck at intersection 
            if self.fsm_state == "INTERSECTION_CONTROL" or self.fsm_state == "INTERSECTION_COORDINATION":
                # duckiebot at intersection
                if self.time_diff > TIME_DIFF_THRESHOLD * 3:                      # TODO: parametrize
                    self.rescue_class = Distress.STUCK_AT_INTERSECTION
                    return self.rescue_class
            tile_type = map.pos_to_semantic(current_pos)
            # stuck in intersection
            if tile_type ==  "intersection":
                self.rescue_class = Distress.STUCK_IN_INTERSECTION
                return self.rescue_class
            # stuck general
            self.rescue_class = Distress.STUCK_GENERAL
            return self.rescue_class
        # 4 everything ok
        self.rescue_class = Distress.NORMAL_OPERATION
        return self.rescue_class
        
         #TODO: get rid off returns and call the attribute in rescue_center_node
    
    
    ''' --- HELPER FUNCTIONS --- '''

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

    # CRASHED_BOT = 2
    # CRASHED_INFRA = 3
    # KEEP_CALM = 4
