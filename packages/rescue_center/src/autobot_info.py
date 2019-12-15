#!/usr/bin/env python
import os
import math
import rospy
import numpy as np
from enum import Enum
from visualization_msgs.msg import Marker, MarkerArray
from rescue_center.msg import AutobotInfoMsg


TIME_DIFF_THRESHOLD = 10

class AutobotInfo():

    ''' --- INITIALIZE --- '''

    def __init__(self):
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
        # only changed, if classfied
        self.onRoad = map.position_on_map(self.position, subtile=True)
        # calculate time difference
        if self.last_movedSimple:
            self.time_diff = self.timestamp-max(self.last_moved, self.last_movedSimple)
        else:
            self.time_diff = self.timestamp-self.last_moved
        #print("Time_diff from classification: ", time_diff)

        # 0. debug mode: change through ros parameter
        debug_param = rospy.get_param('~trigger_rescue') # TODO: one for each autobot
        if debug_param:
            return Distress.DEBUG
        # TODO: uncomment out_of_road case
        # elif not self.onRoad:
        #     self.rescue_class =  Distress.OUT_OF_LANE
        # 2. stuck
        elif self.time_diff > TIME_DIFF_THRESHOLD: #TODO: change this parameter
            self.rescue_class =  Distress.STUCK
        else:
            self.rescue_class = Distress.NORMAL_OPERATION
        return self.rescue_class #TODO: get rid off return and call the attribute in rescue_center_node
    
    
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
    STUCK = 2
    DEBUG = 3

    # CRASHED_BOT = 2
    # CRASHED_INFRA = 3
    # KEEP_CALM = 4
