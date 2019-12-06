#!/usr/bin/env python
import os
import math
import rospy
import numpy as np
from enum import Enum
from visualization_msgs.msg import Marker, MarkerArray


class AutobotInfo():
    def __init__(self):
        self.timestamp = None
        self.fsm_state = None
        self.position = (None, None)
        self.filtered = (float('inf'), float('inf'))
        self.heading = None
        self.last_moved = None
        self.in_rescue = False

    def update_from_marker(self, m)
        self.timestamp = m.header.stamp
        self.position = (m.pose.position.x, m.pose.position.y)
        self.heading = self.quat2angle(m.pose.orientation)

    def update_filtered(self, timestamp, threshold, window):
        if self.distance(self.position, self.filtered) > threshold:
            self.filtered = self.position
            self.last_moved = timestamp
        else:
            self.filtered = self.avg_filter(self.position, self.filtered, 1/window)

    def classifier(self, time_diff, map):
        #TODO: implement logic
        ''' 
        1: out of lane
        2. stuck
          2.1 crashed into infrastructure
          2.2 crashed into other bot
          2.3 stuck for other reasons
        '''
        rescue_class = Distress.NORMAL_OPERATION
        # Check if the duckiebot is moving
        if self.timestamp - self.last_moved < time_diff:
            # Duckiebot is moving
        else:
            # Duckiebot is standing still
            # Try print(map.position_on_map(self.position)) for debugging
            


        '''trigger_rescue = rospy.get_param('~trigger_rescue')
        print(trigger_rescue)
        if trigger_rescue:
            print("Class change triggered")
            rescue_class = 1
        if y>0.5:
            # out of lane
            # TODO: hard code map
            rescue_class = 1
        elif self.id_dict[idx]["FSM_state"] =="KEEP_CALM":
            # duckiebot is stuck
            rescue_class = 2'''
        return rescue_class

    ### HELPER FUNCTIONS

    def quat2angle(self, q):
        siny_cosp = 2 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2 * (q.y*q.y + q.z*q.z)
        return math.atan2(siny_cosp, cosy_cosp)

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
    CRASHED_BOT = 2
    CRASHED_INFRA = 3
    KEEP_CALM = 4
