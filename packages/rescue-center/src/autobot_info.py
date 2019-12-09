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
        self.onRoad = True
        self.rescueClass = Distress.NORMAL_OPERATION

    def update_from_marker(self, m):
        self.timestamp = m.header.stamp
        self.position = (m.pose.position.x, m.pose.position.y)
        self.heading = self.quat2angle(m.pose.orientation)

    def update_filtered(self, timestamp, threshold, window):
        # timestamp of type genpy.rostime.Time
        if self.distance(self.position, self.filtered) > threshold:
            self.filtered = self.position
            self.last_moved = timestamp
            print("moved")
        # else:
        #     self.filtered = self.avg_filter(self.position, self.filtered, 1/window)

    def classifier(self, time_diff, map):
        # only changed, if classfied
        self.onRoad = map.position_on_map(self.position, subtile=True)
        # 0. debug mode: change through ros parameter
        debug_param = rospy.get_param('~trigger_rescue') # TODO: one for each autobot
        if debug_param:
            return Distress.DEBUG
        elif not self.onRoad:
            self.rescue_class =  Distress.OUT_OF_LANE
        # 2. stuck
        elif time_diff > 20: #TODO: change this parameter
            self.rescue_class =  Distress.STUCK
        else:
            self.rescue_class = Distress.NORMAL_OPERATION
        return self.rescue_class #TODO: could get rid off return and just call the attribute in rescue_node
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
    STUCK = 2
    DEBUG = 3

    # CRASHED_BOT = 2
    # CRASHED_INFRA = 3
    # KEEP_CALM = 4
