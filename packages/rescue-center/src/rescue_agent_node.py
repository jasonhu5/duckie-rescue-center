#!/usr/bin/env python
import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
from duckietown_msgs.msg import BoolStamped, Twist2DStamped, FSMState, Pose2DStamped
# from geometry_msgs.msg import TransformStamped, Transform
from visualization_msgs.msg import Marker, MarkerArray

class RescueAgentNode(DTROS):

    def __init__(self, node_name):

        # initialize DTROS parent class
        super(RescueAgentNode, self).__init__(node_name=node_name)
        self.veh_name = rospy.get_param("~distressed_veh") #e.g. autobot27
        self.veh_id = int(''.join([x for x in self.veh_name if x.isdigit()])) # e.g. 27
        self.activated = False
        self.distressType = 0 # int
        self.currentPose = Pose2DStamped() # x, y, theta
        self.current_car_cmd = Twist2DStamped() # v, omega
        self.current_car_cmd.v = 0
        self.current_car_cmd.omega = 0


        # this has to be specified, when launching the node (see below)
  #       <node pkg="rescue_center" type="rescue_agent_node.py" name="rescue_agent_XY" output="screen">
  #             <param name="~distressed_veh" type="string" value="autobot27" />
  #        </node>


        # Subscriber
        # 1. online localization
        self.sub_markers = rospy.Subscriber(
            "/cslam_markers", MarkerArray, self.cb_localization, queue_size=30)
        # 2. distress classification from rescue_noe
        self.sub_distress_classification = rospy.Subscriber("/{}/distress_classification".format(self.veh_name),
                String, self.cb_rescue)

        # Publisher
        # 1. everything ok to rescue_node
        self.pub_everything_ok = rospy.Publisher("{}/rescueDone/".format(self.veh_name), BoolStamped, queue_size=1)
        # 2. car_cmd to car_cmd_switch node
        self.pub_car_cmd = rospy.Publisher(
            "/{}/lane_recovery_node/car_cmd".format(self.veh_name),
            Twist2DStamped,
            queue_size=1,
        )

    # Callback for online localization
    def cb_localization(self, msg):
        '''Saves localization input into self.currentPose'''
        self.log("Received cslam message")
        markers = msg.markers
        for m in markers:
            if m.ns == "duckiebots":
                idx = str(m.id)
                if (idx == self.veh_id):
                    x = m.pose.position.x
                    y = m.pose.position.y
                    print("Deteced duckiebot {} at position ({}, {})".format(idx, x, y))
                    self.currentPose.x = x
                    self.currentPose.y = y
                    # TODO: calculate pose from quaternions and save in w
                    

    # Callback for rescue trigger
    def cb_rescue(self, msg):
        '''Activates rescue operation and stops duckiebot'''
        distress_type = int(msg)
        self.log("Received trigger. Distress Case: {}. Stopping {} now".format(distress_type, self.veh_name))
        if distress_type > 0:
            # this should always be the case, since rescue_node only publishes, if rescue_class >0
            self.activated = True
            self.distressType = distress_type
            # stop duckiebot
            self.current_car_cmd.v = 0
            self.current_car_cmd.omega = 0
            self.pub_car_cmd.publish(self.current_car_cmd)

    
    def calculate_car_cmd(self):
        '''Calculates car_cmd based on distress_type and current duckiebot pose'''
        # TODO: implement actual logic
        if self.distressType == 1:
            # out of lane
            self.current_car_cmd.v = 0
            self.current_car_cmd.omega = 0
        elif self.distressType == 2:
            # stuck
            self.current_car_cmd.v = 0
            self.current_car_cmd.omega = 0 
    

    def finishedRescue(self):
        '''Checks, if the rescue operation has been finished based on current duckiebot pose (similar to classificiation)'''
        # TODO: implemet actual logic 
        return False



    def run(self):

        # publish rate
        rate = rospy.Rate(4) # 10Hz

        while not rospy.is_shutdown():
            self.log("Rescue agent running...")
            if self.activated:
                self.log("{} in rescue operation")
                self.calculate_car_cmd()
                self.pub_car_cmd.publish(self.current_car_cmd)
                if self.finishedRescue():
                    self.activated = False
                    self.distressType = 0
                    msg = BoolStamped()
                    msg.data = True
                    self.pub_everything_ok.publish(msg)
            rate.sleep()


if __name__ == '__main__':
    # create the node
    node = RescueAgentNode(node_name='rescue_agent_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
