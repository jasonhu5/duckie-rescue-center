#!/usr/bin/env python
import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
from duckietown_msgs.msg import BoolStamped, Twist2DStamped, FSMState, Pose2DStamped
from visualization_msgs.msg import Marker, MarkerArray
from autobot_info import AutobotInfo, Distress
from rescue_center.msg import AutobotInfoMsg

# 11 

class RescueAgentNode(DTROS):

    def __init__(self, node_name):

        # initialize DTROS parent class
        super(RescueAgentNode, self).__init__(node_name=node_name)
        self.veh_name = rospy.get_param("~distressed_veh") #e.g. autobot27
        self.veh_id = int(''.join([x for x in self.veh_name if x.isdigit()])) # e.g. 27
        self.activated = False

        self.autobot_info = AutobotInfo()

        # self.distressType = 0 # int
        # self.currentPose = Pose2DStamped() # x, y, theta
        self.current_car_cmd = Twist2DStamped() # v, omega
        self.current_car_cmd.v = 0
        self.current_car_cmd.omega = 0


        # Subscriber
        # 1. online localization
        # self.sub_markers = rospy.Subscriber(
        #     "/cslam_markers", MarkerArray, self.cb_localization, queue_size=30)
        # 2. distress classification from rescue_center_node
        self.sub_distress_classification = rospy.Subscriber("/{}/distress_classification".format(self.veh_name),
                String, self.cb_rescue)

        # Publisher
        # 1. everything ok to rescue_center_node
        self.pub_everything_ok = rospy.Publisher("{}/rescueDone/".format(self.veh_name), BoolStamped, queue_size=1)
        # 2. car_cmd to car_cmd_switch node
        self.pub_car_cmd = rospy.Publisher(
            "/{}/lane_recovery_node/car_cmd".format(self.veh_name),
            Twist2DStamped,
            queue_size=1,
        )

        # for testing
        self.pub_tst = rospy.Publisher(
            "/rescue_agents/test",
            String,
            queue_size=1,
        )
        self.sub_autobot_info = rospy.Subscriber(
            "/{}/autobot_info".format(self.veh_name), AutobotInfoMsg, self.cb_autobot_info
        )

    def cb_autobot_info(self, msg):
        self.pub_tst.publish("Got info for {}".format(self.veh_name))
        # self.pub_info.publish()
        self.autobot_info.timestamp = msg.timestamp
        self.autobot_info.fsm_state = msg.fsm_state
        self.autobot_info.position = (msg.position[0], msg.position[1])
        self.autobot_info.filtered = (msg.filtered[0], msg.filtered[1])
        # msg.heading = 
        self.autobot_info.last_moved = msg.last_moved
        self.autobot_info.in_rescue = msg.in_rescue
        self.autobot_info.onRoad = msg.onRoad
        # msg.path =  
        # print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        # print(vars(info))
        # print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")

    # Callback for online localization
    # def cb_localization(self, msg):
    #     '''Saves localization input into self.currentPose'''
    #     # self.log("Received cslam message")
    #     if self.activated:
    #         markers = msg.markers
    #         for m in markers:
    #             if m.ns == "duckiebots":
    #                 idx = str(m.id)
    #                 if (idx == self.veh_id):
    #                     self.autobot_info.update_from_marker(m)
    #                     self.autobot_info.update_filtered(
    #                         m.header.stamp,
    #                         0.01,
    #                         10,
    #                     ) #TODO: with parameters
    #                     # TODO: calculate pose from quaternions and save in w
                    

    # Callback for rescue trigger
    def cb_rescue(self, msg):
        '''Activates rescue operation and stops duckiebot'''
        distress_type_num = int(msg.data)
        self.log("Received trigger. Distress Case: {}. Stopping {} now".format(distress_type_num, self.veh_name))
        if distress_type_num > 0:
            # this should always be the case, since rescue_center_node only publishes, if rescue_class >0
            self.activated = True
            self.autobot_info.rescue_class = Distress(distress_type_num)
            # stop duckiebot
            self.current_car_cmd.v = 0
            self.current_car_cmd.omega = 0
            self.pub_car_cmd.publish(self.current_car_cmd)

    
    def calculate_car_cmd(self):
        '''Calculates car_cmd based on distress_type and current duckiebot pose'''
        # TODO: implement actual logic
        if self.autobot_info.rescue_class == Distress.OUT_OF_LANE:
            # out of lane
            self.current_car_cmd.v = 0
            self.current_car_cmd.omega = 0
        elif self.autobot_info.rescue_class == Distress.STUCK:
            # stuck
            self.current_car_cmd.v = 0
            self.current_car_cmd.omega = 0
    

    def finishedRescue(self):
        '''Checks, if the rescue operation has been finished based on current duckiebot pose (similar to classificiation)'''
        # TODO: implement actual logic, this will be a different classifier
        debug_param = rospy.get_param('~everythingOK')
        return debug_param


    def run(self):

        # publish rate
        rate = rospy.Rate(4) # 10Hz

        while not rospy.is_shutdown():
            # self.log("Rescue agent running...")
            # self.pub_tst.publish("Hello from autobot{}".format(self.veh_id)) 

            if self.activated:
                self.log("In rescue operation")
                self.calculate_car_cmd()
                self.pub_car_cmd.publish(self.current_car_cmd)
                if self.finishedRescue():
                    self.log("Finished Rescue")
                    self.activated = False
                    self.autobot_info.rescue_class = Distress.NORMAL_OPERATION
                    msg = BoolStamped()
                    msg.data = True
                    self.pub_everything_ok.publish(msg)
                    rospy.set_param('~everythingOK', 'false')

            rate.sleep()


if __name__ == '__main__':
    # create the node
    node = RescueAgentNode(node_name='rescue_agent_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
