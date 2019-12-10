#!/usr/bin/env python
import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
from duckietown_msgs.msg import BoolStamped, Twist2DStamped, FSMState, Pose2DStamped
from visualization_msgs.msg import Marker, MarkerArray
from autobot_info import AutobotInfo, Distress
from rescue_center.msg import AutobotInfoMsg
from simple_map import SimpleMap
import math

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
        self.current_car_cmd = Twist2DStamped() # v, omega
        self.current_car_cmd.v = 0
        self.current_car_cmd.omega = 0
        self.car_cmd_array = list()

        # build map
        map_file_path = os.path.join(
            "/code/catkin_ws/src",
            "duckie-rescue-center/packages/rescue_center/src",
            "test_map.yaml"
        )
        self.map = SimpleMap(map_file_path)

        self.finished_execution = False


        # Subscriber
        # 1. online localization
        # self.sub_markers = rospy.Subscriber(
        #     "/cslam_markers", MarkerArray, self.cb_localization, queue_size=30)
        # 2. distress classification from rescue_center_node
        self.sub_distress_classification = rospy.Subscriber("/{}/distress_classification".format(self.veh_name),
                String, self.cb_rescue)
        # 3. Autobot info from rescue_center_node
        self.sub_autobot_info = rospy.Subscriber(
            "/{}/autobot_info".format(self.veh_name), AutobotInfoMsg, self.cb_autobot_info
        )

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
            self.stopDuckiebot()
            # calculate_car_cmd
            self.calculate_car_cmd()

    
    def calculate_car_cmd(self):
        '''Calculates car_cmd based on distress_type and current duckiebot pose'''
        if self.autobot_info.rescue_class == Distress.OUT_OF_LANE:
            # TODO: implement actual logic
            self.car_cmd_array = list()
        elif self.autobot_info.rescue_class == Distress.STUCK:
            # stuck
            if self.stuckedRight():
                self.moveBack_cmDistance(10, smoothCmd=True) #add cmds to array
                # for i in range(3):
                #     cmd = Twist2DStamped()
                #     cmd.v = -0.5
                #     cmd.omega = 0
                #     self.car_cmd_array.append(cmd) 
                # for i in range(3):
                #     cmd = Twist2DStamped()
                #     cmd.v = 0
                #     cmd.omega = 0.8
                #     self.car_cmd_array.append(cmd) 
            #TODO: stuckedLeft()
        # print(self.car_cmd_array)

    def moveBack_cmDistance(self, distance_inCM, smoothCmd = False, debug = False):
        '''adds cmds to self.car_cmd_array to move back quarterTile = 58.5cm/4 ~ 14 cm'''
        cmd_move = Twist2DStamped()
        if debug:
            cmd_move.v = rospy.get_param('~velocity_backwards')
        else:
            cmd_move.v = -0.1
        cmd_move.omega = 0

        if not smoothCmd:
            cmd_pause = Twist2DStamped()
            cmd_pause.v = 0.0
            cmd_pause.omega = 0.0        
            #this moves 2cm
            cmd_package = list()
            cmd_package.append(cmd_move)
            for i in range(5):
                # TODO: make code nicer
                cmd_package.append(cmd_pause)
        else:
            # TODO: for smooth cmd: 20 percent bigger distance than withou
            cmd_package = [cmd_move]
        
        if distance_inCM < 2:
            self.log("[Error in moveBack_cmDistance]: autobot must move more than 2cm")
        else:
            num_packages = int(math.floor(distance_inCM/2))
        for i in range(num_packages):
            self.car_cmd_array = self.car_cmd_array + cmd_package
    
    def stuckedRight(self):
       '''checks, if duckiebot is stuck left or right'''
       # TODO: implement classifier her
       return True
      

    def finishedRescue(self):
        '''Checks, if the rescue operation has been finished based on current duckiebot pose (similar to classificiation)'''
        # TODO: implement actual logic, this will be a different classifier
        debug_param = rospy.get_param('~everythingOK')
        if debug_param:
            return True
        # if self.goodHeading():
        #     return True
        return False

    def goodHeading(self):
        # TODO: implement logic: hard coded now
        if abs(abs(self.autobot_info.heading) - 180) < 20:
            return True

    def stopDuckiebot(self):
        car_cmd = Twist2DStamped()
        car_cmd.v = 0
        car_cmd.omega = 0
        self.current_car_cmd = car_cmd
        self.pub_car_cmd.publish(self.current_car_cmd)


    def run(self):

        # publish rate
        rate = rospy.Rate(4) # 10Hz

        while not rospy.is_shutdown():
            # self.log("Rescue agent running...")
            # self.pub_tst.publish("Hello from autobot{}".format(self.veh_id)) 

            if self.activated:
                # self.calculate_car_cmd()
                if self.car_cmd_array:
                    self.current_car_cmd = self.car_cmd_array.pop(0)
                    self.pub_car_cmd.publish(self.current_car_cmd)
                    # self.finished_execution = False
                    self.log("Applying {}".format(self.current_car_cmd.v))
                    # if not self.car_cmd_array:
                    #     # just popped out last one
                    #     self.finished_execution = True
                else:
                    self.log("Finished applying commands")
                    self.stopDuckiebot()
                    # self.calculate_car_cmd()

                if self.finishedRescue():
                    self.log("Finished Rescue")
                    self.activated = False
                    self.autobot_info.rescue_class = Distress.NORMAL_OPERATION
                    msg = BoolStamped()
                    msg.data = True
                    self.pub_everything_ok.publish(msg)
                    # rospy.set_param('~everythingOK', 'false')

            rate.sleep()


if __name__ == '__main__':
    # create the node
    node = RescueAgentNode(node_name='rescue_agent_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
