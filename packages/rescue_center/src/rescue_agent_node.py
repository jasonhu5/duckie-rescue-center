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
from stuck_controller import StuckController
from time import sleep
import math
import numpy as np

# Parameters for Open Loop Control
OMEGA_CMD_PER_TURN = 8
DEGREE_PER_TURN = 24
V_CMD_PER_STEP = -0.1
CM_PER_STEP = 2
EXTRA_DISTANCE_IN_M = 0.05
LESS_ANGLE_IN_DEGREE = 20


class RescueAgentNode(DTROS):

    def __init__(self, node_name):

        # initialize DTROS parent class
        super(RescueAgentNode, self).__init__(node_name=node_name)
        self.veh_name = rospy.get_param("~distressed_veh") #e.g. autobot27
        self.veh_id = int(''.join([x for x in self.veh_name if x.isdigit()])) # e.g. 27
        self.activated = False #triggers rescue
        self.autobot_info = AutobotInfo()

        # for open loop control
        self.current_car_cmd = Twist2DStamped() # v, omega
        self.current_car_cmd.v = 0
        self.current_car_cmd.omega = 0
        self.car_cmd_array = list()
        self.finished_execution = False


        # simpleMap for localization
        map_file_path = os.path.join(
            "/code/catkin_ws/src",
            "duckie-rescue-center/packages/rescue_center/src",
            "test_map.yaml"
        )
        self.map = SimpleMap(map_file_path)

        # For closed loop control
        self.controller_counter = 0


        # Subscriber
        # 1. distress classification from rescue_center_node
        self.sub_distress_classification = rospy.Subscriber("/{}/distress_classification".format(self.veh_name),
                String, self.cb_rescue)
        # 2. Autobot info from rescue_center_node
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
        '''callback function for receiving AutobotInfoMsg from rescue_center: 
                saves msg into self.autobot_info'''
        self.pub_tst.publish("Got info for {}".format(self.veh_name))
        self.autobot_info.timestamp = msg.timestamp
        self.autobot_info.fsm_state = msg.fsm_state
        self.autobot_info.position = (msg.position[0], msg.position[1])
        self.autobot_info.filtered = (msg.filtered[0], msg.filtered[1])
        self.autobot_info.last_moved = msg.last_moved
        self.autobot_info.in_rescue = msg.in_rescue 
        self.autobot_info.onRoad = msg.onRoad
        self.autobot_info.heading = msg.heading
        self.autobot_info.headingSimple = msg.headingSimple
        self.autobot_info.positionSimple = (msg.positionSimple[0], msg.positionSimple[1])
        # print("Heading (Rescue Agent): {} ,{}".format(msg.headingSimple, self.autobot_info.headingSimple))
        # print("Position (Rescue Agent): {}, {}".format(msg.positionSimple, self.autobot_info.positionSimple))

    def cb_rescue(self, msg):
        '''Callback function for receiving rescue trigger from rescue-center:
                -activates rescue operation and stops duckiebot
                - for open loop control: calculates car_cmd
            '''
        distress_type_num = int(msg.data)
        self.log("Received trigger. Distress Case: {}. Stopping {} now".format(distress_type_num, self.veh_name))
        if distress_type_num > 0:
            # @Note: this should always be the case, since rescue_center_node only publishes, if rescue_class >0
            self.activated = True
            self.autobot_info.rescue_class = Distress(distress_type_num)
            self.stopDuckiebot()
            # for open loop control:
            # sleep(3)
            # calculate_car_cmd
            # self.calculate_car_cmd()
    
    def calculate_car_cmd(self):
        '''For open loop control: Calculates car_cmd based on distress_type and current duckiebot pose'''
        current_pos = self.autobot_info.position # (x, y)
        current_heading = self.autobot_info.heading # degree
        self.log("Distressed at: {} with heading: {}".format(current_pos, current_heading))
        ideal_pos = self.map.pos_to_ideal_position(current_pos)
        desired_heading = self.map.pos_to_ideal_heading(current_pos)
        print("Desired position: {}, Desired Heading: {}".format(ideal_pos, desired_heading))

        if self.autobot_info.rescue_class == Distress.OUT_OF_LANE:
            # TODO: implement actual logic
            self.car_cmd_array = list()
        elif self.autobot_info.rescue_class == Distress.STUCK:
            # stuck
            if ideal_pos[0] == current_pos[0]:
                #vertical straight tile
                desired_pos_y = ideal_pos[1] + np.sign(ideal_pos[1]-current_pos[1])*EXTRA_DISTANCE_IN_M
                # desired_pos_x = ideal_pos[0]
                # desired_pos = (desired_pos_x, desired_pos_y)
                if current_heading > 90 and current_heading < 180:
                    distance_backwards_cm = 100*abs(desired_pos_y-current_pos[1])/math.sin(math.pi/180*(180-current_heading))
                    print("Distance to move back: {}".format(distance_backwards_cm))
                    angle_to_turn = desired_heading - current_heading - np.sign(desired_heading - current_heading)*LESS_ANGLE_IN_DEGREE
                    print("Angle to turn: {}".format(angle_to_turn))
            
                    self.moveBack_cmDistance(distance_backwards_cm, smoothCmd=False) #add cmds to array
                    self.turn_angle(angle_to_turn, smoothCmd=False)
            # @Note: Those are good hard coded values for a test case:
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

    def moveBack_cmDistance(self, distance_inCM, smoothCmd = False, debug = False):
        '''adds cmds to self.car_cmd_array to move back specified distance: discretized in CM_perStep (2)
        '''
        cmd_move = Twist2DStamped()
        if debug:
            cmd_move.v = rospy.get_param('~velocity_backwards')
        else:
            cmd_move.v = V_CMD_PER_STEP
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
        
        if distance_inCM < CM_PER_STEP:
            self.log("[Error in moveBack_cmDistance]: autobot must move more than {}cm".format(CM_PER_STEP))
        else:
            num_packages = int(math.floor(distance_inCM/CM_PER_STEP))
            for i in range(num_packages):
                self.car_cmd_array = self.car_cmd_array + cmd_package
    
    def turn_angle(self, angle_inDeg, smoothCmd = False):
        '''adds cmds to self.car_cmd_array to turn specified angle: discretized: DEGREE_PER_TURN = 24'''
        # TODO: make code look nicer
        cmd_turn = Twist2DStamped()
        cmd_turn.v = 0
        cmd_turn.omega = np.sign(angle_inDeg)*OMEGA_CMD_PER_TURN
        cmd_turn_array = list()
        for i in range(1):
            cmd_turn_array.append(cmd_turn)

        if not smoothCmd:
            cmd_pause = Twist2DStamped()
            cmd_pause.v = 0.0
            cmd_pause.omega = 0.0        
            #this moves 24 deg
            cmd_package = list()
            for i in range(5):
                # TODO: make code nicer
                cmd_package.append(cmd_pause)
            cmd_package = cmd_turn_array + cmd_package
        else:
            # TODO: overshoots, decrease gain
            cmd_package = list(cmd_turn_array)
        # cmd_package needs 4-5 turns to get 45 degrees
        # self.car_cmd_array = self.car_cmd_array + cmd_package
        if abs(angle_inDeg) < DEGREE_PER_TURN:
            self.log("[Error in moveBack_cmDistance]: autobot must turn more than {} degrees".format(DEGREE_PER_TURN))
        else:
            num_packages = int(math.floor(abs(angle_inDeg)/DEGREE_PER_TURN))
            for i in range(num_packages):
                self.car_cmd_array = self.car_cmd_array + cmd_package

    def rescue_operation_CL(self):
        tol_pos = 0.05
        tol_heading = 10
        dt = 0.2

        C = StuckController(k_P=5, k_I=2, c1=5, c2=0.01)

        current_pos = self.autobot_info.position # (x, y)
        current_heading = self.autobot_info.heading # degree
        # TODO: could incorporate an extra margin
        desired_pos = self.map.pos_to_ideal_position(current_pos)
        desired_heading = self.map.pos_to_ideal_heading(current_pos)

        while(abs(current_pos-desired_pos) > tol_pos or abs(current_heading-desired_heading) > tol_heading):
            # Calculate controller output
            current_p = current_pos[1] if desired_pos[0] == current_pos[0] else current_pos[0]
            desired_p = desired_pos[1] if desired_pos[0] == current_pos[0] else desired_pos[0]
            current_heading += 180 if current_heading <= 0 else -180
            desired_heading += 180 if desired_heading <= 0 else -180
            v_out, omega_out = C.getControlOutput(current_p, current_heading, desired_p, desired_heading, v_ref=0.4, dt_last=dt)
            # Send cmd to duckiebot
            msg = Twist2DStamped()
            msg.v = v_out
            msg.omega = omega_out
            self.pub_car_cmd.publish(msg)
            # Sleep
            sleep(dt)
            # Update measurements
            current_pos = self.autobot_info.position # (x, y)
            current_heading = self.autobot_info.heading # degree
            desired_pos = self.map.pos_to_ideal_position(current_pos)
            desired_heading = self.map.pos_to_ideal_heading(current_pos)
        
        # Stop duckiebot
        msg = Twist2DStamped()
        msg.v = 0
        msg.omega = 0
        self.pub_car_cmd.publish(msg)


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
        k_P = float(os.environ['K_P'])
        k_I = float(os.environ['K_I'])
        c1 = float(os.environ['C1'])
        c2 = float(os.environ['C2'])
        C = StuckController(k_P=k_P, k_I=k_I, c1=c1, c2=c2)

        # C = StuckController(k_P=5, k_I=2, c1=5, c2=0.01)
        # C = StuckController(k_P=10, k_I=0, c1=-5, c2=1)

        tol_pos = 0.02
        tol_heading = 5
        dt = 0.2

        while not rospy.is_shutdown():
            if self.activated:
                if self.autobot_info.positionSimple[0] == 0 and self.autobot_info.positionSimple[0] == 0:
                    current_pos = self.autobot_info.position # (x, y)
                    current_heading = self.autobot_info.heading # degree
                else:
                    current_pos = self.autobot_info.positionSimple # (x, y)
                    current_heading = self.autobot_info.headingSimple # degree
                print("Current: [{}]: pos = {}, phi = {}".format(self.veh_id, current_pos, current_heading))

                # TODO: could incorporate an extra margin
                desired_pos = self.map.pos_to_ideal_position(current_pos)
                desired_heading = self.map.pos_to_ideal_heading(current_pos)
                print("Desired: pos = {}, phi = {}".format(desired_pos, desired_heading))
                
                # Preprocessing
                current_p = current_pos[1] if desired_pos[0] == current_pos[0] else current_pos[0]
                desired_p = desired_pos[1] if desired_pos[0] == current_pos[0] else desired_pos[0]
                # make sure heading is between 0 and 360
                current_heading += 180 #if current_heading <= 0 else -180
                desired_heading += 180 #if desired_heading <= 0 else -180

                print("current_p: {}, desired_p: {}".format(current_p, desired_p))
                print("current_heading: {}, desired_heading: {}".format(current_heading, desired_heading))

                if self.controller_counter < 10:
                    if(abs(current_p-desired_p) > tol_pos or abs(current_heading-desired_heading) > tol_heading):
                        # Calculate controller output
                        v_out, omega_out = C.getControlOutput(current_p, current_heading, desired_p, desired_heading, v_ref=1, dt_last=dt)
                        if self.veh_id == 27:
                            print("[{}]: v = {}, omega = {}".format(self.veh_id, v_out, omega_out))
                        # Send cmd to duckiebot
                        msg = Twist2DStamped()
                        msg.v = v_out
                        msg.omega = omega_out
                        self.pub_car_cmd.publish(msg)
                        # stop car
                        sleep(0.1)
                        self.stopDuckiebot()
                        sleep(1)
                    self.controller_counter += 1
                    print(self.controller_counter)
                else:
                    msg = Twist2DStamped()
                    msg.v = 0
                    msg.omega = 0
                    self.pub_car_cmd.publish(msg)

                    
                # For open loop control   
                # self.calculate_car_cmd()
                # if self.car_cmd_array:
                #     self.current_car_cmd = self.car_cmd_array.pop(0)
                #     self.pub_car_cmd.publish(self.current_car_cmd)
                #     # self.finished_execution = False
                #     self.log("Applying v = {}, w = {}".format(self.current_car_cmd.v, self.current_car_cmd.omega))
                #     # if not self.car_cmd_array:
                #     #     # just popped out last one
                #     #     self.finished_execution = True
                # else:
                #     self.log("Finished applying commands")
                #     self.stopDuckiebot()
                #     # self.calculate_car_cmd()

                # if self.finishedRescue():
                #     self.log("Finished Rescue")
                #     self.activated = False
                #     self.autobot_info.rescue_class = Distress.NORMAL_OPERATION
                #     msg = BoolStamped()
                #     msg.data = True
                #     self.pub_everything_ok.publish(msg)
                    # rospy.set_param('~everythingOK', 'false')
            rate.sleep()


if __name__ == '__main__':
    # create the node
    node = RescueAgentNode(node_name='rescue_agent_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
