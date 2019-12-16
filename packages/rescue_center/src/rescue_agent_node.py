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

# Parameters
TOL_ANGLE_IN_DEG = 30
TOL_POSITION_IN_M = 0.14625  # tileSize/4
KP_CONTROLLER = 6
KI_CONTROLLER = 0.1
C1_CONTROLLER = -5
C2_CONTROLLER = 1
V_REF_CONTROLLER = 0.3
T_EXECUTION = 0.3
T_STOP = 1
NUMBER_RESCUE_CMDS = 3  # before checking readyForLF()


# Parameters for Closed Loop Control

class RescueAgentNode(DTROS):
    """Rescue Agent Node

    - takes care of rescue operation, once duckiebot has been classified as "distressed" (from rescue center).
    - a "Rescue Agent" is launched for every duckiebot, which has been spotted by the localization system
    (in passive mode for non-distressed duckiebots)

    Args:
        node_name (): a unique, descriptive name for the node that ROS will use

    Attributes:
        - veh_name (str): name of the vehicle, e.g. autobot27
        - veh_id (int): ID of the vehicle, e.g. 27
        - activated (boolean): rescue agent sends rescue commands to duckiebot, if and only if self.activated == True
        - autobot_info(:obj:`AutobotInfo`): object, where all current information of the duckiebot is saved (e.g. position, heading, etc.)
        - map (:obj:`SimpleMap`): map object providing functions for calculating desired position and heading,
                                reads in YAML file --> works for other maps as well

        - v_ref(float): backward velocity during closed loop rescue
        - controller_counter (int): counting up, after each rescue cmd. Stops after maximum number of rescue cmds
        - desired_pos(tuple (float, float)): current desired position for rescue operation
        - desired_heading (float): current desired heading for rescue operation (-180 DEG --> 180 DEG, 0 DEG facing positive x-Direction)
        - current_car_cmd (:obj:`Twist2DStamped`): car command, which is being sent to the duckiebot
        - currentPosition: TODO
        - currentHeading: TODO
    Subscriber:
        TODO: define message type Distress and directly pass that (in rescue center)
        - sub_distress_classification: /[self.veh_name]/distress_classification (:obj:`String`):
            distress classification from rescue center as rescue_class.value (int converted to String)
        - sub_autobot_info: /[self.veh_name]/autobot_info (:obj: `AutobotInfoMsg`):
            AutobotInfo message from rescue_center

    Publisher:
        - pub_rescueDone: [self.veh_name]/rescueDone/ (:obj:`BoolStamped`):
            publishes, if rescue operation has been finished
            1. to FSM node, True triggers state transition to LANE_FOLLOWING
            2. to rescue_center: True activates monitoring of duckiebot
        - pub_car_cmd: /[self.veh_name]/lane_recovery_node/car_cmd (:obj: `Twist2DStamped`):
            publishes rescue commands to car_cmd_switch_node
        - pub_test: /rescue_agents/test: test publisher to monitor certain values for debugging
    """

    def __init__(self, node_name):

        # initialize DTROS parent class
        super(RescueAgentNode, self).__init__(node_name=node_name)

        # inialize attributes
        self.veh_name = rospy.get_param("~distressed_veh")
        self.veh_id = int(
            ''.join([x for x in self.veh_name if x.isdigit()]))
        self.activated = False
        self.autobot_info = AutobotInfo(self.veh_id)
        # TODO: save map path somewhere
        map_file_path = os.path.join(
            "/code/catkin_ws/src",
            "duckie-rescue-center/packages/rescue_center/src",
            "test_map.yaml"
        )
        self.map = SimpleMap(map_file_path)
        self.current_car_cmd = Twist2DStamped()
        self.current_car_cmd.v = 0
        self.current_car_cmd.omega = 0
        self.controller_counter = 0
        self.v_ref = V_REF_CONTROLLER
        self.desired_pos = None
        self.desired_heading = None
        self.controller = StuckController(
            k_P=KP_CONTROLLER, k_I=KI_CONTROLLER, c1=C1_CONTROLLER, c2=C2_CONTROLLER)
        self.currentPosition = (None, None)
        self.currentHeading = None

        self.tileType = None

        # Subscriber
        self.sub_distress_classification = rospy.Subscriber("/{}/distress_classification".format(self.veh_name),
                                                            String, self.cb_distress_classification)
        self.sub_autobot_info = rospy.Subscriber(
            "/{}/autobot_info".format(
                self.veh_name), AutobotInfoMsg, self.cb_autobot_info
        )

        # Publisher
        self.pub_rescueDone = rospy.Publisher(
            "{}/rescueDone/".format(self.veh_name), BoolStamped, queue_size=1)
        self.pub_car_cmd = rospy.Publisher(
            "/{}/lane_recovery_node/car_cmd".format(self.veh_name),
            Twist2DStamped,
            queue_size=1,
        )
        self.pub_test = rospy.Publisher(
            "/rescue_agents/test",
            String,
            queue_size=1,
        )

    def cb_autobot_info(self, msg):
        """Callback function triggered by self.sub_autobot_info:
                saves msg into self.autobot_info

            Args:
            - msg (:obj:`AutobotInfoMsg`): received message

            Parameters: None

            Returns: None
        """
        self.pub_test.publish("Got info for {}".format(self.veh_name))
        self.autobot_info.timestamp = msg.timestamp
        self.autobot_info.fsm_state = msg.fsm_state
        self.autobot_info.position = (msg.position[0], msg.position[1])
        self.autobot_info.filtered = (msg.filtered[0], msg.filtered[1])
        self.autobot_info.last_moved = msg.last_moved
        self.autobot_info.in_rescue = msg.in_rescue
        self.autobot_info.onRoad = msg.onRoad
        self.autobot_info.heading = msg.heading
        self.autobot_info.headingSimple = msg.headingSimple
        self.autobot_info.positionSimple = (
            msg.positionSimple[0], msg.positionSimple[1])

    def cb_distress_classification(self, msg):
        """Callback function triggered by self.sub_distress_classification:
                - stops duckiebot, saves distress type in self.autobot_info and triggers rescue operation

            Args:
            - msg (:obj:`AutobotInfoMsg`): received message

            Parameters: None

            Returns: None
        """
        distress_type_num = int(msg.data)
        if distress_type_num > 0:
            # NOTE: this should always be the case, since rescue_center_node only publishes, if rescue_class > 0
            self.stopDuckiebot()
            self.updatePositionAndHeading()
            print("[{}] distressed at: pos = {}, phi = {}".format(
                self.veh_id, self.current_pos, self.current_heading))
            self.activated = True
            self.autobot_info.rescue_class = Distress(distress_type_num)
            print("[{}] Distressed class: {}".format(
                self.veh_id, self.autobot_info.rescue_class))

            desired_pos = self.map.pos_to_ideal_position(
            self.current_pos, heading=self.current_heading)
            if desired_pos is not None:
                if desired_pos != float('inf'):
                    print("[{}] There is a desired position: {}.".format(
                        self.veh_id, desired_pos))
                    self.desired_pos = desired_pos
                else:
                    print("[{}] Desired position is not on map") #TODO: how to rescue?
            else:
                print("[{}] There is no desired position yet.".format(self.veh_id))
            
            print("[{}] Starting rescue operation now".format(self.veh_id))


    def readyForLF(self, recalculateDesired=True, tol_angle=TOL_ANGLE_IN_DEG, tol_pos=TOL_POSITION_IN_M):
        """Checks, if duckiebot is ready for lane following (after rescue operation)

            Args: None

            Parameters:
            - recalculateDesired (Boolean):
                if True, function will recalculate desired position/heading and check tolerance based on those
                if False, function will use last desired position(heading)
            - tol_angle (float): allowed angle tolerance (+/-)
            - tol_pos (float): allowed position tolerance (+/-)

            Returns:
            - (Boolean): True, iff ready for lane following
        """
        # for debugging: /rescue/rescue_agents/rescue_agent_[self.vehID]/everythingOK
        debug_param = rospy.get_param('~everythingOK')
        if debug_param:
            return True

        # check,if duckiebot is back in lane
        current_pos = self.autobot_info.positionSimple  # (x, y)
        current_heading = self.autobot_info.headingSimple  # degree
        if recalculateDesired:
            desired_pos = self.map.pos_to_ideal_position(current_pos)
            desired_heading = self.map.pos_to_ideal_heading(desired_pos)
        else:
            desired_pos = self.desired_pos
            desired_heading = self.desired_heading
        delta_phi = abs(current_heading-desired_heading)
        delta_phi = min(delta_phi, 360-delta_phi)
        delta_d = math.sqrt(
            (desired_pos[0] - current_pos[0])**2 + (desired_pos[1] - current_pos[1])**2)
        print("delta_phi: {}, delta_d: {}".format(delta_phi, delta_d))
        if delta_d < tol_pos and delta_phi < tol_angle:
            if self.tileType != '4way' or self.tileType != '3way':
                return True
        # if duckiebot is not back in lane
        return False
    
    def stopDuckiebot(self):
        """Stops duckiebot

            Args: None

            Parameters: None

            Returns: None
        """
        car_cmd = Twist2DStamped()
        car_cmd.v = 0
        car_cmd.omega = 0
        self.current_car_cmd = car_cmd
        self.pub_car_cmd.publish(self.current_car_cmd)
    
    def goBackToLF(self):
        """Duckiebot goes back to lane following

            Args: None
        
            Parameters: None

            Returns: None
        """
        self.activated = False
        self.autobot_info.rescue_class = Distress.NORMAL_OPERATION
        msg = BoolStamped()
        msg.data = True
        self.pub_rescueDone.publish(msg)
        self.controller_counter = 0
    
    def updatePositionAndHeading(self):
        """ Updates current position and heading from autobotInfo, depending on simple localization is on or off
        """
        if self.autobot_info.positionSimple[0] == 0 and self.autobot_info.positionSimple[1] == 0:
            # simple localization has not been triggered yet (duckiebot did not move) --> take normal localization output
            self.current_pos = self.autobot_info.position  # (x, y)
            self.current_heading = self.autobot_info.heading  # degree
        else:
            # use simple localization
            self.current_pos = self.autobot_info.positionSimple  # (x, y)
            self.current_heading = self.autobot_info.headingSimple  # degree

    def run(self):
        """Main loop of ROS node

            Args: None

            Parameters: None

            Returns: None
        """
        # publish rate
        rate = rospy.Rate(4)  # 10Hz

        while not rospy.is_shutdown():
            if self.activated:
                # in rescue operation

                # --- GET CURRENT POSITION ---#
                self.updatePositionAndHeading()
                self.tileType = self.map.pos_to_semantic(self.current_pos)
                print("[{}] Current: pos = {}, phi = {}".format(
                    self.veh_id, self.current_pos, self.current_heading))

                # --- CALCULATE DESIRED POSITION ---#
                desired_pos = self.map.pos_to_ideal_position(
                    self.current_pos, heading=self.current_heading)
                # check, if at bad position
                if desired_pos == float('inf') or desired_pos == None:
                    # TODO: when does it return None again?
                    # No good rescue point found: e.g. on asphalt next to curves
                    print('[{}] no good point found! Going backwards now...'.format(self.veh_id))
                    self.current_car_cmd.v = -self.v_ref
                    self.current_car_cmd.omega = 0
                    self.pub_car_cmd.publish(self.current_car_cmd)
                    sleep(T_EXECUTION)
                    # stop car
                    self.stopDuckiebot()
                    sleep(T_STOP)
                    continue
                # Check, if duckiebot drove into intersection: 4-way or 3-way:
                if self.tileType == '4way' or self.tileType == '3way':
                    if self.autobot_info.rescue_class != Distress.STUCK_IN_INTERSECTION: 
                        print("[{}] drove into Intersection, check, if ready for LF".format(self.veh_id))
                        self.stopDuckiebot()
                        if self.desired_pos:
                            if self.readyForLF(recalculateDesired=False):
                                print("[{}] Ready for LF".format(self.veh_id))
                                self.goBackToLF()
                                # TODO: implement else case: go forward
                            else: 
                               self.autobot_info.rescue_class = Distress.STUCK_IN_INTERSECTION 
                            continue                        
                
                # --- CALCULATE DESIRED HEADING--- #
                self.desired_pos = desired_pos # need this, because I want to use the old desired_pos in case of an intersection
                self.desired_heading = self.map.pos_to_ideal_heading(
                    self.desired_pos)
                print("[{}] Desired: pos = {}, phi = {}".format(
                    self.veh_id, self.desired_pos, self.desired_heading))
                
                # --- EXECUTE CONTROL COMMANDS---#
                if self.controller_counter < NUMBER_RESCUE_CMDS:
                    # Calculate controller output
                    v_out, omega_out = self.controller.getControlOutput(
                        self.current_pos, self.current_heading, self.desired_pos, self.desired_heading, v_ref=self.v_ref, dt_last=T_EXECUTION)
                    print("[{}]: v = {}, omega = {}".format(
                        self.veh_id, v_out, omega_out))
                    # Send cmd to duckiebot
                    self.current_car_cmd.v = v_out
                    self.current_car_cmd.omega = omega_out
                    self.pub_car_cmd.publish(self.current_car_cmd)
                    sleep(T_EXECUTION)
                    # stop car
                    self.stopDuckiebot()
                    sleep(T_STOP)
                    self.controller_counter += 1
                    print("Finished {} control action".format(self.controller_counter + 1))
                # --- CHECK IF RESCUE IS FINISHED --- #
                else:
                    print("[{}] Finished rescue attempt. Check, if ok...".format(
                        self.veh_id))
                    if self.readyForLF(recalculateDesired=False):
                        print("[{}] Ready for LF".format(self.veh_id))
                        self.goBackToLF()
                    else:
                        self.controller_counter = 0
                        print("[{}] Duckiebot still not ready for LF. New rescue attempt.".format(self.veh_id))
            rate.sleep()


if __name__ == '__main__':
    # create node
    node = RescueAgentNode(node_name='rescue_agent_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
