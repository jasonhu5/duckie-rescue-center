#!/usr/bin/env python
# import os
# import rospy
# from duckietown import DTROS
# from std_msgs.msg import String
# from duckietown_msgs.msg import BoolStamped, Twist2DStamped, FSMState
# # from geometry_msgs.msg import TransformStamped, Transform
# from visualization_msgs.msg import Marker, MarkerArray
#
#
# class RescueManagerNode(DTROS):
#
#     def __init__(self, node_name):
#
#         # initialize DTROS parent class
#         super(RescueManagerNode, self).__init__(node_name=node_name)
#         self.distressed_veh = rospy.get_param("~distressed_veh")
#         self.activated = False
#         self.distressType = 0
#
#
#         # this has to be specified, when launching the node (see below)
#   #       <node pkg="rescue_center" type="rescue_manager_node.py" name="rescue_manager_XY" output="screen">
#   #             <param name="~distressed_veh" type="string" value="autobot27" />
#   #        </node>
#         self.rescue_classification =
#
#         self.sub = cb
#
#         self.sub = rospy.Subscriber("/rescue_node/", FSMState, self.cb_fsm, callback_args=veh_name)
#
#         # get robot names: currently this is passed as ENVIRONMENT Variables
#         # when running docker run
#         n_duckiebots = 3
#         param_name = "AUTOBOT_NAME_"
#         self.veh_list= list()
#         for i in range(n_duckiebots):
#             self.veh_list.append(os.environ[param_name+str(i)])
#         # build dictionary
#         self.id_dict = dict()
#         keys = ["FSM_state", "position_x", "position_y", "heading", "rescue_class", "in_rescue"]
#         for veh_name in self.veh_list:
#             self.id_dict[veh_name] = dict.fromkeys(keys)
#             self.id_dict[veh_name]["in_rescue"] = False
#
#         # subscribe to topics
#         # 1. online viz
#         self.sub_markers = rospy.Subscriber(
#             "/cslam_markers", MarkerArray, self.cb_visualization, queue_size=30)
#         # 2. FSM states
#         self.sub_fsmStates = dict()
#         for veh_name in self.veh_list:
#             topic_name = "/{}/fsm_node/mode".format(veh_name)
#             self.sub_fsmStates[veh_name](rospy.Subscriber("%s"%(topic_name), FSMState, self.cb_fsm, callback_args=veh_name))
#
#         # publish to topics
#         # 1. Trigger for each duckiebot
#         self.pub_trigger = dict()
#         for veh_name in self.veh_list:
#             self.pub_trigger[veh_name](rospy.Publisher(
#                 "/{}/recovery_mode".format(veh_name),
#                 BoolStamped,
#                 queue_size=1,
#             ))
#
#         #
#         # self.pub_car_cmd = rospy.Publisher(
#         #     "/{}/lane_recovery_node/car_cmd".format(self.veh_name),
#         #     Twist2DStamped,
#         #     queue_size=1,
#         # )
#
#         # TODO: remove testing param
#         # param for testing
#         # self.parameters['~trigger_rescue'] = None
#         # self.updateParameters()
#         #
#         # # current trigger status
#         # self.rescue_on = False
#         # self.trigger = self.rescue_on
#         #
#         # self.prev_cmd_pub = True
#
#     # Callback for online viz
#     def cb_visualization(self, msg):
#         markers = msg.markers
#         bot_poses = []
#         for m in markers:
#             if m.ns == "duckiebots":
#                 idx = m.id
#                 x = m.pose.position.x
#                 y = m.pose.position.y
#                 self.id_dict[idx]["position_x"] = x;
#                 self.id_dict[idx]["position_y"] = y;
#                 # self.log("{} {}".format(x, y))
#                 if not self.id_dict[idx]["in_rescue"]:
#                     rescue_class = self.classifier(x, y, idx)
#                     if rescue_class > 0:
#                         # publish rescue_class
#                         msg = BoolStamped()
#                         msg.data = True
#                         self.pub_trigger[idx].publish(msg)
#                         self.id_dict[idx]["in_rescue"] = True
#
#                 # self.trigger = True if y > 0.5 else False
#                 # self.log("{}".format("Should shop" if y > 0.6 else "Fine"))
#
#     def classifier(self, x, y, idx):
#         rescue_class = 0;
#         if y>0.5:
#             # out of lane
#             # TODO: hard code map
#             rescue_class = 1
#         else if self.id_dict[idx]["FSM_state"] =="KEEP_CALM":
#             # duckiebot is stuck
#             rescue_class = 2
#         return rescue_class
#
#     # Callback for fsm state
#     def cb_fsm(self, msg, veh_name):
#         self.id_dict[veh_name]["FSM_state"] = msg;
#
#
#
#     def run(self):
#
#         # publish rate
#         rate = rospy.Rate(4) # 10Hz
#
#         while not rospy.is_shutdown():
#             if self.trigger != self.rescue_on:
#                 self.rescue_on = self.trigger
#                 msg = BoolStamped()
#                 msg.data = self.rescue_on
#                 # self.log("Sending %s to recovery_mode" % msg.data)
#                 self.pub_trigger.publish(msg)
#
#             if self.rescue_on:
#                 car_cmd_msg = Twist2DStamped()
#                 car_cmd_msg.v = -0.1 if not self.prev_cmd_pub else 0.0
#                 car_cmd_msg.omega = 0.0
#                 self.pub_car_cmd.publish(car_cmd_msg)
#                 self.prev_cmd_pub = not self.prev_cmd_pub
#
#             rate.sleep()
#
#
# if __name__ == '__main__':
#     # create the node
#     node = RescueTriggerNode(node_name='rescue_trigger_node')
#     # run node
#     node.run()
#     # keep spinning
#     rospy.spin()
