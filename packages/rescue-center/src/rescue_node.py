#!/usr/bin/env python
import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
from duckietown_msgs.msg import BoolStamped, Twist2DStamped
# from geometry_msgs.msg import TransformStamped, Transform
from visualization_msgs.msg import Marker, MarkerArray


class RescueTriggerNode(DTROS):

    def __init__(self, node_name):

        # initialize DTROS parent class
        super(RescueTriggerNode, self).__init__(node_name=node_name)

        # get robot name
        self.veh_name = os.environ["AUTOBOT_NAME"]

        # subscribe to topics
        # self.sub_poses = rospy.Subscriber(
        #     "/agent_poses", TransformStamped, self.callback, queue_size=30)
        self.sub_markers = rospy.Subscriber(
            "/cslam_markers", MarkerArray, self.callback, queue_size=30)
        # publish to topics
        self.pub_trigger = rospy.Publisher(
            "/{}/recovery_mode".format(self.veh_name),
            BoolStamped,
            queue_size=1,
        )
        self.pub_car_cmd = rospy.Publisher(
            "/{}/lane_recovery_node/car_cmd".format(self.veh_name),
            Twist2DStamped,
            queue_size=1,
        )

        # TODO: remove testing param
        # param for testing
        self.parameters['~trigger_rescue'] = None
        self.updateParameters()

        # current trigger status
        self.rescue_on = False
        self.trigger = self.rescue_on

        self.prev_cmd_pub = True 


    def callback(self, msg):
        markers = msg.markers
        bot_poses = []
        for m in markers:
            if m.ns == "duckiebots":
                idx = m.id
                x = m.pose.position.x
                y = m.pose.position.y
                # self.log("{} {}".format(x, y))

                self.trigger = True if y > 0.5 else False
                # self.log("{}".format("Should shop" if y > 0.6 else "Fine"))


    def run(self):

        # publish rate
        rate = rospy.Rate(4) # 10Hz

        while not rospy.is_shutdown():
            if self.trigger != self.rescue_on:
                self.rescue_on = self.trigger
                msg = BoolStamped()
                msg.data = self.rescue_on
                # self.log("Sending %s to recovery_mode" % msg.data)
                self.pub_trigger.publish(msg)
            
            if self.rescue_on:
                car_cmd_msg = Twist2DStamped()
                car_cmd_msg.v = -0.1 if not self.prev_cmd_pub else 0.0
                car_cmd_msg.omega = 0.0
                self.pub_car_cmd.publish(car_cmd_msg) 
                self.prev_cmd_pub = not self.prev_cmd_pub

            rate.sleep()


if __name__ == '__main__':
    # create the node
    node = RescueTriggerNode(node_name='rescue_trigger_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
