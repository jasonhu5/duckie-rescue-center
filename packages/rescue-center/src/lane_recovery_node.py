#!/usr/bin/env python
import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
from duckietown_msgs.msg import Twist2DStamped, LanePose, WheelsCmdStamped, BoolStamped, FSMState, StopLineReading


class lane_recovery(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        self.veh_name = os.environ["AUTOBOT_NAME"]
        self.pub_car_cmd = rospy.Publisher("/{}/lane_recovery_node/car_cmd".format(
            self.veh_name), Twist2DStamped, queue_size=1)


    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            car_control_msg = Twist2DStamped()
            car_control_msg.v = 0.0
            car_control_msg.omega = 0.0
            self.pub_car_cmd.publish(car_control_msg)
            rate.sleep()


if __name__ == '__main__':
    # create the node
    rospy.init_node("lane_recovery_node", anonymous=False)  # adapted to sonjas default file
    lane_recovery_node = lane_recovery()
    lane_recovery_node.run()
    # keep spinning
    rospy.spin()
