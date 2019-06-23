#!/usr/bin/env python
# -*- coding: utf-8 -*-

import signal
import threading

import rospy
import rospkg
from std_msgs.msg import Float64, String


class Main:

    def __init__(self):
        """
        /shoulder_revolute_servo_controller/command = 1
        /shoulder_flex_servo_controller/command = 2
        /elbow_servo_controller/command = 3
        /wrist_servo_controller/command = 4
        /finger_servo_controller/command = 5
        """

        rospy.init_node("arm")

        self.motor_topics = [
            "/shoulder_revolute_servo_controller/command",
            "/shoulder_flex_servo_controller/command",
            "/elbow_servo_controller/command",
            "/wrist_servo_controller/command",
            "/finger_servo_controller/command"
        ]

        self.publishers = [
            rospy.Publisher(self.motor_topics[0], Float64, queue_size=10),
            rospy.Publisher(self.motor_topics[1], Float64, queue_size=10),
            rospy.Publisher(self.motor_topics[2], Float64, queue_size=10),
            rospy.Publisher(self.motor_topics[3], Float64, queue_size=10),
            rospy.Publisher(self.motor_topics[4], Float64, queue_size=10)
        ]

        self.open_position=[
            0.0,
            -0.7,
            -1.4,
            0.7,
            0
        ]

        self.close_position = [
            0.0,
            -0.7,
            -1.4,
            0.7,
            0.6
        ]

        rospy.Subscriber("/arm/control", String, self.callback)
        rospy.spin()

    def callback(self, message):
        # type: (String) -> None
        for i in range(4):
            self.publishers[i].publish(self.open_position[i])
        rospy.sleep(3.0)
        self.publishers[4].publish(-0.6)
        rospy.sleep(3.0)
        for i in range(5):
            self.publishers[i].publish(self.close_position[i])


if __name__ == "__main__":
    Main()