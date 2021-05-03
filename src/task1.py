#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
from com2009_srv_examples.srv import Approach, ApproachResponse
from tf.transformations import euler_from_quaternion

class ObjectApproacher:
    def __init__(self):
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        rospy.init_node('publisher_node', anonymous=True)
        self.rate = rospy.Rate(10) # hz
        rospy.loginfo("Publisher node is active...")

        self.sub = rospy.Subscriber("scan", LaserScan, self.scan_callback)
        self.sub1 = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.current_rotation = 0
        self.distance_to = 999

        self.rotation_target = 0
        self.start_rotation = 0

        self.turn = False
        rospy.loginfo("Subscriber node is active...")

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def main_loop(self):
        while not self.ctrl_c:

            if self.turn:
                self.vel = Twist()
                self.vel.linear.x = 0
                if self.distance_to > 1:
                    self.turn = False
                    rospy.loginfo("no obstacles in front")
                else:
                    self.vel = Twist()
                    self.vel.angular.z = 0.1
                    rospy.loginfo("turning")
            else:
                if self.distance_to < 0.4:
                    self.turn = True
                else:
                    self.vel = Twist()
                    self.vel.linear.x = 0.1
                    print("moving forwards")

            self.pub.publish(self.vel)
            self.rate.sleep()

    def shutdownhook(self):
        self.shutdown_function()
        self.ctrl_c = True

    def shutdown_function(self):
        self.pub.publish(Twist())
        print("stopping publisher node at: {}".format(rospy.get_time()))

    def scan_callback(self, data):
        left_arc = data.ranges[0:30]
        right_arc = data.ranges[-30:]
        front_arc = np.array(left_arc + right_arc)
        self.distance_to = front_arc.min()

    def odom_callback(self, data):
        (roll, pitch, yaw) = euler_from_quaternion([data.pose.pose.orientation.x,
                                                    data.pose.pose.orientation.y,
                                                    data.pose.pose.orientation.z,
                                                    data.pose.pose.orientation.w],'sxyz')
        self.current_rotation = yaw


if __name__ == "__main__":
    approacher = ObjectApproacher()
    approacher.main_loop()
