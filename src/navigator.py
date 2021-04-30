#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
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
        rospy.loginfo("Subscriber node is active...")

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def main_loop(self):
        while not self.ctrl_c:
            direction = Twist()

            # Obstacle Checking


            if self.rotation_target > self.current_rotation:
                rospy.loginfo(self.rotation_target)
                rospy.loginfo(self.current_rotation)
                direction.angular.z = -0.1

            elif self.distance_to < 0.4:
                rospy.loginfo(self.current_rotation)
                self.start_rotation = self.current_rotation
                self.rotation_target = math.pi / 2
            else:
                direction.linear.x = 0.1
            # Moving

            self.pub.publish(direction)
            self.rate.sleep()

    def shutdownhook(self):
        self.shutdown_function()
        self.ctrl_c = True

    def shutdown_function(self):
        self.pub.publish(Twist())
        print("stopping publisher node at: {}".format(rospy.get_time()))

    def scan_callback(self, data):
        self.distance_to = sum([data.ranges[i] for i in range(-10, 10)]) / 20

    def odom_callback(self, data):
        (roll, pitch, yaw) = euler_from_quaternion([data.pose.pose.orientation.x,
                                                    data.pose.pose.orientation.y,
                                                    data.pose.pose.orientation.z,
                                                    data.pose.pose.orientation.w],'sxyz')
        self.current_rotation = yaw


if __name__ == "__main__":
    approacher = ObjectApproacher()
    approacher.main_loop()
