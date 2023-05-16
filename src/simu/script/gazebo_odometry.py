#!/usr/bin/env python3

'''
This script makes Gazebo less fail by translating gazebo status messages to odometry data.
Since Gazebo also publishes data faster than normal odom data, this script caps the update to 20hz.
Winter Guerra
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import math
import tf
import tf2_ros

class OdometryNode:
    # Set publishers
    pub_odom = rospy.Publisher('/odom', Odometry, queue_size=1)

    def __init__(self):
        # init internals
        self.last_received_pose = Pose()
        self.last_received_twist = Twist()
        self.vel_direction = 1.
        self.last_recieved_stamp = None

        # Set the update rate
        rospy.Timer(rospy.Duration(.02), self.timer_callback) # 50hz

        self.tf_pub = tf2_ros.TransformBroadcaster()

        # Set subscribers
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.sub_robot_pose_update, queue_size=1, buff_size=52428800, tcp_nodelay=True)
        rospy.Subscriber("/ackermann_cmd_mux/output", AckermannDriveStamped, self.get_vel_direction, queue_size=1, buff_size=52428800, tcp_nodelay=True)

    def sub_robot_pose_update(self, msg):
        # Find the index of the robot
        try:
            arrayIndex = msg.name.index('xju::base_link')
        except ValueError as e:
            # Wait for Gazebo to startup
            pass
        else:
            # Extract our current position information
            time_diff = 0.02
            if self.last_recieved_stamp is not None:
                time_diff = (rospy.Time.now() - self.last_recieved_stamp).to_sec()
            if time_diff == 0.:
                return
            self.last_received_twist.linear.x = self.vel_direction * math.sqrt((msg.pose[arrayIndex].position.x - self.last_received_pose.position.x)**2 + (msg.pose[arrayIndex].position.y - self.last_received_pose.position.y)**2) / time_diff
            (r1, p1, y1) = tf.transformations.euler_from_quaternion([msg.pose[arrayIndex].orientation.x, msg.pose[arrayIndex].orientation.y, msg.pose[arrayIndex].orientation.z, msg.pose[arrayIndex].orientation.w])
            (r2, p2, y2) = tf.transformations.euler_from_quaternion([self.last_received_pose.orientation.x, self.last_received_pose.orientation.y, self.last_received_pose.orientation.z, self.last_received_pose.orientation.w])
            self.last_received_twist.angular.z = (y1 - y2) / time_diff
            self.last_received_pose = msg.pose[arrayIndex]
            self.last_recieved_stamp = rospy.Time.now()

    def get_vel_direction(self, msg):
        if msg.drive.speed < 0:
            self.vel_direction = -1.
        else:
            self.vel_direction = 1.

    def timer_callback(self, event):
        if self.last_recieved_stamp is None:
            return

        cmd = Odometry()
        cmd.header.stamp = self.last_recieved_stamp
        cmd.header.frame_id = 'odom'
        cmd.child_frame_id = 'base_link'
        cmd.pose.pose = self.last_received_pose
        cmd.twist.twist = self.last_received_twist
        cmd.pose.covariance =[1e-3, 0, 0, 0, 0, 0,
                              0, 1e-3, 0, 0, 0, 0,
                              0, 0, 1e6, 0, 0, 0,
                              0, 0, 0, 1e6, 0, 0,
                              0, 0, 0, 0, 1e6, 0,
                              0, 0, 0, 0, 0, 1e3]

        cmd.twist.covariance = [1e-9, 0, 0, 0, 0, 0,
                                0, 1e-3, 1e-9, 0, 0, 0,
                                0, 0, 1e6, 0, 0, 0,
                                0, 0, 0, 1e6, 0, 0,
                                0, 0, 0, 0, 1e6, 0,
                                0, 0, 0, 0, 0, 1e-9]


        self.pub_odom.publish(cmd)

        tf = TransformStamped(
            header=Header(
                frame_id=cmd.header.frame_id,
                stamp=cmd.header.stamp
            ),
            child_frame_id=cmd.child_frame_id,
            transform=Transform(
                translation=cmd.pose.pose.position,
                rotation=cmd.pose.pose.orientation
            )
        )
        self.tf_pub.sendTransform(tf)

# Start the node
if __name__ == '__main__':
    rospy.init_node("gazebo_odometry_node")
    node = OdometryNode()
    rospy.spin()