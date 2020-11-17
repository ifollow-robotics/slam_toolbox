#!/usr/bin/env python

import rospy
import tf
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Pose,Twist,Point

class LocalizationMetric:
    def __init__(self):
        rospy.init_node('assess_loc_metric')

        # Subscribers
        rospy.Subscriber("/slam_toolbox/scan_match_score", Float64, self.score_cb)

        # Listen to tf
        self.rate_ = 0.1 # in sec
        self.first_time_ = True
        self.listener_ = tf.TransformListener()
        self.listener_.waitForTransform("/ilogistics_3_0003/odom", "/ilogistics_3_0003/map", rospy.Time(), rospy.Duration(4.0))

        # Publishers
        self.moving_mean_pub_ = rospy.Publisher("/slam_toolbox/score_moving_mean",Float64,queue_size=10)
        self.odom_frame_pose_pub_ = rospy.Publisher("/slam_toolbox/odom_frame_pose",Pose,queue_size=10)
        self.odom_frame_diff_pub_ = rospy.Publisher("/slam_toolbox/odom_frame_differential",Twist,queue_size=10)

        # Global variables
        self.buffer_size_ = 4
        self.scores_ = [0.0]*self.buffer_size_
        self.index_ = 0
        self.current_pose_ = []
        self.current_orient_ = []
        self.previous_pose_ = []
        self.previous_orient_ = []

        # Set timer
        # self.timer = rospy.Timer(rospy.Duration(self.rate_), self.listen_to_tf)

        # Run the node
        self.run()

    def publish_odom_frame(self,trans,rot):
        # Get the current odom frame's position and orientation
        self.current_pose_ = trans
        self.current_orient_ = rot
        pose_msg = Pose()
        (_,_,yaw1) = euler_from_quaternion(rot)
        pose_msg.position.x = trans[0]
        pose_msg.position.y = trans[1]
        pose_msg.orientation.z = yaw1
        self.odom_frame_pose_pub_.publish(pose_msg)
        # Get the differential in x, y and theta
        (_,_,yaw2) = euler_from_quaternion(self.previous_orient_)
        yaw_diff = yaw1 - yaw2
        # Publish differential
        msg = Twist()
        msg.linear.x = self.current_pose_[0] - self.previous_pose_[0]
        msg.linear.y = self.current_pose_[1] - self.previous_pose_[1]
        msg.angular.z = yaw_diff # Yaw angle
        self.odom_frame_diff_pub_.publish(msg)
        # Update last odom frame
        self.previous_pose_ = self.current_pose_
        self.previous_orient_ = self.current_orient_

    def compute_differential_yaw(self,orient1,orient2):
        orient1[3] = -orient1[3]
        return tf.transformations.quaternion_multiply(orient2,orient1)


    def score_cb(self,score_msg):
        self.scores_[self.index_%self.buffer_size_] = score_msg.data
        self.index_+=1
        sum_scores = sum(self.scores_)
        if self.index_ >= self.buffer_size_ and sum_scores != 0.0:
            self.moving_mean_pub_.publish(sum(self.scores_)/self.buffer_size_)

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.listener_.waitForTransform("/ilogistics_3_0003/odom", "/ilogistics_3_0003/map", rospy.Time(), rospy.Duration(2.5))
                (trans,rot) = self.listener_.lookupTransform('/ilogistics_3_0003/odom', '/ilogistics_3_0003/map',rospy.Time.now())
                if self.first_time_:
                    self.previous_pose_ = trans
                    self.previous_orient_ = rot
                    self.first_time_ = False
                # elif self.previous_orient_ != self.current_orient_:
                else:
                    self.publish_odom_frame(trans,rot)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("problem with tf between odom and map in localization assessment")
                continue


if __name__ == '__main__':
    LocalizationMetric()
