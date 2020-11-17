#!/usr/bin/env python

import rospy
import tf
from math import atan2
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PointStamped


class MapAlignement:

    def __init__(self):
        rospy.init_node('map_alignemnt')

        # Global variables
        self.points_in_first_frame_ = []
        self.points_in_second_frame_ = []

        # Subscriber
        self.nb_points_ = 4
        rospy.Subscriber("/clicked_point", PointStamped, self.clicked_point_cb)

        # TF listener
        self.listener_ = tf.TransformListener()
        # self.listener_.waitForTransform("map_gmapping", "map_slam_toolbox", rospy.Time(), rospy.Duration(4.0))

        # TF broadcaster
        self.broadcaster_ = tf.TransformBroadcaster()

        self.run()

    def run(self):
        rospy.spin()

    def clicked_point_cb(self,pt_msg):
        if self.nb_points_ > 0:
            rospy.loginfo("Received the " + str(5-self.nb_points_) + "n-ieme point")
            if self.nb_points_ > 2:
                self.points_in_first_frame_.append(pt_msg.point)
                rospy.loginfo(pt_msg.header)
            else:
                self.points_in_second_frame_.append(pt_msg.point)
                rospy.loginfo(pt_msg.header)
            self.nb_points_ -= 1
            if self.nb_points_ == 0:
                self.compute_tf_between_frames()

    def compute_tf_between_frames(self):
        rospy.loginfo("compute_tf_between_frames")
        rospy.loginfo(self.points_in_first_frame_)
        rospy.loginfo(self.points_in_second_frame_)

        # Get two points in first frame
        # Get two points in second frame
        # Project the first points from first frame to world
        # Project the first points from second frame to world

        # Compute distance between the two --> TRANSLATION BETWEEN THE FRAMES
        dx = self.points_in_second_frame_[0].x-self.points_in_first_frame_[0].x
        dy = self.points_in_second_frame_[0].y-self.points_in_first_frame_[0].y
        translation = [dx,dy,0]
        rospy.loginfo("translation = " + str(translation))

        # Get angle between two points of first frame and world x-axis
        dx = self.points_in_second_frame_[0].x - self.points_in_second_frame_[1].x
        dy = self.points_in_second_frame_[0].y - self.points_in_second_frame_[1].y
        rot1 = atan2(dy,dx)
        rospy.loginfo("rot1 = " + str(rot1))
        # Get angle between two points of second frame and world x-axis
        dx = self.points_in_first_frame_[0].x - self.points_in_first_frame_[1].x
        dy = self.points_in_first_frame_[0].y - self.points_in_first_frame_[1].y
        rot2 = atan2(dy,dx)
        rospy.loginfo("rot2 = " + str(rot2))
        # Compute difference between the two angles --> ROTATION BETWEEN THE FRAMES
        orientation = rot1 - rot2
        rospy.loginfo("orientation = " + str(orientation))
        self.broadcaster_.sendTransform(translation,quaternion_from_euler(0,0,orientation),rospy.Time.now(),"map_slam_toolbox","map_gmapping")

if __name__ == '__main__':
    MapAlignement()
