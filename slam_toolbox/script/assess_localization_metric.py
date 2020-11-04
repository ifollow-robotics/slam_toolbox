#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64

class LocalizationMetric:
    def __init__(self):
        rospy.init_node('assess_loc_metric')

        # Subscribers
        rospy.Subscriber("/slam_toolbox/scan_match_score", Float64, self.score_cb)

        # Publisher
        self.moving_mean_pub_ = rospy.Publisher("/slam_toolbox/score_moving_mean",Float64,queue_size=10)

        # Global variables
        self.buffer_size_ = 10
        self.scores_ = [0.0]*self.buffer_size_
        self.index_ = 0

        # Run the node
        self.run()

    def score_cb(self,score_msg):
        self.scores_[self.index_%self.buffer_size_] = score_msg.data
        self.index_+=1
        sum_scores = sum(self.scores_)
        if self.index_ >= self.buffer_size_ and sum_scores != 0.0:
            self.moving_mean_pub_.publish(sum(self.scores_)/self.buffer_size_)

    def run(self):
        # Spin
        rospy.spin()

if __name__ == '__main__':
    LocalizationMetric()
