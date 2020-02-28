#!/usr/bin/env python

import rospy
import tf2_ros

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class OdomFramePublisher(object):

    def __init__(self):
        rospy.init_node('odom_frame_publisher')
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.sub = rospy.Subscriber(
                '~input', Odometry, self._cb, queue_size=1)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.map_frame_id = rospy.get_param('~map_frame_id','map')
        self.odom_frame_id = rospy.get_param('~odom_frame_id','t265_odom_frame')
        self.pose_frame_id = rospy.get_param('~pose_frame_id','t265_pose_frame')
        self.internal_pose_frame_id = rospy.get_param('~internal_pose_frame_id','internal_t265_pose_frame')
        self.flag = True

        rospy.loginfo('Initialization finished')

    def _cb(self, msg):
        self.sub.unregister()
        if self.flag:
            self.flag = False

            tf_pose2map = self.tfBuffer.lookup_transform(
                    self.map_frame_id,
                    self.pose_frame_id,
                    rospy.Time(0))
            tf_interpose2map = self.tfBuffer.lookup_transform(
                    self.map_frame_id,
                    self.internal_pose_frame_id,
                    rospy.Time(0))
            tf_

            static_transformStamped = TransformStamped()
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = self.map_frame_id
            static_transformStamped.child_frame_id = self.odom_frame_id

            self.broadcaster.sendTransform( static_transformStamped )

            rospy.loginfo('broadcasting tf')

if __name__=='__main__':
    odompub = OdomFramePublisher()
    rospy.spin()
