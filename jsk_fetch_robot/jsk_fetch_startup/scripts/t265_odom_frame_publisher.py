#!/usr/bin/env python

import rospy
import tf2_ros

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class OdomFramePublisher(object):

    def __init__(self):
        self.sub = rospy.Subscriber(
                '~input', Odometry, self._cb, queue_size=1)
        self.broadcaster = tf2_ros.StaticTransformBroadCaster()
        self.parent_frame_id = rospy.get_param('~parent_frame_id','map')
        self.child_frame_id = rospy.get_param('~child_frame_id','t265_pose_frame')
        self.flag = True

    def _cb(self, msg):
        self.sub.unregister()
        if self.flag:
            self.flag = False

            static_transformStamped = TransformStamped()

            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = self.parent_frame_id
            static_transformStamped.child_frame_id = self.child_frame_id

            static_transformStamped.transform.translation.x = msg.pose.pose.position.x
            static_transformStamped.transform.translation.y = msg.pose.pose.position.y
            static_transformStamped.transform.translation.z = msg.pose.pose.position.z

            static_transformStamped.transform.rotation.x = msg.pose.pose.orientation.x
            static_transformStamped.transform.rotation.y = msg.pose.pose.orientation.y
            static_transformStamped.transform.rotation.z = msg.pose.pose.orientation.z
            static_transformStamped.transform.rotation.w = msg.pose.pose.orientation.w

            boardcaster.sendTransform( static_transformStamped )

if __name__=='__main__':
    odompub = OdomFramePublisher()
    rospy.spin()
