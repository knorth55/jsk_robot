#!/usr/bin/env python

import rospy
import tf
import tf_conversions.posemath as pm

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class OdomFramePublisher(object):
    """
    a Class to publish a tf from /map to /t265_odom_frame
        based on tfs from /map to /internal_t265_pose_frame
        and from /t265_odom_frame to t265_pose_frame
        when the node started.
    """

    def __init__(self):
        # Initialization
        rospy.init_node('odom_frame_publisher')
        # rosparam
        self.map_frame_id = rospy.get_param('~map_frame_id','map')
        self.t265_odom_frame_id = rospy.get_param('~t265_odom_frame_id','t265_odom_frame')
        self.t265_pose_frame_id = rospy.get_param('~t265_pose_frame_id','t265_pose_frame')
        self.internal_pose_frame_id = rospy.get_param('~internal_pose_frame_id','internal_t265_pose_frame')
        # Subscriber & Publisher
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.sub = rospy.Subscriber(
                '~input', Odometry, self._cb, queue_size=1)
        #
        self.trans_3 = None
        self.rot_3 = None
        # Others
        self.flag = True
        rospy.loginfo('Initialization finished')

    def spin(self):

        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.trans_3 is not None:
                self.broadcaster.sendTransform(
                        self.trans_3,
                        self.rot_3,
                        rospy.Time.now(),
                        self.t265_odom_frame_id,
                        self.map_frame_id )
            r.sleep()

    def _cb(self, msg):
        self.sub.unregister()
        if self.flag:
            self.flag = False

            tf_pose2odom = self.listener.lookupTransform(
                    self.t265_pose_frame_id,
                    self.t265_odom_frame_id,
                    rospy.Time(0))
            tf_map2pose = self.listener.lookupTransform(
                    self.map_frame_id,
                    self.internal_pose_frame_id,
                    rospy.Time(0))

            frame_pose2odom = pm.fromTf( tf_pose2odom )
            frame_map2pose = pm.fromTf( tf_map2pose )
            ( self.trans_3, self.rot_3 ) = pm.toTf( frame_map2pose * frame_pose2odom )

            rospy.loginfo('broadcasting tf')

if __name__=='__main__':
    odompub = OdomFramePublisher()
    odompub.spin()
