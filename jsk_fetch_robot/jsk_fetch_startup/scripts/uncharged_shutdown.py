#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from power_msgs.msg import BatteryState

class UnchargedShutdown(object):

    def __init__(self):
        rospy.loginfo('Start uncharged_shutdown node.')
        self.subscriber = rospy.Subscriber('/shutdown_unchecked', Empty, self.uncharged_shutdown)
        self.shutdown_pub = rospy.Publisher('/shutdown', Empty, queue_size=1)

    def uncharged_shutdown(self, msg):
        try:
            state = rospy.wait_for_message('/battery_state', BatteryState, timeout=2)
            if state.is_charging:
                return
        except rospy.ROSException as e:
            rospy.logerr("Timeout while waiting for /battery_state")
            pass
        self.shutdown_pub.publish(Empty())

if __name__ == '__main__':
    rospy.init_node('uncharged_shutdown')
    us = UnchargedShutdown()
    rospy.spin()
