#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from power_msgs.msg import BatteryState

class SaveBattery(object):

    def __init__(self):
        rospy.loginfo('Start save_battery node.')
        self.timeout = 1
        self.subscriber = rospy.Subscriber('/save_battery', Empty, self.save_battery)
        self.shutdown_pub = rospy.Publisher('/shutdown', Empty, queue_size=1)

    def save_battery(self, msg):
        try:
            state = rospy.wait_for_message('/battery_state', BatteryState, timeout=self.timeout)
            if state.is_charging:
                return
        except:
            pass
        self.shutdown_pub.publish(Empty())

if __name__ == '__main__':
    rospy.init_node('save_battery')
    sb = SaveBattery()
    rospy.spin()
