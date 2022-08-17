#!/usr/bin/env python
# -*- coding: utf-8 -*-

import actionlib
import rospy

from std_msgs.msg import Empty
from power_msgs.msg import BatteryState
from sound_play.msg import SoundRequestAction, SoundRequestGoal

class UnchargedShutdown(object):

    def __init__(self):
        rospy.loginfo('Start uncharged_shutdown node.')
        self.client_jp = actionlib.SimpleActionClient('/robotsound_jp', SoundRequestAction)
        self.subscriber = rospy.Subscriber('/shutdown_unchecked', Empty, self.uncharged_shutdown)
        self.shutdown_pub = rospy.Publisher('/shutdown', Empty, queue_size=1)

    def uncharged_shutdown(self, msg):
        try:
            state = rospy.wait_for_message('/battery_state', BatteryState, timeout=2)
            if state.is_charging:
                rospy.loginfo("I received shutdown_unchecked, but don't shut down because I am charged.")
                self.speak(self.client_jp,
                            '充電されているのでシャットダウンはしません。',
                            'jp')
                return
        except rospy.ROSException as e:
            rospy.logerr("Timeout while waiting for battery_state")
            pass
        rospy.loginfo("I'm going to shut down to protect the battery.")
        self.speak(self.client_jp,
                   'バッテリ保護のためシャットダウンします。',
                   'jp')
        self.shutdown_pub.publish(Empty())

    def speak(self, client, speech_text, lang=None):
        client.wait_for_server(timeout=rospy.Duration(1.0))
        sound_goal = SoundRequestGoal()
        sound_goal.sound_request.sound = -3
        sound_goal.sound_request.command = 1
        sound_goal.sound_request.volume = 1.0
        if lang is not None:
            sound_goal.sound_request.arg2 = lang
        sound_goal.sound_request.arg = speech_text
        client.send_goal(sound_goal)
        client.wait_for_result()
        return client.get_result()

if __name__ == '__main__':
    rospy.init_node('uncharged_shutdown')
    us = UnchargedShutdown()
    rospy.spin()
