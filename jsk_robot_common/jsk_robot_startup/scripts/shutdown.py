#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
import actionlib
from std_msgs.msg import Empty
from sound_play.msg import SoundRequestAction, SoundRequestGoal

class Shutdown(object):
    """
    This node shuts down or reboots the robot itself
    according to the rostopic.

    Note that this node needs to be run with sudo privileges.

    Usage:
    # Launch node
    $ su [sudo user] -c ". [setup.bash]; rosrun jsk_robot_startup shutdown.py"

    # To shutdown robot
    rostopic pub /shutdown std_msgs/Empty
    # To restart robot
    rostopic pub /reboot std_msgs/Empty
    """

    def __init__(self):
        rospy.loginfo('Start shutdown node.')
        self.client_jp = actionlib.SimpleActionClient('/robotsound_jp', SoundRequestAction)
        rospy.Subscriber('shutdown', Empty, self.shutdown)
        rospy.Subscriber('reboot', Empty, self.reboot)
        self.shutdown_command = rospy.get_param(
            '~shutdown_command', '/sbin/shutdown -h now')
        self.reboot_command = rospy.get_param(
            '~reboot_command', '/sbin/shutdown -r now')

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

    def shutdown(self, msg):
        rospy.loginfo('Shut down robot.')
        self.speak(self.client_jp, 'シャットダウンします。', 'jp')
        ret = os.system(self.shutdown_command)
        if ret != 0:
            rospy.logerr("Failed to call '$ {}'. Check authentication.".format(
                self.shutdown_command))

    def reboot(self, msg):
        rospy.loginfo('Reboot robot.')
        self.speak(self.client_jp, '再起動します。', 'jp')
        ret = os.system(self.reboot_command)
        if ret != 0:
            rospy.logerr("Failed to call '$ {}'. Check authentication.".format(
                self.reboot_command))


if __name__ == '__main__':
    rospy.init_node('shutdown')
    s = Shutdown()
    rospy.spin()
