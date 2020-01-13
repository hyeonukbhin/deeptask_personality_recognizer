#!/usr/bin/python2
# -*- coding: utf-8 -*-
from __future__ import division

import numpy as np
import parselmouth
import rospy
from audio_msgs.msg import AudioData, FeatureData
from std_msgs.msg import String
import sounddevice as sd


class testNode:
    # Calibration pair distance parameter
    def __init__(self):
        rospy.init_node("KIST_test_node", anonymous=False)
        rospy.Subscriber("input_dummy_json", String, self.callback_packet, queue_size=50)
        self.pub = rospy.Publisher("test_output", FeatureData, queue_size=50)

        self.wav_array = np.array([])
        self.voice_feature = FeatureData()
        rospy.spin()

    def callback_packet(self, topic):
        data = topic.data
        self.set_param("parameter/robot_speech", data)

    def set_param(self, input_key="parameter/name", input_value="유정"):
        rospy.set_param(input_key, input_value)


if __name__ == '__main__':
    try:
        testNode()
    except rospy.ROSInterruptException:
        pass