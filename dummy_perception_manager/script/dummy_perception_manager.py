#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import rospy
from std_msgs.msg import String
from audio_msgs.msg import AudioData
import pyaudio
from six.moves import queue
import os
import sounddevice as sd


def main():
    buff = queue.Queue()
    rospy.init_node('dummy_percpetion_manager', anonymous=None)
    global pub
    rospy.Subscriber("sound_information", String, queue_size=1)
    rospy.spin()


def packetCallback(packetData, buff):
    global callerSpeech, idx, callerArray, frameIdx, endFlag, lastFrameIdx
    callerSpeech = packetData.data
    byte_str = makeByteStr(callerSpeech)
    buff.put(byte_str)

if __name__ == '__main__':
    main()

