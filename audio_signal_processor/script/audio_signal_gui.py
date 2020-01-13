#!/usr/bin/python
# -*- coding: utf-8 -*-
import collections
from datetime import datetime

import numpy as np
import pyqtgraph as pg
import rospy
from empatica_e4_msgs.msg import DataArrays
from pyqtgraph.Qt import QtGui, QtCore
from audio_msgs.msg import AudioData
from six.moves import queue

import csv
import json
import yaml

str_utc_time = ""
bvp = []
eda = []
skin_temp = []
ibi = []
hr = []
acc_x = []
acc_y = []
acc_z = []



class AudioSignalGui:
    # Calibration pair distance parameter
    def __init__(self):
        rospy.init_node('audio_signal_gui', anonymous=None)
        buff = queue.Queue()
        rospy.Subscriber("audio_stream", AudioData, self.callback, buff)

        INTERVAL_PLOTTING = 0.25
        TIMEWINDOW = 10.
        GUISIZE_X = 1800
        GUISIZE_Y = 800
        DATASIZE_BVP = 30

        m = DynamicPlotter(INTERVAL_PLOTTING, TIMEWINDOW, GUISIZE_X, GUISIZE_Y, DATASIZE_BVP)
        m.run()
        rospy.spin()

    def callback(self, packetData, buff):
        global bvp
        callerSpeech = packetData.data
        byte_str = self.makeByteStr(callerSpeech)
        buff.put(byte_str)
        bvp = np.fromstring(buff.get(), dtype=np.int16).astype(np.float64)  / 32768
        # print(bvp)
        # print(type(bvp_test[0]))
        # bvp = data.data

    def makeByteStr(self, int16Array):
        byte_str = "".join(map(chr, int16Array))
        return byte_str


class DynamicPlotter():
    def __init__(self, sampleinterval, timewindow, GUISIZE_X, GUISIZE_Y, datasize_bvp):
        # Data stuff
        self.data_comp = ""

        self._interval = int(sampleinterval * 1000)
        self._bufsize = int(timewindow / sampleinterval)

        self.databuffer_timestamp = collections.deque([0.0] * self._bufsize, self._bufsize)
        self.databuffer_bvp = collections.deque([0.0] * self._bufsize * datasize_bvp, self._bufsize * datasize_bvp)

        self.x = np.linspace(-timewindow, 0.0, self._bufsize)
        self.x_bvp = np.linspace(-timewindow, 0.0, self._bufsize * datasize_bvp)

        self.y_bvp = np.zeros(self._bufsize * datasize_bvp, dtype=np.float)

        # PyQtGraph stuff
        self.app = QtGui.QApplication([])

        self.win = pg.GraphicsLayoutWidget(show=True)
        self.win.resize(GUISIZE_X, GUISIZE_Y)
        self.win.setWindowTitle('Empatica E4 Data Plot')
        pg.setConfigOptions(antialias=True)
        self.win.ci.setBorder((50, 50, 100))

        # p1 setting
        self.p1 = self.win.addPlot(title="Photoplethysmography (PPG, Unit = bvp)")
        self.p1.showGrid(x=True, y=True)
        self.p1.setLabel('left', 'BVP', 'nW')
        self.p1.setLabel('bottom', 'time', 's')
        self.p1.setRange(yRange=[-0.1, 0.1])
        # self.p1.hideAxis("bottom")
        self.curve_bvp = self.p1.plot(self.x_bvp, self.y_bvp, pen="r")

        # QTimer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updateplot)
        self.timer.start(self._interval)
        if rospy.is_shutdown():
            self.app.quit()

    def updateplot(self):
        global str_utc_time, bvp, eda, skin_temp, ibi, hr, acc_x, acc_y, acc_z

        # if self.data_comp == str_utc_time:
        #     bvp = np.zeros(16, dtype=np.float)
        #
        # else:
        #     self.data_comp = str_utc_time

        self.databuffer_bvp.extend(bvp)

        self.y_bvp[:] = self.databuffer_bvp
        self.curve_bvp.setData(self.x_bvp, self.y_bvp)
        print(self.y_bvp)
        self.app.processEvents()

        if rospy.is_shutdown():
            self.app.quit()

    def run(self):
        self.app.exec_()


if __name__ == '__main__':
    AudioSignalGui()
