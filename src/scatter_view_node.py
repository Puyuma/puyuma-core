#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import pyqtgraph as pg
import numpy as np
from scatter_view.msg import segment
from scatter_view.msg import segmentArray
from pyqtgraph.Qt import QtGui
from PyQt4.QtCore import QObject, pyqtSignal
import signal
import sys

plotWidget = None
app = None
d = []
phi = []
foo = None

class Foo(QObject):
	trigger = pyqtSignal()
	def c(self):
		self.trigger.connect(self.handler)
	def e(self):
		self.trigger.emit()
	def handler(self):
		plotWidget.clear()
		plotWidget.plot(d,phi,pen=None,symbol='x')

def sig_INT_handler(signal, frame):
	print('You pressed Ctrl+C!')
	app.quit()

def data_cb(msg):
	global d,phi
	size = len(msg.segments)
	rospy.loginfo("get %d data",size)
	d = []
	phi = []
	for seg in msg.segments:
		d.append(seg.d)
		phi.append(seg.phi)
	foo.e()

def main():
	global plotWidget
	global app
	global trigger
	global foo

	app = QtGui.QApplication([])
	foo = Foo()
	rospy.init_node('talker',anonymous=True)

	topic_name = "/xenobot/segment_data"
	rospy.Subscriber(topic_name, segmentArray, data_cb)
	signal.signal(signal.SIGINT, sig_INT_handler)
	foo.c()

	rospy.loginfo("Start listen to %s",topic_name)
	plotWidget = pg.plot(title="Scatter plot",labels={'left': ("phi"), 'bottom': ("d")} )

	dRange=(-25,25)
	phiRange=(0,180)

	view = plotWidget.getViewBox()
	view.setRange(xRange=dRange, yRange=phiRange)

	app.exec_()



if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
