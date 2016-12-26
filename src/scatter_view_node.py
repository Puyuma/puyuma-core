#!/usr/bin/env python

import rospy
import pyqtgraph as pg
import numpy as np
from xenobot.msg import segment
from xenobot.msg import segmentArray
from pyqtgraph.Qt import QtGui
from PyQt4.QtCore import QObject, pyqtSignal
import signal
import sys

plotWidget = None
app = None
d_y = []
phi_y = []
d_w = []
phi_w = []
foo = None

class Foo(QObject):
	trigger = pyqtSignal()
	def c(self):
		self.trigger.connect(self.handler)
	def e(self):
		self.trigger.emit()
	def handler(self):
		plotWidget.clear()
		plotWidget.plot(d_w,phi_w,pen=None,symbol='x',symbolBrush=pg.mkBrush('r'))
		plotWidget.plot(d_y,phi_y,pen=None,symbol='x',symnolbrush=pg.mkBrush('w'))

def sig_INT_handler(signal, frame):
	print('You pressed Ctrl+C!')
	app.quit()

def data_cb(msg):
	global d_y,d_w,phi_y,phi_w

	size = len(msg.segments)
	rospy.loginfo("get %d data",size)
	d_y = []
	d_w = []
	phi_y = []
	phi_w = []

	for seg in msg.segments:
		if(seg.color == 1): #yellow
			d_y.append(seg.d)
			phi_y.append(seg.phi)
		elif (seg.color == 0): #white
			d_w.append(seg.d)
			phi_w.append(seg.phi)
		else:
			rospy.loginfo("invalid color code")

	foo.e()

def main():
	global plotWidget
	global app
	global trigger
	global foo

	app = QtGui.QApplication([])
	foo = Foo()
	rospy.init_node('scatter_view_node',anonymous=True)

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
