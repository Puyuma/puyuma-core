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
d = []
phi = []
colors=  ['w','y','r']
## Color Table ##
#	0	white	#
#	1	yellow	#
#	2	red		#
#				#
################

n = len(colors)
pause_state = False
foo = None

class Foo(QObject):
	trigger = pyqtSignal()
	def c(self):
		self.trigger.connect(self.handler)
	def e(self):
		self.trigger.emit()
	def handler(self):
		if(pause_state == False):
			plotWidget.clear()

			for i in range(n):
				plotWidget.plot(d[i],phi[i],pen=None,symbol='x',symbolBrush=pg.mkBrush(colors[i]))

def sig_INT_handler(signal, frame):
	print('You pressed Ctrl+C!')
	app.quit()

def data_cb(msg):
	global d,phi

	size = len(msg.segments)
	rospy.loginfo("get %d data",size)
	d = []
	phi = []

	for i in range(n):
		d_i = []
		phi_i = []
		d.append(d_i)
		phi.append(phi_i)

	for seg in msg.segments:
		if(seg.color < n):
			d[seg.color].append(seg.d)
			phi[seg.color].append(seg.phi)
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
	topic_name = rospy.get_param('~veh')
	topic_name = topic_name + "/segment_data"
	rospy.Subscriber(topic_name, segmentArray, data_cb)
	signal.signal(signal.SIGINT, sig_INT_handler)
	foo.c()

	rospy.loginfo("Start listen to %s",topic_name)
	plotWidget = pg.plot(title="Scatter plot",labels={'left': ("phi"), 'bottom': ("d")} )

	dRange=(-25,25)
	phiRange=(-90,90)

	view = plotWidget.getViewBox()
	view.setRange(xRange=dRange, yRange=phiRange)

	plotItem = plotWidget.getPlotItem()

	def mouseClick(evt):
		global pause_state
		print "click"
		pause_state = not pause_state

	#proxy = pg.SignalProxy(plotItem.scene().sigMouseMoved, rateLimit=60, slot=mouseMoved)
	plotItem.scene().sigMouseClicked.connect(mouseClick)


	app.exec_()



if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
