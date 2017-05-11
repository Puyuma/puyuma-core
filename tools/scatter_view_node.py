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
g_segs = None

## Should be synchronize with system/lane_detector.hpp
### Colors
WHITE = 0
YELLOW = 1
RED = 2
RESULT = 3
UNKNOWN_COLOR = 4
COLORSIZE = 5

### Edges
LEFT_EDGE = 0
RIGHT_EDGE = 1
UNKNOWN_SIDE = 2
DELETED = 3
EDGESIZE = 4
##

colors =  [ 0 for y in range(COLORSIZE)]
colors[WHITE] = (0,0,255)
colors[YELLOW] = (255,255,0)
colors[RED] = (255,0,0)
colors[RESULT] = (0,255,0)
colors[UNKNOWN_COLOR] = (100,100,100)

symbols = [0 for x in range(EDGESIZE)]
symbols[LEFT_EDGE] = 'o'
symbols[RIGHT_EDGE] = 's'
symbols[UNKNOWN_SIDE] = 't'
symbols[DELETED] = 'x'

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

			for seg in g_segs:
				plotWidget.plot([seg.d],[seg.phi],pen=None,symbol=symbols[seg.side],symbolBrush=pg.mkBrush(colors[seg.color]))

def sig_INT_handler(signal, frame):
	print('You pressed Ctrl+C!')
	app.quit()

def data_cb(msg):
	global d, phi, g_segs

	size = len(msg.segments)
	rospy.loginfo("get %d data",size)
	d = []
	phi = []

	g_segs = msg.segments

#	for i in range(n):
#		d_i = []
#		phi_i = []
#		d.append(d_i)
#		phi.append(phi_i)

#	for seg in msg.segments:
#		if(seg.color < n):
#			d[seg.color].append(seg.d)
#			phi[seg.color].append(seg.phi)
#		else:
#			rospy.loginfo("invalid color code")

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

	plotItem.scene().sigMouseClicked.connect(mouseClick)


	app.exec_()



if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
