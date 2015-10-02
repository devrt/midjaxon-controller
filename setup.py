#!/usr/bin/env python

# setup script for MIDJAXON controllers
#   written by Yosuke Matsusaka

from hrpsys import rtm
import rospy
import os
import time
import subprocess

# requirement
# apt-get install ros-indigo-rtmros-common

# terminate rtcd on exit
plist = []

def terminator():
    global plist
    for p in plist:
        p.terminate()

import atexit
atexit.register(terminator)

# run rtcd to load components
plist.append(subprocess.Popen(["/usr/bin/rtcd", "-d"]))

# run ros bridges
bridges = ['ImageSensorROSBridge', 'PointCloudROSBridge', 'RangeSensorROSBridge']
#bridgepath = subprocess.check_output('rospack find hrpsys_ros_bridge', shell=True).strip()
bridgepath = os.path.expanduser('~/catkin_ws/install/lib/hrpsys_ros_bridge')
for b in bridges:
    plist.append(subprocess.Popen([os.path.join(bridgepath, b)]))


time.sleep(2)

#rtm.nshost=
rtm.nsport=2809
rtm.initCORBA()

mgr = rtm.findRTCmanager()
mgr.load('Joystick')
mgr.load('Joystick2PanTiltAngles')
mgr.load('MidJaxonController')

midjaxon = rtm.findRTC('MIDJAXON')
midjaxon = rtm.findRTC('ImageSensorROSBridgeComp')
js = mgr.create('Joystick')
jscrawl = mgr.create('Joystick2PanTiltAngles')
jscrawl.setProperty('axesIds', '0,1')
jshead = mgr.create('Joystick2PanTiltAngles')
jshead.setProperty('axesIds', '2,3')
midc = mgr.create('MidJaxonController')
#rtabmap = mgr.create('Rtabmap')

# connect between joystick and controller
rtm.connectPorts(js.port('Axes'), jscrawl.port('axes'))
rtm.connectPorts(js.port('Axes'), jshead.port('axes'))

# connect between controller and PD controller
rtm.connectPorts(midc.port('angles'), pdc.port('angleRef'))

# connect between PD controller and robot (this will be done by
# choreonoid BodyRTC conf file)
#rtm.connectPorts(midjaxon.port('u_out'), pdc.port('angle'))
#rtm.connectPorts(pdc.port('torque'), midjaxon.port('q'))

# connect between robot and mapper
#rtm.connectPorts(midjaxon.port('camera'), rtabmap.port('camera'))
#rtm.connectPorts(midc.port('odometry_camera'), pdc.port('odometry'))

from IPython import embed
embed()

terminator()
