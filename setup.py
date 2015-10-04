#!/usr/bin/env python

# setup script for MIDJAXON controllers
#   written by Yosuke Matsusaka

import sys
newpath = []
rospath = []
for p in sys.path:
    if p.find('/opt/ros') == 0:
        print 'temporary remove ros related path %s' % p
        rospath.append(p)
    elif p.find('catkin_ws') >= 0:
        print 'temporary remove ros related path %s' % p
        rospath.append(p)
    else:
        newpath.append(p)
sys.path = newpath
        
import rtm
from hrpsys import OpenHRP
import os
import time
import subprocess
import copy

sys.path.extend(rospath)
import rospy

actual = [ 
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.349066,  0.000000, -1.396263, 
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.349066,  0.000000, -1.396263,  0.000000,  0.000000, 
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000, 
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000 ]

stand = [ 
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.349066,  0.000000, -1.396263, 
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.349066,  0.000000, -1.396263,  0.000000,  0.000000, 
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  1.200000, -1.200000,  1.20000,  -1.200000, 
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000 ]

bothflipup = [ 
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.349066,  0.000000, -1.396263, 
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -0.349066,  0.000000, -1.396263,  0.000000,  0.000000, 
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -1.400000,  1.400000, -1.400000,  1.400000, 
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000 ]

handswide = [ 
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  1.600000,  0.000000,  0.000000, 
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -1.600000,  0.000000,  0.000000,  0.000000,  0.000000, 
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000, 
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000 ]

maskflipper = [ 
    False,  False,  False,  False,  False,  False,  False,  False,  False,  False, 
    False,  False,  False,  False,  False,  False,  False,  False,  False,  False, 
    False,  False,  False,  False,  False,  False,  True,   True,   True,   True, 
    False,  False,  False,  False,  False,  False ]

maskcrawler = [ 
    False,  False,  False,  False,  False,  False,  False,  False,  False,  False, 
    False,  False,  False,  False,  False,  False,  False,  False,  False,  False, 
    False,  False,  False,  False,  False,  False,  False,  False,  False,  False, 
    True,   True,   True,   True,   True,   True ]

def goactual():
    seqsvc.setJointAngles(actual, 5)

def gosit():
    seqsvc.setJointAnglesWithMask(actual, maskflipper, 5)

def gostand():
    seqsvc.setJointAnglesWithMask(stand, maskflipper, 10)

def recover():
    seqsvc.setJointAngles(bothflipup, 3)
    seqsvc.waitInterpolation()
    seqsvc.setJointAngles(handswide, 7)
    seqsvc.waitInterpolation()
    goactual()
    
def setcrawler(lvel, rvel):
    v = copy.deepcopy(actual)
    v[30] = v[31] = v[32] = lvel
    v[33] = v[34] = v[35] = rvel
    seqsvc.setJointAnglesWithMask(v, maskcrawler, 1)
    
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
#plist.append(subprocess.Popen(["/usr/bin/rtcd", "-d"]))

time.sleep(2)

#rtm.nshost=
rtm.nsport=2809
rtm.initCORBA()

mgr = rtm.findRTCmanager()
mgr.load('SequencePlayer')
mgr.load('StateHolder')
mgr.load('ForwardKinematics')
mgr.load('Joystick')
mgr.load('Joystick2PanTiltAngles')
mgr.load('MidJaxonController')

#ns2 = rtm.orb.string_to_object('corbaloc:iiop:%s:%s/NameService' % (nshost, nsport))
#nc2 = ns2._narrow(CosNaming.NamingContext)

midjaxon = rtm.findRTC('MIDJAXON')
rh = rtm.findRTC('PDcontroller0')

seq = mgr.create('SequencePlayer')
seqsvc = rtm.narrow(seq.service('service0'), 'SequencePlayerService', 'hrpsys.OpenHRP')
sh = mgr.create('StateHolder')
fk = mgr.create('ForwardKinematics')

js = mgr.create('Joystick')
js.setProperty('device', '/dev/input/js0')
#js.setProperty('debugLevel', '1')
midc = mgr.create('MidJaxonController')
midc.setProperty('debugLevel', '1')

rtm.connectPorts(sh.port("qOut"), rh.port("angleRef"))
rtm.connectPorts(midjaxon.port("q"), [sh.port("currentQIn"),
                                      fk.port("q"),
                                      midc.port("q")])  # connection for actual joint angles
rtm.connectPorts(sh.port("qOut"), fk.port("qRef"))
rtm.connectPorts(seq.port("qRef"), midc.port("qUpstream"))
rtm.connectPorts(midc.port("qRef"), sh.port("qIn"))
#rtm.connectPorts(seq.port("qRef"), sh.port("qIn"))
rtm.connectPorts(seq.port("zmpRef"), sh.port("zmpIn"))
rtm.connectPorts(seq.port("optionalData"), sh.port("optionalDataIn"))
rtm.connectPorts(sh.port("basePosOut"), [seq.port("basePosInit"),
                                         fk.port("basePosRef")])
rtm.connectPorts(sh.port("baseRpyOut"), [seq.port("baseRpyInit"),
                                         fk.port("baseRpyRef")])
rtm.connectPorts(sh.port("qOut"), seq.port("qInit"))
rtm.connectPorts(sh.port("zmpOut"), seq.port("zmpRefInit"))

# connect between joystick and controller
rtm.connectPorts(js.port('Axes'), midc.port('axes'))
rtm.connectPorts(js.port('Buttons'), midc.port('buttons'))

# connect between controller and PD controller
#rtm.connectPorts(rh.port('angles'), pdc.port('angleRef'))

js.start()
midc.start()
fk.start()
seq.start()
sh.start()

seqsvc.setJointAngles(actual, 0.01)
    
from IPython import embed
embed()

terminator()
