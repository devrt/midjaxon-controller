#!/usr/bin/env python

# setup script for MIDJAXON controllers
#   written by Yosuke Matsusaka

import sys
import math

# temporary disable ros path while importing hrpsys
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
import OpenHRP as OpenHRPOrigin
import os
import time
import subprocess
import copy
import CosNaming

# now, its your time ros
sys.path.extend(rospath)
import roslib; roslib.load_manifest("interactive_markers")
import rospy
from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster

ROBOT_URL = "file:///home/yosuke/catkin_ws/src/rtm-ros-robotics/rtmros_choreonoid/jvrc_models/JAXON_JVRC/MIDJAXON-no-surface.wrl"

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

handsupflipup = [ 
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  math.pi,   0.000000,  0.000000, 
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -math.pi,   0.000000,  0.000000,  0.000000,  0.000000, 
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -1.400000,  1.400000, -1.400000,  1.400000, 
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000 ]

handswide = [
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  1.200000,  0.000000,  0.000000,
    0.000000,  0.000000,  0.000000,  0.000000,  0.000000, -1.200000,  0.000000,  0.000000,  0.000000,  0.000000,
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
    autobalanceoff()
    seqsvc.setJointAngles(handsupflipup, 7)
    seqsvc.waitInterpolation()
    seqsvc.setJointAngles(handsupflipup, 3)
    seqsvc.waitInterpolation()
    seqsvc.setJointAngles(handswide, 12)
    #seqsvc.waitInterpolation()
    #seqsvc.setJointAngles(handswide, 12)
    #seqsvc.waitInterpolation()
    #goactual()
    
def setcrawler(lvel, rvel):
    v = copy.deepcopy(actual)
    v[30] = v[31] = v[32] = lvel
    v[33] = v[34] = v[35] = rvel
    seqsvc.setJointAnglesWithMask(v, maskcrawler, 1)

def autobalanceon():
    midc.setProperty('autobalance', '1')

def autobalanceoff():
    midc.setProperty('autobalance', '0')

def processFeedback(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.loginfo( s + ": pose changed")
    server.applyChanges()

def alignMarker( feedback ):
    pose = feedback.pose
    pose.position.x = round(pose.position.x-0.5)+0.5
    pose.position.y = round(pose.position.y-0.5)+0.5
    server.setPose( feedback.marker_name, pose )
    server.applyChanges()

def make6DofMarker(position):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/BODY"
    int_marker.pose.position = position
    int_marker.scale = 1
    int_marker.name = "simple_6dof"
    int_marker.description = "Simple 6-DOF Control"
    
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    return int_marker

def showmarker():
    server.insert(imarker, processFeedback)
    server.applyChanges()

def hidemarker():
    server.clear()
    server.applyChanges()

def updateFK():
    j = shsvc.getCommand().jointRefs
    planner.setCharacterAllLinkData("robot", OpenHRPOrigin.DynamicsSimulator.JOINT_VALUE, j)
    planner.calcCharacterForwardKinematics("robot")

def movemarker():
    pass
    
def solveIK(base, target, pose):
    updateK()
    ldata = planner.getCharacterLinkData("robot", target, OpenHRPOrigin.DynamicsSimulator.ABS_TRANSFORM)
    lp = OpenHRP.LinkPosition(None, None)
    lp.p = ldata[0:3]
    lp.R = ldata[3:12]
    tf.transformations.quaternion_matrix([int_marker.pose.orientation.x, int_marker.pose.orientation.y, int_marker.pose.orientation.z, int_marker.pose.orientation.w])
    planner.calcCharacterInverseKinematics("robot", "BASE", "LARM_JOINT7", lp)
    return planner.getCharacterAllLinkData("robot", OpenHRPOrigin.DynamicsSimulator.JOINT_VALUE)

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
plist.append(subprocess.Popen(["/usr/bin/openhrp-aist-dynamics-simulator"]))
plist.append(subprocess.Popen(["/usr/bin/openhrp-collision-detector"]))

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

nshost2 = "127.0.0.1"
nsport2 = "2809"
ns2 = rtm.orb.string_to_object('corbaloc:iiop:%s:%s/NameService' % (nshost2, nsport2))
nc2 = ns2._narrow(CosNaming.NamingContext)

midjaxon = rtm.findRTC('MIDJAXON', nc2)
rh = rtm.findRTC('PDcontroller0', nc2)

rtm.rootnc.rebind([CosNaming.NameComponent('MIDJAXON', 'rtc')], midjaxon.ref)
rtm.rootnc.rebind([CosNaming.NameComponent('PDcontroller0', 'rtc')], rh.ref)

seq = mgr.create('SequencePlayer', 'seq')
seqsvc = rtm.narrow(seq.service('service0'), 'SequencePlayerService', 'hrpsys.OpenHRP')
sh = mgr.create('StateHolder', 'sh')
shsvc = rtm.narrow(sh.service('service0'), 'StateHolderService', 'hrpsys.OpenHRP')
fk = mgr.create('ForwardKinematics', 'fk')

js = mgr.create('Joystick', 'js')
js.setProperty('device', '/dev/input/js0')
#js.setProperty('debugLevel', '1')
midc = mgr.create('MidJaxonController', 'midc')
#midc.setProperty('debugLevel', '1')
midc.setProperty('autobalance', '1')

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

# connect gsensor for auto balancing
rtm.connectPorts(midjaxon.port('gsensor'), midc.port('gsensor'))

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

time.sleep(3)
modelloader = rtm.findObject('ModelLoader')
simfactory = rtm.findObject('DynamicsSimulatorFactory')
planner = simfactory.create()
robot = modelloader.loadBodyInfo(ROBOT_URL)
planner.registerCharacter("robot", robot)
planner.init(0.001,
             OpenHRPOrigin.DynamicsSimulator.RUNGE_KUTTA,
             OpenHRPOrigin.DynamicsSimulator.DISABLE_SENSOR)
planner.setGVector([0, 0, 9.8])
planner.setCharacterAllJointModes("robot", OpenHRPOrigin.DynamicsSimulator.HIGH_GAIN_MODE)

rospy.init_node("basic_controls")
server = InteractiveMarkerServer("basic_controls")
position = Point( 0, 0, 0)
imarker = make6DofMarker(position)
server.insert(imarker, processFeedback)
server.applyChanges()
#rospy.spin()

from IPython import embed
embed()

terminator()
