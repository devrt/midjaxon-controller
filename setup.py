#!/usr/bin/env python

# setup script for MIDJAXON controllers
#
# Copyright (c) 2015, Yosuke Matsusaka (MID Academic Promotions Inc.)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
#modification, are permitted provided that the following conditions
#are met:
#
# 1. Redistributions of source code must retain the above copyright
#notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
#notice, this list of conditions and the following disclaimer in the
#documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#contributors may be used to endorse or promote products derived from
#this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
#OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
#AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
#WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#POSSIBILITY OF SUCH DAMAGE.

import sys

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
        
import math
import numpy
import rtm
import RTM
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
from geometry_msgs.msg import Pose
from tf.broadcaster import TransformBroadcaster
from tf.listener import TransformListener
import tf.transformations as tft

JVRC_MODELS_PATH = subprocess.check_output('rospack find jvrc_models', shell=True).strip()
JVRC_BRIDGE_PATH = subprocess.check_output('rospack find hrpsys_ros_bridge_jvrc', shell=True).strip()
ROBOT_URL = "file://" + os.path.join(JVRC_MODELS_PATH, "JAXON_JVRC/MIDJAXON-no-surface.wrl")

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

maskleftarm = [ 
    False,  False,  False,  False,  False,  True,   True,   True,   True,   True, 
    True,   True,   True,   False,  False,  False,  False,  False,  False,  False, 
    False,  False,  False,  False,  False,  False,  False,  False,  False,  False, 
    False,  False,  False,  False,  False,  False ]

maskrightarm = [ 
    False,  False,  False,  False,  False,  False,  False,  False,  False,  False, 
    False,  False,  False,  True,   True,    True,   True,   True,   True,   True, 
     True,  False,  False,  False,  False,  False,  False,  False,  False,  False, 
    False,  False,  False,  False,  False,  False ]

maskleftfinger = [ 
    False,  False,  False,  False,  False,  False,  False,  False,  False,  False, 
    False,  False,  False,  False,  False,  False,  False,  False,  False,  False, 
    False,  True,   True,   False,  False,  False,  False,  False,  False,  False, 
    False,  False,  False,  False,  False,  False ]

maskrightfinger = [ 
    False,  False,  False,  False,  False,  False,  False,  False,  False,  False, 
    False,  False,  False,  False,  False,  False,  False,  False,  False,  False, 
    False,  False,  False,  True,   True,   False,  False,  False,  False,  False, 
    False,  False,  False,  False,  False,  False ]

def goactual(t = 5):
    seqsvc.setJointAngles(actual, t)

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

def headyaw(v, t = 2):
    seqsvc.setJointAngle("HEAD_JOINT0", v, t)

def autobalanceon():
    midc.setProperty('autobalance', '1')

def autobalanceoff():
    midc.setProperty('autobalance', '0')

def processFeedback(feedback):
    global pose
    if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.loginfo("control target pose changed")
        pose = feedback.pose
        print "controltarget: %s" % controltarget
        if controltarget:
            updateFK()
            r = solveIK("CHEST_JOINT2", controltarget, pose)
            if controltarget.find("LARM") >= 0:
                seqsvc.setJointAnglesWithMask(r, maskleftarm, 3)
            if controltarget.find("RARM") >= 0:
                seqsvc.setJointAnglesWithMask(r, maskrightarm, 3)

def processWorkMarkerFeedback(feedback):
    global workmarkerpose
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        workmarkerpose = feedback.pose

def processWorkMarker2Feedback(feedback):
    global workmarker2pose
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        workmarker2pose = feedback.pose

def addpose(pose1, pose2):
    return [pose1.position.x + pose2.position.x,
            pose1.position.y + pose2.position.y,
            pose1.position.z + pose2.position.z]

def subpose(pose1, pose2):
    return [pose1.position.x - pose2.position.x,
            pose1.position.y - pose2.position.y,
            pose1.position.z - pose2.position.z]

def make6DofMarker(position):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/BODY"
    int_marker.pose.position = position
    int_marker.scale = 1
    int_marker.name = "simple_6dof"
    int_marker.description = "ControlTarget"
    
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

def makeXYZMarker(position):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/BODY"
    int_marker.pose.position = position
    int_marker.scale = 1
    int_marker.name = "WorkMarker"
    int_marker.description = "WorkMarker"
    
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
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
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
    global controltarget
    controltarget = None
    server.erase(imarker.name)
    server.applyChanges()

def hrp2pose(hpose):
    pose = Pose()
    pose.position.x = hpose[0]
    pose.position.y = hpose[1]
    pose.position.z = hpose[2]
    m = numpy.zeros([4,4])
    m[0,0] = hpose[3]
    m[0,1] = hpose[4]
    m[0,2] = hpose[5]
    m[1,0] = hpose[6]
    m[1,1] = hpose[7]
    m[1,2] = hpose[8]
    m[2,0] = hpose[9]
    m[2,1] = hpose[10]
    m[2,2] = hpose[11]
    m[3,3] = 1.0
    quat = tft.quaternion_from_matrix(m)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose

def pose2hrp(pose):
    lp = OpenHRPOrigin.LinkPosition(None, None)
    lp.p = [pose.position.x, pose.position.y, pose.position.z]
    lp.R = tft.quaternion_matrix([pose.orientation.x,
                                  pose.orientation.y,
                                  pose.orientation.z,
                                  pose.orientation.w])[:3,:3].reshape(1,9)[0].tolist()
    return lp

def pose2jvrc(pose):
    return [-pose.position.y * 1000,
            (pose.position.x - 1.0) * 1000,
            (pose.position.z + 0.238) * 1000]

def leftcontrol():
    global controltarget
    updateFK()
    pose = hrp2pose(planner.getCharacterLinkData('robot', 'LARM_JOINT7', OpenHRP.DynamicsSimulator.ABS_TRANSFORM))
    showmarker()
    server.setPose(imarker.name, pose)
    server.applyChanges()
    controltarget = 'LARM_JOINT7'

def rightcontrol():
    global controltarget
    updateFK()
    pose = hrp2pose(planner.getCharacterLinkData('robot', 'RARM_JOINT7', OpenHRP.DynamicsSimulator.ABS_TRANSFORM))
    showmarker()
    server.setPose(imarker.name, pose)
    server.applyChanges()
    controltarget = 'RARM_JOINT7'

def leftgrasp(v):
    j = shsvc.getCommand().jointRefs
    j[21] = -v
    j[22] = v
    seqsvc.setJointAnglesWithMask(j, maskleftfinger, 3)
    
def rightgrasp(v):
    j = shsvc.getCommand().jointRefs
    j[23] = -v
    j[24] = v
    seqsvc.setJointAnglesWithMask(j, maskrightfinger, 3)
    
def updateFK():
    j = shsvc.getCommand().jointRefs
    planner.setCharacterAllLinkData("robot", OpenHRPOrigin.DynamicsSimulator.JOINT_VALUE, j)
    planner.calcCharacterForwardKinematics("robot")

def solveIK(base, target, pose):
    lp = pose2hrp(pose)
    planner.calcCharacterInverseKinematics("robot", "CHEST_JOINT2", target, lp)
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

practicemode = False
if len(sys.argv) > 1:
    print "--------------------practice mode------------------"
    practicemode = True
else:
    print "-------------------competition mode-----------------"

# run rtcd to load components
if practicemode == False:
    plist.append(subprocess.Popen(["/usr/bin/rtcd", "-d"]))
else:
    subprocess.check_output('sed -i "s/^model:.*/model: ' + ROBOT_URL.replace('/', '\\/') + '/g" rtc.conf', shell=True)
    plist.append(subprocess.Popen(["/usr/bin/choreonoid", os.path.join(JVRC_BRIDGE_PATH, 'config/midjaxon_pd'+sys.argv[1]+'.cnoid'), '--start-simulation']))
    time.sleep(5)
plist.append(subprocess.Popen(["/usr/bin/openhrp-model-loader"]))
plist.append(subprocess.Popen(["/usr/bin/openhrp-aist-dynamics-simulator"]))
plist.append(subprocess.Popen(["/usr/bin/openhrp-collision-detector"]))

time.sleep(2)

rtm.nshost='localhost'
rtm.nsport=2809
rtm.initCORBA()

# local manager
mgr = rtm.findRTCmanager()

# manager on simulation server
nshost2 = "10.1.4.20"
if practicemode:
    nshost2 = "localhost"
nsport2 = "2809"
ns2 = rtm.orb.string_to_object('corbaloc:iiop:%s:%s/NameService' % (nshost2, nsport2))
nc2 = ns2._narrow(CosNaming.NamingContext)
if practicemode:
    mgr2 = mgr
else:
    mgr2obj = rtm.orb.string_to_object('corbaloc:iiop:%s:2810/manager' % (nshost2))
    mgr2 = rtm.RTCmanager(mgr2obj._narrow(RTM.Manager))

mgr2.load('SequencePlayer')
mgr2.load('StateHolder')
mgr2.load('ForwardKinematics')
mgr2.load('Joystick')
mgr2.load('Joystick2PanTiltAngles')
mgr2.load('MidJaxonController')

midjaxon = rtm.findRTC('MIDJAXON', nc2)
rh = rtm.findRTC('PDcontrollerMIDJAXON0', nc2)
seq = mgr2.create('SequencePlayer', 'seq')
seqsvc = rtm.narrow(seq.service('service0'), 'SequencePlayerService', 'hrpsys.OpenHRP')
sh = mgr2.create('StateHolder', 'sh')
shsvc = rtm.narrow(sh.service('service0'), 'StateHolderService', 'hrpsys.OpenHRP')
fk = mgr2.create('ForwardKinematics', 'fk')

rtm.rootnc.rebind([CosNaming.NameComponent('MIDJAXON', 'rtc')], midjaxon.ref)
rtm.rootnc.rebind([CosNaming.NameComponent('PDcontrollerMIDJAXON0', 'rtc')], rh.ref)
rtm.rootnc.rebind([CosNaming.NameComponent('seq', 'rtc')], seq.ref)
rtm.rootnc.rebind([CosNaming.NameComponent('sh', 'rtc')], sh.ref)
rtm.rootnc.rebind([CosNaming.NameComponent('fk', 'rtc')], fk.ref)

js = mgr.create('Joystick', 'js')
js.setProperty('device', '/dev/input/js0')
#js.setProperty('debugLevel', '1')
midc = mgr2.create('MidJaxonController', 'midc')
#midc.setProperty('debugLevel', '1')
midc.setProperty('autobalance', '1')
rtm.rootnc.rebind([CosNaming.NameComponent('midc', 'rtc')], midc.ref)

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

modelloader = rtm.findObject('ModelLoader')
robot = modelloader.loadBodyInfo(ROBOT_URL)
simfactory = rtm.findObject('DynamicsSimulatorFactory')
planner = simfactory.create()
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
workmarker1 = makeXYZMarker(Point(0, 1.5, 0))
workmarker1.name = "Work1"
workmarker1.description = "Work1"
workmarker2 = makeXYZMarker(Point(0, -1.5, 0))
workmarker2.name = "Work2"
workmarker2.description = "Work2"
server.insert(imarker, processFeedback)
server.insert(workmarker1, processWorkMarkerFeedback)
server.insert(workmarker2, processWorkMarker2Feedback)
server.applyChanges()
#rospy.spin()

hidemarker()

from IPython import embed
embed()

terminator()
