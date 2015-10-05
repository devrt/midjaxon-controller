#!/usr/bin/env python

# setup script for MIDJAXON controllers
#   written by Yosuke Matsusaka

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
        
import rtm
from hrpsys import OpenHRP
import os
import time
import subprocess
import copy

# now, its your time ros
sys.path.extend(rospath)
import roslib; roslib.load_manifest("interactive_markers")
import rospy
from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster

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
    seqsvc.setJointAngles(handsupflipup, 7)
    seqsvc.waitInterpolation()
    seqsvc.setJointAngles(handswide, 12)
    seqsvc.waitInterpolation()
    seqsvc.setJointAngles(handswide, 12)
    #seqsvc.waitInterpolation()
    #goactual()
    
def setcrawler(lvel, rvel):
    v = copy.deepcopy(actual)
    v[30] = v[31] = v[32] = lvel
    v[33] = v[34] = v[35] = rvel
    seqsvc.setJointAnglesWithMask(v, maskcrawler, 1)
    
def processFeedback( feedback ):
    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"

    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo( s + ": button click" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.loginfo( s + ": pose changed")
    server.applyChanges()

def alignMarker( feedback ):
    pose = feedback.pose
    pose.position.x = round(pose.position.x-0.5)+0.5
    pose.position.y = round(pose.position.y-0.5)+0.5
    server.setPose( feedback.marker_name, pose )
    server.applyChanges()

def make6DofMarker( fixed, interaction_mode, position, show_6dof = False):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/WAIST_LINK"
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

    server.insert(int_marker, processFeedback)
    menu_handler.apply( server, int_marker.name )

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

seq = mgr.create('SequencePlayer', 'seq')
seqsvc = rtm.narrow(seq.service('service0'), 'SequencePlayerService', 'hrpsys.OpenHRP')
sh = mgr.create('StateHolder', 'sh')
fk = mgr.create('ForwardKinematics', 'fk')

js = mgr.create('Joystick', 'js')
js.setProperty('device', '/dev/input/js0')
#js.setProperty('debugLevel', '1')
midc = mgr.create('MidJaxonController', 'midc')
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
    
rospy.init_node("basic_controls")
server = InteractiveMarkerServer("basic_controls")
menu_handler = MenuHandler()
menu_handler.insert( "First Entry", callback=processFeedback )
menu_handler.insert( "Second Entry", callback=processFeedback )
position = Point( 0, 0, 0)
make6DofMarker( False, InteractiveMarkerControl.MOVE_ROTATE_3D, position, True )
server.applyChanges()
rospy.spin()

from IPython import embed
embed()

terminator()
