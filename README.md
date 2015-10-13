Controller for MIDJAXON
-----------------------

Complete source code for MIDJAXON virtual robot.
Won the 1st prize of JVRC competition.
Main part of the source is distributed under 3-clause BSD license.

setup.py : High level controller written using rtm.py and rospy (BSD license)

MidJaxonController.[cpp,h] : Low level real-time controller written in C++ (BSD license)

PDcontrollerMIDJAXON.[cpp,h] : Customized version of PDcontroller forked from hrpsys-base (hrpsys-base license)

rtm.py : rtm.py python script with small bug fix (hrpsys-base license)

Blog article:
http://tech.mid-japan.com/blog/2015/10/13/jvrc-midjaxon-sourcecode/

For the virtual robot model, see:
https://github.com/devrt/rtmros_choreonoid
