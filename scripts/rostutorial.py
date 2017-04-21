#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import time
import numpy
from distributed_coverage.msg import EntryExit
from distributed_coverage.msg import Identify
from distributed_coverage.msg import OptCtrl
from distributed_coverage.msg import OptUpdate
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_msgs.msg import String

def talkerdiscovery(nodeid):
    pub = rospy.Publisher('discovery', EntryExit, queue_size=10)
    time.sleep(2)
    rate = rospy.Rate(10) # 10hz
    hello_str=EntryExit()
    hello_str.header.stamp=rospy.Time.now()
    hello_str.state=EntryExit.STATE_ENTRY
    hello_str.node_id=nodeid
    rospy.loginfo(hello_str)
    pub.publish(hello_str)
    rate.sleep()
    
def talkerannounce(nodeid,x,y,p,t):
        pub = rospy.Publisher('announce', Identify, queue_size=10)
        time.sleep(2)
        #rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        #while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        hello_str=Identify()
        hello_str.header.stamp=rospy.Time.now()
        hello_str.node_id=nodeid
        pointmsg=Point()
        pointmsg.x=x
        pointmsg.y=y
        jointstatemsg=JointState()
        jointstatemsg.name=[ 'pan', 'tilt']
        jointstatemsg.position=[p,t]
        hello_str.position=pointmsg
        hello_str.pan_tilt=jointstatemsg
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
        
def talkeroptupdate(nodeid,p,t):
    pub = rospy.Publisher('optctrl', OptUpdate, queue_size=10)
    time.sleep(2)
    rate = rospy.Rate(10) # 10hz
    hello_str=OptUpdate()
    hello_str.header.stamp=rospy.Time.now()
    hello_str.node_id=nodeid
    jointstatemsg=JointState()
    jointstatemsg.name=[ 'pan', 'tilt']
    jointstatemsg.position=[p,t]
    hello_str.pan_tilt=jointstatemsg
    rospy.loginfo(hello_str)
    pub.publish(hello_str)
    rate.sleep()

def talkeroptctrl(nodeid,state):
    pub = rospy.Publisher('optupdate', OptCtrl, queue_size=10)
    time.sleep(2)
    rate = rospy.Rate(10) # 10hz
    hello_str=OptCtrl()
    hello_str.header.stamp=rospy.Time.now()
    hello_str.node_id=nodeid
    hello_str.state=state
    rospy.loginfo(hello_str)
    pub.publish(hello_str)
    rate.sleep()
    
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %r', data)
def callback1(data):
    print('hi')
    print(data.node_id)
    print(data.position)
    print(data.pan_tilt)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('chatter', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

if __name__ == '__main__':
    rospy.init_node('Node1', anonymous=True)
    rospy.Subscriber('optctrl', OptCtrl, callback)
    rospy.Subscriber('discovery', EntryExit, callback)
    talkerdiscovery(5)
    rospy.Subscriber('announce', Identify, callback1)
    talkerannounce(6,3,4,5,7)
    talkeroptctrl(12,'start')
    #try:
     #   talker()
    #except rospy.ROSInterruptException:
     #   pass
    #istener()
