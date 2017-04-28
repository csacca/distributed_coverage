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
from math import pi
from math import tan
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon


# Parameters
xmax=14
ymax=14
xmin=0
ymin=0
fovangle=pi/8
increm=pi/16
anglemin=(-pi/2)+fovangle/2
anglemax=(pi/2)-fovangle/2

# Init variables
maxcov=0
tiltfinal=[0,0,0,0]
side=[]

maxnode=1

##now=[]
#======================================================================================================
def converge(xmin,xmax,ymin,ymax,fovangle,nodes,tilt):
    xp=[]
    x2p=[]
    yp=[]
    y2p=[]
    polygon=[]

    # Classify Nodes based on side
    for i in range(0, len(nodes)):
        if(nodes[i][0] == xmin):
            side.insert(i,2)
        elif(nodes[i][0] == xmax):
            side.insert(i,0)
        elif(nodes[i][1] == ymin):
            side.insert(i,3)
        else:
            side.insert(i,1)
    
    for i in range(0, len(nodes)):
        ptv=[] # 3 vertex points of the ray
        if(side[i]==2):
            yp=(xmax-xmin)*tan(round(fovangle/2-tilt[i],4))+nodes[i][1]
            y2p=(xmax-xmin)*tan(round(-1*fovangle/2-tilt[i],4))+nodes[i][1]
            ptv.append((nodes[i][0],nodes[i][1]))
            if(tilt[i]<=anglemin):
                ptv.append((xmin,ymax))
            else:
                ptv.append((xmax,yp))
            if(tilt[i]>=anglemax):
                ptv.append((xmin,ymin))
            else:
                ptv.append((xmax,y2p))
        elif(side[i]==0):
            yp=(xmax-xmin)*tan(round(fovangle/2+tilt[i],4))+nodes[i][1]
            y2p=(xmax-xmin)*tan(round(-1*fovangle/2+tilt[i],4))+nodes[i][1]
            ptv.append((nodes[i][0],nodes[i][1]))
            if(tilt[i]<=anglemin):
                ptv.append((xmax,ymin))
            else:
                ptv.append((xmin,yp))
            if(tilt[i]>=anglemax):
                ptv.append((xmax,ymax))
            else:
                ptv.append((xmin,y2p))
                
        elif(side[i]==1):
            xp=(ymax-ymin)*tan(round(fovangle/2-tilt[i],4))+nodes[i][0]
            x2p=(ymax-ymin)*tan(round(-1*fovangle/2-tilt[i],4))+nodes[i][0]
            ptv.append((nodes[i][0],nodes[i][1]))
            if(tilt[i]<=anglemin):
                ptv.append((xmax,ymax))
            else:
                ptv.append((xp,ymin))
            if(tilt[i]>=anglemax):
                ptv.append((xmin,ymax))
            else:
                ptv.append((x2p,ymin))
        else:
            xp=(ymax-ymin)*tan(round((fovangle/2)+tilt[i],4))+nodes[i][0]
            x2p=(ymax-ymin)*tan(round(-1*(fovangle/2)+tilt[i],4))+nodes[i][0]
            ptv.append((nodes[i][0],nodes[i][1]))
            if(tilt[i]<=anglemin):
                ptv.append((xmin,ymin))
            else:
                ptv.append((xp,ymax))
            if(tilt[i]>=anglemax):
                ptv.append((xmax,ymin))
            else:
                ptv.append((x2p,ymax))
        polygon.insert(i,Polygon(ptv))
        

    areacov=numpy.zeros((ymax-ymin+1,xmax-xmin+1))
    
    for i in range(xmin,xmax+1):
        for j in range(ymin,ymax+1):
            pointtemp=Point(i,j)
            for k in range(0,len(polygon)):
                if(polygon[k].intersects(pointtemp)):
                    areacov[ymax-j,i]=1

    coverage=numpy.sum(areacov)*100/((ymax-ymin+1)*(xmax-xmin+1))
    
    numpy.set_printoptions(threshold=numpy.nan)
    
    return coverage
#============================================================================ 


def talkerdiscovery(nodeid):
    global pub1
    time.sleep(4)
    rate = rospy.Rate(10) # 10hz
    hello_str=EntryExit()
    hello_str.header.stamp=rospy.Time.now()
    hello_str.state=EntryExit.STATE_ENTRY
    hello_str.node_id=nodeid
    rospy.loginfo(hello_str)
    pub1.publish(hello_str)
    rate.sleep()
    
def talkerannounce(nodeid,x,y,p,t):
        global pub2
        time.sleep(4)
        rate = rospy.Rate(10) # 10hz
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
        pub2.publish(hello_str)
        rate.sleep()
        
def talkeroptupdate(nodeid,p,t):
    global pub3
    time.sleep(4)
    rate = rospy.Rate(10) # 10hz
    hello_str=OptUpdate()
    hello_str.header.stamp=rospy.Time.now()
    hello_str.node_id=nodeid
    jointstatemsg=JointState()
    jointstatemsg.name=[ 'pan', 'tilt']
    jointstatemsg.position=[p,t]
    hello_str.pan_tilt=jointstatemsg
    rospy.loginfo(hello_str)
    pub3.publish(hello_str)
    rate.sleep()

def talkeroptctrl(nodeid,state):
    global pub4
    time.sleep(4)
    rate = rospy.Rate(10) # 10hz
    hello_str=OptCtrl()
    hello_str.header.stamp=rospy.Time.now()
    hello_str.node_id=nodeid
    hello_str.state=state
    rospy.loginfo(hello_str)
    pub4.publish(hello_str)
    rate.sleep()
    
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %r', data)
    
def calldiscovery(data):
    global msg1
    #global entryflag
    global rosid
    ms1=EntryExit()
    msg1=data
    talkerannounce(rosid,rosx,rosy,rostilt,0)
    
def callannounce(data):
    global rosid
    global msg2
    msg2[data.node_id]=data
    time.sleep(1)
    print('All the nodes in network according to')
    print(rosid)
    print(msg2.keys())
    print('Printing NODES')
    for t in msg2:
        print(t)
        
def callupdate(data):
    global msg2
    global newtilt
    global currtilt
    # Update msg2 with new angle
    temp=msg2[data.node_id]
    temp.pan_tilt.position[0]=data.angle
    msg2[data.node_id]=temp
    print('New values')
    print(data.node_id)
    print(msg2)
    print ('-----END-----')
    # Decide if done when max node reached
    if(data.node_id == maxnode):
        change=0
        newtilt=[]
        for t in msg2:
            temp2=msg2[t]
            if(temp2.pan_tilt.position[0]  != currtilt(t-1)):
                change=1
            newtilt.append(temp2.pan_tilt.position[0])
        if(change == 1):
            print('Next Round of iterations')
            talkeroptctrl(1,'next')
        else:
            print('Final Results')
            for t in msg2:
                temp=msg2[t]
                nodes.append([temp.position.x,temp.position.y])
                tilt.append(temp.pan_tilt.position[0])
            print(nodes)
            print(tilt)
    else:
        print('Next Iteration')
        talkeroptctrl(data.node_id+1,'next')
    
def callctrl(data):
    global nodes
    global tilt
    # Optomize a single tilt
    print('Optomizing this node')
    print(data.node_id)
    angle=anglemin
    while(angle<=anglemax):
        tilt[data.node_id-1]=angle
        percent=converge(xmin,xmax,ymin,ymax,fovangle,nodes,tilt)
        if(percent>maxcov):
            maxcov=percent
            tiltfinal=angle
        angle=angle+increm
    # Update that node's angle    
    talkeroptupdate(data.node_id,angle,0)
    

if __name__ == '__main__':
    # Initialize Node
    rospy.init_node('Node1', anonymous=True)
    # Create Publishers
    pub1 = rospy.Publisher('discovery', EntryExit, queue_size=10)
    pub2 = rospy.Publisher('announce', Identify, queue_size=10)
    pub3 = rospy.Publisher('optupdate', OptUpdate, queue_size=10)
    pub4 = rospy.Publisher('optctrl', OptCtrl, queue_size=10)
    # Input Param Setup
    rosid=rospy.get_param('~nodeid',0)
    rosx=rospy.get_param('~posx',0)
    rosy=rospy.get_param('~posy',0)
    rostilt=rospy.get_param('~tilt',0)

    global msg2
    global msg3
    msg2={}

    #Listen for data from other nodes
    rospy.Subscriber('announce', Identify, callannounce) #msg1 should now have array of all node data

    # Announce self to other nodes
    talkerdiscovery(rosid)

    # Listen for data from other nodes
    time.sleep(5)
    print('send self')
    print(rosid)
    talkerannounce(rosid,rosx,rosy,rostilt,0)
    time.sleep(2)
    
    rospy.Subscriber('discovery', EntryExit, calldiscovery)
    print('CAN RUN OTHER NODES')
    time.sleep(5)
    rospy.Subscriber('optupdate', OptUpdate, callupdate)
    rospy.Subscriber('optctrl', OptCtrl, callctrl)
                     
    while(True):
        nodes=[]
        tilt=[]
        print("Wait for and Organize data")
        time.sleep(20)
        for t in msg2:
            temp=msg2[t]
            nodes.append([temp.position.x,temp.position.y])
            tilt.append(temp.pan_tilt.position[0])
        print(nodes)
        print(tilt)
        currtilt=tilt
        print("Begin Optomization")
        time.sleep(2)
        if(rosid ==1):
            talkeroptctrl(rosid,'next')
        else:
            time.sleep(5)
        print("END Optomization")
        time.sleep(25)
                     
        

