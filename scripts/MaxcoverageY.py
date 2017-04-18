#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String

def callback(data):
     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
     
def listener(topic):
 
     # In ROS, nodes are uniquely named. If two nodes with the same
     # node are launched, the previous one is kicked off. The
     # anonymous=True flag means that rospy will choose a unique
     # name for our 'listener' node so that multiple listeners can
     # run simultaneously.
     ##rospy.init_node('listener', anonymous=True)
     rospy.Subscriber(topic, String, callback)
 
     # spin() simply keeps python from exiting until this node is stopped
     rospy.spin()

def talker(topic):
     pub = rospy.Publisher(topic, String, queue_size=10)
     time.sleep(2)
     ##rospy.init_node('talker', anonymous=True)
     rate = rospy.Rate(10) # 10hz
     while not rospy.is_shutdown():
         hello_str = "hello world %s" % rospy.get_time()
         rospy.loginfo(hello_str)
         pub.publish(hello_str)
         rate.sleep()

     
if __name__ == '__main__':
    rospy.init_node('NODESELF', anonymous=True)
    #*enter network
    #*broadcast entry to get other nodes
    #talker(NodeID)
    #*send node id to those nodes
    #listener(Nodes)
    #*assign correct node ID
    #NodeID=min(nodes)+1
    #*on start
    prev=0
    curr=0
    flag=1
    
    
    #if first node, call self
    talker(#Next1)
    #call OnNext
    #on next
    listener(#next node message)
        
    if((prev==curr))and(flag==0):
        talker("End")
        print("hi")
    else:
        prev=curr
        flag=0

    #Compute Optimization
    optimize() #needs global variables

    #send Update
    talker(#update)

    #Next Node    
    talker(#Next+1)
