# Masters Project: Global Converage Maximization in Dynamic network
# Tufts University 2016/2017: Steven & Christopher
# Advisor: Usman Khan
#
#
# Python Code 2/21/2017

# Libraries
import numpy
from math import pi
from math import tan
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import time

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

# User/System Variables
#nodes=[[xmin,6],[xmin,1],[8,ymax],[4,ymin]]
#tilt=[0,0,0,0]
nodes=[[0,3],[14,2],[0,7]]
tilt=[0,0,0]

#nodes=[[9,ymin]]
#tilt=[0]
start=time.time()

        
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

#======================================================================================================
def converge2(xmin,xmax,ymin,ymax,fovangle,nodes,tilt):
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
    # Find the 3 Points for Camera view polygon
    for i in range(0, len(nodes)):
        ptv=[] # 3 vertex points of the ray
        # Evaluates 2 line equations for the view of the camera
        # Creates 3 points based on side and angles
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
        
    # Determine which pixels are covered for any camera node
    areacov=numpy.zeros((ymax-ymin+1,xmax-xmin+1))
    for i in range(xmin,xmax+1):
        for j in range(ymin,ymax+1):
            pointtemp=Point(i,j)
            for k in range(0,len(polygon)):
                # Is the pixel covered by any camera?
                if(polygon[k].intersects(pointtemp)):
                    areacov[ymax-j,i]=1
    # Compute Percent Coverage
    coverage=numpy.sum(areacov)*100/((ymax-ymin+1)*(xmax-xmin+1))
    # Print Format
    numpy.set_printoptions(threshold=numpy.nan)
    print(areacov)
    print('Sensor Nodes: ')
    print(nodes)
    print('Sensor Node Angles(rad): ')
    print(tilt)
    print('Percent Covered: ')
    print(coverage)
    print('Elapsed Time(sec): ')
    print(round(time.time()-start,4))
    print("")
    
    return coverage
#============================================================================                                            

# Main File

print('Initial Tilt (Rads)')
print(tilt)

# Iteratively/Distributively find best angle for that sensor given all info
# Finish when the angles do not change after another iteration
tiltfinal=[]
while True:
    tiltcurr=tilt[:]
    for i in range(0, len(nodes)):
        angle=anglemin
        while(angle<=anglemax):
            print(angle)
            tilt[i]=angle
            percent=converge(xmin,xmax,ymin,ymax,fovangle,nodes,tilt)
            if(percent>maxcov):
                maxcov=percent
                tiltfinal=tilt[:]
            angle=angle+increm
        tilt=tiltfinal[:]
    if(tiltfinal==tiltcurr):
        break
    else:
        tiltcurr=tiltfinal[:]
        
percent=converge2(xmin,xmax,ymin,ymax,fovangle,nodes,tilt)


