#!/usr/bin/env python
import rospy
import numpy as np
import math
import random
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3

class Drone: # change the parameters of the drone
    def __init__(self):
        # initialise variables
        self.dist=[0.]*9
        self.distx=[0.]*9
        self.disty=[0.]*9
        self.ang=[0.]*9
        self.acc=Vector3(0.,0.,0.)
        self.vel=0
        self.go=True
        self.target=0
        # initialise max values
        self.maxaccx=3
        self.maxaccy=30
        # Safe values
        self.maxspeed=3.5 #real = 4.1
        self.closedist=1
        self.safedist=0.5
        
        
# global drone variable
DRONE=Drone()

# Publisher for sending acceleration commands to flappy bird
pub_acc_cmd = rospy.Publisher('/flappy_acc', Vector3, queue_size=1)

def initNode():
    # Here we initialize our node running the automation code
    
    rospy.init_node('flappy_automation_code', anonymous=True)
    
    # Subscribe to topics for velocity and laser scan from Flappy Bird game
    
    rospy.Subscriber("/flappy_laser_scan", LaserScan, laserScanCallback)
    rospy.Subscriber("/flappy_vel", Vector3, velCallback)
        
    #while not rospy.is_shutdown():
    
    # Ros spin to prevent program from exiting
    rospy.spin()

def velCallback(msg):
    # msg has the format of geometry_msgs::Vector3
    # Example of publishing acceleration command on velocity velCallback
#    x = 0.2
#    y = 0

    global DRONE
    DRONE.vel=msg
    
    ## x acceleration ##
    
    # go
    if DRONE.go == True:
        DRONE.acc.x = DRONE.maxaccx
        DRONE.scan = True
#        DRONE.acc.y=0
    
    # Stop not to hit walls, stay at 0.9 m from it
    if 0 < DRONE.distx[4] < DRONE.vel.x**2/(2*DRONE.maxaccx)+DRONE.closedist or 0 < DRONE.distx[3] < DRONE.vel.x**2/(2*DRONE.maxaccx)+DRONE.closedist or 0 < DRONE.distx[5] < DRONE.vel.x**2/(2*DRONE.maxaccx)+DRONE.closedist:
        DRONE.acc.x = -DRONE.maxaccx
        DRONE.go = False
    else:
        DRONE.go = True
        
    # Don't go too fast
    if DRONE.acc.x > 0 and DRONE.vel.x >= DRONE.maxspeed*0.9:
        DRONE.acc.x = 0
    
    ## y acceleration ##
    
    detection()
    if DRONE.go == True:
        DRONE.acc.y = 40*-DRONE.vel.y
    else:
        DRONE.acc.y = 30*(4*DRONE.target-DRONE.vel.y)

    # Don't roof and ground
    
    if DRONE.go == False and 0 < abs(DRONE.disty[0]) < DRONE.safedist and 0 < abs(DRONE.disty[1]) < DRONE.safedist and 0 < abs(DRONE.disty[2]) < DRONE.safedist:
        DRONE.acc.y=DRONE.maxaccy
        DRONE.target = DRONE.closedist
        print("GROUND")
    if DRONE.go == False and 0 < abs(DRONE.disty[8]) < DRONE.safedist and 0 < abs(DRONE.disty[7]) < DRONE.safedist and 0 < abs(DRONE.disty[6]) < DRONE.safedist:
        DRONE.acc.y=-DRONE.maxaccy
        DRONE.target = -DRONE.closedist
        print("ROOF")
        
    # In case of the drone don't see the next hole
    
    if DRONE.go == False and DRONE.target == 0 and DRONE.vel.y < 0.1:
        DRONE.acc.y = 1
    
    # Take car of max acc
    
    if abs(DRONE.acc.y) > DRONE.maxaccy:
        DRONE.acc.y = abs(DRONE.acc.y)/DRONE.acc.y*DRONE.maxaccy
    if abs(DRONE.acc.x) > DRONE.maxaccx:
        DRONE.acc.x = abs(DRONE.acc.x)/DRONE.acc.x*DRONE.maxaccx
    
    ## Writing stuff
    if DRONE.go == True:
        print("go")
    else:
        print("STOP")
    print(DRONE.target)
    print(DRONE.vel.y)
    print(DRONE.acc.y)
    print("-----")

    
    ## Accelerating

    pub_acc_cmd.publish(DRONE.acc)

def detection():
    global DRONE
    
    Hole=[0]*8
    localtarget=[0]*8
    nb=0
    for ii in range(0,8):
        if DRONE.distx[ii] > 1.5*DRONE.closedist and DRONE.distx[ii+1] > 1.5*DRONE.closedist:
            localtarget[ii]=DRONE.closedist*math.tan((DRONE.ang[ii]+DRONE.ang[ii+1])/2)
            Hole[ii]=1
    for ii in range(0,8):
        if Hole[ii]==1:
            nb=nb+1
    if nb != 0:
        DRONE.target=sum(localtarget)/nb
        
    #if DRONE.scan == True:
     #   DRONE.target=DRONE.targetauto
     #   DRONE.scan = False

def laserScanCallback(msg):
    # msg has the format of sensor_msgs::LaserScan
    # print laser angle and range
    global DRONE
    for ii in range(0,9):
        DRONE.dist[ii]=msg.ranges[ii]
        DRONE.ang[ii]=msg.angle_min+ii*msg.angle_increment
        DRONE.distx[ii]=DRONE.dist[ii]*math.cos(DRONE.ang[ii])
        DRONE.disty[ii]=DRONE.dist[ii]*math.sin(DRONE.ang[ii])
        
    #print distance
#    print "Laser range: {}, angle: {}".format(msg.ranges[4], msg.angle_min)

if __name__ == '__main__':
    try:
        
        initNode()
        
    except rospy.ROSInterruptException:
        pass    
    
    
