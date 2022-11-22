#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String, Float32MultiArray, Float32
from enum import Enum, auto
import time
import math
from sensor_msgs.msg import NavSatFix 


class Vessel: 
    """
    Class to collect various ship related statusses
    :param: none
    :return: the created object
    """
    def __init__(self, name_,x_):
        self.x = x_
        self.name = name_.replace(" ","_")
        self.u = [-1,-1]
    
    def getResultantThrust(self):
        return 1
    
class TitoNeri(Vessel):    
    """
    Class representing a Tito Neri model scale vessel
    :param: none
    :return: the created object
    """
    
    ## https://pythonexamples.org/python-callback-function/
    # this suggests implementation of user defined functions can be really simple:
    # printFileLength("sample.txt", callbackFunc1)
    
    """
    def printFileLength(path, callback):
        f = open(path, "r")
        length = len(f.read())
        f.close()
        callback(length)
    """
    
    def __init__(self,name_,x_):
        super().__init__(name_,x_)
        self.aftthruster_rps_to_force = lambda v: numpy.sign(v)*0.0009752*v^2
        self.bow_pwm_to_force = lambda PWM_value: PWM_value*3.575
        self.u = [0,0,0]
        self.alpha = [0,0] 
        
    def getResultantThrust(self):
        return 1

class timedFncTracker:
    """
    Class to track periodic calling of functions
    :param: rate_ the rate of the object to run in hz
    :return: the created object
    """
    def __init__(self,rate_):
        self.tstart = time.time()
        self.tlast =  self.tstart
        self.period = 1/rate_
        self.sequence = 0    
    def isready(self):
        now = time.time()
        if now > self.tlast + self.period:
            self.tlast =  time.time()
            self.sequence = self.sequence + 1
            return 1
        else:
            return 0
    def timeSinceStart(self):
        return time.time() - self.tstart
            
class vesselSim:
    """
    Class that tracks and performs simulation steps on a vessel object with specified dynamics
    :param: rate_ the rate of the object to run in hz
    :return: the created object
    """
    def __init__(self,vesselname_,x0,rate_):
        self.vessel = TitoNeri(vesselname_,x0)
        self.runtimer = timedFncTracker(rate_)
        self.lastt = self.runtimer.tstart
    def simstep(self,t):
        
        # Calculate forces and torques
        Fd = [0,0,0]      # Drag (e.g. linear or quadratic)
        Fc = [0,0,0]      # Coriolis & Centripetal    
        Fact = [0,0,0]    # Actuators (fins, rudders, propellers)
        Fdist = [0,0,0]   # Disturbance (e.g. noise or wind)
        Fext = [0,0,0]    # External (e.g. contact)
        
        Ftotal = Fd + Fc + Fact + Fdist + Fext
        
        M = [[1,0,0],[0,1,0],[0,0,0.1]] # System inertia, including hydrodynamic added mass
        
        # Accelleration
        nu_dot = [0,0,0]
        
        # Velocities
        nu = [0,0,0]
        
        # Displacement
        d_eta = [0,0,0]
        self.vessel.x[0:2] = self.vessel.x[0:2] + d_eta
        
            
def actuationCallback(data,args):
    vessel = args[0]
    vessel.u = [msg.data[0],msg.data[1],msg.data[2],msg.data[3],msg.data[4]]
            
def vesselModelRun():
    posPubTimer = timedFncTracker(2)
    headPubTimer = timedFncTracker(16)
    
    reportStatusTimer = timedFncTracker(0.5)
    
    sim = vesselSim('Hum a Tuna',[52.1,4.37,math.pi/2,1,0,0],40)
    
    posPub = rospy.Publisher(sim.vessel.name+'/geoPos_est',NavSatFix, queue_size=0)
    headPub = rospy.Publisher(sim.vessel.name+'/heading_est', Float32, queue_size=0)
    actSub = rospy.Subscriber(sim.vessel.name+'/u_ref', Float32MultiArray, actuationCallback,(sim.vessel))
    rospy.init_node(sim.vessel.name+'_python_motion_sim_geographical', anonymous=True)
    
    rate = rospy.Rate(100) #hz

    while not rospy.is_shutdown():
        if sim.runtimer.isready():
            # Do a simulation step
            pass 
        
        if posPubTimer.isready():
            # Send position
            msg = NavSatFix()
            msg.header.seq = posPubTimer.sequence
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'world'
            
            msg.latitude = sim.vessel.x[0]
            msg.longitude = sim.vessel.x[1]
            msg.altitude = 0#float("nan")
            posPub.publish(msg)
        
        if headPubTimer.isready():
            # Send heading
            msg = Float32()
            msg.data =  sim.vessel.x[2]
            headPub.publish(msg)

        if reportStatusTimer.isready():  
            # Periodic reporting
            print('['+"{:.3f}".format(sim.runtimer.timeSinceStart())+'] '+sim.vessel.name+' sim seq='+"{:.0f}".format(sim.runtimer.sequence))
            
        rate.sleep()

if __name__ == '__main__':
    try:
        vesselModelRun()
    except rospy.ROSInterruptException:
        pass
    


"""
Ideas

- implement reasonable physics
- random actuation 
- implement reset rosservice
- Periodically changing disturbance (~50grams peak)
- external function for actuator behaviour

- ros2 node 
- ros2 parameters for node
- waypoint controller
- nodelet ros2


Modules:
- control effort allocator

- ros package with Launch that: 
    - starts drift ratio calculator
    - audio player
"""
