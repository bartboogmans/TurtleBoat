#!/usr/bin/env python3

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String, Float32MultiArray, Float32
from enum import Enum, auto

class GlobalState(Enum):
    """
    Class for managing object states in a readable manner
    :param: none
    :return: the created object
    """
    disconnected = auto()
    waiting = auto()
    ready = auto()
    shuttingDown = auto()

class Vessel: 
    """
    Class to collect various ship related statusses
    :param: none
    :return: the created object
    """
    def __init__(self, name_,x_):
        self.x = [1,0,90,0,0,0]
        self.name = "BoatyMcBoatFace"
        
    
class Actuator:
    """
    This is probably too abstract.... hmmm idk about this. 
    
    """
    def __init__(self,state0_,ref0_,outputFnc_,internalUpdateFnc_):
        self.state = state0_ 
        self.ref = ref0_
        self.outputFnc = thrModel_ # =lambda a: [a[0] + 1,a[1]+2]
        self.internalUpdateFnc = internalUpdateFnc_

    def calc_thruster_output(self,dt):
        self.state_var = 
        return thrModel_
    
    def set_state_var_ref(self,state_var_ref_):
        self.state_var_ref = state_var_ref_
        
    
class TitoNeri(Vessel):
    def __init__(self,name_,x_):
        super().__init__(name_,x_)
        self.aftthruster_rps_to_force = lambda v: numpy.sign(v)*0.0009752*v^2
        self.bow_pwm_to_force = lambda PWM_value: PWM_value*3.575
        
        
    
def vesselModelRun():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        vesselModelRun()
    except rospy.ROSInterruptException:
        pass
