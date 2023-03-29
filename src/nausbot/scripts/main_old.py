#!/usr/bin/env python

"""
Ship simulator / software-in-the-loop-system that listens to actuation and outputs simulated ship response.
Initial developer: Bart Boogmans (bartboogmans@hotmail.com)


Use: 
 Install ROS & Setup Environment
 >> rosrun nausbot main.py vessel1name <optional parameters>
 
Initial parameters of the Tito-Neri vessel line are obtained though the publication:
 "Model predictive maneuvering control and energy management for all-electric autonomous ships" https://www.sciencedirect.com/science/article/pii/S0306261919309705
 
Missing parameters (such as dampening coefficients) were estimated using rules of thumb, linearization in characteristic working points or free-body-diagram supported estimates. The values are assumed to be quite reasonable, although a difference between simulation and reality is unavoidable. The signs and general behaviour of the ship dynamics which are herein presented should however give a good feel of how these vessels respond, allowing coarse tests of control algorythms and learning to interact with vessel control systems over ROS.

Notice:
This work can be copied, changed and redistributed as long as documentation remains clear on contributions of origonal contributors. 
This work can be used to generate content (such as: datasets, figures, model-parameters) for publications (including education deliverables such as msc thesis) given that explicit recognition has been given on the use of this tool in the derived work.

"""

import rospy
from std_msgs.msg import String, Float32MultiArray, Float32
from enum import Enum, auto
import time
import math
from sensor_msgs.msg import NavSatFix 
from geometry_msgs.msg import Wrench, Twist
import numpy as np
import argparse
import matplotlib.pyplot as plt


RATE_PUB_HEADING = 16
RATE_PUB_POS = 5
RATE_SIM = 200
RATE_PUB_STATE_AUXILIARY = 5

R_EARTH = 6371000 #m

REFERENCE_RUNTIME_TIMEOUT = 5000 * 1E6  # 5 seconds
ACTUATION_PRIO_MSG_TIMEOUT = 3000 *1E6 # period after last actuation_prio message when normal actuation messages are accepted

# Process function arguments
parser = argparse.ArgumentParser()
parser.add_argument("vesselid", type=str,help="set vessel identifier")
parser.add_argument('-p','--pose0', nargs=6,type=float, help='Starting pose')
parser.add_argument('-v','--velocity0', nargs=6,type=float, help='Starting velocities')
parser.add_argument("-rsim", "--ratesimulator", type=float,help="set rate of simulation")
parser.add_argument("-rhead", "--rateheading", type=float,help="set rate of heading publishing")
parser.add_argument("-rpos", "--rateposition", type=float,help="set rate of position publishing")
parser.add_argument("-raux", "--rateauxiliary", type=float,help="set rate of auxiliary state publishing")
args = parser.parse_args()
VESSEL_ID = args.vesselid

ERRTRACKER1 = 0

if args.ratesimulator:
	RATE_SIM = args.ratesimulator
if args.rateauxiliary:
	RATE_PUB_STATE_AUXILIARY = args.rateauxiliary
if args.rateheading:
	RATE_PUB_HEADING = args.rateheading
if args.rateposition:
	RATE_PUB_POS = args.rateheading

class SimulationState(Enum):
	"""
	Class for managing object states in a readable manner
	:param: none
	:return: the created object
	"""
	initializing = auto()
	ready = auto()
	busy = auto()
	shuttingDown = auto()

class Vessel: 
	"""
	Class to collect various ship related statusses
	:param: none
	:return: the created object
	"""
	def __init__(self, name_,pose_,vel_):
		self.pose = pose_ # [lat long altitude pitch roll yaw] rotations w.r.t. north east down
		self.vel = vel_ # [u,v,w,p,q,r]
		self.name = name_.replace(" ","_")
		
		## Default thruster fields:
		self.ntrh = 4
		self.thrustToForce = list()
		for i in range(self.ntrh):
			self.thrustToForce.append(lambda v: 10*v) # output: Newton, Input: RPS
		
		self.u = np.zeros(self.ntrh)
		self.u_ref = np.zeros(self.ntrh)
		self.u_lims = np.zeros((self.ntrh,2))
		self.alpha = np.zeros(self.ntrh)
		self.alpha_ref = np.zeros(self.ntrh)
		self.alpha_lims = np.zeros((self.ntrh,2))
		self.last_ref_timestamp = time.time_ns()
		self.last_actuation_reference_prio_message = 0

		
	def getResultantThrust(self):
		return 1

class TitoNeri(Vessel):	
	"""
	Class representing a Tito Neri model scale vessel
	:param: none
	:return: the created object
	"""
	
	def __init__(self,name_,pose_,vel_):
		super().__init__(name_,pose_,vel_)

		## Actuator parameters
		self.ntrh = 3
		self.thrustToForce = [lambda v: ((1.925e-5)*v*v*v+(1.061e-2)*v), # output: Newton, Input: RPS
							  lambda v: ((1.925e-5)*v*v*v+(1.061e-2)*v), # output: Newton, Input: RPS
							  lambda PWM_value: PWM_value*3.575] # output: Newton, Input is normalized pwm [-1:1]
		# alternative purely quadratic relation aft thruster: lambda v: np.sign(v)*0.0009752*v**2,
		
		# Define limits of actuators
		self.u_lims = np.array([[-60,60],[-60,60],[-1,1]]) # lower and upper bounds of all thruster outputs
		self.alpha_lims = np.array([[-3/4*math.pi,3/4*math.pi],[-3/4*math.pi,3/4*math.pi],[math.pi/2,math.pi/2]]) # lower and upper bounds of all thruster angles
		
		# Define maximum rate of change of actuators
		self.u_rate_lim = np.array([120,120,0.5]) # [r/s,r/s,1/s]
		self.alpha_rate_lim = np.array([math.pi*2.0,math.pi*2.0,0]) # [rad/sec]
		
		self.u = [0,0,0] # initial actuator output
		self.alpha = [0,0,math.pi/2] # initial actuator orientation
		self.r_thruster = np.array([[-0.42,-0.08,0],[-0.42,+0.08,0],[0.28,0.00,0]])
		
		# Size
		self.l = 0.97
		self.w = 0.30

		# the back and front part of the hull wrt CG
		self.ship_rear_x = -0.42 -0.0952
		self.ship_front_x = self.ship_rear_x + self.l

		## Dynamics
		self.D = np.array([		 [2.6416	 ,0		  ,0		  ,0		  ,0		  ,0		  ],
									[0		  ,21.9034	,0		  ,0		  ,0		  ,-1.0952	],
									[0		  ,0		  ,0		  ,0		  ,0		  ,0		  ],
									[0		  ,0		  ,0		  ,0		  ,0		  ,0		  ],
									[0		  ,0		  ,0		  ,0		  ,0		  ,0		  ],
									[0		  ,-1.0952	,0		  ,0		  ,0		  ,3.7096	 ]])

		self.Mrb = np.array([	   [16.9	   ,0		  ,0		  ,0		  ,0		  ,0		  ],
									[0		  ,16.9	   ,0		  ,0		  ,0		  ,0		  ],
									[0		  ,0		  ,16.9	   ,0		  ,0		  ,0		  ],
									[0		  ,0		  ,0		  ,1		  ,0		  ,0		  ],
									[0		  ,0		  ,0		  ,0		  ,1		  ,0		  ],
									[0		  ,0		  ,0		  ,0		  ,0		  ,0.51	   ]])
		# Note that the current diagonals on inertia in pitch and roll direction are 1. These values are not measured or taken up in the current model, but these points have to be nonzero to make this matrix invertible. 

		self.Ma = np.array([		[1.2		,0		  ,0		  ,0		  ,0		  ,0		  ],
									[0		  ,1.2		,0		  ,0		  ,0		  ,0		  ],
									#[0		  ,49.2	   ,0		  ,0		  ,0		  ,0		  ],
									[0		  ,0		  ,0		  ,0		  ,0		  ,0		  ],
									[0		  ,0		  ,0		  ,0		  ,0		  ,0		  ],
									[0		  ,0		  ,0		  ,0		  ,0		  ,0		  ],
									[0		  ,0		  ,0		  ,0		  ,0		  ,1.8		]])
		self.cg = np.array([0,0,0])

		self.M = self.Mrb + self.Ma
		
		# Variable to track whether this ship has it's reference timed out.
		self.referenceTimedOut = 0
		
	def calc_f_act(self):
		"""
		Calculates resultant force from all the actuators
		:param: none
		:return: resultant force/torque vector
		"""
		Fres = np.array([0,0,0,0,0,0])
		for nthr in range(self.ntrh):
			
			# Check bounds of maximum thrust usage
			if self.u[nthr] < self.u_lims[nthr][0]:
				self.u[nthr] = self.u_lims[nthr][0]
			elif self.u[nthr] > self.u_lims[nthr][1]:
				self.u[nthr] = self.u_lims[nthr][1]
			
			# Check bounds of maximum thruster angles
			if self.alpha[nthr] < self.alpha_lims[nthr][0]:
				self.alpha[nthr] = self.alpha_lims[nthr][0]
			elif self.alpha[nthr] > self.alpha_lims[nthr][1]:
				self.alpha[nthr] = self.alpha_lims[nthr][1]

			# Calculate response
			Fthr_i_thrLocal = np.array([self.thrustToForce[nthr](self.u[nthr]),0,0])
			Fthr_i_body = np.matmul(R6_b_to_n(0,0,self.alpha[nthr]),Fthr_i_thrLocal)
			mi = np.cross(self.r_thruster[nthr],Fthr_i_body)
			
			# Add up response for each thruster
			Fres = Fres + np.array([Fthr_i_body[0],Fthr_i_body[1],Fthr_i_body[2],mi[0],mi[1],mi[2]])
		return Fres

	def set_u(self):
		pass
		
	def getCrb(self):
		return getCoriolisCentripetal(self.vel,self.Mrb)
	def getCa(self):
		return getCoriolisCentripetal(self.vel,self.Ma)

	def getRbn(self):
		return R6_b_to_n(self.pose[3],self.pose[4],self.pose[5])

def getCoriolisCentripetal(v,M):
	"""
	Forms coriolis-centripetal matrix from velocity vector and inertial matrix according to the parameterisation of Fossen's Handbook of marine control (2011) eq. 6.43
	:param: v 1x6 velocity vector [u,v,w,p,q,r]
	:param: M 6x6 Inertial matrix
	:return: 6x6 coriolis matrix

	Tested with same parameters as Fossen2011 op p56

	M1 = np.zeros((6,6))
	M1[0:3,0:3] = 1000*np.eye(3)
	M1[3:6,3:6] = 10000*np.eye(3)
	v = np.array([10,1,1,1,2,3])
	print(getCoriolisCentripetal(v,M1))

	>> Crb = [  [	 0.	  0.	  0.	 -0.   1000.  -1000.]
				[	 0.	  0.	  0.  -1000.	 -0.  10000.]
				[	 0.	  0.	  0.   1000. -10000.	 -0.]
				[	-0.   1000.  -1000.	 -0.  30000. -20000.]
				[ -1000.	 -0.  10000. -30000.	 -0.  10000.]
				[  1000. -10000.	 -0.  20000. -10000.	 -0.]   ]


	"""
	v1 = v[0:3] # v1 = u,v,w
	v2 = v[3:6] # v2 = p,q,r
	M11 = M[0:3,0:3]
	M12 = M[3:6,0:3]
	M21 = M[0:3,3:6]
	M22 = M[3:6,3:6]

	out = np.zeros((6,6))
	out[3:6,0:3] = -skew(np.matmul(M11,v1)+np.matmul(M12,v2))
	out[0:3,3:6] = -skew(np.matmul(M11,v1)+np.matmul(M12,v2))
	out[3:6,3:6] = -skew(np.matmul(M21,v1)+np.matmul(M22,v2))

	return(out)

def skew(v):
	"""
	Forms skew symmetric matrix from vector
	:param: v 1x3 input vector
	:return: 3x3 skew symmetric matrix
	"""
	return np.array([   [0		  ,-v[2]	  ,v[1]   ],
						[v[2]	   ,0		  ,-v[0]  ],
						[-v[1]	  ,v[0]	   ,0	  ]])

def R6_b_to_n(roll,pitch,yaw):
	Rx = np.array([ [1		   ,0			 ,0		  ],
					[0		   ,np.cos(roll)	 ,-np.sin(roll) ],
					[0		   ,np.sin(roll)	 ,np.cos(roll)  ]])


	Ry = np.array([ [np.cos(pitch)   ,0			 ,np.sin(pitch)  ],
					[0		   ,1			 ,0		  ],
					[-np.sin(pitch)  ,0			 ,np.cos(pitch)  ]])


	Rz = np.array([ [np.cos(yaw)   ,-np.sin(yaw)	,0		  ],
					[np.sin(yaw)   ,np.cos(yaw)	 ,0		  ],
					[0		   ,0			 ,1		  ]])
	return np.matmul(Rz,np.matmul(Ry,Rx))

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
	def __init__(self,vesselname_,pose0_,vel0_,rate_):
		self.vessel = TitoNeri(vesselname_,pose0_,vel0_)
		self.runtimer = timedFncTracker(rate_)
		self.lastt = self.runtimer.tstart
		self.state = SimulationState.initializing
		self.ERRTRACKER1 = 0
		self.el = eventlogger(1000,30,self.lastt)
		self.lastF = np.zeros((6,1))
		self.lastF_actuator = np.zeros((6,1))
		

		
	def simstep(self,t):
		if self.state == SimulationState.initializing:
			self.lastt = time.time()
			self.state = SimulationState.ready
			
		elif self.state == SimulationState.ready:
			self.state = SimulationState.busy
			
			# Check for timeout of reference:
			if time.time_ns() - self.vessel.last_ref_timestamp > REFERENCE_RUNTIME_TIMEOUT and not self.vessel.referenceTimedOut:
				self.vessel.referenceTimedOut = 1
				self.vessel.u = np.array([0,0,0])
				self.vessel.alpha = np.array([0,0,0])
				print('No reference in ' + "{:.2f}".format(REFERENCE_RUNTIME_TIMEOUT/1E9) +" seconds -> Stopping vessel actuation") 

			dt = t - self.lastt

			# Calculate forces and torques
			Fd = -1*np.matmul(self.vessel.D,self.vessel.vel)	 # Drag (e.g. linear or quadratic)
			Fc = -1*np.matmul(self.vessel.getCrb()+self.vessel.getCa(),self.vessel.vel)	# Coriolis & Centripetal
			Fact = self.vessel.calc_f_act() # Actuators (fins, rudders, propellers)
			Ftotal = Fd + Fc + Fact
			self.lastF = Ftotal
			self.lastF_actuator = Fact

			# Accelleration
			nu_dot = np.matmul(np.linalg.inv(self.vessel.M),Ftotal)

			# Velocities
			dvel = nu_dot*dt
			self.vessel.vel = self.vessel.vel + dvel

			# Displacement  (cartesian body fixed coordinate system)
			d_eta = self.vessel.vel*np.array([dt])

			# Convert to North-east-down tangent displacements: (cartesian local coordinate system)
			dpos_tangent = np.matmul(self.vessel.getRbn(),d_eta[0:3])

			# Conversion to geographical displacement (geographical global coordinate system)
			dlat =np.rad2deg(np.arctan2(dpos_tangent[0],R_EARTH)) # degrees
			r_earth_at_lat = np.cos(np.deg2rad(self.vessel.pose[0]))*R_EARTH	# Radius of slice of earth at particular latitude
			dlong = np.rad2deg(np.arctan2(dpos_tangent[1],r_earth_at_lat))

			# Set new position (long (deg) ,lat (deg) ,altitude (m) ,roll (rad) ,pitch (rad) ,yaw (rad))
			self.vessel.pose = self.vessel.pose + np.array([dlat,dlong,dpos_tangent[2],d_eta[3],d_eta[4],d_eta[5]])
			self.lastt = t
			self.bound_coordinate_limits_deg()
			if self.vessel.pose[5] <0:
				self.vessel.pose[5]+= 2*math.pi
			elif self.vessel.pose[5] >2*math.pi:
				self.vessel.pose[5]+= -2*math.pi
			
			self.state = SimulationState.ready

	def bound_coordinate_limits_deg(self):

		if self.vessel.pose[0] >360:
			self.vessel.pose[0] += -360
		elif self.vessel.pose[0] <0:
			self.vessel.pose[0] += 360

	def bound_coordinate_limits_rad(self):

		if self.vessel.pose[0] >2*math.pi:
			self.vessel.pose[0] += -2*math.pi
		elif self.vessel.pose[0] <0:
			self.vessel.pose[0] += 2*math.pi

class eventlogger:
	""" Logs simulation step events in matrices for debugging and display.
	:param:
	:return: the created object
	"""
	def __init__(self,datalen,t_end,t_start):
		self.datalen = datalen
		self.t_end = t_end
		self.i = 0
		self.t0 = t_start
		self.log_t = np.zeros((self.datalen,1))
		self.log_acc = np.zeros((self.datalen,6))
		self.log_vel = np.zeros((self.datalen,6))
		self.log_pose = np.zeros((self.datalen,6))
		self.log_Ftotal = np.zeros((self.datalen,6))
		self.log_Fd = np.zeros((self.datalen,6))
		self.log_Fc = np.zeros((self.datalen,6))
		self.log_Fact= np.zeros((self.datalen,6))
		self.log_dt = np.zeros((self.datalen,1))
		self.log_u = np.zeros((self.datalen,3))
		self.log_alpha = np.zeros((self.datalen,3))

	def log(self,t,acc,vel,pose,Ft,Fd,Fc,Fact,dt,u,alpha):
		if self.i <self.datalen:
			self.log_t[self.i,:] = t
			self.log_acc[self.i,:] = acc
			self.log_vel[self.i,:] = vel
			self.log_pose[self.i,:] = pose
			self.log_Ftotal[self.i,:] = Ft
			self.log_Fd[self.i,:] = Fd
			self.log_Fc[self.i,:] = Fc
			self.log_Fact[self.i,:] = Fact
			self.log_dt[self.i,:] = dt
			self.log_u[self.i,:] = u
			self.log_alpha[self.i,:] = alpha
			self.i += 1

		elif self.i == self.datalen:
			self.display()
			self.i += 1

	def display(self):
		markersize_set = 2

		axiss = ['x','y','z','rotx','roty','rotz']

		fig, ((axvelx, axvely, axvelyaw), (axfd, axfc, axposyaw)) = plt.subplots(2, 3,figsize=(12, 9))
		fig.suptitle('Ship state')

		axvelx.plot(self.log_t-self.t0, self.log_vel[:,0])
		axvelx.set_ylabel('u (m/s)')
		axvelx.grid()

		axvely.plot(self.log_t-self.t0, self.log_vel[:,1])
		axvely.set_ylabel('v (m/s)')
		axvely.grid()

		axvelyaw.plot(self.log_t-self.t0, self.log_vel[:,5])
		axvelyaw.set_ylabel('r (rad/s)')
		axvelyaw.set_xlabel('time (s)')
		axvelyaw.grid()

		axposyaw.plot(self.log_t-self.t0, self.log_pose[:,5])
		axposyaw.set_ylabel('heading (rad)')
		axposyaw.set_xlabel('time (s)')
		axposyaw.grid()

		for i in range(6):
			axfd.plot(self.log_t-self.t0, self.log_Fd[:,i],label=axiss[i])
		axfd.legend()
		axfd.set_ylabel('Dampening forces (N or N*m)')
		axfd.set_xlabel('time (s)')
		axfd.grid()

		for i in range(6):
			axfc.plot(self.log_t-self.t0, self.log_Fc[:,i],label=axiss[i])
		axfc.legend()
		axfc.set_ylabel('Coriolis centripetal forces (N or N*m)')
		axfc.set_xlabel('time (s)')
		axfc.grid()

		plt.show()

def actuationCallback(msg,args):
	"""
    Actuation references passed through this callback have secondary priority under callback_control_prio 
    :param control_input: Control inputs array
    :return:
    """
	vessel = args

    # Process the message if enough time has passed since last priority actuation message
	if time.time_ns() - vessel.last_actuation_reference_prio_message > ACTUATION_PRIO_MSG_TIMEOUT:
		process_actuation(msg,vessel)


def actuationCallback_prio(msg,args):
	"""
    Control message callback (priority traffic)
    Actuation references passed through this callback have priority over the default actuation topic
    :param control_input: Control inputs array
    :return:
    """
	vessel = args

	process_actuation(msg,vessel)

	vessel.last_actuation_reference_prio_message = time.time_ns()


def process_actuation(msg,args):
	"""
	Sets tito neri actuation from respective ros topic
	
	Note that this funtion is currently hardcoded towards the current Tito Neri actuation array definition. This is not generalized and prone to change. Future versions can generalize this callback structure. 
	
	Future versions will also have this function assign reference that supports limiting actuator rate change. 
	"""
	vessel = args
	# Set aft thrusters
	for i in [0,1]:
		if not math.isnan(msg.data[i]):
			vessel.u[i] = msg.data[i]/60 # convert from rpm to rps
			vessel.u_ref[i] = msg.data[i]/60 # convert from rpm to rps
			
	# Set bow thruster
	if not math.isnan(msg.data[2]):
		vessel.u[2] = msg.data[2]
		vessel.u_ref[2] = msg.data[2]
	
	# Set thruster angles
	for i in [0,1]:
		if not math.isnan(msg.data[i+3]):
			vessel.alpha[i] = msg.data[i+3]
			vessel.alpha_ref[i] = msg.data[i+3]
	
	# Track last reference events of this ship, for timeout purposes
	if vessel.referenceTimedOut:
		print('Reference detected after timeout. Responding to new broadcast')
		vessel.referenceTimedOut = 0

	vessel.last_ref_timestamp = time.time_ns()
	

def vesselModelRun():
	positionPubTimer = timedFncTracker(RATE_PUB_POS)
	headingPubTimer = timedFncTracker(RATE_PUB_HEADING)
	auxillaryStatePubTimer = timedFncTracker(RATE_PUB_STATE_AUXILIARY)

	reportStatusTimer = timedFncTracker(0.5)
	if args.pose0:
		pose_init = args.pose0
	else:
		pose_init = [52.00153943981206, 4.371986684664603,0,0,0,0]
		
	if args.velocity0:
		vel_init = args.velocity0
	else:
		vel_init = [0.3,0.08,0.00,0,0,0.05]
		
	sim = vesselSim(VESSEL_ID,pose_init,vel_init,RATE_SIM)
	
	# Main functional publishers and subscribers
	positionPub = rospy.Publisher(sim.vessel.name+'/state/geopos',NavSatFix, queue_size=0)
	headingPub = rospy.Publisher(sim.vessel.name+'/state/yaw', Float32, queue_size=0)
	actuatorReferenceSub = rospy.Subscriber(sim.vessel.name+'/reference/actuation', Float32MultiArray, actuationCallback,(sim.vessel))
	actuatorReferenceSub_prio = rospy.Subscriber(sim.vessel.name+'/reference/actuation_prio', Float32MultiArray, actuationCallback_prio,(sim.vessel))
	
	# Publishers that communicate diagnostics and system state
	forcePub = rospy.Publisher(sim.vessel.name+'/diagnostics/sim_resultant_forces', Wrench, queue_size=0)
	forcePub_actuator = rospy.Publisher(sim.vessel.name+'/diagnostics/sim_actuator_forces', Wrench, queue_size=0)
	diagnosticsVelocityPub = rospy.Publisher(sim.vessel.name+'/diagnostics/sim_internal_velocities', Twist, queue_size=0)
	actuatorStatePub = rospy.Publisher(sim.vessel.name+'/diagnostics/actuation', Float32MultiArray, queue_size=0)

	rospy.init_node(sim.vessel.name+'_nausbot_geo', anonymous=True)
	
	rate = rospy.Rate(1000) #hz

	while not rospy.is_shutdown():
		if sim.runtimer.isready():
			sim.simstep(sim.runtimer.tlast)
		
		if positionPubTimer.isready():
			# Publish position
			msg = NavSatFix()
			msg.header.seq = positionPubTimer.sequence
			msg.header.stamp = rospy.Time.now()
			msg.header.frame_id = 'world'
			
			msg.latitude = sim.vessel.pose[0]
			msg.longitude = sim.vessel.pose[1]
			msg.altitude = sim.vessel.pose[2]
			positionPub.publish(msg)
		
		if headingPubTimer.isready():
			# Publish heading
			msg = Float32()
			msg.data =  sim.vessel.pose[5]
			headingPub.publish(msg)

		if auxillaryStatePubTimer.isready():
			# Publish auxiliary states

			# Internal velocities (usually not known in a real scenario, but here anyway published for diagnostic purposes)
			msg = Twist()
			msg.linear.x,msg.linear.y,msg.linear.z = sim.vessel.vel[0:3]
			msg.angular.x,msg.angular.y,msg.angular.z = sim.vessel.vel[3:6]
			diagnosticsVelocityPub.publish(msg)

			# Resultant forces:
			msg = Wrench()
			msg.force.x,msg.force.y,msg.force.z = sim.lastF[0:3]
			msg.torque.x,msg.torque.y,msg.torque.z = sim.lastF[3:6]
			forcePub.publish(msg)

			# Resultant forces of actuators:
			msg = Wrench()
			msg.force.x,msg.force.y,msg.force.z = sim.lastF_actuator[0:3]
			msg.torque.x,msg.torque.y,msg.torque.z = sim.lastF_actuator[3:6]
			forcePub_actuator.publish(msg)

			# actuation reference
			msg = Float32MultiArray()
			msg.data = np.concatenate((sim.vessel.u_ref, sim.vessel.alpha_ref), axis=None)
			actuatorStatePub.publish(msg)

		if reportStatusTimer.isready():  
			# Periodic reporting in terminal
			print('['+"{:.3f}".format(sim.runtimer.timeSinceStart())+'] '+sim.vessel.name+' sim seq='+"{:.0f}".format(sim.runtimer.sequence))
			
		rate.sleep()

if __name__ == '__main__':
	try:
		vesselModelRun()
	except rospy.ROSInterruptException:
		pass