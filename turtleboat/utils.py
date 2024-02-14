from enum import Enum, auto
import numpy as np
from sensor_msgs.msg import JointState

def get_value_from_jointstate(message:JointState,name_item:str,paramtype:int=0):
	""" looks through the jointstate messsage for the specified item 
	
	:param: message = JointState message
	:param: name_item = name of the item to look for
	:param: paramtype = 0 for position, 1 for velocity, 2 for effort
	
	"""
	for i in range(0, len(message.name) ):
		if message.name[i] == name_item:
			# The specified item has been found in the list. 
			# Return the value according to the specified paramtype
			if paramtype == 0:
				return message.position[i]
			elif paramtype == 1:
				return message.velocity[i]
			elif paramtype == 2:
				return message.effort[i]
			else:
				# In case the specified paramtype is not valid
				return np.nan
	
	# In case the specified item is not in the list
	return np.nan

def euler_to_quaternion(roll, pitch, yaw):
	"""
	Converts euler angles to quaternions
	:param: roll, pitch, yaw = euler angles in radians
	:return: quaternion
	"""
	qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
	qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
	qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	return [qx, qy, qz, qw]

class ActuationState(Enum):
	"""
	Object for managing actuator reference status in a readable manner. 
	:param: none
	:return: the created object
	"""
	timeout = auto()
	normal = auto()
	priority = auto()

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

class Statuscolors:
	"""
	Color lookup table for terminal output
	"""

	# Status colors
	# ----------------
	HEADER = '\033[95m'
	OKBLUE = '\033[94m'
	OKCYAN = '\033[96m'
	OKGREEN = '\033[92m'
	WARNING = '\033[93m'
	FAIL = '\033[91m'
	NORMAL = '\033[0m'


def cross3(a:np.ndarray,b:np.ndarray):

	""" 
	Calculates the cross product of two 3D vectors
	"""
	return np.array([a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],dtype=np.float32)

def getCoriolisCentripetal(v:np.ndarray,M:np.ndarray):
	"""
	Forms coriolis-centripetal matrix from velocity vector and inertial matrix according to the parameterization of Fossen's Handbook of marine control (2011) eq. 6.43
	:param: v 1x6 velocity vector [u,v,w,p,q,r]
	:param: M 6x6 Inertial matrix
	:return: 6x6 coriolis matrix
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

def skew(v:np.ndarray):
	"""
	Forms skew symmetric matrix from vector
	:param: v 1x3 input vector
	:return: 3x3 skew symmetric matrix
	"""
	return np.array([	[0		,-v[2]	,v[1]	],
						[v[2]		,0		,-v[0]],
						[-v[1]	,v[0]		,0	]],dtype=np.float32)

def R3_euler_xyz(roll,pitch,yaw):
	"""
	Forms rotation matrix from euler angles in xyz order
	:param: roll, pitch, yaw = euler angles in radians
	:return: 3x3 rotation matrix
	"""

	c1 = np.cos(roll)
	s1 = np.sin(roll)
	c2 = np.cos(pitch)
	s2 = np.sin(pitch)
	c3 = np.cos(yaw)
	s3 = np.sin(yaw)

	return np.array([	[c2*c3, 			-c2*s3, 			s2], 		\
				   		[c1*s3+c3*s1*s2, 	c1*c3-s1*s2*s3, 	-c2*s1], 	\
						[s1*s3-c1*c3*s2, 	c3*s1+c1*s2*s3, 	c1*c2]]  	,dtype=np.float32)
	 