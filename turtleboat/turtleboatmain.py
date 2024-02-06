from std_msgs.msg import Float32MultiArray, Float32
import time
import math
from sensor_msgs.msg import NavSatFix 
from geometry_msgs.msg import Wrench, Twist
import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger as TriggerSrv
from sensor_msgs.msg import Imu
from turtleboat.utils import euler_to_quaternion, ActuationState, SimulationState, cross3, R3_euler_xyz, getCoriolisCentripetal, Statuscolors
R_EARTH = 6371000 #m

class Vessel:	
	"""
	Class representing a Tito Neri model scale vessel
	"""
	
	def __init__(self,pose_,vel_):
		"""
		Constructor for the vessel class
		:param name_: name of the vessel
		:param pose_: initial pose of the vessel
		:param vel_: initial velocity of the vessel
		:return: the created object
		"""

		self.pose = np.array(pose_,dtype=np.float64) # [lat long altitude pitch roll yaw] rotations w.r.t. north east down
		self.vel = np.array(vel_,dtype=np.float32) # [u,v,w,p,q,r]

		## Actuator parameters
		# Each thruster (rotatable or not) is seen as a single actuator in this number. Rotatable thrusters are seen as a single actuator with power-input (generally propeller velocity) and an orientation (generally azimuth angle).
		self.ntrh = 3
		
		self.thrustToForce = [lambda v: ((1.925e-5)*v*v*v+(1.061e-2)*v), # output: Newton, Input: RPS
							lambda v: ((1.925e-5)*v*v*v+(1.061e-2)*v), # output: Newton, Input: RPS
							lambda PWM_value: PWM_value*3.575] # output: Newton, Input is normalized pwm [-1:1]
		# alternative purely quadratic relation aft thruster: lambda v: np.sign(v)*0.0009752*v**2,
		
		# Define actuator reference parameters
		self.u_ref = np.zeros(self.ntrh,dtype=np.float32)
		self.alpha_ref = np.zeros(self.ntrh,dtype=np.float32)
	
		# Define limits of actuators
		self.u_lims = np.array([[-60,60],[-60,60],[-1.0,1.0]],dtype=np.float32) # lower and upper bounds of all thruster outputs
		self.alpha_lims = np.array([[-3/4*math.pi,3/4*math.pi],[-3/4*math.pi,3/4*math.pi],[math.pi/2,math.pi/2]],dtype=np.float32) # lower and upper bounds of all thruster angles
		
		# Define maximum rate of change of actuators
		self.u_rate_lim = np.array([120,120,2.0],dtype=np.float32) # [r/s/s,r/s/s,1/s]
		self.alpha_rate_lim = np.array([math.pi*0.70,math.pi*0.70,0.0],dtype=np.float32) # [rad/sec]
		
		# Define initial actuator state
		self.u = np.zeros(3,dtype=np.float32) # initial actuator output
		self.alpha = np.array([0.0,0.0,math.pi/2],dtype=np.float32) # initial actuator orientation

		# Define position of thrusters wrt Center-Origin (not necessarily CG)
		self.r_thruster = np.array([[-0.42,-0.08,0],[-0.42,+0.08,0],[0.28,0.00,0]],dtype=np.float32)
		
		## Size
		self.l = 0.97
		self.w = 0.30

		# the back and front part of the hull wrt CG
		self.ship_rear_x = -0.42 -0.0952
		self.ship_front_x = self.ship_rear_x + self.l

		## Dynamics
		self.D = np.array([		 [2.6416	 ,0		,0		,0		,0		,0		],
									[0		,21.9034	,0		,0		,0		,-1.0952	],
									[0		,0		,0		,0		,0		,0		],
									[0		,0		,0		,0		,0		,0		],
									[0		,0		,0		,0		,0		,0		],
									[0		,-1.0952	,0		,0		,0		,0.525*0.20	 ]],dtype=np.float32)

		self.Mrb = np.array([		[16.9		,0		,0		,0		,0		,0		],
									[0		,16.9	,0		,0		,0		,0		],
									[0		,0		,16.9		,0		,0		,0		],
									[0		,0		,0		,1		,0		,0		],
									[0		,0		,0		,0		,1		,0		],
									[0		,0		,0		,0		,0		,0.51		]],dtype=np.float32)
		# Note that the current diagonals on inertia in pitch and roll direction are 1. These values are not measured or taken up in the current model, but these points have to be nonzero to make this matrix invertible. 

		self.Ma = np.array([		[1.2		,0		,0		,0		,0		,0		],
									[0		,1.2		,0		,0		,0		,0		],
									#[0		,49.2		,0		,0		,0		,0		],
									[0		,0		,0		,0		,0		,0		],
									[0		,0		,0		,0		,0		,0		],
									[0		,0		,0		,0		,0		,0		],
									[0		,0		,0		,0		,0		,1.8		]],dtype=np.float32)
		# Note that the element on location 1,1 has been simplified where 49.2 has been replaced with 1.2kg. 
		# This is done as this element seemed to cause instability in the simulation. 
		# Work is needed to figure out why this happened, and reimplement the actual measured values.
		
		self.cg = np.array([0,0,0],dtype=np.float32)

		self.M = self.Mrb + self.Ma

		# Inverse of inertia matrix (precalculated for speed)
		self.Minv = np.linalg.inv(self.M)
		
		# Variable to track whether this ship has it's reference timed out.
		self.referenceTimedOut = 0  # todo check if this can be removed for good practice / neatness
		self.last_actuation_reference_prio_message = 0
	
	def resetActuatorReferences(self):
		"""
		Sets all actuators references to zero
		:param: none
		:return: none
		"""
		self.u_ref = np.zeros(self.ntrh)
		self.alpha_ref = np.zeros(self.ntrh)

	def calc_f_act(self):
		"""
		Calculates resultant force from all the actuators
		:param: none
		:return: resultant force/torque vector
		"""
		Fres = np.array([0,0,0,0,0,0],dtype=np.float32)
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
			Fthr_i_thrLocal = np.array([self.thrustToForce[nthr](self.u[nthr]),0,0],dtype=np.float32)
			Fthr_i_body = np.matmul(R3_euler_xyz(0,0,self.alpha[nthr]),Fthr_i_thrLocal)
			mi = cross3(self.r_thruster[nthr],Fthr_i_body)

			
			
			# Add up response for each thruster
			Fres = Fres + np.array([Fthr_i_body[0],Fthr_i_body[1],Fthr_i_body[2],mi[0],mi[1],mi[2]],dtype=np.float32)
		return Fres

	def change_actuators(self,dt):
		"""
		Changes all actuators from their current state towards their reference. 
		This function takes into account:
		- Rate limits
		- Absolute limits

			:param: dt (timestep in nanoseconds)
			:return: none
		"""

		for nthr in range(self.ntrh):
			# Adjust actuator intensities (e.g. propeller speeds)
			e = self.u_ref[nthr] - self.u[nthr]
			step_limit = self.u_rate_lim[nthr] * dt

			if abs(e) < step_limit:
				self.u[nthr] = self.u_ref[nthr]
			else:
				self.u[nthr] = self.u[nthr] + math.copysign(1, e)*step_limit

			# Limit actuator intensities
			if self.u[nthr] < self.u_lims[nthr][0]:
				self.u[nthr] = self.u_lims[nthr][0]
			elif self.u[nthr] > self.u_lims[nthr][1]:
				self.u[nthr] = self.u_lims[nthr][1]
			
			# Adjust actuator orientations (e.g. angles of thrusters)
			e = self.alpha_ref[nthr] - self.alpha[nthr]
			step_limit = self.alpha_rate_lim[nthr] * dt
			if abs(e) < step_limit:
				self.alpha[nthr] = self.alpha_ref[nthr]
			else:
				self.alpha[nthr] = self.alpha[nthr] + math.copysign(1, e)*step_limit

			# Limit actuator orientations
			if self.alpha[nthr] < self.alpha_lims[nthr][0]:
				self.alpha[nthr] = self.alpha_lims[nthr][0]
			elif self.alpha[nthr] > self.alpha_lims[nthr][1]:
				self.alpha[nthr] = self.alpha_lims[nthr][1]

	def getCoriolisCentripetal_total(self):
		return getCoriolisCentripetal(self.vel,self.M)

	def getRbn(self):
		return R3_euler_xyz(self.pose[3],self.pose[4],self.pose[5])
	
	def bound_coordinate_limits(self):
		"""
		Limits the vessel's lat/longitude and orientations to be in the 0-360deg or 0-2pi range
		"""

		# Bound longitude
		if self.pose[0] >360:
			self.pose[0] += -360
		elif self.pose[0] <0:
			self.pose[0] += 360

		# Bound latitude
		if self.pose[1] >360:
			self.pose[1] += -360
		elif self.pose[1] <0:
			self.pose[1] += 360

		# Not bounding altitude (self.pose[2])

		# Bound orientation: pitch
		if self.pose[3] >2*math.pi:
			self.pose[3] += -2*math.pi
		elif self.pose[3] <0:
			self.pose[3] += 2*math.pi

		# Bound orientation: roll
		if self.pose[4] >2*math.pi:
			self.pose[4] += -2*math.pi
		elif self.pose[4] <0:
			self.pose[4] += 2*math.pi

		# Bound orientation: yaw
		if self.pose[5] >2*math.pi:
			self.pose[5] += -2*math.pi
		elif self.pose[5] <0:
			self.pose[5] += 2*math.pi

class VesselSimNode(Node):
	def __init__(self):
		super().__init__('turtleboat_sim')
		
		self.declare_parameters(
            namespace='',
            parameters=[
                ('simulator_frequency_target', 400.0),
				('rate_publish_position', 5.0),
				('rate_publish_heading', 16.0),
				('rate_publish_auxiliary_state', 10.0),
				('stream_auxiliary_state', False),
				('initial_pose', [52.00140854178, 4.37186309232,0.0,0.0,0.0,math.pi/4]),
				('initial_velocity', [np.random.uniform(-0.4,0.4),np.random.uniform(-0.2,0.2),0.00,0.0,0.0,np.random.uniform(-0.05,0.05)]),
				('imu_enabled', True),
				('reference_runtime_timeout', 5.0),
				('period_report_status', 2.0)
				]
        )

		# Set up vessel object
		self.vessel = Vessel(self.get_parameter("initial_pose").get_parameter_value().double_array_value,self.get_parameter("initial_velocity").get_parameter_value().double_array_value)
	
		# Set up event scheduling
		self.timestamp_start = time.time()
		self.timestamp_last_simstep = time.time()
		self.timestamp_status_report = time.time()
		self.timestamp_last_actuator_ref_callback= time.time()
		self.timestamp_last_pos_publish = time.time()
		self.timestamp_last_auxiliary_publish = time.time()
		self.resetTrackers()

		# Set system state
		self.simulationState = SimulationState.initializing
		self.actuationState = ActuationState.timeout

		# Make ros2 publishers and subscribers for main functionality
		self.positionPub = self.create_publisher(NavSatFix, 'telemetry/gnss/fix', 10)
		self.headingPub = self.create_publisher(Float32, 'telemetry/heading', 10)
		self.actuatorReferenceSub_prio = self.create_subscription(Float32MultiArray, 'reference/actuation_prio',self.actuationCallback_prio,10)
		self.actuatorReferenceSub = self.create_subscription(Float32MultiArray, 'reference/actuation',self.actuationCallback,10)
	
		# Optional publishers that communicate diagnostics and system state
		if self.get_parameter("stream_auxiliary_state").get_parameter_value().bool_value:
			self.forcePub_resultant = self.create_publisher(Wrench, 'diagnostics/sim_state/f_resultant', 10)
			self.forcePub_actuator = self.create_publisher(Wrench, 'diagnostics/sim_state/f_actuator', 10)
			self.forcePub_drag = self.create_publisher(Wrench, 'diagnostics/sim_state/f_drag', 10)
			self.forcePub_corioliscentripetal = self.create_publisher(Wrench, 'diagnostics/sim_state/f_corioliscentripetal', 10)
			self.velocityPub = self.create_publisher(Twist, 'diagnostics/sim_state/velocity', 10)
			self.actuatorStatePub = self.create_publisher(Float32MultiArray, 'diagnostics/sim_state/actuator_state', 10)
			self.actuatorRefPub = self.create_publisher(Float32MultiArray, 'diagnostics/sim_state/actuator_reference', 10)
		
		if self.get_parameter('imu_enabled').value:
			self.imuPub = self.create_publisher(Imu, 'telemetry/imu', 10)

		# Create timer objects
		self.timer_simstep = self.create_timer(1/self.get_parameter("simulator_frequency_target").get_parameter_value().double_value, self.timer_callback_simstep)
		self.timer_publish_pos = self.create_timer(1/self.get_parameter("rate_publish_position").get_parameter_value().double_value, self.timer_callback_publish_pos)
		self.timer_publish_heading = self.create_timer(1/self.get_parameter("rate_publish_heading").get_parameter_value().double_value, self.timer_callback_publish_heading)
		self.timer_report_status = self.create_timer(self.get_parameter("period_report_status").get_parameter_value().double_value, self.timer_callback_report_status)
		if self.get_parameter("stream_auxiliary_state").get_parameter_value().bool_value:
			self.timer_publish_auxiliary = self.create_timer(1/self.get_parameter("rate_publish_auxiliary_state").get_parameter_value().double_value, self.timer_callback_publish_auxiliary)


		# Make ros2 service reset pose and velocity from EmptySrv type
		self.srv_reset_pose_and_velocity = self.create_service(TriggerSrv, '/service/reset_pose_and_velocity',self.srv_reset_pose_and_velocity)

	def srv_reset_pose_and_velocity(self,request,response:TriggerSrv.Response):
		"""
		Runs when a request is made to reset the pose and velocity of the vessel
		"""
		self.vessel.pose = np.array(self.get_parameter("initial_pose").get_parameter_value().double_array_value)
		self.vessel.vel = np.array(self.get_parameter("initial_velocity").get_parameter_value().double_array_value)
		response.success = True
		return response

	def timer_callback_simstep(self):
		"""
		Perform one simulation step of the vessel
		"""
		
		self.tracker_iteration_simstep += 1
		
		# Calculate time since last simstep
		now = time.time()
		dt = now - self.timestamp_last_simstep
		
		if self.simulationState == SimulationState.ready:
			self.simulationState = SimulationState.busy # block access to this function until done
			
			# Actuator dynamics
			self.check_actuator_reference_timeout()
			self.vessel.change_actuators(dt)

			# Calculate forces and torques
			self.Fd = -1*np.matmul(self.vessel.D,self.vessel.vel)	 # Drag (e.g. linear or quadratic)
			self.Fc = -1*np.matmul(self.vessel.getCoriolisCentripetal_total(),self.vessel.vel)	# Coriolis & Centripetal
			self.Fact = self.vessel.calc_f_act() # Actuators (fins, rudders, propellers)
			self.Ftotal = self.Fd + self.Fc + self.Fact

			# Accelleration
			nu_dot = np.matmul(self.vessel.Minv,self.Ftotal)

			# Velocities
			dvel = nu_dot*dt
			self.vessel.vel = self.vessel.vel + dvel

			# Displacement(cartesian body fixed coordinate system)
			d_eta = self.vessel.vel*np.array([dt])

			# Convert to North-east-down tangent displacements: (cartesian local coordinate system)
			dpos_tangent = np.matmul(self.vessel.getRbn(),d_eta[0:3])

			# Conversion from displacement in meters (d_north, d_east) to geographical (d_latitude,d_longitude)
			dlat =np.rad2deg(np.arctan2(dpos_tangent[0],R_EARTH)) # degrees
			r_earth_at_lat = np.cos(np.deg2rad(self.vessel.pose[0]))*R_EARTH	# Radius of slice of earth at particular latitude
			dlong = np.rad2deg(np.arctan2(dpos_tangent[1],r_earth_at_lat))

			# Set new position (long (deg) ,lat (deg) ,altitude (m) ,roll (rad) ,pitch (rad) ,yaw (rad))
			self.vessel.pose = self.vessel.pose + np.array([dlat,dlong,dpos_tangent[2],d_eta[3],d_eta[4],d_eta[5]])

			# Bound coordinate and orientation limits
			self.vessel.bound_coordinate_limits()
			
			# Unblock this function
			self.simulationState = SimulationState.ready
		
		elif self.simulationState == SimulationState.initializing:
			# On the first iteration the timestap since last simultation iteration needs to be reset due to startup delay
			# This avoids having a huge dt on the first iteration
			self.timestamp_last_simstep = time.time()
			self.simulationState = SimulationState.ready
		
		# Set the timestamp of the last simulation iteration to the current time for the next iteration
		self.timestamp_last_simstep = now

	def timer_callback_publish_pos(self):
		"""
		Publishes the position of the vessel
		"""
		# add one to the frequency tracker
		self.tracker_callback_pub_pos += 1
		
		# Create message
		msg = NavSatFix()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.header.frame_id = 'world'
		
		msg.latitude = self.vessel.pose[0]
		msg.longitude = self.vessel.pose[1]
		msg.altitude = self.vessel.pose[2]
		
		# Publish message
		self.positionPub.publish(msg)
		
	def timer_callback_publish_heading(self):
		"""
		Publishes the heading of the vessel
		"""
		# add one to the frequency tracker
		self.tracker_pub_heading += 1
		
		# Create message
		msg = Float32()
		msg.data =  self.vessel.pose[5]
		
		# Publish message
		self.headingPub.publish(msg)

		if self.get_parameter('imu_enabled').value:
			msg_imu = Imu()
			msg_imu.header.stamp = self.get_clock().now().to_msg()
			msg_imu.header.frame_id = 'world'
			msg_imu.orientation_covariance = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
			msg_imu.angular_velocity_covariance = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
			msg_imu.linear_acceleration_covariance = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
			quats = euler_to_quaternion(self.vessel.pose[3],self.vessel.pose[4],self.vessel.pose[5])
			msg_imu.orientation.x = quats[0]
			msg_imu.orientation.y = quats[1]
			msg_imu.orientation.z = quats[2]
			msg_imu.orientation.w = quats[3]
			self.imuPub.publish(msg_imu)

	def timer_callback_report_status(self):
		self.print_status()
		self.resetTrackers()
		
	def actuationCallback_prio(self,msg):
		"""
		Runs when an update is published on the priority actuation topic of this vessel.
		This stream has priority over the other normal actuation topic (generally in case of emergency / joystick override). 
		"""
		self.process_actuation(msg)
		self.actuationState = ActuationState.priority

	def actuationCallback(self,msg):
		"""
		Runs when an update is published on the normal actuation topic of this vessel
		"""
		if not self.actuationState == ActuationState.priority:
			self.process_actuation(msg)
			self.actuationState = ActuationState.normal

	def check_actuator_reference_timeout(self):

		if not self.actuationState == ActuationState.timeout:
			if time.time() -self.timestamp_last_actuator_ref_callback > self.get_parameter("reference_runtime_timeout").get_parameter_value().double_value:
				self.actuationState = ActuationState.timeout
				print('Actuation reference timed out')
				stopmsg = Float32MultiArray()
				stopmsg.data = [0.0,0.0,0.0,0.0,0.0]
				self.process_actuation(stopmsg)
		
	def process_actuation(self,msg):
		"""
		Sets tito neri actuation from respective ros topic
		
		Note that this funtion is currently hardcoded towards the current Tito Neri actuation array definition. This is not generalized and prone to change. Future versions can generalize this callback structure. 
		
		Future versions will also have this function assign reference that supports limiting actuator rate change. 
		"""
		if self.actuationState == ActuationState.timeout:
			print('Detected new actuation stream; Listening to commands')
		self.timestamp_last_actuator_ref_callback = time.time()
		self.tracker_callback_actuator_reference += 1

		# Set aft thrusters
		for i in [0,1]:
			if not math.isnan(msg.data[i]):
				self.vessel.u_ref[i] = msg.data[i]/60 # convert from rpm to rps
				
		# Set bow thruster
		if not math.isnan(msg.data[2]):
			self.vessel.u_ref[2] = msg.data[2]
		
		# Set thruster angles
		for i in [0,1]:
			if not math.isnan(msg.data[i+3]):
				self.vessel.alpha_ref[i] = msg.data[i+3]
		
		
		self.timestamp_last_actuator_ref_callback= time.time()
	
	def print_status(self):
		period_report_status = self.get_parameter("period_report_status").get_parameter_value().double_value
		# Determine system frequencies rounded to two decimals
		freq_callback_reference = round(self.tracker_callback_actuator_reference/period_report_status,2)
		freq_callback_simstep = round(self.tracker_iteration_simstep/period_report_status,2)
		freq_callback_pub_pos = round(self.tracker_callback_pub_pos/period_report_status,2)
		freq_callback_pub_heading = round(self.tracker_pub_heading/period_report_status,2)
		freq_callback_pub_aux = round(self.tracker_callback_pub_auxiliary/period_report_status,2)
		
		# Make strings of numbers without color if above zero and and rascolors.FAIL otherwise
		freq_sim_step_str = Statuscolors.OKGREEN +str(freq_callback_simstep)  + Statuscolors.NORMAL if freq_callback_simstep > 0 else Statuscolors.FAIL + str(freq_callback_simstep) + Statuscolors.NORMAL
		freq_pub_pos_str = Statuscolors.OKGREEN +str(freq_callback_pub_pos)  + Statuscolors.NORMAL if freq_callback_pub_pos > 0 else Statuscolors.FAIL + str(freq_callback_pub_pos) + Statuscolors.NORMAL
		freq_pub_heading_str = Statuscolors.OKGREEN +str(freq_callback_pub_heading)  + Statuscolors.NORMAL if freq_callback_pub_heading > 0 else Statuscolors.FAIL + str(freq_callback_pub_heading) + Statuscolors.NORMAL
		freq_pub_aux_str = Statuscolors.OKGREEN +str(freq_callback_pub_aux)  + Statuscolors.NORMAL if freq_callback_pub_aux > 0 else Statuscolors.FAIL + str(freq_callback_pub_aux) + Statuscolors.NORMAL
		freq_actuator_reference_str = Statuscolors.OKGREEN +str(freq_callback_reference)  + Statuscolors.NORMAL if freq_callback_reference > 0 else Statuscolors.FAIL + str(freq_callback_reference) + Statuscolors.NORMAL

		# print vessel name in blue following time, frequencies of mainloop, simstep and actuator reference
		statusstring = 	' f_sim='+freq_sim_step_str+ \
						' f_actuator_ref='+freq_actuator_reference_str+ \
						' f_pub_pos='+freq_pub_pos_str+ \
						' f_pub_heading='+freq_pub_heading_str
		
		if self.get_parameter("stream_auxiliary_state").get_parameter_value().bool_value:
			self.get_logger().info(statusstring+ ' f_pub_aux='+freq_pub_aux_str)
		else:
			self.get_logger().info(statusstring)

	def resetTrackers(self):
		self.tracker_iteration_simstep = 0
		self.tracker_callback_pub_pos = 0
		self.tracker_pub_heading = 0
		self.tracker_callback_pub_auxiliary = 0
		self.tracker_callback_actuator_reference = 0

	def timer_callback_publish_auxiliary(self):
		"""
		Publishes auxiliary states of the vessel such as forces, velocities, actuator states, etc.
		These states are typically not visible with a real ship, but can be useful for diagnostics and debugging.
		"""
		self.tracker_callback_pub_auxiliary += 1

		# Internal velocities (usually not known in a real scenario, but here anyway published for diagnostic purposes)
		msg = Twist()
		msg.linear.x,msg.linear.y,msg.linear.z = self.vessel.vel[0:3]
		msg.angular.x,msg.angular.y,msg.angular.z = self.vessel.vel[3:6]
		self.velocityPub.publish(msg)

		# Resultant forces:
		msg = Wrench()
		msg.force.x,msg.force.y,msg.force.z = self.Ftotal[0:3]
		msg.torque.x,msg.torque.y,msg.torque.z = self.Ftotal[3:6]
		self.forcePub_resultant.publish(msg)

		# Resultant forces of actuators:
		msg = Wrench()
		msg.force.x,msg.force.y,msg.force.z = self.Fact[0:3]
		msg.torque.x,msg.torque.y,msg.torque.z = self.Fact[3:6]
		self.forcePub_actuator.publish(msg)

		# actuation reference
		msg = Float32MultiArray()
		msg.data = [self.vessel.u_ref[0]*60,self.vessel.u_ref[1]*60,self.vessel.u_ref[2], self.vessel.alpha_ref[0], self.vessel.alpha_ref[1], self.vessel.alpha_ref[2]]
		self.actuatorRefPub.publish(msg)

		# actuation state
		msg = Float32MultiArray()
		msg.data = [self.vessel.u[0].astype(float)*60,self.vessel.u[1].astype(float)*60,self.vessel.u[2].astype(float), self.vessel.alpha[0], self.vessel.alpha[1], self.vessel.alpha[2]]
		self.actuatorStatePub.publish(msg)

		# drag
		msg = Wrench()
		msg.force.x,msg.force.y,msg.force.z = self.Fd[0:3]
		msg.torque.x,msg.torque.y,msg.torque.z = self.Fd[3:6]
		self.forcePub_drag.publish(msg)

		# coriolis centripetal forces
		msg = Wrench()
		msg.force.x,msg.force.y,msg.force.z = self.Fc[0:3]
		msg.torque.x,msg.torque.y,msg.torque.z = self.Fc[3:6]
		self.forcePub_corioliscentripetal.publish(msg)

def main(args=None):
	rclpy.init(args=args)

	sim = VesselSimNode()

	# Start the nodes processing thread
	rclpy.spin(sim)

	# at termination of the code (generally with ctrl-c) Destroy the node explicitly
	sim.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()