#!/usr/bin/python

# * Simplified complex trajectory tracking in OFFBOARD mode
# *
# * Author: Sean Smith <s.smith@dal.ca>
# *
# * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# * The above copyright notice and this permission notice shall be included in all
# * copies or substantial portions of the Software.


import rospy
from clover import srv
from std_srvs.srv import Trigger
import math
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
import tf
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

import numpy as np

# Could plot the stored data in SITL (not hardware) if desired:
import matplotlib.pyplot as plt


from time import sleep
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *



# Define the Clover service functions, the only ones used in this application are navigate and land.

get_telemetry = rospy.ServiceProxy('/clover1/get_telemetry', srv.GetTelemetry)
get_Leader = rospy.ServiceProxy('/clover0/get_telemetry', srv.GetTelemetry)  # have access to the leaders position
navigate = rospy.ServiceProxy('/clover1/navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('/clover1/navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('/clover1/set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('/clover1/set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('/clover1/set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('/clover1/set_rates', srv.SetRates)
land = rospy.ServiceProxy('/clover1/land', Trigger)

# Release service is used to allow for complex trajectory publishing i.e it stops the navigate service from publishing setpoints because you dont want two sources of publishing at the same time.
release = rospy.ServiceProxy('/clover1/simple_offboard/release', Trigger)

# Deine math parameter
PI_2 = math.pi/2

# Debugging and logging
xf = []
yf = []
xdispf=[]
ydispf=[]
xL = []
yL = []
YawL = []

# Analyze control input (see if error is being minimized since its difficult to compare inertial velocity to body frame velocity)
velfx=[]
velfy=[]
ex=[]
ey=[]
eyaw=[]


class clover:

	def __init__(self, dx_des, dy_des, REF_FRAME):
	
		self.mode = "idle" # Initialize state of follower
		self.is_tracking_activated = False
		self.pose_data = None
		self.released_flag = False # Flag to track if the release() function has been called
		self.track = PositionTarget()  # For publishing complex trajectory in inertial frame
		self.trackbody = PositionTarget() # For publishing complex trajectory tracking in body frame
		
		self.last_timestamp = None # make sure there is a timestamp
		
		self.subscriber = rospy.Subscriber("leader_commands",String, self.command_callback)
		
		self.leader = rospy.Subscriber("follower", PositionTarget, self.pose_callback) # Subscribe to leader publishing (dont really need to subscribe to this publishing)
		
		# Publisher which will publish to the topic '/clover1/mavros/setpoint_raw/local'. This has a PositionTarget message type: link
		self.publisher = rospy.Publisher('/clover1/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
		
		# The get_telemety does not retreive body frame attitude or position :( therefore we need to get it from another topic
		self.body = rospy.Subscriber('/clover1/mavros/imu/data',Imu, self.updateIMU)
		
		# The get_telemety does not retreive body frame velocity :( therefore we need to get it from another topic
		self.vel_body = rospy.Subscriber('/clover1/mavros/local_position/velocity_body',TwistStamped, self.update_velbody)
		
		# Initialize control parameters here
		self.d_xid = dx_des
		self.d_yid = dy_des
		self.FRAME = REF_FRAME  # Reference frame for complex trajectory tracking
		
		self.dt = 1/50 # 50 hz controller
		self.SX_int = np.array([[0],[0],[0]],dtype=float) # Initialize integrated experssion to zero
		
		self.sign_sx_int = np.array([[0],[0],[0]],dtype=float) # Initialize integrated experssion to zero
		
		self.V_total = np.array([[0],[0],[0]],dtype=float) # Initialize control input to zero
		
		# initialize terminal sliding mode control gains
		# p and q are positive odd integeters and 1< p/q <2 must be satisfied to avoid the singularity
		self.p = 5.0
		self.q = 3.0
		
		

	def state(self,msg):
		self.mode = msg
		
	def updateIMU(self, msg):
        	# extract quaternion components from msg
		quaternion = [msg.orientation.x, msg.orientation.y,msg.orientation.z, msg.orientation.w]
        	
		euler_angles = euler_from_quaternion(quaternion) # Body frame angles (using IMU)
        	
        	# Extract yaw angle (in radians)
        	
        	# Gather yaw for publishing (in body frame)
		#self.Psi_F = euler_angles[2]
		
		self.Psi_dot_F = msg.angular_velocity.z  # yaw_rate in body frame
		
	def update_velbody(self, data):
		t = data
		
		# Gather velocity for publishing (in the body frame)
		self.follow_velocity_x = data.twist.linear.x
		self.follow_velocity_y = data.twist.linear.y
		self.follow_velocity_z = data.twist.linear.z
		
	def controller(self,data):
	
		current_timestamp = data.header.stamp.to_sec()
		
		if self.last_timestamp is not None:
		
			
		
			dt = current_timestamp - self.last_timestamp

		# Position commands (received in the inertial frame)
			self.X_L = data.position.x
			self.Vx_L = data.velocity.x
			self.Y_L = data.position.y
			self.Vy_L = data.velocity.y
			self.Psi_L = data.yaw
			self.Psi_dot_L = data.yaw_rate
		
			# Get current state of the follower 
			telem = get_telemetry(frame_id='map')
		
		
			# Calculate current distance from the leader
			self.d_xi = -(self.X_L - telem.x)*math.cos(self.Psi_L) - (self.Y_L - telem.y)*math.sin(self.Psi_L)
			self.d_yi = (self.X_L - telem.x)*math.sin(self.Psi_L) - (self.Y_L - telem.y)*math.cos(self.Psi_L)
		
			# Current position of follower
			self.X_i = self.X_L + self.d_xi*math.cos(self.Psi_L) - self.d_yi*math.sin(self.Psi_L)
			self.Y_i = self.Y_L + self.d_xi*math.sin(self.Psi_L) + self.d_yi*math.cos(self.Psi_L)
			
			# Desired position of the follower (feed this in to the position controller)
			self.X_id = self.X_L + self.d_xid*math.cos(self.Psi_L) - self.d_yid*math.sin(self.Psi_L)
			self.Y_id = self.Y_L + self.d_xid*math.sin(self.Psi_L) + self.d_yid*math.cos(self.Psi_L)
		
			# calculate error
			self.e_psi = telem.yaw - self.Psi_L
			#self.e_psi = self.Psi_F - self.Psi_L
		
			self.e_Xi = self.d_xid - self.d_xi
			self.e_Yi = self.d_yid - self.d_yi
		
			# translational dynamic matrix formulation
			G_X = np.array([[-math.cos(self.e_psi), math.sin(self.e_psi), 0],[-math.sin(self.e_psi),-math.cos(self.e_psi), 0],[0, 0, 1]],dtype=float)
		
			F_X = np.array([[self.e_Yi*self.Psi_dot_L + self.Vx_L - self.Psi_dot_L*self.d_yid],[-self.e_Xi*self.Psi_dot_L + self.Vy_L + self.Psi_dot_L*self.d_xid],[self.e_psi]],dtype=float)
		
			SX_i = np.array([[self.e_Xi],[self.e_Yi],[self.e_psi]],dtype=float)
		
			# Integral portion:
		
			# Get the current 
		
			self.SX_int = self.SX_int + SX_i*dt
		 
		
			# Derivative
			#mu = np.array([[self.V_total[0,0]],[self.V_total[1,0]],[self.V_total[2,0]]],dtype=float) # state of the follower from EKF
			mu = np.array([[self.follow_velocity_x],[self.follow_velocity_y],[self.Psi_dot_F]],dtype=float)
			
			# Progress dynamic equation
			SX_dot = F_X + np.matmul(G_X,mu)
		
			Tau_1 = np.array([[2.0],[2.0],[1.0]],dtype=float) # gain matrix 1
			Tau_2 = np.array([[2.0],[2.0],[1.0]],dtype=float) # gain matrix 2
			# Super twisting sliding mode controller gains
			eta_1 = np.array([[2.0],[2.0],[1.5]],dtype=float) # gain matrix 1
			eta_2 = np.array([[2.0],[2.0],[1.5]],dtype=float) # gain matrix 1
		
			# Sliding surface
			#s_i = SX_i + Tau_1*self.SX_int + Tau_2*np.power(SX_i,(self.p/self.q)) 
			
			s_i = SX_i + Tau_1*self.SX_int # Simple SMC sliding surface
		
			self.sign_sx_int = self.sign_sx_int + np.tanh(s_i)*dt
		
			# equivalent control
		
			#self.Vx_i = np.linalg.inv(G_x)
			#V_eq = np.matmul(np.linalg.inv(G_X),(-F_X - Tau_1*SX_i - Tau_2*(self.p/self.q)*np.power(SX_i,(self.p/self.q)-1)*SX_dot))
			#V_sw = -eta_1*np.power(np.abs(s_i),0.5)*np.tanh(s_i) - eta_2*self.sign_sx_int
			
			#V_total = V_eq + V_sw
			#V_total = np.matmul(np.linalg.inv(G_X),(-F_X - Tau_1*SX_i - Tau_2*(self.p/self.q)*np.power(SX_i,(self.p/self.q)-1)*SX_dot-eta_1*np.power(np.abs(s_i),0.5)*np.tanh(s_i) - eta_2*self.sign_sx_int))
			
			# Simple SMC for testing
			self.V_total = np.matmul(np.linalg.pinv(G_X),(-F_X - Tau_1*SX_i - eta_2*np.tanh(s_i))) 
			#V_total = -F_X - Tau_1*SX_i - eta_2*np.tanh(s_i)
			
			# The G_X matrix transforms the commands from the leaders frame to the followers frame. Lets transform them 
			# to the inertial frame with the rotation matrix and followers yaw
			Rot = np.array([[math.cos(telem.yaw), -math.sin(telem.yaw)],[math.sin(telem.yaw),math.cos(telem.yaw)]],dtype=float)
			
			Vel_folframe = np.matmul(Rot,np.array([[self.V_total[0,0]],[self.V_total[1,0]]],dtype=float))
			
			#-------Inertial Frame-------------------------------------------------------------------------
			self.track.header.frame_id = 'map'  # Define the frame that will be used
			
			self.track.coordinate_frame = 1 #MAV_FRAME_BODY_NED  # =8
			self.track.type_mask = 32+1024 #64+128+256+1024
			#self.track.type_mask = 8+16+32+64+128+256+1024+2048  # ignore accelerations! and velocity and yaw/yawrate
						# PositionTarget::IGNORE_VX +
						# PositionTarget::IGNORE_VY +
						# PositionTarget::IGNORE_VZ +
						# PositionTarget::IGNORE_AFX +
						# PositionTarget::IGNORE_AFY +
						# PositionTarget::IGNORE_AFZ +
						# PositionTarget::IGNORE_YAW;

			# Gather position for publishing
			self.track.position.x = self.X_id
			self.track.position.y = self.Y_id
			self.track.position.z = data.position.z        
			
			# Gather velocity for publishing
			#self.track.velocity.x = data.velocity.x
			#self.track.velocity.y = data.velocity.y
			self.track.velocity.x = Vel_folframe[0,0]
			self.track.velocity.y = Vel_folframe[1,0]
			#self.track.velocity.x = self.V_total[0,0]
			#self.track.velocity.y = self.V_total[1,0]
			#self.track.velocity.z = data.velocity.z
			
			# Gather acceleration for publishing
			self.track.acceleration_or_force.x = data.acceleration_or_force.x
			self.track.acceleration_or_force.y = data.acceleration_or_force.y
			self.track.acceleration_or_force.z = data.acceleration_or_force.z
			
			# Gather yaw for publishing
			#self.track.yaw = data.yaw
			#self.track.yaw=math.atan2(telem.vy, telem.vx)
			
			# Gather yaw rate for publishing
			#self.track.yaw_rate = data.yaw_rate
			self.track.yaw_rate = self.V_total[2]
			
			# Publish to the setpoint topic
			
			#print(V_total[0])
			#print(type(V_total[0]))
			#print(V_total, np.linalg.pinv(G_X))
			self.publisher.publish(self.track)
			# logging/debugging
			xf.append(self.X_id)
			yf.append(self.Y_id)
			xL.append(self.X_L)
			yL.append(self.Y_L)
			ex.append(self.e_Xi)
			ey.append(self.e_Yi)
			eyaw.append(self.e_psi)
			YawL.append(self.Psi_L*(180/math.pi))
			xdispf.append(self.d_xi)
			ydispf.append(self.d_yi)
			velfx.append(Vel_folframe[0,0])
			velfy.append(Vel_folframe[1,0])
			
			
		#-------------body frame-----------------------------------------------------------------
		
			self.trackbody.header.frame_id = 'body'  # Define the frame that will be used
			
			self.trackbody.coordinate_frame = 8 #MAV_FRAME_BODY_NED  # =8
			
			self.trackbody.type_mask = 1+2+4+32+64+128+256+1024 #ignore accelerations and position
						# PositionTarget::IGNORE_VX +
						# PositionTarget::IGNORE_VY +
						# PositionTarget::IGNORE_VZ +
						# PositionTarget::IGNORE_AFX +
						# PositionTarget::IGNORE_AFY +
						# PositionTarget::IGNORE_AFZ +
						# PositionTarget::IGNORE_YAW;     
			
			# Gather velocity for publishing
			self.trackbody.velocity.x = self.V_total[0,0]
			self.trackbody.velocity.y = self.V_total[1,0]
			self.trackbody.velocity.z = data.velocity.z
			
			# Gather acceleration for publishing
			#self.track.acceleration_or_force.x = data.acceleration_or_force.x
			#self.track.acceleration_or_force.y = data.acceleration_or_force.y
			#self.track.acceleration_or_force.z = data.acceleration_or_force.z
			
			# Gather yaw for publishing
			self.trackbody.yaw = data.yaw
			
			# Gather yaw rate for publishing
			#self.track.yaw_rate = data.yaw_rate
			self.trackbody.yaw_rate = self.V_total[2]
			
			# Publish to the setpoint topic
			
			#print(V_total[0])
			#print(type(V_total[0]))
			#print(V_total, np.linalg.pinv(G_X))
			#self.publisher.publish(self.trackbody)
		
		# Update last_timestamp for the next callback
		self.last_timestamp = current_timestamp


	def command_callback(self,mode_msg):
		# Check if the recived flight mode is different from the current mode
		if mode_msg.data != self.mode:
		# Update the flight mode
			self.mode = mode_msg.data
		
			if self.mode == "hover":
				self.activate_hover()
				self.is_tracking_activated = False
			elif self.mode == "track":
				self.activate_track()
			elif self.mode == "land":
				self.activate_land()
				self.is_tracking_activated = False
			elif self.mode == "idle":
				rospy.loginfo("Clover Idle")
			else:
				rospy.logwarn("Unknown command received.")
	
	def pose_callback(self, pose):
	
		if self.mode == "track" and self.is_tracking_activated == True:
			if not self.released_flag:
				release()
				self.released_flag = True
			self.controller(pose)
			
	def activate_hover(self):
		telem_i = get_telemetry(frame_id='map')  # Current position of follower_drone
		telem_L = get_Leader(frame_id='map')     # Current position of Leader_drone
		# Define the performance parameters here which starts the script
		dx = -(telem_L.x - telem_i.x)*math.cos(telem_L.yaw) - (telem_L.y - telem_i.y)*math.sin(telem_L.yaw)
		
		dy = (telem_L.x - telem_i.x)*math.sin(telem_L.yaw) + (telem_L.y - telem_i.y)*math.cos(telem_L.yaw)
		
		# desired position commands (basically just maintains starting displacements)
		X_i = telem_L.x - dx*math.cos(telem_L.yaw) - dy*math.sin(telem_L.yaw)
		Y_i = telem_L.y + dx*math.sin(telem_L.yaw) - dy*math.cos(telem_L.yaw)
		
		navigate(x = X_i, y= Y_i, z=1, frame_id='map',auto_arm=True)
		
	def activate_track(self):
		rospy.loginfo('Activated trajectory tracking')
		self.is_tracking_activated = True
		
	def activate_land(self):
		land()
		rospy.loginfo('Activated landing')
		sys.exit()
		


	def follower(self):
	
		#rospy.Timer(rospy.Duration(self.dt), pose_callback) # regulate the rate of controller
		#rospy.Timer(rospy.Duration(self.dt), command_callback)
		
		rospy.spin()
		
			

if __name__ == '__main__':
	try:
		rospy.init_node('follower_drone')
		
		q = clover(dx_des = 0.3, dy_des = 0.3, REF_FRAME = 'map')
		q.follower()
		
		plt.figure(1)
		plt.subplot(311)
		plt.plot(xf,'r',label='x-fol')
		plt.plot(xL,'b--',label='x-lead')
		plt.legend()
		plt.grid(True)
		plt.subplot(312)
		plt.plot(yf,'r',label='y-fol')
		plt.plot(yL,'b--',label='y-lead')
		plt.legend()
		plt.grid(True)
		plt.ylabel('Position [m]')
		plt.subplot(313)
		plt.plot(YawL,'r')
		plt.grid(True)
		plt.ylabel('yaw body_L [deg]')
		plt.xlabel('Time [s]')
		
		plt.figure(2)
		plt.subplot(311)
		plt.plot(xdispf,'r',label='xdisp')
		plt.plot(ydispf,'b',label='ydisp')
		plt.ylabel('disp[m]')
		plt.xlabel('Time [s]')
		plt.legend()
		plt.grid(True)
		plt.subplot(312)
		plt.plot(yf,'r',label='y-fol')
		plt.plot(yL,'b--',label='y-lead')
		plt.legend()
		plt.grid(True)
		plt.ylabel('Position [m]')
		plt.subplot(313)
		plt.plot(ex,'r',label='ex')
		plt.plot(ey,'b',label='ey')
		plt.plot(eyaw,'g',label='eyaw')
		plt.ylabel('Error[m]')
		plt.xlabel('Time [s]')
		plt.legend()
		plt.grid(True)
		
		plt.figure(3)
		plt.plot(velfx,'r',label='velfx')
		plt.plot(velfy,'b',label='velfx')
		plt.ylabel('Inertial Vel commands[m/s]')
		plt.xlabel('Time [s]')
		plt.legend()
		plt.grid(True)
		plt.show()
		
		
	except rospy.ROSInterruptException:
		pass



	
