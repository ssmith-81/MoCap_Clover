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


class clover:

	def __init__(self, dx_des, dy_des, REF_FRAME):
	
		self.mode = "idle" # Initialize state of follower
		self.is_tracking_activated = False
		self.pose_data = None
		self.released_flag = False # Flag to track if the release() function has been called
		self.track = PositionTarget()  # For publishing complex trajectory
		
		self.subscriber = rospy.Subscriber("leader_commands",String, self.command_callback)
		
		self.leader = rospy.Subscriber("follower", PositionTarget, self.pose_callback) # Subscribe to leader publishing
		
		# Publisher which will publish to the topic '/clover1/mavros/setpoint_raw/local'. This has a PositionTarget message type: link
		self.publisher = rospy.Publisher('/clover1/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
		
		# Initialize control parameters here
		self.d_xid = dx_des
		self.d_yid = dy_des
		self.FRAME = REF_FRAME  # Reference frame for complex trajectory tracking
		
		self.dt = 1/50 # 50 hz controller
		self.SX_int = np.array([[0],[0],[0]]) # Initialize integrated experssion to zero
		
		self.sign_sx_int = np.array([[0],[0],[0]]) # Initialize integrated experssion to zero
		

	def state(self,msg):
		self.mode = msg
		
	def controller(self,data):

		# Position commands
		self.X_L = data.position.x
		self.Vx_L = data.velocity.x
		self.Y_L = data.position.y
		self.Vy_L = data.velocity.y
		self.Psi_L = data.yaw
		self.Psi_dot_L = data.yaw_rate
		
		# Get current state of the follower
		telem = get_telemetry(frame_id='map')
		
		
		# Calculate current distance from leader
		self.d_xi = -(self.X_L - telem.x)*math.cos(self.Psi_L) - (self.Y_L - telem.y)*math.sin(self.Psi_L)
		self.d_yi = (self.X_L - telem.x)*math.sin(self.Psi_L) + (self.Y_L - telem.y)*math.cos(self.Psi_L)
		
		# desired position commands
		self.X_i = self.X_L - self.d_xi*math.cos(self.Psi_L) - self.d_yi*math.sin(self.Psi_L)
		self.Y_i = self.Y_L + self.d_xi*math.sin(self.Psi_L) - self.d_yi*math.cos(self.Psi_L)	
		
		# calculate error
		self.e_psi = telem.yaw - self.Psi_L
		
		self.e_Xi = self.d_xid - self.d_xi
		self.e_Yi = self.d_yid - self.d_yi
		
		# translational dynamic matrix formulation
		G_X = np.array([[-math.cos(self.e_psi), math.sin(self.e_psi), 0],[-math.sin(self.e_psi),-math.cos(self.e_psi), 0],[0, 0, 1]])
		
		F_X = np.array([[self.e_Yi*self.Psi_dot_L + self.Vx_L - self.Psi_dot_L*self.d_yid],[-self.e_Xi*self.Psi_dot_L + self.Vy_L - self.Psi_dot_L*self.d_xid],[self.e_psi]])
		
		SX = np.array([[self.e_Xi],[self.e_Yi],[self.e_psi]])
		
		# Integral portion
		
		self.SX_int = self.SX_int + SX*self.dt
		 
		
		# Derivative
		mu = np.array([[telem.vx],[telem.vy],[telem.yaw_rate]]) # state of the follower from EKF
		SX_dot = F_x + G_X*mu
		
		Tau_1 = np.array([[3],[3],[3]]) # gain matrix 1
		Tau_2 = np.array([[3],[3],[3]]) # gain matrix 2
		
		# Sliding surface
		s_i = SX_i + Tau_1*SX_int + Tau_2*SX_i**(self.p/self.q) # need to define p and  still
		
		self.sign_sx_int = self.sign_sx_int + np.tanh(s_i)
		
		# equivalent control
		
		#self.Vx_i = np.linalg.inv(G_x)
		V_eq = -F_X - 
		self.track.header.frame_id = self.FRAME  # Define the frame that will be used
			
		self.track.coordinate_frame = 1 #MAV_FRAME_LOCAL_NED  # =1
			
		self.track.type_mask = 0  # Use everything!
						# PositionTarget::IGNORE_VX +
						# PositionTarget::IGNORE_VY +
						# PositionTarget::IGNORE_VZ +
						# PositionTarget::IGNORE_AFX +
						# PositionTarget::IGNORE_AFY +
						# PositionTarget::IGNORE_AFZ +
						# PositionTarget::IGNORE_YAW;

			# Gather position for publishing
		self.track.position.x = self.X_i
		self.track.position.y = self.Y_i
		self.track.position.z = 1
			
			# Gather velocity for publishing
		self.track.velocity.x = data.velocity.x
		self.track.velocity.y = data.velocity.y
		self.track.velocity.z = data.velocity.z
			
			# Gather acceleration for publishing
		self.track.acceleration_or_force.x = data.acceleration_or_force.x
		self.track.acceleration_or_force.y = data.acceleration_or_force.y
		self.track.acceleration_or_force.z = data.acceleration_or_force.z
			
			# Gather yaw for publishing
		self.track.yaw = data.yaw
			
			# Gather yaw rate for publishing
		self.track.yaw_rate = data.yaw_rate
			
			# Publish to the setpoint topic
		self.publisher.publish(self.track)
		


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
	
		rospy.Timer(rospy.Duration(self.dt), pose_callback) # regulate the rate of controller
		rospy.Timer(rospy.Duration(self.dt), command_callback)
		
		rospy.spin()

if __name__ == '__main__':
	try:
		rospy.init_node('follower_drone')
		
		q = clover(dx_des = 0.5, dy_des = 0.5, REF_FRAME = 'map')
		q.follower()
		
	except rospy.ROSInterruptException:
		pass



	
