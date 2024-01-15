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
from std_msgs.msg import String
from sensor_msgs.msg import Imu, LaserScan

from panel_functions import CLOVER_COMPONENTS, CLOVER_STREAM_GEOMETRIC_INTEGRAL, CLOVER_KUTTA, CLOVER_STREAMLINE, CLOVER_noOBSTACLE

from scipy.interpolate import griddata

from tf.transformations import euler_from_quaternion

import numpy as np

# Could plot the stored data in SITL (not hardware) if desired:
import matplotlib.pyplot as plt
from matplotlib import path

from time import sleep
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

rospy.init_node('clover_panel') # Figure8 is the name of the ROS node

# Define the Clover service functions, the only ones used in this application are navigate and land.

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

# Release service is used to allow for complex trajectory publishing i.e it stops the navigate service from publishing setpoints because you dont want two sources of publishing at the same time.
release = rospy.ServiceProxy('simple_offboard/release', Trigger)

# Deine math parameter
PI_2 = math.pi/2

# Debugging and logging
xf = []  # This gathers the 
yf = []
xdispf=[]
ydispf=[]
xa = []
ya = []
YawL = []
YawF = []
YawC = []

# Circle obstacle plot
xa = []
ya = []

# Analyze control input (see if error is being minimized )
velfx=[]
velfy=[]
velcx=[]
velcy=[]
U_infx = []
V_infy=[]
evx=[]
evy=[]
eyaw=[]


			
# This class categorizes all of the functions used for complex rajectory tracking
class clover:

	def __init__(self, FLIGHT_ALTITUDE, RATE, RADIUS, CYCLE_S, REF_FRAME): 
	
		
		# Define the sink location and strength
		self.g_sink = 2.5
		self.xsi = 10
		self.ysi = 10
		
		# Define the source strength and location
		self.g_source = 0.5
		self.xs = 0
		self.ys = 0
		
 		
 		# Free flow constant
		self.U_inf = 0
		self.V_inf = 0
		
		self.alpha = 70*(math.pi/180)
		
		
		
		# Kutta condition flag (decide which one to use)
		self.flagKutta = np.array([0, 0, 1])
		# position 0 is for smooth flow off edge for closed object
		# position 1 is for smooth flow off ege for closed object using point extended into space a small amount
		# position 2 is for smooth flow off the edge of a non closed i.e. sail detection with lidar
		


#---------------------------------------------------------------------------------------------------
		# iNITIALIZE INPUT VELOCITY VALUES
		self.u = 0
		self.v = 0

		# #-------------------- Offline Panel Calculations---------------------------------------------------
			
		# An object was not detected yet, so use the source and sink for velocity based navigation

				## Compute Streamlines with stream function velocity equation
		# Too many gridpoints is not good, it will cause the control loop to run too slow
		# Grid parameters
		self.nGridX = 15;                                                           # X-grid for streamlines and contours
		self.nGridY = 15;                                                           # Y-grid for streamlines and contours
		xVals  = [-1, 11];  # ensured it is extended past the domain incase the clover leaves domain             # X-grid extents [min, max]
		yVals  = [-1, 11];  #-0.3;0.3                                                 # Y-grid extents [min, max]
			
		# Streamline parameters
		stepsize = 0.1;   #0.01                                                     # Step size for streamline propagation
		maxVert  = self.nGridX*self.nGridY*100;                                           # Maximum vertices
		slPct    = 25;                                                          # Percentage of streamlines of the grid


		# Create an array of starting points for the streamlines
		x_range = np.linspace(0, 10, int((10-0)/0.5) + 1)
		y_range = np.linspace(0, 10, int((10-0)/0.5) + 1)

		x_1 = np.zeros(len(y_range))
		y_1 = np.zeros(len(x_range))
		Xsl = np.concatenate((x_1, x_range))
		Ysl = np.concatenate((np.flip(y_range), y_1))
		XYsl = np.vstack((Xsl,Ysl)).T

		# Generate the grid points
		Xgrid = np.linspace(xVals[0], xVals[1], self.nGridX)
		Ygrid = np.linspace(yVals[0], yVals[1], self.nGridY)
		self.XX, self.YY = np.meshgrid(Xgrid, Ygrid)

		self.Vxe = np.zeros((self.nGridX, self.nGridY))
		self.Vye = np.zeros((self.nGridX, self.nGridY))


		for m in range(self.nGridX):
			for n in range(self.nGridY):
				XP, YP = self.XX[m, n], self.YY[m, n]
				u, v = CLOVER_noOBSTACLE(XP, YP, self.U_inf, self.V_inf, self.xs, self.ys, self.xsi, self.ysi, self.g_source, self.g_sink)
				
				self.Vxe[m, n] = u
				self.Vye[m, n] = v

		# Flatten the grid point matices and velocity matrices into vector arrays for the griddata function
		self.XX_f = self.XX.flatten()
		self.YY_f = self.YY.flatten()
		self.Vxe_f = self.Vxe.flatten()
		self.Vye_f = self.Vye.flatten()

		# -------------------------------------------------------------------------------------------------------------------------------

		# global static variables
		self.FLIGHT_ALTITUDE = FLIGHT_ALTITUDE          # fgdf
		self.RATE            = RATE                     # loop rate hz
		self.FRAME           = REF_FRAME                # Reference frame for complex trajectory tracking
		
		self.last_timestamp = None # make sure there is a timestamp

		
		# Publisher which will publish to the topic '/mavros/setpoint_raw/local'. This has a PositionTarget message type: link
		self.publisher = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)


		
		# The get_telemety does not retreive body frame velocity (wasnt working for some reason) :( therefore we need to get it from another topic:
		self.pose_call = rospy.Subscriber('/mavros/local_position/pose',PoseStamped, self.controller)
		
		# Generate the array of lidar angles
		self.lidar_angles = np.linspace(-45*(math.pi/180), 45*(math.pi/180), 32) 

		# Initialize the ranges array for the controller as no object detected (i.e. an array of inf)
		self.obs_detect = np.full_like(self.lidar_angles, np.inf)

		# Subscribe to the Lidar readings
		self.lidar = rospy.Subscriber('/ray_scan',LaserScan,self.lidar_read)

		# Set a timer for the velocity field update function (runs periodically)
		# updates every 8 seconds (0.125 Hz)

		rospy.Timer(rospy.Duration(5), self.velocity_field_update)

		# Set a flag, that will track if a change in envirnment occurred i.e. an object was detected
		# therefore if an object was not detected previously then there is no reason in re calculating
		# the velocity field based on source/sink strengths
		self.flag = False

		
		
		# Define object to publish for the follower (if not using mocap or central system)
		self.follow = PositionTarget()
		
		self.follow.header.frame_id = 'map'  # Define the frame that will be used
			
		self.follow.coordinate_frame = 1 #MAV_FRAME_BODY_NED  # =8
			
		self.follow.type_mask = 0  # use everything!
						# PositionTarget::IGNORE_VX +
						# PositionTarget::IGNORE_VY +
						# PositionTarget::IGNORE_VZ +
						# PositionTarget::IGNORE_AFX +
						# PositionTarget::IGNORE_AFY +
						# PositionTarget::IGNORE_AFZ +
						# PositionTarget::IGNORE_YAW;

		self.current_state = State()
		self.rate = rospy.Rate(20)
	
	def lidar_read(self,data):

		# Update the obstacle detect array
		self.obs_detect = data.ranges

		# Ensure there are actually lidar readings, no point in doing calculations if
		# nothing is detected:
		if any(not np.isinf(range_val) for range_val in self.obs_detect):
	
			# The angles and ranges start at -45 degrees i.e. at the right side, then go counter clockwise up to the top i.e. 45 degrees
			ranges = data.ranges
		
			angles = self.lidar_angles
		
			telem = get_telemetry(frame_id='map')
			
			x_clover = telem.x
			y_clover = telem.y
			yaw = telem.yaw
			
			
			# Polar to Cartesion transformation for all readings (assuming angles are in standard polar coordinates).
			x_local = ranges*np.cos(angles)
			y_local = ranges*np.sin(angles)
			
			# Homogenous transformation matrix for 2D
			R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]) # rotation matrix
			T = np.vstack([np.hstack([R, np.array([[x_clover], [y_clover]])]),[0,0,1]]) # Homogenous transformation matrix
			
			# Lidar readings in homogenous coordinates
			readings_local = np.vstack([x_local, y_local, np.ones_like(x_local)])
			
			# Transform all lidar readings to global coordinates
			self.readings_global = np.dot(T, readings_local)
			
			# Extract the tranformed positions
			self.readings_global = self.readings_global[:2,:].T

			# Dont want to update these here!! we are using in controller and if the values are updated mid calculation for velocity itll cause many issues
			# This is updating faster then the velocity controller calculations
			# self.xa = readings_global[:,0].T
			# self.ya = readings_global[:,1].T

	def velocity_field_update(self, event=None):

		# this needs to run periodically to update the velocity field. But not too much as it is very compuationally expensive, maybe run as a thread or something

		# Get current state of this follower 
		telem = get_telemetry(frame_id='map')

		if any(not np.isinf(range_val) for range_val in self.obs_detect) and telem.z >=0.8: # Dont want to use this during takeoff as the lidar will pick up the ground when the Clover slants

			# Update the obstacle detection flag
			self.flag = True

			# Update the lidar detection readings
			self.xa = self.readings_global[:,0].T
			self.ya = self.readings_global[:,1].T
			# Filter out the inf values in the data point arrays
			self.xa = self.xa[np.isfinite(self.xa)]
			self.ya = self.ya[np.isfinite(self.ya)]

				# Upate the number of panels
			self.n = len(self.xa)-1
				
				# in this case an obstacle was detected so apply panel method navigation
		
				# Get current state of this follower 
			telem = get_telemetry(frame_id='map')

			# #-------------------- Offline Panel Calculations---------------------------------------------------
				
				#This function calculates the location of the control points as well as the
						#right hand side of the stream function equation:

			[xmid, ymid, dx, dy, Sj, phiD, rhs] = CLOVER_COMPONENTS(self.xa, self.ya, self.U_inf, self.V_inf, self.g_source, self.g_sink, self.xs, self.ys, self.xsi, self.ysi, self.n)



				# Convert angles from [deg] to [rad]
			phi = np.deg2rad(phiD)  # Convert from [deg] to [rad]

		

					# Evaluate gemoetric integral matrix without the kutta condition equation
			I = CLOVER_STREAM_GEOMETRIC_INTEGRAL(xmid, ymid, self.xa, self.ya, phi, Sj, self.n)

			# Form the last line of the system of equations with the kutta condition 
			[I, rhs] = CLOVER_KUTTA(I, self.n, self.flagKutta, rhs)

			# calculating the vortex density (and stream function from kutta condition)
			# by solving linear equations given by
			g = np.linalg.solve(I, rhs.T)  # = gamma = Vxy/V_infinity for when we are using U_inf as the free flow magnitude
			# broken down into components (using cos(alpha), and sin(alpha)), and free flow is the only other RHS
			# contribution (no source or sink). It is equal to Vxy 
			# when we define the x and y (U_inf and V_inf) components seperately.

					## Compute Streamlines with stream function velocity equation
			# Too many gridpoints is not good, it will cause the control loop to run too slow
			# Grid parameters
			
			for m in range(self.nGridX):
				for n in range(self.nGridY):
					XP, YP = self.XX[m, n], self.YY[m, n]
					u, v = CLOVER_STREAMLINE(XP, YP, xa, ya, phi, g, Sj, self.U_inf, self.V_inf, self.xs, self.ys, self.xsi, self.ysi, self.g_source, self.g_sink)

					self.Vxe[m, n] = u
					self.Vye[m, n] = v

		else:
			# only run if the flag is true, i.e. an object was previously detected
			if self.flag:
				# An object was not detected yet, so use the source and sink for velocity based navigation
				for m in range(self.nGridX):
					for n in range(self.nGridY):
						XP, YP = self.XX[m, n], self.YY[m, n]
						u, v = CLOVER_noOBSTACLE(XP, YP, self.U_inf, self.V_inf, self.xs, self.ys, self.xsi, self.ysi, self.g_source, self.g_sink)
						
						self.Vxe[m, n] = u
						self.Vye[m, n] = v

				self.flag = False # update the flag variable



		print('here')
		# Flatten the grid point matices and velocity matrices into vectory arrays for the griddata function
		# Update the velocity field:
		self.XX_f = self.XX.flatten()
		self.YY_f = self.YY.flatten()
		self.Vxe_f = self.Vxe.flatten()
		self.Vye_f = self.Vye.flatten()
		
	def controller(self,data):
		
		current_timestamp = data.header.stamp.to_sec()
		
		if self.last_timestamp is not None:

			# Get current state of this follower 
			telem = get_telemetry(frame_id='map')

				
			
				# #-------------------- Offline Panel Calculations---------------------------------------------------
			
				

			# Complete contributions from pre-computed grid distribution
			u = griddata((self.XX_f, self.YY_f),self.Vxe_f,(telem.x,telem.y),method='cubic') #+ self.u_inf #+ u_source #+ u_inf
			v = griddata((self.XX_f, self.YY_f),self.Vye_f,(telem.x,telem.y),method='cubic') #+self.v_inf#+ v_source #+self.v_inf #+ v_source #+ v_inf

				

			# Complete contributions from pre-computed grid distribution
			self.u = u
			self.v = v
					
			# normalize velocities
			vec = np.array([[self.u],[self.v]],dtype=float) 
			magnitude = math.sqrt(self.u**2 + self.v**2)
				
			if magnitude > 0:
				norm_vel = (vec/magnitude)*0.25
			else:
				norm_vel = np.zeros_like(vec)
					
			self.u = norm_vel[0,0]
			self.v = norm_vel[1,0]
					#print(self.u)
					# determine the yaw
			self.omega = math.atan2(self.v,self.u)
		
		# Update last_timestamp for the next callback
		self.last_timestamp = current_timestamp


	def main(self):
	
		# Wait for 5 seconds
		rospy.sleep(3)
		# Takeoff to a desired altitude # x=0.2, y=2
		navigate(x=1,y=1,z = 1, frame_id='map',auto_arm=True) # drone seems to be unstable when I set frame to map... I think there is something wrong with lidar physical values
		
		# Give the Clover time to reach the takeoff altitude
		rospy.sleep(15)
		
		# Define object that will be published
		target = PositionTarget()
		rr = rospy.Rate(self.RATE)
		
		release() # stop navigate service from publishing before beginning the figure-8 publishing
		
		while not rospy.is_shutdown():
			# Trajectory publishing-----------------------------------------------
			target.header.frame_id = self.FRAME  # Define the frame that will be used
			
			target.coordinate_frame = 1 #MAV_FRAME_LOCAL_NED  # =1
			
			target.type_mask = 1+2+64+128+256+2048  # Use everything!
						# PositionTarget::IGNORE_VX +
						# PositionTarget::IGNORE_VY +
						# PositionTarget::IGNORE_VZ +
						# PositionTarget::IGNORE_AFX +
						# PositionTarget::IGNORE_AFY +
						# PositionTarget::IGNORE_AFZ +
						# PositionTarget::IGNORE_YAW;

			# Gather position for publishing
			#target.position.x = posx[k]
			#target.position.y = posy[k]
			target.position.z = self.FLIGHT_ALTITUDE
			
			# Gather velocity for publishing
			target.velocity.x = self.u
			target.velocity.y = self.v
			target.velocity.z = 0
			
			# Gather acceleration for publishing
			target.acceleration_or_force.x = 0
			target.acceleration_or_force.y = 0
			target.acceleration_or_force.z = 0
			
			# Gather yaw for publishing
			target.yaw = self.omega
			
			# Gather yaw rate for publishing
			#target.yaw_rate = yaw_ratec[k]
			
			# Publish to the setpoint topic
			
			
			self.publisher.publish(target)
			
			
			
			# Get current state of this follower 
			telem = get_telemetry(frame_id='map')
			
			# logging/debugging
			xf.append(telem.x)
			yf.append(telem.y)
			evx.append(self.u-telem.vx)
			evy.append(self.v-telem.vy)
			eyaw.append(self.omega-telem.yaw)
			YawC.append(self.omega*(180/math.pi))
			YawF.append(telem.yaw*(180/math.pi))
			#xdispf.append(self.d_xi)
			#ydispf.append(self.d_yi)
			velfx.append(telem.vx)
			velfy.append(telem.vy)
			velcx.append(self.u)
			velcy.append(self.v)
			# U_infx.append(self.U_inf)
			# V_infy.append(self.V_inf)
			
			if math.sqrt((telem.x-self.xsi) ** 2 + (telem.y-self.ysi) ** 2) < 0.6: # 0.4
				navigate(x=0,y=0,z=self.FLIGHT_ALTITUDE, yaw=float('nan'), speed=0.5, frame_id = self.FRAME)
				break
			rr.sleep()
			#rospy.spin()
			
			
			
			
	

		# Wait for 3 seconds
		rospy.sleep(3)
		# Perform landing
		
		land()

if __name__ == '__main__':
	try:
		# Define the performance parameters here which starts the script
		q=clover(FLIGHT_ALTITUDE = 1.0, RATE = 50, RADIUS = 2.0, CYCLE_S = 13, REF_FRAME = 'map')
		
		q.main()
		#print(xa)
		#print(xf)
		
		# Plot logged data for analyses and debugging
		plt.figure(1)
		plt.subplot(211)
		plt.plot(xf,yf,'r',label='x-fol')
		#plt.plot(xa,'b--',label='x-obs')
		plt.fill(xa,ya,'k')
		plt.legend()
		plt.grid(True)
		#plt.subplot(312)
		#plt.plot(yf,'r',label='y-fol')
		#plt.plot(ya,'b--',label='y-obs')
		#plt.legend()
		#plt.grid(True)
		#plt.ylabel('Position [m]')
		plt.subplot(212)
		plt.plot(YawF,'b',label='yaw-F')
		plt.plot(YawC,'g',label='yaw-C')
		plt.legend()
		plt.grid(True)
		plt.ylabel('yaw [deg]')
		plt.xlabel('Time [s]')
		
		# Velocity plot
		plt.figure(2)
		plt.subplot(311)
		plt.plot(velfx,'r',label='vx-vel')
		plt.plot(velcx,'b',label='vx-com')
		plt.ylabel('vel[m/s]')
		plt.xlabel('Time [s]')
		plt.legend()
		plt.grid(True)
		plt.subplot(312)
		plt.plot(velfy,'r',label='vy-vel')
		plt.plot(velcy,'b--',label='vy-com')
		plt.legend()
		plt.grid(True)
		plt.ylabel('Position [m]')
		plt.subplot(313)
		plt.plot(evx,'r',label='evx')
		plt.plot(evy,'b',label='evy')
		plt.plot(eyaw,'g',label='eyaw')
		plt.ylabel('Error[m]')
		plt.xlabel('Time [s]')
		plt.legend()
		plt.grid(True)
		
		# plt.figure(3)
		# plt.plot(U_infx,'r',label='x_inf')
		# plt.plot(V_infy,'b',label='y_inf')
		# #plt.plot(velcx,'g',label='velcx')
		# #plt.plot(velcy,'m',label='velcy')
		# #plt.plot(velLx,'k',label='velLx')
		# #plt.plot(velLy,'y',label='velLy')
		# plt.ylabel('SMC Commands [m/s]')
		# plt.xlabel('Time [s]')
		# plt.legend()
		# plt.grid(True)
		
		#plt.figure(4)
		#plt.subplot(211)
		#plt.plot(ex,'r',label='ex-fol')
		#plt.plot(ey,'b--',label='ey-lead')
		#plt.plot(
		
		#plt.subplot(212)
		#plt.plot(YawL,'b--',label='yaw')
		
		plt.show()
		
	except rospy.ROSInterruptException:
		pass
