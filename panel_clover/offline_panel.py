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
from sensor_msgs.msg import Imu

from panel_functions import CLOVER_COMPONENTS, CLOVER_STREAM_GEOMETRIC_INTEGRAL, CLOVER_KUTTA, CLOVER_STREAMLINE

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

get_telemetry = rospy.ServiceProxy('/clover0/get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('/clover0/navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('/clover0/navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('/clover0/set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('/clover0/set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('/clover0/set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('/clover0/set_rates', srv.SetRates)
land = rospy.ServiceProxy('/clover0/land', Trigger)

# Release service is used to allow for complex trajectory publishing i.e it stops the navigate service from publishing setpoints because you dont want two sources of publishing at the same time.
release = rospy.ServiceProxy('/clover0/simple_offboard/release', Trigger)

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
	
		# Create the circle obstacle geometry
		self.a          = RADIUS                   # radius of circle obstacle in meters
		self.b_x        = 5                      # x-location of the circle obstacle location
		self.b_y        = 5                        # y-location of the circle obstacle location
		
		# Define the number of points for circle plotting purposes and panels
		n = 50
		# Generate setpoints for circle plot
		theta = np.linspace(0,-2*math.pi, n+1)
		
		self.xa = self.b_x + self.a*np.cos(theta)
		self.ya = self.b_y + self.a*np.sin(theta)
		
		for i in range(n):
			xa.append(self.xa[i])
			ya.append(self.ya[i])
		
		# Define the sink location and strength
		self.g_sink = 2.5
		self.xsi = 10
		self.ysi = 10
		
		# Define the source strenght and location
		self.g_source = 0.5
		self.xs = 0
		self.ys = 0
		
 		
 		# Free flow constant
		self.U_inf = 0
		self.V_inf = 0
		
		self.alpha = 70*(math.pi/180)
		
		
		
# 		# Kutta condition flag (decide which one to use)
# 		self.flagKutta = np.array([0, 1])
		
# #-------------------- Offline Panel Calculations---------------------------------------------------
		
# 		#This function calculates the location of the control points as well as the
#                 #right hand side of the stream function equation:

# 		[xmid, ymid, dx, dy, Sj, phiD, rhs] = CLOVER_COMPONENTS(self.xa, self.ya, self.U_inf, self.V_inf, self.g_source, self.g_sink, self.xs, self.ys, self.xsi, self.ysi, n)



# 		# Convert angles from [deg] to [rad]
# 		phi = np.deg2rad(phiD)  # Convert from [deg] to [rad]

# 		# Circle trail point (this is intuitively set from matlab plots, will need to change this later)
# 		xtrail = (self.a + 0.001) * np.cos(45 * np.pi / 180)
# 		ytrail = (self.a + 0.001) * np.sin(45 * np.pi / 180)
# 		trail_point = np.array([self.b_x, self.b_y]) + np.array([xtrail, ytrail])

# 		# Evaluate gemoetric integral matrix without the kutta condition equation
# 		I = CLOVER_STREAM_GEOMETRIC_INTEGRAL(xmid, ymid, self.xa, self.ya, phi, Sj, n)

# # Form the last line of the system of equations with the kutta condition
# 		[I, rhs] = CLOVER_KUTTA(I, trail_point, xa, ya, phi, Sj, n, self.flagKutta, rhs, self.U_inf, self.V_inf, self.xs, self.ys, self.xsi, self.ysi, self.g_source, self.g_sink)

# # calculating the vortex density (and stream function from kutta condition)
# # by solving linear equations given by
# 		g = np.linalg.solve(I, rhs.T)  # = gamma = Vxy/V_infinity for when we are using U_inf as the free flow magnitude
# # broken down into components (using cos(alpha), and sin(alpha)), and free flow is the only other RHS
# # contribution (no source or sink). It is equal to Vxy 
# # when we define the x and y (U_inf and V_inf) components seperately.

# 		## Compute Streamlines with stream function velocity equation

# 		# Grid parameters
# 		nGridX = 100;                                                           # X-grid for streamlines and contours
# 		nGridY = 100;                                                           # Y-grid for streamlines and contours
# 		xVals  = [-1, 11];  # ensured it is extended past the domain incase the clover leaves domain             # X-grid extents [min, max]
# 		yVals  = [-0.5, 11];  #-0.3;0.3                                                 # Y-grid extents [min, max]
			
# 		# Streamline parameters
# 		stepsize = 0.1;   #0.01                                                     # Step size for streamline propagation
# 		maxVert  = nGridX*nGridY*100;                                           # Maximum vertices
# 		slPct    = 25;                                                          # Percentage of streamlines of the grid


# 		# Create an array of starting points for the streamlines
# 		x_range = np.linspace(0, 10, int((10-0)/0.5) + 1)
# 		y_range = np.linspace(0, 10, int((10-0)/0.5) + 1)

# 		x_1 = np.zeros(len(y_range))
# 		y_1 = np.zeros(len(x_range))
# 		Xsl = np.concatenate((x_1, x_range))
# 		Ysl = np.concatenate((np.flip(y_range), y_1))
# 		XYsl = np.vstack((Xsl,Ysl)).T

# 		# Generate the grid points
# 		Xgrid = np.linspace(xVals[0], xVals[1], nGridX)
# 		Ygrid = np.linspace(yVals[0], yVals[1], nGridY)
# 		self.XX, self.YY = np.meshgrid(Xgrid, Ygrid)

# 		self.Vxe = np.zeros((nGridX, nGridY))
# 		self.Vye = np.zeros((nGridX, nGridY))

# 		# Path to figure out if grid point is inside polygon or not
# 		#AF = np.vstack((xa.T,ya.T)).T
# 		AF = np.vstack((xa,ya)).T
# 		#print(AF)
# 		afPath = path.Path(AF)

# 		for m in range(nGridX):
# 			for n in range(nGridY):
# 				XP, YP = self.XX[m, n], self.YY[m, n]
# 				u, v = CLOVER_STREAMLINE(XP, YP, xa, ya, phi, g, Sj, self.U_inf, self.V_inf, self.xs, self.ys, self.xsi, self.ysi, self.g_source, self.g_sink)

# 				if afPath.contains_points([[XP,YP]]):
# 					self.Vxe[m, n] = 0
# 					self.Vye[m, n] = 0
# 				else:
# 					self.Vxe[m, n] = u
# 					self.Vye[m, n] = v
			
		# Load in the gridpoint and velocity grid matrices calculated from the VP program
		self.XX = np.load('XX.npy')
		self.YY = np.load('YY.npy')
		self.Vxe = np.load('Vxe.npy')
		self.Vye = np.load('Vye.npy')

		# Flatten the grid point matices and velocity matrices into vectory arrays for the griddata function
		self.XX = self.XX.flatten()
		self.YY = self.YY.flatten()
		self.Vxe = self.Vxe.flatten()
		self.Vye = self.Vye.flatten()


#---------------------------------------------------------------------------------------------------
		# iNITIALIZE INPUT VELOCITY VALUES
		self.u = 0
		self.v = 0

		# global static variables
		self.FLIGHT_ALTITUDE = FLIGHT_ALTITUDE          # fgdf
		self.RATE            = RATE                     # loop rate hz
		self.FRAME           = REF_FRAME                # Reference frame for complex trajectory tracking
		
		self.last_timestamp = None # make sure there is a timestamp

		
		# Publisher which will publish to the topic '/mavros/setpoint_raw/local'. This has a PositionTarget message type: link
		self.publisher = rospy.Publisher('/clover0/mavros/setpoint_raw/local', PositionTarget, queue_size=10)


		
		# The get_telemety does not retreive body frame velocity (wasnt working for some reason) :( therefore we need to get it from another topic:
		self.pose_call = rospy.Subscriber('/clover0/mavros/local_position/pose',PoseStamped, self.controller)
		
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
		
	def controller(self,data):
		
		current_timestamp = data.header.stamp.to_sec()
		
		if self.last_timestamp is not None:
		
			# Get current state of this follower 
			telem = get_telemetry(frame_id='map')
		
			print(self.Vxe[1])
			# Complete contributions from pre-computed grid distribution
			self.u = griddata((self.XX, self.YY),self.Vxe,(telem.x,telem.y),method='cubic') #+ self.u_inf #+ u_source #+ u_inf
			self.v = griddata((self.XX, self.YY),self.Vye,(telem.x,telem.y),method='cubic') #+self.v_inf#+ v_source #+self.v_inf #+ v_source #+ v_inf
			
			# normalize velocities
			vec = np.array([[self.u],[self.v]],dtype=float) 
			magnitude = math.sqrt(self.u**2 + self.v**2)
		
			if magnitude > 0:
				norm_vel = (vec/magnitude)*0.6
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
		# Takeoff to a desired altitude
		navigate(x=0.2,y=2,z = 1, frame_id='map',auto_arm=True)
		
		# Give the Clover time to reach the takeoff altitude
		rospy.sleep(8)
		
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
			
			if math.sqrt((telem.x-self.xsi) ** 2 + (telem.y-self.ysi) ** 2) < 0.5: # 0.4
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
