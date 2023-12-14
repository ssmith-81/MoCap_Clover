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

from tf.transformations import euler_from_quaternion

import numpy as np

# Could plot the stored data in SITL (not hardware) if desired:
import matplotlib.pyplot as plt


from time import sleep
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

rospy.init_node('Leader') # Figure8 is the name of the ROS node

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
velLx = []
velLy=[]
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
		
		# Define the number of points for circle plotting purposes
		n = 50
		# Generate setpoints for circle plot
		theta = np.linspace(0,-2*math.pi, n+1)
		
		self.xa = self.b_x + self.a*np.cos(theta)
		self.ya = self.b_y + self.a*np.sin(theta)
		
		for i in range(n):
			xa.append(self.xa[i])
			ya.append(self.ya[i])
		
		# Define the sink location and strength
		self.C_sink = 2.6
		self.x_si = 10
		self.y_si = 10
		
		# Define the source strenght and location
		self.C_source = 0.55
		self.x_s = 0
		self.y_s = 0
		
		# Define the vortex strength
		self.C_vortex = -2.0
 		
 		# Free flow constant
		self.U_inf = 1
		self.alpha = 70*(math.pi/180)
		
		# iNITIALIZE INPUT VELOCITY VALUES
		self.u = 0
		self.v = 0
 		
 		# Publisher which will publish to the topic '/mavros/setpoint_velocity/cmd_vel'.
		self.velocity_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',TwistStamped, queue_size=10)

		# global static variables
		self.FLIGHT_ALTITUDE = FLIGHT_ALTITUDE          # fgdf
		self.RATE            = RATE                     # loop rate hz
		self.FRAME           = REF_FRAME                # Reference frame for complex trajectory tracking

		
		# Publisher which will publish to the topic '/mavros/setpoint_raw/local'. This has a PositionTarget message type: link
		self.publisher = rospy.Publisher('/clover0/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
		
		# Publish setpoints on a topic that will be read by the follower drone
		self.follower = rospy.Publisher('follower', PositionTarget, queue_size=10)
		# Publish mode commands on a topic that will be read by the follower drone
		self.follower_mode = rospy.Publisher('leader_commands', String, queue_size=10)
		
		
		# The get_telemety does not retreive body frame attitude or position :( therefore we need to get it from another topic
		#self.body = rospy.Subscriber('/clover/mavros/imu/data',Imu, self.updateIMU)
		
		# The get_telemety does not retreive body frame velocity :( therefore we need to get it from another topic
		#self.vel_body = rospy.Subscriber('/clover/mavros/local_position/velocity_body',TwistStamped, self.update_velbody)
		
		# The get_telemety does not retreive body frame velocity (wasnt working for some reason) :( therefore we need to get it from another topic:
		self.pose_call = rospy.Subscriber('/clover0/mavros/local_position/pose',PoseStamped, self.controller)
		
		#self.current_IMU = Imu()
		# Define object to publish for the follower (if not using mocap or central system)
		self.follow = PositionTarget() # this will be mixed with inertial frame data and body frame (for what is needed for the formation algorithm)
		
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
	
		x = data.pose.position.x
		y = data.pose.position.y
		
		# Sink components
		u_sink = -self.C_sink * ((x - self.x_si) / ((x - self.x_si)**2 + (y - self.y_si)**2))
		v_sink = -self.C_sink * ((y - self.y_si) / ((x - self.x_si)**2 + (y - self.y_si)**2))

		# Source components
		u_source = self.C_source * ((x - self.x_s) / ((x - self.x_s)**2 + (y - self.y_s)**2))
		v_source = self.C_source * ((y - self.y_s) / ((x - self.x_s)**2 + (y - self.y_s)**2))

		# Vortex circle components
		r_o = np.sqrt((x - self.b_x)**2 + (y -self.b_y)**2)
		top = self.b_x * (x - self.b_x)**2 + self.a**2 * (x - self.b_x) + (y - self.b_y) * (2 * self.b_y * x - self.b_x * (y + self.b_y))
		bottom = self.a**4 + 2 * self.a**2 * (self.b_x * (x - self.b_x) + self.b_y * (y - self.b_y)) + (self.b_x**2 + self.b_y**2) * r_o**2
		u_vortex = self.C_vortex * (self.a**2 / r_o**2) * (top / bottom)

		top1 = self.b_y * (y - self.b_y)**2 + self.a**2 * (y - self.b_y) + (x - self.b_x) * (2 * self.b_x * y - self.b_y * (x + self.b_x))
		bottom1 = self.a**4 + 2 * self.a**2 * (self.b_x * (x - self.b_x) + self.b_y * (y - self.b_y)) + (self.b_x**2 + self.b_y**2) * r_o**2
		v_vortex = self.C_vortex * (self.a**2 / r_o**2) * (top1 / bottom1)

		# Uniform flow components
		#u_inf = self.U_inf * np.cos(self.alpha)
		#v_inf = self.U_inf * np.sin(self.alpha)
		
		# Get current state of this follower 
		telem = get_telemetry(frame_id='map')
		
		# uniform flow control method
		self.e_Vx = self.u-telem.vx
		self.e_Vy = self.v-telem.vy
		
		u_inf =  -2*np.tanh(self.e_Vx)  # Define the error dynamic SMC
		v_inf =  -2*np.tanh(self.e_Vy)  # Define the error dynamic SMC

		# Complete contributions
		self.u = u_sink + u_vortex #+ u_inf #+ u_source #+ u_inf
		self.v = v_sink + v_vortex #+v_inf #+ v_source #+ v_inf
		
		# normalize velocities
		vec = np.array([[self.u],[self.v]],dtype=float) 
		magnitude = math.sqrt(self.u**2 + self.v**2)
		
		if magnitude > 0:
			norm_vel = (vec/magnitude)*0.5
		else:
			norm_vel = np.zeros_like(vec)
			
		self.u = norm_vel[0,0]
		self.v = norm_vel[1,0]
		#print(self.u)
		# determine the yaw
		self.omega = math.atan2(self.v,self.u)


	def main(self):
	
		# Wait for 5 seconds
		rospy.sleep(3)
		# Takeoff to a desired altitude
		navigate(x=0,y=0,z = 1, frame_id='map',auto_arm=True)
		
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
			
			target.type_mask = 1+2+2048  # Use everything!
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
			
			if math.sqrt((telem.x-self.x_si) ** 2 + (telem.y-self.y_si) ** 2) < 0.4:
				navigate(x=0,y=0,z=self.FLIGHT_ALTITUDE, yaw=float('nan'), speed=0.5, frame_id = self.FRAME)
				break
			rr.sleep()
			
			
			
			
	

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
		plt.plot(velfx,'r',label='x-vel')
		plt.plot(velcx,'b',label='x-com')
		plt.ylabel('vel[m/s]')
		plt.xlabel('Time [s]')
		plt.legend()
		plt.grid(True)
		plt.subplot(312)
		plt.plot(velfy,'r',label='y-vel')
		plt.plot(velcy,'b--',label='y-com')
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
		
		#plt.figure(3)
		#plt.plot(velfx,'r',label='velfx')
		#plt.plot(velfy,'b',label='velfy')
		#plt.plot(velcx,'g',label='velcx')
		#plt.plot(velcy,'m',label='velcy')
		#plt.plot(velLx,'k',label='velLx')
		#plt.plot(velLy,'y',label='velLy')
		#plt.ylabel('Inertial Vel commands[m/s]')
		#plt.xlabel('Time [s]')
		#plt.legend()
		#plt.grid(True)
		
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
