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

import numpy as np

# Could plot the stored data in SITL (not hardware) if desired:
import matplotlib.pyplot as plt


from time import sleep
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

rospy.init_node('Figure8') # Figure8 is the name of the ROS node

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

# This class defines a few mavros services that can be used, although the Clover services automatically
# take care of these actions including arming for offboard

# maybe add some more that are usable? confirm they are usable just for more illustration prposes
class fcuModes:

	def __init__(self):
		pass

	def setArm(self):
		rospy.wait_for_service('mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
			armService(True)
		except rospy.ServiceException as e:
			print ("Service arming call failed: %s")

	def setOffboardMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='OFFBOARD')
		except rospy.ServiceException as e:
			print ("service set_mode call failed: %s. Offboard Mode could not be set.")
			
	def setAutoLandMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='AUTO.LAND')
		except rospy.ServiceException as e:
			print ('service set_mode call failed: %s. Autoland Mode could not be set.')
			
# This class categorizes all of the functions used for complex rajectory tracking
class clover:

	def __init__(self, FLIGHT_ALTITUDE, RATE, RADIUS, CYCLE_S, REF_FRAME): 
 		
 		# Publisher which will publish to the topic '/mavros/setpoint_velocity/cmd_vel'.
		self.velocity_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',TwistStamped, queue_size=10)

		# global static variables
		self.FLIGHT_ALTITUDE = FLIGHT_ALTITUDE          # fgdf
		self.RATE            = RATE                     # loop rate hz
		self.RADIUS          = RADIUS                   # radius of figure 8 in meters
		self.CYCLE_S         = CYCLE_S                  # time to complete one figure 8 cycle in seconds
		self.STEPS           = int( self.CYCLE_S * self.RATE )
		self.FRAME           = REF_FRAME                # Reference frame for complex trajectory tracking

		
		# Publisher which will publish to the topic '/mavros/setpoint_raw/local'. This has a PositionTarget message type: link
		self.publisher = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
		
		# Subscribe to drone state. Can use this to subscribe to the drone state with mavros, although the get_telemetry Clover function does this (does it?)
		self.state = rospy.Subscriber('mavros/state', State, self.updateState)

		self.current_state = State()
		self.rate = rospy.Rate(20)

	def updateState(self, msg):
		self.current_state = msg
		
	# generate a path following Bernoulli's lemiscate as a parametric equation
        # note this is in ENU coordinates (x right, y forward, z up) since mavros will convert to NED.
        
	def navigate_wait(self,x=0, y=0, z=0, yaw=float('nan'), yaw_rate=0, speed=0.5, frame_id='body', tolerance=0.2, auto_arm=False):

		res  = navigate(x=x,y=y,z=z, yaw=yaw, yaw_rate=yaw_rate, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
		if not res.success:
			return res

		while not rospy.is_shutdown():
			telem = get_telemetry(frame_id='navigate_target')
			if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
				return res
			rospy.sleep(0.2)

	def main(self):
	
		
		i    = 0                        # Set the counter
		dt   = 1.0/self.RATE		# Set the sample time step
		dadt = math.pi*2 / self.CYCLE_S # first derivative of angle with respect to time
		r    = self.RADIUS		# Set the radius of the figure-8
		path = []
		

		# Wait for 5 seconds
		rospy.sleep(3)
		
		# Takeoff with Clovers navigate function to the desured altitude. This can be in the body frame, or map reference frame for Motion Capture System. It waits for the CLovers arrival before moving on to complex trajectory tracking.
		self.navigate_wait(z=self.FLIGHT_ALTITUDE, frame_id='body',auto_arm=True)
		
		
		rospy.loginfo('start figure8')          # Print a notification to the screen when beginning the figure-8
		PI=math.pi
		start = get_telemetry()                 # Use this Clover function to get the current drone state
		start_stamp = rospy.get_rostime()       # Get the current ROS time
		
		# create random time array with enough elements to complete the entire figure-8 sequence
		t = np.arange(0,self.STEPS,1)
		
		# Create arrays for each variable we want to feed information to:
		posx = [1]*len(t)
		posy = [1]*len(t)
		posz = [1]*len(t)
		velx = [1]*len(t)
		vely = [1]*len(t)
		velz = [1]*len(t)
		afx = [1]*len(t)
		afy = [1]*len(t)
		afz = [1]*len(t)
		yawc = [1]*len(t)
		pitchc = [1]*len(t)
		rollc = [1]*len(t)
		yaw_ratec = [1]*len(t)
		
		for i in range(0, self.STEPS):
		
			# calculate the parameter 'a' which is an angle sweeping from -pi/2 to 3pi/2
			# through the figure-8 curve. 
			a = (-math.pi/2) + i*(math.pi*2/self.STEPS)
			# These are definitions that will make position, velocity, and acceleration calulations easier:
			c = math.cos(a)
			c2a = math.cos(2.0*a)
			c4a = math.cos(4.0*a)
			c2am3 = c2a-3.0
			c2am3_cubed = c2am3*c2am3*c2am3
			s = math.sin(a)
			cc = c*c
			ss = s*s
			sspo = (s*s)+1.0 # sin squared plus one
			ssmo = (s*s)-1.0 # sin squared minus one
			sspos = sspo*sspo
			
			# For more information on these equations, refer to the GitBook Clover documentation:
			
			# Position
			# https:#www.wolframalpha.com/input/?i=%28-r*cos%28a%29*sin%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29
			posx[i] = -(r*c*s) / sspo
			# https:#www.wolframalpha.com/input/?i=%28r*cos%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29
			posy[i] =  (r*c)   / sspo
			posz[i] =  self.FLIGHT_ALTITUDE

			# Velocity
			# https:#www.wolframalpha.com/input/?i=derivative+of+%28-r*cos%28a%29*sin%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
			velx[i] =   dadt*r* ( ss*ss + ss + (ssmo*cc) ) / sspos
			# https:#www.wolframalpha.com/input/?i=derivative+of+%28r*cos%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
			vely[i] =  -dadt*r* s*( ss + 2.0*cc + 1.0 )    / sspos
			velz[i] =  0.0

			# Acceleration
			# https:#www.wolframalpha.com/input/?i=second+derivative+of+%28-r*cos%28a%29*sin%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
			afx[i] =  -dadt*dadt*8.0*r*s*c*((3.0*c2a) + 7.0)/ c2am3_cubed
			# https:#www.wolframalpha.com/input/?i=second+derivative+of+%28r*cos%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
			afy[i] =  dadt*dadt*r*c*((44.0*c2a) + c4a - 21.0) / c2am3_cubed
			afz[i] =  0.0

			# calculate yaw as direction of velocity vector:
			yawc[i] = math.atan2(vely[i], velx[i])
			
			# calculate Pitch and Roll angles, if publishing acceleration isnt possible could send these low level commands
			pitchc[i] = math.asin(afx[i]/9.81)
			rollc[i] = math.atan2(afy[i], afz[i])
		
		# calculate yaw_rate by dirty differentiating yaw
		for i in range(0, self.STEPS):
			next = yawc[(i+1)%self.STEPS] # 401%400 = 1 --> used for this reason (when it goes over place 400 or whatever STEPS is)
			curr = yawc[i]
      			# account for wrap around +- PI
			if((next-curr) < -math.pi):
				next = next + math.pi*2
			if((next-curr) >  math.pi):
				next = next - math.pi*2
			yaw_ratec[i] = (next-curr)/dt
		
		# Define object that will be published
		target = PositionTarget()
		rr = rospy.Rate(self.RATE)
		k=0
		release() # stop navigate service from publishing before beginning the figure-8 publishing
		while not rospy.is_shutdown():
		
			target.header.frame_id = self.FRAME  # Define the frame that will be used
			
			target.coordinate_frame = 1 #MAV_FRAME_LOCAL_NED  # =1
			
			target.type_mask = 0  # Use everything!
						# PositionTarget::IGNORE_VX +
						# PositionTarget::IGNORE_VY +
						# PositionTarget::IGNORE_VZ +
						# PositionTarget::IGNORE_AFX +
						# PositionTarget::IGNORE_AFY +
						# PositionTarget::IGNORE_AFZ +
						# PositionTarget::IGNORE_YAW;

			# Gather position for publishing
			target.position.x = posx[k]
			target.position.y = posy[k]
			target.position.z = posz[k]
			
			# Gather velocity for publishing
			target.velocity.x = velx[k]
			target.velocity.y = vely[k]
			target.velocity.z = velz[k]
			
			# Gather acceleration for publishing
			target.acceleration_or_force.x = afx[k]
			target.acceleration_or_force.y = afy[k]
			target.acceleration_or_force.z = afz[k]
			
			# Gather yaw for publishing
			target.yaw = yawc[k]
			
			# Gather yaw rate for publishing
			target.yaw_rate = yaw_ratec[k]
			
			# Publish to the setpoint topic
			self.publisher.publish(target)
			
		
		
			# This loop initializes when the figure-8 is complete, therefore it will navigate back to the origin and setpoint publishing will be continued as needed to avoid entering failsafe mode.
			k = k+1
			if k >= self.STEPS: 
				self.navigate_wait(z=self.FLIGHT_ALTITUDE, yaw=float('nan'), speed=0.5, frame_id = self.FRAME)
				break
			rr.sleep()

		# Wait for 3 seconds
		rospy.sleep(3)
		# Perform landing
		
		res = land()
		 
		if res.success:
			print('Drone is landing')
			
		#res = fcuModes()
		#res.setAutoLandMode()


		# Debug section, need matplotlib to plot the results for SITL
		plt.figure(1)
		plt.subplot(211)
		plt.plot(t,posx,'r',label='x-pos')
		plt.plot(t,posy,'b--',label='y-pos')
		plt.plot(t,posz,'g:',label='z-pos')
		plt.legend()
		plt.grid(True)
		plt.ylabel('Position [m]')
		plt.subplot(212)
		plt.plot(t,velx,'r',label='x-vel')
		plt.plot(t,vely,'b--',label='y-vel')
		plt.plot(t,velz,'g:',label='z-vel')
		plt.legend()
		plt.grid(True)
		plt.ylabel('Velocity [m/s]')
		plt.xlabel('Time [s]')

		plt.figure(2)
		plt.subplot(211)
		plt.plot(t, afx,'r',label='x-acc')
		plt.plot(t, afy,'b--',label='y-acc')
		plt.plot(t, afz,'g:',label='z-acc')
		plt.ylabel('af [m/s^2]')
		plt.legend()
		plt.grid(True)
		plt.subplot(212)
		plt.plot(t,yawc,'r',label='yaw')
		plt.ylabel('Magnitude')
		plt.xlabel('Time [s]')
		plt.plot(t,yaw_ratec,'b--',label='yaw_rate')
		plt.legend()
		plt.grid(True)
		plt.show()
		
		# rospy.spin() # press control + C, the node will stop.

if __name__ == '__main__':
	try:
		# Define the performance parameters here which starts the script
		q=clover(FLIGHT_ALTITUDE = 1.0, RATE = 50, RADIUS = 1, CYCLE_S = 18, REF_FRAME = 'aruco_map')
		
		q.main()
		
	except rospy.ROSInterruptException:
		pass
