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

from panel_functions import CLOVER_COMPONENTS, CLOVER_STREAM_GEOMETRIC_INTEGRAL, CLOVER_KUTTA, CLOVER_STREAMLINE

#from scipy.interpolate import griddata

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


			

		
lidar_angles = np.linspace(-180*(math.pi/180), 180*(math.pi/180), 360) # generate the array of lidar angles

		
		
def lidar_read(data):
	
		# The angles and ranges start at -45 degrees i.e. at the right side, then go counter clockwise up to the top i.e. 45 degrees
	ranges = data.ranges
	
	angles = lidar_angles
	
	
	telem = get_telemetry(frame_id='map')
		
	x_clover = telem.x
	y_clover = telem.y
	yaw = telem.yaw

	# put a safety factor on the detected obstacle
			# Reduce the range by a constant beta for each real range (set as diameter of the clover)
	beta = 0.1
	# Convert ranges to a NumPy array if it's not already
	ranges = np.array(ranges)
	# Subtract beta only from non-infinite values
	# Create a mask for non-infinite values
	non_inf_mask = np.isfinite(ranges)

	# Subtract beta only from non-infinite values
	# print(ranges)
	ranges = np.where(non_inf_mask, ranges - beta, ranges)
	# print(ranges)
		
		# Polar to Cartesion transformation for all readings (assuming angles are in standard polar coordinates).
	x_local = ranges*np.cos(angles)
	y_local = ranges*np.sin(angles)
		
		# Homogenous transformation matrix for 2D
	R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]) # rotation matrix
	T = np.vstack([np.hstack([R, np.array([[x_clover], [y_clover]])]),[0,0,1]]) # Homogenous transformation matrix
		
		# Lidar readings in homogenous coordinates
	readings_local = np.vstack([x_local, y_local, np.ones_like(x_local)])
		
		# Transform all lidar readings to global coordinates
	readings_global = np.dot(T, readings_local)
		
		# Extract the tranformed positions
	readings_global = readings_global[:2,:].T

	#print(readings_global)
	xa = readings_global[:,0]
	ya = readings_global[:,1]
	
	# Append row after row of data (to log readings)
	xf.append(xa.tolist())
	yf.append(ya.tolist())

def main():

	# Subscribe to the Lidar readings
	lidar = rospy.Subscriber('/ray_scan',LaserScan,lidar_read)
		

	
	
		
		

	rospy.spin()

		
			
			
			
			
			
	

		

if __name__ == '__main__':
	try:
		
		
		main()

		plt.figure(1)
		for x_row, y_row in zip(xf, yf):
			plt.plot(x_row,y_row, '-o',label=f'Reading {len(plt.gca().lines)}')
			#plt.fill(xa,ya,'k')
		plt.grid(True)
		plt.legend()
		plt.show()
		
		
	except rospy.ROSInterruptException:
		pass

		
	

