# Important!! for python the arrays start at 0 instead of 1 like matlab, so all definitions had to be adjusted with reference to array and matrix definitions

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import path
from scipy.integrate import simps
from scipy.interpolate import griddata


from clover_functions import CLOVER_COMPONENTS, CLOVER_STREAM_GEOMETRIC_INTEGRAL, CLOVER_KUTTA, CLOVER_STREAMLINE

# Main script
np.seterr(divide='ignore', invalid='ignore')  # Ignore division by zero and invalid value warnings

# Kutta condition flag (decide which one to use)
flagKutta = np.array([0, 1])

nacaseries = input('Enter the 4-digit naca series = ')
c = float(input('Enter the chord length = '))
a = float(input('Enter the angle of attack in degree = '))

n = 60
# Set the uniform velocity value
U_inf = 0
V_inf = 0

# Source and sink strengths
g_source = 0.3
g_sink = 2.75

# Source and Sink Locations
xs, ys = 0, 0
xsi, ysi = 10, 10

# Circle instead of airfoil
center1 = np.array([5, 5])
radius1 = 2

theta1 = np.linspace(0, -2 * np.pi, n + 1)
xa = center1[0] + radius1 * np.cos(theta1)
ya = center1[1] + radius1 * np.sin(theta1)

alpha = np.radians(a)

[xmid, ymid, dx, dy, Sj, phiD, rhs] = CLOVER_COMPONENTS(xa, ya, U_inf, V_inf, g_source, g_sink, xs, ys, xsi, ysi, n)

deltaD = phiD + 90
betaD = deltaD - a
betaD[betaD > 360] -= 360

phi = np.radians(phiD)
beta = np.radians(betaD)


# Circle trailing point
xtrail = (radius1 + 0.001) * np.cos(45 * np.pi / 180)
ytrail = (radius1 + 0.001) * np.sin(45 * np.pi / 180)
trail_point = np.array([center1[0], center1[1]]) + np.array([xtrail, ytrail])

# Evaluate gemoetric integral matrix without the kutta condition equation
I = CLOVER_STREAM_GEOMETRIC_INTEGRAL(xmid, ymid, xa, ya, phi, Sj, n)

# Form the last line of the system of equations with the kutta condition
[I, rhs] = CLOVER_KUTTA(I, trail_point, xa, ya, phi, Sj, n, flagKutta, rhs, U_inf, V_inf, xs, ys, xsi, ysi, g_source, g_sink)

# calculating the vortex density (and stream function from kutta condition)
# by solving linear equations given by
g = np.linalg.solve(I, rhs.T)  # = gamma = Vxy/V_infinity for when we are using U_inf as the free flow magnitude
# broken down into components (using cos(alpha), and sin(alpha)), and free flow is the only other RHS
# contribution (no source or sink). It is equal to Vxy 
# when we define the x and y (U_inf and V_inf) components seperately.


# Plot the airfoil
plt.figure(figsize=(8, 4))
plt.plot(xa, ya, 'ko-')
plt.plot(trail_point[0], trail_point[1], 'o')
plt.xlim() #auto
plt.ylim() # auto
plt.xlabel('X Units')
plt.ylabel('Y Units')
plt.title('Airfoil')
plt.grid(True)
plt.show()

# Calculate and plot the pressure distribution
Cp = 1 - (g[:n] / (U_inf ** 2 + V_inf ** 2)) ** 2 # g solved for all the vortex densities
# and the stream function value on the object surface. 



plt.figure(figsize=(8, 4))
plt.plot(xmid[:n // 2] / c, Cp[:n // 2], '-*r')
plt.plot(xmid[n // 2:] / c, Cp[n // 2:], '-*b')
plt.gca().invert_yaxis()
plt.xlabel('x/c')
plt.ylabel('Cp')
plt.title('Cp vs x/c')
plt.grid(True)
plt.show()

# Plot the airfoil and panel normal vectors
plt.figure(figsize=(8, 4))
plt.fill(xa, ya, 'k')
for i in range(n):
    X = [xmid[i], xmid[i] + Sj[i] * np.cos(np.radians(betaD[i] + a))]
    Y = [ymid[i], ymid[i] + Sj[i] * np.sin(np.radians(betaD[i] + a))]
    plt.plot(X, Y, 'r-', linewidth=2)
plt.xlabel('X Units')
plt.ylabel('Y Units')
plt.xlim() # auto scaling
plt.ylim()
plt.axis('equal')
plt.title('Airfoil and Panel Normal Vectors')
plt.grid(True)
plt.show()


## Compute Streamlines with stream function velocity equation
# Too many gridpoints is not good, it will cause the control loop to run too slow
# in the offline_panel script
 # Grid parameters
nGridX = 20;                                                           # X-grid for streamlines and contours
nGridY = 20;                                                           # Y-grid for streamlines and contours
xVals  = [-1, 11];  # -0.5; 1.5                                                  # X-grid extents [min, max]
yVals  = [-1, 11];  #-0.3;0.3                                                 # Y-grid extents [min, max]
    
# Streamline parameters
stepsize = 0.1;   #0.01                                                     # Step size for streamline propagation
maxVert  = nGridX*nGridY*100;                                           # Maximum vertices
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
Xgrid = np.linspace(xVals[0], xVals[1], nGridX)
Ygrid = np.linspace(yVals[0], yVals[1], nGridY)
XX, YY = np.meshgrid(Xgrid, Ygrid)

Vxe = np.zeros((nGridX, nGridY))
Vye = np.zeros((nGridX, nGridY))

# Path to figure out if grid point is inside polygon or not
#AF = np.vstack((xa.T,ya.T)).T
AF = np.vstack((xa,ya)).T
#print(AF)
afPath = path.Path(AF)

for m in range(nGridX):
    for n in range(nGridY):
        XP, YP = XX[m, n], YY[m, n]
        u, v = CLOVER_STREAMLINE(XP, YP, xa, ya, phi, g, Sj, U_inf, V_inf, xs, ys, xsi, ysi, g_source, g_sink)

        if afPath.contains_points([[XP,YP]]):
            Vxe[m, n] = 0
            Vye[m, n] = 0
        else:
            Vxe[m, n] = u
            Vye[m, n] = v

Vxy = np.sqrt(Vxe**2 + Vye**2)
CpXY = 1 - Vxy**2

fig = plt.figure(5)
plt.cla()
np.seterr(under="ignore")
plt.streamplot(XX,YY,Vxe,Vye,linewidth=0.5,density=40,color='r',arrowstyle='-',start_points=XYsl)
plt.grid(True)
#plt.plot(XX,YY,marker='o',color='blue')
plt.axis('equal')
plt.xlim(xVals)
plt.ylim(yVals)
plt.fill(xa, ya, 'k')
plt.xlabel('X Units')
plt.ylabel('Y Units')
plt.title('Streamlines with Stream Function Velocity Equations')

plt.figure()
plt.clf()
plt.grid(True)
plt.axis('equal')
plt.xlim(xVals)
plt.ylim(yVals)
plt.contourf(XX, YY, CpXY, 100, edgecolors='none')
plt.fill(xa, ya, 'k')
plt.xlabel('X Units')
plt.ylabel('Y Units')
plt.title('Pressure Coefficient Contour')

plt.show()

# ----- export data as data files for the Gazebo to use

# np.save('../XX.npy',XX)
# np.save('../YY.npy',YY)
# np.save('../Vxe.npy',Vxe)
# np.save('../Vye.npy',Vye)

# test = griddata((XX.flatten(),YY.flatten()),Vxe.flatten(),(2,1),method='linear')
# print(test)

# # Test at extracting smooth velocity profile from streamplot function
# start_point = np.array([0.2,2])
# #streamplot = plt.streamplot(XX,YY,Vxe,Vye,linewidth=0.5,density=40,color='r',arrowstyle='-',start_points=start_point)
# streamplot = plt.streamplot(XX,YY,Vxe,Vye,linewidth=0.5,density=40,color='r',arrowstyle='-',start_points=XYsl)

# streamlines = streamplot.lines.get_paths()