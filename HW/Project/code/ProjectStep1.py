'''
MECH ENG 449
SEAN MORTON
PROJECT STEP 2
'''

import core as mr
from geometry import *
from helpers import *
import sys

######


def NextState(robot_config12, robot_speeds9, dt, w_max):
	'''	Calculates the next configuration of the robot based on the current state.

	Input:
	- robot_config12: A 12-vector representing the current configuration
			of the robot (3 variables for the chassis configuration, 
			5 variables for the arm configuration, and 4 variables 
			for the wheel angles).
	- robot_speeds9: A 9-vector of controls indicating the wheel speeds u 
			(4 variables) and the arm joint speeds \dot{\theta} (5 variables).
	- dt: A timestep Δt.
	- w_max: A positive real value indicating the maximum angular 
			speed of the arm joints and the wheels. For example, 
			if this value is 12.3, the angular speed of the wheels 
			and arm joints is limited to the range [-12.3 radians/s,
				12.3 radians/s]. Any speed in the 9-vector of controls
				that is outside this range will be set to the nearest 
				boundary of the range.

	Output: A 12-vector representing the configuration of the 
		robot time Δt later.

	'''
	#variables to define: wheel geometry
	l = 0.47/2.0 #m
	w = 0.3/2.0  #m
	r = 0.0475   #m

	#checking of data
	robot_config12 = np.array(robot_config12)
	robot_speeds9 = np.array(robot_speeds9)

	if len(robot_config12) != 12 or len(robot_speeds9) != 9:
		raise Exception(f"Lengths of arrays: {len(robot_config_12)} {len(robot_speeds9)}")

	#world coords, joint angles theta, wheel angles phi
	q_array      = robot_config12[0:3]
	theta_array  = robot_config12[3:8]
	phi_array    = robot_config12[8:12]

	u_array      = robot_speeds9[0:4]
	thetad_array = robot_speeds9[4:9]

	phi = q_array[0]
	x   = q_array[1]
	y   = q_array[2]  

	#mapping from wheel velocities to velocity in world coords
	c = r/(l+w) #constant for mapping from wheel speeds to omega in world coords
	F = np.array([
		[-c, c,  c, -c],
		[ 1, 1,  1,  1],
		[-1, 1, -1,  1]	
	])

	#do checking if any of the wheel speeds are > w_max; filter w/ numpy function
	u_array = np.clip(u_array, -w_max, w_max)

	#use EulerStep to calculate next arm joint angles, wheel angles,
	ddtheta_list = np.zeros(max(robot_config9.shape)) #as long as the thetad array
	[thetanext, _] = mr.EulerStep(robot_speeds9, ddthetalist)
	
	#use odometry to find next chassis posn 
	u_array = u_array.reshape((4, 1))
	Vb = np.dot(F, u_array)
	Vb6 = np.zeros((6,1))
	Vb6[2:5] = Vb

	#integrate the twist to get posn in world frame
	se3mat = mr.VecToSE3()
	Tcurr_next = mr.MatrixExp(se3mat)

	R_curr = np.array([
		[np.cos(phi), -np.sin(phi), 0],
		[np.sin(phi),  np.cos(phi), 0],
		[          0,           0, 1],
	])

	T_curr = mr.RpToTrans(R_curr, [x, y, 0])
	T_next = np.dot(Tcurr_next, T_next)

	#extract phi, x, y from new world frame coords
	R, p = mr.TransToRp(T_next)
	x_new, y_new = p[0:2]

	so3mat = mr.MatrixLog3(R)
	vec = mr.so3ToVec(so3mat)
	phi_new = max(vec)

	#combine phi, x, y with new wheel + robot angles
	q_new = np.array([phi_new, x_new, y_new])
	robot_config12_new = np.append(q_new, thetanext.flatten())

	return robot_config12_new

if __name__ == '__main__':
    
    #See geometry.py for starting transformation matrices
    #k = 1
    #TrajectoryGenerator(Tse_i, Tsc_i, Tsc_f, Tce_grasp, Tce_standoff, k)

	test_joints = [0, 0, 0, 0, 0, 0.2, -1.6, 0]
	test_wheels = [0,0,0,0]
	robot_config12 = test_joints + test_wheels

	robot_speeds9 = np.zeros(9)
	dt = 0.01
	w_max = 1000

	NextState(robot_config12, robot_speeds9, dt, w_max, \
		Blist, M0e, Tb0)