'''
MECH ENG 449
SEAN MORTON
PROJECT STEP 1
'''

import core as mr
from geometry import *
from helpers import *

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
	robot_config12 = np.array(robot_config12).flatten()
	robot_speeds9 = np.array(robot_speeds9).flatten()

	if len(robot_config12) != 12 or len(robot_speeds9) != 9:
		raise Exception(f"Lengths of arrays: {len(robot_config_12)} {len(robot_speeds9)}")

	#world coords, joint angles theta, wheel angles phi
	q_array      = robot_config12[0:3]
	theta_array  = robot_config12[3:8]
	phi_array    = robot_config12[8:12]

	u_array      = robot_speeds9[0:4]
	thetad_array = robot_speeds9[4:9]

	#old chassis position
	phi, x, y = q_array

	#mapping from wheel velocities to velocity in world coords
	c = 1/(l+w) #constant for mapping from wheel speeds to omega in world coords
	F = r/4 * np.array([
		[-c, c,  c, -c],
		[ 1, 1,  1,  1],
		[-1, 1, -1,  1]	
	])

	#use EulerStep to calculate next arm joint angles, wheel angles,
	thetalist = robot_config12[3:12].reshape((1,9))
	dthetalist = np.append(thetad_array, u_array).reshape((1,9))
	ddthetalist = np.zeros(max(robot_speeds9.shape)).reshape((1,9)) #as long as the thetad array
	[thetanext, _] = mr.EulerStep(thetalist, dthetalist, ddthetalist, dt)

	u_array = u_array.reshape((4, 1))
	Vb = np.dot(F, u_array * dt)
	Vb6 = np.zeros((6,1))
	Vb6[2:5] = Vb
	Vb6 = Vb6.flatten()

	#integrate the twist to get posn in world frame
	se3mat = mr.VecTose3(Vb6)

	#method for calculating q_next from textbook
	wbz, vbx, vby = np.array(Vb).flatten().tolist()
	if np.isclose(wbz, 0, atol=1E-4):
		delta_qb = np.array([0, vbx, vby])
	else:
		delta_qb = np.array([
			wbz,
			(vbx * np.sin(wbz) + vby * (np.cos(wbz)-1) )/wbz,
			(vby * np.sin(wbz) + vbx * (1-np.cos(wbz)) )/wbz
		])

	#get from body frame to the space frame
	q_transform = np.array([
		[1,           0,            0],
		[0, np.cos(phi), -np.sin(phi)],
		[0, np.sin(phi),  np.cos(phi)]
	])

	delta_q = np.dot(q_transform, delta_qb)
	q_new = np.array(q_array).flatten() + np.array(delta_q).flatten()
	robot_config12_new = np.append(q_new, thetanext.flatten())

	return robot_config12_new


def RunNextState(robot_config12, u, thetad, dt, w_max):
	'''Takes an initial configuration of the youBot and simulates 
	constant controls for one second. For example, you can set Δt
	to 0.01 seconds and run a loop that calls NextState 100 times 
	with constant controls (u,thetad). This program will 
	write a csv file, where each line has 13 values separated by 
	commas (the 12-vector consisting of 3 chassis configuration 
	variables, the 5 arm joint angles, and the 4 wheel angles, plus
	a "0" for "gripper open") representing the robot's configuration
	after each integration step. 
	'''
	filename = '../csv/test/next_state.csv'
	#clear out existing data in this file
	f = open(filename, 'w') #clear out old data
	f.close()

	if (len(u) != 4 or len(thetad) != 5):
		raise Exception("TestNextState: size mismatch in u or thetad")

	#let speed controls be constant
	robot_speeds9 = np.append(u, thetad)
	csv_data = np.zeros(13)

	for i in range(100):
		robot_config12_new = NextState(robot_config12, robot_speeds9, dt, w_max)
		csv_data[0:12] = robot_config12_new.flatten()
		write_csv_line(filename, csv_data)
		robot_config12 = robot_config12_new


def TestNextState():
	'''A program for testing NextState() using sample imputs u1,
	u2, and u3. Writing to CSV is handled in RunNextState(), another
	helper function for testing.
	'''

	test_base_joints = [0, 0, 0, 0, 0, 0.2, -1.6, 0]
	test_wheels = [0,0,0,0]
	robot_config12 = test_base_joints + test_wheels

	u1 = [ 10, 10,  10,  10] #x direction velocity
	u2 = [-10, 10, -10,  10] #y
	u3 = [-10, 10,  10, -10] #spinning

	thetad = np.zeros(5)
	dt = 0.01
	#w_max = 1000
	w_max = 5

	RunNextState(robot_config12, u1, thetad, dt, w_max)


if __name__ == '__main__':
    TestNextState()

