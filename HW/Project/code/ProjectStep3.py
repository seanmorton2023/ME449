'''
MECH ENG 449
SEAN MORTON
PROJECT STEP 3
'''

import core as mr
from geometry import *
from helpers import *
import sys

######

def FeedbackControl(X, Xd, Xd_next, Kp, Ki, Xerr_int, dt):
	'''	Calculates the next configuration of the robot based on the current state.

	Input:
		- X          (SE(3)mat): 		The current actual end-effector configuration X (also written Tse).
		- Xd         (SE(3)mat): 		The current end-effector reference configuration Xd (i.e., Tse,d).
		- Xd_next    (SE(3)mat): 		The end-effector reference configuration at the next timestep
										in the reference trajectory, Xd,next (i.e., Tse,d,next), 
										at a time Δt later.
		- Kp, Ki  (R6x6, numpy):		The PI gain matrices.
		- Xerr_int  (6x1 twist): 		the integral of error over time; will be added to and returned by the function
		- dt            (float): 		The timestep Δt between reference trajectory configurations.

	Output: V_new, (Xerr, Xerr_int)
		- V_new     (6x1 twist):		commanded end effector twist
		- Xerr      (6x1 twist):		current error from the desired position
		- Xerr_int  (6x1 twist):		integral of error from desired position

	'''

	#compute adjoint to transform desired EE config into current frame
	Ted = np.dot(mr.TransInv(X), Xd)
	Ad_ed = mr.Adjoint(Ted)
	
	#compute error twist
	se3mat_err = mr.MatrixLog6(Ted)
	Xerr = mr.se3ToVec(se3mat_err)

	#compute Vd by taking matrix log of Xd^-1*Xd_next and dividing by timestep dt
	se3mat_unit = mr.MatrixLog6( np.dot(  mr.TransInv(Xd), Xd_next) )
	Vd = (1/dt) * mr.se3ToVec(se3mat_unit)

	#calculate products + sums of factors
	V_new = np.dot(Ad_ed, Vd) + np.dot(Kp, Xerr) + np.dot(Ki, Xerr_int)

	#add to the integral of error
	Xerr_int += (Xerr * dt)

	print("\nFeedbackControl debug:")
	#print(f"\nVd:         \n{Vd.round(3)}")
	#print(f"\nAd_ed * Vd: \n{np.dot(Ad_ed, Vd).round(3)}")
	print(f"\nV_new:      \n{V_new.round(3)}")
	#print(f"\nXerr:       \n{Xerr.round(3)}")
	#print(f"\nXerr_int:   \n{Xerr_int.round(3)}")

	return V_new, (Xerr, Xerr_int)


#####

def CalculateJe(robot_config8, Tb0, M0e, Blist):
	'''Calculates the combined Jacobian Je = [Jbase, Jarm]
	given a configuration of the robot. Used for solving for
	the joint speeds and wheel speeds of the robot.

	Inputs:
	- robot_config8: 3 chassis posns and 5 joint angles
	- Tb0: offset between the base of the chassis and the
		base of the robot arm
	- M0e: home configuration of the end effector
	- Blist: screw axes of the robot in the end effector frame

	Output:
	- Je: combined Jacobian
	'''
	q_array     = robot_config8[0:3]
	theta_array = robot_config8[3:8]

	#mapping from wheel velocities to velocity in world coords
	r = 0.0475
	l = 0.47/2.0
	w = 0.3/2.0

	c = 1/(l+w) #constant for mapping from wheel speeds to omega in world coords
	F = r/4 * np.array([
		[-c, c,  c, -c],
		[ 1, 1,  1,  1],
		[-1, 1, -1,  1]	
	])

	F6 = np.zeros((6,4))
	F6[2:5, :] = F

	#determine Tsb(q) from omnidirectional robot control
	[phi, x, y] = q_array
	Tsb = ChassisSE3(phi, x, y)

	#determine Jacobian matrices of both arm and mobile base
	#Blist, M0e, Tb0 determined in Geometry file
	Jarm = mr.JacobianBody(Blist, theta_array)
	T0e = mr.FKinBody(M0e, Blist, theta_array)
	Teb = mr.TransInv(np.dot(Tb0, T0e))
	Jbase = np.dot(mr.Adjoint(Teb), F6)

	#J_base is 6x4: 4 wheels
	#J_arm  is 6x5: 5 joints
	Je = np.zeros((6, 9))
	Je[:, 0:4] = Jbase
	Je[:, 4:9] = Jarm

	return Je


def TestFeedbackControl():

	#global Xerr_int
	Xerr_int = 0

	X = np.array([
		[ 0.170, 0, 0.985, 0.387],
		[     0, 1,     0,     0],
		[-0.985, 0, 0.170, 0.570],
		[     0, 0,     0,     1]
	])

	Xd = np.array([
		[ 0, 0, 1, 0.5],
		[ 0, 1, 0,   0],
		[-1, 0, 0, 0.5],
		[ 0, 0, 0,   1]
	])

	Xd_next = np.array([
		[ 0, 0, 1, 0.6],
		[ 0, 1, 0,   0],
		[-1, 0, 0, 0.3],
		[ 0, 0, 0,   1]
	])

	Kp = 0
	Ki = 0
	dt =  0.01

	V_new, _ = FeedbackControl(X, Xd, Xd_next, Kp, Ki, Xerr_int, dt)

	#convert end effector twist into joint and wheel velocities
	robot_config8 = np.array([0, 0, 0, 0, 0, 0.2, -1.6, 0])
	Je = CalculateJe(robot_config8, Tb0, M0e, Blist)
	#u_thetad = np.dot(JPseudoInverse(Je), V_new)
	u_thetad = np.dot(np.linalg.pinv(Je, rcond=1e-4), V_new)

	#print("\nProjectStep3 debug:")
	print(f"\nu, thetadot: \n{u_thetad.round(1)}")

	#----------------------------------#
	#using nonzero Kp

	Kp = np.identity(6)
	Ki = 0
	Xerr_int = 0

	print("\nWith nonzero Kp:")
	V_new, _ = FeedbackControl(X, Xd, Xd_next, Kp, Ki, Xerr_int, dt)
	Je = CalculateJe(robot_config8, Tb0, M0e, Blist)
	u_thetad = np.dot(np.linalg.pinv(Je, rcond=1e-4), V_new)
	print(f"\nu, thetadot: \n{u_thetad.round(1)}")


if __name__ == '__main__':
	TestFeedbackControl()












#### old stuff that I still want to keep for the full integration of the project

'''
	#world coords, joint angles theta, wheel angles phi
	q_array      = robot_config12[0:3]
	theta_array  = robot_config12[3:8]
	phi_array    = robot_config12[8:12]

	u_array      = robot_speeds9[0:4]
	thetad_array = robot_speeds9[4:9]

	#mapping from wheel velocities to velocity in world coords
	c = r/(l+w) #constant for mapping from wheel speeds to omega in world coords
	F = np.array([
		[-c, c,  c, -c],
		[ 1, 1,  1,  1],
		[-1, 1, -1,  1]	
	])

	F6 = np.zeros((6,4))
	F6[2:5, :] = F

	#chassis twist Vb
	Vb  = np.dot( F, u_array)
	Vb6 = np.dot(F6, u_array)

	#determine Tsb(q) from omnidirectional robot control
	phi = q_array[0]
	x   = q_array[1]
	y   = q_array[2]  

	Rsb = np.array([
		[np.cos(phi), -np.sin(phi), 0]
		[np.sin(phi),  np.cos(phi), 0]
		[          0,           0,  1]
	])
	
	Tsb = mr.RpToTrans(Rsb, [x, y, 0.0963])

	#determine Jacobian matrices of both arm and mobile base
	Jarm = mr.JacobianBody(Blist, theta_array)
	T0e = mr.FKinBody(M0e, Blist, theta_array)
	Teb = mr.TransInv(np.dot(Tb0, T0e))
	Jbase = np.dot(mr.Adjoint(Teb), F6)

	#J_base is 6x4: 4 wheels
	#J_arm  is 6x5: 5 joints
	Je = np.zeros((6, 9))
	Je[:, 0:4] = Jbase
	Je[:, 4:9] = Jarm

	#construct end effector twist Ve
	u_thetad = robot_speeds9.reshape((9,1))
	Ve = np.dot(Je, u_thetad)


'''
