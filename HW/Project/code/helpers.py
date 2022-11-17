
import numpy as np
#import code.core as mr
import core as mr


#helper functions
def write_csv_line(csv_filename, data):
    with open(csv_filename, 'a') as f:
        data_str = ','.join([str(i) for i in data]) + '\n'
        f.write(data_str)
##


def write_csv_mat(csv_filename, mat):
    f = open(csv_filename, 'w') #clear out old data
    f.close()

    #datatype handling
    mat = np.matrix(mat).tolist()
    for row in mat:
            write_csv_line(csv_filename, row)
###

def process_array(arr):
    '''Rounds the angles in the trajectory array to be bounded between 0 and 2pi.'''
    return list( map(lambda x: (x % (2*np.pi) ).tolist(), arr)   )

###

def convertSE3toJointAngles(traj, Blist, M, thetalist0, eomg, ev, gripperState):
    '''
    iterates through a list of trans. matrices and:
	    - runs inverse kinematics for the given 
	    - stores joint angles in a a list of lists of angles
    - uses:
	    - traj = CartesianTrajectory(Xstart,Xend,Tf,N,method)
	    - [thetalist,success] = IKinBody(Blist,M,T,thetalist0,eomg,ev)
    - returns the list of joint angles to move through the trajectory
    
    inputs:
        - traj: a list of SE(3) matrices corresponding to a trajectory
        - Blist: the screw axes of the robot expressed in the S frame
        - M: the home configuration of the end effector
        - thetalist0: the initial joint angles of the robot before the traj,  
        used for Inverse Kinematics
        - eomg, ev: allowable error in angular, linear velocity
    '''
    
    #thetalist updates on every iteration of loop
    thetalist_array = np.zeros([len(traj), len(thetalist0)])

    for i, SE3 in enumerate(traj):
        [thetalist,success] = mr.IKinBody(Blist,M,SE3,thetalist0,eomg,ev)
        thetalist_array[i, :] = thetalist
        thetalist0 = thetalist[:]

    #modifications to make to include chassis phi, x, y, and 4 wheels:
    #- add an extra 3 rows onto the beginning of the array
    #- add an extra 4 rows onto the end of the array 
    arr_new = np.zeros([len(traj), 13])
    arr_new[:, 3:8] = thetalist_array
    arr_new[:,-1] = gripperState
    return arr_new

###

def generateStandoffSE3(Tsb, dist, unit_vec):
    '''
    - generate standoff position from object initial position and final position
    - inputs: 
        - Tsb: transformation matrix of object location rel. to. world,
		- dist: scalar distance of standoff above part
        - unit_vec: direction of displacement of standoff
	- operations: applies a translation in the right axis of the object frame
		to produce a transformation matrix for the standoff
	- uses: mr.RpToTrans(R, p), np.identity()
	- returns: a transformation matrix corr. to. standoff
    '''

    return np.matrix(Tsb) * np.matrix(mr.RpToTrans(
                np.identity(3), dist * np.array(unit_vec))  )  

###

def jointAnglesStartToEnd(Xstart, Xend, Tf, N, method, 
                            Blist, M, thetalist0, eomg, ev, gripperState):
    '''
    - generates a trajectory using CartesianTrajectory()
    - converts to a list of joint angles using convertSE3toJointAngles()
    - returns an array of thetalists
    '''
    traj = mr.CartesianTrajectory(Xstart,Xend,Tf,N,method)
    #traj = mr.ScrewTrajectory(Xstart,Xend,Tf,N,method)

    joint_angles_array = convertSE3toJointAngles(traj, Blist, M, thetalist0, eomg, ev, gripperState)
    return joint_angles_array

###
def gripper_motion(thetalist, N, state):
    #return a matrix of all the joint angles we started with, only
    #with a 1 at the end for the gripper state
    return np.tile(   np.append(thetalist[:-1],state),   (N,1))
