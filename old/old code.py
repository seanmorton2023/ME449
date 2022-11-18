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
    #traj = mr.CartesianTrajectory(Xstart,Xend,Tf,N,method)
    traj = mr.ScrewTrajectory(Xstart,Xend,Tf,N,method)

    joint_angles_array = convertSE3toJointAngles(traj, Blist, M, thetalist0, eomg, ev, gripperState)
    return 

def process_array(arr):
    '''Rounds the angles in the trajectory array to be bounded between 0 and 2pi.'''
    return list( map(lambda x: (x % (2*np.pi) ).tolist(), arr)   )

###

    #positions of the object in the frame of the armbase
    T0s = mr.TransInv(np.dot(Tsb_0, Tb0)).tolist()
    T0c_i = np.dot(T0s, Tsc_i).tolist()
    T0c_f = np.dot(T0s, Tsc_f).tolist()

    #orientations of the end effector at object,
    T0e_i = np.dot(T0c_i, Tce_grasp).tolist()
    T0e_f = np.dot(T0c_f, Tce_grasp).tolist()

    #...and at standoffs Ni, Nf
    T0e_i = np.dot(T0c_i, Tce_grasp).tolist()
    T0e_ni = np.dot(T0c_i, Tce_standoff).tolist()
    T0e_f = np.dot(T0c_f, Tce_grasp).tolist()
    T0e_nf = np.dot(T0c_f, Tce_standoff).tolist()

    #define starting and ending positions of each motion
    motion_limits = [
        (M0e, T0e_ni),
        (T0e_ni, T0e_i),
        (None, None),
        (T0e_i, T0e_ni),
        (T0e_ni, T0e_nf),
        (T0e_nf, T0e_f),
        (None, None),
        (T0e_f, T0e_nf)
        ]



def test_matrices():
    #generating standoff SE3
    SE3mat1 = generateStandoffSE3(Tsc_i, 2, [0,0,1])
    expected1 = mr.RpToTrans(np.identity(3), [1, 0, 2.025])
    assert np.allclose(SE3mat1, expected1), f"Received: \n{SE3mat1}\n"
    
    SE3mat2 = generateStandoffSE3(Tsc_i, 2, [0,1,0])
    expected2 = mr.RpToTrans(np.identity(3), [1, 2, 0.025])
    assert np.allclose(SE3mat2, expected2), f"Received: \n{SE3mat2}\n"

    print("All assertions passed")

    #testing datatypes of different matrices
    print("EE rotation matrix and final posn:")
    print(R_tip_to_obj)
    print(T0e_nf)
