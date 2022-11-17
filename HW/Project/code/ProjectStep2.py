'''
MECH ENG 449
SEAN MORTON
PROJECT STEP 2
'''

#import code.core as mr
#from code.geometry import *
#from code.helpers import *

import core as mr
from geometry import *
from helpers import *
import sys

def TrajectoryGenerator(Tsb_0, Tb0, Tse_i, Tsc_i, Tsc_f, \
    Tce_grasp, Tce_standoff, k, \
    M0e, Blist):
    '''
    Generates a trajectory of eight concatenated trajectory segments, as described 
    in the problem statement.

    Parameters:
    - Tse_i: initial config of end effector
    - Tsc_i: cube's initial config
    - Tsc_f: cube's desired final config
    - Tce_grasp: config of end effector rel. to cube while grasping
    - Tce_standoff: end-effector's standoff configuration above the cube, before and after grasping
    - k: The number of trajectory reference configurations per 0.01 seconds
    '''

    #define params needed to make trajectories
    Tf = 22
    N = Tf / (k*0.01)
    method = 5
    eomg = 0.001
    ev = 0.0001
    thetalist0 = [0,0,0,0,0]

    #define time each segment should take
    time_grasp = 0.63
    time_standoff = 2

    t_array = [
        6,
        time_standoff,
        time_grasp,
        time_standoff,
        0,
        time_standoff,
        time_grasp,
        time_standoff,
    ] 
    t_array[4] = Tf - sum(t_array) 

    #define number of datapoints in each segment of traj
    N_array = N_array = list(map(lambda x: int(x /(k*0.01)), t_array))

    #assert sum(N_array) == N, f"Total number of datapoints: {sum(N_array)}"
    #assert sum(t_array) == Tf, f"Total time taken: {sum(t_array)}"

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

    #test out a sample trajectory to make sure function works
    #t0 = time.time()
    #joint_angles_v2 = jointAnglesStartToEnd(M0e, T0e_ni, Tf, N, method, Blist, M0e, thetalist0, eomg, ev)
    #tf = time.time()
    #print(f"Elapsed: {round(tf - t0, 1)} seconds")
    #write_csv_mat('../csv/test_traj.csv', joint_angles_v2)

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

    gripperState = 1
    thetalist = thetalist0[:]

    #np.set_printoptions(threshold=sys.maxsize)

    joint_angles_list = []
    for i in range(len(N_array)):

        #case: gripper needs to take time to open or close
        if (i == 2 or i == 6):
            gripperState = not gripperState
            print(f"Iteration: {i}")

            joint_angles = gripper_motion(joint_angles[-1,:], N_array[i], gripperState)

        #default: travel from one place to another
        else:

            print(f"Iteration: {i}")
            print(f"Thetalist: {thetalist}\n\n")

            joint_angles = jointAnglesStartToEnd(motion_limits[i][0], motion_limits[i][1], \
                t_array[i], N_array[i], method, Blist, M0e, thetalist, eomg, ev, gripperState)

        #thetalist = process_array(joint_angles[-1,3:8])
        thetalist = joint_angles[-1,3:8]

        gripperState = joint_angles[-1,-1]
        joint_angles_list.append(joint_angles)

    #convert list of trajectory segments to array and store as csv
    for arr in joint_angles_list:
        #print(arr)
        print(type(arr))
        print(arr.shape)
    
    arr_new = np.concatenate(joint_angles_list, axis=0)
    write_csv_mat('../csv/test_traj.csv', arr_new)

    #print(joint_angles_1)
    #print(joint_angles_1[-1,3:8])
    #joint_angles_2 = jointAnglesStartToEnd(T0e_ni, T0e_i, \
    #    t_array[1], N_array[1], method, Blist, M0e, joint_angles_1[-1,3:8], eomg, ev)
    
    #write_csv_mat('../csv/test_traj.csv', joint_angles_2)


if __name__ == '__main__':

    #standoff matrices in space frame
    dist = 0.02 #2cm
    Tsn_i = generateStandoffSE3(Tsc_i, dist, [0,0,1]).tolist()
    Tsn_f = generateStandoffSE3(Tsc_f, dist, [0,0,1]).tolist()

    #orientation of end effector frame rel. to. object frame is 135deg rotation about y
    R_ee_to_obj = R.from_euler('y', 135, degrees = True).as_matrix()
    Tce_grasp = mr.RpToTrans(R_ee_to_obj, [0,0,0]) #orientation relative to object
    Tce_standoff = np.dot(
        mr.RpToTrans(np.identity(3),dist * np.array([0,0,1])), \
        mr.RpToTrans(R_ee_to_obj, [0,0,0]) \
    ).tolist() #orientation relative to object at standoff

    #other parameters for starting TrajectoryGenerator
    k = 1
    Tse_i = np.dot(Tsb_0, np.dot(Tb0, M0e)).tolist()

    TrajectoryGenerator(Tsb_0, Tb0, Tse_i, Tsc_i, Tsc_f, \
    Tce_grasp, Tce_standoff, k, \
    M0e, Blist)