'''
MECH ENG 449
SEAN MORTON
PROJECT STEP 2
'''

import core as mr
import numpy as np
from tqdm import tqdm

from geometry import *
from helpers import *

def TrajectoryGenerator(Tse_i, Tsc_i, Tsc_f, Tce_grasp, Tce_standoff, k):
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

    Returns:
    - arr_new: the concatenated array of all 8 segments of the trajectory
    '''

    #define params needed to make trajectories
    method = 5

    #define time each segment should take
    time_grasp = 0.63
    time_standoff = 3

    t_array = [
        8,
        time_standoff,
        time_grasp,
        time_standoff,
        8,
        time_standoff,
        time_grasp,
        time_standoff,
    ] 

    Tf = sum(t_array)
    N = Tf / (k*0.01)

    #define number of datapoints in each segment of traj
    N_array = N_array = list(map(lambda x: int(x /(k*0.01)), t_array))

    Tse_standoff_i = np.dot(Tsc_i, Tce_standoff)
    Tse_grasp_i    = np.dot(Tsc_i, Tce_grasp)
    Tse_standoff_f = np.dot(Tsc_f, Tce_standoff)
    Tse_grasp_f    = np.dot(Tsc_f, Tce_grasp)

    #define starting and ending SE(3) matrices for each segment of 
    #trajectory array
    motion_limits = [
        (Tse_i, Tse_standoff_i),
        (Tse_standoff_i, Tse_grasp_i),
        (None, None),
        (Tse_grasp_i, Tse_standoff_i),
        (Tse_standoff_i, Tse_standoff_f),
        (Tse_standoff_f, Tse_grasp_f),
        (None, None),
        (Tse_grasp_f, Tse_standoff_f)
        ]

    gripperState = 0
    positions_array = []

    #repeat the process of trajectory generation for each segment of motion.
    #for gripping segments, produce a constant set of the same SE(3) matrix and
    #gripper state over time, to allow gripper to open/close
    print("TrajectoryGenerator():")
    for i in tqdm(range(len(N_array))):

        #case: gripper needs to take time to open or close
        if (i == 2 or i == 6):
            gripperState = not gripperState
            array_rep13_list = gripper_motion(array_rep13_list[-1], N_array[i], gripperState)

        #default: travel from one place to another
        else:
            SE3_matrices = mr.ScrewTrajectory(motion_limits[i][0],motion_limits[i][1], \
                t_array[i], N_array[i], method)
            array_rep13_list = [SE3matToArray13(x, gripperState) for x in SE3_matrices]

        gripperState = array_rep13_list[-1][-1]
        positions_array.append(np.array(array_rep13_list))

    #convert list of trajectory segments to array and store as csv
    print("\nTrajGenerator - shape of trajectory segments:")
    for arr in positions_array:
        print(arr.shape)
    
    arr_new = np.concatenate(positions_array, axis=0)
    write_csv_mat('../csv/trajectory.csv', arr_new)
    print("\nResults written to /csv folder.")

    return arr_new


######


if __name__ == '__main__':
    
    #See geometry.py for starting transformation matrices
    #choose starting Tse equal to the values found in CoppeliaSim for Scene 8
    Tse_i = mr.RpToTrans(np.identity(3),[9.4E-2, 9.4E-2, 5.94E-1])
    k = 1
    TrajectoryGenerator(Tse_i, Tsc_i, Tsc_f, Tce_grasp, Tce_standoff, k)