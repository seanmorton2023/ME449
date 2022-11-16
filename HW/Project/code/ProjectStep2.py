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
    Tf = 30
    N = Tf / (k*0.01)
    method = 5
    eomg = 0.001
    ev = 0.0001
    thetalist0 = [0,0,0,0,0]

    #define time each segment should take
    time_grasp = 0.63
    time_standoff = 4

    t_array = [
        8,
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

    assert sum(N_array) == N, f"Total number of datapoints: {sum(N_array)}"
    assert sum(t_array) == Tf, f"Total time taken: {sum(t_array)}"

    #positions of the object in the frame of the armbase
    T0s = mr.TransInv(np.dot(Tsb_0, Tb0)).tolist()
    T0c_i = np.dot(T0s, Tsc_i).tolist()
    T0c_f = np.dot(T0s, Tsc_f).tolist()

    #orientations of the end effector at object,
    T0e_i = np.dot(T0c_i, Tce_grasp).tolist()
    T0e_f = np.dot(T0c_f, Tce_grasp).tolist()

    #...and at standoffs Ni, Nf
    T0e_i = np.dot(T0c_i, Tce_grasp)
    T0e_ni = np.dot(T0c_i, Tce_standoff)
    T0e_f = np.dot(T0c_f, Tce_grasp)
    T0e_nf = np.dot(T0c_f, Tce_standoff)

    #test out a sample trajectory to make sure function works
    t0 = time.time()
    joint_angles_v2 = jointAnglesStartToEnd(M0e, T0e_ni, Tf, N, method, Blist, M0e, thetalist0, eomg, ev)
    tf = time.time()
    print(f"Elapsed: {round(tf - t0, 1)} seconds")
    write_csv_mat('../csv/test_traj.csv', joint_angles_v2)



if __name__ == '__main__':

    #standoff matrices in space frame
    dist = 0.02 #2cm
    Tsn_i = generateStandoffSE3(Tsc_i, dist, [0,0,1]).tolist()
    Tsn_f = generateStandoffSE3(Tsc_f, dist, [0,0,1]).tolist()

    #orientation of end effector frame rel. to. object frame is 135deg rotation about y
    R_ee_to_obj = R.from_euler('y', 135, degrees = True)
    Tce_grasp = mr.RpToTrans(R_tip_to_obj.as_matrix(), [0,0,0]) #orientation relative to object
    Tce_standoff = np.dot(
        mr.RpToTrans(np.identity(3),dist), \
        mr.RpToTrans(R_ee_to_obj, dist * [0,0,1]) \
    ) #orientation relative to object at standoff

    pass
