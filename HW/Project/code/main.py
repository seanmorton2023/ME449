
import core as mr
import numpy as np
from tqdm import tqdm

from helpers import *
from geometry import *

from ProjectStep1 import NextState
from ProjectStep2 import TrajectoryGenerator
from ProjectStep3 import FeedbackControl, CalculateJe


def main():

    #define control parameters
    Kp = np.identity(6)
    Ki = 0
    dt = 0.01
    k = 1
    w_max = 1000

    Xerr_int = 0
    Xerr = 0

    # - Define initial robot posn as 13-vector: robot_config13
    init_base    = [0, 0, 0]       #chassis phi, x, y
    init_joints  = [0, 0, 0, 0, 0] # 5 robot joints
    init_wheels  = [0, 0, 0, 0]    # 4 wheel posns
    gripperState = 0               # open

    robot_config13 = init_base + init_joints + init_wheels + [gripperState]

    #- generate reference trajectory from init. EE config to desired EE config
    #base offset Tb0 and home EE config M0e defined in geometry.py
    [phi, x, y] = init_base
    Tsb_0  = ChassisSE3(phi, x, y).tolist()
    Tse_0 = np.dot(np.dot(Tsb_0, Tb0), M0e)
    ref_traj = TrajectoryGenerator(Tse_0, Tsc_i, Tsc_f, Tce_grasp, Tce_standoff, k)

    #iterate through trajectories generated
    N = len(ref_traj)

    #arrays for storage of results
    Xerr_array = np.zeros((N-1, 6)) 
    robot_config13_array = np.zeros((N,13))
    robot_config13_array[0] = robot_config13[:]

    print("\nMain():")
    for i in tqdm(range(N-1)):

        #calculate current SE(3) config from robot_config13
        base_array    = robot_config13[0:3] 
        joints_array  = robot_config13[3:8]
        robot_config8 = robot_config13[0:8] #base + joints, for Je
        wheels_array  = robot_config13[8:12]

        #find current position of end effector
        [phi, x, y] = base_array
        Tsb  = ChassisSE3(phi, x, y).tolist()
        T0e = mr.FKinBody(M0e, Blist, joints_array)
        Tse = np.dot(np.dot(Tsb, Tb0), T0e)

        #get desired SE(3) matrix from the reference trajectory
        rep13_array            = ref_traj[i] #representation of robot config in 1x13 vector
        Xd, _                  = Array13toSE3Mat(rep13_array) #from helpers.py

        rep13_next             = ref_traj[i+1]
        Xd_next, gripper_next  = Array13toSE3Mat(rep13_next)

        #print("Main() debug:")
        #print(f'\nXd: \n{Xd}')
        #print(f'\nXd_next: \n{Xd_next}')
        #print(f'\nTse: \n{Tse}')


        #apply control law; extract joint and wheel speeds from twist
        V_next, [Xerr, Xerr_int] = FeedbackControl(Tse, Xd, Xd_next, Kp, Ki, Xerr_int, dt)
        Je = CalculateJe(robot_config8, Tb0, M0e, Blist)
        u_thetad = np.dot(np.linalg.pinv(Je, rcond=1e-2), V_next)


        #print("Main() debug:")
        #print(f'\nrobot_config13: \n{robot_config13}')
        #print(f'\nrobot_config12: \n{robot_config13[:12]}')
        #print(f'\nu_thetad: \n{u_thetad}')


        #calculate new phi, x, y, joint angles, wheel angles after following twist
        robot_config12 = NextState(robot_config13[:12], u_thetad, dt, w_max)
        robot_config13[:12] = robot_config12
        robot_config13[-1] = gripper_next

        #add to arrays and move to next iteration of loop
        robot_config13_array[i+1] = robot_config13
        Xerr_array[i,:] = Xerr

    ###

    print("\nArrays generated.")
    print(f"\nRobot configuration, 13-vector: \nshape: {robot_config13_array.shape}")
    print(f"\nError twist array: \nshape: {Xerr_array.shape}")

    #print(robot_config13_array, end='\n\n\n')
    #print(Xerr_array)

    #save robot configuration array and error array to CSV file
    print('\nSaving results:')
    write_csv_mat('../csv/main_robotconfig13.csv', robot_config13_array)
    write_csv_mat('../csv/main_Xerr.csv', Xerr_array)
    print("Results written to /csv folder.")


def PlotErrTwist():
    pass


if __name__ == '__main__':
    main()