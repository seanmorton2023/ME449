
import core as mr
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from tqdm import tqdm
from scipy.spatial.transform import Rotation as R

from helpers import *
from geometry import *

from ProjectStep1 import NextState
from ProjectStep2 import TrajectoryGenerator
from ProjectStep3 import FeedbackControl, CalculateJe


def main(dt):

    #define control parameters

    #--------------------#
    ###best
    #Kp = np.identity(6) 
    #Ki = 0

    #init_base    = [np.pi/4, 0, 0]       #chassis phi, x, y
    #init_joints  = [0, -np.pi/4, 0, 0, np.pi/2] # 5 robot joints
    ##Tsc_i and Tsc_f determined in geometry.py

    #----------------------#
    ##overshoot
    #Kp = 5*np.identity(6) 
    #Ki = np.identity(6)

    #init_base    = [np.pi/4, 0, 0]       #chassis phi, x, y
    #init_joints  = [0, -np.pi/12, 0, 0, np.pi/4] # 5 robot joints
    ##Tsc_i and Tsc_f determined in geometry.py

    #----------------------#
    #newTask
    Kp =  np.identity(6)
    Ki = 0

    init_base    = [np.pi/4, 0, 0]       #chassis phi, x, y
    init_joints  = [0, -np.pi/12, 0, 0, np.pi/4] # 5 robot joints

    #Tse_0 same as other tasks
    Tsc_i =  mr.RpToTrans(np.identity(3), [1, 1, 0.025]).tolist()
    Tsc_f =  mr.RpToTrans(np.identity(3), [2, 1, 0.025]).tolist()

    #----------------------#
    ##joint limits testing
    #Kp = np.identity(6)
    #Ki = 0

    #init_base    = [np.pi/4, 0, 0]       #chassis phi, x, y
    #init_joints  = [0, -np.pi/12, 0, 0, np.pi/4] # 5 robot joints

    ###Tse_0 same as other tasks
    #Tsc_i =  mr.RpToTrans(np.identity(3), [ 0.5, 0.75, 0.025]).tolist()
    #Tsc_f =  mr.RpToTrans(np.identity(3), [-0.5, 0.75, 0.025]).tolist()

    #----------------------#

    #trajectory, error, and motion parameters
    k = 1
    w_max = 10
    rcond = 1E-3 #for Jacobian Pseudoinverse
    Xerr_int = np.zeros((6,1))
    Xerr = 0

    #Define initial robot posn as 13-vector: robot_config13
    init_wheels  = [0, 0, 0, 0]    # 4 wheel posns
    gripperState = 0               # open
    robot_config13 = init_base + init_joints + init_wheels + [gripperState]

    #desired EE initial config
    Tse_0 = np.array([
        [ 0, 0, 1,   0],
        [ 0, 1, 0,   0],
        [-1, 0, 0, 0.5],
        [ 0, 0, 0,   1]
    ])

    #- generate reference trajectory. some frames defined in geometry.py.
    ref_traj = TrajectoryGenerator(Tse_0, Tsc_i, Tsc_f, Tce_grasp, Tce_standoff, k)

    #iterate through trajectories generated
    N = len(ref_traj)

    #arrays for storage of results
    Xerr_array = np.zeros((N-1, 6)) 
    u_thetad = np.zeros(9)

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

        #apply control law; extract joint and wheel speeds from twist
        V_next, [Xerr, Xerr_int] = FeedbackControl(Tse, Xd, Xd_next, Kp, Ki, Xerr_int, dt)
        Je = CalculateJe(robot_config8, Tb0, M0e, Blist)
        u_thetad = np.dot(np.linalg.pinv(Je, rcond=rcond), V_next)
        u_thetad_clip = np.clip(u_thetad, -w_max, w_max)

        #calculate new phi, x, y, joint angles, wheel angles after following twist
        robot_config12 = NextState(robot_config13[:12], u_thetad_clip, dt, w_max)

        #----------------------------------#
        #extra - joint limits
        limit_reached, indices = testJointLimits(robot_config12)

        if (limit_reached):

            Je_inds = np.array(indices) + 4
            for j in Je_inds: Je[:,j] = 0

            u_thetad = np.dot(np.linalg.pinv(Je, rcond=rcond), V_next)
            u_thetad_clip = np.clip(u_thetad, -w_max, w_max)

            #calculate new phi, x, y, joint angles, wheel angles after following twist
            robot_config12 = NextState(robot_config13[:12], u_thetad_clip, dt, w_max)
            pass        

        #---------------------------------#

        #add to arrays and move to next iteration of loop
        robot_config13[:12] = robot_config12[:]
        robot_config13[-1] = gripper_next
        robot_config13_array[i+1] = robot_config13
        Xerr_array[i,:] = Xerr.T

    ###

    print("\nArrays generated.")
    print(f"\nRobot configuration, 13-vector: \nshape: {robot_config13_array.shape}")
    print(f"\nError twist array: \nshape: {Xerr_array.shape}")

    #save robot configuration array and error array to CSV file
    print('\nSaving results:')
    pd.DataFrame(robot_config13_array).to_csv('../csv/main/robotconfig13.csv', header=None, index=None)
    pd.DataFrame(Xerr_array          ).to_csv('../csv/main/Xerr.csv',          header=None, index=None)
    print("Results written to /csv folder.")
    

def testJointLimits(robot_config12):
    '''Takes in the current robot configuration, compares the joint angles
    to the max allowable joint angles for the robot, and returns:
    - limit_reached: logical T/F as to whether any joints violate limit
    - indices: indices of joints (0-4) of joints that violate their
        joint limit

    Note: joint_limits_kuka defined in geometry.py
    '''
    joint_vals = robot_config12[3:8].flatten()

    joints_beyond_limits = np.logical_or(
            joint_vals < joint_limits_kuka[0].flatten(),
            joint_vals > joint_limits_kuka[1].flatten()
    )

    limit_reached = np.any(joints_beyond_limits)
    indices = np.nonzero(joints_beyond_limits)[0].tolist()

    return limit_reached, indices

def PlotTrajectories(dt):
    '''Plotting code will be kept separate from the simulation code so that
    plotting can be done with past results of simulations.

    Arguments: none, takes in the data of interest by loading from
        a file
    '''

    Xerr_array           = pd.read_csv('../csv/main/Xerr.csv',         header=None).to_numpy()
    ang_array = Xerr_array.T[0:3]
    lin_array = Xerr_array.T[3:6]
    t_array = np.arange(0, dt * len(lin_array[0]), dt)
    labels = ['x','y','z']

    plt.figure(1)
    for i, comp in enumerate(ang_array):
        plt.plot(t_array, comp, label= 'w_' + labels[i])
    plt.xlabel('Time (s)')
    plt.ylabel('Magnitude of error (rad/s)')
    plt.legend()
    plt.title("Angular Error components from Xerr")

    plt.figure(2)
    for i, comp in enumerate(lin_array):
        plt.plot(t_array, comp, label= 'v_' + labels[i])
    plt.xlabel('Time (s)')
    plt.ylabel('Magnitude of error (m/s)')
    plt.legend()
    plt.title("Linear Error components from Xerr")

    plt.show()


if __name__ == '__main__':
    dt = 0.01
    main(dt)
    PlotTrajectories(dt)