
import core as mr
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from tqdm import tqdm

from helpers import *
from geometry import *

from ProjectStep1 import NextState
from ProjectStep2 import TrajectoryGenerator
from ProjectStep3 import FeedbackControl, CalculateJe


def main(dt):

    #define control parameters
    #Kp = np.identity(6)
    #Kp = 20*np.identity(6) #3 was a little jittery
    Kp = np.identity(6) #3 was a little jittery

    #should consider different parameters for diff
    #entries in the array - diff. for w, x, y

    Ki = 0
    k = 1
    w_max = 10

    Xerr_int = 0
    Xerr = 0

    # - Define initial robot posn as 13-vector: robot_config13
    init_base    = [np.pi/4, 0, 0.5]       #chassis phi, x, y
    init_joints  = [0, 0, 0, 0, 0] # 5 robot joints
    init_wheels  = [0, 0, 0, 0]    # 4 wheel posns
    gripperState = 0               # open
    robot_config13 = init_base + init_joints + init_wheels + [gripperState]

    #once we introduce error in the initial posn, need to differentiate
    #between expected initial posn and actual intial posn
    expected_init_base    = [0, 0, 0]       #chassis phi, x, y
    expected_init_joints  = [0, 0, 0, 0, 0]

    #- generate reference trajectory from init. EE config to desired EE config
    #base offset Tb0 and home EE config M0e defined in geometry.py
    [phi, x, y] = expected_init_base
    Tsb_0  = ChassisSE3(phi, x, y).tolist()
    T0e = mr.FKinBody(M0e, Blist, expected_init_joints)
    Tse_0 = np.dot(np.dot(Tsb_0, Tb0), T0e)
    ref_traj = TrajectoryGenerator(Tse_0, Tsc_i, Tsc_f, Tce_grasp, Tce_standoff, k)

    #iterate through trajectories generated
    N = len(ref_traj)

    #arrays for storage of results
    Xerr_array = np.zeros((N-1, 6)) 
    u_thetad_array = np.zeros((N-1, 9))
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

        #print("Main() debug:")
        #print(f'\nXd: \n{Xd}')
        #print(f'\nXd_next: \n{Xd_next}')
        #print(f'\nTse: \n{Tse}')


        #apply control law; extract joint and wheel speeds from twist
        V_next, [Xerr, Xerr_int] = FeedbackControl(Tse, Xd, Xd_next, Kp, Ki, Xerr_int, dt)
        Je = CalculateJe(robot_config8, Tb0, M0e, Blist)

        #limit max speeds and accelerations of wheel velocities. speed limits
        #correspond to motor limits, accel limits correspond to limits of dynamics
        #u_thetad = np.dot(np.linalg.pinv(Je, rcond=1e-2), V_next)
        u_thetad_prev = u_thetad
        u_thetad = np.dot(np.linalg.pinv(Je, rcond=5e-3), V_next)
        u_thetad_clip = np.clip(u_thetad, -w_max, w_max)
        u_thetad_clip = np.clip(u_thetad, -0.1 * abs(u_thetad_prev), 0.1*abs(u_thetad_prev))
        #u_thetad = np.dot(np.linalg.pinv(Je, rcond=1e-1), V_next)


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
        u_thetad_array[i] = u_thetad_clip

    ###

    print("\nArrays generated.")
    print(f"\nRobot configuration, 13-vector: \nshape: {robot_config13_array.shape}")
    print(f"\nError twist array: \nshape: {Xerr_array.shape}")

    #print(robot_config13_array, end='\n\n\n')
    #print(Xerr_array)

    #save robot configuration array and error array to CSV file
    print('\nSaving results:')
    #write_csv_mat('../csv/main/robotconfig13.csv', robot_config13_array)
    #write_csv_mat('../csv/main/Xerr.csv', Xerr_array)
    #write_csv_mat('../csv/test/u_thetad.csv', u_thetad_array)
    pd.DataFrame(robot_config13_array).to_csv('../csv/main/robotconfig13.csv', header=None, index=None)
    pd.DataFrame(Xerr_array          ).to_csv('../csv/main/Xerr.csv',          header=None, index=None)
    pd.DataFrame(u_thetad_array      ).to_csv('../csv/test/u_thetad.csv',      header=None, index=None)

    print("Results written to /csv folder.")
    

def PlotTrajectories(dt):
    '''Plotting code will be kept separate from the simulation code so that
    plotting can be done with past results of simulations.

    Arguments: none, takes in the data of interest by loading from
        a file
    '''

    robot_config13_array = pd.read_csv('../csv/main/robotconfig13.csv',header=None).to_numpy()
    Xerr_array           = pd.read_csv('../csv/main/Xerr.csv',         header=None).to_numpy()
    u_thetad_array       = pd.read_csv('../csv/test/u_thetad.csv',     header=None).to_numpy()

    print("\nPlotting:")
    chassis_array = robot_config13_array.T[0:3]
    joints_array = robot_config13_array.T[3:8]
    wheels_array = robot_config13_array.T[8:12]

    legend1 = ['phi','x','y']
    legend2 = ['J1','J2','J3','J4','J5']
    legend3 = ['w1','w2','w3','w4']
    
    title1 = "Chassis Position Over Trajectory"
    title2 = "Joint Values Over Trajectory"
    title3 = "Wheel Angles Over Trajectory"

    ordered_pairs = [(chassis_array, legend1, title1),
                        (joints_array, legend2, title2),
                        (wheels_array, legend3, title3)]

    plt.close('all')

    #plot progression of robot trajectory
    for i, (array, legend, title) in enumerate(ordered_pairs):
        plt.figure(i+1)
        for j, statevar in enumerate(array):
            plt.plot(statevar, label = legend[j])

        plt.xlabel("Index")
        plt.ylabel("State variable value")
        plt.legend()
        plt.title(title)

    #plot magnitude of Xerr over time
    #angular_and_linear_error = [[np.linalg.norm(s[0:3]), np.linalg.norm(s[3:6])] \
    #                            for s in Xerr_array]
    #ang_lin_T = np.array(angular_and_linear_error).T
    #ang_array = ang_lin_T[0].tolist()
    #lin_array = ang_lin_T[1].tolist()

    ang_array = Xerr_array.T[0:3]
    lin_array = Xerr_array.T[3:6]
    t_array = np.arange(0, dt * len(lin_array[0]), dt)
    labels = ['x','y','z']


    plt.figure(4)
    for i, comp in enumerate(ang_array):
        plt.plot(t_array, comp, label= 'w_' + labels[i])
    plt.xlabel('Time (s)')
    plt.ylabel('Magnitude of error (rad/s)')
    plt.legend()
    plt.title("Angular Error components from Xerr")

    plt.figure(5)
    for i, comp in enumerate(lin_array):
        plt.plot(t_array, comp, label= 'v_' + labels[i])
    plt.xlabel('Time (s)')
    plt.ylabel('Magnitude of error (m/s)')
    plt.legend()
    plt.title("Linear Error components from Xerr")


    #---------------------------#
    #for debug only

    u_array = u_thetad_array[:,:4]
    thetad_array = u_thetad_array[:,4:]

    u_accel_array = np.array([u_array[i+1] - u_array[i] for i in range(len(u_array) - 1)])
    joint_accel_array = np.array([thetad_array[i+1] - thetad_array[i] for i in range(len(thetad_array) - 1)])

    
    plt.figure(6)
    for i, u in enumerate(u_array.T):
        plt.plot(u, label=f'u_{i}')

    plt.xlabel('Index')
    plt.ylabel('Magnitude of speed')
    plt.legend()
    plt.title("Wheel Speeds over Trajectory")

    plt.figure(7)
    for i, joint in enumerate(thetad_array.T):
        plt.plot(joint, label=f'Joint {i}')

    plt.xlabel('Index')
    plt.ylabel('Magnitude of speed')
    plt.legend()
    plt.title("Joint Speeds over Trajectory")

    ####

    plt.figure(8)
    for i, u in enumerate(u_accel_array.T):
        plt.plot(u, label=f'u_{i}')

    plt.xlabel('Index')
    plt.ylabel('Magnitude of accel')
    plt.legend()
    plt.title("Wheel Accels over Trajectory")

    plt.figure(9)
    for i, joint in enumerate(joint_accel_array.T):
        plt.plot(joint, label=f'Joint {i}')

    plt.xlabel('Index')
    plt.ylabel('Magnitude of accel')
    plt.legend()
    plt.title("Joint Accels over Trajectory")
   

    plt.show()


if __name__ == '__main__':
    dt = 0.01
    #main(dt)
    PlotTrajectories(dt)