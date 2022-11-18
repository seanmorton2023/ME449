import numpy as np
import core as mr
from geometry import *
from helpers import *
import time

#functions to test: 
'''
write_csv_line(csv_filename, data)
write_csv_mat(csv_filename, mat)
joint_angles_array =  convertSE3toJointAngles(traj, Blist, M, thetalist0, eomg, ev)

SE3mat = generateStandoffSE3(Tsb, dist, unit_vec)
traj_angles = jointAnglesStartToEnd(Xstart, Xend, tF, n, method, 
                            Blist, M, thetalist0, eomg, ev)
'''

def test_csv():
    #CSV writing functions
    arr1 = np.array([0, 5, 6, 7])
    arr2 = np.array([9, 4, 5, 2])
    csv_filename = '../csv/test_data.csv'
    write_csv_line(csv_filename, arr1)
    write_csv_line(csv_filename, arr2)

    mat = np.matrix([
    [1, 2, 3, 4],
    [5, 6, 7, 8],
    [9, 10, 11, 12]
    ])

    write_csv_mat(csv_filename, mat)

def test_traj_creation():
    Tf = 10
    N = 1000
    method = 5
    eomg = 0.001
    ev = 0.0001
    thetalist0 = [0,0,0,0,0]

    #testing generation of Cartesian trajectories
    traj = mr.CartesianTrajectory(M0e, T0e_ni, Tf,N,method)
    write_csv_mat('../csv/test_traj2.csv', traj)


if __name__ == '__main__':

    #datatypes of imported variables
    print("Sample matrix and type:")
    print(Tsc_i)
    print(type(Tsc_i))

    test_csv()
    #test_traj_creation()

    testlist1 = [0,1,2,3,4,5,99,99,99]
    gripper_open = gripper_motion(testlist1,100, 1)
    gripper_closed = gripper_motion(testlist1, 10, 0)
    print(gripper_closed)
