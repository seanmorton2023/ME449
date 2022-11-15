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


def test_matrices():
    #generating standoff SE3
    SE3mat1 = generateStandoffSE3(cuboid_init_SE3_world, 2, [0,0,1])
    expected1 = mr.RpToTrans(np.identity(3), [1, 0, 2.025])
    assert np.allclose(SE3mat1, expected1), f"Received: \n{SE3mat1}\n"
    
    SE3mat2 = generateStandoffSE3(cuboid_init_SE3_world, 2, [0,1,0])
    expected2 = mr.RpToTrans(np.identity(3), [1, 2, 0.025])
    assert np.allclose(SE3mat2, expected2), f"Received: \n{SE3mat2}\n"

    print("All assertions passed")

    #testing datatypes of different matrices
    print("EE rotation matrix and final posn:")
    print(R_tip_to_obj)
    print(ee_to_object_SE3)


if __name__ == '__main__':

    #datatypes of imported variables
    #print(cuboid_init_SE3_world)
    #print(type(cuboid_init_SE3_world))

    Tf = 10
    N = 500
    method = 5
    eomg = 0.001
    ev = 0.0001
    thetalist0 = [0,0,0,0,0]

    #testing generation of Cartesian trajectories
    traj = mr.CartesianTrajectory(ee_init_SE3_world_mod, ee_at_obj_init_SE3, Tf,N,method)
    #traj = mr.CartesianTrajectory(M, armbase_to_ee_at_obj_SE3_mod, Tf,N,method)
    #for i, SE3mat in enumerate(traj):
    #    print(f"Row {i}: \n {SE3mat}")
    print("Generated trajectory.")
    
    t0 = time.time()

    joint_angles_array =  convertSE3toJointAngles(traj, Blist, M, thetalist0, eomg, ev)
    for i, angles in enumerate(joint_angles_array):
        print(f"Row {i}: \n {angles}")

    tf = time.time()
    print(f"Elapsed: {round(tf - t0, 1)} seconds")
    write_csv_mat('../csv/test_traj.csv', joint_angles_array)