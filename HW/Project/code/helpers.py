
import numpy as np
#import code.core as mr
import core as mr


#helper functions

def write_csv_line(csv_filename, data):
    #Appends a single line of data to a CSV file.
    with open(csv_filename, 'a') as f:
        data_str = ','.join([str(i) for i in data]) + '\n'
        f.write(data_str)
##

def write_csv_mat(csv_filename, mat):
    #Clears out existing data in a CSV file and writes a new matrix
    #of data to the file.

    f = open(csv_filename, 'w') #clear out old data
    f.close()

    #datatype handling
    mat = np.matrix(mat).tolist()
    for row in mat:
            write_csv_line(csv_filename, row)
###

def gripper_motion(thetalist, N, state):
    #return a matrix of all the joint angles we started with, only
    #with a 1 at the end for the gripper state
    return np.tile(   np.append(thetalist[:-1],state),   (N,1))

def SE3matToArray13(matrix, gripperState):
    #Takes the SE3 matrix representation of position, and turns it into
    #a 13-by-1 array, for the sake of storing values in a CSV file
    T = np.array(matrix)
    return np.append(np.append(T[:3,:3], T[:3,3]), gripperState).tolist()