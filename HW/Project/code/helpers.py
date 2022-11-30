import numpy as np
import core as mr

#helper functions to be used in several project steps

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
    '''Takes the SE3 matrix representation of position, and turns it into
    a 13-by-1 array, for the sake of storing values in a CSV file.

    !!NOTE!!

    This is used to turn SE(3) matrices from TrajectoryGenerator()
    into a 2D array of positions. Do not confuse this with the 13-vector
    of the robot's configuration: chassis phi+x+y, joint angles, wheel angles, gripper state
    '''
    T = np.array(matrix)
    return np.append(np.append(T[:3,:3], T[:3,3]), gripperState).tolist()

def Array13toSE3Mat(array13):
    '''Takes a 13-by-1 array, and turns it into the SE3 matrix representation of position
     and a gripper state for the sake of trajectory loading.

    !!NOTE!!

    This is used to turn SE(3) matrices from TrajectoryGenerator()
    into a 2D array of positions. Do not confuse this with the 13-vector
    of the robot's configuration: chassis phi+x+y, joint angles, wheel angles, gripper state
    '''
    array13 = np.array(array13)
    if len(array13) != 13:
        raise Exception("Size incorrect: array should be 1x13 or 13x1")

    T = np.zeros((4,4))
    T[:3,:3] = array13[0:9].reshape(3,3)
    T[:3, 3] = array13[9:12]
    T[3,3] = 1
    gripperState = array13[-1]
    return T, gripperState

###

def ChassisSE3(phi, x, y):
    '''Calculates the chassis transformation matrix, in SE(3), based
    on the current angle, x, and y of the chassis.
    '''
    R_curr = np.array([
		[np.cos(phi), -np.sin(phi), 0],
		[np.sin(phi),  np.cos(phi), 0],
		[          0,           0,  1],
	])

    T_curr = mr.RpToTrans(R_curr, [x, y, 0.0963])
    return T_curr

###

if __name__ == '__main__':
    import core as mr
    SE3mat = mr.RpToTrans(2*np.identity(3),[3,4,5])
    print(SE3mat)
    array13 = SE3matToArray13(SE3mat, 1)

    print(array13)
    T, gripperState = Array13toSE3Mat(array13)
    print(T)


    
