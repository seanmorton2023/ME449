import core as mr
import numpy as np

#define existing rotation matrices
R13 = np.matrix([[-0.7071, 0, -0.7071], [0, 1, 0], [0.7071, 0, -0.7071]])
Rs2 = np.matrix([[-0.6964, 0.1736, 0.6964], [-0.1228, -0.9848, 0.1228], [0.7071, 0, 0.7071]])
R15 = np.matrix([[-0.9839, -0.1558, 0.0872], [-0.1564, 0.9877, 0], [-0.0861, -0.0136, -0.9962]])
R12 = np.matrix([[0.7071, 0, -0.7071], [0, 1, 0], [0.7071, 0, 0.7071]])
R34 = np.matrix([[0.6428, 0, -0.7660], [0, 1, 0], [0.7660, 0, 0.6428]])
Rs6 = np.matrix([[-0.1676, 0.3250, -0.9308], [-0.0434, -0.9456, -0.3224], [-0.9849, -0.0136, 0.1726]])
R6b = np.matrix([[-1, 0, 0], [0, 0, 1], [0, 1, 0]])

#define new R matrices in terms of old ones
Rs1 = Rs2 * R12.T
#R12 given
R23 = R12.T * R13
#R34 given
R45 = R34.T * R13.T * R15
R56 = R15.T * R12 * Rs2.T * Rs6
#R6b given

#rotation matrix Rsb
Rsb = Rs6 * R6b

#source for formatting: https://tinyurl.com/2m6w7r8n
np.set_printoptions(formatter={'float_kind':'{:.4f}'.format})

R_array = [Rs1, R12, R23, R34, R45, R56] #include space rotation from s to 1?
#R_array = [R12, R23, R34, R45, R56, R6b]

for i, R in enumerate(R_array):
    #if i == 5:
    #    print("R6b:")
    #else:
    #    print(f"R{i+1}{i+2}:")
    if i == 0:
        print("Rs1:")
    else:
        print(f"R{i}{i+1}:")
    print(R.round(4), end = "\n\n")


#take matrix log of R, then turn so(3) matrix into 3x1 vector
J_vec_array = [mr.so3ToVec(mr.MatrixLog3(R.tolist())) for R in R_array]

#find magnitude of rotation. source for linalg norm: https://tinyurl.com/39e29xr6
J_angle_array = [np.sqrt(x.dot(x)) for x in J_vec_array]

for i, J in enumerate(J_angle_array):
    #if i == 5:
    #    print("Angle J6b:")
    #else:
    #    print(f"Angle J{i+1}{i+2}:")
    if i == 0:
        print("Angle Js1:")
    else:
        print(f"Angle J{i}{i+1}:")
    print(J.round(4))
    print(f"{round(J*360/2/3.14159, 2)} degrees",end = "\n\n")

#coppeliasim takes in 6 angles in radians, one for each joint
print("Joint angle vector (C-x C-v into CoppeliaSim:)")
print(J_angle_array, end = '\n\n')


####


print(f"Rotation matrix Rsb:")
print(Rsb, end = '\n\n')