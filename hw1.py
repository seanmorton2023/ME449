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

#define new matrices in terms of old ones
#R12 given
R23 = R12.T * R13
#R34 given
R45 = R34.T * R13.T * R15
R56 = R15.T * R12 * Rs2.T * Rs6

#source for formatting: https://tinyurl.com/2m6w7r8n
np.set_printoptions(formatter={'float_kind':'{:.4f}'.format})

R_array = [R12, R23, R34, R45, R56]
for i, R in enumerate(R_array):
    print(f"R{i+1}{i+2}:")
    print(R.round(4), end = "\n\n")


#turn all rotation matrices into joint angles
J12 = mr.so3ToVec(mr.MatrixLog3(R12.tolist()))
J23 = mr.so3ToVec(mr.MatrixLog3(R23.tolist()))
J34 = mr.so3ToVec(mr.MatrixLog3(R34.tolist()))
J45 = mr.so3ToVec(mr.MatrixLog3(R45.tolist()))
J56 = mr.so3ToVec(mr.MatrixLog3(R56.tolist()))

J_array = [J12, J23, J34, J45, J56]
for i, J in enumerate(J_array):
    print(f"J{i+1}{i+2}:")
    print(J, end = "\n\n")



