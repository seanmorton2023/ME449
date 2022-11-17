#holds the M, G, and S/B parameters for the YouBot.
#import code.core as mr
import core as mr
import numpy as np
from scipy.spatial.transform import Rotation as R
from helpers import *

Blist = np.array([
    [0, 0, 1,     0, 0, 0],
    [0, -1, 0, -0.5076, 0, 0],
    [0, -1, 0, -0.3526, 0, 0],
    [0, -1, 0, -0.2176, 0, 0],
    [0, 0, 1,      0, 0, 0],    
]).T

#parameters given by the problem
Tsb_0  = mr.RpToTrans(np.identity(3), [ 0.406, 0, 0.0963]).tolist()
Tb0    = mr.RpToTrans(np.identity(3), [0.1662, 0, 0.0026]).tolist()
M0e    = mr.RpToTrans(np.identity(3), [ 0.033, 0, 0.6546]).tolist()
Tsc_i =  mr.RpToTrans(np.identity(3), [     1, 0,  0.025]).tolist()
Tsc_f = np.array([
    [ 0, 1, 0,     0],
    [-1, 0, 0,    -1],
    [ 0, 0, 1, 0.025],
    [ 0, 0, 0,     1]
    ])

#standoff matrices in space frame
dist = 0.02 #2cm
Tsn_i = generateStandoffSE3(Tsc_i, dist, [0,0,1]).tolist()
Tsn_f = generateStandoffSE3(Tsc_f, dist, [0,0,1]).tolist()

#orientation of end effector frame rel. to. object frame is 135deg rotation about y
R_tip_to_obj = R.from_euler('y', 135, degrees = True)
Tce = mr.RpToTrans(R_tip_to_obj.as_matrix(), [0,0,0]) #orientation relative to object
Tne = mr.RpToTrans(R_tip_to_obj.as_matrix(), [0,0,0]) #orientation relative to standoff

#positions of the object in the frame of the armbase
T0s = mr.TransInv(np.dot(Tsb_0, Tb0)).tolist()
T0c_i = np.dot(T0s, Tsc_i).tolist()
T0c_f = np.dot(T0s, Tsc_f).tolist()

#orientations of the end effector at object,
T0e_i = np.dot(T0c_i, Tce).tolist()
T0e_f = np.dot(T0c_f, Tce).tolist()

#...and at standoffs Ni, Nf
T0n_i = np.dot(T0s, Tsn_i).tolist()
T0e_ni = np.dot(T0n_i, Tne).tolist()
T0n_f = np.dot(T0s, Tsn_f).tolist()
T0e_nf = np.dot(T0n_f, Tne).tolist()

if __name__ == '__main__':
    print("Make sure these match up with what you see in CoppeliaSim!")
    print("\nCurrent SE3 (world to end effector):")
    print(np.dot(Tsb_0, np.dot(Tb0, M0e)))
    print("\nCurrent SE3 (world to YouBot base):")
    print(Tsb_0)

    print("\nPosition of the end effector in frame 0:")
    print(M0e)
    print("\nPosition of the object in frame 0:")
    print(T0c_i)
