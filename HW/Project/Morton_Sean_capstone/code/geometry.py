import core as mr
import numpy as np
from scipy.spatial.transform import Rotation as R

#This file holds important parameters of the YouBot and of the default path-planning scene
#that were used in several steps of the project.

Blist = np.array([
    [0,  0, 1,       0, 0.033, 0],
    [0, -1, 0, -0.5076,     0, 0],
    [0, -1, 0, -0.3526,     0, 0],
    [0, -1, 0, -0.2176,     0, 0],
    [0,  0, 1,       0,     0, 0],    
]).T

#parameters given by the problem
Tsc_i =  mr.RpToTrans(np.identity(3), [     1, 0,  0.025]).tolist()
Tsc_f = np.array([
    [ 0, 1, 0,     0],
    [-1, 0, 0,    -1],
    [ 0, 0, 1, 0.025],
    [ 0, 0, 0,     1]
    ])

joint_limits_kuka = (2*np.pi / 360) * np.array([
    [  -169,   169],
    [   -65,    90],
    [  -151,   146],
    [-102.5, 102.5],
    [-167.5, 167.5]    
]).T

Tb0    = mr.RpToTrans(np.identity(3), [0.1662, 0, 0.0026]).tolist()
M0e    = mr.RpToTrans(np.identity(3), [ 0.033, 0, 0.6546]).tolist()

#orientation of end effector frame rel. to. object frame is 135deg rotation about y
dist = 0.08 #in m, i.e. dist*100 cm
R_ee_to_obj = R.from_euler('y', 135, degrees = True).as_matrix()
Tce_grasp = mr.RpToTrans(R_ee_to_obj, [0,0,0]) #orientation relative to object
Tce_standoff = np.dot(
    mr.RpToTrans(np.identity(3),dist * np.array([0,0,1])), \
    mr.RpToTrans(R_ee_to_obj, [0,0,0]) \
).tolist() #orientation relative to object at standoff

