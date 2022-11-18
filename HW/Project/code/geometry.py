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
Tsc_i =  mr.RpToTrans(np.identity(3), [     1, 0,  0.025]).tolist()
Tsc_f = np.array([
    [ 0, 1, 0,     0],
    [-1, 0, 0,    -1],
    [ 0, 0, 1, 0.025],
    [ 0, 0, 0,     1]
    ])


Tsb_0  = mr.RpToTrans(np.identity(3), [ 0.406, 0, 0.0963]).tolist()
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

#other parameters for starting TrajectoryGenerator
#choose starting Tse equal to the values found in CoppeliaSim for Scene 8
Tse_i = mr.RpToTrans(np.identity(3),[9.4E-2, 9.4E-2, 5.94E-1])