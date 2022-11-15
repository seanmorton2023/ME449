#holds the M, G, and S/B parameters for the YouBot.
#import code.core as mr
import core as mr
import numpy as np
from scipy.spatial.transform import Rotation as R
from helpers import *

Slist = np.array([
    [0, 0, 1,      0, 0, 0],
    [0, 1, 0, -0.147, 0, 0],
    [0, 1, 0, -0.302, 0, 0],
    [0, 1, 0, -0.437, 0, 0],
    [0, 0, 1,      0, 0, 0],
]).T.tolist()

#Blist = np.array([
#    [0, 0, 1,     0, 0, 0],
#    [0, 1, 0, 0.548, 0, 0],
#    [0, 1, 0, 0.353, 0, 0],
#    [0, 1, 0, 0.218, 0, 0],
#    [0, 0, 1,      0, 0, 0],    
#]).T.tolist()

Blist = np.array([
    [0, 0, 1,     0, 0, 0],
    [0, -1, 0, -0.5076, 0, 0],
    [0, -1, 0, -0.3526, 0, 0],
    [0, -1, 0, -0.2176, 0, 0],
    [0, 0, 1,      0, 0, 0],    
]).T.tolist()

#parameters given by the problem
Tsb_0 = mr.RpToTrans(np.identity(3), [0.506, 0, 0.0963])
Tb0 = mr.RpToTrans(np.identity(3), [0.1662, 0, 0.0026] )
M0e = mr.RpToTrans(np.identity(3), [0.033, 0, 0.6546] )
Tsc_i =  mr.RpToTrans(np.identity(3), [1,  0, 0.025])
Tsc_f = np.array([
    [ 0, 1, 0,     0],
    [-1, 0, 0,    -1],
    [ 0, 0, 1, 0.025],
    [ 0, 0, 0,     1]
    ])

#standoff matrices in space frame
dist = 0.02 #2cm
Tsn_i = generateStandoffSE3(Tsc_i, dist, [0,0,1])
Tsn_f = generateStandoffSE3(Tsc_f, dist, [0,0,1])

#orientation of end effector frame rel. to. object frame is 135deg rotation about y
R_tip_to_obj = R.from_euler('y', 135, degrees = True)
Tce = mr.RpToTrans(R_tip_to_obj.as_matrix(), [0,0,0]) #orientation relative to object
Tne = mr.RpToTrans(R_tip_to_obj.as_matrix(), [0,0,0]) #orientation relative to standoff

#positions of the object in the frame of the armbase
T0s = mr.TransInv(np.dot(Tsb_0, Tb0))
T0c_i = np.dot(T0s, Tsc_i)
T0c_f = np.dot(T0s, Tsc_f)

#orientations of the end effector at object,
T0e_i = np.dot(T0c_i, Tce)
T0e_f = np.dot(T0c_f, Tce)

#...and at standoffs Ni, Nf
T0n_i = np.dot(T0s, Tsn_i)
T0e_ni = np.dot(T0n_i, Tne)
T0n_f = np.dot(T0s, Tsn_f)
T0e_nf = np.dot(T0n_f, Tne)





##geometry for the world that's specific to Scene 6 on;y
#cuboid_init_SE3_world      = mr.RpToTrans(np.identity(3), [1,  0, 0.025])
#goal_frame_SE3_world       = mr.RpToTrans(np.identity(3), [0, -1, 0.025])
#mobilebase_SE3_init_world  = mr.RpToTrans(np.identity(3), [8.8125E-2, 8.8125E-2, 0.18445])
#arm_base_SE3_init_world    = mr.RpToTrans(np.identity(3), [0.2602, 9.4E-2, .1929])
#tip_position_SE3_world     =  mr.RpToTrans(np.identity(3), [.19965, 1.0032E-3, .72962])

##geometry that's generalized for all configurations of robot
#R_tip_to_obj = R.from_euler('y', 135, degrees = True)
#ee_to_object_SE3 = mr.RpToTrans(R_tip_to_obj.as_matrix(), [0,0,0])
#ee_at_obj_init_SE3 =  np.asarray( \
#    np.matrix(cuboid_init_SE3_world) *  np.matrix(mr.TransInv(ee_to_object_SE3 )) )
#ee_at_obj_end_SE3 =  np.asarray( \
#    np.matrix(goal_frame_SE3_world) *  np.matrix(mr.TransInv(ee_to_object_SE3 )) )


##geometry for altered scene with the robot closer to the block
##arm_base_SE3_init_mod   = mr.RpToTrans(np.identity(3), [0.6722, 0, 9.89E-2])
#arm_base_SE3_init_mod   = mr.RpToTrans(np.identity(3), [0, 0, 9.6E-2])
#ee_init_SE3_world_mod = arm_base_SE3_init_mod.dot(M)
#armbase_to_objectinit_SE3_mod = np.asarray( \
#    np.matrix(mr.TransInv(arm_base_SE3_init_mod)) *  np.matrix(cuboid_init_SE3_world ) )
#armbase_to_ee_at_obj_SE3_mod = np.asarray( \
#    armbase_to_objectinit_SE3_mod.dot( mr.TransInv(ee_at_obj_init_SE3)  ))




if __name__ == '__main__':
    print("World to base SE3:")
    print(arm_base_SE3_init_mod)
    print("\nBase to EE SE3:")
    print(armbase_to_ee_at_obj_SE3_mod) #should have a positive x and negative z