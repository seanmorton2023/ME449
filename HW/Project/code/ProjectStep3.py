
def Step3Draft(robot_config12, robot_speeds9, dt, w_max, 
			  Blist, M0e, Tb0):
	'''	Calculates the next configuration of the robot based on the current state.

	Input:
	- robot_config12: A 12-vector representing the current configuration
			of the robot (3 variables for the chassis configuration, 
			5 variables for the arm configuration, and 4 variables 
			for the wheel angles).
	- robot_speeds9: A 9-vector of controls indicating the wheel speeds u 
			(4 variables) and the arm joint speeds \dot{\theta} (5 variables).
	- dt: A timestep Δt.
	- w_max: A positive real value indicating the maximum angular 
			speed of the arm joints and the wheels. For example, 
			if this value is 12.3, the angular speed of the wheels 
			and arm joints is limited to the range [-12.3 radians/s,
				12.3 radians/s]. Any speed in the 9-vector of controls
				that is outside this range will be set to the nearest 
				boundary of the range.
	- Blist: list of screw axes expressed in the end effector frame
	- M0e: SE(3) matrix; home configuration of the end effector relative to
			frame 0 at base of arm on chassis
	- Tb0: SE(3) matrix; constant offset of base of robot arm on chassis from
			the chassis frame

	Output: A 12-vector representing the configuration of the 
		robot time Δt later.

	'''
	#variables to define: wheel geometry
	l = 0.47/2.0 #m
	w = 0.3/2.0  #m
	r = 0.0475   #m
	#Tb0 = mr.RpToTrans(np.identity(3), [0.1662, 0, 0.0026])

	#checking of data
	robot_config12 = np.array(robot_config12)
	robot_speeds9 = np.array(robot_speeds9)

	if len(robot_config12) != 12 or len(robot_speeds9) != 9:
		raise Exception(f"Lengths of arrays: {len(robot_config_12)} {len(robot_speeds9)}")

	#world coords, joint angles theta, wheel angles phi
	q_array      = robot_config12[0:3]
	theta_array  = robot_config12[3:8]
	phi_array    = robot_config12[8:12]

	u_array      = robot_speeds9[0:4]
	thetad_array = robot_speeds9[4:9]

	#mapping from wheel velocities to velocity in world coords
	c = r/(l+w) #constant for mapping from wheel speeds to omega in world coords
	F = np.array([
		[-c, c,  c, -c],
		[ 1, 1,  1,  1],
		[-1, 1, -1,  1]	
	])

	F6 = np.zeros((6,4))
	F6[2:5, :] = F

	#chassis twist Vb
	Vb  = np.dot( F, u_array)
	Vb6 = np.dot(F6, u_array)

	#determine Tsb(q) from omnidirectional robot control
	phi = q_array[0]
	x   = q_array[1]
	y   = q_array[2]  

	Rsb = np.array([
		[np.cos(phi), -np.sin(phi), 0]
		[np.sin(phi),  np.cos(phi), 0]
		[          0,           0,  1]
	])
	
	Tsb = mr.RpToTrans(Rsb, [x, y, 0.0963])

	#determine Jacobian matrices of both arm and mobile base
	Jarm = mr.JacobianBody(Blist, theta_array)
	T0e = mr.FKinBody(M0e, Blist, theta_array)
	Teb = mr.TransInv(np.dot(Tb0, T0e))
	Jbase = np.dot(mr.Adjoint(Teb), F6)

	#J_base is 6x4: 4 wheels
	#J_arm  is 6x5: 5 joints
	Je = np.zeros((6, 9))
	Je[:, 0:4] = Jbase
	Je[:, 4:9] = Jarm

	#construct end effector twist Ve
	u_thetad = robot_speeds9.reshape((9,1))
	Ve = np.dot(Je, u_thetad)
