Sean Morton
ME449 Final Project
Milestone 2
11/21/22
______________


Software:
The code used for Submission 2 is divided into 3 files:
- geometry.py: contains the Blist of the Kuka robot and transformation
	matrices used to run TrajectoryGenerator
- helpers.py: includes simpler functions used to format or process
	data in a couple lines of code
- ProjectStep2: contains
	- the TrajectoryGenerator function
	- a main() function to run to reproduce the csv file provided in the
		zip folder

Results:
- Code produces SE(3) matrices that correspond to a continuous trajectory
	from initial position to standoff 1, object initial position, back to 
	standoff 1, to standoff 2, to object final position, back to standoff 2.
	Gripper closes onto the object at initial position and releases it at
	final position.
- Demo MP4 file shows these results
- Code has not yet been designed to plan trajectory time scaling around
	the limits of motor joint velocities
- Starting position of robot is also slightly offset from the starting position
	given in CoppeliaSim
