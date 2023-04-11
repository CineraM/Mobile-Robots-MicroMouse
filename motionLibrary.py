from controller import Robot
import math
from statistics import mean

wheel_radius = 1.6/2
axel_length = 2.28

class robotPose:
	def __init__(self,x,y,theta,previous_left_encoder = 0,previous_right_encoder=0):
		self.x = x
		self.y = y
		self.theta = theta
		self.previous_left_encoder = previous_left_encoder
		self.previous_right_encoder = previous_right_encoder
	def update_pose(self,left_encoder,right_encoder,heading):
		distance_traveled = mean([self.previous_right_encoder-right_encoder,self.previous_left_encoder-left_encoder])*.8
		self.x = self.x + (abs(distance_traveled)*math.cos(math.radians(heading)))
		self.y = self.y + (abs(distance_traveled)*math.sin(math.radians(heading)))
		self.theta = math.radians(heading)
		self.previous_left_encoder = left_encoder
		self.previous_right_encoder = right_encoder
	def print_pose(self):
		print('#####################################')
		print('X: \t', self.x)
		print('Y: \t',self.y)
		print('Theta: \t',self.theta)


#######################################################
# Cleans the IMU readings so that the are in degrees 
# and in the range of [0,359]
#######################################################
def imu_cleaner(imu_reading):
    rad_out = imu_reading
    if rad_out < 0:
        rad_out = rad_out + 2*math.pi
    return math.degrees(rad_out)

#######################################################
# Calculated the saturation speed
#######################################################
def forward_saturation(v):
    max_v = 4
    if v >= max_v :
        v = max_v
    elif v <= -max_v:
        v = -max_v
    else:
        v=v
    return v


#######################################################
# Calculated the forward speed using PID
#######################################################
def forward_PID(robot, D_maintain = 2.5, K_p = 5):
    # Using distance Sensors
    #fd = frontDistanceSensor.getValue()*39.37
    # Using Lidar
    fd = min(robot.lidar.getRangeImage()[0:5] + robot.lidar.getRangeImage()[354:359])*39.37
    error = fd - D_maintain
    return forward_saturation(K_p*error)

#######################################################
# Calculated the motor speeds using PID for side wall
# Distances
#######################################################
def wall_follow_PID(robot, D_min = 4, K_p = 2, wall = 'R'):
    V_f = forward_PID(robot,K_p=5)
    # rightDistanceSensor = robot.getDevice('right distance sensor')
    # rd = rightDistanceSensor.getValue()*39.37
    rd = min(robot.lidar.getRangeImage()[1:91])*39.37
    # leftDistanceSensor = robot.getDevice('left distance sensor')
    # ld = leftDistanceSensor.getValue()*39.37    
    ld = min(robot.lidar.getRangeImage()[270:359])*39.37
    # Right wall follow

    if wall == 'R':
        # Calculate the error
        error = (D_min - rd)
        # Too Close
        if rd < D_min:
            # print('Too Close:', error)
            V_r = forward_saturation(V_f)
            V_l = forward_saturation(V_f - abs((K_p*error))) 
        # Too Far
        elif rd > D_min:
            # print('Too Far: ', error) 
            V_r = forward_saturation(V_f- abs((K_p*error)))
            V_l = forward_saturation(V_f)
        # Too close to other wall
        elif ld < 3:
            error = (3 - ld) 
            V_r = forward_saturation(V_f- abs((K_p*error)))
            V_l = forward_saturation(0)

        # Goldy locks
        else :
            V_l = V_f 
            V_r = V_f
    
    # Left wall follow
    else:
        # Calculate the error
        error = (D_min - ld)
        # To Close
        if ld < D_min:
            # print('Too Close:', error)
            V_r = forward_saturation(V_f- abs((K_p*error)))
            V_l = forward_saturation(V_f)
        # Too Far
        elif ld > D_min:
            # print('Too Far: ', error)
            error = (D_min - ld)
            V_r = forward_saturation(V_f)
            V_l = forward_saturation(V_f - abs((K_p*error)))
        
        # Too close to other wall
        elif rd < 3:
            error = (3 - rd) 
            V_r = forward_saturation(0)
            V_l = forward_saturation(V_f- abs((K_p*error)))
        else :
            V_l = V_f 
            V_r = V_f

    
    # Return the left and right velocities to apply to the motor
    return V_l, V_r

'''
Class:    Chances_Robot
	ATRB:	The following are owned by an instance of Chances_Robot
		robot:	creates a webots controller that is used to get the simulation 
				timestep and anvance the simulation
		
		timestep:	creates an instance of the simulations timestep value
		
		frontDistanceSensor: sensor to get the distance on the front
		
		leftDistanceSensor: sensor to get the distance on the left

		rightDistanceSensor: sensor to get the distance on the right

		rearDistanceSensor: sensor to get the distance on the rear

		lidar:	A range sensor with 360 readings evenly distributed around the
				robot at 1-degree intervals

		imu: 	Inertial Unit for getting the robots orientation

		camera_front:	Camera fancing the front of the robot

		camera_right:	Camera fancing the front of the robot

		camera_rear:	Camera fancing the front of the robot

		camera_left:	Camera fancing the front of the robot

		leftMotor:		Controller for the left motor

		rightMotor:		Controller for the right motor

		left_wheel_encoder:		Controller for the left position sensor

		right_wheel_encoder:	Controller for the right position sensor

'''
class HamBot:
	def __init__(self):
		#######################################################
		# Creates Robot
		#######################################################
		self.robot = Robot()

		#######################################################
		# Sets the time step of the current world
		#######################################################
		self.timestep = int(self.robot.getBasicTimeStep())

		#######################################################
		# Gets Robots Distance Sensors
		# Documentation:
		#  https://cyberbotics.com/doc/reference/distancesensor
		#######################################################
		self.frontDistanceSensor = self.robot.getDevice('front distance sensor')
		self.leftDistanceSensor = self.robot.getDevice('left distance sensor')
		self.rightDistanceSensor = self.robot.getDevice('right distance sensor')
		self.rearDistanceSensor = self.robot.getDevice('rear distance sensor')
		self.frontDistanceSensor.enable(self.timestep)
		self.leftDistanceSensor.enable(self.timestep)
		self.rightDistanceSensor.enable(self.timestep)
		self.rearDistanceSensor.enable(self.timestep)

		#######################################################
		# Gets Robots Lidar Distance Sensors
		# Documentation:
		#  https://cyberbotics.com/doc/reference/lidar
		#######################################################
		self.lidar = self.robot.getDevice('lidar')
		self.lidar.enable(self.timestep)
		
		#######################################################
		# Gets Robots Camera
		# Documentation:
		#  https://cyberbotics.com/doc/reference/camera
		#######################################################
		self.camera_front = self.robot.getDevice('cameraFront')
		self.camera_front.enable(self.timestep)
		self.camera_front.recognitionEnable(self.timestep)

		self.camera_right = self.robot.getDevice('cameraRight')
		self.camera_right.enable(self.timestep)
		self.camera_right.recognitionEnable(self.timestep)

		self.camera_rear = self.robot.getDevice('cameraRear')
		self.camera_rear.enable(self.timestep)
		self.camera_rear.recognitionEnable(self.timestep)

		self.camera_left = self.robot.getDevice('cameraLeft')
		self.camera_left.enable(self.timestep)
		self.camera_left.recognitionEnable(self.timestep)

		#######################################################
		# Gets Robots Motors
		# Documentation:
		#  https://cyberbotics.com/doc/reference/motor
		#######################################################
		self.leftMotor = self.robot.getDevice('left wheel motor')
		self.rightMotor = self.robot.getDevice('right wheel motor')
		self.leftMotor.setPosition(float('inf'))
		self.rightMotor.setPosition(float('inf'))
		self.leftMotor.setVelocity(0)
		self.rightMotor.setVelocity(0)

		#######################################################
		# Gets Robot's the position sensors
		# Documentation:
		#  https://cyberbotics.com/doc/reference/positionsensor
		#######################################################
		self.left_wheel_encoder = self.robot.getDevice('left wheel sensor')
		self.right_wheel_encoder = self.robot.getDevice('right wheel sensor')
		self.left_wheel_encoder.enable(self.timestep)
		self.right_wheel_encoder.enable(self.timestep)

		#######################################################
		# Gets Robot's IMU sensors
		# Documentation:
		#  https://cyberbotics.com/doc/reference/inertialunit
		#######################################################
		self.imu = self.robot.getDevice('inertial unit')
		self.imu.enable(self.timestep)

		#######################################################
		# Sets initial robot's pose estimation 
		#######################################################
		self.x = 0
		self.y = 0
		self.theta = math.radians(imu_cleaner(self.imu.getRollPitchYaw()[2]))
		self.previous_left_encoder = 0
		self.previous_right_encoder = 0



	#######################################################
	# Sets inital robot's pose estimation
	#######################################################
	def set_pose(self,x,y):
		self.x = x
		self.y = y
		self.theta = math.radians(imu_cleaner(self.imu.getRollPitchYaw()[2]))
		self.previous_left_encoder = 0
		self.previous_right_encoder = 0

	#######################################################
	# Updates robot's pose estimation, should be called
	# every timestep for best results
	#######################################################
	def update_pose(self):
		distance_traveled = mean([self.previous_right_encoder-self.right_wheel_encoder.getValue(),self.previous_left_encoder-self.left_wheel_encoder.getValue()])*.8
		self.x = self.x + (abs(distance_traveled)*math.cos(math.radians(imu_cleaner(self.imu.getRollPitchYaw()[2]))))
		self.y = self.y + (abs(distance_traveled)*math.sin(math.radians(imu_cleaner(self.imu.getRollPitchYaw()[2]))))
		self.theta = math.radians(imu_cleaner(self.imu.getRollPitchYaw()[2]))
		self.previous_left_encoder = self.left_wheel_encoder.getValue()
		self.previous_right_encoder = self.right_wheel_encoder.getValue()
	
	#######################################################
	# Prints the robot's estimated pose
	#######################################################
	def print_pose(self):
		print('#####################################')
		print('X: \t', self.x)
		print('Y: \t',self.y)
		print('Theta: \t',self.theta)

	#######################################################
	# General driving forward a distance of D function.
	# will update the pose of the robot based on D, and the
	# current heading of the robot.
	#######################################################            
	def driveD(self,D,V=5):
	    
		start_position = self.left_wheel_encoder.getValue()

		# Calculates velocity of each motor and the robot
		phi = V / wheel_radius                # rad/sec

		# Calculates Time need to move a distance D
		T   = D/V               # sec

		self.leftMotor.setVelocity(phi)
		self.rightMotor.setVelocity(phi)

		while self.robot.step(self.timestep) != -1:
			# Checks if wheel distance is larger than D
			if wheel_radius*abs(self.left_wheel_encoder.getValue() - start_position) >= D-0.01:
				self.leftMotor.setVelocity(0)
				self.rightMotor.setVelocity(0)
				break
			self.update_pose()
			self.print_pose()
	   
	#######################################################
	# General circular motion function that drives the robot.
	# around a circle with radius R
	# R < 0 counter clockwise
	# R > 0 clockwise
	#######################################################            
	def circleR(self,R,V=4,direction='right',percent = 1):
	    
		# Determines direction and sets proper speeds
		omega = V/abs(R)
		if R < 0 :
			sign = -1
		else:
			sign = 1

		# Right and Clockwise
		if direction == 'right' and sign > 0:
			vl = omega*(abs(R) + sign*(axel_length/2))
			vr = omega*(abs(R) - sign*(axel_length/2))
			D = 2*math.pi*(abs(R) + sign*(axel_length/2))
		# Right and Counter Clockwise
		elif direction == 'right' and sign < 0:
			vl = -omega*(abs(R) - sign*(axel_length/2))
			vr = -omega*(abs(R) + sign*(axel_length/2))
			D = 2*math.pi*(abs(R) - sign*(axel_length/2))
		# Left and Clockwise
		elif direction == 'left' and sign > 0:
			vl = -omega*(abs(R) - sign*(axel_length/2))
			vr = -omega*(abs(R) + sign*(axel_length/2))
			D = 2*math.pi*(abs(R) - sign*(axel_length/2))
		# Left and Counter Clockwise
		elif direction == 'left' and sign < 0:
			vl = omega*(abs(R) + sign*(axel_length/2))
			vr = omega*(abs(R) - sign*(axel_length/2))
			D = 2*math.pi*(abs(R) + sign*(axel_length/2))

		D = D*percent
		phi_l = vl/wheel_radius
		phi_r = vr/wheel_radius

		# Checks to see if speed is to high 
		if abs(phi_l) > self.leftMotor.getMaxVelocity() or abs(phi_r) > self.rightMotor.getMaxVelocity():
			print("Speed is too great for robot")
			return

		# Gets starting postion of left wheel encoder to use as a stoping condition
		start_position = self.left_wheel_encoder.getValue()

		# Sets motor speeds and sets start time
		self.leftMotor.setVelocity(phi_l)
		self.rightMotor.setVelocity(phi_r)

		while self.robot.step(self.timestep) != -1:
		# Checks if wheel distance is larger than D
			if wheel_radius*abs(self.left_wheel_encoder.getValue() - start_position) >= D-0.02:
				self.leftMotor.setVelocity(0)
				self.rightMotor.setVelocity(0)
				break
			self.update_pose()
			self.print_pose()
			

	#######################################################
	# General function to rotate the robot by degree
	#######################################################       
	def rotate(self,degree):
	    
	    # Determines Rotation and sets proper speeds
		if degree < 0 :
			sign = -1
		else:
			sign = 1
		X_rad = math.radians(degree)
		phi = sign*2

		# Calculates time need for rotation
		omega = 2*abs(phi)*wheel_radius / axel_length
		T = abs(X_rad / omega)
		# end_heading = (predicted_pose[3] - degree)%360

		t_start = self.robot.getTime()
		self.leftMotor.setVelocity(phi)
		self.rightMotor.setVelocity(-phi)

		starting_theta = round(imu_cleaner(self.imu.getRollPitchYaw()[2]))
		end_heading = round((starting_theta - degree)%360,2)

		marg_error = .01

		while self.robot.step(self.timestep) != -1:
			current_heading = imu_cleaner(self.imu.getRollPitchYaw()[2])
			east_flag = True if end_heading <= 4 or end_heading >= 356 else False
			if (self.robot.getTime() - t_start) >= T:

				if east_flag:
					current_heading = current_heading - 360 if current_heading > 355 else current_heading
				if current_heading > (end_heading+marg_error):
					self.leftMotor.setVelocity(.01)
					self.rightMotor.setVelocity(-.01)
				elif current_heading < (end_heading-marg_error):
					self.leftMotor.setVelocity(-.01)
					self.rightMotor.setVelocity(.01)
				else:
					self.leftMotor.setVelocity(0)
					self.rightMotor.setVelocity(0)
					break

			self.update_pose()
			self.print_pose()
