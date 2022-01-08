"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2022
Code for robot "swerve drivetrain prototype"
contact@team4096.org

Some code adapted from:
https://github.com/SwerveDriveSpecialties

Some code adapted from:
https://github.com/SwerveDriveSpecialties
"""

"""
Prepend these to any port IDs.
DIO = Digital I/O
AIN = Analog Input
PWM = Pulse Width Modulation
CAN = Controller Area Network
PCM = Pneumatic Control Module
PDP = Power Distribution Panel
"""

from commands2._impl import TrapezoidProfileCommand
from wpilib import controller
from wpimath import trajectory
import math
from wpimath.trajectory._trajectory import TrajectoryConfig

### CONSTANTS ###

# Is running simulator. Value is set in robot.py, robotInit
IS_SIMULATION		= False

# Directions, mainly used for swerve module positions on drivetrain
FRONT_LEFT			= 'front_left'
FRONT_RIGHT			= 'front_right'
BACK_LEFT			= 'back_left'
BACK_RIGHT			= 'back_right'

# Used in ctre configurations
CLOCKWISE			= True
COUNTER_CLOCKWISE	= False

# Field centric directions/axes
FIELD_FRONT_BACK	= 'field_front_back'
FIELD_LEFT_RIGHT	= 'field_left_right'
FIELD_ROTATION		= 'field_rotation'

# Robot drivebase dimensions, in inches and meters
DRIVE_BASE_WIDTH	= 23.5
DRIVE_BASE_LENGTH	= 23.5
DRIVETRAIN_TRACKWIDTH_METERS	= 0.5969
DRIVETRAIN_WHEELBASE_METERS		= 0.5969

# Robot speeds
MAX_VEL_METERS 		= 6380 / 60 * (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0) * 0.10033 * math.pi
MAX_ANG_VEL_RAD    	= MAX_VEL_METERS / math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0)
MAX_ANG_ACCEL      	= 6 * math.pi #This isn't used anywhere? Also just a WAG, so probably not accurate

# Input thresholds
LOWER_INPUT_THRESHOLD	= 0.1
INPUT_MULTIPLIER		= 0.65

# Swerve Module 1 (Front Left)

DRIVE_MOTOR_1_ID	= 7
STEER_MOTOR_1_ID	= 6
STEER_ENCODER_1_ID	= 8
STEER_ENCODER_1_OFFSET = 360-317.4

# Swerve Module 2 (Front Right)

DRIVE_MOTOR_2_ID	= 4
STEER_MOTOR_2_ID	= 3
STEER_ENCODER_2_ID	= 5
STEER_ENCODER_2_OFFSET = 360-67.5

# Swerve Module 3 (Back Left)

DRIVE_MOTOR_3_ID	= 10
STEER_MOTOR_3_ID	= 9
STEER_ENCODER_3_ID	= 11
STEER_ENCODER_3_OFFSET = 360-243.0

# Swerve Module 4 (Back Right)

DRIVE_MOTOR_4_ID	= 1
STEER_MOTOR_4_ID	= 0
STEER_ENCODER_4_ID	= 2
STEER_ENCODER_4_OFFSET = 360-306.9




# Other

CAN_GYRO_TALON_ID   = 12
CAN_PDP             = 13

# Auto
AUTO_RESOLUTION         = .01 #path resolution in seconds
MAX_VEL_METERS_AUTO     = 1 #This is the max velocity you want the robot to drive at, not its true max velocity
MAX_ANG_VEL_RAD_AUTO    = MAX_VEL_METERS_AUTO / math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0) #This is the max velocity you want the robot to rotate at, not its true max rotational velocity
MAX_ACCEL_AUTO			= 4 #This is the max rate you want the robot to accelerate at, not its true max acceleration
MAX_ANG_ACCEL_AUTO      = 6 * math.pi #This is the max rate you want the robot to accelerate at, not its true max acceleration
X_KP                    = 0.12667925	#0.73225
X_KI                    = 0.0346173		#0.2001
X_KD                    = 0.01165449	#0.067367
Y_KP                    = X_KP
Y_KI                    = X_KI
Y_KD                    = X_KD
THETA_KP                = .232
THETA_KI                = 0.0625
THETA_KD                = 0


# Drive reduction * wheel diameter * pi * motor encoder ticks per 100ms * 10 (to get the encoder units to ticks per second)
# THIS IS THE CALCULATED VALUE, THIS VALUE SHOULD BE MEASURED AND CALIBRATED
METERS_PER_DRIVE_TICK   = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0) * 0.10033 * math.pi / 2048 * 10

