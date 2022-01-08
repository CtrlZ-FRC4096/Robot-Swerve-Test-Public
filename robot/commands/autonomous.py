# This is to help vscode
from typing import TYPE_CHECKING

from wpilib._wpilib import SmartDashboard, wait
from wpilib.controller import PIDController, ProfiledPIDController
from wpimath.kinematics._kinematics import SwerveModuleState
from wpimath.trajectory import TrapezoidProfile
from wpimath.geometry._geometry import Pose2d, Rotation2d
from commands.drivetrain import Drive_Swerve
#from pathplannerlib.swerve_path_controller import SwervePathController
#from commands.drive_follow_path import DriveFollowPath

import pathplannerlib.pathplanner
if TYPE_CHECKING:
	from robot import Robot
from commands2 import CommandBase
from commands2._impl import CommandBase
from wpimath import geometry, controller
from wpimath.kinematics import ChassisSpeeds
from wpilib import Timer, controller
import const
import math


class DriveTrajectory(CommandBase):
	def __init__(self, robot: "Robot", path):
		super().__init__()
		self.robot 	= robot
		self.path 	= path
		self.target = pathplannerlib.pathplanner.PathPlanner.loadPath(self.path, const.MAX_VEL_METERS_AUTO, const.MAX_ACCEL_AUTO, False)
		self.timer = Timer()
		self.robot.drivetrain.swerve_module_front_left.set(0, math.radians(0))
		self.robot.drivetrain.swerve_module_front_right.set(0, math.radians(0))
		self.robot.drivetrain.swerve_module_back_left.set(0, math.radians(0))
		self.robot.drivetrain.swerve_module_back_right.set(0, math.radians(0))
		self.robot.drivetrain.odometry.resetPosition(self.target.getSample(0).pose, Rotation2d(0))
		self.hcontroller = betterHolonomicDriveController()
		self.timer.start()

	# Overrides
	def getRequirements(self):
		return set( [self.robot.drivetrain] )

	def execute(self):
		targetState = self.target.getSample(self.timer.get())
		currentPose = self.robot.drivetrain.getPose2d()
		self.hcontroller.drive(self.robot, currentPose, targetState)
		#print('Current pose:')
		#print(currentPose)
		###  speeds = self.hcontroller.calculate(currentPose, targetState, targetState.pose.rotation())
		#speeds = self.hcontroller.calculate(currentPose, targetPose, targetVel, targetState.pose.rotation())
		###  theta = self.testThetaController.calculate(currentPose.rotation().radians(), targetState.pose.rotation().radians())
		#theta = self.testThetaController.calculate(0, 0)
		###  print('current angle, target angle, difference:')
		###  print(currentPose.rotation().radians())
		###  print(targetState.pose.rotation().radians())
		###  print(targetState.pose.rotation().radians() - currentPose.rotation().radians())
		###  print('Controller calc:')
		###  print(speeds)
		###  print('testThetaController values:')
		###  print(theta)
		###  print('Position error:')
		###  print(self.testThetaController.getPositionError())
		###  print('Current pose:')
		###  print(currentPose)
		###  print('Target state:')
		###  print(targetState)
		#print('Target pose:')
		#print(targetPose)
		#print('Target vel:')
		#print(targetVel)
		###  print('Angle ref:')
		###  print(targetState.pose.rotation())
		"""calcs = self.hcontroller.calculate(currentPose, targetState)
		vx = calcs[0]
		vy = calcs[1]
		omega = calcs[2]
		Drive_Swerve(self.robot, vy, vx, 0)"""
		#self.robot.drivetrain.set_all_states(self.robot.drivetrain.get_kinematics().toSwerveModuleStates(speeds))


	def isFinished(self):
		return self.timer.hasElapsed(self.target.totalTime())

	def end(self, interrupted):
		self.timer.stop()
		self.robot.drivetrain.defense()
		pass

	def interrupted(self):
		self.end()

class betterHolonomicDriveController():
	def __init__(self):
		'''
		Creates drive controller
		'''
		self.x_controller = controller.PIDController(const.X_KP, const.X_KI, const.X_KD)
		self.y_controller = controller.PIDController(const.Y_KP, const.Y_KI, const.Y_KD)
		self.theta_controller = controller.PIDController(const.THETA_KP, const.THETA_KI, const.THETA_KD)
		self.x_controller.reset()
		self.y_controller.reset()
		self.theta_controller.reset()

	def calculate(self, currentPose, targetState):
		'''
		Calculates chassis speeds
		@parameter (Pose2d) currentPose		Robot's current pose
		@parameter (State) targetState		Robot's target state
		@returns (tuple) (vx, vy, omega)	Robot's left-right, forward-backward, and angular velocities (m/s, m/s, rad/s)
		'''
		# calculates the percent outputs in the x, y, and theta directions
		vx = self.x_controller.calculate(currentPose.X(), targetState.pose.X())
		vy = self.y_controller.calculate(currentPose.Y(), targetState.pose.Y())
		# current and target angles must be put into the range [-math.pi, math.pi)
		targetHeading = self.optimize(currentPose, targetState)
		omega = self.theta_controller.calculate(currentPose.rotation().radians(), targetHeading)
		#print('currentPose, targetPose, (vx, vy, omega):')
		#print(currentPose)
		#print(targetState.pose)
		#print('currentPose, targetPose, current heading, target heading, (vx, vy, omega):')
		#print(currentPose, targetState.pose, currentPose.rotation().radians(), targetHeading)
		#print((vx, vy, omega))
		return (vx, vy, omega)

	def optimize(self, currentPose, targetState):
		'''
		.enableContinuousInput() doesn't work so we have this
		@parameter (Pose2d) currentPose		Robot's current pose
		@parameter (State) targetState		Robot's target state
		@returns (double) targetHeading		optimized heading
		'''
		currentHeading = currentPose.rotation().radians()
		targetHeading = targetState.pose.rotation().radians()
		if targetHeading - currentHeading > math.pi:
			targetHeading -= (2 * math.pi)
			#print('optimized')
		elif targetHeading - currentHeading < -math.pi:
			targetHeading += (2 * math.pi)
			#print('optimized')
		return targetHeading

	def drive(self, robot, currentPose, targetState):
		'''
		Calculates chassis speeds and drives the robot
		'''
		calcs = self.calculate(currentPose, targetState)
		left_right = calcs[0]
		forward_back = calcs[1]
		omega = calcs[2]
		chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
		    vx = left_right,
		    vy = forward_back,
		    omega = omega,
		    robotAngle = currentPose.rotation(),
		)
		#print('ChassisSpeeds')
		#print(chassis_speeds)
		robot.drivetrain.drive(chassis_speeds)
		robot.drivetrain.periodic()
		#SmartDashboard.putData('X PID', self.x_controller)
		#SmartDashboard.putData('Y PID', self.y_controller)
		#moduleStates = robot.drivetrain.get_all_states()
		#robot.drivetrain.odometry.update(robot.drivetrain.get_rotation(), moduleStates[0], moduleStates[1], moduleStates[2], moduleStates[3])


