"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2022
Code for robot "swerve drivetrain prototype"
contact@team4096.org

Some code adapted from:
https://github.com/SwerveDriveSpecialties
"""

# This is to help vscode
import math
from typing import TYPE_CHECKING

from wpimath.geometry._geometry import Pose2d
if TYPE_CHECKING:
	from robot import Robot

# from wpilib.command import Command, CommandGroup, WaitCommand
import commands2
from commands2 import Command, CommandBase

import wpimath.kinematics
from wpimath.geometry import Translation2d, Rotation2d


class Drive_Swerve(Command):
	def __init__(self, robot: "Robot", get_left_right, get_forward_back, get_rotate):
		'''
		initializes tank drive movement.
		:param robot: the robot object
		:param get_r: Used to get the x angle, function that determines y direction
		:param get_y: Used to get the y angle, function that determines the y value and direction
		rotation and direction of rotation. Z value must be given if it separate from the joystick.
		'''
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.drivetrain)
		# self.setInterruptible(True)

		self.get_left_right = get_left_right
		self.get_forward_back = get_forward_back
		self.get_rotate = get_rotate

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.drivetrain] )

	# Methods
	def execute(self):
		"""
		Get r and y values from our joystick axes and send them to subsystem
		"""
		left_right = self.get_left_right()
		forward_back = self.get_forward_back()
		rotate = self.get_rotate()

		# left_right = self.get_left_right.getAsDouble()
		# forward_back = self.get_forward_back.getAsDouble()
		# rotation = self.get_rotate.getAsDouble()

		# print('joy =', left_right)
		# print('{0:.2f}, {1:.2f}, {2:.2f}'.format( left_right, forward_back, rotate ))

		"""
		def fromFieldRelativeSpeeds(
			vx: meters_per_second,
			vy: meters_per_second,
			omega: radians_per_second,
			robotAngle: wpimath.geometry._geometry.Rotation2d)

			-> ChassisSpeeds:
		"""

		# ChassisSpeeds does it weird, +X is considered forward, +Y is left
		chassis_speeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
		    vx = forward_back,
		    vy = -left_right,
		    omega = rotate * 2,
		    robotAngle = self.robot.drivetrain.get_gyro_rotation(),
		)

		self.robot.drivetrain.drive(chassis_speeds)

	def isFinished(self):
		return False

	def end(self, interrupted):
		self.robot.drivetrain.stop()

	def interrupted(self):
		self.end()


class Zero_Gyro(Command):
	def __init__(self, robot: "Robot"):
		'''
		Zeros gyro
		'''
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.drivetrain)
		# self.setInterruptible(True)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.drivetrain] )

	# Methods
	def execute(self):
		"""
		Resets gyro
		"""
		error = self.robot.drivetrain.gyro.setFusedHeading(0, 30)
		if error != 0:
			print('Failed to reset gyro. Error code ', error)

	def isFinished(self):
		pass

	def end(self, interrupted):
		pass

	def interrupted(self):
		self.end()

class Reset_Odometry(Command):
	def __init__(self, robot: "Robot"):
		'''
		Resets odometry
		'''
		super().__init__()

		self.robot = robot

		self.hasRequirement(self.robot.drivetrain)
		# self.setInterruptible(True)

	# Ovrerides
	def getRequirements(self):
		return set( [self.robot.drivetrain] )

	# Methods
	def execute(self):
		"""
		Resets odometry
		"""
		self.robot.drivetrain.odometry.reset(Pose2d(Translation2d(0, 0), Rotation2d(0)))

	def isFinished(self):
		pass

	def end(self, interrupted):
		pass

	def interrupted(self):
		self.end()