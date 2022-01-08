"""
Ctrl-Z FRC Team 4096
FIRST Robotics Competition 2022
Code for robot "swerve drivetrain prototype"
contact@team4096.org

Some code adapted from:
https://github.com/SwerveDriveSpecialties
"""

# This is to help vscode
from typing import TYPE_CHECKING

from wpilib._wpilib import SmartDashboard, XboxController

from swervelib.mk4_module_configuration import MK4_Module_Configuration
if TYPE_CHECKING:
	from robot import Robot

import wpilib

###  IMPORTS ###

# Subsystems

# Commands
import commands.drivetrain

# Controls
from customcontroller import XboxCommandController
from commands2.button import Button
from commands2 import ParallelCommandGroup

from wpilib import XboxController
import wpilib.interfaces
import const
from wpilib import Timer


class OI:
	"""
	Operator Input - This class ties together controls and commands.
	"""

	def __init__(self, robot: "Robot"):
		self.robot = robot

		# Controllers

		# self.driver1 = XboxController(0)
		self.driver1 = XboxCommandController(0)
		# self.driver2 = XboxCommandController(1)


		### Driving ###

		self.driver1.LEFT_JOY_Y.setInverted(True)
		self.driver1.RIGHT_JOY_X.setInverted(True)

		self.driver1.LEFT_JOY_X.setDeadzone(.18)
		self.driver1.LEFT_JOY_Y.setDeadzone(.18)
		self.driver1.RIGHT_JOY_X.setDeadzone(.18)

		self.driver1.LEFT_JOY_X.setModifier(lambda x: x * abs(x))
		self.driver1.LEFT_JOY_Y.setModifier(lambda x: x * abs(x))
		self.driver1.RIGHT_JOY_X.setModifier(lambda x: x * abs(x))



		self.drive_command = commands.drivetrain.Drive_Swerve(
			self.robot,
			self.driver1.LEFT_JOY_X,
			self.driver1.LEFT_JOY_Y,
			self.driver1.RIGHT_JOY_X,
		)

		self.driver1.A.whenPressed(commands.drivetrain.Zero_Gyro(self.robot))
		self.driver1.B.whenPressed(commands.drivetrain.Reset_Odometry(self.robot))

		# self.drive_command = commands.drivetrain.Drive_Swerve(
		# 	self.robot,
		# 	self.modify_axis(self.driver1.getY(wpilib.interfaces.GenericHID.Hand.kLeftHand)),
		# 	self.modify_axis(self.driver1.getX(wpilib.interfaces.GenericHID.Hand.kLeftHand)),
		# 	self.modify_axis(self.driver1.getX(wpilib.interfaces.GenericHID.Hand.kRightHand))
		# )

		self.robot.drivetrain.setDefaultCommand(self.drive_command)


	# def deadband(self, value, deadband):
	# 	if abs(value) > deadband:
	# 		if value > 0:
	# 			return (value - deadband) / (1.0 - deadband)
	# 		else:
	# 			return (value + deadband) / (1.0 - deadband)
	# 	else:
	# 		return 0.0


	# def modify_axis(self, value):
	# 	value = self.deadband(value, 0.05)
	# 	value = value * value
	# 	return value


	def log( self ):
		pass
